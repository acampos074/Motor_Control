/*
 * HES.c
 *
 *  Created on: Jun 25, 2022
 *      Author: Andres Campos
 */
#include "main.h"
#include "HES.h"
//#include "FOC.h"
#include "Calibration.h"
#include "EccComp.h"
#include "stm32f4xx_it.h"
#include <stdint.h>
#include <stdio.h>

#define ENC_BITS         14      // 14-bit encoder
//#define ENC_COUNTS       16384   // 2^14
#define LUT_SIZE         128     // Number of entries in your LUT
#define LUT_SHIFT        7       // 14 bits - 7 bits = 7 (used for indexing)


extern DAC_HandleTypeDef hdac;
extern SPI_HandleTypeDef hspi3;

extern calibration_t cal;
extern foc_t foc;
extern hes_t hes;

/**
 * @brief Eccentricity Look-Up Table (128 entries)
 * These values represent the correction (in counts) to be added to the raw signal.
 */
const int16_t eccentricity_lut[128] = {
    -17,   -8,  -38,  -21,   -4,    9,   33,   33,
     28,   27,   -5,  -19,  -21,  -17,    6,   30,
     34,   31,   37,   18,   11,  -14,  -33,   -7,
     10,   23,   15,    9,   12,    7,   -6,  -26,
    -33,  -30,  -11,  -14,  -10,  -12,   -7,  -13,
    -20,  -13,  -11,  -23,  -25,  -32,  -17,  -14,
    -18,  -20,  -10,    3,   15,  -12,  -25,  -27,
    -17,  -17,  -14,   -3,   18,   34,   27,   18,
     -3,  -16,  -10,  -11,    3,   20,   40,   50,
     43,   40,   10,   -6,  -12,   -5,   26,   38,
     48,   43,   44,   30,   22,   -2,  -28,   -4,
     11,   32,   21,   19,   18,   14,    0,  -17,
    -23,  -22,   -9,   -6,   -8,   -4,   -4,   -5,
    -20,  -15,  -15,  -18,  -25,  -24,  -19,  -10,
    -19,  -15,   -9,   12,   14,   -4,  -24,  -25,
    -19,  -11,  -17,   -5,   10,   30,   34,   -1
};

void warmup_HES(void)
{
	uint16_t tx_msg = 0;
	uint8_t SPI_HES_DATA_TX[2] = {0x00,0x00};
	uint8_t SPI_HES_DATA_RX[2] = {0x00,0x00};

	tx_msg = ( (PARC<<15) | (W0_READ<<14) | ANGLEUNC ); // ANGLEUNC ANGLECOM
	for(int i=0;i<100;i++)
	{
		SPI_HES_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
		SPI_HES_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
		// SPI read/write
		// Lower CS line
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi3, SPI_HES_DATA_TX, SPI_HES_DATA_RX, 1, 100);
		// wait for transmission complete
		while( hspi3.State == HAL_SPI_STATE_BUSY );
		// Set CS line high
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	}
}

void sample_HES(foc_t *foc,calibration_t *cal,hes_t *hes)
{
	// Capture reset state before the turns-init block can modify first_sample
	uint8_t was_uninit = !foc->first_sample;

	foc->theta_mech_old = foc->theta_mech;

	// Apply eccentricity LUT correction (no-op until ecc_build_lut() has run)
	uint16_t raw = hes->position_raw;
	//raw = (uint16_t)(((int32_t)raw + ecc_correct(raw)) & 0x3FFF);
	foc->theta_mech_raw = compensate_eccentricity(raw);

	// Mechanical Angle
	foc->theta_mech = (float)(foc->theta_mech_raw - foc->mech_offset)/(ENC_COUNTS);
	hes->int_angle = (int)foc->theta_mech;
	foc->theta_mech = TWO_PI*(foc->theta_mech - (float)hes->int_angle);
	foc->theta_mech = foc->theta_mech < 0 ? foc->theta_mech + TWO_PI : foc->theta_mech;

	// Electrical Angle (NPP*theta_mech)
	foc->theta_elec = (NPP*(float)(foc->theta_mech_raw - foc->elec_offset))/(ENC_COUNTS);
	hes->int_angle = (int)foc->theta_elec;
	foc->theta_elec = TWO_PI*(foc->theta_elec - (float)hes->int_angle);
	foc->theta_elec = foc->theta_elec < 0 ? foc->theta_elec + TWO_PI : foc->theta_elec;

	// Sensor rollover
	int rollover = 0;
	float angle_delta = foc->theta_mech - foc->theta_mech_old;
	if(angle_delta > PI)
	{
		rollover = -1; // this will bring the turns variable back to zero if we haven't rolled over
	}
	else if(angle_delta < -PI)
	{
		rollover = 1;
	}
	foc->turns += rollover; // Always keep the turns variable up to date
	// Initialize the turns variable
	if(!foc->first_sample)
	{
		foc->turns = 0;
		foc->first_sample = 1;
	}

	// Multi-turn position
	foc->theta_mech_multiturn[0] = foc->theta_mech + TWO_PI*(float)foc->turns;

	// Alpha-beta (g-h) velocity filter — tune ALPHA_AB / BETA_AB in FOC.h
	// Predict: x_pred = ab_pos + DT * ab_vel
	// Update:  ab_pos  = x_pred + ALPHA_AB * r
	//          ab_vel += (BETA_AB / DT) * r       r = measurement - x_pred
	static float ab_pos = 0.0f;
	static float ab_vel = 0.0f;

	if (was_uninit) {
		ab_pos = foc->theta_mech_multiturn[0];
		ab_vel = 0.0f;
	} else {
		float x_pred = ab_pos + DT * ab_vel;
		float r      = foc->theta_mech_multiturn[0] - x_pred;
		ab_pos       = x_pred + ALPHA_AB * r;
		ab_vel      += (BETA_AB / DT) * r;
	}
	foc->theta_dot_mech = ab_vel;
	//foc->theta_dot_mech = (foc->theta_mech_multiturn[0] - foc->theta_mech_multiturn[N_POS_SAMPLES-1])/(2.0*DT*(float)(N_POS_SAMPLES-1));

#if DEBUG_SCOPE
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)((foc->theta_mech_raw*4095.0f)/ENC_COUNTS));
#endif
}

void print_HES(foc_t *foc)
{
	printf("theta_mech_raw: %d cnts", foc->theta_mech_raw);
	printf("\r\n");
	printf("theta_mech: %f rad", foc->theta_mech);
	printf("\r\n");
	printf("theta_mech_multiturn: %f rad", foc->theta_mech_multiturn[0]);
	printf("\r\n");
	printf("theta_elec: %f rad", foc->theta_elec);
	printf("\r\n");
	printf("theta_dot_mech: %f rad/sec", foc->theta_dot_mech);
	printf("\r\n");
	printf("turns: %d", foc->turns);
	printf("\r\n");
}

uint8_t HES_ReadSensorDMA(hes_t *hes)
{
	static uint8_t txBuf[2];
	hes->tx_msg = ( (PARC<<15) | (W0_READ<<14) | ANGLEUNC ); // ANGLEUNC ANGLECOM
	txBuf[0] = (uint8_t) (hes->tx_msg & 0x00FF);
	txBuf[1] = (uint8_t) ((hes->tx_msg & 0xFF00)>>8);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); // pull CS pin low
	if (HAL_SPI_TransmitReceive_DMA(hes->spiHandle,txBuf,hes->HES_RxBuf,1) == HAL_OK){
#if DEBUG_SCOPE
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
#endif
		return 1;
	}
	else{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // pull CS pin high
		return 0;
	}

}

void HES_ReadSensorDMA_Complete(hes_t *hes)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // pull CS pin high
	//hes->position_raw = ( (hes->HES_RxBuf[1]<<8) | hes->HES_RxBuf[0] ) & 0x3FFF;
	hes->position_raw = ( (hes->HES_RxBuf[1]<<8) | hes->HES_RxBuf[0] ) & 0x3FFF;

}


/**
 * @brief Compensates for encoder eccentricity using linear interpolation.
 * @param position_raw The raw 16-bit variable (expected range 0 - 16383).
 * @return uint16_t The compensated 14-bit position.
 */
uint16_t compensate_eccentricity(uint16_t position_raw) {
    // 1. Safety Mask: Ensure we only deal with 14 bits (0-16383)
    // in case your variable has noise in the upper 2 bits.
    position_raw &= 0x3FFF;

    // 2. Indexing (14-bit signal >> 7 = 7-bit index for 128 entries)
    uint32_t index = position_raw >> (ENC_BITS - LUT_SHIFT);
    uint32_t next_index = (index + 1) % LUT_SIZE;

    // 3. Fraction (The "distance" between two LUT entries)
    uint16_t fraction = position_raw & 0x7F; // Same as position_raw & (range - 1)
    uint16_t range = 128;

    // 4. Interpolate the correction value
    int16_t corr0 = eccentricity_lut[index];
    int16_t corr1 = eccentricity_lut[next_index];

    // Fixed-point interpolation: y = y0 + (diff * frac) / range
    int32_t interpolated_correction = corr0 + ((int32_t)(corr1 - corr0) * (int32_t)fraction) / range;

    // 5. Apply correction
    int32_t corrected_pos = (int32_t)position_raw + interpolated_correction;

    // 6. Final Wrap-around logic
    if (corrected_pos < 0) {
        corrected_pos += ENC_COUNTS;
    } else if (corrected_pos >= ENC_COUNTS) {
        corrected_pos -= ENC_COUNTS;
    }

    return (uint16_t)corrected_pos;
}



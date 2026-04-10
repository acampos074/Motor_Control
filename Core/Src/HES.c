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
#include "stm32f4xx_it.h"
#include <stdint.h>
#include <stdio.h>

extern DAC_HandleTypeDef hdac;
extern SPI_HandleTypeDef hspi3;

extern calibration_t cal;
extern foc_t foc;
extern hes_t hes;

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
	/*
	uint16_t position_raw;
	uint16_t tx_msg = 0;
	uint8_t SPI_HES_DATA_TX[2] = {0x00,0x00};
	uint8_t SPI_HES_DATA_RX[2] = {0x00,0x00};
	int int_angle;
	*/

	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);
	// Shift previous samples
	foc->theta_mech_old = foc->theta_mech;
	for(int i = N_POS_SAMPLES-1; i > 0; i--)
	{
		foc->theta_mech_multiturn[i] = foc->theta_mech_multiturn[i-1];
	}
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);

//	hes->tx_msg = ( (PARC<<15) | (W0_READ<<14) | ANGLEUNC ); // ANGLEUNC ANGLECOM
//	hes->SPI_HES_DATA_TX[0] = (uint8_t) (hes->tx_msg & 0x00FF);
//	hes->SPI_HES_DATA_TX[1] = (uint8_t) ((hes->tx_msg & 0xFF00)>>8);


	// SPI read/write
	// SPI baudrate prescaler of 8bit works
	// changed pre-scaler to 2bit to see if a faster baudrate works
	// Lower CS line
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	// !!!!!!!!! COMMENTED THIS TO TEST DMA !!!!!!!!!!!
	//HAL_SPI_TransmitReceive(&hspi3, hes->SPI_HES_DATA_TX, hes->SPI_HES_DATA_RX, 1, 100);
	// wait for transmission complete
	//while( hspi3.State == HAL_SPI_STATE_BUSY );
	//HAL_SPI_TransmitReceive_DMA(&hspi3,hes->SPI_HES_DATA_TX,hes->SPI_HES_DATA_RX,1);


	// Set CS line high
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

	//hes->position_raw = ( (hes->SPI_HES_DATA_RX[1]<<8) | hes->SPI_HES_DATA_RX[0] ) & 0x3FFF;
	//foc->theta_mech_raw = ( (hes->SPI_HES_DATA_RX[1]<<8) | hes->SPI_HES_DATA_RX[0] ) & 0x3FFF;

	foc->theta_mech_raw = hes->position_raw;
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);


	// Linearize HES
	/*
	int off_1 = cal->lut_arr[(foc->theta_mech_raw)>>9];				// lookup table lower entry
	int off_2 = cal->lut_arr[((foc->theta_mech_raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(foc->theta_mech_raw - ((foc->theta_mech_raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	foc->theta_mech_lin = foc->theta_mech_raw + off_interp; // ** changed to minus!! need to bring this back to a plus
	*/
	//foc->theta_mech_raw = hes->position_raw; // replaced polling SPI with DMA which happens in the background


	// Mechanical Angle
	foc->theta_mech = (float)(foc->theta_mech_raw - foc->mech_offset)/(ENC_COUNTS); // calibrated angle
	//foc->theta_mech = (float)(foc->theta_mech_lin - foc->mech_offset)/(ENC_COUNTS); // calibrated angle
	hes->int_angle = (int)foc->theta_mech;
	foc->theta_mech = TWO_PI*(foc->theta_mech - (float)hes->int_angle);
	foc->theta_mech = foc->theta_mech < 0 ? foc->theta_mech + TWO_PI : foc->theta_mech;

	// Electrical Angle (NPP*theta_mech)
	foc->theta_elec = (NPP*(float)(foc->theta_mech_raw - foc->elec_offset))/(ENC_COUNTS); // calibrated angle
	//foc->theta_elec = (NPP*(float)(foc->theta_mech_lin - foc->elec_offset))/(ENC_COUNTS); // calibrated angle
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

	// Mechanical Angular Velocity
	foc->theta_dot_mech = (foc->theta_mech_multiturn[0] - foc->theta_mech_multiturn[N_POS_SAMPLES-1])/(2.0*DT*(float)(N_POS_SAMPLES-1));
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

	//position_raw = HES & 0x3FFF;
	//HES = (uint16_t) ((HES*4095.0f)/16384.0f);  // 12bit = 4095 & 14bit = 16384
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t) ((position_raw*4095.0f)/16384.0f));

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
	//if (HAL_SPI_TransmitReceive_DMA(hes->spiHandle,txBuf,(uint8_t *)hes->HES_RxBuf,2) == HAL_OK){
	if (HAL_SPI_TransmitReceive_DMA(hes->spiHandle,txBuf,hes->HES_RxBuf,1) == HAL_OK){
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5); // for debugging
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



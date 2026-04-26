/*
 * FOC.c
 *
 *  Created on: Jun 25, 2022
 *      Author: Andres Campos
 */

#include "main.h"
#include "stm32f4xx_it.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "HES.h"
#include "FOC.h"
#include "DRV.h"
#include "FOC_Math.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;
extern DAC_HandleTypeDef hdac;

// Use int32_t to accommodate larger uNm values
const int32_t cogging_torque_lut[512] = {
       -62,    -52,    -43,    -34,    -25,    -17,    -10,     -4,
         1,      5,      9,     11,     13,     14,     14,     14,
        13,     11,      9,      7,      4,      0,     -2,     -6,
       -11,    -15,    -21,    -26,    -32,    -37,    -44,    -50,
       -56,    -62,    -69,    -75,    -81,    -88,    -94,   -100,
      -106,   -112,   -118,   -124,   -129,   -133,   -136,   -139,
      -140,   -139,   -136,   -131,   -124,   -114,   -102,    -87,
       -70,    -51,    -31,    -11,      8,     28,     45,     60,
        72,     80,     84,     83,     78,     69,     55,     38,
        17,     -6,    -31,    -59,    -87,   -115,   -143,   -170,
      -196,   -221,   -243,   -263,   -280,   -294,   -304,   -311,
      -314,   -313,   -307,   -297,   -282,   -262,   -238,   -210,
      -178,   -144,   -107,    -70,    -34,      1,     34,     64,
        89,    110,    125,    134,    138,    136,    128,    115,
        98,     77,     52,     24,     -4,    -35,    -66,    -96,
      -126,   -154,   -180,   -203,   -222,   -237,   -248,   -254,
      -254,   -249,   -239,   -223,   -202,   -177,   -147,   -114,
       -78,    -40,     -1,     37,     75,    111,    144,    172,
       197,    216,    229,    235,    235,    229,    216,    197,
       172,    143,    108,     71,     31,     -9,    -50,    -89,
      -127,   -161,   -190,   -215,   -233,   -246,   -251,   -251,
      -244,   -231,   -212,   -189,   -162,   -133,   -101,    -67,
       -34,      0,     31,     63,     92,    119,    143,    165,
       183,    197,    208,    214,    216,    214,    207,    195,
       178,    157,    133,    105,     75,     43,     11,    -20,
       -51,    -80,   -105,   -127,   -144,   -157,   -165,   -167,
      -164,   -156,   -144,   -128,   -108,    -86,    -61,    -34,
        -7,     20,     47,     74,     99,    122,    144,    163,
       179,    192,    201,    207,    210,    209,    205,    197,
       187,    173,    158,    140,    121,    101,     81,     62,
        43,     25,      9,     -4,    -16,    -26,    -33,    -38,
       -41,    -41,    -39,    -35,    -28,    -20,    -10,      0,
        13,     28,     43,     59,     76,     94,    111,    128,
       144,    160,    175,    188,    201,    212,    222,    231,
       238,    245,    251,    255,    259,    262,    263,    263,
       261,    258,    254,    248,    240,    231,    220,    208,
       194,    180,    164,    148,    132,    115,     99,     83,
        68,     53,     40,     28,     17,      9,      3,      0,
        -2,     -1,      3,     10,     20,     32,     48,     66,
        86,    107,    128,    150,    170,    189,    206,    219,
       228,    233,    233,    228,    218,    204,    185,    163,
       137,    109,     79,     48,     16,    -14,    -44,    -73,
       -99,   -123,   -144,   -160,   -173,   -182,   -186,   -186,
      -181,   -171,   -156,   -137,   -115,    -89,    -61,    -31,
         0,     30,     60,     87,    113,    134,    152,    165,
       174,    177,    175,    169,    157,    140,    119,     95,
        67,     36,      3,    -30,    -66,   -101,   -135,   -169,
      -200,   -229,   -255,   -278,   -296,   -311,   -321,   -325,
      -325,   -320,   -310,   -295,   -275,   -250,   -222,   -190,
      -156,   -120,    -83,    -46,    -10,     24,     56,     84,
       109,    128,    142,    150,    152,    148,    137,    120,
        97,     70,     38,      3,    -33,    -72,   -110,   -147,
      -182,   -214,   -242,   -265,   -283,   -295,   -302,   -303,
      -298,   -288,   -273,   -254,   -231,   -204,   -175,   -143,
      -109,    -74,    -39,     -4,     29,     61,     91,    118,
       140,    158,    171,    178,    179,    174,    163,    147,
       125,    100,     70,     39,      6,    -26,    -58,    -88,
      -115,   -139,   -158,   -172,   -182,   -186,   -186,   -181,
      -171,   -158,   -141,   -121,    -99,    -76,    -51,    -25,
         0,     24,     49,     72,     93,    113,    130,    146,
       158,    168,    175,    179,    179,    177,    172,    164,
       154,    141,    127,    112,     95,     78,     60,     43,
        26,      9,     -7,    -23,    -39,    -55,    -71,    -88,
      -105,   -124,   -143,   -163,   -184,   -206,   -227,   -248
};


void zero_current(foc_t *foc)
{
    int n = 1000;
    uint32_t adc_a_offset = 0;
    uint32_t adc_b_offset = 0;

    set_zero_DC();

    for (int i = 0; i < n; i++) {
    	sample_ADC(foc);
    	adc_a_offset += foc->pi.adc2_ia_raw;
    	adc_b_offset += foc->pi.adc1_ib_raw;
    }
    foc->pi.adc2_offset = adc_a_offset/n;
    foc->pi.adc1_offset = adc_b_offset/n;

    printf("\rADC_a_offset: %u cnts\r\n", foc->pi.adc2_offset);
    printf("\rADC_b_offset: %u cnts\r\n", foc->pi.adc1_offset);

    sample_ADC(foc);
    printf("\rADC_a: %d cnts\r\n", (int)(foc->pi.adc2_ia_raw - foc->pi.adc2_offset));
    printf("\rADC_b: %d cnts\r\n", (int)(foc->pi.adc1_ib_raw - foc->pi.adc1_offset));
}

void uvw(float theta_elec,float vd,float vq,float *u,float *v,float *w)
{
	float cf = FastCos(theta_elec);
	float sf = FastSin(theta_elec);

	*u = cf*vd - sf*vq;
	*v = (0.86602540378f*sf-0.5f*cf)*vd - (-0.86602540378f*cf-0.5f*sf)*vq;
	*w = (-0.86602540378f*sf-0.5f*cf)*vd - (0.86602540378f*cf-0.5f*sf)*vq;
}
void svm(float u, float v, float w, float vm, float *dc_u, float *dc_v, float *dc_w)
{
	float v_offset = 0.5f*( fminf3(u,v,w) + fmaxf3(u,v,w) );

	*dc_u = fmaxf(fminf( (0.5f + ((u - v_offset)/vm)),DC_MAX),DC_MIN);
	*dc_v = fmaxf(fminf( (0.5f + ((v - v_offset)/vm)),DC_MAX),DC_MIN);
	*dc_w = fmaxf(fminf( (0.5f + ((w - v_offset)/vm)),DC_MAX),DC_MIN);
}
void dq0(float theta_elec,float i_a,float i_b,float i_c,float *i_d,float *i_q)
{
	float cf = FastCos(theta_elec);
	float sf = FastSin(theta_elec);

	*i_d = 0.6666667f*(cf*i_a + (0.86602540378f*sf-.5f*cf)*i_b + (-0.86602540378f*sf-.5f*cf)*i_c); // swaped b & c
	*i_q = 0.6666667f*(-sf*i_a - (-0.86602540378f*cf-.5f*sf)*i_b - (0.86602540378f*cf-.5f*sf)*i_c);
}


void set_duty_cycle(uint8_t flag,float dc_u,float dc_v, float dc_w)
{
	if(flag == 0) // Don't swap
	{
		//TIM1->CCR3 = Fs*(1.0f-0.75f); // A // ***********!!!!!!!!!!!!!!!!!!!!!!!
		//TIM1->CCR2 = Fs*(1.0f-0.25f); // B
		//TIM1->CCR1 = Fs*(1.0f-0.10f); // C
		//TIM1->CCR3 = Fs*(1.0f-dc_u); // A
		//TIM1->CCR2 = Fs*(1.0f-dc_v); // B
		//TIM1->CCR1 = Fs*(1.0f-dc_w); // C //
		TIM1->CCR3 = Fs*(1-dc_u); // A
		TIM1->CCR2 = Fs*(1-dc_v); // B
		TIM1->CCR1 = Fs*(1-dc_w); // C // Added 1-dc on 04/13/26

	}
	else // Swap
	{
		TIM1->CCR3 = Fs*(1-dc_u);
		TIM1->CCR1 = Fs*(1-dc_v);
		TIM1->CCR2 = Fs*(1-dc_w); //

	}
}

void set_zero_DC(void)
{
	TIM1->CCR3 = 0; // A
	TIM1->CCR2 = 0; // B
	TIM1->CCR1 = 0; // C
}

/* Kick off next conversion first (locks sample to PWM bottom = counter 0 = ISR entry),
 * then read the previous tick's result from the DR — already populated, no race.
 * STM32F446 regular channels cannot use TIM1 TRGO — software trigger only.
 * Sequence: ISR fires → SWSTART → read old DR → compute. New result ready in ~1 µs,
 * well before the next ISR (25 µs). */
void read_ADC(foc_t *foc)
{
	// Trigger next conversion immediately at ISR entry (PWM triangle bottom)
	ADC1->CR2 |= ADC_CR2_SWSTART;

	if (foc->phase_order_flag == 0)
	{
		foc->pi.adc2_ia_raw = HAL_ADC_GetValue(&hadc2); // i_a
		foc->pi.adc1_ib_raw = HAL_ADC_GetValue(&hadc1); // i_b
	}
	else
	{
		foc->pi.adc2_ia_raw = HAL_ADC_GetValue(&hadc1); // i_b
		foc->pi.adc1_ib_raw = HAL_ADC_GetValue(&hadc2); // i_a
	}

	foc->VM_raw = HAL_ADC_GetValue(&hadc3);
	foc->VM     = VOLTS_PER_COUNTS * foc->VM_raw;

	foc->i_a = AMPS_PER_COUNTS * ((float)foc->pi.adc2_ia_raw - (float)foc->pi.adc2_offset);
	foc->i_b = AMPS_PER_COUNTS * ((float)foc->pi.adc1_ib_raw - (float)foc->pi.adc1_offset);
	foc->i_c = -(foc->i_a) - (foc->i_b);
}

/* Blocking sample — for use OUTSIDE the ISR only (e.g. zero_current()).
 * Polls for the next TIM1-triggered conversion to complete (≤ 25 µs at 40 kHz). */
void sample_ADC(foc_t *foc)
{
	HAL_ADC_PollForConversion(&hadc1, 1); // 1 ms timeout — far more than the 25 µs period
	read_ADC(foc);
}

void commutate(foc_t *foc,float theta_elec)
{

    foc->v_max = OVERMODULATION*foc->VM*(DC_MAX-DC_MIN)*SQRT1_3;
    foc->i_max = I_MAX;

	// 2. === DQ0 Transform ===
	// Calibration/open-loop use the reference angle theta_elec; all other modes use measured
	if(foc->mode == MODE_CALIBRATION || foc->mode == MODE_OPEN_LOOP_TEST)
	{
		dq0(theta_elec,foc->i_a,foc->i_b,foc->i_c,&foc->i_d,&foc->i_q);
	}
	else
	{
		dq0(foc->theta_elec,foc->i_a,foc->i_b,foc->i_c,&foc->i_d,&foc->i_q);
	}

	// IIR low-pass on id/iq — filters quantization & switching noise before PI
	static float id_filt = 0.0f;
	static float iq_filt = 0.0f;
	id_filt = IQ_FILTER_ALPHA * foc->i_d + (1.0f - IQ_FILTER_ALPHA) * id_filt;
	iq_filt = IQ_FILTER_ALPHA * foc->i_q + (1.0f - IQ_FILTER_ALPHA) * iq_filt;
	foc->i_d = id_filt;
	foc->i_q = iq_filt;

	// 3. === PI current controller ===

	// 3.1 Calculate compensation in Nm and convert to Amps (I = Torque / Kt)
	// Note: get_cogging_compensation returns the NEGATIVE of the cogging torque
	float cogging_comp_amps = get_cogging_compensation(foc->theta_mech) / KT;

	// 3.2 Add to the reference so the PI controller drives the motor to cancel the ripple
	float iq_error = (foc->pi.iq_ref + cogging_comp_amps) - foc->i_q;

	float id_error = foc->pi.id_ref - foc->i_d;
	//float iq_error = foc->pi.iq_ref - foc->i_q;

	// 4. Command voltages — mode selects the voltage source
	if(foc->mode == MODE_CALIBRATION || foc->mode == MODE_OPEN_LOOP_TEST)
	{
		foc->pi.vd_cmd = 0.8f;
		foc->pi.vq_cmd = 0.0f;
	}
	else if(foc->mode == MODE_VOLTAGE_FOC)
	{
		foc->pi.vd_cmd = 0.0f;
		//foc->pi.vq_cmd = 0.5f;
		foc->pi.vq_cmd = foc->can_rx.vq_cmd;
	}
	else if(foc->mode == MODE_SYSTEM_ID)
	{
		foc->pi.vd_cmd = 0.2f;
		foc->pi.vq_cmd = 0.0f;
	}
	else
	{
		foc->pi.vd_cmd = pdGain*id_error + foc->pi.Sum_id_error;
		foc->pi.vd_cmd = fmaxf(fminf(foc->pi.vd_cmd, foc->v_max), -foc->v_max);
		float vq_max = sqrtf(foc->v_max*foc->v_max - foc->pi.vd_cmd*foc->pi.vd_cmd);

		foc->pi.vq_cmd = pqGain*iq_error + foc->pi.Sum_iq_error;
		foc->pi.vq_cmd = fmaxf(fminf(foc->pi.vq_cmd, vq_max), -vq_max);
	}

	// Only accumulate integrators when the PI output is actually driving the plant.
	// In open-loop modes (CALIBRATION, OPEN_LOOP_TEST, SYSTEM_ID, VOLTAGE_FOC) the
	// voltage commands are set directly, so integrating the current error would cause
	// wind-up that produces a current spike on the next transition to closed-loop control.
	if(foc->mode == MODE_CURRENT        || foc->mode == MODE_SPEED    ||
	   foc->mode == MODE_POSITION       || foc->mode == MODE_TORQUE   ||
	   foc->mode == MODE_SYSID_COASTDOWN)
	{
		foc->pi.Sum_id_error += pdGain*idGain*id_error;
		foc->pi.Sum_id_error = fmaxf(fminf(foc->pi.Sum_id_error, foc->v_max), -foc->v_max); // saturate integrator

		foc->pi.Sum_iq_error += pqGain*iqGain*iq_error;
		foc->pi.Sum_iq_error = fmaxf(fminf(foc->pi.Sum_iq_error, foc->v_max), -foc->v_max); // saturate integrator
	}

	// 5. === Limit the voltage commands to not overmodulate ===
	float cmd_mag = sqrtf(foc->pi.vq_cmd*foc->pi.vq_cmd + foc->pi.vd_cmd*foc->pi.vd_cmd);
	if(cmd_mag > foc->v_max){
		foc->pi.vd_cmd = foc->pi.vd_cmd*(foc->v_max/cmd_mag);
		foc->pi.vq_cmd = foc->pi.vq_cmd*(foc->v_max/cmd_mag);
	}

	// 6. === Inverse DQ0 Transform ===
	if(foc->mode == MODE_CALIBRATION || foc->mode == MODE_OPEN_LOOP_TEST)
	{
		uvw(theta_elec,foc->pi.vd_cmd,foc->pi.vq_cmd,&foc->v_u,&foc->v_v,&foc->v_w);
	}
	else
	{
		uvw(foc->theta_elec,foc->pi.vd_cmd,foc->pi.vq_cmd,&foc->v_u,&foc->v_v,&foc->v_w);
	}


	// 7. === SVM ===
	svm(foc->v_u,foc->v_v,foc->v_w,foc->VM,&foc->dc_u,&foc->dc_v,&foc->dc_w);

	// 8. === Set DC ===
	set_duty_cycle(foc->phase_order_flag,foc->dc_u,foc->dc_v,foc->dc_w);

#if DEBUG_SCOPE
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)((foc->theta_dot_mech*4095)/50.0f));
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, foc->pi.adc2_ia_raw);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, foc->pi.adc1_ib_raw);
#endif
}

void torque_control(foc_t *foc)
{
	static float time = 0;
	time = time + 0.001f; // outer loop runs at 1 kHz

	float torque_cmd;
	if(foc->mode == MODE_POSITION)
	{
		torque_cmd = foc->kp*(foc->can_rx.q - foc->theta_mech_multiturn[0]) + foc->kd*(foc->theta_dot_mech_cmd - foc->theta_dot_mech);
		foc->pi.iq_ref = torque_cmd / KT;
		foc->pi.id_ref = 0.0f;
	}
	else if(foc->mode == MODE_SPEED)
	{
		torque_cmd = foc->kp*(foc->can_rx.dq - foc->theta_dot_mech);
		foc->pi.iq_ref = torque_cmd / KT;
		foc->pi.id_ref = 0.0f;
	}
	else if(foc->mode == MODE_CURRENT)
	{
		foc->pi.iq_ref = foc->can_rx.iq_cmd;
		foc->pi.id_ref = 0.0f;
	}
	else if(foc->mode == MODE_TORQUE)
	{
		foc->pi.iq_ref = foc->can_rx.torque_cmd / KT;
		foc->pi.id_ref = 0.0f;
	}
}

void reset_variables(foc_t *foc,calibration_t *cal,hes_t *hes,SPI_HandleTypeDef *spiHandle)
{
	hes->spiHandle = spiHandle;
	hes->counter = 0;

	cal->theta_mech_ref = 0;
	cal->loop_count = 0;
	cal->sample_count = 0;
	cal->started = 0;

	foc->mode = MODE_IDLE;
	// NOTE: set phase_order_flag = 1 if the motor magnet is assembled in the reverse direction
	foc->phase_order_flag = 0;

	foc->kp = 0.0f;//0.05f; // commented out on 10/21
	foc->kd = 0.0f;//0.012f; // commented out on 10/21
	foc->p_des = 0;
	foc->torque_control_counter = 0;
	foc->torque_control_counter_total = 0;

	foc->i_a = 0;
	foc->i_b = 0;
	foc->i_c = 0;
	foc->i_d = 0;
	foc->i_q = 0;
	foc->pi.iq_dot = 0;
	foc->VM = 24.0f;
	foc->pi.iq_ref = 0;
	foc->pi.id_ref = 0;
	foc->pi.Sum_iq_error =0;
	foc->pi.Sum_id_error =0;
	foc->pi.vq_cmd = 0;
	foc->pi.vd_cmd = 0;
	foc->v_u=0;
	foc->v_v=0;
	foc->v_w=0;
	foc->pi.adc1_offset = 0;
	foc->pi.adc2_offset = 0;
	foc->theta_elec = 0;
	foc->theta_mech = 0;
	foc->GAIN = 25.0;
	for (int i = 0; i < N_POS_SAMPLES; i++ )
	{
		foc->theta_mech_multiturn[ i ] = 0;
	}
	foc->theta_dot_mech = 0;
	foc->turns = 0;
	foc->first_sample = 0;
	foc->elec_offset = 0;
	foc->mech_offset = 0;
	foc->thetadot_mech = 0;
	foc->dc_u = 0;
	foc->dc_v = 0;
	foc->dc_w = 0;
	foc->v_max = 0;
	foc->i_max = 0;
	foc->can_rx.torques = 0;
	foc->can_rx.dq = 0;
	foc->can_rx.vq_cmd = 0;
}

void print_flags(foc_t *foc)
{
	const char *mode_names[] = {
		"MODE_IDLE", "MODE_CALIBRATION", "MODE_OPEN_LOOP_TEST",
		"MODE_SYSTEM_ID", "MODE_VOLTAGE_FOC", "MODE_CURRENT",
		"MODE_SPEED", "MODE_POSITION", "MODE_TORQUE",
		"MODE_SYSID_COASTDOWN"
	};
	printf("mode: %s", mode_names[foc->mode]);
	printf("\r\n");
	printf("phase_order_flag: %d", foc->phase_order_flag);
	printf("\r\n");
}


float get_cogging_compensation(uint16_t raw_pos) {
    // 1. Calculate Index and Fraction
    uint16_t idx0 = (raw_pos >> (ENC_BITS - COGGING_LUT_BITS)) & (COGGING_LUT_SIZE - 1);
    uint16_t idx1 = (idx0 + 1) & (COGGING_LUT_SIZE - 1);

    // Fraction between samples (0.0 to 1.0)
    uint16_t frac_raw = raw_pos & 0x1F; // bottom 5 bits (14-9=5)
    float fraction = (float)frac_raw / 32.0f;

    // 2. Interpolate in uNm
    float val0 = (float)cogging_torque_lut[idx0];
    float val1 = (float)cogging_torque_lut[idx1];
    float interpolated_uNm = val0 + fraction * (val1 - val0);

    // 3. Convert uNm to Nm (divide by 1,000,000)
    // Negate because we want to subtract the cogging effect
    return -(interpolated_uNm / 1000000.0f);
}







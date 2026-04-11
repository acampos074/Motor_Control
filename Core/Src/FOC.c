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

void zero_current(foc_t *foc)
{
	//printf("\rHere!\r\n");
    int n = 1000;
    //uint32_t adc_a_raw = 0;
    //uint32_t adc_b_raw = 0;
    uint32_t adc_a_offset = 0;
    uint32_t adc_b_offset = 0;

    set_zero_DC();
    //printf("\rHere!!\r\n");

    for (int i = 0; i<n; i++){               // Average n samples
    	sample_ADC(foc);
    	//HAL_ADC_Start(&hadc1);
    	//HAL_ADC_Start(&hadc2);
      	//adc_a_raw = HAL_ADC_GetValue(&hadc2);
      	//adc_b_raw = HAL_ADC_GetValue(&hadc1);
    	adc_a_offset += foc->pi.adc2_ia_raw;
    	adc_b_offset += foc->pi.adc1_ib_raw;
     }
    //printf("\rHere!!!\r\n");
    foc->pi.adc2_offset = adc_a_offset/n;
    foc->pi.adc1_offset = adc_b_offset/n;

  	printf("\rADC_a_offset: %u cnts", (foc->pi.adc2_offset));
  	printf("\r\n");
  	printf("\rADC_b_offset: %u cnts", (foc->pi.adc1_offset));
  	printf("\r\n");
  	sample_ADC(foc);
  	//HAL_ADC_Start(&hadc1);
  	//HAL_ADC_Start(&hadc2);
  	//foc->pi.adc2_ia_raw = HAL_ADC_GetValue(&hadc2);
  	//foc->pi.adc1_ib_raw = HAL_ADC_GetValue(&hadc1);
  	printf("\rADC_a: %d cnts", (int)(foc->pi.adc2_ia_raw-foc->pi.adc2_offset) );
  	printf("\r\n");
  	printf("\rADC_b: %d cnts", (int)(foc->pi.adc1_ib_raw-foc->pi.adc1_offset) );
  	printf("\r\n");
}

void uvw(float theta_elec,float vd,float vq,float *u,float *v,float *w)
{
	float cf = FastCos(theta_elec);
	float sf = FastSin(theta_elec);

	*u = cf*vd - sf*vq;
	*v = (0.86602540378f*sf-0.5f*cf)*vd - (-0.86602540378f*cf-0.5f*sf)*vq;
	*w = (-0.86602540378f*sf-0.5f*cf)*vd - (0.86602540378f*cf-0.5f*sf)*vq;
}
void svm(float u,float v, float w,float *dc_u,float *dc_v,float *dc_w)
{
		float v_offset = 0.5f*( fminf3(u,v,w) + fmaxf3(u,v,w) );

		*dc_u = fmaxf(fminf( (0.5f + ((u - v_offset)/V_BUS)),DC_MAX),DC_MIN);
		*dc_v = fmaxf(fminf( (0.5f + ((v - v_offset)/V_BUS)),DC_MAX),DC_MIN);
		*dc_w = fmaxf(fminf( (0.5f + ((w - v_offset)/V_BUS)),DC_MAX),DC_MIN);
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
		//TIM1->CCR1 = Fs*(1.0f-dc_w); // C // Need to remove the 1-dc. TODO since 09/17/23
		TIM1->CCR3 = Fs*(dc_u); // A
		TIM1->CCR2 = Fs*(dc_v); // B
		TIM1->CCR1 = Fs*(dc_w); // C // Removed 1-dc on 06/19/24

	}
	else // Swap
	{
		TIM1->CCR3 = Fs*(dc_u);
		TIM1->CCR1 = Fs*(dc_v);
		TIM1->CCR2 = Fs*(dc_w); // removed the 1-dc value when flipping the board

	}
}

void set_zero_DC(void)
{
	TIM1->CCR3 = 0; // A
	TIM1->CCR2 = 0; // B
	TIM1->CCR1 = 0; // C
}

void sample_ADC(foc_t *foc)
{
	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	//HAL_ADC_Start(&hadc3);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	if(foc->phase_order_flag == 0)
	{
		foc->pi.adc2_ia_raw = HAL_ADC_GetValue(&hadc2); // i_a
		foc->pi.adc1_ib_raw = HAL_ADC_GetValue(&hadc1); // i_b
	}
	else
	{
		foc->pi.adc2_ia_raw = HAL_ADC_GetValue(&hadc1); // i_b
		foc->pi.adc1_ib_raw = HAL_ADC_GetValue(&hadc2); // i_a
	}

	//printf("\rHere!!!\r\n");
	//HAL_ADC_Start(&hadc1);
	//printf("\rHere!!!!\r\n");

	//printf("\rHere!!!!!\r\n");

	foc->VM_raw = HAL_ADC_GetValue(&hadc3); // motor voltage
	foc->VM = VOLTS_PER_COUNTS*foc->VM_raw;

	foc->i_a = AMPS_PER_COUNTS*((float)foc->pi.adc2_ia_raw - (float)foc->pi.adc2_offset);
	foc->i_b = AMPS_PER_COUNTS*((float)foc->pi.adc1_ib_raw - (float)foc->pi.adc1_offset);
	foc->i_c = -(foc->i_a) - (foc->i_b);

}

void commutate(foc_t *foc,float theta_elec)
{

	// 1. === Sample currents ===
	//sample_ADC(foc);

    foc->v_max = OVERMODULATION*foc->VM*(DC_MAX-DC_MIN)*SQRT1_3;
	//foc->v_max = V_BUS;
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

	// 3. === PI current controller ===
	float id_error = foc->pi.id_ref - foc->i_d;
	float iq_error = foc->pi.iq_ref - foc->i_q;

	// 4. Command voltages — mode selects the voltage source
	if(foc->mode == MODE_CALIBRATION || foc->mode == MODE_OPEN_LOOP_TEST)
	{
		foc->pi.vd_cmd = 0.8f;
		foc->pi.vq_cmd = 0.0f;
	}
	else if(foc->mode == MODE_VOLTAGE_FOC)
	{
		foc->pi.vd_cmd = 0.0f;
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
	if(foc->mode == MODE_CURRENT  || foc->mode == MODE_SPEED ||
	   foc->mode == MODE_POSITION || foc->mode == MODE_TORQUE)
	{
		foc->pi.Sum_id_error += pdGain*idGain*id_error;
		foc->pi.Sum_id_error = fmaxf(fminf(foc->pi.Sum_id_error, foc->v_max), -foc->v_max); // saturate integrator

		foc->pi.Sum_iq_error += pqGain*iqGain*iq_error;
		foc->pi.Sum_iq_error = fmaxf(fminf(foc->pi.Sum_iq_error, foc->v_max), -foc->v_max); // saturate integrator
	}

	// 5. === Limit the voltage commands to not overmodulate ===
	float cmd_mag = sqrtf(foc->pi.vq_cmd*foc->pi.vq_cmd + foc->pi.vd_cmd*foc->pi.vd_cmd); // take the L2 norm
	if(cmd_mag > V_BUS){
		foc->pi.vd_cmd = foc->pi.vd_cmd*(V_BUS/cmd_mag);
		foc->pi.vq_cmd = foc->pi.vq_cmd*(V_BUS/cmd_mag);
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
	svm(foc->v_u,foc->v_v,foc->v_w,&foc->dc_u,&foc->dc_v,&foc->dc_w);

	// 8. === Set DC ===
	// === 321 === (rotates CW)
	//set_duty_cycle(foc->phase_order_flag,foc->dc_u,foc->dc_v,foc->dc_w);

	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)((foc->theta_dot_mech*4095)/50.0f));
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, foc->pi.adc2_ia_raw);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, foc->pi.adc1_ib_raw);

}

void commutate_v2(foc_t *foc,float theta_elec)
{

    foc->v_max = OVERMODULATION*foc->VM*(DC_MAX-DC_MIN)*SQRT1_3;
	//foc->v_max = V_BUS;
    foc->i_max = I_MAX;

	// 1. === DQ0 Transform ===
    //foc->pi.iq_dot = foc->i_q;
    dq0(foc->theta_elec,foc->i_a,foc->i_b,foc->i_c,&foc->i_d,&foc->i_q);
    //foc->pi.iq_dot = (foc->i_q-foc->pi.iq_dot)*ONE_OVER_DT;

	// 2. === PI current controller ===
	float id_error = foc->pi.id_ref - foc->i_d;
	float iq_error = foc->pi.iq_ref - foc->i_q;

	foc->pi.vd_cmd = pdGain*id_error + foc->pi.Sum_id_error;
	foc->pi.vd_cmd = fmaxf(fminf(foc->pi.vd_cmd, foc->v_max), -foc->v_max);
	float vq_max = sqrtf(foc->v_max*foc->v_max - foc->pi.vd_cmd*foc->pi.vd_cmd);

	foc->pi.vq_cmd = pqGain*iq_error + foc->pi.Sum_iq_error;
	foc->pi.vq_cmd = fmaxf(fminf(foc->pi.vq_cmd, vq_max), -vq_max);

	foc->pi.Sum_id_error += pdGain*idGain*id_error;
	foc->pi.Sum_id_error = fmaxf(fminf(foc->pi.Sum_id_error, foc->v_max), -foc->v_max); // saturate integrator

	foc->pi.Sum_iq_error += pqGain*iqGain*iq_error;
	foc->pi.Sum_iq_error = fmaxf(fminf(foc->pi.Sum_iq_error, foc->v_max), -foc->v_max); // saturate integrator

	// 3. === Limit the voltage commands to not overmodulate ===
	float cmd_mag = sqrtf(foc->pi.vq_cmd*foc->pi.vq_cmd + foc->pi.vd_cmd*foc->pi.vd_cmd); // take the L2 norm
	if(cmd_mag > V_BUS){
		foc->pi.vd_cmd = foc->pi.vd_cmd*(V_BUS/cmd_mag);
		foc->pi.vq_cmd = foc->pi.vq_cmd*(V_BUS/cmd_mag);
	}

	// 4. === Inverse DQ0 Transform ===
	uvw(foc->theta_elec,foc->pi.vd_cmd,foc->pi.vq_cmd,&foc->v_u,&foc->v_v,&foc->v_w);

	// 5. === SVM ===
	svm(foc->v_u,foc->v_v,foc->v_w,&foc->dc_u,&foc->dc_v,&foc->dc_w);

	// 6. === Set DC ===
	// === 321 === (rotates CW)
	//set_duty_cycle(foc->phase_order_flag,foc->dc_u,foc->dc_v,foc->dc_w);

	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)((foc->theta_dot_mech*4095)/50.0f));
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, foc->pi.adc2_ia_raw);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, foc->pi.adc1_ib_raw);

}

void torque_control(foc_t *foc)
{
	// PD control
	// PD position control
	static float time = 0;
	//float GearRatio = 12.0f; // just for troubleshooting

	time = time + 0.001f; // since sampling frequency is 1kHz
	// foc->theta_mech_multiturn[0]
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

	//float torque_cmd = KP*(foc->p_des-foc->theta_mech)*foc->pos_control_flag + KD*(foc->theta_dot_mech_cmd - foc->theta_dot_mech);
	// P velocity control
	//foc->pi.iq_ref = torque_cmd/(KT*GR)/GearRatio;
	//foc->pi.iq_ref = torque_cmd/(KT*GR);
	//foc->pi.id_ref = 0.0f;
	//printf("tau: %f  iq_ref: %f\r\n",torque_cmd,foc->pi.iq_ref);

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
		"MODE_SPEED", "MODE_POSITION", "MODE_TORQUE"
	};
	printf("mode: %s", mode_names[foc->mode]);
	printf("\r\n");
	printf("phase_order_flag: %d", foc->phase_order_flag);
	printf("\r\n");
}







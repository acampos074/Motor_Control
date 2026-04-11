/*
 * Calibration.c
 *
 *  Created on: Jun 26, 2022
 *      Author: Andres Campos
 */


#include "main.h"
#include "stm32f4xx_it.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "can.h"

#include "HES.h"
#include "FOC.h"
#include "DRV.h"
#include "FOC_Math.h"
#include "Calibration.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac;
extern CAN_TxHeaderTypeDef TxMessage;
extern uint8_t CANTxData[8];
extern uint32_t TxMailbox;


void calibrate_HES2(foc_t *foc,calibration_t *cal,float loop_count,hes_t *hes)
{
	// 0. Initialize calibration parameters
	if(!cal->started)
	{
		cal->started = 1; // so that we enter here only once
		cal->start_count = loop_count; // this is the counter reference
		cal->sample_count = 0;
		cal->time = (float)(loop_count - cal->start_count)*DT; // cal->time will only grow
		cal->next_sample_time = cal->time;
		cal->theta_mech_ref = 0; // zero mech ref
	}
	// 0.1 update current time step
	cal->time = (float)(loop_count - cal->start_count)*DT; // cal->time will only grow

	// 1. Stabilize the motor

	// 2. Rotate forwards
	if(cal->time < TWO_PI*NPP/W_CAL) // if we haven't completed one revolution forward
	{
		cal->theta_mech_ref += W_CAL*DT; // update reference step
		cal->theta_elec = cal->theta_mech_ref;
		commutate(foc,cal->theta_elec);
		// (samples per pole pair)*(pole-pair)
		// current time step (time) gets updated for every timer interrupt
		if(cal->time > cal->next_sample_time) // next_sample_time keep increasing
		{
			int count_ref = cal->theta_mech_ref*(float)ENC_COUNTS/(TWO_PI*NPP); // theta elec [counts/rads]
			int error = foc->theta_mech_raw - count_ref; // encoder error
			cal->error_arr[cal->sample_count] = error + ENC_COUNTS*(error<0);
			printf("%.3f %d %d %d %.3f\r\n", cal->time,cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_mech_ref);
			/*
			int theta_mech_ref_int = float_to_uint(cal->theta_mech_ref,-TWO_PI,TWO_PI);
			CANTxData[0] = 0;    // get 8 MSB
			CANTxData[1] = theta_mech_ref_int>>8;
			CANTxData[2] = theta_mech_ref_int&0xFF;
			HAL_CAN_AddTxMessage(&hcan1, &TxMessage, CANTxData, &TxMailbox); // CAN Tx
			*/

			cal->next_sample_time += TWO_PI/(W_CAL*SAMPLES_PER_PPAIR);
			if(cal->sample_count == NPP*SAMPLES_PER_PPAIR-1) // 896 sample points
			{
				return;
			}
			//HES_ReadSensorDMA(hes); // !!!!!!!!!!!!!!!!!!! ADDED THIS LINE TO TEST DMA
			sample_HES(foc,cal,hes);
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t) ((foc->theta_mech_raw*4095.0f)/16384.0f));
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t) ((angle*4095.0f)/16384.0f));
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t) ((cal->theta_mech_ref*4095.0f)/TWO_PI));
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t) ((foc->theta_mech_raw*4095.0f)/16384.0f));
			cal->sample_count++; // revolution counter keeps increasing

		}
		return; // exits calibration function
	}

	// 3. Rotate backwards
	else if(cal->time < 2.0f*TWO_PI*NPP/W_CAL)
	{
		cal->theta_mech_ref -= W_CAL*DT; // update reference step
		cal->theta_elec = cal->theta_mech_ref;
		commutate(foc,cal->theta_elec);
		if((cal->time > cal->next_sample_time) && (cal->sample_count > 0))
		{
			int count_ref = cal->theta_mech_ref*(float)ENC_COUNTS/(TWO_PI*NPP); // theta elec [counts/rads]
			int error = foc->theta_mech_raw - count_ref; // encoder error
			error += ENC_COUNTS*(error<0);

			cal->error_arr[cal->sample_count] = (cal->error_arr[cal->sample_count] + error)/2; // takes the average of forward vs backwards
			printf("%.3f %d %d %d %.3f\r\n", cal->time,cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_mech_ref);
			/*
			int theta_mech_ref_int = float_to_uint(cal->theta_mech_ref,-TWO_PI,TWO_PI);
			CANTxData[0] = 0;    // get 8 MSB
			CANTxData[1] = theta_mech_ref_int>>8;
			CANTxData[2] = theta_mech_ref_int&0xFF;
			HAL_CAN_AddTxMessage(&hcan1, &TxMessage, CANTxData, &TxMailbox); // CAN Tx
			*/
			cal->next_sample_time += TWO_PI/(W_CAL*SAMPLES_PER_PPAIR);
			cal->sample_count--; // revolution counter keeps decreasing until it's less than 0
			//HES_ReadSensorDMA(hes); // !!!!!!!!!!!!!!!!!!! ADDED THIS LINE TO TEST DMA
			sample_HES(foc,cal,hes);
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t) ((cal->theta_mech_ref*4095.0f)/TWO_PI));
			HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t) ((foc->theta_mech_raw*4095.0f)/16384.0f));
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t) ((angle*4095.0f)/16384.0f));
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);
		}
		return;
	}

	// Estimate average offset

	int theta_elec_mean = 0;
	for(int i=0; i<(NPP*SAMPLES_PER_PPAIR); i++)
	{
		theta_elec_mean += cal->error_arr[i];
	}
	foc->elec_offset = theta_elec_mean/(NPP*SAMPLES_PER_PPAIR);

	commutate(foc,cal->theta_elec*0);
	int theta_mech_mean = 0;
	int n = 100;
	for(int i=0;i<n;i++)
	{
		//HES_ReadSensorDMA(hes); // !!!!!!!!!!!!!!!!!!! ADDED THIS LINE TO TEST DMA
		sample_HES(foc,cal,hes);
		theta_mech_mean += foc->theta_mech_raw;
		printf("theta_mech_raw: %d cnts\r\n",foc->theta_mech_raw);
		printf("vd: %f V\r\n",foc->vd_cmd);
	}
	foc->mech_offset = theta_mech_mean/n;
	set_zero_DC();
	printf("mech_offset: %d cnts\r\n",foc->mech_offset);

	// === Generate LUT ===
	printf("\r === LUT === \r\n");
	int window = SAMPLES_PER_PPAIR;
	int lut_offset = (ENC_COUNTS-cal->error_arr[0])*N_LUT/ENC_COUNTS;
	for(int i = 0; i<N_LUT; i++)
	{
		int moving_avg = 0;
		for(int j = (-window)/2; j<(window)/2; j++)
		{
			int index = i*NPP*SAMPLES_PER_PPAIR/N_LUT + j;
			if(index<0)
			{
				index += (SAMPLES_PER_PPAIR*NPP);
			}
			else if(index>(SAMPLES_PER_PPAIR*NPP-1))
			{
				index -= (SAMPLES_PER_PPAIR*NPP);
			}
			moving_avg += cal->error_arr[index];
		}
		moving_avg = moving_avg/window;
		int lut_index = lut_offset + i;
		if(lut_index>(N_LUT-1))
		{
			lut_index -= N_LUT;
		}
		cal->lut_arr[lut_index] = moving_avg - foc->elec_offset;
		printf("%d  %d\r\n", lut_index, moving_avg - foc->elec_offset);

	}

	set_zero_DC();
	foc->mode = MODE_IDLE; // exit calibration state
	cal->started = 0;
	cal->time = 0;
	cal->loop_count = 0;
  	printf("theta_elec_offset: %d cnts",foc->elec_offset);
  	printf("\r\n");

	//int theta_mech_ref_int = float_to_uint(cal->theta_mech_ref,-TWO_PI,TWO_PI);
	//CANTxData[0] = 1;    // get 8 MSB
	//CANTxData[1] = theta_mech_ref_int>>8;
	//CANTxData[2] = theta_mech_ref_int&0xFF;
	//HAL_CAN_AddTxMessage(&hcan1, &TxMessage, CANTxData, &TxMailbox); // CAN Tx

}

void open_loop_test(foc_t *foc,calibration_t *cal,float loop_count)
{
	// 0. Initialize calibration parameters
	if(!cal->started)
	{
		cal->started = 1; // so that we enter here only once
		cal->start_count = loop_count; // this is the counter reference
		cal->sample_count = 0;
		cal->time = (float)(loop_count - cal->start_count)*DT; // cal->time will only grow
		cal->next_sample_time = cal->time;
		cal->theta_mech_ref = 0; // zero mech ref
	}
	// 0.1 update current time step
	cal->time = (float)(loop_count - cal->start_count)*DT; // cal->time will only grow

	// 1. Stabilize the motor
	// 2. Rotate forwards
	if(cal->time < TWO_PI*NPP/W_CAL) // if we haven't completed one revolution forward
	{
		cal->theta_mech_ref += W_CAL*DT; // update reference step
		cal->theta_elec = cal->theta_mech_ref;
		commutate(foc,cal->theta_elec);
		// (samples per pole pair)*(pole-pair)
		// current time step (time) gets updated for every timer interrupr
		if(cal->time > cal->next_sample_time) // next_sample_time keep increasing
		{
			int count_ref = cal->theta_mech_ref*(float)ENC_COUNTS/(TWO_PI*NPP); // theta elec [counts/rads]
			int error = foc->theta_mech_raw - count_ref; // encoder error
			cal->error_arr[cal->sample_count] = error + ENC_COUNTS*(error<0);
			//printf("%.3f %d %d %d %.3f\r\n", cal->time,cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_mech_ref);
			//printf("%d %.3f %.3f\r\n",cal->sample_count,foc->theta_mech,foc->theta_elec);
			printf("%d m:%.3f e:%.3f ia:%.3f ib:%.3f ic:%.3f u:%.3f v:%.3f w:%.3f id:%.3f iq:%.3f\r\n",cal->sample_count,foc->theta_mech,foc->theta_elec,foc->i_a,foc->i_b,foc->i_c,foc->v_u,foc->v_v,foc->v_w,foc->i_d,foc->i_q);
			cal->next_sample_time += TWO_PI/(W_CAL*SAMPLES_PER_PPAIR);
			if(cal->sample_count == NPP*SAMPLES_PER_PPAIR-1) // 896 sample points
			{
				return;
			}
			//sample_HES(foc,cal);
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t) ((foc->theta_mech_raw*4095.0f)/16384.0f));
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t) ((angle*4095.0f)/16384.0f));
			cal->sample_count++; // revolution counter keeps increasing

		}
		return; // exits calibration function
	}

	// 3. Rotate backwards
	else if(cal->time < 2.0f*TWO_PI*NPP/W_CAL)
	{
		cal->theta_mech_ref -= W_CAL*DT; // update reference step
		cal->theta_elec = cal->theta_mech_ref;
		commutate(foc,cal->theta_elec);
		if((cal->time > cal->next_sample_time) && (cal->sample_count > 0))
		{
			int count_ref = cal->theta_mech_ref*(float)ENC_COUNTS/(TWO_PI*NPP); // theta elec [counts/rads]
			int error = foc->theta_mech_raw - count_ref; // encoder error
			error += ENC_COUNTS*(error<0);

			cal->error_arr[cal->sample_count] = (cal->error_arr[cal->sample_count] + error)/2; // takes the average of forward vs backwards
			//printf("%.3f %d %d %d %.3f\r\n", cal->time,cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_mech_ref);
			//printf("%d m:%.3f e:%.3f ia:%.3f ib:%.3f ic:%.3f u:%.3f v:%.3f w:%.3f\r\n",cal->sample_count,foc->theta_mech,foc->theta_elec,foc->i_a,foc->i_b,foc->i_c,foc->v_u,foc->v_v,foc->v_w);
			printf("%d m:%.3f e:%.3f ia:%.3f ib:%.3f ic:%.3f u:%.3f v:%.3f w:%.3f id:%.3f iq:%.3f\r\n",cal->sample_count,foc->theta_mech,foc->theta_elec,foc->i_a,foc->i_b,foc->i_c,foc->v_u,foc->v_v,foc->v_w,foc->i_d,foc->i_q);
			cal->next_sample_time += TWO_PI/(W_CAL*SAMPLES_PER_PPAIR);
			cal->sample_count--; // revolution counter keeps decreasing until it's less than 0
			//sample_HES(foc,cal);
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t) ((foc->theta_mech_raw*4095.0f)/16384.0f));
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t) ((angle*4095.0f)/16384.0f));
		}
		return;
	}

	set_zero_DC();
	foc->mode = MODE_IDLE; // exit open loop test state
	cal->started = 0;
	cal->time = 0;
	cal->loop_count = 0;
}



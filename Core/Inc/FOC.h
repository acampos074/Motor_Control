/*
 * FOC.h
 *
 *  Created on: Jun 25, 2022
 *      Author: Andres Campos
 */

#ifndef INC_FOC_H_
#define INC_FOC_H_

#include <stdint.h>
//#include "HES.h"

/*
//These set works (BW: 200Hz)
#define Fs 0x08C9 // FOC Sampling Frequency 40kHz
#define pqGain 0.71821f
#define iqGain 0.015834f
#define pdGain 0.71821f
#define idGain 0.015834f
*/

/*
 // These set works (BW: 25Hz)
#define Fs 0x0E10 // 0x0E10 FOC Sampling Frequency of 25kHz
#define pqGain 0.090206f // 0.090206f
#define iqGain 0.025215f
#define pdGain 0.090206f
#define idGain 0.025215f
*/

/*
 // Current control BW of 313Hz at 25kHz sampling
#define Fs 0x0E10 // FOC Sampling Frequency 25kHz
#define pqGain 1.1276f
#define iqGain 0.025215f
#define pdGain 1.1276f
#define idGain 0.025215f
*/

 // Current control BW of 103Hz at 33kHz sampling. Acoustics are nice with these set.
//#define Fs 0x0CE4 // FOC Sampling Frequency 33kHz double check this
//#define pqGain 0.37096f
//#define iqGain 0.019161f
//#define pdGain 0.37096f
//#define idGain 0.019161f

 // Current control BW of 125Hz at 40kHz sampling
// --- Timing: change F_PWM here and Fs/DT/ONE_OVER_DT update automatically ---
#define F_CLK           180000000UL             // System clock Hz (180 MHz)
#define F_PWM           40000UL                 // PWM / FOC loop frequency Hz
#define Fs              ((F_CLK / (2 * F_PWM)) - 1) // TIM1 ARR for center-aligned PWM (= 0x08C9 = 2249 at 40kHz)
#define DT              (1.0f / (float)F_PWM)   // FOC sample period (s)
#define ONE_OVER_DT     ((float)F_PWM)          // 1 / DT

// Outer (torque/position/speed) loop runs at TORQUE_LOOP_HZ — derived from F_PWM
#define TORQUE_LOOP_HZ  1000U                   // 1 kHz outer loop
#define TORQUE_DIVIDER  (F_PWM / TORQUE_LOOP_HZ) // ISR ticks per outer-loop update (= 40 at 40kHz)

#define pqGain 0.44888f
#define iqGain 0.015834f
#define pdGain 0.44888f
#define idGain 0.015834f


/*
 // Current control BW of 1kHz gains at 25kHz sampling
#define Fs 0x0E10 // FOC Sampling Frequency 25kHz
#define pqGain 3.7586f // 0.090206f
#define iqGain 0.025215f
#define pdGain 3.7586f
#define idGain 0.025215f
*/

/*
// These set works (BW: 30Hz)
#define Fs 0x0BB8 // (0x0BB8 FOC Sampling Frequency of 30kHz)
#define pqGain 0.27005f
#define iqGain 0.021057f
#define pdGain 0.27005f
#define idGain 0.021057f
*/

/* This set was last used on 08/01/23
// These set works (BW: 72Hz)
#define Fs 0x0C1E // (0x0C1E FOC Sampling Frequency of 29kHz)
#define pqGain 0.26114f
#define iqGain 0.021775f
#define pdGain 0.26114f
#define idGain 0.021775f
*/

//
//#define AMPS_PER_COUNTS 0.04028320312f // Using Gain of 40V/V (2*82.5A/ 4096 counts) Updated this on 10/11/23 to try it
#define AMPS_PER_COUNTS 0.02014160156f // Using Gain of 40V/V (82.5A/ 4096 counts)
//#define I_MAX 20.0f // 20 amps max Updated this to try it on 10/11/23
#define I_MAX 40.0f // 40 amps max
#define VOLTS_PER_COUNTS 0.012890625f // 52.8V / 4096 counts
#define V_MAX 24.0F
#define SPEED_MAX 300.0f
#define TAU_MAX 2.0f

// HES Defines
//#define N_POS_SAMPLES 20
#define N_POS_SAMPLES 20
#define _CPR 16384
#define NPP 7 // number of pole pairs
#define TWO_PI 6.28318530718f
#define PI 3.14159265359f
#define RADS_PER_COUNTS  0.000383495197f //

// Calibration parameters
#define W_CAL 20.0f // calibration rotation speed (rad/sec)
#define SAMPLES_PER_PPAIR 128
#define N_CAL SAMPLES_PER_PPAIR*NPP
#define N_LUT 128

#define DC_MAX 0.94f // changed from 0.95 to 0.9
#define DC_MIN 0.00f // changed from 0.05 to 0.1
#define SQRT1_3 0.57735026919f
#define OVERMODULATION 1.15f

#define V_BUS 24.0f
#define l_d (567e-6f)
#define l_q (567e-6f) // Henries
#define wb  0.00594f // phase flux linkage (volt.sec/rad)
#define GR 12.0f // gear ratio
//#define KT 0.0059447f // torque constant (Nm/Amps)
#define KT 0.0217f // 0.023f //.037352f // torque constant (Nm/Amps)
//#define KP 0.0001f //0.05f // 0.018 & 0.024 works ok (i.e. still stable for position control)
//#define KD 0.0001f //0.012f // 0.024f works well for speed control

typedef struct {
	float i_a ,i_b ,i_c ;                              // ABC currents
	float i_d ,i_q; // DQ currents
	float iq_dot;
	float iq_ref, id_ref;
	float Sum_iq_error, Sum_id_error;
	float vq_cmd, vd_cmd;
	float  v_u,v_v,v_w;
	uint16_t adc2_ia_raw,adc1_ib_raw;                             // ABC raw currents
	uint16_t adc1_offset, adc2_offset;
	uint16_t theta_mech_raw;
	uint16_t VM_raw;
	float can_rx_torques;
	float  VM;
	float theta_elec,theta_mech,theta_mech_old;
	float theta_mech_multiturn[N_POS_SAMPLES];
	float theta_dot_mech;
	int mech_offset,elec_offset;
	int theta_mech_lin;
	int turns,first_sample;
	float thetadot_mech;
	float dc_u,dc_v,dc_w;
	float flag;
	float v_max,i_max;
	float p_des,theta_dot_mech_cmd;
	float can_rx_dq,can_rx_q,can_rx_vq_cmd,can_rx_iq_cmd,can_rx_torque_cmd;
	float kp,kd;
	float GAIN;
	uint8_t controller_flag;
	uint8_t phase_order_flag;
	uint8_t cal_flag;
	uint8_t open_loop_test_flag;
	uint8_t systemID_flag;
	uint8_t voltage_FOC_flag;
	uint8_t torque_control_counter;
	uint32_t torque_control_counter_total;
	uint8_t pos_control_flag,speed_control_flag,current_control_flag,torque_control_flag;



} foc_t;

typedef struct {

	float theta_elec;
	int loop_count; // calibration counter

	float theta_mech_ref;
	int start_count;
	uint8_t started;
	float time; // calibration timer
	int sample_count;
	float next_sample_time;
	int error_arr[N_CAL];
	int lut_arr[N_LUT];

} calibration_t;

typedef struct {

	uint16_t position_raw;
	uint16_t tx_msg;
	uint8_t SPI_HES_DATA_TX[2];
	uint8_t SPI_HES_DATA_RX[2];
	int int_angle;
	SPI_HandleTypeDef *spiHandle;
	uint8_t HES_RxBuf[2];
	float counter;

} hes_t;




void commutate(foc_t *foc,float theta_elec);
void commutate_v2(foc_t *foc,float theta_elec);
void torque_control(foc_t *foc);
//void calibrate_HES2(foc_t *foc,calibration_t *cal,float loop_count);
//void calibrate_HES(foc_t *foc,float theta_elec,float theta_mech);
void reset_variables(foc_t *foc,calibration_t *cal,hes_t *hes,SPI_HandleTypeDef *spiHandle);
void uvw(float theta_elec,float vd,float vq,float *u,float *v,float *w);
void svm(float u,float v, float w,float *dc_u,float *dc_v,float *dc_w);
void dq0(float theta_elec,float i_a,float i_b,float i_c,float *i_d,float *i_q);

void set_duty_cycle(uint8_t flag,float dc_u,float dc_v, float dc_w);
void set_zero_DC(void);
void sample_ADC(foc_t *foc);
void zero_current(foc_t *foc);
void print_flags(foc_t *foc);


#endif /* INC_FOC_H_ */

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
#define F_PWM           30000UL                 // PWM / FOC loop frequency Hz
#define Fs              ((F_CLK / (2 * F_PWM)) - 1) // TIM1 ARR for center-aligned PWM (= 0x08C9 = 2249 at 40kHz)
#define DT              (1.0f / (float)F_PWM)   // FOC sample period (s)
#define ONE_OVER_DT     ((float)F_PWM)          // 1 / DT

// Outer (torque/position/speed) loop runs at TORQUE_LOOP_HZ — derived from F_PWM
#define TORQUE_LOOP_HZ  1000U                   // 1 kHz outer loop
#define TORQUE_DIVIDER  (F_PWM / TORQUE_LOOP_HZ) // ISR ticks per outer-loop update (= 40 at 40kHz)

/*
 // Current control BW of 1kHz gains at 40kHz sampling
#define pqGain 0.44888f
#define iqGain 0.015834f
#define pdGain 0.44888f
#define idGain 0.015834f
*/


/*
 // Current control BW of 1kHz gains at 25kHz sampling
#define Fs 0x0E10 // FOC Sampling Frequency 25kHz
#define pqGain 3.7586f // 0.090206f
#define iqGain 0.025215f
#define pdGain 3.7586f
#define idGain 0.025215f
*/


// These set works (BW: 30Hz)
//#define Fs 0x0BB8 // (0x0BB8 = 3000 FOC Sampling Frequency of 30kHz)
#define pqGain 0.27005f
#define iqGain 0.021057f
#define pdGain 0.27005f
#define idGain 0.021057f


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

// IIR low-pass filter on id/iq after Park transform.
// α=0.1 @ 30 kHz → fc ≈ 477 Hz, well above the ~125 Hz current-loop BW.
// Increase α for less lag (less filtering); decrease for more smoothing.
#define IQ_FILTER_ALPHA 0.1f

// Alpha-beta (g-h) velocity filter
// Predict: x_pred = x_est + DT * v_est
// Update:  x_est  = x_pred + ALPHA_AB * r
//          v_est += (BETA_AB / DT) * r       where r = measurement - x_pred
// Critical-damping relationship: BETA_AB = ALPHA_AB^2 / (2 - ALPHA_AB)
// ALPHA_AB=0.05 → BETA_AB≈0.00128 (critical);  0.001 is slightly under-damped.
// Velocity time constant τ ≈ ALPHA_AB * DT / BETA_AB (≈ 1.25 ms at these values).
// Increase ALPHA_AB / BETA_AB for faster response; decrease for more smoothing.
#define ALPHA_AB  0.05f
#define BETA_AB   0.001f

// Calibration parameters
#define W_CAL 20.0f // calibration rotation speed (rad/sec)
#define SAMPLES_PER_PPAIR 128
#define N_CAL SAMPLES_PER_PPAIR*NPP

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

// Debug instrumentation — set to 1 to enable, 0 for production builds
#define DEBUG_SCOPE   0   // DAC / GPIO oscilloscope probes
#define DEBUG_VERBOSE 0   // extra diagnostic printf output

// Control mode — mutually exclusive, replaces the 8 boolean flags
typedef enum {
    MODE_IDLE           = 0,  // PWM off, no commutation
    MODE_CALIBRATION,         // open-loop encoder calibration sweep
    MODE_OPEN_LOOP_TEST,      // open-loop diagnostic rotation
    MODE_SYSTEM_ID,           // constant voltage injection for parameter ID
    MODE_VOLTAGE_FOC,         // direct vq command via CAN (no current PI)
    MODE_CURRENT,             // closed-loop: direct iq_ref command
    MODE_SPEED,               // closed-loop: P speed → iq_ref
    MODE_POSITION,            // closed-loop: PD position → iq_ref
    MODE_TORQUE,              // closed-loop: torque command → iq_ref (1 kHz update)
    MODE_SYSID_COASTDOWN,     // system ID: spin-up → steady-state → coast-down log
} control_mode_t;

// PI current controller state — integrators, references, voltage commands, ADC data
typedef struct {
	float    iq_ref, id_ref;              // current references [A]
	float    Sum_iq_error, Sum_id_error;  // integrators [V]
	float    vq_cmd, vd_cmd;             // voltage commands [V]
	float    iq_dot;                      // current derivative (reserved for feedforward)
	uint16_t adc2_ia_raw, adc1_ib_raw;   // raw ADC counts for phase currents
	uint16_t adc1_offset, adc2_offset;   // current sensor zero offsets [counts]
} pi_state_t;

// CAN receive buffer — setpoints and commands from the host controller
typedef struct {
	float vq_cmd;       // voltage command (MODE_VOLTAGE_FOC) [V]
	float iq_cmd;       // current command (MODE_CURRENT) [A]
	float torque_cmd;   // torque command (MODE_TORQUE) [Nm]
	float q;            // position setpoint (MODE_POSITION) [rad]
	float dq;           // speed setpoint (MODE_SPEED) [rad/s]
	float torques;      // general torque field
} can_rx_t;

typedef struct {
	// --- Motor / sensor state ---
	float i_a, i_b, i_c;                  // phase currents [A]
	float i_d, i_q;                        // DQ currents [A]
	float v_u, v_v, v_w;                  // phase voltages [V]
	float dc_u, dc_v, dc_w;               // duty cycles [0–1]
	float v_max, i_max;                    // voltage and current limits
	float VM;                              // measured bus voltage [V]
	uint16_t VM_raw;                       // raw bus voltage ADC count
	uint16_t theta_mech_raw;              // raw encoder count [0–16383]
	float theta_elec, theta_mech, theta_mech_old; // angles [rad]
	float theta_mech_multiturn[N_POS_SAMPLES];    // position history for velocity
	float theta_dot_mech;                 // angular velocity [rad/s]
	float thetadot_mech;                  // duplicate velocity field (unused)
	int   mech_offset, elec_offset;       // calibrated angle offsets [counts]
	int   turns, first_sample;            // multi-turn tracking state
	float flag;                           // reserved / debug

	// --- Outer-loop setpoints & gains ---
	float p_des, theta_dot_mech_cmd;      // position and speed setpoints [rad, rad/s]
	float kp, kd;                         // outer-loop PD gains (CAN-tunable)
	float GAIN;                           // setpoint scaling factor

	// --- Sub-modules ---
	pi_state_t   pi;      // PI current controller state
	can_rx_t     can_rx;  // CAN receive buffer

	// --- Control bookkeeping ---
	control_mode_t mode;          // active control mode
	uint8_t phase_order_flag;     // hardware: 0=normal, 1=swapped phase order
	uint8_t torque_control_counter;
	uint32_t torque_control_counter_total;

	// TODO: hes_t belongs in HES.h — move when header dependency is resolved

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
void torque_control(foc_t *foc);
//void calibrate_HES2(foc_t *foc,calibration_t *cal,float loop_count);
//void calibrate_HES(foc_t *foc,float theta_elec,float theta_mech);
void reset_variables(foc_t *foc,calibration_t *cal,hes_t *hes,SPI_HandleTypeDef *spiHandle);
void uvw(float theta_elec,float vd,float vq,float *u,float *v,float *w);
void svm(float u, float v, float w, float vm, float *dc_u, float *dc_v, float *dc_w);
void dq0(float theta_elec,float i_a,float i_b,float i_c,float *i_d,float *i_q);

void set_duty_cycle(uint8_t flag,float dc_u,float dc_v, float dc_w);
void set_zero_DC(void);
void read_ADC(foc_t *foc);
void sample_ADC(foc_t *foc);   // blocking poll — outside ISR only
void zero_current(foc_t *foc);
void print_flags(foc_t *foc);


#endif /* INC_FOC_H_ */

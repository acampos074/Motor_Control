/*
 * SysID.h
 *
 * Coast-down system identification — estimates viscous damping B.
 *
 * Sequence triggered by pressing 'd':
 *   Phase 0 (spin-up)   : velocity P controller drives omega → SYSID_VEL_TARGET
 *                         iq_ref = SYSID_KV * (target - omega), capped at ±SYSID_IQ_MAX
 *   Phase 1 (steady-ss) : average iq and omega over SYSID_SS_COUNT ticks
 *   Phase 2 (coast-down): zero iq, log theta_dot_mech at 1 kHz → sysid_vel_buf
 *
 * B = KT * iq_ss / vel_ss   (steady-state torque balance at known speed)
 */

#ifndef INC_SYSID_H_
#define INC_SYSID_H_

#include "FOC.h"

// ── tuneable parameters ────────────────────────────────────────────────────
#define SYSID_N_SAMPLES         2000    // coast-down buffer length: 2 s at 1 kHz
#define SYSID_VEL_TARGET        30.0f   // target spin-up velocity [rad/s]
#define SYSID_KV                0.05f   // velocity P gain: iq_ref = KV*(target-omega) [A/(rad/s)]
#define SYSID_IQ_MAX            0.5f    // current saturation during spin-up [A]
#define SYSID_VEL_STABLE_THRESH 1.0f    // |omega - target| band for steady-state [rad/s]
#define SYSID_STABLE_COUNT      300     // ticks inside band before declaring steady state (= 300 ms)
#define SYSID_SS_COUNT          100     // averaging window for iq_ss / vel_ss (= 100 ms)
#define SYSID_VEL_LIMIT         500.0f  // hard abort if |omega| exceeds this [rad/s]
// ──────────────────────────────────────────────────────────────────────────

extern float            sysid_vel_buf[SYSID_N_SAMPLES]; // coast-down velocity log [rad/s]
extern float            sysid_vel_ss;                   // steady-state velocity   [rad/s]
extern float            sysid_iq_ss;                    // steady-state iq         [A]
extern volatile uint8_t sysid_done;                     // set by ISR when buffer full
extern volatile uint8_t sysid_aborted;                  // set by ISR on velocity limit trip

void sysid_reset(foc_t *foc);   // call before entering MODE_SYSID_COASTDOWN
void sysid_step(foc_t *foc);    // call at 1 kHz from ISR (TORQUE_DIVIDER gate)
void sysid_print_if_done(void); // call from main context (_HW_Process_Pending_Ints)

#endif /* INC_SYSID_H_ */

/*
 * SysID.h
 *
 * Coast-down system identification — estimates viscous damping B.
 *
 * Sequence triggered by pressing 'd':
 *   Phase 0 (spin-up)   : hold iq = SYSID_IQ until velocity stabilises
 *   Phase 1 (steady-ss) : average velocity over SYSID_SS_COUNT ticks → vel_ss
 *   Phase 2 (coast-down): zero iq, log theta_dot_mech at 1 kHz → sysid_vel_buf
 *
 * B = KT * SYSID_IQ / vel_ss   (from steady-state torque balance)
 */

#ifndef INC_SYSID_H_
#define INC_SYSID_H_

#include "FOC.h"

// ── tuneable parameters ────────────────────────────────────────────────────
#define SYSID_N_SAMPLES         2000    // buffer length: 2 s at 1 kHz
#define SYSID_IQ                0.01f   // spin-up iq command [A]
#define SYSID_VEL_STABLE_THRESH 0.05f   // |Δω| threshold for stability [rad/s]
#define SYSID_STABLE_COUNT      300     // consecutive stable 1-kHz ticks (= 300 ms)
#define SYSID_SS_COUNT          100     // averaging window for vel_ss (= 100 ms)
// ──────────────────────────────────────────────────────────────────────────

extern float           sysid_vel_buf[SYSID_N_SAMPLES]; // coast-down velocity log [rad/s]
extern float           sysid_vel_ss;                   // steady-state velocity   [rad/s]
extern volatile uint8_t sysid_done;                    // set by ISR when buffer full

void sysid_reset(foc_t *foc);  // call before entering MODE_SYSID_COASTDOWN
void sysid_step(foc_t *foc);   // call at 1 kHz from ISR (TORQUE_DIVIDER gate)
void sysid_print_if_done(void); // call from main context (_HW_Process_Pending_Ints)

#endif /* INC_SYSID_H_ */

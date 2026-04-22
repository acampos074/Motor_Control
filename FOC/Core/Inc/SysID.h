/*
 * SysID.h
 *
 * Coast-down system identification — estimates B/J from velocity decay.
 *
 * Sequence triggered by pressing 'd':
 *   Phase 0 (spin-up)   : open-loop Vq = SYSID_VQ, Vd = 0 for SYSID_SPINUP_COUNT ms
 *                         last SYSID_SS_COUNT ms averaged → vel_ss
 *   Phase 1 (coast-down): Vq = 0, log theta_dot_mech at 1 kHz for 1 s
 *
 * B/J estimated from log-linear fit of the coast-down decay:
 *   theta_dot(t) = theta_dot_0 * exp(-B/J * t)
 */

#ifndef INC_SYSID_H_
#define INC_SYSID_H_

#include "FOC.h"

// ── tuneable parameters ────────────────────────────────────────────────────
#define SYSID_N_SAMPLES      500    // coast-down buffer length: 1 s at 1 kHz
#define SYSID_VQ             0.5f    // open-loop spin-up voltage [V]
#define SYSID_SPINUP_COUNT   2000    // spin-up duration [1 kHz ticks = 2 s]
#define SYSID_SS_COUNT       100     // averaging window for vel_ss at end of spin-up [ticks]
#define SYSID_VEL_LIMIT      500.0f  // hard abort if |omega| exceeds this [rad/s]
// ──────────────────────────────────────────────────────────────────────────

extern float            sysid_vel_buf[SYSID_N_SAMPLES]; // coast-down velocity log [rad/s]
extern float            sysid_vel_ss;                   // averaged velocity at end of spin-up [rad/s]
extern volatile uint8_t sysid_done;                     // set by ISR when buffer full
extern volatile uint8_t sysid_aborted;                  // set by ISR on velocity limit trip

void sysid_reset(foc_t *foc);        // call before entering MODE_SYSID_COASTDOWN
void sysid_step(foc_t *foc);         // call at 1 kHz from ISR (TORQUE_DIVIDER gate)
void sysid_commutate(foc_t *foc);    // call at 30 kHz from ISR — applies open-loop Vq
void sysid_print_if_done(void);      // call from main context (_HW_Process_Pending_Ints)

#endif /* INC_SYSID_H_ */

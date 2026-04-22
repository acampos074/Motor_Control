/*
 * SysID.c
 *
 * Coast-down system identification.
 * sysid_step() is called at 1 kHz (TORQUE_DIVIDER gate in ISR).
 * commutate() continues to run at 30 kHz throughout all phases so the
 * current PI stays active and iq tracks the commanded reference cleanly.
 */

#include "SysID.h"
#include "FOC.h"
#include <math.h>
#include <stdio.h>

// ── public state ──────────────────────────────────────────────────────────
float            sysid_vel_buf[SYSID_N_SAMPLES];
float            sysid_vel_ss = 0.0f;
volatile uint8_t sysid_done   = 0;

// ── private state (reset by sysid_reset) ─────────────────────────────────
static uint8_t  sysid_phase   = 0;
static uint16_t stable_count  = 0;
static float    vel_prev      = 0.0f;
static float    ss_sum        = 0.0f;
static uint16_t ss_count      = 0;
static uint16_t sysid_idx     = 0;

void sysid_reset(foc_t *foc)
{
    sysid_phase   = 0;
    stable_count  = 0;
    vel_prev      = 0.0f;
    ss_sum        = 0.0f;
    ss_count      = 0;
    sysid_idx     = 0;
    sysid_vel_ss  = 0.0f;
    sysid_done    = 0;
    foc->pi.iq_ref       = 0.0f;
    foc->pi.id_ref       = 0.0f;
    foc->pi.Sum_iq_error = 0.0f;
    foc->pi.Sum_id_error = 0.0f;
}

void sysid_step(foc_t *foc)
{
    switch (sysid_phase)
    {
        case 0: // ── spin-up: hold iq until velocity stabilises ──────────
            foc->pi.iq_ref = SYSID_IQ;
            foc->pi.id_ref = 0.0f;

            if (fabsf(foc->theta_dot_mech - vel_prev) < SYSID_VEL_STABLE_THRESH) {
                stable_count++;
            } else {
                stable_count = 0;
            }
            vel_prev = foc->theta_dot_mech;

            if (stable_count >= SYSID_STABLE_COUNT) {
                ss_sum      = 0.0f;
                ss_count    = 0;
                sysid_phase = 1;
            }
            break;

        case 1: // ── record steady-state velocity ─────────────────────────
            foc->pi.iq_ref = SYSID_IQ;
            foc->pi.id_ref = 0.0f;
            ss_sum += foc->theta_dot_mech;
            ss_count++;

            if (ss_count >= SYSID_SS_COUNT) {
                sysid_vel_ss = ss_sum / (float)SYSID_SS_COUNT;
                // zero integrators so PI starts coast-down from a clean state
                foc->pi.Sum_iq_error = 0.0f;
                foc->pi.Sum_id_error = 0.0f;
                foc->pi.iq_ref = 0.0f;
                foc->pi.id_ref = 0.0f;
                sysid_idx   = 0;
                sysid_phase = 2;
            }
            break;

        case 2: // ── coast-down: log velocity decay ───────────────────────
            foc->pi.iq_ref = 0.0f;
            foc->pi.id_ref = 0.0f;

            if (sysid_idx < SYSID_N_SAMPLES) {
                sysid_vel_buf[sysid_idx++] = foc->theta_dot_mech;
            }

            if (sysid_idx >= SYSID_N_SAMPLES) {
                sysid_done = 1;
                foc->mode  = MODE_IDLE;
                set_zero_DC();
            }
            break;

        default:
            break;
    }
}

void sysid_print_if_done(void)
{
    if (!sysid_done) return;

    if (sysid_vel_ss < 0.05f) {
        printf("\r\n[SysID] WARNING: vel_ss = %.4f rad/s — motor may not have moved.\r\n"
               "        Try increasing SYSID_IQ above stiction torque.\r\n",
               sysid_vel_ss);
    } else {
        float B = KT * SYSID_IQ / sysid_vel_ss;
        printf("\r\n--- Coast-down SysID ---\r\n");
        printf("vel_ss : %.4f rad/s\r\n", sysid_vel_ss);
        printf("B      : %.6f Nm/(rad/s)\r\n", B);
        printf("\r\nt_ms,vel_rad_s\r\n");
        for (int i = 0; i < SYSID_N_SAMPLES; i++) {
            printf("%d,%.4f\r\n", i, sysid_vel_buf[i]);
        }
        printf("--- end ---\r\n");
    }

    sysid_done = 0;
}

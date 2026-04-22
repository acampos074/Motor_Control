/*
 * SysID.c
 *
 * Coast-down system identification.
 * sysid_step()      : called at 1 kHz (TORQUE_DIVIDER gate in ISR)
 * sysid_commutate() : called at 30 kHz from ISR — applies open-loop Vq
 *
 * Phase 0 (spin-up) : sysid_vq_active = SYSID_VQ for SYSID_SPINUP_COUNT ticks.
 *                     Last SYSID_SS_COUNT ticks averaged → vel_ss, iq_ss.
 * Phase 1 (coast-down): sysid_vq_active = 0, log theta_dot_mech for 2 s.
 *
 * B estimated from torque balance at steady state:
 *   KT * iq_ss = B * omega_ss  →  B = KT * iq_ss / omega_ss
 *
 * B/J estimated from log-linear least-squares fit of coast-down:
 *   log(omega(t)) = log(omega_0) - (B/J)*t  →  slope = -B/J
 */

#include "main.h"   // must come first — provides HAL types needed by FOC.h
#include "SysID.h"
#include "DebugLog.h"
#include <math.h>
#include <stdio.h>

// ── public state ──────────────────────────────────────────────────────────
float            sysid_vel_buf[SYSID_N_SAMPLES];
float            sysid_vel_ss  = 0.0f;
float            sysid_iq_ss   = 0.0f;
volatile uint8_t sysid_done    = 0;
volatile uint8_t sysid_aborted = 0;

// ── private state (reset by sysid_reset) ─────────────────────────────────
static uint8_t  sysid_phase     = 0;
static uint16_t sysid_tick      = 0;
static float    ss_vel_sum      = 0.0f;
static float    ss_iq_sum       = 0.0f;
static uint16_t ss_count        = 0;
static uint16_t sysid_idx       = 0;
static float    sysid_vq_active = 0.0f; // applied by sysid_commutate() at 30 kHz

void sysid_reset(foc_t *foc)
{
    sysid_phase     = 0;
    sysid_tick      = 0;
    ss_vel_sum      = 0.0f;
    ss_iq_sum       = 0.0f;
    ss_count        = 0;
    sysid_idx       = 0;
    sysid_vel_ss    = 0.0f;
    sysid_iq_ss     = 0.0f;
    sysid_done      = 0;
    sysid_aborted   = 0;
    sysid_vq_active = 0.0f;
    foc->pi.iq_ref       = 0.0f;
    foc->pi.id_ref       = 0.0f;
    foc->pi.Sum_iq_error = 0.0f;
    foc->pi.Sum_id_error = 0.0f;
}

void sysid_step(foc_t *foc)
{
    // hard safety cutoff — abort on any phase if motor spins too fast
    if (fabsf(foc->theta_dot_mech) > SYSID_VEL_LIMIT) {
        sysid_vq_active      = 0.0f;
        foc->pi.iq_ref       = 0.0f;
        foc->pi.id_ref       = 0.0f;
        foc->pi.Sum_iq_error = 0.0f;
        foc->pi.Sum_id_error = 0.0f;
        foc->mode            = MODE_IDLE;
        set_zero_DC();
        sysid_aborted = 1;
        sysid_done    = 1;
        return;
    }

    switch (sysid_phase)
    {
        case 0: // ── open-loop spin-up: apply SYSID_VQ for SYSID_SPINUP_COUNT ticks ──
        {
            sysid_vq_active = SYSID_VQ;
            sysid_tick++;

            // average vel and iq during the last SYSID_SS_COUNT ticks of spin-up
            if (sysid_tick > (SYSID_SPINUP_COUNT - SYSID_SS_COUNT)) {
                ss_vel_sum += foc->theta_dot_mech;
                ss_iq_sum  += foc->i_q;
                ss_count++;
            }

            if (sysid_tick % 100 == 0) {
                dbg_log("[sysID p0] tick:%u vq:%.2f vel:%.2f iq:%.3f\r\n",
                        sysid_tick, (double)sysid_vq_active,
                        (double)foc->theta_dot_mech, (double)foc->i_q);
            }

            if (sysid_tick >= SYSID_SPINUP_COUNT) {
                sysid_vel_ss = (ss_count > 0) ? ss_vel_sum / (float)ss_count : 0.0f;
                sysid_iq_ss  = (ss_count > 0) ? ss_iq_sum  / (float)ss_count : 0.0f;

                sysid_vq_active = 0.0f;
                foc->pi.iq_ref  = 0.0f;
                foc->pi.id_ref  = 0.0f;
                sysid_idx       = 0;
                sysid_phase     = 1;
            }
            break;
        }

        case 1: // ── coast-down: log velocity decay ───────────────────────────────
        {
            sysid_vq_active = 0.0f;
            foc->pi.iq_ref  = 0.0f;
            foc->pi.id_ref  = 0.0f;

            if (sysid_idx < SYSID_N_SAMPLES) {
                sysid_vel_buf[sysid_idx++] = foc->theta_dot_mech;
            }

            if (sysid_idx >= SYSID_N_SAMPLES) {
                sysid_done = 1;
                foc->mode  = MODE_IDLE;
                set_zero_DC();
            }
            break;
        }

        default:
            break;
    }
}

void sysid_commutate(foc_t *foc)
{
    // measure dq currents — needed for iq_ss averaging and for the IIR filter state
    dq0(foc->theta_elec, foc->i_a, foc->i_b, foc->i_c, &foc->i_d, &foc->i_q);

    // keep IIR filter running so i_q is well-conditioned for iq_ss averaging
    static float id_filt = 0.0f;
    static float iq_filt = 0.0f;
    id_filt = IQ_FILTER_ALPHA * foc->i_d + (1.0f - IQ_FILTER_ALPHA) * id_filt;
    iq_filt = IQ_FILTER_ALPHA * foc->i_q + (1.0f - IQ_FILTER_ALPHA) * iq_filt;
    foc->i_d = id_filt;
    foc->i_q = iq_filt;

    // open-loop drive: Vd = 0, Vq = sysid_vq_active (set by sysid_step)
    float u, v, w;
    uvw(foc->theta_elec, 0.0f, sysid_vq_active, &u, &v, &w);
    svm(u, v, w, foc->VM, &foc->dc_u, &foc->dc_v, &foc->dc_w);
    set_duty_cycle(foc->phase_order_flag, foc->dc_u, foc->dc_v, foc->dc_w);
}

void sysid_print_if_done(void)
{
    if (!sysid_done) return;

    if (sysid_aborted) {
        printf("\r\n[SysID] ABORTED: |omega| exceeded %.1f rad/s. Motor stopped.\r\n"
               "        Lower SYSID_VQ or raise SYSID_VEL_LIMIT.\r\n",
               (double)SYSID_VEL_LIMIT);
        sysid_done    = 0;
        sysid_aborted = 0;
        return;
    }

    if (sysid_vel_ss < 0.5f) {
        printf("\r\n[SysID] WARNING: vel_ss = %.4f rad/s — motor may not have spun up.\r\n"
               "        Try increasing SYSID_VQ or SYSID_SPINUP_COUNT.\r\n",
               (double)sysid_vel_ss);
    } else {
        // B from torque balance at steady state: KT*iq_ss = B*omega_ss
        float B = KT * sysid_iq_ss / sysid_vel_ss;

        // B/J from log-linear least-squares fit of coast-down
        float sum_t = 0.0f, sum_logv = 0.0f, sum_t2 = 0.0f, sum_t_logv = 0.0f;
        int   n = 0;
        float vel_thresh = sysid_vel_ss * 0.05f; // 5% of vel_ss — stays above noise floor
        for (int i = 0; i < SYSID_N_SAMPLES; i++) {
            if (sysid_vel_buf[i] > vel_thresh) {
                float t    = (float)i * 0.001f;
                float logv = logf(sysid_vel_buf[i]);
                sum_t      += t;
                sum_logv   += logv;
                sum_t2     += t * t;
                sum_t_logv += t * logv;
                n++;
            }
        }
        float BoverJ = 0.0f;
        if (n > 10) {
            float denom = (float)n * sum_t2 - sum_t * sum_t;
            if (fabsf(denom) > 1e-12f) {
                float slope = ((float)n * sum_t_logv - sum_t * sum_logv) / denom;
                BoverJ = -slope;
            }
        }
        float J = (BoverJ > 1e-9f) ? B / BoverJ : 0.0f;

        printf("\r\n--- Coast-down SysID ---\r\n");
        printf("vel_ss : %.4f rad/s\r\n",                 (double)sysid_vel_ss);
        printf("iq_ss  : %.4f A\r\n",                     (double)sysid_iq_ss);
        printf("B      : %.6f Nm/(rad/s)  (KT*iq_ss/omega_ss)\r\n", (double)B);
        printf("B/J    : %.6f rad/s^2     (log-linear fit)\r\n",     (double)BoverJ);
        printf("J      : %.6f kg.m^2      (B / (B/J))\r\n",          (double)J);
        printf("\r\nt_ms,vel_rad_s\r\n");
        for (int i = 0; i < SYSID_N_SAMPLES; i++) {
            printf("%d,%.4f\r\n", i, (double)sysid_vel_buf[i]);
        }
        printf("--- end ---\r\n");
    }

    sysid_done = 0;
}

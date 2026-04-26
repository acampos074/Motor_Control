/*
 * EccComp.c
 *
 * Encoder eccentricity compensation.
 *
 * calibrate_HES2() stores error_arr[N_CAL] = (raw - commanded) mod ENC_COUNTS.
 * The DC mean of this array is elec_offset (already subtracted in sample_HES).
 * The LUT captures the residual per-position AC error:
 *
 *   lut[b] = dc_mean - avg(error_arr[i] for samples i in bin b)
 *
 * Adding lut[b] to the raw count removes the position-dependent deviation
 * that elec_offset (a single scalar) cannot.
 */

#include "main.h"
#include "EccComp.h"
#include "Calibration.h"

int16_t ecc_lut[ECC_LUT_SIZE] = {0};
uint8_t ecc_lut_valid          = 0;

void ecc_build_lut(calibration_t *cal)
{
    // DC mean — same value used for elec_offset in calibrate_HES2()
    int32_t dc_sum = 0;
    for (int i = 0; i < N_CAL; i++)
        dc_sum += cal->error_arr[i];
    int32_t dc_mean = dc_sum / N_CAL;

    // Each bin holds exactly NPP = N_CAL/ECC_LUT_SIZE consecutive samples.
    // Compute avg directly with a nested loop — no intermediate arrays needed.
    for (int b = 0; b < ECC_LUT_SIZE; b++) {
        int32_t bin_sum = 0;
        int     start   = b * NPP;
        for (int k = 0; k < NPP; k++)
            bin_sum += cal->error_arr[start + k];
        ecc_lut[b] = (int16_t)(dc_mean - (bin_sum / NPP));
    }

    ecc_lut_valid = 1;
}

int16_t ecc_correct(uint16_t raw_count)
{
    if (!ecc_lut_valid) return 0;
    return ecc_lut[raw_count >> 7];   // raw * ECC_LUT_SIZE / ENC_COUNTS
}

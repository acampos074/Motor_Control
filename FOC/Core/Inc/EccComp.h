/*
 * EccComp.h
 *
 * Encoder eccentricity compensation via a 128-entry int16_t LUT.
 *
 * The LUT is built once from calibrate_HES2() error data by ecc_build_lut().
 * At runtime, ecc_correct() returns the correction (encoder counts) to add
 * to the raw position before computing theta_mech / theta_elec.
 *
 * LUT indexing:  bin = raw_count >> 7   (raw * 128 / 16384)
 * Application:   corrected = (raw + ecc_correct(raw)) & 0x3FFF
 */

#ifndef INC_ECCCOMP_H_
#define INC_ECCCOMP_H_

#include <stdint.h>
#include "FOC.h"

#define ECC_LUT_SIZE  128   // entries per mechanical revolution; must equal SAMPLES_PER_PPAIR

extern int16_t  ecc_lut[ECC_LUT_SIZE]; // correction table [encoder counts]
extern uint8_t  ecc_lut_valid;         // 1 once ecc_build_lut() has run

// Build LUT from calibration error array.  Call from main context after calibration.
void   ecc_build_lut(calibration_t *cal);

// Return correction for a given raw encoder count.  Safe to call from ISR.
int16_t ecc_correct(uint16_t raw_count);

#endif /* INC_ECCCOMP_H_ */

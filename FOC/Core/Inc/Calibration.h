/*
 * Calibration.h
 *
 *  Created on: Jun 26, 2022
 *      Author: Andres Campos
 */

#ifndef INC_CALIBRATION_H_
#define INC_CALIBRATION_H_

void calibrate_HES2(foc_t *foc,calibration_t *cal,float loop_count,hes_t *hes);
void open_loop_test(foc_t *foc,calibration_t *cal,float loop_count);

// Set by ISR at end of calibration; cleared by cal_print_error_lut()
extern volatile uint8_t cal_dump_pending;

// Call from main context to print the full error_arr as CSV
void cal_print_error_lut(void);

#endif /* INC_CALIBRATION_H_ */

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

#endif /* INC_CALIBRATION_H_ */

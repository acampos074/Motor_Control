/*
 * HES.h
 *
 *  Created on: Jun 25, 2022
 *      Author: Andres Campos
 */

#ifndef INC_HES_H_
#define INC_HES_H_

//#include "main.h"
//#include "Calibration.h"
#include "FOC.h"
#include <stdint.h>

#define ENC_COUNTS       16384.0f
#define RADS_PER_COUNTS  0.000383495197f // 2*PI/16384 [rads/counts]
#define W0_READ		     0x01 // for a read command
#define ANGLEUNC	     0x3FFE // Measured angle without dynamic angle error compensation
#define ANGLECOM	     0x3FFF // Measured angle with dynamic angle error compensation
#define PARC             0x00


void sample_HES(foc_t *foc,calibration_t *cal,hes_t *hes);
void warmup_HES(void);
void print_HES(foc_t *foc);
uint8_t HES_ReadSensorDMA(hes_t *hes);
void HES_ReadSensorDMA_Complete(hes_t *hes);
uint16_t compensate_eccentricity(uint16_t position_raw);


#endif /* INC_HES_H_ */

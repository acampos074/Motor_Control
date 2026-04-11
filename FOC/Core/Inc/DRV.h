/*
 * DRV.h
 *
 *  Created on: Jun 25, 2022
 *      Author: Andres Campos
 */

#ifndef INC_DRV_H_
#define INC_DRV_H_

#include "FOC.h"
#include <stdint.h>

// DCR Defines
#define W0_WRITE					0x00 // for a write command
#define W0_READ						0x01 // for a read command
#define DCR 						0x02 // Driver Control Register
#define PWM_MODE_3X					0x01 // 3x PWM Mode
#define CLR_FLT						0x01 // to clear latched fault bits


// CSA
#define CSACR 						0x06 // Current Sense Amplifier Control Register
#define SEN_LVL_1V					0x03
#define SEN_LVL_0_25V				0x00 // Sense OCP 0.25V
#define CSA_GAIN_40					0x03
#define CSA_CAL_A_EN				0x01
#define CSA_CAL_A_DIS				0x00
#define CSA_CAL_B_EN				0x01
#define CSA_CAL_B_DIS				0x00
#define CSA_CAL_C_EN				0x01
#define CSA_CAL_C_DIS				0x00
#define VREF_DIV_BI					0x01 // Reference voltage is divided by 2
#define DIS_SEN_EN					0x00 // Sense overcurrent fault is enabled
#define DIS_SEN_DIS					0x01 // Sense overcurrent fault is disabled

// OCP
#define OCPCR 						0x05 // Overcurrent Protection Control Register
#define TRETRY_50US					0x01 // VDS_OCP & SEN_OCP retry time is 50-microseconds
#define DEAD_TIME_50NS				0x00 // 50ns dead time
#define VDS_LVL_0_45V				0x05
#define VDS_LVL_1_88V				0x0F
#define OCP_MODE_RETRY				0x01 // Overcurrent causes an automatic retrying fault
#define OCP_DEG_4US					0x01
#define OCP_DEG_8US					0x03






#endif /* INC_DRV_H_ */

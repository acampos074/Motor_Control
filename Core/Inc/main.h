/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

int float_to_uint16(float x, float x_min, float x_max);
float uint16_to_float(int x_int, float x_min, float x_max);
float uint8_to_float(int x_int, float x_min, float x_max);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_iB_Pin GPIO_PIN_0
#define ADC_iB_GPIO_Port GPIOC
#define ADC_iA_Pin GPIO_PIN_1
#define ADC_iA_GPIO_Port GPIOC
#define ADC_VM_Pin GPIO_PIN_0
#define ADC_VM_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define DEBUG_PIN_Pin GPIO_PIN_5
#define DEBUG_PIN_GPIO_Port GPIOC
#define DRV_CS_Pin GPIO_PIN_12
#define DRV_CS_GPIO_Port GPIOB
#define DRV_SCK_Pin GPIO_PIN_13
#define DRV_SCK_GPIO_Port GPIOB
#define DRV_MISO_Pin GPIO_PIN_14
#define DRV_MISO_GPIO_Port GPIOB
#define DRV_MOSI_Pin GPIO_PIN_15
#define DRV_MOSI_GPIO_Port GPIOB
#define PWM_C_Pin GPIO_PIN_8
#define PWM_C_GPIO_Port GPIOA
#define PWM_B_Pin GPIO_PIN_9
#define PWM_B_GPIO_Port GPIOA
#define PWM_A_Pin GPIO_PIN_10
#define PWM_A_GPIO_Port GPIOA
#define DRV_ENABLE_Pin GPIO_PIN_11
#define DRV_ENABLE_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define HES_CS_Pin GPIO_PIN_15
#define HES_CS_GPIO_Port GPIOA
#define HES_SCK_Pin GPIO_PIN_10
#define HES_SCK_GPIO_Port GPIOC
#define HES_MISO_Pin GPIO_PIN_11
#define HES_MISO_GPIO_Port GPIOC
#define HES_MOSI_Pin GPIO_PIN_12
#define HES_MOSI_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ES_Port.h"
#include "ES_Timers.h"
#include "HES.h"
#include "FOC.h"
#include "Calibration.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static volatile uint8_t TickCount;
static volatile uint32_t DacValue;

static volatile uint16_t HES;
//static volatile uint16_t tx_msg;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DAC_HandleTypeDef hdac;
extern foc_t foc;
extern calibration_t cal;
extern hes_t hes;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  ++TickCount;
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream5 global interrupt.
  */
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi3_tx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
	//uint16_t adc_a_raw,adc_b_raw;
	//uint16_t DacVal = 0;

	//int counter = 0;
	/*
	if(DacValue < 4095){
		//DacValue = DacValue + 1;
		//DacValue = 4094;
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DacValue);
		DacValue++;
	}
	else{
		DacValue = 0;
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DacValue);
	}
	*/



	// 1. Read HES position
	//foc.theta_mech_raw = READ_HES(); // returns the raw position measurement
	//foc.theta_mech = (float)(foc.theta_mech_raw)*RADS_PER_COUNTS;
	//foc.theta_elec = foc.theta_mech*NPP*TWO_PI; // Needs more work !!
	//foc.theta_mech_raw = HES;
	// 2. Read current ia & ib
  	//adc_a_raw = HAL_ADC_GetValue(&hadc1);
  	//adc_b_raw = HAL_ADC_GetValue(&hadc2);
	/*
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc3);
	foc.adc2_ia_raw = HAL_ADC_GetValue(&hadc2); // i_a
	foc.adc1_ib_raw = HAL_ADC_GetValue(&hadc1); // i_b
	foc.VM_raw = HAL_ADC_GetValue(&hadc3); // motor voltage
	*/
	// === Sample HES ===
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
	/*
	if(DacValue == 0)
	{
		DacValue = 4095;
	}
	else
	{
		DacValue = 0;
	}
	*/
    // !!!!!!!!!!!!!!!!!!!!!!!
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);
	sample_ADC(&foc); // 4.2us !!!!!!!!!!!!!!!!!!!!!!!!!!
	//HES_ReadSensorDMA(&hes);
	sample_HES(&foc,&cal,&hes);

	//sample_ADC(&foc); // 4.2us !!!!!!!!!!!!!!!!!!!!!!!!!!

	//for(int i=0;i<10000;i++){}

	//printf("%ld\r\n",TIM1->CNT);


	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);
	//HES_ReadSensorDMA(&hes);
	//sample_HES(&foc,&cal,&hes); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  	// if controller flag is on, then commutate
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);
	if(foc.voltage_FOC_flag == 1)
	{
		commutate(&foc,foc.theta_elec);

		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
		//printf("%d \r\n",foc.torque_control_counter);
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)(foc.i_q*100000) );
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)(foc.i_q*4096));
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t)(foc.i_d*4096));
	}

	else if(foc.controller_flag == 1)
	{
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);

				// Uncomment for local torque controller
		        // TORQUE_DIVIDER = F_PWM / TORQUE_LOOP_HZ — updates automatically when F_PWM changes
		        if(foc.torque_control_counter > TORQUE_DIVIDER)
				//if(foc.torque_control_counter > 25 && foc.torque_control_counter_total < 25*100)
				{
					torque_control(&foc); // **** ==== Uncomment to implement torque control within the MCU ==== ****
					foc.torque_control_counter = 0;

					 /*
					foc.torque_control_counter_total++;
					if(foc.torque_control_counter_total == 25*100)
					{
						HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);// debug pin
						foc.iq_ref = 0;
						foc.id_ref = 0;
						set_zero_DC();
					}
					 */

					//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
				}

				foc.torque_control_counter++;
				//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);
				commutate_v2(&foc,foc.theta_elec);
				//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

	}

	else if(foc.cal_flag == 1)
	{
		cal.loop_count++;
		calibrate_HES2(&foc,&cal,cal.loop_count,&hes);
	}
	else if(foc.open_loop_test_flag == 1)
	{
		cal.loop_count++;
		open_loop_test(&foc,&cal,cal.loop_count);
	}
	else if(foc.systemID_flag == 1)
	{
		cal.loop_count++;
		commutate(&foc,foc.theta_elec);
		//DacVal = (uint16_t)(foc.i_d*4096);
		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DacVal );
		if(cal.loop_count > 400) // 10ms (400 samples at 40khz)
		{
			foc.systemID_flag = 0; // set flag to zero to exit system ID state
			cal.loop_count = 0;
			set_zero_DC();
			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		}
		printf("ia:%.3f ib:%.3f ic:%.3f u:%.3f v:%.3f w:%.3f id:%.3f iq:%.3f\r\n",foc.i_a,foc.i_b,foc.i_c,foc.v_u,foc.v_v,foc.v_w,foc.i_d,foc.i_q);

	}
	else
	{
		// do nothing
	}
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)((foc.theta_dot_mech*4095)/50.0f));
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);


  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
bool _HW_Process_Pending_Ints( void )
{
   while (TickCount > 0)
   {
      /* call the framework tick response to actually run the timers */
      ES_Timer_Tick_Resp();
      TickCount--;
   }
   return true; // always return true to allow loop test in ES_Run to proceed
}
/* USER CODE END 1 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_Port.h"
#include "termio.h"

#include <stdio.h>
#include "stm32f4xx.h"                  // Device header

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "DRV.h"
#include "FOC.h"
#include "HES.h"
#include "Calibration.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define clrScrn() 	printf("\x1b[2J")
#define goHome()	printf("\x1b[1,1H")
#define clrLine()	printf("\x1b[K")

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t UART2_rxBuffer;
uint8_t RxData = 0;
static volatile uint32_t DacState;
static volatile uint32_t DacState_2;

uint8_t CANFlag = 0;

//static double global_count = 0;

CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxMessage;
CAN_RxHeaderTypeDef RxMessage;
uint8_t CANTxData[8];
uint32_t TxMailbox;
uint8_t CANRxData[8];

uint8_t hes_buffer_rx[2];
uint16_t hes_tx_message = ( (PARC<<15) | (W0_READ<<14) | ANGLEUNC );
uint8_t hes_buffer_tx[2]={0x00,0x00};

//SPI_HandleTypeDef hspi3;

//float dtc_u;
//float dtc_v;
//float dtc_w;
//uint32_t adc_a_raw;
//uint32_t adc_b_raw;
//uint32_t adc_a_offset = 0;
//uint32_t adc_b_offset = 0;

//ADC_HandleTypeDef hadc1;
//ADC_HandleTypeDef hadc2;
//SPI_HandleTypeDef hspi2;

foc_t foc;
calibration_t cal;
hes_t hes;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void init_DRV(foc_t* foc);
/*
int float_to_uint(float x, float x_min, float x_max);
float uint16_to_float(int x_int, float x_min, float x_max);
float uint8_to_float(int x_int, float x_min, float x_max);
*/
//void zero_current(foc_t *foc);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//hes.counter++;
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5); // debug pin
	//sample_ADC(&foc); // 4.2us !!!!!!!!!!!!!!!!!!!!!!!!!!
	//HES_ReadSensorDMA(&hes);
	//sample_HES(&foc,&cal,&hes);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);

//
//	//for(int i=0;i<10000;i++){}
//
//	//printf("%ld\r\n",TIM1->CNT);
//
//
//	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
//	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);
	HES_ReadSensorDMA(&hes);
//	sample_HES(&foc,&cal,&hes); //!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
//
//	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//  	// if controller flag is on, then commutate
//	if(foc.voltage_FOC_flag == 1)
//	{
//		commutate(&foc,foc.theta_elec);
//
//		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
//		//printf("%d \r\n",foc.torque_control_counter);
//		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)(foc.i_q*100000) );
//		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)(foc.i_q*4096));
//		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t)(foc.i_d*4096));
//	}
//
//	else if(foc.controller_flag == 1)
//	{
//		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
//
//				// Uncomment for local torque controller
//				if(foc.torque_control_counter > 29)  // use 25 if using 25khz sampling freq
//				//if(foc.torque_control_counter > 25 && foc.torque_control_counter_total < 25*100)
//				{
//					torque_control(&foc); // **** ==== Uncomment to implement torque control within the MCU ==== ****
//					foc.torque_control_counter = 0;
//
//					 /*
//					foc.torque_control_counter_total++;
//					if(foc.torque_control_counter_total == 25*100)
//					{
//						HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);// debug pin
//						foc.iq_ref = 0;
//						foc.id_ref = 0;
//						set_zero_DC();
//					}
//					 */
//
//					//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//				}
//
//				foc.torque_control_counter++;
//				//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4094);
//				commutate_v2(&foc,foc.theta_elec);
//				//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
//
//	}
//
//	else if(foc.cal_flag == 1)
//	{
//		cal.loop_count++;
//		calibrate_HES2(&foc,&cal,cal.loop_count,&hes);
//	}
//	else if(foc.open_loop_test_flag == 1)
//	{
//		cal.loop_count++;
//		open_loop_test(&foc,&cal,cal.loop_count);
//	}
//	else if(foc.systemID_flag == 1)
//	{
//		cal.loop_count++;
//		commutate(&foc,foc.theta_elec);
//		//DacVal = (uint16_t)(foc.i_d*4096);
//		//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DacVal );
//		if(cal.loop_count > 200)
//		{
//			foc.systemID_flag = 0; // set flag to zero to exit system ID state
//			cal.loop_count = 0;
//			set_zero_DC();
//			//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//		}
//		printf("ia:%.3f ib:%.3f ic:%.3f u:%.3f v:%.3f w:%.3f id:%.3f iq:%.3f\r\n",foc.i_a,foc.i_b,foc.i_c,foc.v_u,foc.v_v,foc.v_w,foc.i_d,foc.i_q);
//
//	}
//	else
//	{
//		// do nothing
//	}
//	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint16_t)((foc.theta_dot_mech*4095)/50.0f));
//	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	//hes.counter++;
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5); // debug pin
	HES_ReadSensorDMA_Complete(&hes);
	/*
	if(hspi->Instance == SPI3)
	{
		HES_ReadSensorDMA_Complete(&hes);
	}
	*/
	//while(1);
	/*
	hes_tx_message = ( (PARC<<15) | (W0_READ<<14) | ANGLEUNC ); // ANGLEUNC ANGLECOM
	hes_buffer_tx[0] = (uint8_t) (hes_tx_message & 0x00FF);
	hes_buffer_tx[1] = (uint8_t) ((hes_tx_message & 0xFF00)>>8);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive_DMA(&hspi,hes_buffer_tx,hes_buffer_rx,2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	foc.theta_mech_raw = ( (hes_buffer_rx[1]<<8) | hes_buffer_rx[0] ) & 0x3FFF;
	*/
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	//foc.theta_mech_raw = ( (hes_buffer_rx[1]<<8) | hes_buffer_rx[0] ) & 0x3FFF;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_DAC_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */

  /* CAN Config */
  HAL_CAN_Start(&hcan1);
  // Activate Rx CAN interrupt


  CANTxData[0] = 0x01;
  CANTxData[1] = 0x02;
  CANTxData[2] = 0x03;
  CANTxData[3] = 0x04;
  CANTxData[4] = 0x05;
  CANTxData[5] = 0x06;
  CANTxData[6] = 0x07;
  CANTxData[7] = 0x08; // in a single command, one can send 64 bits of data max

  /* DAC Config */
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  DacState = 0;
  DacState_2 = 0;

  /*ADC Config */
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  /* Initialize the SPI DMA */

	//hes_buffer_tx[0] = (uint8_t) (hes_tx_message & 0x00FF);
	//hes_buffer_tx[1] = (uint8_t) ((hes_tx_message & 0xFF00)>>8);
  //hes->spiHandle = hspi3;



  hes_tx_message = ( (PARC<<15) | (W0_READ<<14) | ANGLEUNC ); // ANGLEUNC ANGLECOM
  hes_buffer_tx[0] = (uint8_t) (hes_tx_message & 0x00FF);
  hes_buffer_tx[1] = (uint8_t) ((hes_tx_message & 0xFF00)>>8);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive_DMA(&hspi3,hes_buffer_tx,hes_buffer_rx,2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  foc.theta_mech_raw = ( (hes_buffer_rx[1]<<8) | hes_buffer_rx[0] ) & 0x3FFF;


  /* Init Terminal */
  //TERMIO_Init(); commented out since we're using HAL drivers for this /* Initialize Serial interface */
  clrScrn();

  printf("\n\r\n");
  printf("Starting Test Harness for \r\n");
  printf("the 2nd Generation Events & Services Framework V2.2\r\n");
  printf("%s %s\n",__TIME__, __DATE__);
  printf("\n\r\n");
  printf("MENU:\n\r");
  printf("Press 'q' Print List of Commands \n\r");
  printf("Press 'c' Print Variables \n\r");
  printf("Press 'k' Calibrate the HES \n\r");
  printf("Press 'p' Open Loop Test Post HES Calibration \n\r");
  printf("Press 'f' Run Voltage FOC \n\r");
  printf("Press 'i' System ID \n\r");
  printf("Press 'm' Turn ON the PI Controller \n\r");
  printf("Press 'o' Turn OFF the PI Controller \n\r");
  printf("Press 's' Query current state \n\r");
  printf("\n\r\n");

  /* Reset FOC Variables */
  reset_variables(&foc,&cal,&hes,&hspi3);
  warmup_HES();
  HAL_Delay(1000); // 1000 ms delay

  /* DRV INIT */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
  HAL_Delay(2000); // 1000 ms delay
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // start with CS line high for DRV
  init_DRV(&foc);

  /* PWM Config */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  TIM1->CCR1 = (TIM1->ARR)*1050*0; // PA8
  TIM1->CCR2 = (TIM1->ARR)*500*0; // PA9
  TIM1->CCR3 = (TIM1->ARR)*250*0; // PA10

  /* Debug Pin */
  //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);

  /* HES CS Line */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET); // start with CS line high for HES

  /* Set Interrupt Priorities */
  NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1); // commutation > communication
  NVIC_SetPriority(CAN1_RX0_IRQn, 3);
  NVIC_SetPriority(USART2_IRQn, 4);

  /* Set DMA interrupt priority and enable */
  //MX_DMA_Init();

  /* Enable Interrupts */
  HAL_UART_Receive_IT(&huart2, &UART2_rxBuffer, 1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

  // Now initialize the Events and Services Framework and start it running
  ES_Return_t ErrorType;
  ErrorType = ES_Initialize(ES_Timer_RATE_1mS); // SysTick 1 msec interrupts

  if ( ErrorType == Success ) {

	  ErrorType = ES_Run();
  }
  //if we got to here, there was an error
  switch (ErrorType){
  	  case FailedPost:
	    printf("Failed on attempt to Post\n");
	    break;
	  case FailedPointer:
	    printf("Failed on NULL pointer\n");
	    break;
	  case FailedInit:
	    printf("Failed Initialization\n");
	    break;
	 default:
	    printf("Other Failure\n");
	    break;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//printf("count %f \n\r",global_count);
	//HAL_Delay(1000); // 1000 ms delay
//	printf("\r\nHello World");

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

int __io_putchar(int ch) {
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 0xffff);
	return 0;
}

//int __io_getchar(void) {
//	HAL_UART_Receive(&huart2,(uint8_t *)UART2_rxBuffer, 1, 0xffff);
//	return 0;
//}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    //HAL_UART_Transmit(&huart2, &UART2_rxBuffer, 1, 100);
	RxData=1;
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5); // Toggle Green LED
    HAL_UART_Receive_IT(&huart2, &UART2_rxBuffer, 1);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	//double GearRatio = 12.0;
	if(DacState == 0)
	{
		DacState = 4095;
		DacState_2 = 0;

	}
	else
	{
		DacState = 0;
		DacState_2 = 4095;
	}

	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DacState);



  // Read CAN
	//CANFlag = 1;
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5);
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5); // Toggle Green LED
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5); // Toggle Green LED
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxMessage, CANRxData); // CAN Rx
	//compose_msg(&TxMessage,foc.theta_mech,foc.theta_dot_mech);
	int theta_int,theta_dot_int,torque_cmd_int, led_flag, gen_cmd, dq_cmd_int,q_cmd_int,vq_cmd_int,iq_cmd_int,can_rx_vq_cmd_int,can_rx_iq_cmd_int;
	uint8_t kp_int,kd_int;
	float theta_float,theta_dot_float, dq_cmd_float,q_cmd_float,torque_cmd_float,kp_float,kd_float;

	//theta_int = float_to_uint(foc.theta_mech,-6.2831f,6.2831f);
	theta_int = float_to_uint16(foc.theta_mech_multiturn[0],-TWO_PI*GR,TWO_PI*GR);
	theta_dot_int = float_to_uint16(foc.theta_dot_mech,-SPEED_MAX,SPEED_MAX);
	//torque_cmd_int = float_to_uint(20.0f,-40.0f,40.0f);
	//torque_cmd_int = float_to_uint16(foc.i_a,-40.0f,40.0f);
	vq_cmd_int = float_to_uint16(foc.vq_cmd,-V_MAX,V_MAX); // control voltage signal
	iq_cmd_int = float_to_uint16(foc.i_q,-I_MAX,I_MAX); // measured current signal
	// This is for troubleshooting the CAN Tx message
	//theta_int = float_to_uint(1.521,-6.2831f,6.2831f);
	//theta_dot_int = float_to_uint(21.23,-65.0f,65.0f);
	//theta_float = uint16_to_float(theta_int,-6.2831f,6.2831f);
	//theta_dot_float = uint16_to_float(theta_dot_int,-SPEED_MAX,SPEED_MAX);

	CANTxData[0] = theta_int>>8;    // get 8 MSB
	CANTxData[1] = theta_int&0xFF;  // get 8 LSB
	CANTxData[2] = theta_dot_int>>8;
	CANTxData[3] = theta_dot_int&0xFF;
	//CANTxData[4] = torque_cmd_int>>8;
	//CANTxData[5] = torque_cmd_int&0xFF;
	CANTxData[4] = vq_cmd_int>>8;
	CANTxData[5] = vq_cmd_int&0xFF;
	CANTxData[6] = iq_cmd_int>>8;
	CANTxData[7] = iq_cmd_int&0xFF;


	//for(int i=0;i<1000;i++){} // Delay
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, CANTxData, &TxMailbox); // CAN Tx
	//decompose_msg(&TxMessagefoc.can_rx_torques);
	//int torque_int = (CANRxData[0]<<8 | CANRxData[1]);



	//printf("KP: %u \n\r",kp_int);
	//printf("KP: %.3f \n\r",foc.kp);
	//printf("KD: %.3f \n\r",foc.kd);
	/*
	if(led_flag == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // turn LED on
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET); // turn LED off
	}
	*/
	led_flag = CANRxData[0];
	gen_cmd = CANRxData[1];
	// switch statements to parse different CAN messages
	switch(gen_cmd)
	{
		case 1: // calibrate HES
			//foc.cal_flag = 1;
			CANFlag = 1;
			break;
		case 2: // turn off PI controller
			CANFlag = 2;
			/*
			foc.cal_flag = 0;
			foc.controller_flag = 0;
			foc.voltage_FOC_flag = 0;
			set_zero_DC();
			*/
			break;
		case 3: // run voltage FOC
			CANFlag = 3;
			//foc.voltage_FOC_flag = 1;
			can_rx_vq_cmd_int = (CANRxData[2]<<8 | CANRxData[3]);
			foc.can_rx_vq_cmd = uint16_to_float(can_rx_vq_cmd_int,-V_MAX,V_MAX);
			break;
		case 4: // position controller on
			CANFlag = 4;

			q_cmd_int = (CANRxData[2]<<8 | CANRxData[3]);
			kp_int = CANRxData[4];
			kd_int = CANRxData[5];
			q_cmd_float = uint16_to_float(q_cmd_int,-TWO_PI*GR,TWO_PI*GR);
			foc.can_rx_q = q_cmd_float;
			//foc.p_des = q_cmd_float;
			kp_float = uint8_to_float(kp_int,0.0f,0.1/foc.GAIN);
			kd_float = uint8_to_float(kd_int,0.0f,0.1/foc.GAIN);
			foc.kp = kp_float;
			foc.kd = kd_float;
			/*
		    foc.first_sample = 0.0;
		    foc.iq_ref = 0.0;
		    foc.id_ref = 0.0;
		    foc.theta_dot_mech_cmd = 0.1f;
		    //foc.theta_dot_mech_cmd = 30.0f;
		    foc.torque_control_counter_total = 0;
		    foc.p_des = 0.09f;
		    foc.controller_flag = 1;
		    foc.pos_control_flag = 1;
		    */
		    break;
		case 5: // speed controller on
			CANFlag = 5;
			dq_cmd_int = (CANRxData[2]<<8 | CANRxData[3]);
			kp_int = CANRxData[4];
			kd_int = CANRxData[5];
			dq_cmd_float = uint16_to_float(dq_cmd_int,-SPEED_MAX,SPEED_MAX);
			foc.can_rx_dq = dq_cmd_float;
			kp_float = uint8_to_float(kp_int,0.0f,0.1/foc.GAIN);
			kd_float = uint8_to_float(kd_int,0.0f,0.1/foc.GAIN);
			foc.kp = kp_float;
			foc.kd = kd_float;
			break;
		case 6: // dyno test on
			CANFlag = 3;
			can_rx_vq_cmd_int = (CANRxData[2]<<8 | CANRxData[3]);
			foc.can_rx_vq_cmd = uint16_to_float(can_rx_vq_cmd_int,-V_MAX,V_MAX);
			break;
		case 7:
			CANFlag = 7;
			break;
		case 8:
			CANFlag = 8;
			can_rx_iq_cmd_int = (CANRxData[2]<<8 | CANRxData[3]);
			foc.can_rx_iq_cmd = uint16_to_float(can_rx_iq_cmd_int,-I_MAX,I_MAX);
			break;
		case 9:
			CANFlag = 9;
			torque_cmd_int = (CANRxData[2]<<8 | CANRxData[3]);
			kp_int = CANRxData[4];
			kd_int = CANRxData[5];
			torque_cmd_float = uint16_to_float(torque_cmd_int,-TAU_MAX,TAU_MAX);
			foc.can_rx_torque_cmd = torque_cmd_float;
			kp_float = uint8_to_float(kp_int,0.0f,0.1/foc.GAIN);
			kd_float = uint8_to_float(kd_int,0.0f,0.1/foc.GAIN);
			foc.kp = kp_float;
			foc.kd = kd_float;
			break;
		default :
			CANFlag = 0;
	}

	/*
	dq_cmd_float = uint16_to_float(dq_cmd_int,-65.0f,65.0f);
	foc.can_rx_dq = dq_cmd_float;
	kp_float = uint8_to_float(kp_int,0.0f,0.1f);
	kd_float = uint8_to_float(kd_int,0.0f,0.1f);
	foc.kp = kp_float;
	foc.kd = kd_float;
	*/

	//foc.can_rx_torques = uint16_to_float(torque_int,-I_MAX*KT*GR,I_MAX*KT*GR);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, DacState_2);
	//printf("tau: %.3f \n\r",foc.can_rx_torques);
	//printf("tau: %.3f theta_mech: %.3f theta_dot_mech: %.3f\n\r",foc.can_rx_torques,foc.theta_mech,foc.theta_dot_mech);

	// ===== Comment to turn off torque control from Mujoco to MCU =====
	//foc.iq_ref = foc.can_rx_torques /(KT*GR);
	foc.id_ref = 0.0f;
	//for(int i=0;i<8;i++){
	//	CANTxData[i] = CANRxData[i];
	//}
	// send CAN message
	//HAL_CAN_AddTxMessage(&hcan1, &TxMessage, CANTxData, &TxMailbox);
	//printf("%d %.3f %d %d %.3f %.3f %.3f %.3f\n\r",torque_int,foc.can_rx_torques,theta_int,theta_dot_int,theta_float,foc.theta_mech,theta_dot_float,foc.theta_dot_mech);
	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

}

int float_to_uint16(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x-offset)*((float)((1<<16)-1))/span);
}

float uint16_to_float(int x_int, float x_min, float x_max)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<16)-1)) + offset;
}

float uint8_to_float(int x_int, float x_min, float x_max)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<8)-1)) + offset;
}

void init_DRV(foc_t* foc)
{
	uint32_t tx_msg = 0;
	uint8_t SPI_DRV_DATA_TX[2] = {0x00,0x00};
	uint8_t SPI_DRV_DATA_RX[2] = {0x00,0x00};

    foc->adc1_offset = 0;
    foc->adc2_offset = 0;

	// Enable DRV
	printf("\n\r DRV ENABLE: \n\r");
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(1000); // 1000 ms delay

	// === Data Control Register ===
	tx_msg = ( (W0_WRITE<<15) | (DCR<<11) | (PWM_MODE_3X<<5) | (CLR_FLT) );
	SPI_DRV_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
	SPI_DRV_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
	printf("\n\r DCR Config: \n\r");
  	printf("\rTX[0]: 0x%04X",SPI_DRV_DATA_TX[0]);
  	printf("\r\n");
  	printf("\rTX[1]: 0x%04X",SPI_DRV_DATA_TX[1]);
  	printf("\r\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, SPI_DRV_DATA_TX, SPI_DRV_DATA_RX, 1, 100);
	// wait for transmission complete
	while( hspi2.State == HAL_SPI_STATE_BUSY );
	// Set CS line high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  	printf("\rRX[0]: 0x%04X",SPI_DRV_DATA_RX[0]);
  	printf("\r\n");
  	printf("\rRX[1]: 0x%04X",SPI_DRV_DATA_RX[1]);
  	printf("\r\n");
	HAL_Delay(1000); // 1000 ms delay
	tx_msg = 0;
	tx_msg = ( (W0_READ<<15) | (DCR<<11) );
	SPI_DRV_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
	SPI_DRV_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, SPI_DRV_DATA_TX, SPI_DRV_DATA_RX, 1, 100);
	// wait for transmission complete
	while( hspi2.State == HAL_SPI_STATE_BUSY );
	// Set CS line high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  	printf("\rOCP[0]: 0x%04X",SPI_DRV_DATA_RX[0]);
  	printf("\r\n");
  	printf("\rOCP[1]: 0x%04X",SPI_DRV_DATA_RX[1]);
  	printf("\r\n");

	// === Current sense amplifier control register ===
	tx_msg = 0;
	//tx_msg = ( (W0_WRITE<<15) | (CSACR<<11) | (VREF_DIV_BI<<9) | (CSA_GAIN_40<<6) | (DIS_SEN_EN<<5) | (CSA_CAL_A_EN<<4)| (CSA_CAL_B_EN<<3) | (CSA_CAL_C_EN<<2) | (SEN_LVL_0_25V) );
	tx_msg = ( (W0_WRITE<<15) | (CSACR<<11) | (VREF_DIV_BI<<9) | (CSA_GAIN_40<<6) | (DIS_SEN_EN<<5) | (CSA_CAL_A_EN<<4)| (CSA_CAL_B_EN<<3) | (SEN_LVL_0_25V) );
	//tx_msg = ( (W0_WRITE<<15) | (CSACR<<11) | (VREF_DIV_BI<<9) | (CSA_GAIN_40<<6) | (CSA_CAL_A_EN<<4)| (CSA_CAL_B_EN<<3) | (SEN_LVL_1V) );
	SPI_DRV_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
	SPI_DRV_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
	printf("\n\r CSA Config 1: \n\r");
  	printf("\rTX[0]: 0x%04X",SPI_DRV_DATA_TX[0]);
  	printf("\r\n");
  	printf("\rTX[1]: 0x%04X",SPI_DRV_DATA_TX[1]);
  	printf("\r\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, SPI_DRV_DATA_TX, SPI_DRV_DATA_RX, 1, 100);
	// wait for transmission complete
	while( hspi2.State == HAL_SPI_STATE_BUSY );
	// Set CS line high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  	printf("\rRX[0]: 0x%04X",SPI_DRV_DATA_RX[0]);
  	printf("\r\n");
  	printf("\rRX[1]: 0x%04X",SPI_DRV_DATA_RX[1]);
  	printf("\r\n");
	HAL_Delay(1000); // 1000 ms delay

	// === Calibrate current sense amplifier ===
	zero_current(foc);
	//printf("\rHere!!!!\r\n");
	HAL_Delay(1000); // 1000 ms delay

	// === Current sense amplifier control register ===
	tx_msg = 0;
	//tx_msg = ( (W0_WRITE<<15) | (CSACR<<11) | (VREF_DIV_BI<<9) | (CSA_GAIN_40<<6) | (DIS_SEN_EN<<5) | (CSA_CAL_A_DIS<<4)| (CSA_CAL_B_DIS<<3) | (CSA_CAL_C_DIS<<2) | (SEN_LVL_0_25V) );
	tx_msg = ( (W0_WRITE<<15) | (CSACR<<11) | (VREF_DIV_BI<<9) | (CSA_GAIN_40<<6) | (DIS_SEN_EN<<5) | (CSA_CAL_A_DIS<<4)| (CSA_CAL_B_DIS<<3) | (SEN_LVL_0_25V) );
	SPI_DRV_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
	SPI_DRV_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
	printf("\n\r CSA Config 2: \n\r");
  	printf("\rTX[0]: 0x%04X",SPI_DRV_DATA_TX[0]);
  	printf("\r\n");
  	printf("\rTX[1]: 0x%04X",SPI_DRV_DATA_TX[1]);
  	printf("\r\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, SPI_DRV_DATA_TX, SPI_DRV_DATA_RX, 1, 100);
	// wait for transmission complete
	while( hspi2.State == HAL_SPI_STATE_BUSY );
	// Set CS line high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  	printf("\rRX[0]: 0x%04X",SPI_DRV_DATA_RX[0]);
  	printf("\r\n");
  	printf("\rRX[1]: 0x%04X",SPI_DRV_DATA_RX[1]);
  	printf("\r\n");
	HAL_Delay(1000); // 1000 ms delay
	tx_msg = 0;
	tx_msg = ( (W0_READ<<15) | (CSACR<<11) );
	SPI_DRV_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
	SPI_DRV_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, SPI_DRV_DATA_TX, SPI_DRV_DATA_RX, 1, 100);
	// wait for transmission complete
	while( hspi2.State == HAL_SPI_STATE_BUSY );
	// Set CS line high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  	printf("\rOCP[0]: 0x%04X",SPI_DRV_DATA_RX[0]);
  	printf("\r\n");
  	printf("\rOCP[1]: 0x%04X",SPI_DRV_DATA_RX[1]);
  	printf("\r\n");

	// === Overcurrent protection control register ===
	tx_msg = 0;
	tx_msg = ( (W0_WRITE<<15) | (OCPCR<<11) | (TRETRY_50US<<10) | (DEAD_TIME_50NS<<8) | (OCP_MODE_RETRY<<6) | (OCP_DEG_4US<<4) | (VDS_LVL_0_45V) );
	SPI_DRV_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
	SPI_DRV_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
	printf("\n\r OCP Config: \n\r");
  	printf("\rTX[0]: 0x%04X",SPI_DRV_DATA_TX[0]);
  	printf("\r\n");
  	printf("\rTX[1]: 0x%04X",SPI_DRV_DATA_TX[1]);
  	printf("\r\n");
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, SPI_DRV_DATA_TX, SPI_DRV_DATA_RX, 1, 100);
	// wait for transmission complete
	while( hspi2.State == HAL_SPI_STATE_BUSY );
	// Set CS line high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  	printf("\rRX[0]: 0x%04X",SPI_DRV_DATA_RX[0]);
  	printf("\r\n");
  	printf("\rRX[1]: 0x%04X",SPI_DRV_DATA_RX[1]);
  	printf("\r\n");
	HAL_Delay(1000); // 1000 ms delay
	tx_msg = 0;
	tx_msg = ( (W0_READ<<15) | (OCPCR<<11) );
	SPI_DRV_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
	SPI_DRV_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, SPI_DRV_DATA_TX, SPI_DRV_DATA_RX, 1, 100);
	// wait for transmission complete
	while( hspi2.State == HAL_SPI_STATE_BUSY );
	// Set CS line high
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  	printf("\rOCP[0]: 0x%04X",SPI_DRV_DATA_RX[0]);
  	printf("\r\n");
  	printf("\rOCP[1]: 0x%04X",SPI_DRV_DATA_RX[1]);
  	printf("\r\n");

}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

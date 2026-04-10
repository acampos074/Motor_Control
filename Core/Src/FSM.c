/****************************************************************************
 Module
   FSM.c

 Revision
   1.0.0

 Description
   This is the finite state machine of the motor controller

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 06/02/22 09:04 aca      initial release
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for the framework and this service
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "FSM.h"

#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include <stdlib.h>
#include <stdint.h>
#include "FOC.h"
#include "DRV.h"
#include "HES.h"
#include "Calibration.h"
/*----------------------------- Module Defines ----------------------------*/
/*
#define RADS_PER_COUNTS  0.000383495197f //
#define W0_READ		0x01 // for a read command
#define ANGLEUNC	0x3FFE // Measured angle without dynamic angle error compensation
#define ANGLECOM	0x3FFF // Measured angle with dynamic angle error compensation
#define PARC        0x00
*/
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
//uint16_t DRV_Read(uint16_t reg);

/*---------------------------- Module Variables ---------------------------*/
// type of state variable
static TemplateState_t CurrentState;
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
extern TIM_HandleTypeDef htim1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
uint32_t AdcValue = 0;
extern CAN_TxHeaderTypeDef TxMessage;
extern CAN_RxHeaderTypeDef RxMessage;
extern uint8_t CANTxData[8];
extern uint32_t TxMailbox;
extern CAN_HandleTypeDef hcan1;
extern uint8_t CANFlag;

extern SPI_HandleTypeDef hspi2;
uint8_t SPI_DATA_TX[2] = {0xFB, 0xD8};

extern DAC_HandleTypeDef hdac;
extern foc_t foc;
extern calibration_t cal;
extern hes_t hes;
//extern static double global_count;
//extern volatile double global_count;

uint16_t VM_sense;

/*
extern SPI_HandleTypeDef hspi3;

uint8_t SPI_HES_DATA_TX[2];
uint8_t SPI_HES_DATA_RX[2];
uint16_t HES;
uint16_t tx_msg;
*/

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitFSM

 Parameters
     uint8_t : the priority of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     A. Campos, 06/02/22, 09:05
****************************************************************************/
bool InitFSM ( uint8_t Priority )
{
  ES_Event ThisEvent;
  MyPriority = Priority;
  // Go to the initial PseudoState
  CurrentState = InitPState;
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;

  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}

/****************************************************************************
 Function
     PostFSM

 Parameters
     ES_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     A. Campos, 06/02/22, 09:08
****************************************************************************/
bool PostFSM( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunFSM

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   
 Author
   A. Campos, 06/02/22, 09:10
****************************************************************************/
ES_Event RunFSM( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  switch ( CurrentState )
  {
  	  case InitPState : // if current state is initial pseudo state
  	       if ( ThisEvent.EventType == ES_INIT )// only respond to ES_Init
  	        {
  	    	   	 // Initialize the drv8323 & calibrates current amplifier sensors

  	            CurrentState = Waiting4UserInput;
  	         }
  	         break;
  	  case CalibratingHES :
  		  // Trapped in this state until cal_flag is set to Zero
  		  if(foc.cal_flag == 0)
  		  {
  			CurrentState = Waiting4UserInput;
  		  }
  		  break;
  	  case OpenLoopTest :
  		  // Trapped in this state until cal_flag is set to Zero
  		  if(foc.open_loop_test_flag == 0)
  		  {
  			CurrentState = Waiting4UserInput;
  		  }
  		  break;
  	  case SystemID :
  		  // Trapped in this state until systemID_flag is set to Zero
  		  if(foc.systemID_flag == 0)
  		  {
  			CurrentState = Waiting4UserInput;
  		  }
  		  break;
  	  case Waiting4UserInput : // if current state is waiting for user input
  		  switch ( ThisEvent.EventType )
  		  {
  		  	  case ES_NEW_KEY : // if we get a keystroke event
  		        if( 'c' == ThisEvent.EventParam ){
  		      	  //printf("\rHES Calibration\r\n");
  		      	  //HAL_ADC_Start(&hadc1);
  		      	  //HAL_ADC_Start(&hadc2);
  		      	  //AdcValue = HAL_ADC_GetValue(&hadc2);
  		      	  //printf("\rADC: %f V", (float)((AdcValue*3.3)/4096.0));
  		      	  printf("\r\n");
  		      	  // start calibration routine
  		      	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5); // debug pin
  		      	  sample_HES(&foc,&cal,&hes);
  		      	  sample_ADC(&foc);
  		      	  printf("\rADC_offset_1: %u cnts", foc.adc1_offset);
  		      	  printf("\r\n");
  		      	  printf("\rADC_offset_2: %u cnts", foc.adc2_offset);
  		      	  printf("\r\n");
  		      	  printf("\rADC_raw_2: %u cnts", foc.adc2_ia_raw);
  		      	  printf("\r\n");
  		      	  printf("\rADC_raw_1: %u cnts", foc.adc1_ib_raw);
  		      	  printf("\r\n");
  		      	  printf("\ri_a: %f A", foc.i_a);
  		      	  printf("\r\n");
  		      	  printf("\ri_b: %f A", foc.i_b);
  		      	  printf("\r\n");
  		      	  printf("\ri_c: %f A", foc.i_c);
  		      	  printf("\r\n");
  		      	  printf("\rTorque: %f Nm",foc.iq_ref*(KT*GR));
  		      	  printf("\r\n");
  		      	  printf("\riq_ref: %f A",foc.iq_ref);
  		      	  printf("\r\n");
  		      	  printf("\ri_q: %f A",foc.i_q);
  		      	  printf("\r count %f ",hes.counter);
  		      	  //foc.theta_mech_raw = READ_HES();
  		      	  //printf("\rHES_raw: %u cnts", foc.theta_mech_raw);
  		      	  printf("\r\n");
  		      	  //HAL_ADC_Start(&hadc3);
  		      	  //VM_sense = HAL_ADC_GetValue(&hadc3);
  		      	  printf("\rVM_Sense: %f V", foc.VM);
  		      	  printf("\r\n");
  		      	  print_HES(&foc);
  		      	  print_flags(&foc);
  		      	  printf("\rCAN Torques: %f Nm", foc.can_rx_torques);
  		      	  printf("\r\n");



  		      	  /*
  		      	  tx_msg = ( (PARC<<15) | (W0_READ<<14) | ANGLEUNC ); // ANGLEUNC ANGLECOM
  		      	  printf("\rtx_msg: 0x%04X",tx_msg);
  		      	  SPI_HES_DATA_TX[0] = (uint8_t) (tx_msg & 0x00FF);
  		      	  SPI_HES_DATA_TX[1] = (uint8_t) ((tx_msg & 0xFF00)>>8);
  		      	  printf("\rTX[0]: 0x%04X",SPI_HES_DATA_TX[0]);
  		      	  printf("\r\n");
  		      	  printf("\rTX[1]: 0x%04X",SPI_HES_DATA_TX[1]);
  		      	  printf("\r\n");
  		      	  // SPI read/write
  		      	  // Lower CS line
  		      	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
  		      	  HAL_SPI_TransmitReceive(&hspi3, SPI_HES_DATA_TX, SPI_HES_DATA_RX, 1, 100);
  		      	  // wait for transmission complete
  		      	  while( hspi3.State == HAL_SPI_STATE_BUSY );
  		      	  // Set CS line high
  		      	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
  		      	  printf("\rRX[0]: 0x%04X",SPI_HES_DATA_RX[0]);
  		      	  printf("\r\n");
  		      	  printf("\rRX[1]: 0x%04X",SPI_HES_DATA_RX[1]);
  		      	  printf("\r\n");
  		      	  HES = ( (SPI_HES_DATA_RX[1]<<8) | SPI_HES_DATA_RX[0] );
  		      	  HES = HES & 0x3FFF;
  		      	  printf("\rHES: %u",HES);
  		      	  printf("\r\n");
  		      	  //HES = (uint16_t)(( ((SPI_HES_DATA_RX[1]<<8 | SPI_HES_DATA_RX[0]) & 0x3FFF)*4095.0f)/16384.0f); // 14bit = 16384
  		      	  HES = (uint16_t) ((HES*4095.0f)/16384.0f);
  		      	  //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, HES);
  		      	  */
  		        }
  		        else if( 'k' == ThisEvent.EventParam ){
  		        	printf("\rHES Calibration \r\n");
  		        	foc.cal_flag = 1;
  		        	CurrentState = CalibratingHES;
  		        }
  		        else if( 'p' == ThisEvent.EventParam ){
  		        	printf("\rOpen Loop Test \r\n");
  		        	//cal.loop_count = 0;
  		        	foc.open_loop_test_flag = 1;
  		        	CurrentState = OpenLoopTest;
  		        }
  		        else if( 'f' == ThisEvent.EventParam ){
  		        	//printf("\rVoltage FOC Test \r\n");
  		        	foc.voltage_FOC_flag = 1;
  		        }
  		        else if( 'd' == ThisEvent.EventParam ){
  		        	printf("\rNew Set Point \r\n");
  		        	foc.p_des += 1.0f;
  		        	if(foc.p_des > TWO_PI)
  		        	{
  		        		foc.p_des = (foc.p_des-TWO_PI);
  		        	}
  		        	//foc.voltage_FOC_flag = 1;
  		        }
  		        else if( 'q' == ThisEvent.EventParam ){
  		        	Print_Serial_Commands();
  		        }
  		        else if( 'i' == ThisEvent.EventParam ){
  		        	printf("\rSystem ID Test \r\n");
  		        	//cal.loop_count = 0;
  		        	foc.systemID_flag = 1;
  		        	CurrentState = SystemID;
  		        	set_zero_DC();
  		        	//HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
  		        }
  		        else if( 'm' == ThisEvent.EventParam ){
  		      	  //printf("\rController ON\r\n");
  		          // Pack response
  		      	  /*
  		      	  CANTxData[0] = 0x01;
  		      	  CANTxData[1] = 0x02;
  		      	  CANTxData[2] = 0x03;
  		      	  CANTxData[3] = 0x04;
  		      	  CANTxData[4] = 0x05;
  		      	  CANTxData[5] = 0x06;
  		      	  CANTxData[6] = 0x07;
  		      	  CANTxData[7] = 0x08;
  		      	  */
  		      	  // Transmit the CAN message
  		      	  //HAL_CAN_AddTxMessage(&hcan1, &TxMessage, CANTxData, &TxMailbox);
  		      	  //CANFlag = 1;
  		      	  //HAL_Delay(1000);
  		      	  // send non-zero torque command

  		      	  // set flag to true
  		      	  //HAL_TIM_Base_Start_IT(&htim1);
  		      	  //foc.iq_ref = 0.1*0; // anything less than 0.1 the controller has issues with tracking
  		    	  //foc.id_ref = 0.2;
  		      	  //foc.iq_ref = 0.08; // anything less than 0.1 the controller has issues with tracking
  		    	  //foc.id_ref = 0.0;
  		      	  //foc.first_sample = 0.0;
  		      	  //foc.iq_ref = 0.0;
  		      	  //foc.id_ref = 0.0;
  		      	  foc.theta_dot_mech_cmd = 0.1f;
  		      	  //foc.theta_dot_mech_cmd = 30.0f;
  		      	  //foc.torque_control_counter_total = 0;
  		      	  //foc.p_des = 0.09f;
  		      	  foc.controller_flag = 1;
  		      	  foc.pos_control_flag = 1;
		      	  foc.speed_control_flag = 0;
		      	  foc.current_control_flag = 0;
		      	  foc.torque_control_flag = 0;

  		        }
  		        else if('w' == ThisEvent.EventParam )
  		        {
  		        	//printf("\rSpeed Controller ON\r\n");
    		      	  //foc.first_sample = 0.0;
    		      	  //foc.iq_ref = 0.0;
    		      	  //foc.id_ref = 0.0;
    		      	  //foc.theta_dot_mech_cmd = 0.1f;
    		      	  //foc.theta_dot_mech_cmd = 30.0f;
    		      	  //foc.torque_control_counter_total = 0;
    		      	  //foc.p_des = 0.09f;
    		      	  foc.controller_flag = 1;
    		      	  foc.pos_control_flag = 0;
    		      	  foc.speed_control_flag = 1;
    		      	  foc.current_control_flag = 0;
    		      	  foc.torque_control_flag = 0;

  		        }
  		        else if('x' == ThisEvent.EventParam )
  		        {
    		      	  foc.controller_flag = 1;
    		      	  foc.pos_control_flag = 0;
    		      	  foc.speed_control_flag = 0;
    		      	  foc.current_control_flag = 1;
    		      	  foc.torque_control_flag = 0;
  		        }
  		        else if('z' == ThisEvent.EventParam )
  		        {
    		      	  foc.controller_flag = 1;
    		      	  foc.pos_control_flag = 0;
    		      	  foc.speed_control_flag = 0;
    		      	  foc.current_control_flag = 0;
    		      	  foc.torque_control_flag = 1;
  		        }
  		        else if( 'o' == ThisEvent.EventParam ){
  		      	 printf("\rController OFF\r\n");
  		      	 // send zero torque command to controller
  		      	 // Lower CS line
  		      	 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
  		      	 //HAL_SPI_Transmit(&hspi2, SPI_DATA_TX, 1, 100);
  		      	 // Set CS line high
  		      	 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  		      	 //HAL_TIM_Base_Stop_IT(&htim1);
  		      	 foc.cal_flag = 0;
  		      	 foc.controller_flag = 0;
  		      	 foc.voltage_FOC_flag = 0;
				 foc.pos_control_flag = 0;
				 foc.speed_control_flag = 0;
				 foc.current_control_flag = 0;
  		      	 set_zero_DC();
  		      	 //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0 );
  		      	 //HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0 );
  		      	 CurrentState = Waiting4UserInput;

  		        }
  		        else if ('s' == ThisEvent.EventParam ){
  		    	 if(QuerySM() == Waiting4UserInput){printf("\rWaiting4UserInput\r\n");}
  		    	 if(QuerySM() == InitPState){printf("\rInitPState\r\n");}
  		       }
  		        else{
  		        	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5); // Toggle Green LED
  		        	printf("\rDefault \r\n");
  		        }
  		       break;
  		  	  case ES_CAN_RX:
  		  		  printf("\rCAN RX!!!\r\n");
  		  		  //HAL_Delay(1);
  		  		  //for(int i=0;i<1000;i++){} // add a small delay
  		  		  // Build CAN TX packages with position and velocity
  		  		  //CANTxData[0] = 0x01;
  		  		  //CANTxData[1] = 0x02;
  		  		  //CANTxData[2] = 0x03;
  		  		  //CANTxData[3] = 0x04;
  		  		  //CANTxData[4] = 0x05;
  		  		  //CANTxData[5] = 0x06;
  		  		  //CANTxData[6] = 0x07;
  		  		  //CANTxData[7] = 0x08;
  		  		  //HAL_CAN_AddTxMessage(&hcan1, &TxMessage, CANTxData, &TxMailbox);
  		  		  CANFlag = 0;
  		  		  break;
  		  	  default :

  		  		  ;
  		  } // end switch of ES_NEW_KEY event
  		  break;
  	  default :
  			  ;

  }

  return ReturnEvent;
}

//  switch (ThisEvent.EventType){
//    case ES_INIT :
//      printf("\rES_INIT received in FSM\r\n");
//      break;
//    case ES_NEW_KEY :  // announce
//      if( 'c' == ThisEvent.EventParam ){
//    	  printf("\rHES Calibration\r\n");
//      }
//      if( 'm' == ThisEvent.EventParam ){
//    	  printf("\rController ON\r\n");
//      }
//     if( 'o' == ThisEvent.EventParam ){
//    	 printf("\rController OFF\r\n");
//      }
//     break;
//    case ES_DEFAULT :  // don't do anything
//      break;
//    default :
//      break;
//  }
//  return ReturnEvent;
//}

/****************************************************************************
 Function
     QuerySM

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the state machine
 Notes

 Author
     A. Campos, 06/02/22, 10:21
****************************************************************************/
TemplateState_t QuerySM ( void )
{
   return(CurrentState);
}

/***************************************************************************
 private functions
 ***************************************************************************/
//uint16_t DRV_Read(uint16_t reg)
//{
//	uint16_t val;
//
//	// send a read command and return the status on "reg" register address
//	GPIOB->ODR &= ~(0x01 << 12);
//	val = (SPI2->DR = ( (1<<15) | (reg<<11) ));
//	Delay(12); //6us delay
//	GPIOB->ODR |= (0x01 << 12);
//
//	val = SPI2->DR;
//	return val;
//}

void Print_Serial_Commands(void)
{
	  printf("\n\r\n");
	  printf("MENU:\n\r");
	  printf("Press 'q' Print List of Commands \n\r");
	  printf("Press 'c' Print Variables \n\r");
	  printf("Press 'k' Calibrate the HES \n\r");
	  printf("Press 'p' Open Loop Test Post HES Calibration \n\r");
	  printf("Press 'f' Run Voltage FOC \n\r");
	  printf("Press 'i' System ID \n\r");
	  printf("Press 'm' Turn ON the Position PI Controller \n\r");
	  printf("Press 'w' Turn ON the Speed PI Controller \n\r");
	  printf("Press 'o' Turn OFF the PI Controller \n\r");
	  printf("Press 's' Query current state \n\r");
	  printf("\n\r\n");
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/



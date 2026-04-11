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
/*---------------------------- Module Functions ---------------------------*/

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

extern DAC_HandleTypeDef hdac;
extern foc_t foc;
extern calibration_t cal;
extern hes_t hes;
//extern static double global_count;
//extern volatile double global_count;

uint16_t VM_sense;

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
  		  if(foc.mode == MODE_IDLE)
  		  {
  			CurrentState = Waiting4UserInput;
  		  }
  		  break;
  	  case OpenLoopTest :
  		  if(foc.mode == MODE_IDLE)
  		  {
  			CurrentState = Waiting4UserInput;
  		  }
  		  break;
  	  case SystemID :
  		  if(foc.mode == MODE_IDLE)
  		  {
  			CurrentState = Waiting4UserInput;
  		  }
  		  break;
  	  case Waiting4UserInput : // if current state is waiting for user input
  		  switch ( ThisEvent.EventType )
  		  {
  		  	  case ES_NEW_KEY : // if we get a keystroke event
  		        if( 'c' == ThisEvent.EventParam ){
  		      	  printf("\r\n");
  		      	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_5); // debug pin
  		      	  sample_HES(&foc,&cal,&hes);
  		      	  sample_ADC(&foc);
  		      	  printf("\rADC_offset_1: %u cnts", foc.pi.adc1_offset);
  		      	  printf("\r\n");
  		      	  printf("\rADC_offset_2: %u cnts", foc.pi.adc2_offset);
  		      	  printf("\r\n");
  		      	  printf("\rADC_raw_2: %u cnts", foc.pi.adc2_ia_raw);
  		      	  printf("\r\n");
  		      	  printf("\rADC_raw_1: %u cnts", foc.pi.adc1_ib_raw);
  		      	  printf("\r\n");
  		      	  printf("\ri_a: %f A", foc.i_a);
  		      	  printf("\r\n");
  		      	  printf("\ri_b: %f A", foc.i_b);
  		      	  printf("\r\n");
  		      	  printf("\ri_c: %f A", foc.i_c);
  		      	  printf("\r\n");
  		      	  printf("\rTorque: %f Nm",foc.pi.iq_ref*(KT*GR));
  		      	  printf("\r\n");
  		      	  printf("\riq_ref: %f A",foc.pi.iq_ref);
  		      	  printf("\r\n");
  		      	  printf("\ri_q: %f A",foc.i_q);
  		      	  printf("\r count %f ",hes.counter);
  		      	  printf("\r\n");
  		      	  printf("\rVM_Sense: %f V", foc.VM);
  		      	  printf("\r\n");
  		      	  print_HES(&foc);
  		      	  print_flags(&foc);
  		      	  printf("\rCAN Torques: %f Nm", foc.can_rx.torques);
  		      	  printf("\r\n");
  		        }
  		        else if( 'k' == ThisEvent.EventParam ){
  		        	printf("\rHES Calibration \r\n");
  		        	foc.mode = MODE_CALIBRATION;
  		        	CurrentState = CalibratingHES;
  		        }
  		        else if( 'p' == ThisEvent.EventParam ){
  		        	printf("\rOpen Loop Test \r\n");
  		        	foc.mode = MODE_OPEN_LOOP_TEST;
  		        	CurrentState = OpenLoopTest;
  		        }
  		        else if( 'f' == ThisEvent.EventParam ){
  		        	foc.mode = MODE_VOLTAGE_FOC;
  		        }
  		        else if( 'd' == ThisEvent.EventParam ){
  		        	printf("\rNew Set Point \r\n");
  		        	foc.p_des += 1.0f;
  		        	if(foc.p_des > TWO_PI)
  		        	{
  		        		foc.p_des = (foc.p_des-TWO_PI);
  		        	}
  		        }
  		        else if( 'q' == ThisEvent.EventParam ){
  		        	Print_Serial_Commands();
  		        }
  		        else if( 'i' == ThisEvent.EventParam ){
  		        	printf("\rSystem ID Test \r\n");
  		        	foc.mode = MODE_SYSTEM_ID;
  		        	CurrentState = SystemID;
  		        	set_zero_DC();
  		        }
  		        else if( 'm' == ThisEvent.EventParam ){
  		      	  foc.theta_dot_mech_cmd = 0.1f;
  		      	  foc.mode = MODE_POSITION;
  		        }
  		        else if('w' == ThisEvent.EventParam )
  		        {
  		        	foc.mode = MODE_SPEED;
  		        }
  		        else if('x' == ThisEvent.EventParam )
  		        {
  		        	foc.mode = MODE_CURRENT;
  		        }
  		        else if('z' == ThisEvent.EventParam )
  		        {
  		        	foc.mode = MODE_TORQUE;
  		        }
  		        else if( 'o' == ThisEvent.EventParam ){
  		      	 printf("\rController OFF\r\n");
  		      	 foc.mode = MODE_IDLE;
  		      	 set_zero_DC();
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



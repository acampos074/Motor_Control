/****************************************************************************
 
  Header file for FSM
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef FSM_H
#define FSM_H

#include "ES_Configure.h"
#include "ES_Types.h"

// State definitions for the query function
typedef enum { InitPState, CalibratingHES, OpenLoopTest,SystemID, Waiting4UserInput } TemplateState_t ;

// Public Function Prototypes

bool InitFSM ( uint8_t Priority );
bool PostFSM( ES_Event ThisEvent );
ES_Event RunFSM( ES_Event ThisEvent );
TemplateState_t QuerySM ( void );
void Print_Serial_Commands(void);

#endif /* ServTemplate_H */


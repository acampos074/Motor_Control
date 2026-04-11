/*
 * termio.h
 *
 *  Created on: Oct 24, 2020
 *      Author: Andres C.
 */

#ifndef TERMIO_H_
#define TERMIO_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "ES_Port.h"


/* receives character from the terminal channel - BLOCKING */
//unsigned char TERMIO_GetChar(void);

/* sends a character to the terminal channel
   wait for output buffer empty */
//void TERMIO_PutChar(unsigned char ch);

/* initializes the communication channel */
/* set baud rate to 115.2 kbaud and turn on Rx and Tx */
//void TERMIO_Init(void);
/* checks for a character from the terminal channel */
int kbhit(void);


#endif /* TERMIO_H_ */

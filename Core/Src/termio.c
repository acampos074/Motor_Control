/*-----------------------------------------------------------------------------
 * Name:    termio.c
 *-----------------------------------------------------------------------------*/

#include "stm32f4xx.h"                  // Device header
//#include "RTE_Components.h"             // Component selection
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include "termio.h"

// Baudrate
//#define USART_BAUDRATE          115200

// USART clock
//#define USART_CLK             40000000

/* Define  Baudrate setting (BRR) for USART */
//#define __DIV(__PCLK, __BAUD)       ((__PCLK*25)/(4*__BAUD))
//#define __DIVMANT(__PCLK, __BAUD)   (__DIV(__PCLK, __BAUD)/100)
//#define __DIVFRAQ(__PCLK, __BAUD)   (((__DIV(__PCLK, __BAUD) - (__DIVMANT(__PCLK, __BAUD) * 100)) * 16 + 50) / 100)
//#define __USART_BRR(__PCLK, __BAUD) ((__DIVMANT(__PCLK, __BAUD) << 4)|(__DIVFRAQ(__PCLK, __BAUD) & 0x0F))

/* Define  USART */
#define USARTx  USART2 


//void TERMIO_Init(void) {
//
//	#ifdef RTE_Compiler_IO_STDOUT_User
//  RCC->AHB1ENR  |=   ( 1ul <<  0);                        /* Enable GPIOA clock */
//  RCC->APB1ENR  |=   ( 1ul << 17);                        /* Enable USART#2 clock */
//
//  /* Configure PA3 to USART2_RX, PA2 to USART2_TX */
//  GPIOA->AFR[0] &= ~((15ul << 4* 3) | (15ul << 4* 2) );
//  GPIOA->AFR[0] |=  (( 7ul << 4* 3) | ( 7ul << 4* 2) );
//  GPIOA->MODER  &= ~(( 3ul << 2* 3) | ( 3ul << 2* 2) );
//  GPIOA->MODER  |=  (( 2ul << 2* 3) | ( 2ul << 2* 2) );
//
//  USARTx->BRR  = __USART_BRR(USART_CLK, USART_BAUDRATE);  /* 115200 baud @ 12MHz   */
//  USARTx->CR3    = 0x0000;                                /* no flow control */
//  USARTx->CR2    = 0x0000;                                /* 1 stop bit */
//  USARTx->CR1    = ((   1ul <<  2) |                      /* enable RX */
//                    (   1ul <<  3) |                      /* enable TX */
//	                  (   1ul <<  5) |                      /* enable RXNEIE */
//                    (   0ul << 12) |                      /* 1 start bit, 8 data bits */
//                    (   1ul << 13) );                     /* enable USART */
//
//	#endif
//
//}
//
//
///**
//  Put a character to the stdout
//
//  \param[in]   ch  Character to output
//  \return          The character written, or -1 on write error.
//*/
//int stdout_putchar (int ch) {
//
//	#ifdef RTE_Compiler_IO_STDOUT_User
//		while (!(USARTx->SR & 0x0080));
//		USARTx->DR = (ch & 0xFF);
//	#endif
//		return (ch);
//}
//
//void TERMIO_PutChar(unsigned char ch) {
//	#ifdef RTE_Compiler_IO_STDOUT_User
//		while (!(USARTx->SR & 0x0080));
//		USARTx->DR = (ch & 0xFF);
//	#endif
//
//}
//
///**
//  Get a character from stdin
//
//  \return          The character written, or -1 on write error.
//*/
//int stdin_getchar (void) {
//
//	int ch;
//	#ifdef RTE_Compiler_IO_STDIN_User
//		while (!(USARTx->SR & 0x0020));
//		ch = (USARTx->DR & 0xFF);
//	#endif
//  return (ch);
//}
//
//unsigned char TERMIO_GetChar(void) {
//	int ch;
//	#ifdef RTE_Compiler_IO_STDIN_User
//		while (!(USARTx->SR & 0x0020));
//		ch = (USARTx->DR & 0xFF);
//	#endif
//  return (ch);
//}

int kbhit(void) {
	/* checks for a character from the terminal channel */
	//if((USARTx->SR & 0x0020))
	//if((USARTx->CR1 & 0x0020))
	//if((USARTx->DR & 0x00))
	if(RxData == 1)
		return 1;
	else
		return 0;
}



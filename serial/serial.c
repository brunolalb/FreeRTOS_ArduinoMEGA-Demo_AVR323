/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/* serial.c
 *
 * Interrupt driven Serial port driver
 *
 *
 * Compiler: WinAVR
 * Burner: AVR Dude (STK500v2)
 * IDE: Eclipse Mars.2
 *
 * Description:
 * 	Based on the official Demo for AVR323 (WINAVR)
 * 	Implements only USART0
 *
 * Initial version (2016-08-11): Bruno Landau Albrecht (brunolalb@gmail.com)
 *
 */


#include <stdlib.h>
#include <avr/interrupt.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "serial.h"

#define serBAUD_DIV_CONSTANT			( ( uint32_t ) 8 )

/* Constants for writing to UCSRnA. */
#define serDOUBLE_SPEED					( ( unsigned char ) 0b00000010 )
#define serFRAME_ERROR_BIT				( ( unsigned char ) 0b00010000 )
#define serOVERRUN_ERROR_BIT			( ( unsigned char ) 0b00001000 )
#define serPARITY_ERROR_BIT				( ( unsigned char ) 0b00000100 )
#define serRECEIVE_COMPLETE				( ( unsigned char ) 0b10000000 )

/* Constants for writing to UCSRnB. */
#define serRX_INT_ENABLE				( ( unsigned char ) 0b10000000 )
#define serRX_ENABLE					( ( unsigned char ) 0b00010000 )
#define serTX_ENABLE					( ( unsigned char ) 0b00001000 )
#define serTX_INT_ENABLE				( ( unsigned char ) 0b00100000 )

/* Constants for writing to UCSRnC. */
#define serEIGHT_DATA_BITS				( ( unsigned char ) 0b00000110 )

static QueueHandle_t xRxedChars; 
static QueueHandle_t xCharsForTx; 

#define vInterrupt0On()										\
{															\
	unsigned char ucByte;									\
															\
	ucByte = UCSR0B;										\
	ucByte |= serTX_INT_ENABLE;								\
	UCSR0B = ucByte;										\
}																				
/*-----------------------------------------------------------*/

#define vInterrupt0Off()									\
{															\
	unsigned char ucInByte;									\
															\
	ucInByte = UCSR0B;										\
	ucInByte &= ~serTX_INT_ENABLE;							\
	UCSR0B = ucInByte;										\
}
/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( uint32_t ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
uint32_t ulBaudRateCounter;
unsigned char ucByte;

	portENTER_CRITICAL();
	{
		/* Create the queues used by the com test task. */
		xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );
		xCharsForTx = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( signed char ) );

		/* Calculate the baud rate register value from the equation in the data sheet, around page 203.
		 *	We'll be using double speed mode (U2Xn = 1)
		 *	UBRRn = (fOSC / (8*BAUD)) - 1
		 */
		ulBaudRateCounter = ( configCPU_CLOCK_HZ / ( serBAUD_DIV_CONSTANT * ulWantedBaud ) ) - ( uint32_t ) 1;


		/* Set the baud rate. */	
		UBRR0 = ulBaudRateCounter;

		/* Choose the double speed mode */
		ucByte = UCSR0A;
		ucByte |= ( serDOUBLE_SPEED );
		UCSR0A = ucByte;

		/* Enable the Rx interrupt.  The Tx interrupt will get enabled
		later. Also enable the Rx and Tx. */
		UCSR0B = ( serRX_INT_ENABLE | serRX_ENABLE | serTX_ENABLE );

		/* Set the data bits to 8, 1 stop bit, no parity (8n1). */
		UCSR0C = ( serEIGHT_DATA_BITS );
	}
	portEXIT_CRITICAL();
	
	/* Unlike other ports, this serial code does not allow for more than one
	com port.  We therefore don't return a pointer to a port structure and can
	instead just return NULL. */
	return NULL;
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime )
{
	/* Only one port is supported by now. */
	( void ) pxPort;

	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) )
	{
		return pdTRUE;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime )
{
	/* Only one port is supported by now. */
	( void ) pxPort;

	/* Return false if after the block time there is no room on the Tx queue. */
	if( xQueueSend( xCharsForTx, &cOutChar, xBlockTime ) != pdPASS )
	{
		return pdFAIL;
	}

	vInterrupt0On();

	return pdPASS;
}
/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
unsigned char ucByte;

	/* The parameter is not used. */
	( void ) xPort;

	/* Turn off the interrupts.  We may also want to delete the queues and/or
	re-install the original ISR. */

	portENTER_CRITICAL();
	{
		vInterrupt0Off();
		ucByte = UCSR0B;
		ucByte &= ~( serRX_INT_ENABLE | serRX_ENABLE | serTX_ENABLE );
		UCSR0B = ucByte;
	}
	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------
 * INTERRUPTS
 *-----------------------------------------------------------
 */

ISR( USART0_RX_vect )
{
signed char cChar;
unsigned char ucChar;
signed portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* If some problem occurred (Frame Error, Data Overrun or Parity), just flush the buffer and move on */
	ucChar = UCSR0A;
	if (ucChar & (serFRAME_ERROR_BIT | serOVERRUN_ERROR_BIT | serPARITY_ERROR_BIT))
	{
		while ( UCSR0A & serRECEIVE_COMPLETE) cChar = UDR0;
	}
	else
	{
		/* Get the character and post it on the queue of Rxed characters.
		If the post causes a task to wake force a context switch as the woken task
		may have a higher priority than the task we have interrupted. */
		cChar = UDR0;

		xQueueSendFromISR( xRxedChars, &cChar, &xHigherPriorityTaskWoken );

		if( xHigherPriorityTaskWoken != pdFALSE )
		{
			taskYIELD();
		}
	}
}
/*-----------------------------------------------------------*/

ISR( USART0_UDRE_vect )
{
signed char cChar, cTaskWoken;

	if( xQueueReceiveFromISR( xCharsForTx, &cChar, &cTaskWoken ) == pdTRUE )
	{
		/* Send the next character queued for Tx. */
		UDR0 = cChar;
	}
	else
	{
		/* Queue empty, nothing to send. */
		vInterrupt0Off();
	}
}


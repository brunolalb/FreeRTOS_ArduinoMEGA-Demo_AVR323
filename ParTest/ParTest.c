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

/*
 * Demo Blinky project
 *
 * ArduinoMEGA with FreeRTOS 9.0.0
 *
 * Compiler: WinAVR
 * Burner: AVR Dude (STK500v2)
 * IDE: Eclipse Mars.2
 *
 * Description::
 * main() creates one queue, and two tasks. It then starts the scheduler.
 *
 * The Queue Send Task:
 * The queue send task is implemented by the prvQueueSendTask() function in
 * this file.  prvQueueSendTask() sits in a loop that causes it to repeatedly
 * block for 200 (simulated as far as the scheduler is concerned, but in
 * reality much longer - see notes above) milliseconds, before sending the
 * value 100 to the queue that was created within main_blinky().  Once the
 * value is sent, the task loops back around to block for another 200
 * (simulated) milliseconds.
 *
 * The Queue Receive Task:
 * The queue receive task is implemented by the prvQueueReceiveTask() function
 * in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly
 * blocks on attempts to read data from the queue that was created within
 * main_blinky().  When data is received, the task checks the value of the
 * data, and if the value equals the expected 100, outputs a message.  The
 * 'block time' parameter passed to the queue receive function specifies that
 * the task should be held in the Blocked state indefinitely to wait for data
 * to be available on the queue.  The queue receive task will only leave the
 * Blocked state when the queue send task writes to the queue.  As the queue
 * send task writes to the queue every 200 (simulated - see notes above)
 * milliseconds, the queue receive task leaves the Blocked state every 200
 * milliseconds, and therefore outputs a message every 200 milliseconds.
 *
 * Initial version (2016-08-05): Bruno Landau Albrecht (brunolalb@gmail.com)
 *
 */

#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

#define partstLEDS_OUTPUT			( ( unsigned char ) 0b11000000 )
#define partstALL_OUTPUTS_OFF		( ( unsigned char ) 0b00111111 )
#define partstMAX_OUTPUT_LED		( ( unsigned char ) 7 )
#define partstMIN_OUTPUT_LED		( ( unsigned char ) 6 )

static volatile unsigned char ucCurrentOutputValue = partstALL_OUTPUTS_OFF; /*lint !e956 File scope parameters okay here. */

/*-----------------------------------------------------------*/

void vParTestInitialise( void )
{
	ucCurrentOutputValue = partstALL_OUTPUTS_OFF;

	/* Set port B direction to outputs.  Start with all output off. */
	DDRB = partstLEDS_OUTPUT;
	PORTB &= ucCurrentOutputValue;
}
/*-----------------------------------------------------------*/

void vParTestSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue )
{
unsigned char ucBit = ( unsigned char ) 1;

	if(( uxLED <= partstMAX_OUTPUT_LED ) && ( uxLED >= partstMIN_OUTPUT_LED ))
	{
		ucBit <<= uxLED;	

		vTaskSuspendAll();
		{
			if( xValue == pdFALSE )
			{
				ucBit ^= ( unsigned char ) 0xff;
				ucCurrentOutputValue &= ucBit;
				PORTB &= ucCurrentOutputValue;
			}
			else
			{
				ucCurrentOutputValue |= ucBit;
				PORTB |= ucCurrentOutputValue;
			}
		}
		xTaskResumeAll();
	}
}
/*-----------------------------------------------------------*/

void vParTestToggleLED( unsigned portBASE_TYPE uxLED )
{
unsigned char ucBit;

	if(( uxLED <= partstMAX_OUTPUT_LED ) && ( uxLED >= partstMIN_OUTPUT_LED ))
	{
		ucBit = ( ( unsigned char ) 1 ) << uxLED;

		vTaskSuspendAll();
		{

			PINB = ucBit;
			ucCurrentOutputValue = PORTB;
		}
		xTaskResumeAll();			
	}
}



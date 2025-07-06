/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */

#include "port.h"
#include "gd32f30x.h"
#include "gd32f30x_usart.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"
#include "bsp_usart2.h"
/* ----------------------- static functions ---------------------------------*/
static void prvvUARTTxReadyISR( void );
static void prvvUARTRxISR( void );

/* ----------------------- Start implementation -----------------------------*/

void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
	if(xRxEnable == TRUE)
	{
		usart_interrupt_enable(USART2, USART_INT_RBNE);
	}
	else
	{
		usart_interrupt_disable(USART2, USART_INT_RBNE);
	}
	if(xTxEnable == TRUE)
	{
		gpio_bit_set(GPIOB, GPIO_PIN_9);//rs485 output mode
		usart_interrupt_enable(USART2, USART_INT_TC);
	}
	else
	{
		usart_interrupt_disable(USART2, USART_INT_TC);
	}
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{		
		USART_NVIC();
		USART2_Config((uint16_t)ulBaudRate);
		usart_interrupt_enable(USART2, USART_INT_RBNE);
	
    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{
    /* Put a byte in the UARTs transmit buffer. This function is called
     * by the protocol stack if pxMBFrameCBTransmitterEmpty( ) has been
     * called. */
		usart_data_transmit(USART2,ucByte);
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
    /* Return the byte in the UARTs receive buffer. This function is called
     * by the protocol stack after pxMBFrameCBByteReceived( ) has been called.
     */
		*pucByte = usart_data_receive(USART2);
    return TRUE;
}

/* Create an interrupt handler for the transmit buffer empty interrupt
 * (or an equivalent) for your target processor. This function should then
 * call pxMBFrameCBTransmitterEmpty( ) which tells the protocol stack that
 * a new character can be sent. The protocol stack will then call 
 * xMBPortSerialPutByte( ) to send the character.
 */
static void prvvUARTTxReadyISR( void )
{
    pxMBFrameCBTransmitterEmpty(  );
}

/* Create an interrupt handler for the receive interrupt for your target
 * processor. This function should then call pxMBFrameCBByteReceived( ). The
 * protocol stack will then call xMBPortSerialGetByte( ) to retrieve the
 * character.
 */
static void prvvUARTRxISR( void )
{
    pxMBFrameCBByteReceived(  );
}


void USART2_IRQHandler(void)
{

    if(usart_flag_get(USART2, USART_FLAG_RBNE) == SET) 
    {
        prvvUARTRxISR();  
        usart_flag_clear(USART2, USART_FLAG_RBNE); 
    }


    if(usart_flag_get(USART2, USART_FLAG_ORERR) == SET) 
    {
        usart_flag_clear(USART2, USART_FLAG_ORERR); 
        prvvUARTRxISR();  
    }


    if(usart_flag_get(USART2, USART_FLAG_TC) == SET) 
    {
        prvvUARTTxReadyISR(); 

        usart_flag_clear(USART2, USART_FLAG_TC); 
				gpio_bit_reset(GPIOB, GPIO_PIN_9);// rs485 recieve mode
    }
}

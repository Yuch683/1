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

/* ----------------------- Platform includes --------------------------------*/
#include "port.h"
#include "gd32f30x.h"
#include "timer1.h"

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/
static void prvvTIMERExpiredISR( void );

/* ----------------------- Start implementation -----------------------------*/
BOOL
xMBPortTimersInit( USHORT usTim1Timerout50us )
{
		timer1_init(usTim1Timerout50us);
		timer1_nvic();
    return TRUE;
}


void
vMBPortTimersEnable(  )
{
		/* Enable the timer with the timeout passed to xMBPortTimersInit( ) */
		timer_interrupt_flag_clear(TIMER1,TIMER_INT_UP);
    timer_counter_value_config(TIMER1, 0); 
    /* TIMER1 counter enable */
    timer_enable(TIMER1);
}

void
vMBPortTimersDisable(  )
{
	  /* Disable any pending timers. */
    timer_counter_value_config(TIMER1, 0); 
    /* TIMER1 counter enable */
    timer_disable(TIMER1);
 
}

/* Create an ISR which is called whenever the timer has expired. This function
 * must then call pxMBPortCBTimerExpired( ) to notify the protocol stack that
 * the timer has expired.
 */
static void prvvTIMERExpiredISR( void )
{
    ( void )pxMBPortCBTimerExpired(  );
}


void TIMER1_IRQHandler(void)
{
    if(timer_interrupt_flag_get(TIMER1, TIMER_INT_UP) != RESET)
    {
        timer_interrupt_flag_clear(TIMER1, TIMER_INT_UP);
			
        prvvTIMERExpiredISR();
    }
}

#include "timer1.h"
#include "gd32f30x.h"
#include "gd32f30x_timer.h"
#include "gd32f30x_rcu.h"


void timer1_init(uint16_t period)
{  
		rcu_periph_clock_enable(RCU_TIMER1);
		timer_internal_clock_config(TIMER1);
		timer_deinit(TIMER1);
 
    timer_parameter_struct timer_initpara;		
	
    timer_initpara.prescaler         = 6000-1 ;    //50us
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER1,&timer_initpara);		
		
		timer_interrupt_enable(TIMER1, TIMER_INT_UP);
		timer_interrupt_flag_clear(TIMER1, TIMER_INT_UP);
    timer_auto_reload_shadow_enable(TIMER1);
    timer_enable(TIMER1);
}

void timer1_nvic(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(TIMER1_IRQn, 0, 2); 
}

#include "gd32f30x.h"
#include "gd32f30x_timer.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_usart.h"
#include <stdio.h>
#include "bsp_usart2.h"



void USART2_Config(uint16_t buad,uint8_t DataBits,uint16_t eParity)
{
	rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_USART2);
	rcu_periph_clock_enable(RCU_AF);
	
	gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10);
	gpio_init(GPIOB,GPIO_MODE_IN_FLOATING,GPIO_OSPEED_50MHZ,GPIO_PIN_11);
	gpio_init(GPIOB,GPIO_MODE_OUT_OD,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
	
	gpio_bit_reset(GPIOB,GPIO_PIN_9);
	
	//usart_init
	usart_deinit(USART2);
	usart_baudrate_set(USART2, buad);
	usart_word_length_set(USART2, DataBits);
  usart_stop_bit_set(USART2, USART_STB_1BIT);
  usart_parity_config(USART2, eParity);
  usart_hardware_flow_rts_config(USART2, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(USART2, USART_CTS_DISABLE);
	usart_receive_config(USART2, USART_RECEIVE_ENABLE);
	usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
	usart_enable(USART2);
}



void USART_NVIC(void)
{
			nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
			nvic_irq_enable(USART2_IRQn, 0, 0);
}



int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART2, (uint8_t)ch);

    while(usart_flag_get(USART2, USART_FLAG_TC) == RESET);

    return (ch);
}


int fgetc(FILE *f)
{
    while(usart_flag_get(USART2, USART_FLAG_RBNE) == RESET);

    return (int)usart_data_receive(USART2);
}

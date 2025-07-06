#include "gd32f30x.h"
#include "gd32f30x_timer.h"
#include "gd32f30x_rcu.h"
#include "gd32f30x_usart.h"
#include <stdio.h>
#include "bsp_usart2.h"

uint8_t SendBuff[SENDBUFF_SIZE];

void USART2_Config(uint16_t buad)
{
	rcu_periph_clock_enable(RCU_GPIOB);
  rcu_periph_clock_enable(RCU_USART2);
	
	gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_10|GPIO_PIN_11);
	gpio_init(GPIOB,GPIO_MODE_OUT_OD,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
	
	gpio_bit_reset(GPIOB,GPIO_PIN_9);
	
	//usart_init
	usart_deinit(USART2);
	usart_baudrate_set(USART2, buad);
	usart_word_length_set(USART2, USART_WL_8BIT);
  usart_stop_bit_set(USART2, USART_STB_1BIT);
  usart_parity_config(USART2, USART_PM_NONE);
  usart_hardware_flow_rts_config(USART2, USART_RTS_DISABLE);
  usart_hardware_flow_cts_config(USART2, USART_CTS_DISABLE);
	usart_receive_config(USART2, USART_RECEIVE_ENABLE);
	usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
	usart_enable(USART2);
}
	



void USART_NVIC(void)
{
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
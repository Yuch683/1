#ifndef __USART1_H
#define	__USART1_H

#include "gd32f30x.h"
#include <stdio.h>

#define USART2_DR_Base  0x40004804		// 0x40013800 + 0x04 = 0x40013804
#define SENDBUFF_SIZE   5000

void USART2_Config(uint16_t buad);
void USART2_DMA_Config(void);
void USART_NVIC(void);

#endif /* __USART1_H */

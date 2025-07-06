#ifndef __TIMER1_H
#define	__TIMER1_H

#include "gd32f30x.h"

void timer1_init(uint16_t period);
void timer1_nvic(void);

#endif /* __TIMER1_H */


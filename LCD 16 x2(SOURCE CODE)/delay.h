#include "stm32f4xx.h"

#ifndef __Delay_H
#define __Delay_h

void delay_ms(u32 ms);
void delay_us(u32 us);
void TimeTickDec();
void SystickInit(u32 TickTime);

#endif

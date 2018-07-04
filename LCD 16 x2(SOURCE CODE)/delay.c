#include "delay.h"
#include "stm32f4xx.h"
static volatile u32 delay_counter;

void TimeTickDec()
{
  if(delay_counter!=0)
    delay_counter--;
}
void delay_us(u32 us)
{
  delay_counter=us;
  while(delay_counter);
}
void delay_ms(u32 ms)
{
  while(ms--)delay_us(1000);
}


#include "tm_stm32f4_hd44780.h"
#include "tm_stm32f4_delay.h"
#include "stm32f4xx_gpio.h"

#include "delay.h"

#include <stdio.h>

#define lcd_bl_on()			GPIOE->BSRRL  = (1<<8)
#define lcd_bl_off()		GPIOE->BSRRH  = (1<<8)
#define lcd_xy				TM_HD44780_CursorSet
#define lcd_clr				TM_HD44780_Clear

#define TickTime 1000000

char buff[];

void SysTick_Handler(void)
{
  TimeTickDec();
}

void lcd_init()
{
	GPIO_InitTypeDef			GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10| GPIO_Pin_11| GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	TM_DELAY_Init();
	TM_HD44780_Init(16, 2);
	TM_HD44780_Clear();
	GPIO_ResetBits(GPIOE,GPIO_Pin_10);
	lcd_bl_on();

	//Show cursor
	//TM_HD44780_CursorOn();
}

int main(void)
{

	SystemInit();
	RCC_HSEConfig(RCC_HSE_ON);
	while(!RCC_WaitForHSEStartUp());

	while(SysTick_Config(SystemCoreClock/TickTime)!=0);

	lcd_init();

	lcd_bl_on();delay_ms(1000);
	lcd_bl_off();delay_ms(1000);
	lcd_bl_on();



    while(1)
    {
		lcd_xy(0,0);
		lcd_write("hello world");
		lcd_bl_on();delay_ms(1000);
		lcd_clr();
		lcd_xy(0,1);
		lcd_write("stm32f407");
    	lcd_bl_on();delay_ms(1000);
    	lcd_clr();
    }
}

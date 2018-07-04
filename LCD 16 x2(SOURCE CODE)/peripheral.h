#ifndef __peripheral_H
#define __peripheral_H

#include "stm32f4xx.h"
#include "tm_stm32f4_hd44780.h"

#define TIM7_prescaler	8400
#define TIM7_period		4000 //400ms

#define TIM6_prescaler	8400
#define TIM6_period		1000 //100ms

#define led0_on			GPIOD->BSRRH  = (1<<0)
#define led1_on			GPIOD->BSRRH  = (1<<1)
#define led2_on			GPIOD->BSRRH  = (1<<2)
#define led3_on			GPIOD->BSRRH  = (1<<3)
#define lcd_bl_off		GPIOE->BSRRH  = (1<<8)

#define led0_off		GPIOD->BSRRL  = (1<<0)
#define led1_off		GPIOD->BSRRL  = (1<<1)
#define led2_off		GPIOD->BSRRL  = (1<<2)
#define led3_off		GPIOD->BSRRL  = (1<<3)
#define lcd_bl_on		GPIOE->BSRRL  = (1<<8)

#define led0_togle		GPIOD->ODR ^= GPIO_Pin_0
#define led1_togle		GPIOD->ODR ^= GPIO_Pin_1
#define led2_togle		GPIOD->ODR ^= GPIO_Pin_2
#define led3_togle		GPIOD->ODR ^= GPIO_Pin_3
#define lcd_bl_togle	GPIOE->ODR ^= GPIO_Pin_8

#define button0 	!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)
#define button1 	!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_1)
#define button2 	!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_2)
#define button3 	!GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_3)

#define pwm_a		1
#define	set_MA_1	GPIO_SetBits(GPIOD,GPIO_Pin_8)
#define	set_MA_0	GPIO_SetBits(GPIOD,GPIO_Pin_9)
#define	reset_MA_1	GPIO_ResetBits(GPIOD,GPIO_Pin_8)
#define	reset_MA_0	GPIO_ResetBits(GPIOD,GPIO_Pin_9)

#define pwm_b		2
#define	set_MB_1	GPIO_SetBits(GPIOD,GPIO_Pin_10)
#define	set_MB_0	GPIO_SetBits(GPIOD,GPIO_Pin_11)
#define	reset_MB_1	GPIO_ResetBits(GPIOD,GPIO_Pin_10)
#define	reset_MB_0	GPIO_ResetBits(GPIOD,GPIO_Pin_11)

#define pwm_c		3
#define	set_MC_1	GPIO_SetBits(GPIOC,GPIO_Pin_8)
#define	set_MC_0	GPIO_SetBits(GPIOC,GPIO_Pin_9)
#define	reset_MC_1	GPIO_ResetBits(GPIOC,GPIO_Pin_8)
#define	reset_MC_0	GPIO_ResetBits(GPIOC,GPIO_Pin_9)

#define pwm_d		4
#define	set_MD_1	GPIO_SetBits(GPIOA,GPIO_Pin_8)
#define	set_MD_0	GPIO_SetBits(GPIOA,GPIO_Pin_10)
#define	reset_MD_1	GPIO_ResetBits(GPIOA,GPIO_Pin_8)
#define	reset_MD_0	GPIO_ResetBits(GPIOA,GPIO_Pin_10)

#define	MA_0		(GPIOD,9)
#define	MB_1		(GPIOD,10)
#define	MB_0		(GPIOD,11)
#define	MC_1		(GPIOC,8)
#define	MC_0		(GPIOC,9)
#define	MD_1		(GPIOA,8)
#define	MD_0		(GPIOA,10)

#define MA			1
#define MB			2
#define MC			3
#define MD			4

#define stop 		0
#define fwd 		1
#define bwd 		2

#define lcd_xy			TM_HD44780_CursorSet
#define lcd_clr			TM_HD44780_Clear

#define BT_Send(s) 		USART_SendString(USART2,s)
#define FTDI_Send(s) 	USART_SendString(USART3,s)

void USART2_init();
void USART3_init();
void USART6_init();
void USART_SendString(USART_TypeDef* USARTx,char *stringBuff);

void I2C2_init();
uint8_t I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);
void I2C_stop(I2C_TypeDef* I2Cx);
uint8_t check_i2c_device(I2C_TypeDef* I2Cx, uint8_t addr);

void HMC5883_init();
void HMC5883_read(uint8_t* data);

void TIM4_pwm_init();
void TIM4_pwm(uint8_t chan, uint16_t duty);
void m_control_init();

void TIM3_init();
void TIM6_init();
void TIM7_init();
void EXTI_12_15_init();
void button_init();
void led_init();
void lcd_init();

void adc1_init();
float read_bat();

void us_init();
float read_us(uint8_t);

#endif

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_i2c.h"
#include "misc.h"
#include "tm_stm32f4_delay.h"
#include "peripheral.h"

/* Timer prescaler and period configuration
 * TIMx period=TIMx_clock/(TIMx_prescaler*TIMx_period)
 * for TIM1,TIM8,TIM9,TIM10,and TIM11 -> TIM_clock=164MHz(max timer clock)
 * other TIMx -> TIM_clock=84MHz(max timer clock)
 *
 * note:
 * - TIMx_prescaler value must be between 1 to 65536 (16bit)
 * - TIMx_period value must be between 1 to 65536(16bit), except for TIM2 and TIM5 (32bit)
 *
 * example:
 * TIM7 period= 1 second(designated)
 * ->84000000/(8400*10000)= 1
 * ->1 second period
 *
 */

char debug_buff[];

GPIO_InitTypeDef			GPIO_InitStructure;
USART_InitTypeDef			USART_InitStructure;
NVIC_InitTypeDef			NVIC_InitStructure;
TIM_TimeBaseInitTypeDef		TIM_BaseInitStructure;
TIM_OCInitTypeDef 			TIM_OCInitStructure;
EXTI_InitTypeDef   			EXTI_InitStructure;
ADC_CommonInitTypeDef 		ADC_CommonInitStructure;
ADC_InitTypeDef       		ADC_InitStructure;
I2C_InitTypeDef 			I2C_InitStructure;

void USART2_init()
{
	/* USARTx configured as follow:
	        - BaudRate = 115200 baud
	        - Word Length = 8 Bits
	        - One Stop Bit
	        - No parity
	        - Hardware flow control disabled (RTS and CTS signals)
	        - Receive and transmit enabled
	  */

	  /* Enable GPIO clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	  /* Enable UART clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	  /* Connect PXx to USARTx_Rx*/
	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	  /* Configure USART Tx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_2;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Configure USART Rx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate 			= 115200;
	  USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits 			= USART_StopBits_1;
	  USART_InitStructure.USART_Parity 				= USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART2, &USART_InitStructure);

	  /* Enable USART */
	  USART_Cmd(USART2, ENABLE);
	  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	  NVIC_InitStructure.NVIC_IRQChannel 					= USART2_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 3;
	  NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
void USART3_init()
{
	/* USARTx configured as follow:
	        - BaudRate = 115200 baud
	        - Word Length = 8 Bits
	        - One Stop Bit
	        - No parity
	        - Hardware flow control disabled (RTS and CTS signals)
	        - Receive and transmit enabled
	  */

	  /* Enable GPIO clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  /* Enable UART clock */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	  /* Connect PXx to USARTx_Rx*/
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

	  /* Configure USART Tx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_10;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Configure USART Rx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate 			= 1382400;
	  USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits 			= USART_StopBits_1;
	  USART_InitStructure.USART_Parity 				= USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART3, &USART_InitStructure);

	  /* Enable USART */
	  USART_Cmd(USART3, ENABLE);
	  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	  NVIC_InitStructure.NVIC_IRQChannel 					= USART3_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 2;
	  NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
void USART6_init()
{
	/* USARTx configured as follow:
			- BaudRate = 115200 baud
			- Word Length = 8 Bits
			- One Stop Bit
			- No parity
			- Hardware flow control disabled (RTS and CTS signals)
			- Receive and transmit enabled
	  */

	  /* Enable GPIO clock */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	  /* Enable UART clock */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	  /* Connect PXx to USARTx_Tx*/
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	  /* Connect PXx to USARTx_Rx*/
	  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	  /* Configure USART Tx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType	= GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Configure USART Rx as alternate function  */
	  GPIO_InitStructure.GPIO_Pin 	= GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate 			= 115200;
	  USART_InitStructure.USART_WordLength 			= USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits 			= USART_StopBits_1;
	  USART_InitStructure.USART_Parity 				= USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode 				= USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART6, &USART_InitStructure);

	  /* Enable USART */
	  USART_Cmd(USART6, ENABLE);
	  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

	  NVIC_InitStructure.NVIC_IRQChannel 					= USART6_IRQn;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority 	= 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority 		= 1;
	  NVIC_InitStructure.NVIC_IRQChannelCmd 				= ENABLE;
	  NVIC_Init(&NVIC_InitStructure);
}
void USART_SendString(USART_TypeDef* USARTx,char *stringBuff)
{
    while(*stringBuff)// Loop while there are more characters to send.
    {
        USART_SendData(USARTx, *stringBuff++);
        while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET); //wait for next character to send
    }
}

void I2C2_init()
{
	/* enable APB1 peripheral clock for I2C1*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	/* enable the peripheral clock for the pins used by
	 PB10 for I2C SCL and PB11 for I2C1_SDL*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; // Pins 6(I2C1_SCL) and 9(I2C1_SDA)
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;// this defines the output type as open drain
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStructure);// now all the values are passed to the GPIO_Init()

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Disable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C2,&I2C_InitStructure);

	/* Enable the I2C peripheral */
	I2C_Cmd(I2C2, ENABLE);
}
/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
uint8_t I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	//debug here
	uint16_t timeout=2000;
	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			if (--timeout == 0x00)
			{
				return 0;
			}
		}
	}
	else if(direction == I2C_Direction_Receiver)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
			if (--timeout == 0x00)
			{
				return 0;
			}
		}
	}
	return 1;
}
/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}
/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}
/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx)
{
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}
uint8_t check_i2c_device(I2C_TypeDef* I2Cx, uint8_t addr)
{
	uint8_t connected=0;
	if(I2C_start(I2Cx,addr,I2C_Direction_Transmitter))
	{
		connected = 1;
	}

	/* STOP I2C */
	I2C_stop(I2Cx);

	/* Return status */
	return connected;
}

void HMC5883_init()
{
	sprintf(debug_buff,"device=%d\n",check_i2c_device(I2C2,0x3D));
	FTDI_Send(debug_buff);
	if(check_i2c_device(I2C2,0x3D))
	{
		I2C_start(I2C2,0x3D,I2C_Direction_Transmitter);
		I2C_write(I2C2,0x02);
		I2C_write(I2C2,0x00);
		I2C_stop(I2C2);
	}
}
void HMC5883_read(uint8_t* data)
{
	uint8_t i;
	if(check_i2c_device(I2C2,0x3D))
	{
		I2C_start(I2C2,0x3D,I2C_Direction_Transmitter);
		I2C_write(I2C2,0x03);
		I2C_stop(I2C2);
		I2C_start(I2C2,0x3D,I2C_Direction_Receiver);
		for(i=0;i<=4;++i)
		{
			data[i]=I2C_read_ack(I2C2);
		}
		data[5]=I2C_read_nack(I2C2);
	}
	else for(i=0;i<=5;++i) data[i]=0;
}

void EXTI_12_15_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PA0 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect EXTI Line to PB pins */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource12);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource13);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource14);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource15);

	/* Configure EXTI Lines */
	EXTI_InitStructure.EXTI_Line = EXTI_Line12 | EXTI_Line13| EXTI_Line14| EXTI_Line15;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM3_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_BaseInitStructure.TIM_Prescaler=TIM6_prescaler-1; //84MHz/8400=10kHz timer tick
	TIM_BaseInitStructure.TIM_Period=TIM6_period-1;
	TIM_BaseInitStructure.TIM_ClockDivision=0;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_BaseInitStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}
void TIM6_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_BaseInitStructure.TIM_Prescaler=TIM6_prescaler-1; //84MHz/8400=10kHz timer tick
	TIM_BaseInitStructure.TIM_Period=TIM6_period-1;
	TIM_BaseInitStructure.TIM_ClockDivision=0;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_BaseInitStructure);

	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}
void TIM7_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_BaseInitStructure.TIM_Prescaler=TIM7_prescaler-1; //84MHz/8400=10kHz timer tick
	TIM_BaseInitStructure.TIM_Period=TIM7_period-1; //period = 1s
	TIM_BaseInitStructure.TIM_ClockDivision=1;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_BaseInitStructure);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}
void TIM4_pwm_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);

	//Configuration
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	TIM_BaseInitStructure.TIM_Prescaler=84;
	TIM_BaseInitStructure.TIM_Period=1000;
	TIM_BaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_ClockDivision=0;
	TIM_TimeBaseInit(TIM4,&TIM_BaseInitStructure);

	TIM_Cmd(TIM4, ENABLE);

	//PWM channel 1
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;//CCR1_Val;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OC1Init(TIM4,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4,TIM_OCPreload_Enable);

	//PWM channel 2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	//PWM channel 3
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	//PWM channel 4
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);

}
void TIM4_pwm(uint8_t chn, uint16_t duty)
{
	if(duty>1000)duty=1000;
	if(chn==1)TIM4->CCR1 = duty;else
	if(chn==2)TIM4->CCR2 = duty;else
	if(chn==3)TIM4->CCR3 = duty;else
	if(chn==4)TIM4->CCR4 = duty;
}
void m_control_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_10| GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

void motor(uint8_t mtr, uint8_t dir, uint16_t pwm)
{
	if(mtr==MA)
	{
		if(dir==stop)
		{
			reset_MA_0;
			reset_MA_1;
			TIM4_pwm(pwm_a,0);
		}
		else if(dir==fwd)
		{
			set_MA_0;
			reset_MA_1;
			TIM4_pwm(pwm_a,pwm);
		}
		else if(dir==bwd)
		{
			reset_MA_0;
			set_MA_1;
			TIM4_pwm(pwm_a,pwm);
		}
	}

	else if(mtr==MB)
	{
		if(dir==stop)
		{
			reset_MB_0;
			reset_MB_1;
			TIM4_pwm(pwm_b,0);
		}
		else if(dir==fwd)
		{
			set_MB_0;
			reset_MB_1;
			TIM4_pwm(pwm_b,pwm);
		}
		else if(dir==bwd)
		{
			reset_MB_0;
			set_MB_1;
			TIM4_pwm(pwm_b,pwm);
		}
	}

	else if(mtr==MC)
	{
		if(dir==stop)
		{
			reset_MC_0;
			reset_MC_1;
			TIM4_pwm(pwm_c,0);
		}
		else if(dir==fwd)
		{
			set_MC_0;
			reset_MC_1;
			TIM4_pwm(pwm_c,pwm);
		}
		else if(dir==bwd)
		{
			reset_MC_0;
			set_MC_1;
			TIM4_pwm(pwm_c,pwm);
		}
	}

	else if(mtr==MD)
	{
		if(dir==stop)
		{
			reset_MD_0;
			reset_MD_1;
			TIM4_pwm(pwm_d,0);
		}
		else if(dir==fwd)
		{
			set_MD_0;
			reset_MD_1;
			TIM4_pwm(pwm_d,pwm);
		}
		else if(dir==bwd)
		{
			reset_MD_0;
			set_MD_1;
			TIM4_pwm(pwm_d,pwm);
		}
	}
}

void adc1_init()
{
	 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	 GPIO_Init(GPIOA, &GPIO_InitStructure);

	 ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	 ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	 ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	 ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	 ADC_CommonInit(&ADC_CommonInitStructure);

	 ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	 ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	 ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	 ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	 ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	 ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	 ADC_InitStructure.ADC_NbrOfConversion = 1;
	 ADC_Init(ADC1, &ADC_InitStructure);

	 ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_56Cycles);
	 ADC_EOCOnEachRegularChannelCmd(ADC1, ENABLE);

	 ADC_Cmd(ADC1, ENABLE);

}
float read_bat()
{
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
	return((float)(ADC_GetConversionValue(ADC1)*18.81)/4096);
}

void button_init()
{
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	  /* Configure PC0, PC1, PC2 and PC3 in push button-intput pushpull mode */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);
}
void led_init()
{
	/*!< At this stage the microcontroller clock setting is already configured,
	       this is done through SystemInit() function which is called from startup
	       file (startup_stm32f4xx.s) before to branch to application main.
	       To reconfigure the default setting of SystemInit() function, refer to
	        system_stm32f4xx.c file
	     */

	  /* GPIOD Periph clock enable */
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	  /* Configure PD0:red, PD1:yellow, PD2:green and PD3:blue in led-output pushpull mode */
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);

	  GPIO_Write(GPIOD,0x00ff);
}

void lcd_init()
{
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
	  lcd_bl_on;

	  //Show cursor
	  TM_HD44780_CursorOn();

	  //Write new text
	  //TM_HD44780_Puts(0, 0, "Bismillah");
}

void us_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

float read_us(uint8_t n)
{
	uint32_t time, timeout,us_port;
	uint16_t trigger_pin,echo_pin;

	switch (n)
	{
		case 0:
			us_port		=GPIOD;
			trigger_pin	=GPIO_Pin_4;
			echo_pin	=GPIO_Pin_5;
			break;
		case 1:
			us_port		=GPIOD;
			trigger_pin	=GPIO_Pin_6;
			echo_pin	=GPIO_Pin_7;
			break;
	}

	GPIO_ResetBits(us_port,trigger_pin);/* Trigger low */
	delay_us(2);/* Delay 2 us */
	GPIO_SetBits(us_port,trigger_pin);/* Trigger high for 10us */
	delay_us(10);/* Delay 10 us */
	GPIO_ResetBits(us_port,trigger_pin);/* Trigger low */

	timeout = 1000000;/* Give some timeout for response */

	while (!GPIO_ReadInputDataBit(us_port,echo_pin))
	{
		if (timeout-- == 0x00)	return -1;
	}

	/* Start time */
	time = 0;
	timeout = 25000;

	while (GPIO_ReadInputDataBit(us_port,echo_pin))/* Wait till signal is low */
	{
		time++;/* Increase time */
		if (timeout-- == 0x00)	return -1;

		delay_us(1);/* Delay 1us */
	}

	/* Convert us to cm */
	/* Return distance */
	return ((float)time*0.0176 - 0.25);
}

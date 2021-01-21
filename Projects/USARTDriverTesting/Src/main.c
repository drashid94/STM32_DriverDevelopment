#include "usart_driver.h"
#include "gpio_driver.h"
#include <stdio.h>
#include <string.h>

//USART3 Giving problems

char msg[19] = "WHAT DUH HEW?!?!\n\r";
USART_Config_t usart;

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

void GPIO_Init(void)
{
	GPIO_Config_t usart_pins;
	usart_pins.pGPIOx = GPIOC;
	usart_pins.GPIO_Mode = GPIO_MODE_ALTFUN;
	usart_pins.GPIO_Speed = GPIO_SPEED_FAST;
	usart_pins.GPIO_OutputType = GPIO_OT_PP;
	usart_pins.GPIO_PUPD = GPIO_PULLUP;
	usart_pins.GPIO_AltFunMode = AF8;

	GPIO_PeriClockControl(&usart_pins, ENABLE);

	usart_pins.GPIO_PinNumber = GPIO_PIN_10; // TX
	GPIO_Init(&usart_pins);

	usart_pins.GPIO_PinNumber = GPIO_PIN_11; // RX
	GPIO_Init(&usart_pins);
}

void USART_Init(void)
{
	//USART Init
	usart.pUSARTx = USART4;
	usart.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart.USART_Baud = USART_STD_BAUD_9600;
	usart.USART_Mode = USART_MODE_ONLY_TX;
	usart.USART_StopBits = USART_STOPBITS_1;
	usart.USART_ParityControl = USART_PARITY_DISABLE;
	usart.USART_WordLength = USART_WORDLEN_8BITS;
	USART_Init(&usart);
}

int main()
{
	GPIO_Init();
	USART_Init();
	USART_Peripheral_Control(&usart, ENABLE);
	while(1)
	{
		USART_Send_Data(&usart, (uint8_t*)msg, strlen(msg));
		delay();
	}
}

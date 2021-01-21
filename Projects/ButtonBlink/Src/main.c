//#include atom c intellisense"stm32f446xx.h"
#include "gpio_driver.h"


void delay(void)
{
	for(int i = 0; i < 50000 i++)
}


void GPIO_Init(void)
{
	GPIO_Config_t GPIOLED, GPIOBTN;

	//Button Init
	GPIOBTN.pGPIOx = GPIOB;
	GPIOBTN.GPIO_PinNumber = GPIO_PIN_5;
	GPIOBTN.GPIO_Mode = GPIO_MODE_IN;
	GPIOBTN.GPIO_Speed = GPIO_SPEED_FAST;
	GPIO_PeriClockControl(&GPIOBTN, ENABLE);
	
}

int main(void)
{
	GPIO_Config_t GPIOLED,GPIOBTN;
	
	GPIOLED.pGPIOx = GPIOA;
	
	GPIOLED.GPIO_PinNumber = GPIO_PIN_12;
	
	GPIOLED.GPIO_Mode = GPIO_MODE_OUT;
	GPIOLED.GPIO_OutputType = GPIO_OT_PP;
	GPIOLED.GPIO_PUPD = GPIO_NO_RESISTOR;
	
	GPIOLED.GPIO_Speed = GPIO_SPEED_FAST;
	GPIO_PeriClockControl(&GPIOLED, ENABLE);
	GPIO_Init(&GPIOBTN);
	GPIO_Init(&GPIOLED);
	
	while(1)
	{
		if(GPIO_ReadFromInputpin(&GPIOBTN))
		{
			GPIO_ToggleOutputPin(&GPIOLED,GPIO_PIN_12);
			delay();
		}
	}
	return 0;
}

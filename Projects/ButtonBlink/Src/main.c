//#include atom c intellisense"stm32f446xx.h"
#include "gpio_driver.h"


void delay(void)
{
	uint32_t i = 0;
	while(i < 500000)
	{
		i++;
	}
}
int main(void)
{
	GPIO_Config_t GPIOLED,GPIOBTN;
	GPIOBTN.pGPIOx = GPIOB;
	GPIOLED.pGPIOx = GPIOA;
	GPIOBTN.GPIO_PinNumber = GPIO_PIN_5;
	GPIOLED.GPIO_PinNumber = GPIO_PIN_12;
	GPIOBTN.GPIO_Mode = GPIO_MODE_IN;
	GPIOLED.GPIO_Mode = GPIO_MODE_OUT;
	GPIOLED.GPIO_OutputType = GPIO_OT_PP;
	GPIOLED.GPIO_PUPD = GPIO_NO_RESISTOR;
	GPIOBTN.GPIO_Speed = GPIO_SPEED_FAST;
	GPIOLED.GPIO_Speed = GPIO_SPEED_FAST;
	GPIO_PeriClockControl(&GPIOBTN, ENABLE);
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

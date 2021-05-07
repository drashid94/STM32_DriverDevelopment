#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "stm32f446xx.h"

//Possinble GPIO Modes 
#define GPIO_MODE_IN			0
#define GPIO_MODE_OUT			1
#define GPIO_MODE_ALTFUN		2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

//Possible GPIOA Output Types
#define GPIO_OT_PP			0
#define GPIO_OT_OD			1	

//Possible GPIO Output Speeds
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_SUPER		3

//GPIO Pull UP or Pull DOWN
#define GPIO_NO_RESISTOR		0
#define GPIO_PULLUP 			1
#define GPIO_PULLDOWN 			2

//GPIO Pin Numbers
#define GPIO_PIN_0 			0
#define GPIO_PIN_1 			1
#define GPIO_PIN_2 			2
#define GPIO_PIN_3 			3
#define GPIO_PIN_4 			4
#define GPIO_PIN_5 			5
#define GPIO_PIN_6 			6
#define GPIO_PIN_7 			7
#define GPIO_PIN_8 			8
#define GPIO_PIN_9 			9
#define GPIO_PIN_10 			10
#define GPIO_PIN_11 			11
#define GPIO_PIN_12 			12
#define GPIO_PIN_13 			13
#define GPIO_PIN_14 			14
#define GPIO_PIN_15 			15

//GPIO Alternate Function Modes
#define AF0	 			0
#define AF1	 			1
#define AF2	 			2
#define AF3	 			3
#define AF4	 			4
#define AF5 				5
#define AF6	 			6
#define AF7 				7
#define AF8	 			8
#define AF9 				9
#define AF10 				10
#define AF11 				11
#define AF12 				12
#define AF13 				13
#define AF14 				14
#define AF15 				15

//Other Useful Macros
#define GPIO_LOW			0
#define GPIO_HIGH			1

typedef struct
{
	GPIO_TypeDef *pGPIOx;
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_Mode;
	uint8_t GPIO_Speed;
	uint8_t GPIO_OutputType;
	uint8_t GPIO_PUPD;
	uint8_t GPIO_AltFunMode;
}GPIO_Config_t;

void GPIO_Init(GPIO_Config_t *pGPIOConfig);

void GPIO_DeInit(GPIO_Config_t *pGPIOConfig);

void GPIO_PeriClockControl(GPIO_Config_t *pGPIOConfig, uint8_t ENorDIS);

uint8_t GPIO_ReadFromInputpin(GPIO_Config_t *pGPIOConfig);

uint16_t GPIO_ReadFromInputPort(GPIO_Config_t *pGPIOConfig);

void GPIO_WriteToOutputPin(GPIO_Config_t *pGPIOConfig, uint8_t PinNumber, uint8_t Value);

void GPIO_WritetoOutputPort(GPIO_Config_t *pGPIOConfig, uint16_t Value);

void GPIO_ToggleOutputPin(GPIO_Config_t *pGPIOConfig, uint8_t PinNumber);

void GPIO_IRQConfig(uint8_t IRQNumber,  uint8_t ENorDIS);

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void GPIO_IRQHandling(uint8_t PinNumber);

#endif

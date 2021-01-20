#ifndef STM32F446XX_GPIO_DRV_H
#define STM32F446XX_GPIO_DRV_H

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
#define GPIO_OT_PP					0
#define GPIO_OT_OD					1	

//Possible GPIO Output Speeds
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_SUPER		3

//GPIO Pull UP or Pull DOWN
#define GPIO_NO_RESISTOR		0
#define GPIO_PULLUP 				1
#define GPIO_PULLDOWN 			2

//GPIO Pin Numbers
#define GPIO_PIN_0 					0
#define GPIO_PIN_1 					1
#define GPIO_PIN_2 					2
#define GPIO_PIN_3 					3
#define GPIO_PIN_4 					4
#define GPIO_PIN_5 					5
#define GPIO_PIN_6 					6
#define GPIO_PIN_7 					7
#define GPIO_PIN_8 					8
#define GPIO_PIN_9 					9
#define GPIO_PIN_10 				10
#define GPIO_PIN_11 				11
#define GPIO_PIN_12 				12
#define GPIO_PIN_13 				13
#define GPIO_PIN_14 				14
#define GPIO_PIN_15 				15

//GPIO Alternate Function Modes
#define AF0	 								0
#define AF1	 								1
#define AF2	 								2
#define AF3	 								3
#define AF4	 								4
#define AF5 								5
#define AF6	 								6
#define AF7 								7
#define AF8	 								8
#define AF9 								9
#define AF10 								10
#define AF11 								11
#define AF12 								12
#define AF13 								13
#define AF14 								14
#define AF15 								15

//Other Useful Macros
#define GPIO_LOW				0
#define GPIO_HIGH				1

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



/**************************************************
* @fn 					- GPIO_Init
*	@brief				- Initializes desired GPIO pin
*	@param[in]		- Conifg structure for pin
*	@param[in]		- 
* @param[in]		-
* @return 			- none
*	@Note					- none
*/

//Clear all bits before setting them
void GPIO_Init(GPIO_Config_t *pGPIOConfig)
{
	uint32_t temp = 0;
	if(pGPIOConfig->GPIO_Mode <= GPIO_MODE_ANALOG) // If non-interrupt mode
	{
		//Configure mode of GPIO pin
		temp = (pGPIOConfig->GPIO_Mode << (2 * pGPIOConfig->GPIO_PinNumber));
		pGPIOConfig->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOConfig->GPIO_PinNumber));
		pGPIOConfig->pGPIOx->MODER |= temp;
	}else
	{
		pGPIOConfig->pGPIOx->MODER &= ~( 1 << (2 * pGPIOConfig->GPIO_PinNumber));
		// Must set the Falling Edge or Rising edge detection based on specified GPIO MODE
		if(pGPIOConfig->GPIO_Mode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOConfig->GPIO_PinNumber);  
			EXTI->RTSR &= ~(1 << pGPIOConfig->GPIO_PinNumber); 
		}else if(pGPIOConfig->GPIO_Mode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIOConfig->GPIO_PinNumber);
		//	EXTI->FTSR &= ~(1 << pGPIOConfig->GPIO_PinNumber); 		
		}else if(pGPIOConfig->GPIO_Mode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1 << pGPIOConfig->GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOConfig->GPIO_PinNumber);
		}
		// Muse set the GPIO Port selection in SYSCFG: EXTICR
		uint8_t temp1 = 0;
		uint8_t temp2 = 0;
		temp1 = (pGPIOConfig->GPIO_PinNumber/4);
		temp2 = (pGPIOConfig->GPIO_PinNumber%4);
		SYSCFG_PCLK_EN;
		SYSCFG->EXTICR[temp1] |= ((GPIO_GET_PORT_CODE(pGPIOConfig->pGPIOx) << temp2*4));
		//Must enable the interrupt request by setting the corresponding IMR register
		EXTI->IMR |= (1 << pGPIOConfig->GPIO_PinNumber);
	}
	//Configure GPIO Speed
	temp = 0;
	temp = (pGPIOConfig->GPIO_Speed << (2 * pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->OSPEEDR |= temp;
	
	//Configure GPIO Output Type
	temp = 0;
	temp = (pGPIOConfig->GPIO_OutputType << (pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->OTYPER &= ~(0x3 << (pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->OTYPER |= temp;
	
	//Configure GPIO Internal Resistor
	temp = 0;
	temp = (pGPIOConfig->GPIO_PUPD << (2 * pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOConfig->GPIO_PinNumber));
	pGPIOConfig->pGPIOx->PUPDR |= temp;
	
	//Configure Alternate Function
	if(pGPIOConfig->GPIO_Mode == GPIO_MODE_ALTFUN)
	{
		temp = 0;
		uint8_t temp1 = pGPIOConfig->GPIO_PinNumber % 8;
		if(pGPIOConfig->GPIO_PinNumber < AF8) // Uses AFL register
		{
			temp = pGPIOConfig->GPIO_AltFunMode << (4 * pGPIOConfig->GPIO_PinNumber);
			pGPIOConfig->pGPIOx->AFRL &= ~(0xF << (4 * pGPIOConfig->GPIO_PinNumber));
			pGPIOConfig->pGPIOx->AFRL |= temp;		
		}else
		{
			temp = (pGPIOConfig->GPIO_AltFunMode << (4 * temp1));
			pGPIOConfig->pGPIOx->AFRH &= ~(0xF << (4 * temp1));
			pGPIOConfig->pGPIOx->AFRH |= temp;
		}
	}
	
}
/**************************************************
* @fn 					- GPIO_Deinit
*	@brief				- Clears the registers for a GPIO port
*	@param[in]		- Base address for the GPIO peripheral
*	@param[in]		- 
* @return 			- none
*	@Note					- none
*/
void GPIO_DeInit(GPIO_Config_t *pGPIOConfig)
{
		if (pGPIOConfig->pGPIOx == GPIOA)
		{
			GPIOA_REG_RESET();
		}else if (pGPIOConfig->pGPIOx == GPIOB)
		{
			GPIOB_REG_RESET();
		}else if (pGPIOConfig->pGPIOx == GPIOC)
		{
			GPIOC_REG_RESET();
		}else if (pGPIOConfig->pGPIOx == GPIOD)
		{
			GPIOD_REG_RESET();
		}else if (pGPIOConfig->pGPIOx == GPIOE)
		{
			GPIOE_REG_RESET();
		}else if(pGPIOConfig->pGPIOx == GPIOF)
		{
			GPIOF_REG_RESET();
		}else if (pGPIOConfig->pGPIOx == GPIOH)
		{
			GPIOH_REG_RESET();
		}
}

/**************************************************
* @fn 					- GPIO_PeriClockControl
*	@brief				- Enables or Disables peripheral clock control for the
*									given GPIO Port
*	@param[in]		- Base address for the GPIO peripheral
*	@param[in]		- Enable or Disable command
* @return 			- none
*	@Note					- none
****************************************************/
void GPIO_PeriClockControl(GPIO_Config_t *pGPIOConfig, uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		if (pGPIOConfig->pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN;
		}else if (pGPIOConfig->pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN;
		}else if (pGPIOConfig->pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN;
		}else if (pGPIOConfig->pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN;
		}else if (pGPIOConfig->pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN;
		}else if(pGPIOConfig->pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN;
		}else if (pGPIOConfig->pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN;
		}
	}else
	{
		if (pGPIOConfig->pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS;
		}else if (pGPIOConfig->pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS;
		}else if (pGPIOConfig->pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS;
		}else if (pGPIOConfig->pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS;
		}else if (pGPIOConfig->pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS;
		}else if(pGPIOConfig->pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS;
		}else if (pGPIOConfig->pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS;
		}
	}
}

/**************************************************
* @fn 					- GPIO_ReadFromInputPin
*	@brief				- Reads from a GPIO pin
*	@param[in]		- Base address of GPIO port
*	@param[in]		- Pin Number
* @param[in]		-	
* @return 			- uint8_t
*	@Note					- none
*/
uint8_t GPIO_ReadFromInputpin(GPIO_Config_t *pGPIOConfig)
{
	uint8_t value = 0;
	value =  ((pGPIOConfig->pGPIOx->IDR >> pGPIOConfig->GPIO_PinNumber) & (0x00000001));
	return value;
}

/**************************************************
* @fn 					- GPIO_ReadFromInputPort
*	@brief				- Reads from a GPIO Port
*	@param[in]		- Base address of GPIO port
*	@param[in]		- 
* @param[in]		-
* @return 			- uint16_t
*	@Note					- none
*/
uint16_t GPIO_ReadFromInputPort(GPIO_Config_t *pGPIOConfig)
{
	uint16_t value = 0;
	value = (pGPIOConfig->pGPIOx->IDR);
	return value;
}

/**************************************************
* @fn 					- GPIO_WriteToOutputPin
*	@brief				- Writes to an output GPIO pin
*	@param[in]		- Base address for GPIO port
*	@param[in]		- Pin Number
* @param[in]		- Desired output value
* @return 			- none
*	@Note					- none
*/
void GPIO_WriteToOutputPin(GPIO_Config_t *pGPIOConfig, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_HIGH)
	{
		pGPIOConfig->pGPIOx->ODR |= (1 << PinNumber);
	}else if(Value == GPIO_LOW)
	{
		pGPIOConfig->pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
/**************************************************
* @fn 					- GPIO_WriteToOutputPort
*	@brief				- Writes to an output GPIO port
*	@param[in]		- Base address for GPIO port
*	@param[in]		- Desired output value
* @param[in]		- 
* @return 			- none
*	@Note					- none
*/
void GPIO_WritetoOutputPort(GPIO_Config_t *pGPIOConfig, uint16_t Value)
{
		pGPIOConfig->pGPIOx->ODR = Value;
}


/**************************************************
* @fn 					- GPIO_ToggleOutputPin
*	@brief				- Toggles an output GPIO pin
*	@param[in]		- Base address for GPIO port
*	@param[in]		- Pin Number
* @param[in]		- 
* @return 			- none
*	@Note					- none
*/
void GPIO_ToggleOutputPin(GPIO_Config_t *pGPIOConfig, uint8_t PinNumber)
{
	pGPIOConfig->pGPIOx->ODR ^=  (1 << PinNumber);
}

/**************************************************
* @fn 					- GPIO_IRQConfig
*	@brief				- Configures a perihperal as Interrupt
*	@param[in]		- IRQ Number or Position on NVIC
*	@param[in]		- Desired Priority
* @param[in]		- Enable or Disable (True for Enable)
* @return 			- none
*	@Note					- none
*/
void GPIO_IRQConfig(uint8_t IRQNumber,  uint8_t ENorDIS)
{
	if(ENorDIS == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Set ISER0 Register (Must dereference the address and set value)
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber >31 && IRQNumber < 64)
		{
			//Set ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//SET ISER2 Register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}		
	}else if(ENorDIS == DISABLE)
	{
				if(IRQNumber <= 31)
		{
			//Set ICER0 Register (Must dereference the address and set value)
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber >31 && IRQNumber < 64)
		{
			//Set ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//SET ICER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}	
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp = 0;
	uint8_t temp1 = 0;
	temp = IRQNumber / 4;
	temp1 = IRQNumber % 4;
	uint8_t shiftvalue = temp1 * 8 + (8 - NO_OF_PRIORITY_BITS_USED);
	*(NVIC_PR_BASE + (temp)) |= (IRQPriority << shiftvalue);
}
/**************************************************
* @fn 					- GPIO_IRQHandling
*	@brief				- Clears pending register bit to begin interrupt handling
*	@param[in]		- Pin Number
*	@param[in]		- 
* @param[in]		- 
* @return 			- none
*	@Note					- none
*/
void GPIO_IRQHandling(uint8_t PinNumber)
{

	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber); // To clear pending register you must write one..
	}
}
#endif

#include "USART_Driver.h"

void USART_PeriClockControl(USART_Config_t * pUSARt, uint8_t EN)
{
	if(EN == ENABLE)
	{
		if(pUSARt->pUSART == USART1)
		{
			USART1_PCLK_EN;
		}
		else if(pUSARt->pUSART == USART2)
		{
			USART2_PCLK_EN;
		}
		else if(pUSARt->pUSART == USART3)
		{
			USART3_PCLK_EN;
		}
		else if(pUSARt->pUSART == USART4)
		{
			USART4_PCLK_EN;
		}
		else if(pUSARt->pUSART == USART5)
		{
			USART5_PCLK_EN;
		}
		else if(pUSARt->pUSART == USART6)
		{
			USART6_PCLK_EN;
		}
	}
	else
	{
		if(pUSARt->pUSART == USART1)
		{
			USART1_PCLK_DIS;
		}
		else if(pUSARt->pUSART == USART2)
		{
			USART2_PCLK_DIS;
		}
		else if(pUSARt->pUSART == USART3)
		{
			USART3_PCLK_DIS;
		}
		else if(pUSARt->pUSART == USART4)
		{
			USART4_PCLK_DIS;
		}
		else if(pUSARt->pUSART == USART5)
		{
			USART5_PCLK_DIS;
		}
		else if(pUSARt->pUSART == USART6)
		{
			USART6_PCLK_DIS;
		}
	}
}

void USART_Peripheral_Control(USART_Config_t * pUSARt, uint8_t EN)
{
	if(EN == ENABLE)
	{
		
	}
}

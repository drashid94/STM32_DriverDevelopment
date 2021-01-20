#include "stm32f446xx_spi_drv.h"

//SPI Event Callbacks
#define SPI_CALLBACK_TX_CMPLT	1
#define SPI_CALLBACK_RX_CMPLT	2
#define SPI_CALLBACK_OVR_ERR	3


//HELPER FUNC's
void SPI_INTERRUPT_TX_HANDLE(SPI_Config *pSPIx)
{
		if(pSPIx->SPI_Data_Format == SPI_DF_16)
		{
			uint16_t Data = *((uint16_t*)pSPIx->TxBuffer);
			pSPIx->pSPIx->DR = Data;
			Data = pSPIx->pSPIx->DR;
			pSPIx->TxLen--;
			pSPIx->TxLen--;
			pSPIx->TxBuffer++;
			pSPIx->TxBuffer++;
		}else
		{
			uint8_t Data = *(pSPIx->TxBuffer);
			pSPIx->pSPIx->DR = Data;
			Data = pSPIx->pSPIx->DR;
			pSPIx->TxLen--;
			pSPIx->TxBuffer++;
		}
			if(!pSPIx->TxLen)
			{
				pSPIx->pSPIx->CR2 &= ~(1 << 7); // Disable interrupts after handled
				pSPIx->TxBuffer = NULL;
				pSPIx->TxState = SPI_STATUS_READY;
				
			}
}
void SPI_INTERRUPT_RX_HANDLE(SPI_Config *pSPIx)
{
		if(pSPIx->SPI_Data_Format == SPI_DF_16)
		{
			*((uint16_t*)pSPIx->RxBuffer) = pSPIx->pSPIx->DR;
			pSPIx->RxLen--;
			pSPIx->RxLen--;
			pSPIx->RxBuffer++;
			pSPIx->RxBuffer++;
		}else
		{
			*(pSPIx->RxBuffer) = pSPIx->pSPIx->DR;			
			pSPIx->RxLen--;
			pSPIx->RxBuffer++;
		}
		if(!pSPIx->TxLen)
		{
			pSPIx->pSPIx->CR2 &= ~(1 << 6); // Disable interrupts after handled
			pSPIx->RxBuffer = NULL;
			pSPIx->RxState = SPI_STATUS_READY;		
		}
}
void SPI_INTERRUPT_OVR_HANDLE(SPI_Config *pSPIx)
{
	uint8_t temp = 0;
	temp = pSPIx->pSPIx->DR;
	temp = (pSPIx->pSPIx->SR & (1 << 6));
	(void)temp;
}
void SPI_PeriClockControl(SPI_Config *pSPIx, uint8_t ENorDIS)	
{
	if(ENorDIS == ENABLE)
	{
		if (pSPIx->pSPIx == SPI1)
		{
			SPI1_PCLK_EN;
		}else if (pSPIx->pSPIx == SPI2)
		{
			SPI2_PCLK_EN;
		}else if (pSPIx->pSPIx == SPI3)
		{
			SPI3_PCLK_EN;
		}
	}else
	{
		if (pSPIx->pSPIx == SPI1)
		{
			SPI1_PCLK_DIS;
		}else if (pSPIx->pSPIx == SPI2)
		{
			SPI2_PCLK_DIS;
		}else if (pSPIx->pSPIx == SPI3)
		{
			SPI3_PCLK_DIS;
		}
	}
}
void SPI_Periph_Control(SPI_Config *pSPIx, uint8_t ENorDIS)
{
	pSPIx->pSPIx->CR1 |= (ENorDIS << 6); 
}

void SPI_Init(SPI_Config *pSPIx)
{
	if(pSPIx->SPI_Mode == SPI_MODE_MASTER)
	{
		pSPIx->pSPIx->CR1 &= ~(1 << 2); // Clear bit
		pSPIx->pSPIx->CR1 |= (SPI_MODE_MASTER << 2); // Set MTSR bit
	}else
	{
		pSPIx->pSPIx->CR1 &= ~(1 << 2); // Clear bit for Slave Mode
	}
	if(pSPIx->SPI_Configuration == SPI_CONFIGURATION_FD)
	{
		pSPIx->pSPIx->CR1 &= ~(1 << 15);
	}else if(pSPIx->SPI_Configuration == SPI_CONFIGURATION_HD)
	{
		pSPIx->pSPIx->CR1 |= (1 << 15);
	}else if(pSPIx->SPI_Configuration == SPI_CONFIGURATION_SP_RX)
	{
		pSPIx->pSPIx->CR1 &= ~(1 << 15);
		pSPIx->pSPIx->CR1 |= (1 << 10);
	}
	if(pSPIx->SPI_Data_Format == SPI_DF_8)
	{
		pSPIx->pSPIx->CR1 &= ~(1 << 11);
	}else if(pSPIx->SPI_Data_Format == SPI_DF_16)
	{
		pSPIx->pSPIx->CR1 |= (1 << 11);
	}
	if(pSPIx->SPI_CPHA == SPI_CPHA_FIRST_EDGE)
	{
		pSPIx->pSPIx->CR1 &= ~(1 << 0);
	}else if(pSPIx->SPI_CPHA == SPI_CPHA_SECOND_EDGE)
	{
		pSPIx->pSPIx->CR1 |= (1 << 0);
	}
	pSPIx->pSPIx->CR1 |= (pSPIx->SPI_Speed << 3); // Set speed
	pSPIx->pSPIx->CR1 |= (pSPIx->SPI_CPOL << 1); // Set clock polarity
	pSPIx->pSPIx->CR1 |= (pSPIx->SPI_SSM << 9); // Set Software Slave Select
}

void SPI_LSB_First(SPI_Config *pSPIx, uint8_t ENorDIS)
{
	if(ENorDIS == 1 || ENorDIS == 0)
	{
		pSPIx->pSPIx->CR1 |= (ENorDIS << 7);
	}
}

void SPI_DEInit(SPI_Config *pSPIx);

void SPI_Send(SPI_Config *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(!(pSPIx->pSPIx->SR & (1 << 1)));
		if(pSPIx->SPI_Data_Format == SPI_DF_16)
		{
			uint16_t Data = *((uint16_t*)pTxBuffer);
			pSPIx->pSPIx->DR = Data;
			//Data = pSPIx->pSPIx->DR;
			Len--;
			Len--;
			pTxBuffer++;
			pTxBuffer++;
		}else
		{
			uint8_t Data = *pTxBuffer;
			pSPIx->pSPIx->DR = Data;
			Data = pSPIx->pSPIx->DR;
			Len--;
			pTxBuffer++;
		}
	}
}
uint8_t SPI_Get_Flag_Status(SPI_Config *pSPIx, uint8_t Bit_Number)
{
	return (pSPIx->pSPIx->SR & (1 << Bit_Number));
}
void SPI_SSI_CONFIG(SPI_Config *pSPIx, uint8_t ENorDIS)
{
	pSPIx->pSPIx->CR1 |= (ENorDIS << 8);
}
void SPI_SSOE_CONFIG(SPI_Config *pSPIx, uint8_t ENorDIS)
{
	pSPIx->pSPIx->CR2 |= (ENorDIS << 2);
}
void SPI_Enable(SPI_Config *pSPIx)
{
	pSPIx->pSPIx->CR1 |= (1 << 6);
}
void SPI_Disable(SPI_Config *pSPIx)
{
	pSPIx->pSPIx->CR1 &= ~(1 << 6);
}
void SPI_Receive(SPI_Config *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
		while(Len > 0)
	{
		while(!(pSPIx->pSPIx->SR & (1 << 0)));
		if(pSPIx->SPI_Data_Format == SPI_DF_16)
		{
			uint16_t Data = pSPIx->pSPIx->DR;
			*((uint16_t*)pRxBuffer) = Data;
			Len--;
			Len--;
			pRxBuffer++;
			pRxBuffer++;
		}else
		{
			uint8_t Data = 0;
			Data = pSPIx->pSPIx->DR;
			*pRxBuffer = Data;
			Len--;
			pRxBuffer++;
		}
	}
}

void SPI_ITReceive(SPI_Config *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIx->RxState;
	if(state != SPI_STATUS_RxBusy)
	{
		pSPIx->RxBuffer = pRxBuffer;
		pSPIx->RxLen = Len;
		pSPIx->RxState = SPI_STATUS_RxBusy;
		pSPIx->pSPIx->CR2 |= (1 << 6);
	}
}

void SPI_ITSend(SPI_Config *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIx->TxState;
	if(state != SPI_STATUS_TxBusy)
	{
		pSPIx->TxBuffer = pTxBuffer; // Save Tx Buffer Location
		pSPIx->TxLen = Len;					 // Save Tx Len
		pSPIx->TxState = SPI_STATUS_TxBusy;	// Set the status for SPI as busy
		
		pSPIx->pSPIx->CR2 |= (1 << 7); // Enable Tx busy interrupt. Generates interrupt when TXE is set (aka empty)
	}
}

void SPI_IRQConfig(SPI_Config *pSPIx, uint8_t IRQNumber, uint8_t ENorDIS)
{
		if(ENorDIS == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Set ISER0 Register (Must dereference the address and set value)
			*NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber >31 & IRQNumber < 64)
		{
			//Set ISER1 Register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 & IRQNumber < 96)
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
		}else if(IRQNumber >31 & IRQNumber < 64)
		{
			//Set ICER1 Register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber >= 64 & IRQNumber < 96)
		{
			//SET ICER2 Register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}	
	}
}

void SPI_IRQPriorityConfig(SPI_Config *pSPIx, uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp = 0;
	uint8_t temp1 = 0;
	temp = IRQNumber / 4;
	temp1 = IRQNumber % 4;
	uint8_t shiftvalue = temp1 * 8 + (8 - NO_OF_PRIORITY_BITS_USED);
	*(NVIC_PR_BASE + (temp)) |= (IRQPriority << shiftvalue);
}

void SPI_IRQHandler(SPI_Config *pSPIx)
{
	//Find out what caused the interrupt
	uint8_t temp1 = (pSPIx->pSPIx->SR & (1 << 1));
	uint8_t temp2 = (pSPIx->pSPIx->CR2 & (1 << 7));
	if(temp1 && temp2)
	{
		SPI_INTERRUPT_TX_HANDLE(pSPIx);
	}
	temp1 = (pSPIx->pSPIx->SR & (1 << 0));
	temp2 = (pSPIx->pSPIx->CR2 & (1 << 6));
	if(temp1 && temp2)
	{
		SPI_INTERRUPT_RX_HANDLE(pSPIx);
	}
	temp1 = (pSPIx->pSPIx->SR & (1 << 6));
	temp2 = (pSPIx->pSPIx->CR2 & (1 << 5));
	if(temp1 && temp2)
	{
		SPI_INTERRUPT_OVR_HANDLE(pSPIx);
	}
}

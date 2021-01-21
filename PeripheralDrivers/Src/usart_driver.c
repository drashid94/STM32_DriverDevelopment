#include "usart_driver.h"

//Enable clock of USART Peripheral
void USART_PeriClockControl(USART_Config_t * pUSARt, uint8_t EN)
{
	if(EN == ENABLE)
	{
		if(pUSARt->pUSARTx == USART1)
		{
			USART1_PCLK_EN;
		}
		else if(pUSARt->pUSARTx == USART2)
		{
			USART2_PCLK_EN;
		}
		else if(pUSARt->pUSARTx == USART3)
		{
			USART3_PCLK_EN;
		}
		else if(pUSARt->pUSARTx == USART4)
		{
			USART4_PCLK_EN;
		}
		else if(pUSARt->pUSARTx == USART5)
		{
			USART5_PCLK_EN;
		}
		else if(pUSARt->pUSARTx == USART6)
		{
			USART6_PCLK_EN;
		}
	}
	else
	{
		if(pUSARt->pUSARTx == USART1)
		{
			USART1_PCLK_DIS;
		}
		else if(pUSARt->pUSARTx == USART2)
		{
			USART2_PCLK_DIS;
		}
		else if(pUSARt->pUSARTx == USART3)
		{
			USART3_PCLK_DIS;
		}
		else if(pUSARt->pUSARTx == USART4)
		{
			USART4_PCLK_DIS;
		}
		else if(pUSARt->pUSARTx == USART5)
		{
			USART5_PCLK_DIS;
		}
		else if(pUSARt->pUSARTx == USART6)
		{
			USART6_PCLK_DIS;
		}
	}
}

void USART_SetBaud(USART_Config_t * pUSART)
{

	//Variable to hold the APB clock
	uint32_t PCLK;

	uint32_t usartdiv;

	uint32_t baud = pUSART->USART_Baud;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

  uint32_t tempreg = 0;

  //Get the value of APB bus clock in to the variable PCLK
  if(pUSART->pUSARTx == USART1 || pUSART->pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLK = RCC_Get_APBClck2_Freq();
  }else
  {
	   PCLK = RCC_Get_APBClck1_Freq();
  }

  //Check for OVER8 configuration bit
  if(pUSART->pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 , over sampling by 8
	   usartdiv = ((25 * PCLK) / (2 *baud));
  }else
  {
	   //over sampling by 16
	   usartdiv = ((25 * PCLK) / (4 * baud));
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in appropriate bit position . refer USART_BRR
  tempreg |= M_part << 4;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSART->pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 , over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100)& ((uint8_t)0x07);

   }else
   {
	   //over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fractional part in appropriate bit position . refer USART_BRR
  tempreg |= F_part;

  //copy the value of tempreg in to BRR register
  pUSART->pUSARTx->BRR = tempreg;
}

void USART_Peripheral_Control(USART_Config_t * pUSART, uint8_t EN)
{
	if(EN == ENABLE)
	{
		pUSART->pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		pUSART->pUSARTx->CR1 &= ~(1 << USART_CR1_UE);		
	}
}

void USART_Init(USART_Config_t * pUSART)
{

	//Temporary variable
	uint32_t tempreg = 0;

	//Implement the code to enable the Clock for given USART peripheral
	USART_PeriClockControl(pUSART, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item

	if (pUSART->USART_Mode == USART_MODE_ONLY_TX)
	{
		tempreg |= (1 << USART_CR1_TE);
	}
	else if(pUSART->USART_Mode == USART_MODE_ONLY_RX)
	{
		tempreg |= (1 << USART_CR1_RE);
	}
	else // This mush mean that the device is in both TX and RX mode
	{
		tempreg |= (1 << USART_CR1_RE);
		tempreg |= (1 << USART_CR1_TE);
	}
	//Load the tempreg with the correct wordlength
	tempreg |= pUSART->USART_WordLength << USART_CR1_M ;


    //Parity Control
	if ( pUSART->USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enable the parity control 
		tempreg |= ( 1 << USART_CR1_PCE);

	}else if (pUSART->USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control 
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity 
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register 
	pUSART->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission 
	tempreg |= pUSART->USART_StopBits << USART_CR2_STOP;

	//Program the CR2 register 
	pUSART->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;
	
	//Configuration of USART hardware flow control 
	if ( pUSART->USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control 
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSART->USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control 
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSART->USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control 
		tempreg |= (1 << USART_CR3_RTSE);
		tempreg |= (1 << USART_CR3_CTSE);
	}

	pUSART->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(baud register)******************************************/
	USART_SetBaud(pUSART);
}

void USART_Send_Data(USART_Config_t * pUSART, uint8_t * tx_buff, uint32_t Len)
{
	uint16_t * pdata;
   //Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until TXE flag is set in the SR
		while(! USART_Get_Flag_Status(pUSART, USART_FLAG_TXE));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSART->USART_WordLength == USART_WORDLEN_9BITS)
		{
			pdata = (uint16_t*) tx_buff;
			pUSART->pUSARTx->DR = (*pdata & (uint16_t)0x01FF); // Mask bits other than the first 9 bits
			
			//check for USART_ParityControl
			if(pUSART->USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment tx_buff twice 
				tx_buff++;
				tx_buff++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				tx_buff++;
			}
		}
		else
		{
			//This is 8bit data transfer 
			pUSART->pUSARTx->DR = (*tx_buff);
			
			//Implement the code to increment the buffer address
			tx_buff++;
		}
	}

	//Implement the code to wait till TC flag is set in the SR
	while( ! USART_Get_Flag_Status(pUSART, USART_FLAG_TC));
}

void USART_Recieve_Data(USART_Config_t * pUSART, uint8_t * rx_buff, uint32_t Len)
{
	//Loop over until "Len" number of bytes are transferred
	for(uint32_t i = 0 ; i < Len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! USART_Get_Flag_Status(pUSART, USART_FLAG_RXNE));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSART->USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSART->USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) rx_buff) = (pUSART->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the rx_buff two times
				rx_buff++;
				rx_buff++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*rx_buff = (pUSART->pUSARTx->DR  & (uint8_t)0xFF);
				 
				//Increment the rx_buff
				rx_buff++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame
			//check are we using USART_ParityControl control or not
			if(pUSART->USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *rx_buff = (pUSART->pUSARTx->DR & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *rx_buff = (pUSART->pUSARTx->DR & (uint8_t)0X7F);

			}			
			rx_buff++;
		}
	}
}

uint8_t USART_Send_DataIT(USART_Config_t *pUSART, uint8_t *tx_buff, uint32_t Len)
{
	uint8_t txstate = pUSART->pUSART_t->TxState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSART->pUSART_t->Len = Len;
		pUSART->pUSART_t->tx_buff = tx_buff;
		pUSART->pUSART_t->TxState = USART_BUSY_IN_TX;

		//Implement the code to enable interrupt for TXE
		pUSART->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		//Implement the code to enable interrupt for TC 
		pUSART->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}
	return txstate; // so application can wait until transmission is done
}

uint8_t USART_ReceiveDataIT(USART_Config_t *pUSART, uint8_t *rx_buff, uint32_t Len)
{
	uint8_t rxstate = pUSART->pUSART_t->RxState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSART->pUSART_t->Len = Len;
		pUSART->pUSART_t->rx_buff = rx_buff;
		pUSART->pUSART_t->RxState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSART->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	return rxstate;
}

// Use the SR register bit macros to identify the flag
uint8_t USART_Get_Flag_Status(USART_Config_t * pUSART, uint8_t flag_name)
{
    if(pUSART->pUSARTx->SR & flag_name)
    {
    	return SET;
    }

   return RESET;
}

//Clear Flag: Use SR Register Bit Macros to define the flag
// void USART_Clear_Flag(USART_Config_t * pUSART, uint8_t flag_name)
// {
// 	uint32_t dummyread;
// 	dummyread = pUSART->pUSARTx->SR;
// 	(void) dummyread;
// }

void USART_IRQ_Interrupt_Config(uint8_t IRQNumber, uint8_t EN)
{
	if(EN == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{		
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{		
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{		
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}
}

void USART_IRQ_Priority_Config(uint8_t IRQNumber, uint16_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_OF_PRIORITY_BITS_USED) ;

	*(  NVIC_PR_BASE + iprx ) |=  ( IRQPriority << shift_amount );
}


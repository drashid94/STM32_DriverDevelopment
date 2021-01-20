#include "i2c_driver.h"

static void I2C_Generate_Start_Condition(I2C_Config_t* pI2C)
{
	/* Set Start bit in CR1 register */
	pI2C->I2C_Addr->CR1 |= (1 << I2C_CR1_START);
	/* Wait for SB=1 (successful start condition) */
	while(!(pI2C->I2C_Addr->SR1 & (1)));
}


static void I2C_Generate_Stop_Condition(I2C_Config_t* pI2C)
{
	pI2C->I2C_Addr->CR1 |= (1<<I2C_CR1_STOP);
}


void I2C_Execute_Address_Phase(I2C_Config_t* pI2C, uint8_t SlaveAddr, uint8_t rw)
{
	//Must send Slave Address with R/W bit set to 0 in the address phase
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~1;
	SlaveAddr |= rw;
	pI2C->I2C_Addr->DR = SlaveAddr;
}

void I2C_Per_Enable(I2C_Config_t *pI2C, uint8_t EN)
{
	pI2C->I2C_Addr->CR1 |= 1;
}


void I2C_Init(I2C_Config_t* pI2C)
{
	uint16_t tempreg;
	/* Enable Peripheral Clock */
	if(pI2C->I2C_Addr == I2C1)
	{
		I2C1_PCLK_EN;
	}else if(pI2C->I2C_Addr == I2C2)
	{
		I2C2_PCLK_EN;
	}else	if(pI2C->I2C_Addr == I2C3)
	{
		I2C3_PCLK_EN;
	}	
	/* Serial Clock Configuration */
	uint32_t Clk_Freq = RCC_Get_APBClck_Freq();
	pI2C->I2C_Addr->CR2 |= (Clk_Freq << 0); 
	
	/* Configure Device Address */
	pI2C->I2C_Addr->OAR1 |= (1 << 14);
	pI2C->I2C_Addr->OAR1 |= (pI2C->Device_Addr << 1);
	
	/* CCR Configuration (Serial Clock Speed) */
	if(pI2C->SclSpeed < 100000)
	{
		//Standard Mode
		uint32_t ccr_value = Clk_Freq*1000000/(2*pI2C->SclSpeed);
		pI2C->I2C_Addr->CCR |= (ccr_value & 0xfff);
	}else
	{
		//Fast Mode
		uint32_t ccr_value = 0;
		if(pI2C->FM_Duty_Cycle == I2C_CONFIG_DUTY_2)
		{
			ccr_value = Clk_Freq/(3*pI2C->SclSpeed);
		}else
		{
			ccr_value = Clk_Freq/(25*pI2C->SclSpeed);
		}
		pI2C->I2C_Addr->CCR |= (1 << 15); // Enables Fast Mode
		pI2C->I2C_Addr->CCR |= (ccr_value);
		pI2C->I2C_Addr->CCR |= (pI2C->FM_Duty_Cycle << 14);
	}
	// trise calculation
	if(pI2C->SclSpeed <= I2C_CONFIG_SCLSPEED_SM)
	{
		//Standard mode 		
		tempreg = (RCC_Get_APBClck_Freq()) + 1;		
	}else
	{
		tempreg = (RCC_Get_APBClck_Freq() * 300) + 1;
	}
	pI2C->I2C_Addr->TRISE = (tempreg & 0x3F);
}

void I2C_ACK_Control(I2C_Config_t* I2C, uint8_t EN)
{
	I2C->I2C_Addr->CR1 &= ~(1 << I2C_CR1_ACK);
	I2C->I2C_Addr->CR1 |= (EN << I2C_CR1_ACK);
}

void I2C_MSend(I2C_Config_t* pI2C, uint8_t* pTxBuffer, uint32_t len, uint8_t SlaveAddr)
{
	//Start Condition
	I2C_Generate_Start_Condition(pI2C);
	//Addressing phase + R/W bit
	I2C_Execute_Address_Phase(pI2C, SlaveAddr, 0);
	//Clear Addr Flag by reading SR1 and SR2
	while( !  (pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_ADDR))   );
	uint32_t dummyread = pI2C->I2C_Addr->SR1;
	dummyread = pI2C->I2C_Addr->SR2;
	(void) dummyread;
	//Send data until length is 0
	while(len > 0)
	{
		while(!(pI2C->I2C_Addr->SR1 & (1<<7))); //wait until TxE bit is set (empty DR)
		pI2C->I2C_Addr->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}
	//Before generating stop, wait for TxE to be set adn BTF to be set
	while(!(pI2C->I2C_Addr->SR1 & (1<<7)));	
	while(!(pI2C->I2C_Addr->SR1 & (1<<I2C_SR1_BTF)));
	I2C_Generate_Stop_Condition(pI2C);
}

void I2C_MRec(I2C_Config_t* pI2C, uint8_t* pRxBuffer, uint32_t len, uint8_t SlaveAddr)
{
	//Generate start condition
	I2C_Generate_Start_Condition(pI2C);
	//Address Phase
	I2C_Execute_Address_Phase(pI2C, SlaveAddr, 1);

	
	if (len == 1) // if length is equal to one, must disable and stop before clearing the addres bit
	{
		I2C_ACK_Control(pI2C, DISABLE);
		while( !  (pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_ADDR))   );
		uint32_t dummyread = pI2C->I2C_Addr->SR1;
		dummyread = pI2C->I2C_Addr->SR2;
		(void) dummyread;
		while(!(pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_RXNE)));
		pI2C->I2C_Addr->CR1 |= (1 << I2C_CR1_STOP);
		*pRxBuffer = pI2C->I2C_Addr->DR;
	}
	if(len > 1)
	{
		while( !  (pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_ADDR))   );
		uint32_t dummyread = pI2C->I2C_Addr->SR1;
		dummyread = pI2C->I2C_Addr->SR2;
		(void) dummyread;
		
		for(int i = len; i > 0; i--)
		{
			while(!(pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_RXNE)));
			if ( i == 2)
			{
				I2C_ACK_Control(pI2C, DISABLE);
				pI2C->I2C_Addr->CR1 |= (1 << I2C_CR1_STOP);
			}
			*pRxBuffer = pI2C->I2C_Addr->DR;
			pRxBuffer++;
		}
	}
	if (pI2C->Ack_Control == ENABLE)
	{
		I2C_ACK_Control(pI2C, ENABLE);
	}
}

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable)
{
	if(Enable == ENABLE)
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
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}		
	}else if(Enable == DISABLE)
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t temp = 0;
	uint8_t temp1 = 0;
	temp = IRQNumber / 4;
	temp1 = IRQNumber % 4;
	uint8_t shiftvalue = temp1 * 8 + (8 - NO_OF_PRIORITY_BITS_USED);
	*(NVIC_PR_BASE + (temp)) |= (IRQPriority << shiftvalue);
}

void I2C_ITControl(I2C_Config_t* pI2C, uint8_t EN)
{
	if(EN == TRUE)
	{
		pI2C->I2C_Addr->CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2C->I2C_Addr->CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2C->I2C_Addr->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		pI2C->I2C_Addr->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2C->I2C_Addr->CR2 &= ~(1 << I2C_CR2_ITERREN);
		pI2C->I2C_Addr->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	}
}

uint8_t I2C_MRecIT(I2C_Config_t* pI2C, uint8_t* pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
		uint8_t busy = pI2C->TxRxState;
		if(busy != I2C_BUSY_IN_RX && busy != I2C_BUSY_IN_TX)
		{
			pI2C->pRxbuffer = pRxBuffer;
			pI2C->TxRxState = I2C_BUSY_IN_RX;
			pI2C->Rxlen = len;
			pI2C->Rxsize = len;
			pI2C->Device_Addr = SlaveAddr;
			pI2C->Sr = Sr;
			
			//Generate start condition
			I2C_Generate_Start_Condition(pI2C);
			
			//Enable Interrupt control bits
			pI2C->I2C_Addr->CR2 |= (1 << I2C_CR2_ITBUFEN);
			pI2C->I2C_Addr->CR2 |= (1 << I2C_CR2_ITEVTEN);
			pI2C->I2C_Addr->CR2 |= (1 << I2C_CR2_ITERREN);
			
		}
		return busy;
}
uint8_t I2C_MSendIT(I2C_Config_t* pI2C, uint8_t* pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t busy = pI2C->TxRxState;
	if(busy != I2C_BUSY_IN_TX && busy != I2C_BUSY_IN_RX)
	{
		pI2C->pTxBuffer = pTxBuffer;
		pI2C->TxLen = len;
		pI2C->TxRxState = I2C_BUSY_IN_TX;
		pI2C->Device_Addr = SlaveAddr;
		pI2C->Sr = Sr;
		
		//generate start condition
		I2C_Generate_Start_Condition(pI2C);
		
		//enable ITBUFEN control bit
		pI2C->I2C_Addr->CR2 |= ( 1 << I2C_CR2_ITBUFEN);
		
		//enable ITEVFEN control bit
		pI2C->I2C_Addr->CR2 |= (1 << I2C_CR2_ITEVTEN);
		
		// enable ITERRFEN control bit
		pI2C->I2C_Addr->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return busy;
}


void I2C_EV_IRQHandling(I2C_Config_t *pI2C) //Interrupt Handling for both Master and Slave mode for the device
{
	
		uint32_t temp1, temp2, temp4;
		temp1 = pI2C->I2C_Addr->CR2 & (1 << I2C_CR2_ITEVTEN);
		temp2 = pI2C->I2C_Addr->CR2 & (1 << I2C_CR2_ITBUFEN);
		temp4 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_SB);
	//1. Handle for interrupt generated by SB event (Start condition)
		if(temp1 && temp4)
		{
			//SB Interrupt -> Perform the address phase
			if(pI2C->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_Execute_Address_Phase(pI2C, pI2C->Device_Addr, 0); // 1 is to trigger the write function
			}
			else if(pI2C->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_Execute_Address_Phase(pI2C, pI2C->Device_Addr, 1);
			}
		}

		temp4 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_ADDR);
	//2. Handle for interrupt generated by ADDR event
		if(temp1 && temp4)
		{
			//ADDR interrupt
			// clear addr flag
				if(pI2C->I2C_Addr->SR2 & (1 << I2C_SR2_MSL) && pI2C->TxRxState == I2C_BUSY_IN_RX) // if device is in master mode and it is busy in rx
				{				
						if(pI2C->Rxsize == 1)
						{
							//first we must disable the acking
							I2C_ACK_Control(pI2C, DISABLE);
						}
						uint32_t dummyread = pI2C->I2C_Addr->SR1;
						dummyread = pI2C->I2C_Addr->SR2;
						(void) dummyread;
				}
				else // if device is in mastertx or slave
				{
						uint32_t dummyread = pI2C->I2C_Addr->SR1;
						dummyread = pI2C->I2C_Addr->SR2;
						(void) dummyread;
				}
					
					
		}
	
		temp4 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_BTF);
	//3. Handle for interrupt generated by BTF event
		if(temp1 && temp4)
		{
			// BTF Flag is set
			if(pI2C->TxRxState == I2C_BUSY_IN_TX)
			{
				// make sure that the TXE is also set
				if(pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_TXE) && !pI2C->TxLen)
				{
					// BTF, TXE = 1
					//1. generate stop condition
						if(!pI2C->Sr)
							I2C_Generate_Stop_Condition(pI2C);
					//2. reset all the member elements of the handle structure
						I2C_Close_TX(pI2C);
					//. notify the application about transmition complete
						I2C_ApplicationEventCallback(pI2C, I2C_EV_TX_CMPLT);
				}
			}
		}
		
		temp4 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_STOPF);
	//4. Handle for interrupt generated by STOPF event
		if(temp1 && temp4)
		{
			//Stop Flag is set
			//This mode can only be executed in slave mode as Stop flag will not be set in master mode
			//Clear the stop flag: read Sr1, write to CR1
			//we already read from Sr1 to get the temp values
			pI2C->I2C_Addr->CR1 |= 0x0000;
			//Notify Application that stop event has occured
			I2C_ApplicationEventCallback(pI2C, I2C_EV_STOPF);
		}
		
		temp4 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_TXE);
	//5. Handle for interrupt generated by TXE event
		if(temp1 && temp2 && temp4 && (pI2C->I2C_Addr->SR2 & (1 << I2C_SR2_MSL)))
		{
			// TXE interrupt
			// Must do data transmition
			
			if(pI2C->TxRxState == I2C_BUSY_IN_TX && pI2C->TxLen > 0)
			{
				//1. load the data into DR
				pI2C->I2C_Addr->DR = *pI2C->pTxBuffer;
				
				//2. decrement the TXlen
				pI2C->TxLen--;
				
				//3. Increment the buffer address
				pI2C->pTxBuffer++;
			}
		}
		else if(temp1 && temp2 && temp4 && !(pI2C->I2C_Addr->SR2 & (1 << I2C_SR2_MSL)))
		{
			//slave
			//check if device is in transmit mode
			if(pI2C->I2C_Addr->SR2 & (1 << I2C_SR2_TRA))
			{
				//notify application
				I2C_ApplicationEventCallback(pI2C, I2C_EV_SLAVE_SEND);
				
				
				
			}
		}

		temp4 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_RXNE);		
	//6. Handle for interrupt generated by RXNE event
		if( temp1 && temp2 && temp4)
		{
			// RXNE interrupt
			if(pI2C->TxRxState == I2C_BUSY_IN_RX && pI2C->I2C_Addr->SR2 & (1 << I2C_SR2_MSL))
			{
				// We do the data reception
				if(pI2C->Rxsize == 1)
				{
					*pI2C->pRxbuffer = pI2C->I2C_Addr->DR;
					pI2C->Rxlen--;
				}
				else if (pI2C->Rxsize > 1)
				{
					if(pI2C->Rxlen == 2)
					{
						I2C_ACK_Control(pI2C, DISABLE);
					}
					*pI2C->pRxbuffer = pI2C->I2C_Addr->DR;
					pI2C->Rxlen--;
					pI2C->pRxbuffer++;
				}
				if(pI2C->Rxlen == 0)
				{
					//close communication
					I2C_Generate_Stop_Condition(pI2C);
					
					//Close Rx
					I2C_Close_RX(pI2C);
					
					//Notify application
					I2C_ApplicationEventCallback(pI2C, I2C_EV_RX_CMPLT);
					
				}
			}
			else if(!(pI2C->I2C_Addr->SR2 & (1 << I2C_SR2_TRA)))
			{
				//Notify application
				I2C_ApplicationEventCallback(pI2C, I2C_EV_SLAVE_RCV);
				
			}
		}
}

void I2C_Close_RX(I2C_Config_t* pI2C)
{
	pI2C->I2C_Addr->CR2 *= ~(1 << I2C_CR2_ITBUFEN);
	pI2C->I2C_Addr->CR2 *= ~(1 << I2C_CR2_ITERREN);
	pI2C->I2C_Addr->CR2 *= ~(1 << I2C_CR2_ITEVTEN);
	
	pI2C->TxRxState = I2C_READY;
	pI2C->pRxbuffer = NULL;
	pI2C->Rxlen = 0;
	pI2C->Rxsize = 0;
	if(pI2C->Ack_Control == ENABLE)
		I2C_ACK_Control(pI2C, ENABLE);
}

void I2C_Close_TX(I2C_Config_t* pI2C)
{
	pI2C->I2C_Addr->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	pI2C->I2C_Addr->CR2 &= ~(1 << I2C_CR2_ITERREN);
	pI2C->I2C_Addr->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
	
	pI2C->TxRxState = I2C_READY;
	pI2C->pTxBuffer = NULL;
	pI2C->TxLen = 0;
	if(pI2C->Ack_Control == ENABLE)
		I2C_ACK_Control(pI2C, ENABLE);
}

void I2C_ERR_IRQHandling(I2C_Config_t *pI2C)
{
	uint32_t temp1, temp2;

	temp2 = pI2C->I2C_Addr->CR2 & (1 << I2C_CR2_ITERREN);
	
	temp1 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_BERR);
	//Check bus error
	if(temp1 && temp2)
	{
		//Clear Bus Error flag
		pI2C->I2C_Addr->SR1 &= ~(1 << I2C_SR1_BERR);
		
		//Notify application
		I2C_ApplicationEventCallback(pI2C, I2C_ERR_BERR);
	}
	
	temp1 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2)
	{
		//Clear ARLO flag
		pI2C->I2C_Addr->SR1 &= ~(1 << I2C_SR1_ARLO);
		
		//Notify application
		I2C_ApplicationEventCallback(pI2C, I2C_ERR_ARLO);
	}
	
	temp1 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_AF);
	if(temp1 && temp2)
	{
		//Clear AF flag
		pI2C->I2C_Addr->SR1 &= ~(1 << I2C_SR1_AF);
		
		//Notify application
		I2C_ApplicationEventCallback(pI2C, I2C_ERR_AF);
	}
	
	temp1 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_OVR);
	if (temp1 && temp2)
	{
		//Clear OVR flag
		pI2C->I2C_Addr->SR1 &= ~(1 << I2C_SR1_OVR);
		
		//Notify application
		I2C_ApplicationEventCallback(pI2C, I2C_ERR_OVR);
	}
	
	temp1 = pI2C->I2C_Addr->SR1 & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2)
	{
		//Clear timeout flag
		pI2C->I2C_Addr->SR1 &= ~(1 << I2C_SR1_TIMEOUT);
		
		//Notify application
		I2C_ApplicationEventCallback(pI2C, I2C_ERR_TIMEOUT);
	}
}

void I2C_SSendIT(I2C_Config_t* pI2C, uint8_t data)
{
	pI2C->I2C_Addr->DR = data;
}

uint8_t I2C_SRecIT(I2C_Config_t* pI2C)
{
	return pI2C->I2C_Addr->DR;
}


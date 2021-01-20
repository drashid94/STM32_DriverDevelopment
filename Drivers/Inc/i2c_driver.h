#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H
#include "stm32f446xx.h"
#include "rcc_driver.h"

#define I2C_CONFIG_SCLSPEED_FM 		300000
#define I2C_CONFIG_SCLSPEED_SM 		50000

#define I2C_CONFIG_ACKENABLE_ON		1
#define I2C_CONFIG_ACKENABLE_OFF	0

#define I2C_CONFIG_DUTY_2			0
#define I2C_CONFIG_DUTY_16_9		1

#define I2C_READY					1
#define I2C_BUSY_IN_RX				2
#define I2C_BUSY_IN_TX				3

#define I2C_EV_TX_CMPLT 			0
#define I2C_EV_RX_CMPLT				1
#define I2C_EV_STOPF				2
#define I2C_ERR_BERR				3
#define I2C_ERR_ARLO				4
#define I2C_ERR_AF					5
#define I2C_ERR_OVR					6
#define I2C_ERR_TIMEOUT				7
#define I2C_EV_SLAVE_RCV			8
#define I2C_EV_SLAVE_SEND			9
#define I2C_ERR_AF_FAIL				10

typedef struct{
	I2C_TypeDef* I2C_Addr;
	uint32_t SclSpeed;
	uint8_t Device_Addr;
	uint8_t Ack_Control;
	uint8_t FM_Duty_Cycle;
	// Interrupt Additions
	uint8_t *pTxBuffer;
	uint8_t *pRxbuffer;
	uint32_t TxLen;
	uint32_t Rxlen;
	uint8_t TxRxState;
	uint32_t Rxsize;
	uint8_t Sr;
}I2C_Config_t;

void I2C_Init(I2C_Config_t* pI2C);

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t Enable);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

void I2C_EV_IRQHandling(I2C_Config_t *pI2C);
void I2C_ERR_IRQHandling(I2C_Config_t *pI2C);

void I2C_Close_RX(I2C_Config_t* pI2C);
void I2C_Close_TX(I2C_Config_t* pI2C);

void I2C_Reset_Handle(I2C_Config_t* pI2C);
void I2C_ACK_Control(I2C_Config_t* pI2C, uint8_t EN);

void I2C_MSend(I2C_Config_t* pI2C, uint8_t* pTxBuffer, uint32_t len, uint8_t SlaveAddr);
uint8_t I2C_MSendIT(I2C_Config_t* pI2C, uint8_t* pTxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_MRec(I2C_Config_t* pI2C, uint8_t* pRxBuffer, uint32_t len, uint8_t SlaveAddr);
uint8_t I2C_MRecIT(I2C_Config_t* pI2C, uint8_t* pRxBuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);

void I2C_SSend(I2C_Config_t* pI2C, uint8_t* pTxBuffer, uint32_t len);
void I2C_SSendIT(I2C_Config_t* pI2C, uint8_t data);

void I2C_SRec(I2C_Config_t* pI2C, uint8_t* pRxBuffer, uint32_t len);
uint8_t I2C_SRecIT(I2C_Config_t* pI2C);

void I2C_ITControl(I2C_Config_t* pI2C, uint8_t EN);

void I2C_Per_Enable(I2C_Config_t* pI2C, uint8_t EN);

void I2C_ApplicationEventCallback(I2C_Config_t* pI2C, uint8_t callback_code);
#endif

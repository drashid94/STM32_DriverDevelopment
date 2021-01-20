#ifndef STM32F446XX_SPI_DRV_H
#define STM32F446XX_SPI_DRV_H
#include "stm32f446xx.h"

/* SPI MACROS */
#define SPI_MODE_SLAVE					0
#define SPI_MODE_MASTER					1

#define SPI_SPEED_FAST					0
#define SPI_SPEED_MED						3
#define SPI_SPEED_SLOW					6

#define SPI_CONFIGURATION_FD		0
#define SPI_CONFIGURATION_HD		1
#define SPI_CONFIGURATION_SP_RX	0
#define SPI_CONFIGURATION_SP_TX	1

#define SPI_DF_8								1
#define SPI_DF_16								0

#define SPI_CPHA_FIRST_EDGE			0
#define SPI_CPHA_SECOND_EDGE		1

#define SPI_CPOL_IDLE_LOW				0
#define SPI_CPOL_IDLE_HIGH			1

#define SPI_SSM_SW_EN						1
#define SPI_SSM_SW_DIS					0

#define SPI_STATUS_READY				0
#define SPI_STATUS_TxBusy				1
#define SPI_STATUS_RxBusy				2

typedef struct{
	SPI_TypeDef *pSPIx;
	uint8_t *TxBuffer;
	uint8_t *RxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;
	uint8_t SPI_Mode;
	uint8_t SPI_Configuration;
	uint8_t SPI_Data_Format;
	uint8_t SPI_CPHA;
	uint8_t SPI_CPOL;
	uint8_t SPI_SSM;
	uint8_t	SPI_Speed;
}SPI_Config;

void SPI_PeriClockControl(SPI_Config *pSPIx, uint8_t ENorDIS);
void SPI_Init(SPI_Config *pSPIx);
void SPI_DEInit(SPI_Config *pSPIx);
void SPI_Send(SPI_Config *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_Receive(SPI_Config *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_Per_Enable(SPI_Config *pSPIx, uint8_t ENoRDIS);
void SPI_Enable(SPI_Config *pSPIx);
void SPI_Disable(SPI_Config *pSPIx);
void SPI_LSB_First(SPI_Config *pSPIx, uint8_t ENorDIS);
uint8_t SPI_Get_Flag_Status(SPI_Config *pSPIx, uint8_t bit); 
void SPI_SSI_CONFIG(SPI_Config *pSPIx, uint8_t ENorDIS);
void SPI_SSOE_CONFIG(SPI_Config *pSPIx, uint8_t ENorDIS);
void SPI_ITSend(SPI_Config *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ITReceive(SPI_Config *pSPIx, uint8_t *pRxBuffer, uint32_t Len);
void SPI_IRQConfig(SPI_Config *pSPIx, uint8_t IRQNumber, uint8_t ENorDIS);
void SPI_IRQPriorityConfig(SPI_Config *pSPIx, uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandler(SPI_Config *pSPIx);


#endif

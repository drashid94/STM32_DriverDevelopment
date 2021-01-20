#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "stm32f446xx.h"

#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TXRX  2

/*
 *@USART_Baud
 *Possible options for USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define SUART_STD_BAUD_3M 					3000000

/*
 *@USART_ParityControl
 *Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD   2
#define USART_PARITY_EN_EVEN  1
#define USART_PARITY_DISABLE   0

/*
 *@USART_WordLength
 *Possible options for USART_WordLength
 */
#define USART_WORDLEN_8BITS  0
#define USART_WORDLEN_9BITS  1

/*
 *@USART_NoOfStopBits
 *Possible options for USART_NoOfStopBits
 */
#define USART_STOPBITS_1     0
#define USART_STOPBITS_0_5   1
#define USART_STOPBITS_2     2
#define USART_STOPBITS_1_5   3

/*
 *@USART_HWFlowControl
 *Possible options for USART_HWFlowControl
 */
#define USART_HW_FLOW_CTRL_NONE    	0
#define USART_HW_FLOW_CTRL_CTS    	1
#define USART_HW_FLOW_CTRL_RTS    	2
#define USART_HW_FLOW_CTRL_CTS_RTS	3


typedef struct
{
	uint8_t USART_Mode;
	uint8_t USART_Baud;
	uint8_t USART_StopBits;
	uint8_t USART_word_length;
	uint8_t USART_Parity_control;
	uint8_t HWFlowControl;
	USART_TypeDef * pUSART;
}USART_Config_t;

void USART_PeriClockControl(USART_Config_t * pUSARt, uint8_t EN);

void USART_Init(USART_Config_t * pUSART);
void USART_DeInit(USART_Config_t * pUSART);

void USART_Send_Data(USART_Config_t * pUSART, uint8_t * tx_buff, uint32_t Len);
void USART_Receive_Data(USART_Config_t * pUSART, uint8_t * rx_buff, uint32_t Len);
void USART_Send_DataIT(USART_Config_t * pUSART, uint8_t * tx_buff, uint32_t Len);
void USART_Receive_DataIT(USART_Config_t * pUSART, uint8_t * rx_buff, uint32_t Len);

void USART_Peripheral_Control(USART_Config_t * pUSARt, uint8_t EN);

uint8_t USART_Get_Flag_Status(USART_Config_t * pUSART, uint8_t flag_name);

void USART_Clear_Flag(USART_Config_t * pUSART, uint8_t flag_name);

void USART_IRQ_Interrupt_Config(USART_Config_t * pUSART, uint8_t EN);

void IRQ_Priority_Config(USART_Config_t * pUSART, uint32_t priority);

void IRQ_Handling(USART_Config_t * pUSART);

void Application_Event_Callback(USART_Config_t * pUSART, uint8_t EV);

#endif

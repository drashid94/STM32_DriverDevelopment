#ifndef STM32F446XX_H
#define STM32F446XX_H
#include <stdint.h>
#include <stddef.h>

/*******************************Processor Specifoc Details*********************************************/
//NVIC ISER Register Addresses (Enable interrupt on corresponding IRQ Num)
#define NVIC_ISER0   					((_vo uint32_t*)0xE000E100)
#define NVIC_ISER1   					((_vo uint32_t*)0xE000E104)
#define NVIC_ISER2   					((_vo uint32_t*)0xE000E108)
#define NVIC_ISER3   					((_vo uint32_t*)0xE000E10C)
//NVIC ISER Register Addresses	(Disable interrupt on corresponding IRQ Num)
#define NVIC_ICER0   					((_vo uint32_t*)0xE000E180)
#define NVIC_ICER1   					((_vo uint32_t*)0xE000E184)
#define NVIC_ICER2   					((_vo uint32_t*)0xE000E188)
#define NVIC_ICER3   					((_vo uint32_t*)0xE000E18C)
//NVIC Priority Register Addresses
#define NVIC_PR_BASE 					((_vo uint32_t*)0xE000E400)
//Number of priority bits that can be set by user in NVIC. (16 possible levels (2^4))
#define NO_OF_PRIORITY_BITS_USED 		4

#define _vo volatile
	
/* BASE ADDRESSES FOR MEMORY*/
#define FLASH_BASEADDR 					0x08000000U
#define SRAM1_ADDR 						0x200000000U
#define SRAM2_ADDR 						0x2001C000U
#define SRAM 							SRAM1_ADDR
#define ROM BASE_ADDR					0x1FFF0000U

/* BASE ADDRESSES FOR BUS DOMAINS */
#define PERIPH_BASE						0x40000000U
#define APB1_BASEADDR					PERIPH_BASE
#define APB2_BASEADDR					0x40010000U
#define AHB1_BASEADDR					0x40020000U
#define AHB2_BASEADDR					0x50000000U
#define AHB3_BASEADDR					0x60000000U

/* BASE ADDRESSES FOR PERIPHERALS HANGING ON AHB1 */
#define GPIOA_BASEADDR 					(AHB1_BASEADDR)
#define GPIOB_BASEADDR 					(AHB1_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR 					(AHB1_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR 					(AHB1_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR 					(AHB1_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR 					(AHB1_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR 					(AHB1_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR 					(AHB1_BASEADDR + 0x1C00U)
#define RCC_BASEADDR					(AHB1_BASEADDR + 0x3800)

/* BASE ADDRESSES FOR PERIPHERALS ON APB1 */
#define I2C1_BASEADDR 					(APB1_BASEADDR + 0x5400)
#define I2C2_BASEADDR 					(APB1_BASEADDR + 0x5800)
#define I2C3_BASEADDR 					(APB1_BASEADDR + 0x5C00)

#define SPI2_BASEADDR 					(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR 					(APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR 				(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR 				(APB1_BASEADDR + 0x4800)
#define USART4_BASEADDR 				(APB1_BASEADDR + 0x4C00)
#define USART5_BASEADDR 				(APB1_BASEADDR + 0x5000)

/* BASE ADDRESSES FOR PERIPHERALS ON APB2 */
#define SPI1_BASEADDR					(APB2_BASEADDR + 0x3000)
#define USART1_BASEADDR					(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR					(APB2_BASEADDR + 0x1400)
#define SYSCFG_BASEADDR					(APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR					(APB2_BASEADDR + 0x3C00)

/* REGISTER DEFINITIONS */
typedef struct
{
	_vo uint32_t MODER;
	_vo uint32_t OTYPER;
	_vo uint32_t OSPEEDR;
	_vo uint32_t PUPDR;
	_vo uint32_t IDR;
	_vo uint32_t ODR;
	_vo uint32_t BSRR;
	_vo uint32_t LCKR;
	_vo uint32_t AFRL; // AF[0] is AFRL register; AF[1] is AFRH register
	_vo uint32_t AFRH;
}GPIO_TypeDef;

#define GPIOA 							((GPIO_TypeDef*) GPIOA_BASEADDR)
#define GPIOB 							((GPIO_TypeDef*) GPIOB_BASEADDR)
#define GPIOC 							((GPIO_TypeDef*) GPIOC_BASEADDR)
#define GPIOD 							((GPIO_TypeDef*) GPIOD_BASEADDR)
#define GPIOE 							((GPIO_TypeDef*) GPIOE_BASEADDR)
#define GPIOF 							((GPIO_TypeDef*) GPIOF_BASEADDR)
#define GPIOG 							((GPIO_TypeDef*) GPIOG_BASEADDR)
#define GPIOH 							((GPIO_TypeDef*) GPIOH_BASEADDR)

typedef struct
{
	_vo uint32_t CR;
	_vo uint32_t PLLCFGR;
	_vo uint32_t CFGR;
	_vo uint32_t CIR;
	_vo uint32_t AHB1RSTR;
	_vo uint32_t AHB2RSTR;
	_vo uint32_t AHB3RSTR;
	_vo uint32_t RESRVED1;
	_vo uint32_t APB1RSTR;
	_vo uint32_t APB2RSTR;
	_vo uint32_t RESERVED2;
	_vo uint32_t RESERVED3;
	_vo uint32_t AHB1ENR;
	_vo uint32_t AHB2ENR;
	_vo uint32_t AHB3ENR;
	_vo uint32_t RESERVED4;
	_vo uint32_t APB1ENR;
	_vo uint32_t APB2ENR;
	_vo uint32_t RESERVED5;
	_vo uint32_t RESERVED6;
	_vo uint32_t AHB1LPENR;
	_vo uint32_t AHB2LPENR;
	_vo uint32_t AHB3LPENR;
	_vo uint32_t RESERVED7;
	_vo uint32_t APB1LPENR;
	_vo uint32_t APB2LPENR;
	_vo uint32_t RESERVED8;
	_vo uint32_t RESERVED9;
	_vo uint32_t BDCR;
	_vo uint32_t CSR;
	_vo uint32_t RESERVED10;
	_vo uint32_t RESERVED11;
	_vo uint32_t SSCGR;
	_vo uint32_t PLLI2SCFGR;
	_vo uint32_t PLLSAI;
	_vo uint32_t DCKCFGR;
	_vo uint32_t CKGATENR;
	_vo uint32_t DCKCFGR2;
}RCCTypeDef;

#define RCC ((RCCTypeDef*) RCC_BASEADDR)

typedef struct{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t CRCPR;
	_vo uint32_t RXCRCR;
	_vo uint32_t TXCRCR;
	_vo uint32_t I2SCFGR;
	_vo uint32_t I2SPR;	
}SPI_TypeDef;

#define SPI1	((SPI_TypeDef*)SPI1_BASEADDR)
#define SPI2	((SPI_TypeDef*)SPI2_BASEADDR)
#define SPI3	((SPI_TypeDef*)SPI3_BASEADDR)

typedef struct{
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t OAR1;
	_vo uint32_t OAR2;
	_vo uint32_t DR;
	_vo uint32_t SR1;
	_vo uint32_t SR2;
	_vo uint32_t CCR;
	_vo uint32_t TRISE;
	_vo uint32_t FLTR;
}I2C_TypeDef;

#define I2C1	((I2C_TypeDef*)I2C1_BASEADDR)
#define I2C2	((I2C_TypeDef*)I2C2_BASEADDR)
#define I2C3	((I2C_TypeDef*)I2C3_BASEADDR)

typedef struct
{
	_vo uint32_t IMR;
	_vo uint32_t EMR;
	_vo uint32_t RTSR;
	_vo uint32_t FTSR;
	_vo uint32_t SWIER;
	_vo uint32_t PR;
}EXTI_TypeDef;

#define EXTI ((EXTI_TypeDef*) EXTI_BASEADDR)

typedef struct
{
	_vo uint32_t MEMRMP;
	_vo uint32_t PMC;
	_vo uint32_t EXTICR[4];
	uint32_t RESERVED[2];
	_vo uint32_t CMPCR;
	uint32_t RESERVED1[2];
	_vo uint32_t CFGR;
}SYSCFG_TypeDef;

#define SYSCFG 	((SYSCFG_TypeDef*)SYSCFG_BASEADDR)

typedef struct
{
	_vo uint32_t SR;
	_vo uint32_t DR;
	_vo uint32_t BRR;
	_vo uint32_t CR1;
	_vo uint32_t CR2;
	_vo uint32_t CR3;
	_vo uint32_t GTPR;
}USART_TypeDef;

#define USART1 						((USART_TypeDef*) USART1_BASEADDR)
#define USART2 						((USART_TypeDef*) USART2_BASEADDR)
#define USART3 						((USART_TypeDef*) USART3_BASEADDR)
#define USART4 						((USART_TypeDef*) USART4_BASEADDR)
#define USART5 						((USART_TypeDef*) USART5_BASEADDR)
#define USART6 						((USART_TypeDef*) USART6_BASEADDR)


/* Clock enable defintions for GPIO peripherals */
#define GPIOA_PCLK_EN				RCC->AHB1ENR |= (1 << 0)
#define GPIOB_PCLK_EN				RCC->AHB1ENR |= (1 << 1)
#define GPIOC_PCLK_EN 				RCC->AHB1ENR |= (1 << 2)
#define GPIOD_PCLK_EN 				RCC->AHB1ENR |= (1 << 3)
#define GPIOE_PCLK_EN 				RCC->AHB1ENR |= (1 << 4)
#define GPIOF_PCLK_EN 				RCC->AHB1ENR |= (1 << 5)
#define GPIOG_PCLK_EN 				RCC->AHB1ENR |= (1 << 6)
#define GPIOH_PCLK_EN 				RCC->AHB1ENR |= (1 << 7)

/* Clcok enable definitions for I2C peripherals */
#define I2C3_PCLK_EN 				RCC->AHB1ENR |= (1 << 23)
#define I2C2_PCLK_EN 				RCC->AHB1ENR |= (1 << 22)
#define I2C1_PCLK_EN				RCC->APB1ENR |= (1 << 21)

/* Clock enable definitions for SPI peripherals */
#define SPI1_PCLK_EN 				RCC->APB2ENR |= (1 << 12)
#define SPI2_PCLK_EN 				RCC->APB1ENR |= (1 << 14)
#define SPI3_PCLK_EN 				RCC->APB1ENR |= (1 << 15)

/*USART Enable Macros */
#define USART1_PCLK_EN 				(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN 				(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN 				(RCC->APB1ENR |= (1 << 18))
#define USART4_PCLK_EN  			(RCC->APB1ENR |= (1 << 19))
#define USART5_PCLK_EN  			(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN 				(RCC->APB2ENR |= (1 << 5))

/* Clock enable for SYSCFG peripheral */
#define SYSCFG_PCLK_EN				RCC->APB2ENR |= (1 << 14)

/* Clock disable defintions for GPIO peripherals */
#define GPIOA_PCLK_DIS				RCC->AHB1ENR &= ~(1 << 0)
#define GPIOB_PCLK_DIS				RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_PCLK_DIS 				RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_PCLK_DIS 				RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_PCLK_DIS 				RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_PCLK_DIS  			RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_PCLK_DIS 				RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_PCLK_DIS 				RCC->AHB1ENR &= ~(1 << 7)

/* Clcok disable definitions for I2C peripherals */
#define I2C3_PCLK_DIS 				RCC->AHB1ENR &= ~(1 << 23)
#define I2C2_PCLK_DIS 				RCC->AHB1ENR &= ~(1 << 22)
#define I2C1_PCLK_DIS				RCC->APB1ENR &= ~(1 << 21)

/* Clock disable definitions for SPI peripherals */
#define SPI1_PCLK_DIS 				RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DIS 				RCC->APB1ENR &= ~(1 << 14)
#define SPI3_PCLK_DIS 				RCC->APB1ENR &= ~(1 << 15)

/* Clock disable definitions for USART peripherals */
#define USART1_PCLK_DIS				RCC->APB2ENR &= ~(1 << 4);
#define USART2_PCLK_DIS				RCC->APB1ENR &= ~(1 << 17);
#define USART3_PCLK_DIS				RCC->APB1ENR &= ~(1 << 18);
#define USART4_PCLK_DIS				RCC->APB1ENR &= ~(1 << 19);
#define USART5_PCLK_DIS				RCC->APB1ENR &= ~(1 << 20);
#define USART6_PCLK_DIS				RCC->APB2ENR &= ~(1 << 5);

/* Clock disable for SYSCFG peripheral */
#define SYSCFG_PCLK_DIS				RCC->APB2ENR &= ~(1 << 14)

//IRQ Number Definitions
#define EXTI0_IRQ 					6
#define EXTI1_IRQ 					7
#define EXTI2_IRQ 					8
#define EXTI3_IRQ 					9
#define EXTI4_IRQ 					10
#define EXTI9_5_IRQ 				23
#define EXTI10_15_IRQ 				40
#define SPI1_IRQ_POS				35
#define SPI2_IRQ_POS				36
#define SPI3_IRQ_POS				51
#define IRQ_NO_I2C1_EV     			31
#define IRQ_NO_I2C1_ER     			32
#define IRQ_NO_I2C2_EV     			33
#define IRQ_NO_I2C2_ER     			34
#define IRQ_NO_I2C3_EV     			72
#define IRQ_NO_I2C3_ER     			73

/*
USEFUL MACROS
*/
#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET 						DISABLE
#define GPIO_PIN_SET 				SET
#define GPIO_PIN_RESET 				RESET
#define TRUE 						1
#define FALSE						0

// GPIO Register Reset
#define GPIOA_REG_RESET()			do{RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); }while(0)
#define GPIOB_REG_RESET()			do{RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); }while(0)
#define GPIOC_REG_RESET()			do{RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); }while(0)
#define GPIOD_REG_RESET()			do{RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); }while(0)
#define GPIOE_REG_RESET()			do{RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); }while(0)
#define GPIOF_REG_RESET()			do{RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); }while(0)
#define GPIOG_REG_RESET()			do{RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); }while(0)
#define GPIOH_REG_RESET()			do{RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); }while(0)

/* Bit Position Macros: SPI*/
#define SPI_CR1_CPHA     			0
#define SPI_CR1_CPOL      			1
#define SPI_CR1_MSTR     			2
#define SPI_CR1_BR   				3
#define SPI_CR1_SPE     			6
#define SPI_CR1_LSBFIRST   			7
#define SPI_CR1_SSI     			8
#define SPI_CR1_SSM      			9
#define SPI_CR1_RXONLY      		10
#define SPI_CR1_DFF     			11
#define SPI_CR1_CRCNEXT   			12
#define SPI_CR1_CRCEN   			13
#define SPI_CR1_BIDIOE     			14
#define SPI_CR1_BIDIMODE      		15
#define SPI_CR2_RXDMAEN		 		0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_FRF					4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE				7
#define SPI_SR_RXNE					0
#define SPI_SR_TXE				 	1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR					3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY					7
#define SPI_SR_FRE					8

/* Bit Position Macros: I2C*/

#define I2C_CR1_PE					0
#define I2C_CR1_NOSTRETCH  			7
#define I2C_CR1_START 				8
#define I2C_CR1_STOP  			 	9
#define I2C_CR1_ACK 				10
#define I2C_CR1_SWRST  			 	15

#define I2C_CR2_FREQ				0
#define I2C_CR2_ITERREN			 	8
#define I2C_CR2_ITEVTEN			 	9
#define I2C_CR2_ITBUFEN 		  	10

#define I2C_OAR1_ADD0    		  	0
#define I2C_OAR1_ADD71 				1
#define I2C_OAR1_ADD98  			8
#define I2C_OAR1_ADDMODE   			15

#define I2C_SR1_SB 					0
#define I2C_SR1_ADDR 				1
#define I2C_SR1_BTF 				2
#define I2C_SR1_ADD10 				3
#define I2C_SR1_STOPF 				4
#define I2C_SR1_RXNE 				6
#define I2C_SR1_TXE 				7
#define I2C_SR1_BERR 				8
#define I2C_SR1_ARLO 				9
#define I2C_SR1_AF 					10
#define I2C_SR1_OVR 				11
#define I2C_SR1_TIMEOUT 			14

#define I2C_SR2_MSL					0
#define I2C_SR2_BUSY 				1
#define I2C_SR2_TRA 				2
#define I2C_SR2_GENCALL 			4
#define I2C_SR2_DUALF 				7

#define I2C_CCR_CCR 				0
#define I2C_CCR_DUTY 				14
#define I2C_CCR_FS  				15

/*Bit Position Macros : USART */
#define USART_CR1_SBK				0
#define USART_CR1_RWU 				1
#define USART_CR1_RE  				2
#define USART_CR1_TE 				3
#define USART_CR1_IDLEIE 			4
#define USART_CR1_RXNEIE  			5
#define USART_CR1_TCIE				6
#define USART_CR1_TXEIE				7
#define USART_CR1_PEIE 				8
#define USART_CR1_PS 				9
#define USART_CR1_PCE 				10
#define USART_CR1_WAKE  			11
#define USART_CR1_M 				12
#define USART_CR1_UE 				13
#define USART_CR1_OVER8  			15

#define USART_CR2_ADD   			0
#define USART_CR2_LBDL   			5
#define USART_CR2_LBDIE  			6
#define USART_CR2_LBCL   			8
#define USART_CR2_CPHA   			9
#define USART_CR2_CPOL   			10
#define USART_CR2_STOP   			12
#define USART_CR2_LINEN   			14

#define USART_CR3_EIE   			0
#define USART_CR3_IREN   			1
#define USART_CR3_IRLP  			2
#define USART_CR3_HDSEL   			3
#define USART_CR3_NACK   			4
#define USART_CR3_SCEN   			5
#define USART_CR3_DMAR  			6
#define USART_CR3_DMAT   			7
#define USART_CR3_RTSE   			8
#define USART_CR3_CTSE   			9
#define USART_CR3_CTSIE   			10
#define USART_CR3_ONEBIT   			11

#define USART_SR_PE        			0
#define USART_SR_FE        			1
#define USART_SR_NE        			2
#define USART_SR_ORE       			3
#define USART_SR_IDLE       		4
#define USART_SR_RXNE        		5
#define USART_SR_TC        			6
#define USART_SR_TXE        		7
#define USART_SR_LBD        		8
#define USART_SR_CTS        		9

// GPIO Port Code 
#define GPIO_GET_PORT_CODE(x)		((x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7: 0)
															
#endif

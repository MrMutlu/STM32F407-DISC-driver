/*
 * stm32f4xx.h
 *
 *  Created on: Mar 9, 2024
 *      Author: Mutlu
 */

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stdint.h>
///////////// Processor Specific Details ( STM32F407VG uses ARM CORTEX MX ) ///////////////////////
// NVIC ISERx Register Addresses //
#define NVIC_ISER0					((volatile uint32_t*) 0xE000E100U)
#define NVIC_ISER1					((volatile uint32_t*) 0xE000E104U)
#define NVIC_ISER2					((volatile uint32_t*) 0xE000E108U)
#define NVIC_ISER3					((volatile uint32_t*) 0xE000E10CU)
/* TO DO: There are 7 in total */

// NVIC ICERx Register Addresses //
#define NVIC_ICER0					((volatile uint32_t*) 0xE000E180U)
#define NVIC_ICER1					((volatile uint32_t*) 0xE000E184U)
#define NVIC_ICER2					((volatile uint32_t*) 0xE000E188U)
#define NVIC_ICER3					((volatile uint32_t*) 0xE000E18CU)
/* TO DO: Do others */

// NVIC Priority Register Address //
#define NVIC_PR_BASE_ADDR 	((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED			4 					//Number of priority bits implemented in Priority register


// Base addresses of FLASH and SRAM memories //
#define FLASH_BASEADDR				0x08000000U 			//FLASH address starts at here
#define SRAM1_BASEADDR				0x20000000U				//SRAM1 address starts at here ( 112 KB )
#define SRAM2_BASEADDR				0x2001C000U				//SRAM2 address starts at here ( 16 KB )
#define ROM_BASEADDR				0x1FFF0000U				//SYSTEM MEMORY address starts at here

// AHBx and APBx peripheral addresses //
#define APB1_BASEADDR				0x40000000U
#define APB2_BASEADDR				0x40010000U
#define AHB1_BASEADDR				0x40020000U
#define AHB2_BASEADDR				0x50000000U

// AHB1 peripheral addresses //
#define GPIOA_BASEADDR				0x40020000U
#define GPIOB_BASEADDR				0x40020400U
#define GPIOC_BASEADDR				0x40020800U
#define GPIOD_BASEADDR				0x40020C00U
#define GPIOE_BASEADDR				0x40021000U
#define GPIOF_BASEADDR				0x40021400U
#define GPIOG_BASEADDR				0x40021800U
#define GPIOH_BASEADDR				0x40021C00U
#define GPIOI_BASEADDR				0x40022000U
#define RCC_BASEADDR				0x40023800U
/* TO DO: Add other AHB1 peripheral addresses */

// APB1 peripheral addresses //
#define TIM2_BASEADDR				0x40000000U
#define TIM3_BASEADDR				0x40000400U
#define TIM4_BASEADDR				0x40000800U
#define TIM5_BASEADDR				0x40000C00U
#define TIM6_BASEADDR				0x40001000U
#define TIM7_BASEADDR				0x40001400U
#define TIM12_BASEADDR				0x40001800U
#define TIM13_BASEADDR				0x40001C00U
#define TIM14_BASEADDR				0x40002000U
#define SPI2_BASEADDR				0x40003800U
#define I2S2_BASEADDR				SPI2_BASEADDR 			// I2S2 and SPI2 shares same address
#define SPI3_BASEADDR				0x40003C00U				// I2S3 and SPI3 shares same address
#define I2S3_BASEADDR				SPI3_BASEADDR
#define USART2_BASEADDR				0x40004400U
#define USART3_BASEADDR 			0x40004800U
#define UART4_BASEADDR				0x40004C00U
#define UART5_BASEADDR				0x40005000U
#define I2C1_BASEADDR				0x40005400U
#define I2C2_BASEADDR				0x40005800U
#define I2C3_BASEADDR				0x40005C00U
/* TO DO: Add other APB1 peripheral addresses */

// APB2 peripheral addresses //
#define TIM1_BASEADDR				0x40010000U
#define TIM8_BASEADDR				0x40010400U
#define USART1_BASEADDR				0x40011000U
#define USART6_BASEADDR				0x40011400U
#define ADC123_BASEADDR				0x40012000U // ADC1-ADC2-ADC3
#define SPI1_BASEADDR				0x40013000U
#define EXTI_BASEADDR				0x40013C00U
#define TIM9_BASEADDR				0x40014000U
#define TIM10_BASEADDR				0x40014400U
#define TIM11_BASEADDR				0x40014800U
#define SYSCFG_BASEADDR				0x40013800U
/* TO DO: Add other APB2 peripheral addresses */

/*****************PERIPHERAL REGISTER STRUCTURES********************/
typedef struct
{
	volatile uint32_t MODER;		//Input,Output,Alternate and Analog Mode Register//
	volatile uint32_t OTYPER;		//Output Type Register ( push-pull or open-drain)//
	volatile uint32_t OSPEEDR;		//Output Speed Register//
	volatile uint32_t PUPDR;		//Pull-up/down Register//
	volatile uint32_t IDR;			//Input Data Register(read-only)//
	volatile uint32_t ODR;			//Output Data Register(R/W)//
	volatile uint32_t BSRR;			//Port Set/Reset Register//
	volatile uint32_t LCKR;			//Port Lock Register//
	volatile uint32_t AFRL;			//Alternate Function Register ( for 0-7 ports)//
	volatile uint32_t AFRH;			//Alternate Function Register ( for 8-15 ports)//

}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;			//clock control register//
	volatile uint32_t PLLCFGR;		//PLL configuration register//
	//PLL is a clock generation engine in the MCU which is used to generate the clock speed which is much higher than the internal HSI or external clock . In MCU, most of the peripheral like USB,Ethernet PHY cannot work if you clock the MCU by low speed  HSI or HSE. //
	volatile uint32_t CFGR;			//clock configuration register//
	volatile uint32_t CIR;			//clock interrupt register//
	volatile uint32_t AHB1RSTR;		//AHB1 peripheral reset register//
	volatile uint32_t AHB2RSTR;		//AHB2 peripheral reset register//
	volatile uint32_t AHB3RSTR;		//APB1 peripheral reset register//
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;		//APB1 peripheral reset register//
	volatile uint32_t APB2RSTR;		//APB2 peripheral reset register//
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	volatile uint32_t AHB1ENR;		//AHB1 peripheral clock enable register//
	volatile uint32_t AHB2ENR;		//AHB2 peripheral clock enable register//
	volatile uint32_t AHB3ENR;		//AHB3 peripheral clock enable register//
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR;		//APB1 peripheral clock enable register//
	volatile uint32_t APB2ENR;		//APB2 peripheral clock enable register//
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	volatile uint32_t AHB1LPENR;	//AHB1 peripheral clock enable in low power mode register//
	volatile uint32_t AHB2LPENR;	//AHB2 peripheral clock enable in low power mode register//
	volatile uint32_t AHB3LPENR;	//AHB3 peripheral clock enable in low power mode register//
	uint32_t RESERVED6;
	volatile uint32_t APB1LPENR;	//APB1 peripheral clock enable in low power mode register//
	volatile uint32_t APB2LPENR;	//APB2 peripheral clock enabled in low power mode//
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	volatile uint32_t BDCR;			//Backup domain control register//
	volatile uint32_t CSR;			//clock control & status register//
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	volatile uint32_t SSCGR;		//spread spectrum clock generation register//
	volatile uint32_t PLLI2SCFGR;	//PLLI2S configuration register//

}RCC_RegDef_t;

typedef struct
{
	volatile uint32_t IMR;			//Interrupt mask register//
	volatile uint32_t EMR;			//Event mask register//
	volatile uint32_t RTSR;			//Rising trigger selection register//
	volatile uint32_t FTSR;			//Falling trigger selection register//
	volatile uint32_t SWIER;		//Software interrupt event register//
	volatile uint32_t PR;			//Pending register//

}EXTI_RegDef_t;

typedef struct
{
	volatile uint32_t MEMRMP;		//Memory remap register//
	volatile uint32_t PMC;			//Peripheral mode configuration register//
	volatile uint32_t EXTICR[4];	//External interrupt configuration register//
	volatile uint32_t CMPCR;		//Compensation cell control register//

}SYSCFG_RegDef_t;

typedef struct
{
	volatile uint32_t CR1;			//Control-1 register//
	volatile uint32_t CR2;			//Control-2 register//
	volatile uint32_t SR;			//Status register//
	volatile uint32_t DR;			//Data register//
	volatile uint32_t CRCPR;		//CRC Polynomial register//
	volatile uint32_t RXCRCR;		//RX CRC register register//
	volatile uint32_t TXCRCR;		//TX CRC register register//
	volatile uint32_t I2SCFGR;		//I2S Configuration register//
	volatile uint32_t I2SPR;		//I2S Prescaler register//

}SPI_RegDef_t;


// Peripheral Definitions ( Base addresses -> xxx_RegDef_t ) //
#define GPIOA 			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI 			((GPIO_RegDef_t*) GPIOI_BASEADDR)
#define RCC				((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)
#define SPI1			((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*) SPI3_BASEADDR)

// Clock Enable Macros for GPIOx peripherals //
#define GPIOA_CLK_EN()		(RCC->AHB1ENR |= (1<<0)) // turns AHB1ENR's bit 0 to 1 ( bit 0 control GPIOA )
#define GPIOB_CLK_EN()		(RCC->AHB1ENR |= (1<<1)) // turns AHB1ENR's bit 1 to 1 ( bit 1 control GPIOB )
#define GPIOC_CLK_EN()		(RCC->AHB1ENR |= (1<<2)) // turns AHB1ENR's bit 2 to 1 ( bit 2 control GPIOC )
#define GPIOD_CLK_EN()		(RCC->AHB1ENR |= (1<<3)) // turns AHB1ENR's bit 3 to 1 ( bit 3 control GPIOD )
#define GPIOE_CLK_EN()		(RCC->AHB1ENR |= (1<<4)) // turns AHB1ENR's bit 4 to 1 ( bit 4 control GPIOE )
#define GPIOF_CLK_EN()		(RCC->AHB1ENR |= (1<<5)) // turns AHB1ENR's bit 5 to 1 ( bit 5 control GPIOF )
#define GPIOG_CLK_EN()		(RCC->AHB1ENR |= (1<<6)) // turns AHB1ENR's bit 6 to 1 ( bit 6 control GPIOG )
#define GPIOH_CLK_EN()		(RCC->AHB1ENR |= (1<<7)) // turns AHB1ENR's bit 7 to 1 ( bit 7 control GPIOH )
#define GPIOI_CLK_EN()		(RCC->AHB1ENR |= (1<<8)) // turns AHB1ENR's bit 8 to 1 ( bit 8 control GPIOI )

// Clock Enable Macros for I2Cx peripherals //
#define I2C1_CLK_EN()		(RCC->APB1ENR |= (1<<21)) // turns APB1ENR bit 21 to 1 ( bit 21 control I2C1 )
#define I2C2_CLK_EN()		(RCC->APB1ENR |= (1<<22)) // turns APB1ENR bit 22 to 1 ( bit 22 control I2C2 )
#define I2C3_CLK_EN()		(RCC->APB1ENR |= (1<<23)) // turns APB1ENR bit 22 to 1 ( bit 23 control I2C3 )

// Clock Enable Macros for SPIx peripherals //
#define SPI1_CLK_EN()		(RCC->APB2ENR |= (1<<12)) // turns APB2ENR bit 12 to 1 ( bit 12 control SPI1 )
#define SPI2_CLK_EN()		(RCC->APB1ENR |= (1<<14)) // turns APB1ENR bit 14 to 1 ( bit 14 control SPI2 )
#define SPI3_CLK_EN()		(RCC->APB1ENR |= (1<<15))

// Clock Enable Macros for USARTx/UARTx peripherals //
#define USART1_CLK_EN()		(RCC->APB2ENR |= (1<<4))
#define USART6_CLK_EN()		(RCC->APB2ENR |= (1<<5))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1<<17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1<<18))
#define UART4_CLK_EN()		(RCC->APB1ENR |= (1<<19))
#define UART5_CLK_EN()		(RCC->APB1ENR |= (1<<20))

// Clock Enable Macro for SYSCFG peripheral //
#define SYSCFG_CLK_EN()		(RCC->APB2ENR |= (1<<14))

// Clock Disable Macros for GPIOx peripherals //
#define GPIOA_CLK_DI()		(RCC->AHB1ENR &= ~(1<<0)) // turns AHB1ENR's bit 0 to 1 ( bit 0 control GPIOA )
#define GPIOB_CLK_DI()		(RCC->AHB1ENR &= ~(1<<1)) // turns AHB1ENR's bit 1 to 0 ( bit 1 control GPIOB )
#define GPIOC_CLK_DI()		(RCC->AHB1ENR &= ~(1<<2)) // turns AHB1ENR's bit 2 to 0 ( bit 2 control GPIOC )
#define GPIOD_CLK_DI()		(RCC->AHB1ENR &= ~(1<<3)) // turns AHB1ENR's bit 3 to 0 ( bit 3 control GPIOD )
#define GPIOE_CLK_DI()		(RCC->AHB1ENR &= ~(1<<4)) // turns AHB1ENR's bit 4 to 0 ( bit 4 control GPIOE )
#define GPIOF_CLK_DI()		(RCC->AHB1ENR &= ~(1<<5)) // turns AHB1ENR's bit 5 to 0 ( bit 5 control GPIOF )
#define GPIOG_CLK_DI()		(RCC->AHB1ENR &= ~(1<<6)) // turns AHB1ENR's bit 6 to 0 ( bit 6 control GPIOG )
#define GPIOH_CLK_DI()		(RCC->AHB1ENR &= ~(1<<7)) // turns AHB1ENR's bit 7 to 0 ( bit 7 control GPIOH )
#define GPIOI_CLK_DI()		(RCC->AHB1ENR &= ~(1<<8)) // turns AHB1ENR's bit 8 to 0 ( bit 8 control GPIOI )

// Clock Disable Macros for I2Cx peripherals //
#define I2C1_CLK_DI()		(RCC->APB1ENR &= ~(1<<21)) // turns APB1ENR bit 21 to 0 ( bit 21 control I2C1 )
#define I2C2_CLK_DI()		(RCC->APB1ENR &= ~(1<<22)) // turns APB1ENR bit 22 to 0 ( bit 22 control I2C2 )
#define I2C3_CLK_DI()		(RCC->APB1ENR &= ~(1<<23)) // turns APB1ENR bit 22 to 0 ( bit 23 control I2C3 )

// Clock Disable Macros for SPIx peripherals //
#define SPI1_CLK_DI()		(RCC->APB2ENR &= ~(1<<12)) // turns APB2ENR bit 12 to 0 ( bit 12 control SPI1 )
#define SPI2_CLK_DI()		(RCC->APB1ENR &= ~(1<<14)) // turns APB1ENR bit 14 to 0 ( bit 14 control SPI2 )
#define SPI3_CLK_DI()		(RCC->APB1ENR &= ~(1<<15))

// Clock Disable Macros for USARTx/UARTx peripherals //
#define USART1_CLK_DI()		(RCC->APB2ENR &= ~(1<<4))
#define USART6_CLK_DI()		(RCC->APB2ENR &= ~(1<<5))
#define USART2_CLK_DI()		(RCC->APB1ENR &= ~(1<<17))
#define USART3_CLK_DI()		(RCC->APB1ENR &= ~(1<<18))
#define UART4_CLK_DI()		(RCC->APB1ENR &= ~(1<<19))
#define UART5_CLK_DI()		(RCC->APB1ENR &= ~(1<<20))

// Clock Disable Macro for SYSCFG peripheral //
#define SYSCFG_CLK_DI()		(RCC->APB2ENR &= ~(1<<14))

// Macros to Reset GPIOx peripherals //
/* First we set the register and then reset it */
/* "do ... while..." condition is a technique in "C" programming to execute multiple "C" statements using single "C" macro */
/* Don't put semi-colon ";" after while(0) because when we call this macro, there is a semi-colon at there */
#define GPIOA_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 0));	(RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 1));	(RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 2));	(RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 3));	(RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 4));	(RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 5));	(RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 6));	(RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 7));	(RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()	do{ (RCC->AHB1RSTR |= (1 << 8));	(RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

//Macros to Reset SPIx peripherals
#define SPI1_REG_RESET()	do { (RCC->APB2RSTR |= (1 << 12));	(RCC->APB2RSTR &= ~(1 << 12)); }while(0)	/* Sets SPI1 reset pin and then Resets */
#define SPI2_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 14));	(RCC->APB1RSTR &= ~(1 << 14)); }while(0)	/* Sets SPI2 reset pin and then Resets */
#define SPI3_REG_RESET()	do { (RCC->APB1RSTR |= (1 << 15));	(RCC->APB1RSTR &= ~(1 << 15)); }while(0)	/* Sets SPI3 reset pin and then Resets */


// Port Code for given GPIOx Base Address //
#define GPIO_BASEADDR_TO_CODE(x)	  ( (x == GPIOA) ? 0 :\
										(x == GPIOB) ? 1 :\
										(x == GPIOC) ? 2 :\
										(x == GPIOD) ? 3 :\
										(x == GPIOE) ? 4 :\
										(x == GPIOF) ? 5 :\
										(x == GPIOG) ? 6 :\
										(x == GPIOH) ? 7 :\
										(x == GPIOI) ? 8 :0 ) // this basically checks for wanted GPIOx and if not, checks other GPIOx ports
															  // and after that returns to 1-2-3-4... and so on for that GPIOx port
															  // this is called conditional or ternary operators used in 'C'


//IRQ(Interrupt Request) Numbers //
#define IRQ_NO_EXTI0 			6
#define IRQ_NO_EXTI1 			7
#define IRQ_NO_EXTI2 			8
#define IRQ_NO_EXTI3 			9
#define IRQ_NO_EXTI4 			10
#define IRQ_NO_EXTI9_5 			23
#define IRQ_NO_EXTI15_10 		40

//IRQ Priority Macros //
#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15

// General Macros
#define ENABLE 					1
#define DISABLE 				0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET
#define FLAG_SET				SET
#define FLAG_RESET				RESET
// Bit Position Definitions of SPI Peripheral //
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


#include "stm32f407_gpio_driver.h"
#include "stm32f407_spi_driver.h"

#endif /* INC_STM32F4XX_H_ */

/*
 * stm32f407_spi_driver.h
 *
 *  Created on: Apr 20, 2024
 *      Author: Mutlu
 */

#ifndef INC_STM32F407_SPI_DRIVER_H_
#define INC_STM32F407_SPI_DRIVER_H_

#include "stm32f4xx.h"

//	Configuration Structure for SPIx Peripheral //
typedef struct
{
	uint8_t SPI_DeviceMode;			// @SPI_DeviceMode
	uint8_t SPI_BusConfig;			// @SPI_BusConfig
	uint8_t SPI_SCLKSpeed;			// @SPI_SCLKSpeed
	uint8_t SPI_DFF;				// @SPI_DFF
	uint8_t SPI_CPOL;				// @SPI_CPOL
	uint8_t SPI_CPHA;				// @SPI_CPHA
	uint8_t SPI_SSM;				// @SPI_SSM

}SPI_Config_t;

// Handle Structure for SPIx Peripheral //
typedef struct
{
	SPI_RegDef_t	*pSPIx;			//This holds the base address of SPIx (x = 1,2,3)	//
	SPI_Config_t	SPIConfig;		//This hold SPI configuration settings	//
}SPI_Handle_t;

// @SPI_DeviceMode	//
#define SPI_MODE_MASTER 				1
#define SPI_MODE_SLAVE 					0

// @SPI_BusConfig	//
#define SPI_FULL_DUPLEX					1
#define SPI_HALF_DUPLEX					2
#define SPI_RXONLY_SIMPLEX				3

// @SPI_SCLKSpeed	//
#define SPI_SPEED_DIV2					0
#define SPI_SPEED_DIV4					1
#define SPI_SPEED_DIV8					2
#define SPI_SPEED_DIV16					3
#define SPI_SPEED_DIV32					4
#define SPI_SPEED_DIV64					5
#define SPI_SPEED_DIV128				6
#define SPI_SPEED_DIV256				7

// @SPI_DFF	//
#define SPI_DFF_8bits					0
#define SPI_DFF_16bits					1

// @SPI_CPOL //
#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

// @SPI_CPHA //
#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

// @SPI_SSM //
#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

// SPI related Status Flag Definitions //
#define SPI_TXE_FLAG					( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG					( 1 << SPI_SR_BSY)

///////////////////////////////////
// APIs supported by this driver //
///////////////////////////////////
//	Peripheral Clock Setup //
void SPI_ClkControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

// Init - DeInit //
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Flags and Other Control APIs//
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

// Data send and Receive //
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);
//IRQ Configuration and ISR Handling //
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F407_SPI_DRIVER_H_ */

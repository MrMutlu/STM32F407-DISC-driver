/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Apr 20, 2024
 *      Author: Mutlu
 */


#include "stm32f407_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_ClkControl
 *
 * @brief             -	This function enables or disables peripheral clock for SPIx peripheral
 *
 * @param[in]         -	Base address of the SPI peripheral ( SPI_RegDef_t *pSPIx )
 *
 * @param[in]         -	ENABLE or DISABLE macros ( uint8_t EnOrDi )
 *
 * @return            - None
 *
 * @Note              -	None

 */
void SPI_ClkControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_CLK_EN();
		} else if(pSPIx == SPI2)
		{
			SPI2_CLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_CLK_EN();
		}
	}else
	{
		if(pSPIx == SPI1)
		{
			SPI1_CLK_DI();
		} else if(pSPIx == SPI2)
		{
			SPI2_CLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_CLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  -	SPI_Init
 *
 * @brief             -	This function configures SPIx Peripheral
 *
 * @param[in]         -	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *
 * @return            - None
 *
 * @Note              -	None

 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Enable the peripheral Clock //
	SPI_ClkControl(pSPIHandle->pSPIx, ENABLE);
	//Configure Control Register //
	uint32_t tempreg = 0;

	//1. Configure Device Mode //
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure Bus Config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_FULL_DUPLEX)
	{
		//BIDIMODE bit should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_HALF_DUPLEX)
	{
		//BIDIMODE bit should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_RXONLY_SIMPLEX)
	{
		//BIDIMODE bit should be cleared and RXONLY bit should be set
		tempreg &= ~(1 << SPI_CR1_BIDIMODE); //BIDIMODE cleared
		tempreg |= (1 << SPI_CR1_RXONLY);  //RXONLY set

	}
	//3. Configure SPI serial clock speed ( baud rate )//
	tempreg |= (pSPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR);

	//4. Configure DFF //
	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// 5. Configure CPOL //
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 6. Configure CPHA //
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	pSPIHandle->pSPIx->CR1 = tempreg;


}

/*********************************************************************
 * @fn      		  -	SPI_DeInit
 *
 * @brief             -	This function resets the register of SPIx Peripheral
 *
 * @param[in]         -	Base address of the SPI peripheral ( SPI_RegDef_t *pSPIx )
 *
 * @return            -	None
 *
 * @Note              -	None

 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  -	SPI_GetFlagStatus
 *
 * @brief             -	This function resets the register of SPIx Peripheral
 *
 * @param[in]         -	Base address of the SPI peripheral ( SPI_RegDef_t *pSPIx )
 *
 * @param[in]         -	Flag Name ( uint32_t FlagName )
 *
 * @return            -	Flag Status
 *
 * @Note              -	None

 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  -	SPI_SendData
 *
 * @brief             -	This function is used for sending data
 *
 * @param[in]         -	Base address of the SPI peripheral ( SPI_RegDef_t *pSPIx )
 *
 * @param[in]         -	TX buffer ( uint8_t *pTxBuffer )
 *
 * @param[in]         -	Data length ( uint32_t len )
 *
 * @return            -	None
 *
 * @Note              -	This is blocking call

 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		// 1. wait until TXE is set.
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF bit in CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		{
			// 16 Bit DFF
			// 1. Load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 Bit DFF
			pSPIx->DR = *(pTxBuffer);
			len--;
			pTxBuffer++;

		}

	}
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function is used for receiving data
 *
 * @param[in]		  - Base address of the SPI peripheral ( SPI_RegDef_t *pSPIx )
 * @param[in]		  - RX buffer ( uint8_t *pRxBuffer )
 * @param[in]  		  - Data length ( uint32_t len )
 *
 * @return            - None
 *
 * @Note              - None

 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// 1. Wait until RXNE is set
		while (SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

		// 2. Check the DFF bit in CR1
		if (pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
		{
			// 16 bit DFF
			// 1. Load the data from the DR to RxBuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			len--;
			len--;	// len is decremented twice because 2 bytes of data has been sent
			(uint16_t*)pRxBuffer++;
		}else
		{
			// 8 bit DFF
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}
	}
}

/***********************************************************************************************************
 *  @fn				- SPI_PeripheralControl
 *
 *  @brief			- This function enables or disable a given SPIx peripheral
 *
 *  @param[in]		- Base address of the SPIx peripheral ( SPI_RegDef_t *pSPIx )
 *  @param[in]		- ENABLE or DISABLE macros ( uint8_t EnorDi )
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/***********************************************************************************************************
 *  @fn				- SPI_SSOEConfig
 *
 *  @brief			- This function makes NSS signal internally high and avoids MODF error
 *
 *  @param[in]		- Base address of the SPIx peripheral ( SPI_RegDef_t *pSPIx )
 *  @param[in]		- ENABLE or DISABLE macros (  uint8_t EnorDi )
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/***********************************************************************************************************
 *  @fn				- SPI_SSIConfig
 *
 *  @brief			- This function makes NSS signal internally high and avoids MODF error
 *
 *  @param[in]		- Base address of the SPIx peripheral ( SPI_RegDef_t *pSPIx )
 *  @param[in]		- ENABLE or DISABLE macros ( uint8_t EnorDi )
 *
 *  @return			- none
 *
 *  @Note			- none
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi)
{

}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority)
{

}

/*********************************************************************
 * @fn      		  -
 *
 * @brief             -
 *
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{

}

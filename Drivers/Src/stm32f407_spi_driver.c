/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Apr 20, 2024
 *      Author: Mutlu
 */


#include "stm32f407_spi_driver.h"


static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
 *  @fn				-	SPI_SendDataIT
 *
 *  @brief			-	This function returns state of Tx
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *  @param[in]		-	TX buffer ( uint8_t *pTxBuffer )
 *  @param[in]		-	Data length ( uint32_t len )
 *
 *  @return			-	state of Tx
 *
 *  @Note			-	none
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		//1. Save the Tx buffer address and len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->Txlen = len;

		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
}

/***********************************************************************************************************
 *  @fn				-	SPI_ReceiveDataIT
 *
 *  @brief			-	This function returns state of Rx
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *  @param[in]		-	TX buffer ( uint8_t *pRxBuffer )
 *  @param[in]		-	Data length ( uint32_t len )
 *
 *  @return			-	state of Rx
 *
 *  @Note			-	none
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		//1. Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->Rxlen = len;

		//2. Mark the SPI state as busy in transmission so that
		// no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		//4. Data transmission will be handled by the ISR code
	}

	return state;
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


/***********************************************************************************************************
 *  @fn				-	SPI_IRQInterruptConfig
 *
 *  @brief			-	This function configurates an exception
 *
 *  @param[in]		-	IRQ number of an exception ( uint8_t IRQNumber )
 *  @param[in]		-	ENABLE or DISABLE macros ( uint8_t EnorDi )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber < 32)
		{
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));

		}
	} else {
		if (IRQNumber < 32)
		{
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);

		} else if (IRQNumber >= 32 && IRQNumber < 64)
		{
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		} else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));

		}
	}
}

/***********************************************************************************************************
 *  @fn				-	SPI_IRQPriorityConfig
 *
 *  @brief			-	This function configurates a priority of an exception
 *
 *  @param[in]		-	IRQ number of an exception ( uint8_t IRQNumber )
 *  @param[in]		-	IRQ priority of an exception ( uint32_t IRQPriority )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_IRQPriorityConfig (uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out the IPR register
	uint8_t IPRx = IRQNumber / 4;
	uint8_t IPRx_section = IRQNumber % 4;

	// in each section of IRQ Priority register, first 4 bits are not implemented in STM32F407x.
	// no of priority bits implemented depends on vendor
	uint8_t shift_amount = (8 * IPRx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + IPRx) |= IRQPriority << shift_amount;
}

/***********************************************************************************************************
 *  @fn				-	SPI_IRQHandling
 *
 *  @brief			-	This function handles an exception for a given SPI pin
 *
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	// check for TXE
	temp1 = pSPIHandle->pSPIx->SR & ( 1<< SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1<< SPI_CR2_TXEIE);

	if (temp1 && temp2)
	{
		// handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// check for RXNE
	temp1 = pSPIHandle->pSPIx->SR & ( 1<< SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & ( 1<< SPI_CR2_RXNEIE);

	if (temp1 && temp2)
	{
		// handle RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// check for CRC => not implemented in this driver

	// check for OVR flag
	temp1 = pSPIHandle->pSPIx->SR & ( 1<< SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2)
	{
		// handle OVR
		spi_ovr_interrupt_handle(pSPIHandle);
	}
}

/***********************************************************************************************************
 *  @fn				-	spi_txe_interrupt_handle
 *
 *  @brief			-	This function is helper function of SPI TXE interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit DFF
		// load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTxBuffer);
		pSPIHandle->Txlen--;
		pSPIHandle->Txlen--;// Len is decremented twice because 2 bytes of data has been sent
		(uint16_t*)pSPIHandle->pTxBuffer++;
	} else
	{
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->Txlen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->Txlen)
	{
		// close the SPI communication when TxLen is zero
		// inform the application that TX is over
		// prevent interrupts with setting up of TXE flag
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/***********************************************************************************************************
 *  @fn				-	spi_rxne_interrupt_handle
 *
 *  @brief			-	This function is helper function of SPI RXNE interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit DFF
		// load the data into the DR
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->Rxlen--;
		pSPIHandle->Rxlen--;// Len is decremented twice because 2 bytes of data has been sent
		(uint16_t*) pSPIHandle->pRxBuffer++;
	} else
	{
		// 8 bit DFF
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR ;
		pSPIHandle->Rxlen--;
		pSPIHandle->pRxBuffer++;
	}

	if (!pSPIHandle->Rxlen)
	{
		// reception is complete
		// turn off the RXNEIE interrupt
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

/***********************************************************************************************************
 *  @fn				-	spi_ovr_interrupt_handle
 *
 *  @brief			-	This function is helper function of SPI OVR interrupt handle
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	// clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void) temp; // to prevent unused "temp" variable error

	// inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/***********************************************************************************************************
 *  @fn				-	SPI_ClearOVRFlag
 *
 *  @brief			-	This function clears the OVR flag
 *
 *  @param[in]		-	Base address of the SPI peripheral ( SPI_RegDef_t *pSPIx )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void) temp;
}

/***********************************************************************************************************
 *  @fn				-	SPI_CloseTransmission
 *
 *  @brief			-	This function closes SPI transmission
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->Txlen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/***********************************************************************************************************
 *  @fn				-	SPI_CloseReception
 *
 *  @brief			-	This function closes SPI reception
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->Rxlen = 0;
	pSPIHandle->RxState = SPI_READY;
}

/***********************************************************************************************************
 *  @fn				-	SPI_ApplicationEventCallback
 *
 *  @brief			-	This function overrides to Application Event Callback
 *
 *  @param[in]		-	Base address and user configuration of the SPI peripheral ( SPI_Handle_t *pSPIHandle )
 *  @param[in]		-	Event name ( uint8_t AppEv )
 *
 *  @return			-	none
 *
 *  @Note			-	none
 *
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
	// the user application may override this
}

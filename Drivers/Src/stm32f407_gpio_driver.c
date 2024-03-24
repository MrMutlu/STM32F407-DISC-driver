/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Mar 12, 2024
 *      Author: Mutlu
 */

#include "stm32f407_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             -	Initializes GPIO
 *
 * @param[in]         -	Handler for GPIO configuration and its' register ( GPIO_PinConfig )
 *
 * @return            - None
 *
 * @Note              - None

 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; // temporary register

	////// 1. Configure the mode of GPIO pin //////
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_AN) // ( since @GPIO_PinModes are integer , we use "<=" not "boolean" )
	{
		// non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << ( 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Pin Mode register use 2 bits, so we multiplied with 2
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear MODER registered temp values before set. (0x3) is binary "11" so negate and make them "00" in binary
		pGPIOHandle->pGPIOx->MODER |= temp; // set MODER registered temp values
	} else {
		// for interrupt
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FT){

			///// 1. Configure the Falling-Trigger Selection Register (FTSR) /////
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit //
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RT){

			///// 1. Configure the Rising-Trigger Selection Register (RTSR) /////
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit //
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RFT){

			///// 1. Configure both RTSR and FTSR /////
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		///// 2. Configure the GPIO port selection in SYSCFG_EXTICR /////
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4; //this calculates EXTICR[temp1] since we use EXTICR[4] as there is 4 registers for that.
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;	//this calculates which EXTIx will be used in EXTICR
		uint8_t PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // PortCode to store
		SYSCFG_CLK_EN(); // Enable the clock
		SYSCFG->EXTICR[temp1] =  PortCode << ( temp2 * 4); // Multiplying 4 because each EXTIx takes 4 bits

		///// 3. Enable EXTI interrupt delivery using IMR
		EXTI->IMR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	temp = 0; // reseting to use it again.

	////// 2. Configure the output speed of GPIO pin //////
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Speed Mode register use 2 bits, so we multiplied with 2
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear OSPEEDR registered temp values before set. (0x3) is binary "11" so negate and make them "00" in binary
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // save Output speed registered temp values
	temp = 0; // reseting to use it again.

	////// 3. Configure the pull-up / pull-down settings of GPIO pin //////
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPD << ( 2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // PU/PD register use 2 bits, so we multiplied with 2
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear PUPDR registered temp values before set. (0x3) is binary "11" so negate and make them "00" in binary
	pGPIOHandle->pGPIOx->PUPDR |= temp; // set PU/PD registered temp values
	temp = 0; // reseting to use it again.

	////// 4. Configure the Output Type ( Push-pull / Open-drain ) //////
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOutType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); // Output type register use 1 bits
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clear OTYPER registered temp values before set. (0x1) is binary "1" so negate and make them "0" in binary
	pGPIOHandle->pGPIOx->OTYPER |= temp; // set Output type registered temp values
	temp = 0; // reseting to use it again.

	////// 5. Configure Alternate Functionality //////
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT )
	{
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8; 	// Dividing to 8 we get 0 or 1, based on that we use register of LOW (0-7 bits) or HIGH (8-15 bits)
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;		// Result of temp2 will be used on shifting bits.
		if(temp1 == 0) pGPIOHandle->pGPIOx->AFRL &= ~(0xF << ( 4 * temp2)); // If port 0-7 selected, we get 0 so clear "AFRL".(we are shifting 4 bits and for negate 4 bits, we use "0xF" or "0000".)
		else pGPIOHandle->pGPIOx->AFRH &= ~(0xF << ( 4 * temp2)); // If port 8-15 selected, we get 1 so clear "AFRH".(we are shifting 4 bits and for negate 4 bits, we use "0xF" or "0000".)
		if(temp1 == 0) pGPIOHandle->pGPIOx->AFRL |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << ( 4 * temp2)); // If port 0-7 selected, we get 0 so set "AFRL".
		else pGPIOHandle->pGPIOx->AFRH |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << ( 4 * temp2)); // If port 8-15 selected, we get 1 so set "AFRH".
		/* Each register use 4 bits so shifting and multiplying with 4.  */
	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - De-initializes GPIO
 *
 * @param[in]         -Handler for GPIO configuration and its' register ( GPIO_PinConfig )
 *
 * @return            - None
 *
 * @Note              - None

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	} else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ClkControl
 *
 * @brief             - This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]         - Base address of the GPIO peripheral ( GPIO_RegDef_t *pGPIOx )
 * @param[in]         -	ENABLE or DISABLE macros ( uint8_t EnOrDi )
 *
 * @return            - None
 *
 * @Note              - None

 */
void GPIO_ClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		} else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_CLK_EN();
		}
	}else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_CLK_DI();
		} else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_CLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_CLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_CLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_CLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_CLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_CLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_CLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadPin
 *
 * @brief             - Read the current value of the pin ( 0 or 1 )
 *
 * @param[in]         -	Base address of the GPIO peripheral ( GPIO_RegDef_t *pGPIOx )
 * @param[in]         -	Pin number 0 to 15 ( uint8_t Pin )
 *
 * @return            - 0 or 1
 *
 * @Note              - None

 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin){
	uint8_t value;
/* Read the pin value using the GPIOx->IDR register */
/* Mask and shift operation for the bit corresponding to Pin No */
	value = (int8_t)((pGPIOx->IDR >> Pin) & 0x00000001);
/* ">> Pin" shifts the bit of the interested pin to the least significant bit position (LSB). This way, the value of the pin specified by Pin no is moved to the rightmost bit. */
/* "& 0x00000001" masks all bits except the least significant bit. This extracts only the value of the pin of interest, which can be either 0 or 1. */

	return value;

}

/*********************************************************************
 * @fn      		  - GPIO_ReadPort
 *
 * @brief             - Read the current values of the whole port
 *
 * @param[in]         - Base address of the GPIO peripheral ( GPIO_RegDef_t *pGPIOx )
 *
 * @return            - 0 or 1
 *
 * @Note              - None

 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
/* This value contains the state of all pins in the port */
	value = (int16_t) pGPIOx->IDR; // Don't need to shift and mask it since we are reading whole register

	return value;

}

/*********************************************************************
 * @fn      		  - GPIO_WritePin
 *
 * @brief             - Write the value ( 0 or 1 ) to wanted pin no.
 *
 * @param[in]         - Base address of the GPIO peripheral ( GPIO_RegDef_t *pGPIOx )
 * @param[in]         - Pin number 0 to 15 ( uint8_t Pin )
 * @param[in]         - Value to write either 0 or 1 ( uint8_t Val )
 *
 * @return            - None
 *
 * @Note              - None

 */
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t Val)
{
	if(Val == GPIO_PIN_SET)
	{
		// Write 1 to output data register at the bit field corresponding pin no
		pGPIOx->ODR |= ( 1 << Pin);
	}
	else
	{
		// Write 0 to output data register at the bit field corresponding pin no
		pGPIOx->ODR &= ~( 1 << Pin);

	}
}
/*********************************************************************
 * @fn      		  - GPIO_WritePort
 *
 * @brief             - Write the value ( 0 or 1 ) to wanted port.
 *
 * @param[in]         - Base address of the GPIO peripheral ( GPIO_RegDef_t *pGPIOx )
 * @param[in]         - Value to write either 0 or 1 ( uint8_t Val )
 *
 * @return            - None
 *
 * @Note              - None

 */
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t Val)
{
	// Write value to output data register at the bit field corresponding port
	pGPIOx->ODR = Val;
}

/*********************************************************************
 * @fn      		  - GPIO_TogglePin
 *
 * @brief             - Toggle the value to wanted pin no
 *
 * @param[in]         - Base address of the GPIO peripheral ( GPIO_RegDef_t *pGPIOx )
 * @param[in]         - Value to write either 0 or 1 ( uint8_t Val )
 *
 * @return            - None
 *
 * @Note              -	None

 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t Pin)
{
	pGPIOx->ODR  ^= (1 << Pin);
}

/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - Configuration setting for IRQ
 *
 * @param[in]         -	IRQ Number (uint8_t IRQNum)
 * @param[in]         -	Enable or Disable  (uint8_t EnOrDi)
 *
 * @return            - None
 *
 * @Note              -	ISERx and ICERx is used on processor's data sheet

 */
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(IRQNum <= 31)
		{
			// Program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNum );
		} else if(IRQNum < 31 && IRQNum < 64)
		{
			// Program ISER1 register
			*NVIC_ISER1 |= ( 1 << IRQNum % 32);

		} else if(IRQNum <= 64 && IRQNum < 96)
		{
			// Program ISER2 register
			*NVIC_ISER3 |= ( 1 << IRQNum % 64);

		}
	} else
	{
		if(IRQNum <=31 )
		{
			// Program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNum );

		} else if(IRQNum < 31 && IRQNum < 64)
		{
			// Program ICER1 register
			*NVIC_ICER1 |= ( 1 << IRQNum % 32);

		} else if(IRQNum <= 64 && IRQNum < 96)
		{
			// Program ICER2 register
			*NVIC_ICER2 |= ( 1 << IRQNum % 64 );

		}
	}
}
/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - Configures Priority setting for IRQ
 *
 * @param[in]         - Priority (uint8_t IRQPriority)
 * @param[in]         - IRQ Number (uint8_t IRQNum)
 *
 * @return            - None
 *
 * @Note              - IPRx and IPRxSection is used on processor's data sheet

 */
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority)
{
	// 1. Find out IPR Register
	uint8_t iprx = IRQNum / 4; 			// Find out which IPR register
	uint8_t iprxSection = IRQNum % 4;
	uint8_t shift_amount = ( 8 * iprxSection) + ( 8 - NO_PR_BITS_IMPLEMENTED); //each section is 8 bits so multiply with 8, and then shifting 4 more bits
																			   //since NO_PR_BITS_IMPLEMENTED is 4.
	*(NVIC_PR_BASE_ADDR + iprx) |= IRQPriority << shift_amount;

}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -	IRQHandler
 *
 * @param[in]         - Pin Number (uint8_t Pin)
 *
 * @return            -	None
 *
 * @Note              - None

 */
void GPIO_IRQHandling(uint8_t Pin)
{
	// Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & ( 1 << Pin))
	{
		// Clear
		EXTI->PR |= ( 1 << Pin);
	}
}

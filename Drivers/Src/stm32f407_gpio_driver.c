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
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// 1. Configure the mode of GPIO pin //
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_AN) // ( since @GPIO_PinModes are integer , we use "<=" not boolean )
	{

	} else {
		// for interrupt
	}


}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

}

/*********************************************************************
 * @fn      		  - GPIO_ClkControl
 *
 * @brief             - This function enables or disables peripheral clock for given GPIO port
 *
 * @param[in]         - Base address of the GPIO peripheral ( GPIO_RegDef_t *pGPIOx )
 * @param[in]         -	ENABLE or DISABLE macros ( uint8_t EnOrDi )
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

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
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin)
{

}
/*********************************************************************
 * @fn      		  - GPIO_ReadPort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{

}
/*********************************************************************
 * @fn      		  - GPIO_WritePin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t Val)
{

}
/*********************************************************************
 * @fn      		  - GPIO_WritePort
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t Val)
{

}
/*********************************************************************
 * @fn      		  - GPIO_TogglePin
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t Pin)
{

}

/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQConfig(uint8_t IRQNum, uint8_t IRQPriority, uint8_t EnOrDi)
{

}
/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -

 */
void GPIO_IRQHandling(uint8_t Pin)
{

}

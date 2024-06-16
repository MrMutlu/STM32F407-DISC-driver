/*
 * stm32f407_rcc_driver.h
 *
 *  Created on: May 12, 2024
 *      Author: Mutlu
 */

#ifndef INC_STM32F407_RCC_DRIVER_H_
#define INC_STM32F407_RCC_DRIVER_H_

#include "stm32f4xx.h"

uint32_t RCC_GetPLLOutputClock();
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F407_RCC_DRIVER_H_ */

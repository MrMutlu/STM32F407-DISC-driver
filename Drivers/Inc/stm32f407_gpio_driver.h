/*
 * stm32f407_gpio_driver.h
 *
 *  Created on: Mar 12, 2024
 *      Author: Mutlu
 */

#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include "stm32f4xx.h"

// Configuration Structure for a GPIO pin //
typedef struct {
	// uint8_t used because they are usually small bits (0-15)//
	uint8_t GPIO_PinNumber;				// @GPIO_PinNumbers
	uint8_t GPIO_PinMode;				// @GPIO_PinModes
	uint8_t GPIO_PinSpeed;				// @GPIO_PinSpeeds
	uint8_t GPIO_PinPUPD;				// @GPIO_PinPUPDs
	uint8_t GPIO_PinOutType;			// @GPIO_PinOutTypes
	uint8_t GPIO_PinAltFuncMode;

} GPIO_Pin_Config_t;

// Handle Structure for a GPIO pin //
typedef struct {
	GPIO_RegDef_t *pGPIOx;// This holds the base address of the GPIO port to which the pin belongs //
	GPIO_Pin_Config_t GPIO_PinConfig; // This holds GPIO pin configuration settings //

} GPIO_Handle_t;

// @GPIO_PinNumbers
// GPIOx pin Numbers //
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

// @GPIO_PinModes
// GPIO pin Port Modes ( GPIOx_MODER ) //
#define GPIO_MODE_IN 		0  	// Input mode
#define GPIO_MODE_OUT 		1	// Output mode
#define GPIO_MODE_ALT 		2	// Alternate Function mode
#define GPIO_MODE_AN 		3	// Analog mode
#define GPIO_MODE_IN_FT 	4	// Falling Edge Interrupt
#define GPIO_MODE_IN_RT 	5	// Rising Edge Interrupt
#define GPIO_MODE_IN_RFT 	6	// Falling and Rising Edge Interrupt

//@GPIO_PinOutTypes
// GPIO pin Output Type Modes ( GPIOx_OTYPER) //
#define GPIO_OUT_TYPE_PP 	0	// Push-Pull
#define GPIO_OUT_TYPE_OD	1 	// Open-Drain

//@GPIO_PinSpeeds
// GPIO pin Output Speed Modes ( GPIOx_OSPEEDR ) //
#define GPIO_SPD_LOW		0
#define GPIO_SPD_MED		1
#define GPIO_SPD_FAST		2
#define GPIO_SPD_HIGH		3

//@GPIO_PinPUPDs
// GPIO pin Pull-up - Pull-down Modes  (GPIOx_PUPDR ) //
#define GPIO_NO_PUPD		0
#define GPIO_PULLUP			1
#define GPIO_PULLDOWN		2

///////////////////////////////////
// APIs supported by this driver //
///////////////////////////////////
// Init and De-init //
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

// Peripheral Clock Setup //
void GPIO_ClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

// Data Readings and Writings //
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t Pin);
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t Pin, uint8_t Val);
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint16_t Val);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t Pin);

// IRQ Configuration and ISR handling //
void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t Pin);

#endif /* INC_STM32F407_GPIO_DRIVER_H_ */

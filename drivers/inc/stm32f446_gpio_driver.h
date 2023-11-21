/*
 * stm32f446_gpio_driver.h
 *
 *  Created on: Oct 23, 2023
 *      Author: sandi
 */

#ifndef INC_STM32F446_GPIO_DRIVER_H_
#define INC_STM32F446_GPIO_DRIVER_H_

#include "drivers/inc/stm32f446.h"

/*
 * GPIO Pin Number
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * GPIO Pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_RFT		6

/*
 * GPIO Pin Possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * GPIO Pin possible speed
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED		1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * GPIO Pin possible
 */
#define GPIO_PIN_NOPUPD 	0
#define GPIO_PIN_PU 		1
#define GPIO_PIN_PD 		2

/*
 * GPIO initialisation
 * Enable/Disable GPIO port clock
 * Read from a GPIO pin
 * Wtite to GPIO pin
 * Configure alternate functionality
 * Interrupt handling
 */

/*
 * GPIO Pin Configuration Structure
 * GPIO_PinNumber,GPIO_PinMode is user input
 * Others are calculated using pinNumber and then value is set in GPIO_RegDef_t(GPIO register)
 */
typedef struct{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_Pinconfig_t;

/*
 *GPIO pin handle configuration
 */
typedef struct{
	GPIO_RegDef_t *GPIOx;
	GPIO_Pinconfig_t GPIO_Pinconfig;
}GPIO_Handle_t;

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/*
 * GPIO Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F446_GPIO_DRIVER_H_ */

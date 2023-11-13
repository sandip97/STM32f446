/*
 * stm32f446.h
 *
 *  Created on: Oct 23, 2023
 *      Author: sandi
 */

#ifndef INC_STM32F446_H_
#define INC_STM32F446_H_

#include<stdint.h>

/*
 * Generic macros
 */
#define __vo volatile
#define ENABLE 			1
#define DISABLE 		0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

/*
 * Base address of Flash and SRAM memoroes
 */

#define FLASH_BASEADDR 			0x08000000U	/*Base address for main memory*/
#define SRAM1_BASEADDR 			0x20000000U /*Base address for SRAM1 memory 112KB*/
#define SRAM2_BASEADDR 			0x2001C000U
#define ROM 					0x1FFF0000U
#define SRAM 					SRAM_BASEADDR

/*
 * Base address of Peripheral bus
 */
#define PERIPH_BASE 			0x40000000U
#define APB1PERIPH_BASE 		PERIPH_BASE
#define APB2PERIPH_BASE 		0x40010000U
#define AHB1PERIPH_BASE 		0x40020000U
#define AHB2PERIPH_BASE 		0x50000000U
#define AHB3PERIPH_BASE 		0x60000000U

/*
 * Base address of Peripheral on AHB1 bus
 */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASE + 0x0000)//0x40020000U
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE + 0x0400)//0x40020400U
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE + 0x0800)//0x40020800U
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE + 0x0C00)//0x40020C00U
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE + 0x1000)//0x40021000U
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE + 0x1400)//0x40021400U
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE + 0x1800)//0x40021800U
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE + 0x1C00)//0x40021C00U
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x3800)
/*
 * Base address of Peripheral on APB1 bus
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE + 0x5C00)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE + 0x3C00)
#define USART2_BASEADDR			(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE + 0x5000)

/*
 * Base address of Peripheral on APB2 bus
 */
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)
#define USART1_BASEADDR			(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE + 0x1400)
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x3C00)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE + 0x3800)

/*******************Peripheral register definintion structures******************/

typedef struct{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDER;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFRL;
	__vo uint32_t AFRH;

}GPIO_RegDef_t;


/*
 * Peripheral address definition
 */
#define GPIOA (GPIO_RegDef_t *)GPIOA_BASEADDR
#define GPIOB (GPIO_RegDef_t *)GPIOB_BASEADDR
#define GPIOC (GPIO_RegDef_t *)GPIOC_BASEADDR
#define GPIOD (GPIO_RegDef_t *)GPIOD_BASEADDR
#define GPIOE (GPIO_RegDef_t *)GPIOE_BASEADDR
#define GPIOF (GPIO_RegDef_t *)GPIOF_BASEADDR
#define GPIOG (GPIO_RegDef_t *)GPIOG_BASEADDR
#define GPIOH (GPIO_RegDef_t *)GPIOH_BASEADDR

/*
 * GPIO reset Register
 */
#define GPIOA_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<0));(RCC->AHB1RSTR &=~(1<<0));}while(0)
#define GPIOB_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<1));(RCC->AHB1RSTR &=~(1<<1));}while(0)
#define GPIOC_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<2));(RCC->AHB1RSTR &=~(1<<2));}while(0)
#define GPIOD_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<3));(RCC->AHB1RSTR &=~(1<<3));}while(0)
#define GPIOE_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<4));(RCC->AHB1RSTR &=~(1<<4));}while(0)
#define GPIOF_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<5));(RCC->AHB1RSTR &=~(1<<5));}while(0)
#define GPIOG_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<6));(RCC->AHB1RSTR &=~(1<<6));}while(0)
#define GPIOH_REG_RESET()			do{(RCC->AHB1RSTR |=(1<<7));(RCC->AHB1RSTR &=~(1<<7));}while(0)


/*******************Peripheral register definintion structures******************/

typedef struct{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t 	  RESSERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t   	  RESSERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t      RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t 	  RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t 	  RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
}RCC_RegDef_t;

#define RCC ((RCC_RegDef_t *)RCC_BASEADDR)

/*
 * GPIO Clock Enable Macro
 */
#define GPIOA_PCLK_EN() (RCC->AHB1ENR|=(1<<0))
#define GPIOB_PCLK_EN() (RCC->AHB1ENR|=(1<<1))
#define GPIOC_PCLK_EN() (RCC->AHB1ENR|=(1<<2))
#define GPIOD_PCLK_EN() (RCC->AHB1ENR|=(1<<3))
#define GPIOE_PCLK_EN() (RCC->AHB1ENR|=(1<<4))
#define GPIOF_PCLK_EN() (RCC->AHB1ENR|=(1<<5))
#define GPIOG_PCLK_EN() (RCC->AHB1ENR|=(1<<6))
#define GPIOH_PCLK_EN() (RCC->AHB1ENR|=(1<<7))

/*
 * GPIO Clock Disable Macro
 */
#define GPIOA_PCLK_DI() (RCC->AHB1ENR &=~(1<<0))
#define GPIOB_PCLK_DI() (RCC->AHB1ENR &=~(1<<1))
#define GPIOC_PCLK_DI() (RCC->AHB1ENR &=~(1<<2))
#define GPIOD_PCLK_DI() (RCC->AHB1ENR &=~(1<<3))
#define GPIOE_PCLK_DI() (RCC->AHB1ENR &=~(1<<4))
#define GPIOF_PCLK_DI() (RCC->AHB1ENR &=~(1<<5))
#define GPIOG_PCLK_DI() (RCC->AHB1ENR &=~(1<<6))
#define GPIOH_PCLK_DI() (RCC->AHB1ENR &=~(1<<7))

/*
 * I2Cx Peripherals clock enable
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1<<21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1<<22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1<<23))

/*
 * I2Cx Peripherals clock disable
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= (1<<21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= (1<<22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= (1<<23))

/*
 * SPIx Peripherals clock enable
 */
#define SPI1_PCLK_EN() (RCC->APB2ENR |= (1<<12))
#define SPI2_PCLK_EN() (RCC->APB1ENR |= (1<<14))
#define SPI3_PCLK_EN() (RCC->APB1ENR |= (1<<15))
#define SPI4_PCLK_EN() (RCC->APB2ENR |= (1<<13))

/*
 * SPIx Peripherals clock disable
 */
#define SPI1_PCLK_DI() (RCC->APB2ENR &= (1<<12))
#define SPI2_PCLK_DI() (RCC->APB1ENR &= (1<<14))
#define SPI3_PCLK_DI() (RCC->APB1ENR &= (1<<15))
#define SPI4_PCLK_DI() (RCC->APB2ENR &= (1<<13))

/*
 * USARTx Peripherals clock enable
 */
#define USART1_PCLK_EN() (RCC->APB2ENR |= (1<<4))
#define USART2_PCLK_EN() (RCC->APB1ENR |= (1<<17))
#define USART3_PCLK_EN() (RCC->APB1ENR |= (1<<18))
#define USART6_PCLK_EN() (RCC->APB2ENR |= (1<<5))

/*
 * USARTx Peripherals clock disable
 */
#define USART1_PCLK_DI() (RCC->APB2ENR &= (1<<4))
#define USART2_PCLK_DI() (RCC->APB1ENR &= (1<<17))
#define USART3_PCLK_DI() (RCC->APB1ENR &= (1<<18))
#define USART6_PCLK_DI() (RCC->APB2ENR &= (1<<5))

/*
 * SYSCFG Peripherals clock enable
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1<<14))

/*
 * SYSCFG Peripherals clock disable
 */
#define SYSCFG_PCLK_DI() (RCC->APB2ENR &= (1<<14))






#endif /* INC_STM32F446_H_ */

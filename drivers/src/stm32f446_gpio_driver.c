/*
 * stm32f446_gpio_driver.c
 *
 *  Created on: Oct 23, 2023
 *      Author: sandi
 */

#include "stm32f446_gpio_driver.h"

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi){
	if(EnorDi==ENABLE){
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_EN();
		}else if(pGPIOx==GPIOB){
			GPIOB_PCLK_EN();
		}else if(pGPIOx==GPIOC){
			GPIOC_PCLK_EN();
		}else if(pGPIOx==GPIOD){
			GPIOD_PCLK_EN();
		}else if(pGPIOx==GPIOE){
			GPIOE_PCLK_EN();
		}else if(pGPIOx==GPIOF){
			GPIOF_PCLK_EN();
		}else if(pGPIOx==GPIOG){
			GPIOG_PCLK_EN();
		}else if(pGPIOx==GPIOH){
			GPIOH_PCLK_EN();
		}else{

		}
	}
	else{
		if(pGPIOx==GPIOA){
			GPIOA_PCLK_DI();
		}else if(pGPIOx==GPIOB){
			GPIOB_PCLK_DI();
		}else if(pGPIOx==GPIOC){
			GPIOC_PCLK_DI();
		}else if(pGPIOx==GPIOD){
			GPIOD_PCLK_DI();
		}else if(pGPIOx==GPIOE){
			GPIOE_PCLK_DI();
		}else if(pGPIOx==GPIOF){
			GPIOF_PCLK_DI();
		}else if(pGPIOx==GPIOG){
			GPIOG_PCLK_DI();
		}else if(pGPIOx==GPIOH){
			GPIOH_PCLK_DI();
		}else{

		}
	}
}


/*
 * GPIO Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	//Configure the mode of GPIO Pin
	int temp=0;
	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//Non interrupt modes
		temp=pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
		pGPIOHandle->GPIOx->MODER &=~(0x3 << (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
		pGPIOHandle->GPIOx->MODER |= temp;
	}else{

	}
	temp=0;

	//Configure the Pin speed
	temp=pGPIOHandle->GPIO_Pinconfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
	pGPIOHandle->GPIOx->OSPEEDER &=~(0x3 << (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGPIOHandle->GPIOx->OSPEEDER |= temp;
	temp=0;

	//Configure the GPIO Pull up/Pull down resistor
	temp=pGPIOHandle->GPIO_Pinconfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
	pGPIOHandle->GPIOx->PUPDR &=~(0x3 << (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGPIOHandle->GPIOx->PUPDR |= temp;
	temp=0;

	//configure the Pin output type
	temp=pGPIOHandle->GPIO_Pinconfig.GPIO_PinOPType << (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber);
	pGPIOHandle->GPIOx->OTYPER &=~(0x1 << (pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber));
	pGPIOHandle->GPIOx->OTYPER |= temp;
	temp=0;

	//Configure the alternate functionality
	if(pGPIOHandle->GPIO_Pinconfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1=0,temp2=0;
		temp1=pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber/8;
		temp2=pGPIOHandle->GPIO_Pinconfig.GPIO_PinNumber%8;
		if(temp1==0){
			pGPIOHandle->GPIOx->AFRL &=~(0xFF << (4*temp2));
			pGPIOHandle->GPIOx->AFRL=pGPIOHandle->GPIO_Pinconfig.GPIO_PinAltFunMode<<(4*temp2);
		}else if(temp1==1){
			pGPIOHandle->GPIOx->AFRH &=~(0xFF << (4*temp2));
			pGPIOHandle->GPIOx->AFRH=pGPIOHandle->GPIO_Pinconfig.GPIO_PinAltFunMode<<(4*temp2);
		}else{
			//Do Nothing
		}
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx==GPIOA){
		GPIOA_REG_RESET();
	}else if(pGPIOx==GPIOB){
		GPIOB_REG_RESET();
	}else if(pGPIOx==GPIOC){
		GPIOC_REG_RESET();
	}else if(pGPIOx==GPIOD){
		GPIOD_REG_RESET();
	}else if(pGPIOx==GPIOE){
		GPIOE_REG_RESET();
	}else if(pGPIOx==GPIOF){
		GPIOF_REG_RESET();
	}else if(pGPIOx==GPIOG){
		GPIOG_REG_RESET();
	}else if(pGPIOx==GPIOH){
		GPIOH_REG_RESET();
	}else{

	}
}

/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	uint8_t value;
	value=(uint8_t)((pGPIOx->IDR>>PinNumber)&0x00000001);
	return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value;
		value=(uint16_t)pGPIOx->IDR;
		return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber,uint8_t value){
	if(value==GPIO_PIN_SET){
		//write 1 to the output data register at the bit position corresponding to pinNumber
		pGPIOx->ODR |= (1 << PinNumber);
	}else{
		//Write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint8_t value){
	pGPIOx->ODR |= value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx,uint8_t PinNumber){
	pGPIOx->ODR ^= (1<<PinNumber);
}

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);

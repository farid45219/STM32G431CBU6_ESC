
#include "stm32g431xx.h"
#include "cdefs.h"
#include "led.h"

void LED_Init(void){
	//Enable clock to GPIOC->PC6
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	//Output low GPIOC->PC6
	GPIOC->ODR   &=~GPIO_ODR_OD6;
	//Clear to set input
	GPIOC->MODER &=~GPIO_MODER_MODE6_Msk;
	//Set Bit0 to general purpose output
	GPIOC->MODER |= GPIO_MODER_MODE6_0;
}

void LED_Set_State(uint8_t val){
	if(val == ON){
		//Set output high
		GPIOC->ODR   |= GPIO_ODR_OD6;
	}
	else if(val == OFF){
		//Set output low
		GPIOC->ODR   &=~GPIO_ODR_OD6;
	}
	
}


	
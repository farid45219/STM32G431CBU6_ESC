

#include "stm32g431xx.h"
#include "driver.h"
#include "cdefs.h"


void Driver_GPIO_Init(void){
	
	//UH->PA8 , UL->PC13
	//VH->PA9 , VL->PA12
	//WH->PA10, WL->PB15
	
	//Enable clock for respective PORTS
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;
	
	//Config UH->PA8
	GPIOA->ODR   &=~GPIO_ODR_OD8;
	GPIOA->MODER &=~GPIO_MODER_MODE8;
	GPIOA->MODER |= GPIO_MODER_MODE8_0;
	
	//Config UL->PC13
	GPIOC->ODR   &=~GPIO_ODR_OD13;
	GPIOC->MODER &=~GPIO_MODER_MODE13;
	GPIOC->MODER |= GPIO_MODER_MODE13_0;
	
	//Config VH->PA9
	GPIOA->ODR   &=~GPIO_ODR_OD9;
	GPIOA->MODER &=~GPIO_MODER_MODE9;
	GPIOA->MODER |= GPIO_MODER_MODE9_0;
	
	//Config VL->PA12
	GPIOA->ODR   &=~GPIO_ODR_OD12;
	GPIOA->MODER &=~GPIO_MODER_MODE12;
	GPIOA->MODER |= GPIO_MODER_MODE12_0;
	
	//Config WH->PA10
	GPIOA->ODR   &=~GPIO_ODR_OD10;
	GPIOA->MODER &=~GPIO_MODER_MODE10;
	GPIOA->MODER |= GPIO_MODER_MODE10_0;
	
	//Config WL->PB15
	GPIOB->ODR   &=~GPIO_ODR_OD15;
	GPIOB->MODER &=~GPIO_MODER_MODE15;
	GPIOB->MODER |= GPIO_MODER_MODE15_0;
}


void Driver_Set_UH(uint8_t val){
	if(val == ON){
		//Set output high
		GPIOA->ODR   |= GPIO_ODR_OD8;
	}
	else if(val == OFF){
		//Set output low
		GPIOA->ODR   &=~GPIO_ODR_OD8;
	}
}

void Driver_Set_UL(uint8_t val){
	if(val == ON){
		//Set output high
		GPIOC->ODR   |= GPIO_ODR_OD13;
	}
	else if(val == OFF){
		//Set output low
		GPIOC->ODR   &=~GPIO_ODR_OD13;
	}
}

void Driver_Set_VH(uint8_t val){
	if(val == ON){
		//Set output high
		GPIOA->ODR   |= GPIO_ODR_OD9;
	}
	else if(val == OFF){
		//Set output low
		GPIOA->ODR   &=~GPIO_ODR_OD9;
	}
}

void Driver_Set_VL(uint8_t val){
	if(val == ON){
		//Set output high
		GPIOA->ODR   |= GPIO_ODR_OD12;
	}
	else if(val == OFF){
		//Set output low
		GPIOA->ODR   &=~GPIO_ODR_OD12;
	}
}

void Driver_Set_WH(uint8_t val){
	if(val == ON){
		//Set output high
		GPIOA->ODR   |= GPIO_ODR_OD10;
	}
	else if(val == OFF){
		//Set output low
		GPIOA->ODR   &=~GPIO_ODR_OD10;
	}
}

void Driver_Set_WL(uint8_t val){
	if(val == ON){
		//Set output high
		GPIOB->ODR   |= GPIO_ODR_OD15;
	}
	else if(val == OFF){
		//Set output low
		GPIOB->ODR   &=~GPIO_ODR_OD15;
	}
}


void Driver_Rough_Delay(void){
	for(uint32_t i=0;i<20;i++){
		__NOP();
	}
}

void Driver_Set_U(uint8_t val){
	if( val == LOGIC_HIGH ){
		Driver_Set_UL(OFF);
		Driver_Rough_Delay();
		Driver_Set_UH(ON);
	}
	else{
		Driver_Set_UH(OFF);
		Driver_Rough_Delay();
		Driver_Set_UL(ON);
	}
}

void Driver_Set_V(uint8_t val){
	if( val == LOGIC_HIGH ){
		Driver_Set_VL(OFF);
		Driver_Rough_Delay();
		Driver_Set_VH(ON);
	}
	else{
		Driver_Set_VH(OFF);
		Driver_Rough_Delay();
		Driver_Set_VL(ON);
	}
}

void Driver_Set_W(uint8_t val){
	if( val == LOGIC_HIGH ){
		Driver_Set_WL(OFF);
		Driver_Rough_Delay();
		Driver_Set_WH(ON);
	}
	else{
		Driver_Set_WH(OFF);
		Driver_Rough_Delay();
		Driver_Set_WL(ON);
	}
}



void Driver_Set_Phases(uint8_t u, uint8_t v, uint8_t w){
	Driver_Set_U(u);
	Driver_Set_V(v);
	Driver_Set_W(w);
}

void Driver_Set_Phase_Values(uint8_t val){
	uint8_t u,v,w;
	if( val & 0b100){
		u = LOGIC_HIGH;
	}
	else{
		u = LOGIC_LOW;
	}
	
	if( val & 0b010){
		v = LOGIC_HIGH;
	}
	else{
		v = LOGIC_LOW;
	}
	
	if( val & 0b001){
		w = LOGIC_HIGH;
	}
	else{
		w = LOGIC_LOW;
	}
		
		
	Driver_Set_Phases(u, v, w);
}
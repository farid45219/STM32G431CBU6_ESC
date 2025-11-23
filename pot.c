

#include "stm32g431xx.h"
#include "cdefs.h"
#include "pot.h"


void Pot_GPIO_Init(void){
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODE12;
}

void Pot_ADC_Init(void){
	//Pot connected to PB12->ADC1_IN11
	RCC->AHB2ENR |= RCC_AHB2ENR_ADC12EN;
	ADC1->CR &=~ ADC_CR_DEEPPWD;
	ADC1->CR |=  ADC_CR_ADVREGEN;
	//Wait for Vref to stabilize
	for(uint32_t i=0;i<500;i++){
		__NOP();
	}
	//ADC1->ISR |= ADC_ISR_ADRDY;
	//Run calibration before enable
	//ADC1->CR  |= ADC_CR_ADCAL;
	//while ( ADC1->CR & ADC_CR_ADCAL ){
		//add timeout
	//}
	//adc enable
  ADC12_COMMON->CCR &=~ ADC_CCR_CKMODE;
	ADC12_COMMON->CCR |= 3 << ADC_CCR_CKMODE_Pos;
	
	
	ADC1->CR  |= ADC_CR_ADEN;
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0){
		//add timeout
	}
	//channel 11, single conversion
	ADC1->SQR1 = 11<<6;
}


void Pot_Init(void){
	Pot_GPIO_Init();
	Pot_ADC_Init();
}



uint16_t Pot_ADC_Read(void){
	ADC1->CR |= ADC_CR_ADSTART;
	while( (ADC1->ISR & ADC_ISR_EOC) == 0){
		
	}
	return ADC1->DR;
}


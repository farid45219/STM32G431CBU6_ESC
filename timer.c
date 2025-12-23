

#include "stm32g431xx.h"
#include "driver_spwm.h"
#include "cdefs.h"
#include "timer.h"


void Timer_Init(uint32_t UpdateRateHz){
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;
	TIM4->PSC      = 0;
	TIM4->ARR      = (16000000/UpdateRateHz);
	TIM4->DIER    |= TIM_DIER_UIE;
	TIM4->CR1     |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 0);
}

void TIM4_IRQHandler(void){
	if(TIM4->SR & TIM_SR_UIF){
    Driver_SPWM_Update();
		TIM4->SR &=~ TIM_SR_UIF;
	}
}
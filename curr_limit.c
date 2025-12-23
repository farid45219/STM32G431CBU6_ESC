

#include "stm32g431xx.h"
#include "curr_limit.h"


void CurrLimit_OPAMP_Init(void){
	//Enable OPAMP clock
  RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
  
  //OPAMP1: PGA mode, Gain=16
	//VINM0(PA3) and VINP0(PA1) are inputs
  OPAMP1->CSR = OPAMP_CSR_OPAMPxEN |OPAMP_CSR_PGGAIN_3 ;
	
  //OPAMP2: PGA mode, Gain=16
	//VINM0(PA3) and VINP0(PA1) are inputs
  OPAMP2->CSR = OPAMP_CSR_OPAMPxEN |OPAMP_CSR_PGGAIN_3 ;
	
  //OPAMP3: PGA mode, Gain=16
	//VINM0(PA3) and VINP0(PA1) are inputs
  OPAMP3->CSR = OPAMP_CSR_OPAMPxEN |OPAMP_CSR_PGGAIN_3 ;
	
} 


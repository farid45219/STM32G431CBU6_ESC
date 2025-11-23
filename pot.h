

#ifndef  _POT_H_
#define  _POT_H_

#include "stm32g431xx.h"

void     Pot_GPIO_Init(void);
void     Pot_ADC_Init(void);
void     Pot_Init(void);
uint16_t Pot_ADC_Read(void);



#endif



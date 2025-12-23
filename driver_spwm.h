

#ifndef _DRIVER_SPWM_H_
#define _DRIVER_SPWM_H_

#include "stm32g431xx.h"



#define  DRIVER_SPWM_INTERRUPT_RATE    10000
#define  DRIVER_SPWM_CARRIER_FREQ_HZ   30000
#define  DRIVER_SPWM_POWER_DIV_FACT    4
#define  DRIVER_SPWM_SVPWM_STEP_SIZE   1



void     Driver_SPWM_GPIO_Init(void);
void     Driver_SPWM_Timer_Init(void);
void     Driver_SPWM_Sine_Table_Init(void);
void     Driver_SPWM_Set_Val(uint16_t u, uint16_t v, uint16_t w);
void     Driver_SPWM_Update(void);

#endif



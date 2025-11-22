

#ifndef _DRIVER_H_
#define _DRIVER_H_

#include "stm32g431xx.h"

void     Driver_GPIO_Init(void);

void     Driver_Set_UH(uint8_t val);
void     Driver_Set_UL(uint8_t val);
void     Driver_Set_VH(uint8_t val);
void     Driver_Set_VL(uint8_t val);
void     Driver_Set_WH(uint8_t val);
void     Driver_Set_WL(uint8_t val);

void     Driver_Rough_Delay(void);
void     Driver_Set_U(uint8_t val);
void     Driver_Set_V(uint8_t val);
void     Driver_Set_W(uint8_t val);

void     Driver_Set_Phases(uint8_t u, uint8_t v, uint8_t w);
void     Driver_Set_Phase_Values(uint8_t val);

#endif



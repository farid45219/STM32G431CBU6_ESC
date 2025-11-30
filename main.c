
#include "stm32g431xx.h"
#include "driver_spwm.h"
#include "driver.h"
#include "debug.h"
#include "led.h"
#include "pot.h"


int main(void){
	
	LED_Init();
	Pot_Init();
	Debug_Init(38400);
	//Driver_GPIO_Init();
	Driver_SPWM_GPIO_Init();
	
	for(uint32_t i=0;i<500000;i++){
			__NOP();
	}
	
	
	Driver_SPWM_Timer_Init();
	Driver_SPWM_Sine_Table_Init();
	Driver_SPWM_Set_Val(0, 0, 0);
	
	uint16_t angle = 0;
	uint16_t u, v, w;
	
	while(1){
		
		
		u = angle + 0;
		v = angle + 120;
		w = angle + 240;
		
		if(v > 360){
			v -= 360;
		}
    
    if(w > 360){
			w -= 360;
		}
		
		angle++;
		if(angle > 360){
			angle = 0;
		}
		
		Driver_SPWM_Set_Val(u, v, w);
		
		for(uint32_t i=0;i<10000;i++){
			__NOP();
		}
		
		
	}
	
}



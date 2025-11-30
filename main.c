
#include "stm32g431xx.h"
#include "driver_spwm.h"
#include "driver.h"
#include "debug.h"
#include "led.h"
#include "pot.h"


uint8_t phase_index = 0;
uint8_t phase_val[6] = {
	0b001,
	0b011,
	0b010,
	0b110,
	0b100,
	0b101
};

uint16_t delay = 4095, buffer = 4095;

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
		
		//Driver_SPWM_Set_Val(u, v, w);
		
		for(uint32_t i=0;i<100;i++){
			__NOP();
		}
		
		
	}
	
}



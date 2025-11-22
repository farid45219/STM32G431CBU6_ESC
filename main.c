
#include "stm32g431xx.h"
#include "driver.h"
#include "led.h"


uint8_t phase_index = 0;
uint8_t phase_val[6] = {
	0b001,
	0b011,
	0b010,
	0b110,
	0b100,
	0b101
};



int main(void){
	
	LED_Init();
	Driver_GPIO_Init();
	for(uint32_t i=0;i<500000;i++){
			__NOP();
	}
	
	while(1){
		
		LED_Set_State(1);
		for(uint32_t i=0;i<500;i++){
			__NOP();
		}
		LED_Set_State(0);
		Driver_Set_Phase_Values(phase_val[phase_index]);
		phase_index++;
		if(phase_index >= 6){
			phase_index = 0;
		}
		
	}
	
}



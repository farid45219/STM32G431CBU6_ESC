
#include "stm32g431xx.h"
#include "driver_spwm.h"
#include "driver.h"
#include "debug.h"
#include "timer.h"
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
	
	Timer_Init(DRIVER_SPWM_INTERRUPT_RATE);
	
	
	while(1){
		
		
		
	}
	
}




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
	Driver_GPIO_Init();
	//Driver_SPWM_GPIO_Init();
	for(uint32_t i=0;i<500000;i++){
			__NOP();
	}
	
	while(1){
		
		//LED_Set_State(1);
		//LED_Set_State(0);
		
		//Debug_Tx_Parameter_SP("Target", Pot_ADC_Read());
		//Debug_Tx_Parameter_NL("Current", buffer);
		
		Driver_Set_Phase_Values(phase_val[phase_index]);
		phase_index++;
		if(phase_index >= 6){
			phase_index = 0;
			delay = Pot_ADC_Read();
		}
		
		if(buffer < delay){
			buffer+=5;
		}
		else{
			buffer-=5;
		}
		
		if(buffer < 500){
			buffer = 500;
		}
		
		for(uint32_t i=0;i<buffer;i++){
			__NOP();
		}
		
		
	}
	
}



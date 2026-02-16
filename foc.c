
#include "stm32g431xx.h"
#include "math.h"
#include "foc.h"

typedef struct foc_t{
	float Ia;
	float Ib;
	float Ic;
	float Ialpha;
	float Ibeta;
	float Id;
	float Iq;
}foc_t;


foc_t FOC;


void FOC_Forward_Clark_Transform(void){
	FOC.Ialpha = FOC.Ia;
	FOC.Ibeta = (1/sqrt(3)) * (FOC.Ia + 2* FOC.Ib);
}

void FOC_Forward_Park_Transform(float theta){
	float rad;
	rad = 0.017453 * theta;
	FOC.Id = FOC.Ialpha*cos(rad) + FOC.Ibeta*sin(rad);
	
}




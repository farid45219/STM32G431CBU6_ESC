
#include "stm32g431xx.h"
#include "math.h"
#include "foc.h"


#define  INV_SQRT3        (0.577350269f)
#define  DRIVING_VOLTAGE  (12.0f)
#define  ANTI_WINDUP_VMAX (DRIVING_VOLTAGE*INV_SQRT3)


typedef struct foc_pi_t{
	//target Id current
	float IdTarget;
	//target Iq current
	float IqTarget;
	//d error
	float Ed;
	//q error
	float Eq;
	//Integral of d error
	float IEd;
	//Integral of q error
	float IEq;
}foc_pi_t;

typedef struct foc_t{
	float    Ia;
	float    Ib;
	float    Ic;
	float    Ialpha;
	float    Ibeta;
	float    Id;
	float    Iq;
	float    Vd;
	float    Vq;
	float    Vmag;
	float    Valpha;
	float    Vbeta;
	float    Va;
	float    Vb;
	float    Vc;
	foc_pi_t PI;
}foc_t;


foc_t FOC;


void FOC_Forward_Clarke_Transform(void){
	FOC.Ialpha = FOC.Ia;
	FOC.Ibeta  = INV_SQRT3 * (FOC.Ia + 2* FOC.Ib);
}

void FOC_Forward_Park_Transform(float theta_rad){
	float sin_rad, cos_rad;
	sin_rad = sin(theta_rad);
	cos_rad = cos(theta_rad);
	FOC.Id =  FOC.Ialpha * cos_rad + FOC.Ibeta * sin_rad;
	FOC.Iq = -FOC.Ialpha * sin_rad + FOC.Ibeta * cos_rad;
}

void FOC_Run_PI_Controller(void){
	float Scale;
	FOC.PI.Ed   = FOC.PI.IdTarget - FOC.Id;
	FOC.PI.Eq   = FOC.PI.IqTarget - FOC.Iq;
	FOC.Vd = FOC.PI.Ed * FOC_D_KP + FOC.PI.IEd * FOC_D_KI;
  FOC.Vq = FOC.PI.Eq * FOC_Q_KP + FOC.PI.IEq * FOC_Q_KI;
	FOC.Vmag = sqrtf(FOC.Vd * FOC.Vd + FOC.Vq * FOC.Vq);
	
	if (FOC.Vmag <= ANTI_WINDUP_VMAX){
    FOC.PI.IEd += FOC.PI.Ed;
	  FOC.PI.IEq += FOC.PI.Eq;
	}
	
	FOC.Vd = FOC.PI.Ed * FOC_D_KP + FOC.PI.IEd * FOC_D_KI;
  FOC.Vq = FOC.PI.Eq * FOC_Q_KP + FOC.PI.IEq * FOC_Q_KI;
	
	FOC.Vmag = sqrtf(FOC.Vd * FOC.Vd + FOC.Vq * FOC.Vq);
	
  if (FOC.Vmag > ANTI_WINDUP_VMAX){
    Scale = ANTI_WINDUP_VMAX / FOC.Vmag;
    FOC.Vd *= Scale;
    FOC.Vq *= Scale;
	}
}

void FOC_Inverse_Park_Transform(float theta_rad){
	float sin_rad, cos_rad;
	sin_rad = sin(theta_rad);
	cos_rad = cos(theta_rad);
	FOC.Valpha = FOC.Vd * cos_rad - FOC.Vq * sin_rad;
	FOC.Vbeta  = FOC.Vd * sin_rad + FOC.Vq * cos_rad;
}

void FOC_Inverse_Clarke_Transform(void){
	FOC.Va = FOC.Valpha;
	FOC.Vb = -0.5 * FOC.Valpha + 0.866 * FOC.Vbeta;
	FOC.Vc = -0.5 * FOC.Valpha - 0.866 * FOC.Vbeta;
}


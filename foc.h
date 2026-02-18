


#ifndef  _FOC_H_
#define  _FOC_H_

#define  FOC_D_KP           10
#define  FOC_D_KI           10
#define  FOC_MAX_D_IERROR   10

#define  FOC_Q_KP           10
#define  FOC_Q_KI           10
#define  FOC_MAX_Q_IERROR   10


void     FOC_Forward_Clarke_Transform(void);
void     FOC_Forward_Park_Transform(float theta_rad);
void     FOC_Run_PI_Controller(void);
void     FOC_Inverse_Park_Transform(float theta_rad);
void     FOC_Inverse_Clarke_Transform(void);


#endif




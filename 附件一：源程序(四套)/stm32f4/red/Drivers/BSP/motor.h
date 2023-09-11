#ifndef __MOTOR_H
#define __MOTOR_H	

#include "main.h"

#define MAXDUTY 10000

typedef struct 
{
	float vx;
	float vw;
}Motor_t;

void MotorInit(Motor_t* motor);
void MotorCtrl_L(int16_t duty);
void MotorCtrl_R(int16_t duty);
void MotorCtrl(int16_t duty1,int16_t duty2);

#endif

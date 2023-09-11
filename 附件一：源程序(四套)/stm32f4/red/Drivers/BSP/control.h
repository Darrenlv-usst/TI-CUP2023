#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"

#define SERVO_MIDX 1574
#define SERVO_MIDY 2720
#define xoffset 0.06
#define yoffset 0.005
#define PI 3.1415926
#define SERVOX TIM3->CCR3
#define SERVOY TIM3->CCR4

typedef struct 
{
	int x;
	int y;
}Servo_t;

typedef struct 
{
	float x;
	float y;
}Point_t;
float abs(float a);
void Pos2Pwm(float x, float y, Servo_t *servo);
void Reset(Point_t *pos, Servo_t *servo);
float Move(float x,float y,int i ,float step);
#endif

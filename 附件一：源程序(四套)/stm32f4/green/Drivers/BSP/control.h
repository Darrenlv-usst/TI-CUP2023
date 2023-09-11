#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"

#define SERVO_MIDX 2250
#define SERVO_MIDY 2250
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
	double x;
	double y;
}Point_t;
double abs(double a);
void Pos2Pwm(double x, double y, Servo_t *servo);
void Reset(Point_t *pos, Servo_t *servo);
void Move(double targetx, double targety, Point_t *pos, Servo_t *servo);
#endif

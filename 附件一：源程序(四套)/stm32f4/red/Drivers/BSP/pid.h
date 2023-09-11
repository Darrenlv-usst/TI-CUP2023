#ifndef __PID_H
#define __PID_H		
 
#include "mpu6050.h"
#include "main.h"

typedef struct 
{
	float kp;
	float ki;
	float kd;
	
	float pout;
	float iout;
	float dout;
	float out;
	
	float intergrator;
	float imax;
	float max;
	
	float last_error;
	float last_derivative;
}pid_t;



void PID_L_Init(pid_t* pid);
void PID_R_Init(pid_t* pid);
void PID_DIR_Init(pid_t* pid);
void PID_SERVOX_Init(pid_t* pid);
void PID_SERVOY_Init(pid_t* pid);
void PID_DIS_Init(pid_t* pid);
float PID_Loc(pid_t* pid, float error);
//float PID_Vertical(pid_t * pid, MPU6050_t * mpu6050);
double PID_SERVO(pid_t* pid, double error);
void PID_SERVO_POINT_Init(pid_t* pid);
void PID_SERVO_POINTX_Init(pid_t* pid);
void PID_SERVO_POINTY_Init(pid_t* pid);
float PID_DIR(pid_t* pid, float error);
#endif

#include "pid.h"



void PID_L_Init(pid_t* pid)
{
//	pid->kp = 60;
//	pid->ki = 8;
//	pid->kd = 1;
//	pid->imax = 450;
//	pid->max = 6000;
	pid->kp = 50;
	pid->ki = 7.5;
	pid->kd = 2.5;
	pid->imax = 400;
	pid->max = 5000;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}

void PID_R_Init(pid_t* pid)
{
	pid->kp = 50;
	pid->ki = 7.5;
	pid->kd = 2.5;
	pid->imax = 400;
	pid->max = 5000;
//	pid->kp = 60;
//	pid->ki = 8;
//	pid->kd = 1;
//	pid->imax = 450;
//	pid->max = 6000;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}

void PID_DIR_Init(pid_t* pid)
{
	pid->kp = 1.1;
	pid->ki = 0;
	pid->kd = 0.15;
	pid->imax = 300;
	pid->max = 5000;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}

void PID_DIS_Init(pid_t* pid)
{
	pid->kp = 4;
	pid->ki = 0;
	pid->kd = 0.5;
	pid->imax = 300;
	pid->max = 4000;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}

void PID_SERVOX_Init(pid_t* pid)
{
	pid->kp = 0.24;
	pid->ki = 0;
	pid->kd = 0.03;
	pid->imax = 300;
	pid->max = 100;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}

void PID_SERVOY_Init(pid_t* pid)
{
	pid->kp = 0.24;
	pid->ki = 0;
	pid->kd = 0.03;
	pid->imax = 300;
	pid->max = 100;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}

void PID_SERVO_POINT_Init(pid_t* pid)
{
	pid->kp = 0.15;
	pid->ki = 0;
	pid->kd = 0.02;
	pid->imax = 300;
	pid->max = 100;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}

void PID_SERVO_POINTX_Init(pid_t* pid)
{
	pid->kp = 0.35;
	pid->ki = 0;
	pid->kd = 0.05;
	pid->imax = 300;
	pid->max = 100;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}
void PID_SERVO_POINTY_Init(pid_t* pid)
{
	pid->kp = 0.35;
	pid->ki = 0;
	pid->kd = 0.05;
	pid->imax = 300;
	pid->max = 100;
  pid->pout = 0;
  pid->iout = 0;
  pid->dout = 0;
  pid->out = 0;
  pid->intergrator = 0;
  pid->last_error = 0;
  pid->last_derivative = 0;
}




float Abs(float dat)
{
	return dat >= 0 ? dat : -dat;
}

float Limit(float dat, float max, float min)
{
	if(dat > max) dat = max;
	if(dat < min) dat = min;
	return dat;
}

float PID_Loc(pid_t* pid, float error)
{
	float beta;
	if(Abs(error)>60) beta = 0;
	else if(Abs(error)>20) beta = 0.6;
	else if(Abs(error)>10) beta = 0.9;
	else beta = 1;
	
	pid->intergrator += error;
	pid->intergrator = Limit(pid->intergrator, pid->imax, -pid->imax);
	
	
	pid->pout = pid->kp * error;
	pid->iout = beta * pid->ki * pid->intergrator;
	
	pid->dout = pid->kd * pid->last_error;
	
	pid->last_error = error;
	
	pid->out = pid->pout + pid->iout + pid->dout;
	
	pid->out = Limit(pid->out, pid->max, -pid->max); 
	
	return pid->out;
}

float PID_DIR(pid_t* pid, float error)
{
	pid->pout = pid->kp * error;

	pid->dout = pid->kd * pid->last_error;
	
	pid->last_error = error;
	
	pid->out = pid->pout + pid->dout;
	
	pid->out = Limit(pid->out, 80, -80); 
	
	return pid->out;
}

double PID_SERVO(pid_t* pid, double error)
{
	pid->pout = pid->kp * error;

	pid->dout = pid->kd * pid->last_error;
	
	pid->last_error = error;
	
	pid->out = pid->pout + pid->dout;
	
	pid->out = Limit(pid->out, pid->max, -pid->max); 
	
	return pid->out;
}


//float PID_Vertical(pid_t * pid, MPU6050_t * mpu6050)
//{
//	if(mpu6050->KalmanAngleX<1&&mpu6050->KalmanAngleX>-1)
//		mpu6050->KalmanAngleX = 0;
//	if(mpu6050->Ax<0.1&&mpu6050->Ax>-0.1)
//		mpu6050->Ax = 0;
//	return pid->kp*mpu6050->KalmanAngleX + pid->kd*mpu6050->Ax;
//}

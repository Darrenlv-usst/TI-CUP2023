
#include "motor.h"
#include "tim.h"

void MotorInit(Motor_t* motor)
{
	motor->vx = 120;
	motor->vw = 0;
}

//void MotorCtrl_L(int16_t duty)
//{
//	if(duty>=0)
//	{
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,duty);
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
//	}
//	else
//	{
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,-duty);
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
//	}
//}

//void MotorCtrl_R(int16_t duty)
//{
//	if(duty>=0)
//	{
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,duty);
//	}
//	else
//	{
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,-duty);
//		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
//	}
//}

void MotorCtrl_L(int16_t duty)
{
	if(duty>=0)
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,duty);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_4,-duty);
	}
}

void MotorCtrl_R(int16_t duty)
{
	if(duty>=0)
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,duty);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,0);
		
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_1,-duty);
		__HAL_TIM_SET_COMPARE(&htim8,TIM_CHANNEL_2,0);
	}
}


void MotorCtrl(int16_t duty1,int16_t duty2)
{
	MotorCtrl_L(duty1);
	MotorCtrl_R(duty2);
}


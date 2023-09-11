#include "encoder.h"
#include "tim.h"

void Encoder_Init(Encoder_t* encoder)
{
	encoder->Left = 0;
	encoder->Right = 0;
}

void EncoderGet(Encoder_t* encoder)
{
//	encoder->Left = -(short)__HAL_TIM_GET_COUNTER(&htim5);
//	__HAL_TIM_SET_COUNTER(&htim5,0);
//	
//	encoder->Right = (short)__HAL_TIM_GET_COUNTER(&htim4);
//	__HAL_TIM_SET_COUNTER(&htim4,0);
	
	encoder->Right = (short)__HAL_TIM_GET_COUNTER(&htim5);
	__HAL_TIM_SET_COUNTER(&htim5,0);
	
	encoder->Left = -(short)__HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4,0);
}

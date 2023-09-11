#ifndef __KEY_H__
#define __KEY_H__

#include "main.h"

#define KEY0 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_0)
#define KEY1 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_1)
#define KEY2 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_2)
#define KEY3 HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_3)

uint8_t Key_Scan(void);






#endif

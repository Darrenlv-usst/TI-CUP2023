#ifndef __DELAY_H
#define __DELAY_H

#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#define USE_FREERTOS_TIME //OR USE_FREERTOS_TIME

#ifdef USE_HAL_TIME
#define delay_ms(ms) HAL_Delay(ms)
#endif
#ifdef USE_FREERTOS_TIME
#define delay_ms(ms) osDelay(ms)
#endif


	
#endif

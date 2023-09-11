#include "key.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
uint8_t Key_Scan(void)
{
	static uint8_t key_up = 0;
	uint8_t key_val = 0;
	if(KEY0&&KEY1&&KEY2&&KEY3) key_up = 1;
	else if(key_up&&(!KEY0||!KEY1||!KEY2||!KEY3))
	{
		osDelay(10);
		if(!KEY0||!KEY1||!KEY2||!KEY3)
		{
			key_up = 0;
			if(!KEY0) key_val = 1;
			else if(!KEY1) key_val = 2;
			else if(!KEY2) key_val = 3;
			else if(!KEY3) key_val = 4;
		}
	}
	return key_val;
}

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "pid.h"
#include "usart.h"
#include "mpu6050.h"
#include "oled.h"
#include "motor.h"
#include "bsp_uart.h"
#include "encoder.h"
#include "mpu6050.h"
#include "key.h"
#include "control.h"
//#include "delay.h"
//#include "eMPL/inv_mpu.h"
//#include "eMPL/inv_mpu_dmp_motion_driver.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/*Structrue*/
Motor_t motor;
Encoder_t encoder;
MPU6050_t MPU6050;

pid_t pid_SERVOX;
pid_t pid_SERVOY;
pid_t pid_POINT;
pid_t pid_POINTX;
pid_t pid_POINTY;
Servo_t servo;
Point_t pos,LU,LD,RU,RD;


/*flag*/
uint8_t delay_flag = 0;
uint8_t rec_flag = 0;
uint8_t rec_ok_flag = 0;
uint8_t rx_ack = 0;
uint8_t task = 0;
uint8_t pause_flag = 0;
uint8_t reset_flag = 0;
uint8_t point_state = 0;

/*control*/
int Encoder_All_L = 0;
int Encoder_All_R = 0;
int dir_error = 0;
float angle = 0;
float servo_hori_error = 0;
float servo_vert_error = 0;

float servo_hori_all = 1400;
float servo_vert_all = 2500;
uint16_t servo_midx = 0;
uint16_t servo_midy = 0;
uint16_t servo_savex = 0;
uint16_t servo_savey = 0;
/*point*/

/*uart*/
uint8_t rx_len = 0;
uint8_t RxBuffer[255];

/* USER CODE END Variables */
osThreadId MotorTaskHandle;
osThreadId OledTaskHandle;
osThreadId Uart1TaskHandle;
osThreadId mpu6050Handle;
osThreadId ServoTaskHandle;
osThreadId KeyTaskHandle;
osThreadId LedTaskHandle;
osSemaphoreId UartRecSemHandle;
osSemaphoreId CrossSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartMotorTask(void const * argument);
void StartOledTask(void const * argument);
void StartUart1Task(void const * argument);
void Startmpu6050(void const * argument);
void StartServoTask(void const * argument);
void StartKeyTask(void const * argument);
void StartLedTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	PID_SERVOX_Init(&pid_SERVOX);
	PID_SERVOX_Init(&pid_SERVOY);
	PID_SERVO_POINT_Init(&pid_POINT);
	PID_SERVO_POINTX_Init(&pid_POINTX);
	PID_SERVO_POINTY_Init(&pid_POINTY);
	MotorInit(&motor);
	Encoder_Init(&encoder);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of UartRecSem */
  osSemaphoreDef(UartRecSem);
  UartRecSemHandle = osSemaphoreCreate(osSemaphore(UartRecSem), 1);

  /* definition and creation of CrossSem */
  osSemaphoreDef(CrossSem);
  CrossSemHandle = osSemaphoreCreate(osSemaphore(CrossSem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	//osSemaphoreWait(UartRecSemHandle,100);
	osSemaphoreWait(UartRecSemHandle,100);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of MotorTask */
  osThreadDef(MotorTask, StartMotorTask, osPriorityAboveNormal, 0, 128);
  MotorTaskHandle = osThreadCreate(osThread(MotorTask), NULL);

  /* definition and creation of OledTask */
  osThreadDef(OledTask, StartOledTask, osPriorityNormal, 0, 128);
  OledTaskHandle = osThreadCreate(osThread(OledTask), NULL);

  /* definition and creation of Uart1Task */
  osThreadDef(Uart1Task, StartUart1Task, osPriorityAboveNormal, 0, 128);
  Uart1TaskHandle = osThreadCreate(osThread(Uart1Task), NULL);

  /* definition and creation of mpu6050 */
  osThreadDef(mpu6050, Startmpu6050, osPriorityNormal, 0, 128);
  mpu6050Handle = osThreadCreate(osThread(mpu6050), NULL);

  /* definition and creation of ServoTask */
  osThreadDef(ServoTask, StartServoTask, osPriorityAboveNormal, 0, 128);
  ServoTaskHandle = osThreadCreate(osThread(ServoTask), NULL);

  /* definition and creation of KeyTask */
  osThreadDef(KeyTask, StartKeyTask, osPriorityNormal, 0, 128);
  KeyTaskHandle = osThreadCreate(osThread(KeyTask), NULL);

  /* definition and creation of LedTask */
  osThreadDef(LedTask, StartLedTask, osPriorityNormal, 0, 128);
  LedTaskHandle = osThreadCreate(osThread(LedTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief  Function implementing the MotorTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartMotorTask */
__weak void StartMotorTask(void const * argument)
{
  /* USER CODE BEGIN StartMotorTask */
	int v1,v2;
	static int i = 0;
	
  /* Infinite loop */
  for(;;)
  {
//		Encoder_All_L += encoder.Left;
//		Encoder_All_R += encoder.Right;
//		EncoderGet(&encoder);
////			UART_SendToAnon(encoder.Left);

//			MotorCtrl((int)v1,(int)v2);
		
    osDelay(10);
  }
  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
__weak void StartOledTask(void const * argument)
{
  /* USER CODE BEGIN StartOledTask */
	char txt[30];
	
  /* Infinite loop */
  for(;;)
  {
//		taskENTER_CRITICAL();
		sprintf(txt,"reset:%d",reset_flag);
		OLED_ShowString(0,2,txt);
		sprintf(txt,"pause:%d",pause_flag);
		OLED_ShowString(0,4,txt);
		sprintf(txt,"task:%d",task);
		OLED_ShowString(0,6,txt);
//		taskEXIT_CRITICAL();
    osDelay(100);
		
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartUart1Task */
/**
* @brief Function implementing the Uart1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUart1Task */
__weak void StartUart1Task(void const * argument)
{
  /* USER CODE BEGIN StartUart1Task */
	
	//注意初始化更改位置
	char txt[40];
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,(uint8_t *)RxBuffer,255);
	
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(UartRecSemHandle,portMAX_DELAY);
		//taskENTER_CRITICAL();

//		HAL_UART_Transmit(&huart1,RxBuffer,rx_len,100);
		
//		OLED_ShowString(0,6,RxBuffer);
		if(RxBuffer[0]==0x12)
		{
			OLED_ShowString(0,0,"ok");
			SERVOY = RxBuffer[1]<<8|RxBuffer[2];
		}
		if(RxBuffer[0]==0x13)
		{
			OLED_ShowString(0,0,"ok");
			SERVOX = RxBuffer[1]<<8|RxBuffer[2];
		}
		if(RxBuffer[0]==0xA1 && RxBuffer[rx_len-1]==0x1A)
		{
			if(RxBuffer[1] == 0x11)
			{
				if(task == 1)
				{
					servo_midx = SERVOX;
					servo_midy = SERVOY;
				}
				rec_ok_flag = 1;
			}
			if(RxBuffer[1] == 0x33)
			{	
				rx_ack = 1;
			}
			if(RxBuffer[1] == 0x44)
			{
				point_state++;
				sprintf(txt,"point_num%d",point_state);
				OLED_ShowString(0,0,txt);
			}

			else if(RxBuffer[1] == 0x02)
			{
				if(RxBuffer[2] == 1)
					servo_hori_error = -RxBuffer[3];
				else
					servo_hori_error = RxBuffer[3];
				if(RxBuffer[4] == 1)
					servo_vert_error = -RxBuffer[5];
				else
					servo_vert_error = RxBuffer[5];
//				if(servo_hori_error < 2)
//					servo_hori_error /= 2.0;
//				if(servo_vert_error < 2)
//					servo_vert_error /= 2.0;
				rec_flag = 1;
			}
	
//			else if(RxBuffer[1] == 0x03)
//			{
//				OLED_ShowString(0,4,"--------");
//				if(RxBuffer[2] == 3)
//				{
//					if(RxBuffer[3]==0) servo_vert_error = RxBuffer[4];
//					else if(RxBuffer[3]==1) servo_vert_error = -RxBuffer[4];
//					if(RxBuffer[5]==0) servo_hori_error = RxBuffer[6];
//					else if(RxBuffer[5]==1) servo_hori_error = -RxBuffer[6];
//					if(RxBuffer[7]==0) servo_vert_error += RxBuffer[8]/2 + 80;
//					else if(RxBuffer[7]==1) servo_vert_error += -RxBuffer[8]/2 + 80;
//					if(RxBuffer[9]==0) servo_hori_error += RxBuffer[10]/2 + 80;
//					else if(RxBuffer[9]==1) servo_hori_error += -RxBuffer[10]/2 + 80;
//					sprintf(txt,"  cross:%d  ",servo_vert_error);
//					OLED_ShowString(0,0,txt);
//					sprintf(txt,"  dir:%d  ",servo_hori_error);
//					OLED_ShowString(0,2,txt);
//				}
//			}
		}
		


//		sprintf(txt,"%d",rx_len);
//		OLED_ShowString(0,6,(uint8_t *)txt);

		//taskEXIT_CRITICAL();

		
    osDelay(1);
  }
  /* USER CODE END StartUart1Task */
}

/* USER CODE BEGIN Header_Startmpu6050 */
/**
* @brief Function implementing the mpu6050 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startmpu6050 */
__weak void Startmpu6050(void const * argument)
{
  /* USER CODE BEGIN Startmpu6050 */
	char txt[10];
//	while(MPU_Init());
//	while(mpu_dmp_init());
  /* Infinite loop */
  for(;;)
  {
		
//		MPU6050_Read_All(&hi2c1, &MPU6050);
//		if(MPU6050.Gz<0.5&&MPU6050.Gz>-0.5)
//			MPU6050.Gz = 0;
//		yaw += MPU6050.Gz/7000*360;
		
    osDelay(10);
  }
  /* USER CODE END Startmpu6050 */
}

/* USER CODE BEGIN Header_StartServoTask */
/**
* @brief Function implementing the ServoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartServoTask */
__weak void StartServoTask(void const * argument)
{
  /* USER CODE BEGIN StartServoTask */
	static int servo_val = 500;
	static uint16_t step_task2 = 0;
	static int i;
	char txt[20];
  /* Infinite loop */
  for(;;)
  {
		if(pause_flag == 1)
		{
			
		}
		else if(reset_flag == 1)
		{
//			if(rec_flag ==1)
//			{
//				rec_flag = 0;
//				servo_hori_all += PID_SERVO(&pid_POINTX,-servo_hori_error);
//				servo_vert_all += PID_SERVO(&pid_POINTY,-servo_vert_error);
//				SERVOX = (uint16_t)servo_hori_all;
//				SERVOY = (uint16_t)servo_vert_all;
//			}
//			if(rec_ok_flag == 1)
//				reset_flag = 0;
		}
		else if(task == 1 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"1",1,100);
			if(rec_flag ==1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_POINTX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_POINTY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 2 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"2",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
//				if(abs(servo_hori_error)<=1) servo_hori_error=0;
//				if(abs(servo_vert_error)<=1) servo_vert_error=0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 3 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"3",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 4 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"4",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 5 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"3",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 6 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"4",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 7 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"3",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 8 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"4",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 9 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"3",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		else if(task == 10 && rec_ok_flag == 0)
		{
			if(rx_ack == 0)
				HAL_UART_Transmit(&huart3,"4",1,100);
			if(rec_flag == 1)
			{
				rec_flag = 0;
				servo_hori_all += PID_SERVO(&pid_SERVOX,-servo_hori_error);
				servo_vert_all += PID_SERVO(&pid_SERVOY,-servo_vert_error);
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			}
		}
		
		osDelay(10);

  }
  /* USER CODE END StartServoTask */
}

/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief Function implementing the KeyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyTask */
__weak void StartKeyTask(void const * argument)
{
  /* USER CODE BEGIN StartKeyTask */
  /* Infinite loop */
  for(;;)
  {
		switch(Key_Scan())
		{
			case 1:
//				HAL_UART_Transmit(&huart3,"1",1,100);
//				reset_flag = 1;
			
				servo_hori_all = servo_midx;
				servo_vert_all = servo_midy;
				SERVOX = (uint16_t)servo_hori_all;
				SERVOY = (uint16_t)servo_vert_all;
			
//				reset_flag ^= 1;
//				if(reset_flag == 1)
//				{
////					HAL_UART_Transmit(&huart1,"r",1,100);
//					servo_savex = SERVOX;
//					servo_savey = SERVOY;
//				}
//				else
//				{
////					HAL_UART_Transmit(&huart1,"R",1,100);
//					servo_hori_all = servo_savex;
//					servo_vert_all = servo_savey;
//					SERVOX = (uint16_t)servo_hori_all;
//					SERVOY = (uint16_t)servo_vert_all;
//				}
				break;
			case 2:
				pause_flag ^= 1;
//				if(pause_flag == 1)
//					HAL_UART_Transmit(&huart1,"p",1,100);
//				else
//					HAL_UART_Transmit(&huart1,"P",1,100);
				break;
			case 4:
				task ++;
				rec_ok_flag = 0;
				rx_ack = 0;
				break;
			case 3:
				if(point_state<5)
					HAL_UART_Transmit(&huart3,"9",1,100);
				else
				{
					rec_ok_flag = 0;
					rx_ack = 0;
				}
				break;
			default:break;
		}
    osDelay(1);
  }
  /* USER CODE END StartKeyTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the LedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
__weak void StartLedTask(void const * argument)
{
  /* USER CODE BEGIN StartLedTask */
  /* Infinite loop */
  for(;;)
  {
//		HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
    osDelay(1);
  }
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

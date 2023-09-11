#include "bsp_uart.h"


extern DMA_HandleTypeDef hdma_usart1_tx;

void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length)
{
    //等待上一次的数据发送完毕
	while(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY){};
    //while(__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
	
    //关闭DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    //开始发送数据
    HAL_UART_Transmit_DMA(&huart1, buffer, length);
}


void UART_SendToAnon(int A)
{
	uint8_t BUFF[30];
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt=0;
	BUFF[_cnt++]=0xAA;//帧头
	BUFF[_cnt++]=0xFF;//目标地址
	BUFF[_cnt++]=0XF1;//功能码
	BUFF[_cnt++]=0x02;//数据长度
	BUFF[_cnt++]=BYTE0(A);//数据内容,小段模式，低位在前
	BUFF[_cnt++]=BYTE1(A);//需要将字节进行拆分，调用上面的宏定义即可。
//	BUFF[_cnt++]=BYTE0(B);
//	BUFF[_cnt++]=BYTE1(B);	
//	BUFF[_cnt++]=C;
	//SC和AC的校验直接抄最上面上面简介的即可
	for(int i=0;i<BUFF[3]+4;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	
	
	//UART1_TX_DMA_Send(BUFF,_cnt);
	HAL_UART_Transmit(&huart1,BUFF,_cnt,10);//串口逐个发送数据
	//HAL_Delay(10);
	//while(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY);
}



//串口1的DMA发送printf
//void Debug_printf(const char *format, ...)
//{
//	uint32_t length = 0;
//	va_list args;
//	
//	__va_start(args, format);
//	
//	length = vsnprintf((char*)BUFF, sizeof(BUFF), (char*)format, args);
//	
//	UART1_TX_DMA_Send(BUFF, length);
//}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) //如果是串口1
	{
		// 在F7系列是可以不写的，F1必须写
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_FLAG_TCIF3_7); //清除DMA2_Steam7传输完成标志
		HAL_UART_DMAStop(&huart1);		//传输完成以后关闭串口DMA,缺了这一句会死机
	}
}


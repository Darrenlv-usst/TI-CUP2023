#include "bsp_uart.h"


extern DMA_HandleTypeDef hdma_usart1_tx;

void UART1_TX_DMA_Send(uint8_t *buffer, uint16_t length)
{
    //�ȴ���һ�ε����ݷ������
	while(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY){};
    //while(__HAL_DMA_GET_COUNTER(&hdma_usart1_tx));
	
    //�ر�DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    //��ʼ��������
    HAL_UART_Transmit_DMA(&huart1, buffer, length);
}


void UART_SendToAnon(int A)
{
	uint8_t BUFF[30];
	uint8_t sumcheck = 0;
	uint8_t addcheck = 0;
	uint8_t _cnt=0;
	BUFF[_cnt++]=0xAA;//֡ͷ
	BUFF[_cnt++]=0xFF;//Ŀ���ַ
	BUFF[_cnt++]=0XF1;//������
	BUFF[_cnt++]=0x02;//���ݳ���
	BUFF[_cnt++]=BYTE0(A);//��������,С��ģʽ����λ��ǰ
	BUFF[_cnt++]=BYTE1(A);//��Ҫ���ֽڽ��в�֣���������ĺ궨�弴�ɡ�
//	BUFF[_cnt++]=BYTE0(B);
//	BUFF[_cnt++]=BYTE1(B);	
//	BUFF[_cnt++]=C;
	//SC��AC��У��ֱ�ӳ�������������ļ���
	for(int i=0;i<BUFF[3]+4;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	
	
	//UART1_TX_DMA_Send(BUFF,_cnt);
	HAL_UART_Transmit(&huart1,BUFF,_cnt,10);//���������������
	//HAL_Delay(10);
	//while(HAL_DMA_GetState(&hdma_usart1_tx) != HAL_DMA_STATE_READY);
}



//����1��DMA����printf
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
	if (huart->Instance == USART1) //����Ǵ���1
	{
		// ��F7ϵ���ǿ��Բ�д�ģ�F1����д
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_FLAG_TCIF3_7); //���DMA2_Steam7������ɱ�־
		HAL_UART_DMAStop(&huart1);		//��������Ժ�رմ���DMA,ȱ����һ�������
	}
}


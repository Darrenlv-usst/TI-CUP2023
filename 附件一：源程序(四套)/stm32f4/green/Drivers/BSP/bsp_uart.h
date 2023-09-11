#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "main.h"
#include "usart.h"



typedef struct 
{

    uint8_t *tx_data;
}UartHead;


typedef struct 
{
    UartHead uart2_head;
}BspUartStr;

void UART_SendToAnon(int A);
void printf2(char* fmt,...);
#endif

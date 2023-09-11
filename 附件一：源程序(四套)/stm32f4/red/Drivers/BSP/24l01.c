#include "24L01.h"
#include "spi.h"
#include "usart.h"
#include "stdio.h"
#include "oled.h"

//NRF24L01 驱动函数

unsigned char idel_mode_flag = 0;
unsigned char mode_time_counter = 0;

const unsigned char INIT_ADDR0[5]= {0x02,0x3A,0xB1,0xB1,0x01};
const unsigned char INIT_ADDR1[5]= {0x02,0x3A,0x01,0x01,0x01};
const unsigned char INIT_ADDR2[5]= {0x03,0x3A,0x01,0x01,0x01};
const unsigned char INIT_ADDR3[5]= {0x04,0x3A,0x01,0x01,0x01};
const unsigned char INIT_ADDR4[5]= {0x05,0x3A,0x01,0x01,0x01};
const unsigned char INIT_ADDR5[5]= {0x06,0x3A,0x01,0x01,0x01};

#define CH_Num	120
//#define debug_out(fmt,args...)  printf(fmt,##args)
//#define debug_out(fmt,args...) 

void delay_us(uint32_t n)
{
	unsigned char i;

	while(n--)
	{
		i = 8;
		while(i--);
	}
}
//初始化24L01的IO口
void NRF24L01_Init(void)
{
	//spi init
	//gpio init
	Clr_NRF24L01_CE;    // chip enable
	Set_NRF24L01_CSN;   // Spi disable
	delay_us(100);
}

//封装spi读写函数
unsigned char nRF24_SPI_Send_Byte(unsigned char txdata)
{
	unsigned char rxdata;
	HAL_SPI_TransmitReceive(&hspi2, &txdata, &rxdata, 1, 0x10);
	return(rxdata);							// return read unsigned char
}


//通过SPI写寄存器
unsigned char NRF24L01_Write_Reg(unsigned char regaddr,unsigned char data)
{
	unsigned char status;
	Clr_NRF24L01_CSN;                    //使能SPI传输
	status =nRF24_SPI_Send_Byte(regaddr); //发送寄存器号
	nRF24_SPI_Send_Byte(data);            //写入寄存器的值
	Set_NRF24L01_CSN;                    //禁止SPI传输
	return(status);       		         //返回状态值
}
//读取SPI寄存器值 ，regaddr:要读的寄存器
unsigned char NRF24L01_Read_Reg(unsigned char regaddr)
{
	unsigned char reg_val;
	Clr_NRF24L01_CSN;                //使能SPI传输
	nRF24_SPI_Send_Byte(regaddr);     //发送寄存器号
	reg_val=nRF24_SPI_Send_Byte(0XFF);//读取寄存器内容
	Set_NRF24L01_CSN;                //禁止SPI传输
	return(reg_val);                 //返回状态值
}
//在指定位置读出指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值
unsigned char NRF24L01_Read_Buf(unsigned char regaddr,unsigned char *pBuf,unsigned char datalen)
{
	unsigned char status,u8_ctr;
	Clr_NRF24L01_CSN;                     //使能SPI传输
	status=nRF24_SPI_Send_Byte(regaddr);   //发送寄存器值(位置),并读取状态值
	//for(u8_ctr=0;u8_ctr<datalen;u8_ctr++)pBuf[u8_ctr]=SPI_ReadWriteByte(0XFF);//读出数据
	HAL_SPI_Receive(&hspi2, pBuf, datalen, 0x10);
	Set_NRF24L01_CSN;                     //关闭SPI传输
	return status;                        //返回读到的状态值
}
//在指定位置写指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值
unsigned char NRF24L01_Write_Buf(unsigned char regaddr, unsigned char *pBuf, unsigned char datalen)
{
	unsigned char status,u8_ctr;
	Clr_NRF24L01_CSN;                                    //使能SPI传输
	status = nRF24_SPI_Send_Byte(regaddr);                //发送寄存器值(位置),并读取状态值
	//for(u8_ctr=0; u8_ctr<datalen; u8_ctr++)SPI_ReadWriteByte(*pBuf++); //写入数据
	HAL_SPI_Transmit(&hspi2, pBuf, datalen, 0x10);
	Set_NRF24L01_CSN;                                    //关闭SPI传输
	return status;                                       //返回读到的状态值
}
//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:发送完成状况
unsigned char NRF24L01_TxPacket(unsigned char *txbuf)
{
	unsigned char state;
	Clr_NRF24L01_CE;
	NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
	Set_NRF24L01_CE;                                     //启动发送
	while(READ_NRF24L01_IRQ!=0);                         //等待发送完成
	state=NRF24L01_Read_Reg(STATUS);                     //读取状态寄存器的值
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //清除TX_DS或MAX_RT中断标志
	if(state&MAX_TX)                                     //达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);               //清除TX FIFO寄存器
		HAL_UART_Transmit(&huart1,"MAXerror!\r\n",11,100);
		//debug_out("TX MAX_TX error!\r\n");
		return MAX_TX;
	}
	if(state&TX_OK)                                      //发送完成
	{
		HAL_UART_Transmit(&huart1,"TXOK!\r\n",7,100);
		//debug_out("TX OK!\r\n");
		return TX_OK;
	}
	HAL_UART_Transmit(&huart1,"TXerror!\r\n",10,100);
	//debug_out("TX other error!\r\n");
	return 0xff;                                         //其他原因发送失败
}

//启动NRF24L01发送一次数据
//txbuf:待发送数据首地址
//返回值:0，接收完成；其他，错误代码
unsigned char NRF24L01_RxPacket(unsigned char *rxbuf)
{
	unsigned char state;
	state=NRF24L01_Read_Reg(STATUS);                //读取状态寄存器的值
	NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
	if(state&TX_OK)
	{
		//debug_out("RX send ack!\r\n"); //成功发送ACK
	}
	if(state&RX_OK)                                 //接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
		NRF24L01_Write_Reg(FLUSH_RX,0xff);          //清除RX FIFO寄存器
		//debug_out("RX read data!\r\n");
		return 0;
	}
	return 1;                                      //没收到任何数据
}

//该函数初始化NRF24L01到RX模式
//设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
//当CE变高后,即进入RX模式,并可以接收数据了
void RX_Mode(void)
{
	Clr_NRF24L01_CE;
	//写RX节点地址
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(unsigned char*)INIT_ADDR0,RX_ADR_WIDTH);

	//使能通道0的自动应答
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);
	//使能通道0的接收地址
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);
	//设置RF通信频率
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,CH_Num);
	//选择通道0的有效数据宽度
	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);
	//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);
	//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX接收模式
	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG, 0x0f);
	//CE为高,进入接收模式
	Set_NRF24L01_CE;
}

//该函数初始化NRF24L01到TX模式
//设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,
//选择RF频道,波特率和LNA HCURR PWR_UP,CRC使能
//当CE变高后,即进入RX模式,并可以接收数据了
//CE为高大于10us,则启动发送.
void TX_Mode(void)
{
  //启动发送模式
	Clr_NRF24L01_CE;
	//写TX节点地址
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(unsigned char*)INIT_ADDR0,TX_ADR_WIDTH);
	//设置TX节点地址,主要为了使能ACK
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(unsigned char*)INIT_ADDR0,RX_ADR_WIDTH);

	//使能通道0的自动应答
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);
	//使能通道0的接收地址
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);
	//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);
	//设置RF通道为40
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,CH_Num);
	//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);
	//配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,PRIM_RX发送模式,开启所有中断
	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG,0x0e);
	// CE为高,10us后启动发送
	Set_NRF24L01_CE;
}

//上电检测NRF24L01是否在位
//写5个数据然后再读回来进行比较，
//相同时返回值:0，表示在位;否则返回1，表示不在位
unsigned char NRF24L01_Check(void)
{
	unsigned char buf[5]= {0XA5,0XA5,0XA5,0XA5,0XA5};
	unsigned char buf1[5];
	unsigned char i;
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.
	NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //读出写入的地址
	for(i=0; i<5; i++)if(buf1[i]!=0XA5)break;
	if(i!=5)
	{
		OLED_ShowString(0,0,"FAIL");
		//debug_out(("nRF24L01 TEST FAIL\r\n"));
		return 1;                               //NRF24L01不在位
	}
	OLED_ShowString(0,0,"TRUE");
	//debug_out(("nRF24L01 TEST OK\r\n"));
	return 0;		                           //NRF24L01在位
}

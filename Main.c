/*********************************************************************
功能:
	RFID TARGET发射程序
说明:
	本程序采用定时2秒发射ID一次,发射完毕进入低功耗模式,低功耗电流3uA
	ID长度为6 byte
**********************************************************************/


#include <Nordic\reg24le1.h>
#include <stdint.h>
#include "API.h"
#include "hal_delay.h"
//#include "lib_eeprom255.h"

//#include "hal_uart.h"
//#include "hal_clk.h"

#define LED9 P06
#define LED

#define INTERRUPT_RFIRQ	9
#define INTERRUPT_TICK 	13  

#define TX_ADR_WIDTH    5   										// RF收发地址共5 bytes 
#define TX_PLOAD_WIDTH  9  											// 数据包长度为20 bytes

uint8_t key[5] = {1,1,7,0,8};										//发卡密码：tag([4],[8])=(1,8)
uint8_t tag_key[5] = {2,2,8,1,9};								//tag密码：card([5],[7])=(2,1)

uint8_t const TX_ADDRESS[TX_ADR_WIDTH]  = {0x34,0x56,0x78,0x90,0x14}; // 定义RF收发地址


uint8_t data id_buf[TX_PLOAD_WIDTH]={0xff, 0x01, 0x02, 0x03, 0x04, 0x05};

uint8_t bdata sta;
sbit	RX_DR	=sta^6;
sbit	TX_DS	=sta^5;
sbit	MAX_RT	=sta^4;

uint8_t eepromdata;
uint8_t uart_rx_buf[11],radio_rx_buf[11],uart_rx_num = 0;
uint8_t radio_tx_flag = 0,uart_tx_flag = 0;

uint8_t i = 0;

/**************************************************
功能：硬件SPI读写
**************************************************/
uint8_t SPI_RW(uint8_t value)
{
		SPIRDAT = value;
  											       
		while(!(SPIRSTAT & 0x02));  					// 等待SPI传输完成

		return SPIRDAT;             					// 返回读出值
}
/**************************************************
功能：写RF寄存器，读RF状态值
**************************************************/
uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value)
{
		uint8_t status;

  	RFCSN = 0;                   	
  	status = SPI_RW(reg);      					// 选择RF寄存器
  	SPI_RW(value);             					// 写入数据
  	RFCSN = 1;                   	

  	return(status);            					// 返回RF状态值
}
/**************************************************
功能：读RF寄存器
**************************************************/
uint8_t SPI_Read(uint8_t reg)
{
		uint8_t reg_val;

  	RFCSN = 0;                			
  	SPI_RW(reg);            					// 选择RF寄存器
  	reg_val = SPI_RW(0);    					// 读出数据
  	RFCSN = 1;                			

  	return(reg_val);        					// 返回RF状态值
}
/**************************************************
功能：把缓冲区的多字节数据写到RF寄存器
**************************************************/
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
		uint8_t status,byte_ctr;

  	RFCSN = 0;                   		
  	status = SPI_RW(reg);    												// 选择RF寄存器
  	for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) 		// 连接写入数据
    SPI_RW(*pBuf++);
  	RFCSN = 1;                 			
  	return(status);          												// 返回RF状态值
}
/**************************************************
功能：读RF寄存器多字节数据到缓冲区
**************************************************/
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

  	RFCSN = 0;                    		
  	status = SPI_RW(reg);       										// 选择RF寄存器

  	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
    	pBuf[byte_ctr] = SPI_RW(0);    								// 连接读出数据

  	RFCSN = 1;                          

  	return(status);                    							// 返回RF状态值
}
/**************************************************
功能：设置为掉电模式
**************************************************/
void PD_Mode(void)
{
		RFCE=0;
  	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0c);   				// PWR_UP=0
}
/**************************************************
功能：设置为接收模式
**************************************************/
void RX_Mode(void)
{
		RFCE=0;
  	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);   			// 上电, CRC为2 bytes,接收模式,允许RX_DR产生中断
  	RFCE = 1; 																		// 启动接收模式
}
/**************************************************
功能：设置为发射模式
**************************************************/
void TX_Mode(uint8_t *rf_data,uint8_t num)
{
		num = TX_PLOAD_WIDTH;
  	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     													// 上电, CRC为2 bytes,接收模式,允许RX_DR产生中断
		SPI_Write_Buf(WR_TX_PLOAD, rf_data, TX_PLOAD_WIDTH); 								// 写数据到FIFO
		RFCE=1;																															// 启动发射
		delay_us(15);																												// 发射脉冲
		RFCE=0;												
}
/**************************************************
功能：RF初始化
**************************************************/
void rf_init(void)
{
  	RFCE = 0;                                   											// RF关闭
  	RFCKEN = 1;                                 											// 启动RF时钟
  	RF = 1;                                     											// 允许RF中断

		delay_ms(10);

  	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    	// 设置发射地址长度
  	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 	// 设置接收地址长度

  	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      												// 启动自动应答功能
  	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  												// PIPE0接收数据
  	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); 												// 自动重传10次
  	SPI_RW_Reg(WRITE_REG + RF_CH, 40);        												// RF频率2440MHz
  	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0f);   												// 发射功率0dBm, 传输速率2Mbps,
  	SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); 								// PIPE0 接收数据包长度			
}
/**************************************************
功能：串口初始化子程序
说明：波特率19.2K，使用内部波特率发生器
**************************************************/
void uart_init(void)
{
    ES0 = 0;                      				// 关UART0中断
    REN0 = 1;                     				// 允许接收
    SM0 = 0;                      				// 串口模式1，8bit可变波特率
    SM1 = 1;                   
    PCON |= 0x80;                 				// SMOD = 1
    ADCON |= 0x80;                				// 选择内部波特率发生器

    S0RELL = 0xe6;                				// 波特率19.2K(十进制998=十六进制0x03e6)
    S0RELH = 0x03;
    TI0 = 0;					  									// 清发送完成标志
		S0BUF=0x99;					  								// 送初值
}
/**************************************************
功能：向串口发送1 byte数据
**************************************************/
void uart_putchar(uint8_t x)
{
		while (!TI0);														// 等待发送完成
		TI0=0;																	// 清发送完成标志
		S0BUF=x;																// 发送数据
}
/**************************************************
功能:I/O口初始化
**************************************************/
void io_init(void)
{
  	P0DIR = 0xf0;							   	// 设定I/O口输入输出
  	P1DIR = 0xff;					                 
} 
/**************************************************
功能:LED配置，LED9
**************************************************/
void led_init()
{
	P0DIR &= 0xBF;				//P06 out
	LED9 = 1;
}
/**************************************************
功能：uart中断服务程序
**************************************************/
//void UART_IRQ(void) interrupt INTERRUPT_UART0
//{
//		char tmp;
//	
////		LED9 = !LED9;
//		if (RI0 == 1)
//		{
//			RI0 = 0;
//			tmp = S0BUF;		
//			if(tmp == '\n' && uart_rx_num == 10)
////			if(uart_rx_num == 8)
//			{
//				radio_tx_flag = 1;
////				LED9 = !LED9;
//			}
////			else
////			{
////					
////			}
//			uart_rx_buf[uart_rx_num++] = S0BUF;
//		}			
//}

void UART_IRQ(void) interrupt INTERRUPT_UART0
{
//		LED9 = !LED9;
		if (RI0 == 1)
		{
			RI0 = 0;
			uart_rx_buf[uart_rx_num++] = S0BUF;			
			if(uart_rx_num == 11)
			{
				radio_tx_flag = 1;
			}
		}			
}
/**************************************************
功能：RF中断服务程序
**************************************************/
void RF_IRQ(void) interrupt INTERRUPT_RFIRQ
{
		sta=SPI_Read(STATUS);																					// 读出状态值
	
//		LED9 = !LED9;
		if(RX_DR)									
		{
			RFCE=0;																											//关闭RF,进入待机模式
			SPI_Read_Buf(RD_RX_PLOAD,radio_rx_buf,TX_PLOAD_WIDTH);			// 读出FIFO的数据
			SPI_RW_Reg(FLUSH_RX,0);																			// 清除RX的FIFO
			
			uart_tx_flag = 1;
		}
		
		if(MAX_RT)
	{
			SPI_RW_Reg(FLUSH_TX,0);							// 清除TX的FIFO !!!达到最大重传次数，清空TX——buffer，确保不会PID加一后，被继续发送
  }
	
		SPI_RW_Reg(WRITE_REG+STATUS,0x70);														// 清除所有中断标志 
}
/**************************************************
功能：主程序
**************************************************/
void main(void)
{
		rf_init();																				// RF初始化 
		uart_init();																			// uart初始化
		io_init();                      
		led_init();
		ES0 = 1;
		EA=1;

		while(1)
		{
			if(radio_tx_flag)
			{
				if(uart_rx_buf[9] == 0x0d && uart_rx_buf[10] == 0x0a)
				{
//						uart_putchar(0x01);
						RFCE=0;																		//关闭RF
						TX_Mode(uart_rx_buf,9);										// 发射数据
						while (!(TX_DS|MAX_RT));									// 等待发射结束
						sta = 0;
						PD_Mode();																// 关RF
						LED9 = !LED9;
						radio_tx_flag = 0;
				
						for(i = 0;i < 11;i++)
						{
						uart_rx_buf[i] = '\0';
						}
						uart_rx_num = 0;
			
						RX_Mode();																//进入接收模式
					
				}
				else
				{
						uart_putchar(0xff);
					
//						uart_putchar(uart_rx_buf[0]);
//						uart_putchar(uart_rx_buf[1]);
//						uart_putchar(uart_rx_buf[2]);
//						uart_putchar(uart_rx_buf[3]);
//						uart_putchar(uart_rx_buf[4]);
//						uart_putchar(uart_rx_buf[5]);
//						uart_putchar(uart_rx_buf[6]);
//						uart_putchar(uart_rx_buf[7]);
//						uart_putchar(uart_rx_buf[8]);
					
						uart_putchar(0x0d);												//\r
						uart_putchar(0x0a);												//\n
						for(i = 0;i < 11;i++)
						{
								uart_rx_buf[i] = '\0';
						}
						uart_rx_num = 0;
						radio_tx_flag = 0;
				}
				
		}
				
			
		if(RX_DR)
		{
				sta = 0;
//				LED9 = !LED9;
//			RFCE=0;																				//关闭RF
				if(radio_rx_buf[5] == tag_key[1] && radio_rx_buf[7] == tag_key[3])
				{
						uart_putchar(radio_rx_buf[0]);
						uart_putchar(radio_rx_buf[1]);
						uart_putchar(radio_rx_buf[2]);
						uart_putchar(radio_rx_buf[3]);
//					uart_putchar(radio_rx_buf[4]);
//					uart_putchar(radio_rx_buf[5]);
//					uart_putchar(radio_rx_buf[6]);
//					uart_putchar(radio_rx_buf[7]);
//					uart_putchar(radio_rx_buf[8]);
						uart_putchar(0x0d);												//\r
						uart_putchar(0x0a);												//\n
				}
				
				for(i = 0;i < 11;i++)
				{
						radio_rx_buf[i] = '\0';
				}
		}
						
	}
}              				



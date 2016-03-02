/*********************************************************************
����:
	RFID TARGET�������
˵��:
	��������ö�ʱ2�뷢��IDһ��,������Ͻ���͹���ģʽ,�͹��ĵ���3uA
	ID����Ϊ6 byte
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

#define TX_ADR_WIDTH    5   										// RF�շ���ַ��5 bytes 
#define TX_PLOAD_WIDTH  9  											// ���ݰ�����Ϊ20 bytes

uint8_t key[5] = {1,1,7,0,8};										//�������룺tag([4],[8])=(1,8)
uint8_t tag_key[5] = {2,2,8,1,9};								//tag���룺card([5],[7])=(2,1)

uint8_t const TX_ADDRESS[TX_ADR_WIDTH]  = {0x34,0x56,0x78,0x90,0x14}; // ����RF�շ���ַ


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
���ܣ�Ӳ��SPI��д
**************************************************/
uint8_t SPI_RW(uint8_t value)
{
		SPIRDAT = value;
  											       
		while(!(SPIRSTAT & 0x02));  					// �ȴ�SPI�������

		return SPIRDAT;             					// ���ض���ֵ
}
/**************************************************
���ܣ�дRF�Ĵ�������RF״ֵ̬
**************************************************/
uint8_t SPI_RW_Reg(uint8_t reg, uint8_t value)
{
		uint8_t status;

  	RFCSN = 0;                   	
  	status = SPI_RW(reg);      					// ѡ��RF�Ĵ���
  	SPI_RW(value);             					// д������
  	RFCSN = 1;                   	

  	return(status);            					// ����RF״ֵ̬
}
/**************************************************
���ܣ���RF�Ĵ���
**************************************************/
uint8_t SPI_Read(uint8_t reg)
{
		uint8_t reg_val;

  	RFCSN = 0;                			
  	SPI_RW(reg);            					// ѡ��RF�Ĵ���
  	reg_val = SPI_RW(0);    					// ��������
  	RFCSN = 1;                			

  	return(reg_val);        					// ����RF״ֵ̬
}
/**************************************************
���ܣ��ѻ������Ķ��ֽ�����д��RF�Ĵ���
**************************************************/
uint8_t SPI_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
		uint8_t status,byte_ctr;

  	RFCSN = 0;                   		
  	status = SPI_RW(reg);    												// ѡ��RF�Ĵ���
  	for(byte_ctr=0; byte_ctr<bytes; byte_ctr++) 		// ����д������
    SPI_RW(*pBuf++);
  	RFCSN = 1;                 			
  	return(status);          												// ����RF״ֵ̬
}
/**************************************************
���ܣ���RF�Ĵ������ֽ����ݵ�������
**************************************************/
uint8_t SPI_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t bytes)
{
	uint8_t status,byte_ctr;

  	RFCSN = 0;                    		
  	status = SPI_RW(reg);       										// ѡ��RF�Ĵ���

  	for(byte_ctr=0;byte_ctr<bytes;byte_ctr++)
    	pBuf[byte_ctr] = SPI_RW(0);    								// ���Ӷ�������

  	RFCSN = 1;                          

  	return(status);                    							// ����RF״ֵ̬
}
/**************************************************
���ܣ�����Ϊ����ģʽ
**************************************************/
void PD_Mode(void)
{
		RFCE=0;
  	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0c);   				// PWR_UP=0
}
/**************************************************
���ܣ�����Ϊ����ģʽ
**************************************************/
void RX_Mode(void)
{
		RFCE=0;
  	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);   			// �ϵ�, CRCΪ2 bytes,����ģʽ,����RX_DR�����ж�
  	RFCE = 1; 																		// ��������ģʽ
}
/**************************************************
���ܣ�����Ϊ����ģʽ
**************************************************/
void TX_Mode(uint8_t *rf_data,uint8_t num)
{
		num = TX_PLOAD_WIDTH;
  	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);     													// �ϵ�, CRCΪ2 bytes,����ģʽ,����RX_DR�����ж�
		SPI_Write_Buf(WR_TX_PLOAD, rf_data, TX_PLOAD_WIDTH); 								// д���ݵ�FIFO
		RFCE=1;																															// ��������
		delay_us(15);																												// ��������
		RFCE=0;												
}
/**************************************************
���ܣ�RF��ʼ��
**************************************************/
void rf_init(void)
{
  	RFCE = 0;                                   											// RF�ر�
  	RFCKEN = 1;                                 											// ����RFʱ��
  	RF = 1;                                     											// ����RF�ж�

		delay_ms(10);

  	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    	// ���÷����ַ����
  	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); 	// ���ý��յ�ַ����

  	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      												// �����Զ�Ӧ����
  	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  												// PIPE0��������
  	SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); 												// �Զ��ش�10��
  	SPI_RW_Reg(WRITE_REG + RF_CH, 40);        												// RFƵ��2440MHz
  	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x0f);   												// ���书��0dBm, ��������2Mbps,
  	SPI_RW_Reg(WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH); 								// PIPE0 �������ݰ�����			
}
/**************************************************
���ܣ����ڳ�ʼ���ӳ���
˵����������19.2K��ʹ���ڲ������ʷ�����
**************************************************/
void uart_init(void)
{
    ES0 = 0;                      				// ��UART0�ж�
    REN0 = 1;                     				// �������
    SM0 = 0;                      				// ����ģʽ1��8bit�ɱ䲨����
    SM1 = 1;                   
    PCON |= 0x80;                 				// SMOD = 1
    ADCON |= 0x80;                				// ѡ���ڲ������ʷ�����

    S0RELL = 0xe6;                				// ������19.2K(ʮ����998=ʮ������0x03e6)
    S0RELH = 0x03;
    TI0 = 0;					  									// �巢����ɱ�־
		S0BUF=0x99;					  								// �ͳ�ֵ
}
/**************************************************
���ܣ��򴮿ڷ���1 byte����
**************************************************/
void uart_putchar(uint8_t x)
{
		while (!TI0);														// �ȴ��������
		TI0=0;																	// �巢����ɱ�־
		S0BUF=x;																// ��������
}
/**************************************************
����:I/O�ڳ�ʼ��
**************************************************/
void io_init(void)
{
  	P0DIR = 0xf0;							   	// �趨I/O���������
  	P1DIR = 0xff;					                 
} 
/**************************************************
����:LED���ã�LED9
**************************************************/
void led_init()
{
	P0DIR &= 0xBF;				//P06 out
	LED9 = 1;
}
/**************************************************
���ܣ�uart�жϷ������
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
���ܣ�RF�жϷ������
**************************************************/
void RF_IRQ(void) interrupt INTERRUPT_RFIRQ
{
		sta=SPI_Read(STATUS);																					// ����״ֵ̬
	
//		LED9 = !LED9;
		if(RX_DR)									
		{
			RFCE=0;																											//�ر�RF,�������ģʽ
			SPI_Read_Buf(RD_RX_PLOAD,radio_rx_buf,TX_PLOAD_WIDTH);			// ����FIFO������
			SPI_RW_Reg(FLUSH_RX,0);																			// ���RX��FIFO
			
			uart_tx_flag = 1;
		}
		
		if(MAX_RT)
	{
			SPI_RW_Reg(FLUSH_TX,0);							// ���TX��FIFO !!!�ﵽ����ش����������TX����buffer��ȷ������PID��һ�󣬱���������
  }
	
		SPI_RW_Reg(WRITE_REG+STATUS,0x70);														// ��������жϱ�־ 
}
/**************************************************
���ܣ�������
**************************************************/
void main(void)
{
		rf_init();																				// RF��ʼ�� 
		uart_init();																			// uart��ʼ��
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
						RFCE=0;																		//�ر�RF
						TX_Mode(uart_rx_buf,9);										// ��������
						while (!(TX_DS|MAX_RT));									// �ȴ��������
						sta = 0;
						PD_Mode();																// ��RF
						LED9 = !LED9;
						radio_tx_flag = 0;
				
						for(i = 0;i < 11;i++)
						{
						uart_rx_buf[i] = '\0';
						}
						uart_rx_num = 0;
			
						RX_Mode();																//�������ģʽ
					
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
//			RFCE=0;																				//�ر�RF
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



#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "predefine.h"
#include "usart.h"
#include "crc16.h"


#define TRUE    1
#define FALSE   0
#define BEGIN   1
#define END      0

uint8_t failue=0;
uint8_t ledType = 0;

/*标志位相关变量*/
uint8_t flag_LedON = END;    //是否开始点亮LED
uint8_t flag_NewData = FALSE;    //是否是新数据

/*定时器相关变量*/
uint16_t tLimt_rxTime=0;
uint32_t  tLedCount= 0;    //LED计时计数器

/*数组定义区*/
uint8_t rxUart0[20];   //串口0数据缓存区
uint8_t rxUart1[20];   //串口1数据缓存区
uint8_t FrameData[20];    //

/*函数声明区*/
void Led_Slake(void);
void SystemInit(void);
void Timer3_Init(void);
void Led_Display(uint8_t ledType);
uint8_t EEPROM_Read(uint16_t Addr);
void DealFrameData(uint8_t* p_FrameData);
void EEPROM_Write(uint16_t Addr,uint8_t Data);
void FrameProcess(uint8_t* dstArray, uint8_t type);
void EEPROM_Successive_Write(uint16_t Addr,uint8_t Data);
uint8_t DecodeProtocol(uint8_t* p_rxUart0, uint8_t* p_FrameData);


/**********************************************************************************************
* 函 数 名  ：main
* 功能说明：c程序入口
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
int main(void)
{
	SystemInit();
	EEPROM_Successive_Write(0x00,0x04);    //将EEPROM前500个字节填充成0x02
	
	while(1)
	{
		//USART0_TransmitByte(0xAA);
		//MPCM_USART0_TransmitByte(0xAA, 0xC1);
		//_delay_ms(100);
		if(flag_NewData)    //串口有新数据到来
		{
			//USART1_TransmitFrame(rxUart0);    //test used
			if(DecodeProtocol(rxUart0, FrameData))    //对数据进行帧校验
			{
				DealFrameData(FrameData);    //开始处理数据
				//USART0_TransmitFrame(FrameData);
			}
			else
			{
				//again
			}
			flag_NewData = FALSE;
		}
		Led_Display(ledType);
	}
	return 0;
}

/**********************************************************************************************
* 功能说明：串口0中断接收函数
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
ISR(USART0_RX_vect)
{
	volatile static uint8_t temp, countRx, NinthData;
	
	NinthData = (UCSR0B & (1<<RXB80));    //提取第9位数据，即RXB8的值
	temp = UDR0;    //若是先接收缓存数据，会改变UCSR1A和UCSR1B的值，因此在接收之前先保存好寄存器的值
	
	if((NinthData != 0) && (temp == 0xC1))    //数据是地址且为0xC1, 开始响应
	{
		UCSR0A&=~(1<<MPCM0);    //清零MPCM开始接收数据
		countRx = 0;
		return;
	}
	
	if(NinthData == 0)    //如果是数据
	{
		rxUart0[countRx++] = temp;
	}
	
	if(countRx == 17)    //一个完整的数据帧是17个字节
	{
		UCSR0A |= (1<<MPCM0);    //读取最后一个数据，置位MPCM等待下次被寻址
		flag_NewData = TRUE;
	}
}

/**********************************************************************************************
* 函 数 名  ：EEPROM_Write
* 功能说明：EEPROM写函数
* 输入参数：Addr--> 要写入的地址    Data--> 要写入的数据
* 返 回 值 ：void
**********************************************************************************************/
void EEPROM_Write(uint16_t Addr,uint8_t Data)
{
	while( EECR & ( 1<<EEPE ));   //等待上一次写操作结束
	EEAR = Addr;                              //写地址
	EEDR = Data;                             //写数据
	EECR |= (1<<EEMPE);             //开主机写使能
	EECR |= (1<<EEPE);                //开写使能
}

/**********************************************************************************************
* 函 数 名  ：EEPROM_Read
* 功能说明：EEPROM读函数
* 输入参数：Addr--> 要读取的地址
* 返 回 值 ：从Addr地址中读到的数据
**********************************************************************************************/
uint8_t EEPROM_Read(uint16_t Addr)
{
	while(EECR & (1<<EEPE));
	EEAR = Addr;
	EECR |= (1<<EERE);             //开读使能
	return EEDR;                        //返回读取的数据
}

/**********************************************************************************************
* 函 数 名  ：EEPROM_Successive_Write
* 功能说明：EEPROM连续写函数
* 输入参数：Addr--> 要写入的地址    Data--> 要写入的数据
* 返 回 值 ：从Addr地址中读到的数据
**********************************************************************************************/
void EEPROM_Successive_Write(uint16_t Addr,uint8_t Data)
{
	uint16_t i;
	for(i=0; i<500; i++)
	{
		EEPROM_Write(Addr+i,Data);
		_delay_ms(1);
	}
}

/**********************************************************************************************
* 函 数 名  ：Time3_Init
* 功能说明：定时器3初始化 TCNTn = 65536 - (FOSC/Prescaler)*Timing(s)
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
void Timer3_Init(void)
{
	TIFR3 = 0;    //定时器3的中断标志寄存器清零
	
	/*65536-(11059200/64)*0.1=48256=BC80*/
	TCNT3H = 0xBC;
	TCNT3L = 0x80;
	
	TCCR3B|=(1<<CS31)|(1<<CS30);   //64分频
	TIMSK3=(1<<TOIE3);    //定时器3溢出中断使能
}

/**********************************************************************************************
* 功能说明：定时器3溢出中断函数。100ms进入一次中断
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
ISR(TIMER3_OVF_vect)
{
	TCNT3H=0xBC;
	TCNT3L=0x80;
	
	tLimt_rxTime++;  //LED开始定时熄灭
	if(flag_LedON == BEGIN) tLedCount++;    //LED开始定时熄灭
}

/***************************************************************************************************************
* 函 数 名 ：DecodeProtocol
* 功能说明：对机感板发送过来的数据进行帧校验, 通过校验后将数据存入FrameData中
* 输入参数：p_rxUart0 --> rxUart0   p_FrameData --> FrameData
* 返 回 值 ：1 or 0
***************************************************************************************************************/
uint8_t DecodeProtocol(uint8_t* p_rxUart0, uint8_t* p_FrameData)
{
	uint8_t* p_Check = p_rxUart0;
	
	uint16_t check = Check_CRC16(p_Check, 15);    //将数据的前15个字节进行CRC16运算，与接收到的CRC16进行比较
	uint16_t CRC16 = p_rxUart0[15]<<8 | p_rxUart0[16];
	
	if((p_rxUart0[0] == 0xAA) && (check == CRC16))    //是AA格式的帧数据，且通过CRC16校验
	{
		memcpy(p_FrameData, p_rxUart0, 17);    //将rxUart0中的前17个数据存入FrameData中
		return 1;
	}
	
	return 0;
}

/**********************************************************************************************
* 函 数 名 ：DealFrameData
* 功能说明：处理帧数据, 有线数据传给CPU3, 无线数据传给CPU4
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
void DealFrameData(uint8_t* p_FrameData)
{
	uint16_t addr;
	
	addr = p_FrameData[7]<<8 | p_FrameData[8];    //取地址
	ledType = EEPROM_Read(addr);    //从EEPROM中读取该地址对应的灯号
	p_FrameData[10] = (p_FrameData[10]&0xF0)|ledType;    //补全无线数据的灯信号
	
	FrameProcess(p_FrameData, 0x41);    //修改type, 灯码后重新计算CRC的值
	MPCM_USART1_TransmitFrame(FrameData, 0x9B);    //发送给MsgDeal_1
	
	flag_LedON = BEGIN;    //开始亮灯
	tLedCount = 0;
}

/**********************************************************************************************
* 函 数 名  ：FrameProcess
* 功能说明：将源数组按照规则组成一帧数据
* 输入参数：dstArray -- 目的数组   type -- 帧类型
* 返 回 值 ：void
*********************************************************************************************/
void FrameProcess(uint8_t* dstArray, uint8_t type)
{
	uint8_t* p_dstArray = dstArray;
	uint16_t checkValue = 0;

	p_dstArray[1] = type;
	
	checkValue = Check_CRC16(dstArray, 15);
	p_dstArray[15] = (uint8_t)(checkValue>>8);    //CRC
	p_dstArray[16] = (uint8_t)(checkValue);    //CRC
}

/************************************************************************************************
* 函 数 名 ：Led_Display
* 功能说明：亮灯函数
* 输入参数：ledType
* 返 回 值 ：void
************************************************************************************************/
void Led_Display(uint8_t ledType)
{
	switch(ledType&0x0F)
	{
		case 0x01:    //白灯
		{
			PORTA = 0x04;
			if(tLedCount >= 30)
			{
				Led_Slake();
			}
			break;
		}
		case 0x02:    //蓝灯
		{
			PORTA=0x02;
			if(tLedCount >= 30)
			{
				Led_Slake();
			}
			break;
		}
		case 0x03:    //红灯
		{
			PORTA=0x01;
			if(tLedCount >= 30)
			{
				Led_Slake();
			}
			break;
		}
		default: break;
	}
}


/************************************************************************************************
* 函 数 名 ：Led_Slake
* 功能说明：灭灯函数，亮灯3s后开始执行此函数
* 输入参数：void
* 返 回 值 ：void
************************************************************************************************/
void Led_Slake(void)
{
	//USART0_TransmitString("OK\n");    //test
	PORTA=0x08;    //亮绿灯
	tLedCount = 0;
	flag_LedON = END;
	ledType = 0x00;
}

/************************************************************************************************
* 函 数 名 ：SystemInit
* 功能说明：系统初始化函数 串口0初始化、串口1初始化、定时器3初始化、开中断
* 输入参数：void
* 返 回 值 ：void
************************************************************************************************/
void SystemInit(void)
{
	DDRA=0xff;
	USART0_Init(115200,1);
	USART1_Init(115200,1);
	Timer3_Init();
	sei();
}

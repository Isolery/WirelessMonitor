#include <avr/io.h>
#include "predefine.h"
#include "usart.h"

/**********************************************************************************************
* 函 数 名  ：USART0_Init
* 功能说明：串口0初始化函数  UBRRn = FOSC/16/baudRate - 1
* 输入参数：baudRate --> 波特率  MPCMn --> 0 工作在普通模式  1 工作在MPCM模式
* 返 回 值 ：void
**********************************************************************************************/
void USART0_Init(uint32_t baudRate, uint8_t MPCMn)
{
    uint16_t MYUBRRn = FOSC/16/baudRate-1;
		
	UBRR0H = (uint8_t)(MYUBRRn>>8);
	UBRR0L  = (uint8_t)MYUBRRn;
	UCSR0A|=MPCMn;     //U2X=0 | MPCM=0 or 1
	if(MPCMn) 
	{
		UCSR0B|= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0)|(1<<UCSZ02);  
	}
	else 
	{
		UCSR0B|= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);  //接收中断、接收、发送使能 
	}
	UCSR0C|= (1<<UCSZ01)|(1<<UCSZ00);    //数据位：8+MPCNn | 停止位：1 | 校验位：NONE
}

/**********************************************************************************************
* 函 数 名  ：USART0_TransmitByte
* 功能说明：串口0发送一个字节
* 输入参数：data --> 发送的字节
* 返 回 值 ：void
**********************************************************************************************/
void USART0_TransmitByte(uint8_t data)
{
    /* Wait for empty transmit buffer */
	while(!(UCSR0A&(1<<UDRE0)));    
	/* Put data into buffer, sends the data */
	UDR0=data;
}

/**********************************************************************************************
* 函 数 名  ：USART0_TransmitString
* 功能说明：串口0发送一个字符串
* 输入参数：cString --> 发送的字符串
* 返 回 值 ：void
**********************************************************************************************/
void USART0_TransmitString(const uint8_t* cString)
{
	 for(; *cString!='\0'; )
	 {
        while(!(UCSR0A&(1<<UDRE0)));    
		UDR0 = *cString++;  
	 }
}

/**********************************************************************************************
* 函 数 名  ：USART0_TransmitArray
* 功能说明：串口0发送一个数组
* 输入参数：Array --> 发送的数组    len --> 数组的长度
* 返 回 值 ：void
**********************************************************************************************/
void USART0_TransmitArray(const uint8_t* Array, uint8_t len)
{
	uint8_t i;
	for(i=0; i<len; i++)
	{
		while(!(UCSR0A&(1<<UDRE0)));
		UDR0 = *Array++;
	}
}

/********************************************************************************************************************************
* 函 数 名  ：USART0_TransmitFrame
* 功能说明：串口0发送一个帧数数据，帧数据的第3个字节代表后面数据的长度，加上帧头，type，CRCx2 共5个字节
* 输入参数：Frame --> 帧数组
* 返 回 值 ：void
*********************************************************************************************************************************/
void USART0_TransmitFrame(const uint8_t* Frame)
{
	uint8_t len = *(Frame+2) + 5;
	while(len--)
	{
		while(!(UCSR0A&(1<<UDRE0)));
		UDR0 = *Frame++;
	}
}

/**********************************************************************************************
* 函 数 名  ：MPCM_USART0_TransmitByte
* 功能说明：串口0工作在MPCM模式，串口0发送一个字节
* 输入参数：data --> 发送的字节  addr --> 地址
* 返 回 值 ：void
**********************************************************************************************/
void MPCM_USART0_TransmitByte(uint8_t data, uint8_t addr)
{
	UCSR0B|=(1<<TXB80);    //第9位写1表示地址
    UDR0=addr;     //发送第一个数据
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	while(!(UCSR0A&(1<<UDRE0)));    
	UCSR0B&=~(1<<TXB80);    //第9位写0表示数据 
	UDR0=data;
}

/**********************************************************************************************
* 函 数 名  ：MPCM_USART0_TransmitString
* 功能说明：串口0发送一个字符串
* 输入参数：cString --> 发送的字符串  addr --> 地址
* 返 回 值 ：void
**********************************************************************************************/
void MPCM_USART0_TransmitString(const uint8_t* cString, uint8_t addr)
{
	UCSR0B|=(1<<TXB80);    //第9位写1表示地址
    UDR0=addr;     //发送第一个数据
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	for(; *cString!='\0'; )
	 {
		while(!(UCSR0A&(1<<UDRE0)));    
		UCSR0B&=~(1<<TXB80);    //第9位写0表示数据 
		UDR0 = *cString++;  
	 }
}

/**********************************************************************************************
* 函 数 名  ：MPCM_USART0_TransmitArray
* 功能说明：串口0发送一个数组
* 输入参数：Array --> 发送的数组    len --> 数组的长度
* 返 回 值 ：void
**********************************************************************************************/
void MPCM_USART0_TransmitArray(const uint8_t* Array, uint8_t len, uint8_t addr)
{
	uint8_t i;
	UCSR0B|=(1<<TXB80);    //第9位写1表示地址
	UDR0=addr;     //发送第一个数据
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	for(i=0; i<len; i++)           
	{   
		while(!(UCSR0A&(1<<UDRE0)));   
		UCSR0B&=~(1<<TXB80);    //第9位写0表示数据 
		UDR0 = *Array++;  
	}
}

/********************************************************************************************************************************
* 函 数 名  ：MPCM_USART0_TransmitFrame
* 功能说明：串口0发送一个帧数数据，帧数据的第3个字节代表后面数据的长度，加上帧头，type，CRCx2 共5个字节
* 输入参数：Frame --> 帧数组
* 返 回 值 ：void
*********************************************************************************************************************************/
void MPCM_USART0_TransmitFrame(const uint8_t* Frame, uint8_t addr)
{
	uint8_t len = *(Frame+2) + 5;
	UCSR0B|=(1<<TXB80);    //第9位写1表示地址
	UDR0=addr;     //发送第一个数据
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	while(len--)          
	{
		while( !(UCSR0A & (1<<UDRE0)) );  
		UCSR0B&=~(1<<TXB80);    //第9位写0表示数据   
		UDR0 = *Frame++;  
	}
}

/**********************************************************************************************
* 函 数 名  ：USART1_Init
* 功能说明：串口1初始化函数  UBRRn = FOSC/16/baudRate - 1
* 输入参数：baudRate --> 波特率  MPCMn --> 0 工作在普通模式  1 工作在MPCM模式
* 返 回 值 ：void
**********************************************************************************************/
void USART1_Init(uint32_t baudRate, uint8_t MPCMn)
{
    uint16_t MYUBRRn = FOSC/16/baudRate-1;
		
	UBRR1H = (uint8_t)(MYUBRRn>>8);
	UBRR1L  = (uint8_t)MYUBRRn;
	UCSR1A|=MPCMn;     //U2X=0 | MPCM=0 or 1
	if(MPCMn) 
	{
		UCSR1B|= (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1)|(1<<UCSZ12);  
	}
	else 
	{
		UCSR1B|= (1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1);  //接收中断、接收、发送使能 
	}
	UCSR1C|= (1<<UCSZ11)|(1<<UCSZ10);    //数据位：8+MPCNn | 停止位：1 | 校验位：NONE
}

/**********************************************************************************************
* 函 数 名  ：USART1_TransmitByte
* 功能说明：串口1发送一个字节
* 输入参数：data --> 发送的字节
* 返 回 值 ：void
**********************************************************************************************/
void USART1_TransmitByte(uint8_t data)
{
    /* Wait for empty transmit buffer */
	while(!(UCSR1A&(1<<UDRE1)));    
	/* Put data into buffer, sends the data */
	UDR1=data;
}

/**********************************************************************************************
* 函 数 名  ：USART1_TransmitString
* 功能说明：串口0发送一个字符串
* 输入参数：cString --> 发送的字符串
* 返 回 值 ：void
**********************************************************************************************/
void USART1_TransmitString(const uint8_t* cString)
{
	for(; *cString!='\0'; )
	{
		while(!(UCSR1A&(1<<UDRE1)));    
		UDR1 = *cString++;  
		//if(*cString == '\0') break;   //字符串最后一位
	}
}

/**********************************************************************************************
* 函 数 名  ：USART1_TransmitArray
* 功能说明：串口0发送一个数组
* 输入参数：Array --> 发送的数组    len --> 数组的长度
* 返 回 值 ：void
**********************************************************************************************/
void USART1_TransmitArray(const uint8_t* Array, uint8_t len)
{
	uint8_t i;
	for(i=0; i<len; i++)          
	{
		while(!(UCSR1A&(1<<UDRE1)));    
		UDR1 = *Array++;  
	}
}

/********************************************************************************************************************************
* 函 数 名  ：USART1_TransmitFrame
* 功能说明：串口1发送一个帧数数据，帧数据的第3个字节代表后面数据的长度，加上帧头，type，CRCx2 共5个字节
* 输入参数：Frame --> 帧数组
* 返 回 值 ：void
*********************************************************************************************************************************/
void USART1_TransmitFrame(const uint8_t* Frame)
{
	uint8_t len = *(Frame+2) + 5;
	while(len--)          
	{
		while(!(UCSR1A&(1<<UDRE1)));    
		UDR1 = *Frame++;  
	}
}

/**********************************************************************************************
* 函 数 名  ：MPCM_USART1_TransmitByte
* 功能说明：串口1发送一个字节
* 输入参数：data --> 发送的字节  addr --> 地址
* 返 回 值 ：void
**********************************************************************************************/
void MPCM_USART1_TransmitByte(uint8_t data, uint8_t addr)
{
	UCSR1B|=(1<<TXB81);    //第9位写1表示地址
	UDR1=addr;     //发送第一个数据
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	while( !(UCSR1A & (1<<UDRE1)) );    
	UCSR1B&=~(1<<TXB81);    //第9位写0表示数据 
	UDR1=data;
}

/**********************************************************************************************
* 函 数 名  ：MPCM_USART1_TransmitString
* 功能说明：串口1发送一个字符串
* 输入参数：cString --> 发送的字符串  addr --> 地址
* 返 回 值 ：void
**********************************************************************************************/
void MPCM_USART1_TransmitString(const uint8_t* cString, uint8_t addr)
{
	UCSR1B|=(1<<TXB81);    //第9位写1表示地址
	UDR1=addr;     //发送第一个数据
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	for(; *cString!='\0'; )
	{
		while(!(UCSR1A&(1<<UDRE1)));    
		UCSR1B&=~(1<<TXB81);    //第9位写0表示数据 
		UDR1 = *cString++;  
		//if(*cString == '\0') break;   //字符串最后一位
	}
}

/**********************************************************************************************
* 函 数 名  ：MPCM_USART1_TransmitArray
* 功能说明：串口1发送一个数组
* 输入参数：Array --> 发送的数组    len --> 数组的长度
* 返 回 值 ：void
**********************************************************************************************/
void MPCM_USART1_TransmitArray(const uint8_t* Array, uint8_t len, uint8_t addr)
{
	uint8_t i;
	UCSR1B|=(1<<TXB81);    //第9位写1表示地址
	UDR1=addr;     //发送第一个数据
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	for(i=0; i<len; i++)           
	{   
		while( !(UCSR1A & (1<<UDRE1)) );   
		UCSR1B&=~(1<<TXB81);    //第9位写0表示数据 
		UDR1 = *Array++;  
	}
}

/********************************************************************************************************************************
* 函 数 名  ：MPCM_USART1_TransmitFrame
* 功能说明：串口1发送一个帧数数据，帧数据的第3个字节代表后面数据的长度，加上帧头，type，CRCx2 共5个字节
* 输入参数：Frame --> 帧数组
* 返 回 值 ：void
*********************************************************************************************************************************/
void MPCM_USART1_TransmitFrame(const uint8_t* Frame, uint8_t addr)
{
	uint8_t len = *(Frame+2) + 5;
	UCSR1B|=(1<<TXB81);    //第9位写1表示地址
	UDR1=addr;     //发送第一个数据
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	while(len--)          
	{
		while( !(UCSR1A&(1<<UDRE1)) );  
		UCSR1B&=~(1<<TXB81);    //第9位写0表示数据   
		UDR1 = *Frame++;  
	}
}

/********************************************************************************************************************************
* 函 数 名  ：MPCM_USART1_TransmitMonitorFrame
* 功能说明：串口1发送一个帧数数据，帧数据的第3个字节代表后面数据的长度，加上帧头，type，CRCx2 共5个字节
* 输入参数：Frame --> 帧数组
* 返 回 值 ：void
*********************************************************************************************************************************/
void MPCM_USART1_TransmitMonitorFrame(const uint8_t* Frame)
{
	uint8_t i;
		
	UCSR1B|=(1<<TXB81);    //第9位写1表示地址
	UDR1 = Frame[0];     //发送第一位数据，表示地址
	asm("nop"); asm("nop"); asm("nop"); asm("nop"); 
	for(i=1; i<17; i++)      
	{
		while( !(UCSR1A&(1<<UDRE1)) );  
		UCSR1B&=~(1<<TXB81);    //第9位写0表示数据   
		UDR1 = Frame[i];    //  
	}
}

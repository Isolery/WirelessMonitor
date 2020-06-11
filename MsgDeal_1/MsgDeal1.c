#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <util/twi.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usart.h"
#include "predefine.h"
#include "usart.h"
#include "crc16.h"

#define ZERO      0x01
#define BACK      0x02
#define FRONT    0x04

#define TRUE     1
#define FALSE    0
#define BEGIN    1
#define END       0
#define VALID    1
#define INVALID 0

#define ACK()     (TWCR = (1<<TWINT)|(1<<TWEA)|(1<<TWEN)|(1<<TWIE))
#define NACK()  (TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWIE))

uint8_t lastdirect;  
uint8_t *pHandle = (uint8_t *)(0x0200);    //保存手柄方向  0x04 --> 前向  0x02 --> 后向
uint8_t ledType = 0; 

/*标志位相关变量*/
uint8_t NewData_Uart0 = FALSE;    //串口0是否接收到新数据
uint8_t NewData_Uart1 = FALSE;    //串口1是否接收到新数据
uint8_t NewData_TWI = FALSE;    //是否是新数据
uint8_t flag_LedON = END;    //是否开始点亮LED
uint8_t CheckCorrect = FALSE;    //串口1接收的数据是否正确


/*定时器相关变量*/
uint32_t  tLedCount = 0;    //LED计时计数器

/*数组定义区*/
uint8_t rxTWI[17];      //TWI接收缓存区
uint8_t rxUart0[20];   //串口0数据缓存区，与CPU1，CPU4进行通信
uint8_t rxUart1[20];   //串口1数据缓存区，与监控主机进行通信
uint8_t MonitorFrameData[20];    //保存从监控主机传过来的一帧数据
uint8_t MCUFrameData[20];    //保存从CPU1或CPU4传过来的一帧数据
uint8_t TransmitToMonitor[16];    //发送给监控主机
uint8_t ReceiveFromMonitor[16];    //从监控主机接收数据



/*函数声明区*/
void TWI_Init();
void GPIO_Init(void);
void Led_Slake(void);
void SystemInit(void);
void Timer3_Init(void);
void Led_Display(uint8_t ledType);
uint8_t CheckSum(uint8_t* srcArray, uint8_t len);
uint8_t CalculateSum(uint8_t* srcArray, uint8_t len);
void DealMonitorFrameData(uint8_t* pMonitorFrameData);
uint8_t MCUProtocolCheck(uint8_t* prxUart0, uint8_t* pMCUFrameData);
uint8_t MonitorProtocolCheck(uint8_t* prxUart1, uint8_t* pMonitorFrameData);
void Prepare_Transmit(uint8_t* pTransmitToMonitor, uint8_t* pMCUFrameData);

/**********************************************************************************************
* 函 数 名  ：main
* 功能说明：C程序入口
* 输入参数：void
* 返 回 值 ：0
*********************************************************************************************/
int main(void)
{
    SystemInit(); 
		
	while(1)
	{
		if(NewData_Uart0)    //串口0有新数据到来
		{
			if(MCUProtocolCheck(rxUart0, MCUFrameData) == VALID)
			{
				//USART0_TransmitArray(MCUFrameData, 17);
				ledType = MCUFrameData[10];
				flag_LedON = BEGIN;
				tLedCount = 0;
			}
			NewData_Uart0 = FALSE;
		}
		
		if(NewData_TWI)    //TWI接收到新数据
		{
			//USART1_TransmitArray(rxTWI, sizeof(rxTWI));
			
			if(MCUProtocolCheck(rxTWI, MCUFrameData) == VALID)
			{
				ledType = MCUFrameData[10];
				flag_LedON = BEGIN;
				tLedCount = 0;
			}
			NewData_TWI = FALSE;
		}
			
		if(NewData_Uart1)    //串口1有新数据到来
		{
			if(MonitorProtocolCheck(rxUart1, MonitorFrameData) == VALID)    //对监控主机的数据进行帧校验    
			{
				CheckCorrect = TRUE;
				DealMonitorFrameData(MonitorFrameData);    //开始处理数据
			}
			NewData_Uart1 = FALSE;
			CheckCorrect = FALSE;			
		}			
		Led_Display(ledType);    //点亮LED
	}
	return 0;
}

/**********************************************************************************************
* 功能说明：串口0中断接收函数, 与CPU1或CPU4进行通讯
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
ISR(USART0_RX_vect)
{ 
    volatile static uint8_t temp, countRx, NinthData;
		
	NinthData = (UCSR0B & (1<<RXB80));    //提取第9位数据，即RXB8的值
	temp = UDR0;    //若是先接收缓存数据，会改变UCSR1A和UCSR1B的值，因此在接收之前先保存好寄存器的值 
		
	if((NinthData != 0) && (temp == 0x9B))    //数据是地址且为0x9B, 开始响应
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
		NewData_Uart0 = TRUE;
	}
} 

/**********************************************************************************************
* 功能说明：串口1中断接收函数, 与监控主机进行通讯
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
ISR(USART1_RX_vect)
{ 
    volatile static uint8_t  temp, countRx, NinthData;
    
	NinthData = (UCSR1B & (1<<RXB81));    //提取第9位数据，即RXB8的值
	temp = UDR1;    //若是先接收缓存数据，会改变UCSR1A和UCSR1B的值，因此在接收之前先保存好寄存器的值 
		
	if((NinthData != 0) && (temp == 0x9B))    //数据是地址且为0x9B, 开始响应
	{
		UCSR1A&=~(1<<MPCM1);    //清零MPCM开始接收数据
		countRx = 0;
		rxUart1[countRx++] = temp;    //为了后面方便起见，将地址也存入数组中
		return;
	}
		
	if(NinthData == 0)    //如果是数据
	{
		rxUart1[countRx++] = temp;
	}
		
	if(countRx == 17)
	{
		UCSR1A |= (1<<MPCM1);    //读取最后一个数据，置位MPCM等待下次被寻址
		NewData_Uart1 = TRUE;
	}
}

/**********************************************************************************************
* 功能说明：定时器3溢出中断函数  2ms进入一次中断
* 输入参数：void
* 返 回 值 ：void
*********************************************************************************************/
ISR(TIMER3_OVF_vect)
{
    TCNT3H=0xFE;      
    TCNT3L=0xA6;    
		
	if(flag_LedON == BEGIN) tLedCount++;    //LED开始定时熄灭
}

/******************************************************************************************
* 功能说明：TWI中断函数，SR模式
* 输入参数：void
* 返 回 值 ：void
******************************************************************************************/
ISR(TWI_vect)
{
	volatile static uint8_t countRx, temp;
	switch(TW_STATUS)
	{
		case TW_SR_SLA_ACK:                         //被主机SLA+W寻址，返回ACK
		case TW_SR_ARB_LOST_SLA_ACK:        //作为主机仲裁失败且SLA+W被接收，返回ACK
		case TW_SR_GCALL_ACK:                     //接收到广播地址，返回ACK
		case TW_SR_ARB_LOST_GCALL_ACK:    //作为主机仲裁失败且接收到广播地址，返回ACK
		{
			countRx = 0;
			ACK(); break;   //返回ACK
		}
		
		case TW_SR_DATA_ACK:    //被SLA+W寻址, 数据已接收, 产生ACK
		case TW_SR_GCALL_DATA_ACK:    //被广播寻址, 数据已接收, 产生ACK
		{
			temp = TWDR;
			rxTWI[countRx++] = temp;
			if(countRx == 17)
			{
				NewData_TWI = TRUE;
			}
			ACK(); break;   //返回ACK
		}
		
		case TW_SR_DATA_NACK:    //被SLA+W寻址, 数据已接收, 产生NACK
		case TW_SR_GCALL_DATA_NACK:    //被广播寻址, 数据已接收, 产生NACK
		NACK(); break;    //返回NACK
		
		case TW_SR_STOP:    //在以从机工作时接收到STOP或重复START
		ACK(); break;    //返回ACK
		
		default: break;
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
	ETIFR = 0;    //定时器3的中断标志寄存器清零
	
	/*65536-(11059200/64)*0.002=65190=FEA6*/
	TCNT3H = 0xFE;
	TCNT3L = 0xA6;
	
	TCCR3B |= (1<<CS31)|(1<<CS30);   //64分频
	ETIMSK = (1<<TOIE3);    //定时器3溢出中断使能
}
/******************************************************************************************
* 函 数 名  ：TWI_Init
* 功能说明：TWI初始化 --> 波特率、TWAR、TWCR
* 输入参数：void
* 返 回 值 ：void
*******************************************************************************************/
void TWI_Init()
{
	DDRD &=~((1<<0)|(1<<1)); PORTD |= (1<<0)|(1<<1);    //设置SLC，SDA内部上拉，若硬件有上拉电阻，这行代码可以不用
	TWAR = 0xA0;
	TWSR = 0;    //预分频设置为0
	TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE);    //使能TWI接口、TWI应答、TWI中断; 其余位清零
	sei();
}

/************************************************************************************************
* 函 数 名 ：SystemInit
* 功能说明：系统初始化函数 串口0初始化、串口1初始化、定时器3初始化、开中断
* 输入参数：void
* 返 回 值 ：void
************************************************************************************************/
void SystemInit(void)
{
    GPIO_Init();
	USART0_Init(115200,1);    //串口0初始化，波特率：115200，工作方式：MPCM
	USART1_Init(115200,1);    //串口1初始化，波特率：115200，工作方式：MPCM
	Timer3_Init();    //定时器3初始化，每2ms进入一次中断
	TWI_Init();
	sei();
}

/************************************************************************************************
* 函 数 名 ：GPIO_Init
* 功能说明：端口引脚初始化
* 输入参数：void
* 返 回 值 ：void
************************************************************************************************/
void GPIO_Init(void)
{
    DDRA = 0xFF;
	PORTA &= ~(1<<7);    //Bit7 :  0 --> 前向  1--> 后向    初始手柄方向为前向
	DDRE |=(1<<3); 
	PORTE &= ~(1<<3);    //串口转485模块配置为接收状态
		
	if(*pHandle == 0x02)    //如果复位前手柄方向是后向,但目前是零位.则把手柄调到后向
	{
		PORTA |= (1<<7);
	}
		
	if(*pHandle == 0x04)    //如果复位前手柄方向是前向,但目前是零位.则把手柄调到前向
	{
		PORTA &= ~(1<<7); 
	}
}

/***************************************************************************************************************
* 函 数 名 ：MCUProtocolCheck
* 功能说明：对机感板或通讯板发送过来的数据进行帧校验, 通过校验后将数据存入MCUFrameData中
* 输入参数：prx --> rxUart0 or rxTWI    pMCUFrameData --> MCUFrameData
* 返 回 值 ：1 or 0
***************************************************************************************************************/
uint8_t MCUProtocolCheck(uint8_t* prx, uint8_t* pMCUFrameData)
{
	uint8_t* p = prx;
		
	uint16_t check = Check_CRC16(p, 15);    //将数据的前15个字节进行CRC16运算，与接收到的CRC16进行比较   
	uint16_t CRC16 = prx[15]<<8 | prx[16];
		
	if((prx[0] == 0xAA) && (check == CRC16))    //是AA格式的帧数据，且通过CRC16校验
	{
		memcpy(pMCUFrameData,prx,17);    //将rxUart0中的前17个数据存入FrameData中 
		return VALID;
	}
		
	return INVALID;
}

/***************************************************************************************************************
* 函 数 名 ：MonitorProtocolCheck
* 功能说明：对监控主机发送过来的数据进行帧校验, 通过校验后将数据存入MonitorFrameData中
* 输入参数：prxUart1 --> rxUart1   pMonitorFrameData --> MonitorFrameData
* 返 回 值 ：1 or 0
***************************************************************************************************************/
uint8_t MonitorProtocolCheck(uint8_t* prxUart1, uint8_t* pMonitorFrameData)
{
    uint8_t check = 0;
	uint8_t* p = prxUart1;
		
	check = CheckSum(p, 17);
	if( ((prxUart1[1] == 0xA0) || (prxUart1[1] == 0x50)) && (check == 0) )
	{
		memcpy(pMonitorFrameData, prxUart1, 17);    //校验通过后，将rxUart1中的前17位数据帧保存到MonitorFrameData中
		return VALID;    //返回数据有效
	}
	else
	{
		return INVALID;    //返回数据无效
	}
}

/***************************************************************************************************************
* 函 数 名 ：CheckSum
* 功能说明：通过校验和检查数据是否正确，并返回校验后的结果
* 输入参数：dstArray --> 校验的数组首地址    len --> 参加校验的长度 
* 返 回 值 ：uint8_t sum
***************************************************************************************************************/
uint8_t CheckSum(uint8_t* srcArray, uint8_t len)
{
    uint8_t* pCheck = srcArray;
	uint8_t sum = 0;    //主机将地址也放在校验和中
		
	while(len--)
	{
		sum += *pCheck++;
	}
	return sum;
}

/***************************************************************************************************************
* 函 数 名 ：CalculateSum
* 功能说明：计算校验和并返回计算后的结果
* 输入参数：srcArray --> 计算校验和的数组首地址    len --> 参加计算的长度  
* 返 回 值 ：uint8_t  校验和
***************************************************************************************************************/
uint8_t CalculateSum(uint8_t* srcArray, uint8_t len)
{
    uint8_t* p = srcArray;
	uint8_t i, sum = 0;    //定义变量注意要初始化
		
	for(i=0; i<len; i++)
	{
		sum += *p++;
	}
	sum = (~sum) + 1;
	return sum;
}

/***************************************************************************************************************
* 函 数 名 ：DealMonitorFrameData
* 功能说明：处理从监控主机接收到的数据转而向监控主机发送数据
* 输入参数：pMonitorFrameData --> MonitorFrameData
* 返 回 值 ：void
***************************************************************************************************************/
void DealMonitorFrameData(uint8_t* pMonitorFrameData)
{
    uint8_t HandleDirect = pMonitorFrameData[2]&0x07;    //手柄方向
	/*bit2 - FRONT  bit1 - BACK  bit0 -- ZERO*/
	switch(HandleDirect)
	{
		case BACK :    //后退
		{
			PORTA |= 0x80;    //通过硬件端口来实现前后向的切换功能
			lastdirect = BACK;
			//*pHandle = BACK;
			*pHandle = 0xAA;
			break;
		}
			
		case FRONT :
		{
			PORTA&=0x7F;
			lastdirect = FRONT;
			*pHandle = FRONT;
			break;
		}
			
		default: break;
	}	
	Prepare_Transmit(TransmitToMonitor, MCUFrameData);     //按照监控主机的数据帧格式将要发送的数据放在TransmitToMonitor中
	PORTE |= (1<<3);    //串口转485模块配置为发送状态
	_delay_ms(1);
	MPCM_USART1_TransmitMonitorFrame(TransmitToMonitor);   //发送给监控主机
	PORTE &= ~(1<<3);    //串口转485模块配置为接收状态
	_delay_ms(1);
}

/***************************************************************************************************************
* 函 数 名 ：Prepare_Transmit
* 功能说明：补充TransmitToMonitor数组，准备发送给监控主机
* 输入参数：pTransmitToMonitor --> TransmitToMonitor  pMCUFrameData --> MCUFrameData
* 返 回 值 ：void
***************************************************************************************************************/
void Prepare_Transmit(uint8_t* pTransmitToMonitor, uint8_t* pMCUFrameData)
{
    uint8_t i;
	uint8_t* p = pTransmitToMonitor;
		
	pTransmitToMonitor[0] = 0x3B;    //Monitor Address
		
	if(CheckCorrect)
	{
		pTransmitToMonitor[1] = 0xCF;
	}
	else
	{
		pTransmitToMonitor[1] = 0x5F;
	}
		
	pTransmitToMonitor[2] = pMCUFrameData[11];    //距离
	pTransmitToMonitor[3] = 0;    //距离
		
	pTransmitToMonitor[4] = pMCUFrameData[10];  //地面信号状态及场号
		
	pTransmitToMonitor[5] = pMCUFrameData[13];    //地面点序号
		
	pTransmitToMonitor[6] = pMCUFrameData[12];    
	pTransmitToMonitor[7] = pMCUFrameData[14];    //车站号
		
	for(i=8; i<16; i++)
	{
		pTransmitToMonitor[i] = 0;
	}
		
	memset(pMCUFrameData, '\0', 17);    //每次调用MCUFrameData后都要清空该数组，避免相同的数据重复上传
	pTransmitToMonitor[16] = CalculateSum(p, 15);
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
			if(tLedCount >= 1500)
			{
				Led_Slake();
			}
			break;
		}
		case 0x02:    //蓝灯
		{
			PORTA=0x02;
			if(tLedCount >= 1500)
			{
				Led_Slake();
			}
			break;
		}
		case 0x03:    //红灯
		{
			PORTA=0x01;
			if(tLedCount >= 1500)
			{
				Led_Slake();
			}
			break;
		}
		case 0x04:    //故障灯
		{
			PORTA=0x01;    //暂时没定义
			if(tLedCount >= 1500)
			{
				Led_Slake();
			}
		}
		case 0x0D:    //自动开车键PA1
		{
			;    //暂时没定义
			if(tLedCount >= 1500)
			{
				Led_Slake();
			}
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
	PORTA=0x08;    //亮绿灯
	tLedCount = 0;
	flag_LedON = END;
	ledType = 0x00;
}
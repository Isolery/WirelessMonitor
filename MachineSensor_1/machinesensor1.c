#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"
#include "predefine.h"
#include "usart.h"
#include "crc16.h"
#include <util/delay.h>
#include <avr/interrupt.h>

#define UART1_EN         UCSR1B|= (1<< RXEN1)
#define UART1_DEN       UCSR1B&=~(1<< RXEN1)

#define UART0_EN          UCSR0B|=(1<< RXEN0)
#define UART0_DEN        UCSR0B&=~(1<< RXEN0)

//定时清除存储的点
#define CLEAR_GAP0      30000         //收到预告点后计时1min       CLEAR_GAP0*2 ms
#define CLEAR_GAP1      500            //收到正向码后计时1s            CLEAR_GAP1*2 ms
 /*CLEAR_GAP2 = 90000, 为调试方便修改成如下*/
#define CLEAR_GAP2      5000          //收到正向码后计时3min        CLEAR_GAP2*2 ms 
#define CLEAR_GAP3      3500          //收到反向码后计时7s             CLEAR_GAP3*2 ms
#define CLEAR_GAP4      30000        //收到反向码后计时数1min     CLEAR_GAP4*2 ms

#define TRUE    1
#define FALSE   0
#define BEGIN   1
#define END      0

uint8_t ledType = 0;    

/*标志位相关变量*/
uint8_t flag_NewData = FALSE;    //是否是新数据
uint8_t flag_Store = END;    //是否开始存储串口数据
uint8_t flag_LedON = END;    //是否开始点亮LED
uint8_t flag_RxFinish = FALSE;
uint8_t clearFlag_YGD = END;
uint8_t clearFlag_Forward_DD = END;
uint8_t clearFlag_Rear_DD = END;

/*定时器相关变量*/
uint32_t  tLedCount= 0;    //LED计时计数器
uint32_t  tCountYGD  = 0;			   //预告点计时计数器 
uint32_t  tCountFD  = 0;			    //正向计时计数器 
uint32_t  tCountRD  = 0;			   //反向计时计数器  

/*数组定义区*/
uint8_t FrameData[20];    //保存从机感传过来的一帧数据
uint8_t rxUart0[20];    //串口0数据缓存区, 接收RLM传输的数据
uint8_t rxUart1[20];   //串口1数据缓存区
uint8_t EpcData[12] = {0xBB,0xE1,0x0,0x0,0x0,0x0,0x02,0x51,0x32,0x44,0x02,0x53};   //从RLM帧数据中提取有用的数据, E1 -- 有线   E0 -- 无线  
uint8_t storeFdirYGD[13];              //存储正向预告点1                      
uint8_t storeFdirYGD2[13];            //存储正向预告点2                       
uint8_t storeRedirDD[13];             //存储反向定点                         
uint8_t storeLastportdata[13];       //存储上次上报的定点数据     

uint8_t test[] = {0xAA, 0x11, 0x0C, 0xBB, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x02, 0x52, 0x32, 0x44, 0x02, 0x53, 0xE5, 0x2D};   


/*函数声明区*/
void Led_Slake(void);
void SystemInit(void);
void Time_up_Clear(void);
void Deal_RLM_Data(void);
void Led_Display(uint8_t ledType);
void RuleCheck(const uint8_t* p_EpcData);
uint8_t DecodeProtocol(const uint8_t* p_rxUart0, uint8_t* p_EpcData);
void FrameProcess(uint8_t* dstArray, const uint8_t* srcArray, uint8_t type, uint8_t len);
void check_preportdata(uint8_t* p_EpcData, uint8_t* p_storeRedirDD, uint8_t* p_storeFdirYGD, uint8_t* p_storeFdirYGD2);
uint8_t check_portdata(uint8_t* p_EpcData, uint8_t* p_storeFdirYGD, uint8_t* p_storeFdirYGD2, uint8_t* p_storeRedirDD, uint8_t* p_storeLastportdata);


/**********************************************************************************************
* 函 数 名  ：main
* 功能说明：c程序入口
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
int main(void)
{
	SystemInit(); 
	
	while(1)
	{
		if(flag_RxFinish)    //串口0有新数据到来
		{
			UART0_DEN;
			
			Deal_RLM_Data();	
			flag_RxFinish = FALSE;
			
			UART0_EN;
		}
		else
		{
			_delay_ms(100);
		}
		Time_up_Clear();    //定时清0存储的数据
		Led_Display(ledType);    //点亮LED灯，只在有线信号时有用 
	}
	return 0;
}

/**********************************************************************************************
* 功能说明：串口0中断接收函数
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
ISR(USART0_RX_vect)
{
	volatile static uint8_t temp, lastdata, countRx;
	temp = UDR0;
		
	switch(temp)
	{
		case 0xAA:
		{
			if(lastdata == 0xFF)
			{
				rxUart0[countRx++] = temp;
				lastdata = temp;
			}
			else
			{
				countRx = 0;
				rxUart0[countRx++] = temp;
				lastdata = temp;
			}
			break;
		}
				
		case 0xFF:
		{
			if(lastdata == 0xFF)
			{
				rxUart0[countRx++] = temp;
			}
			else
			{
				lastdata = temp;
			}
			break;
		}
			
		case 0x55:
		{
			if(lastdata == 0xFF)
			{
				rxUart0[countRx++] = temp;
				lastdata = temp;
			}
			else if(countRx < 0x03)
			{
				countRx = 0;
			}
			else
			{
				rxUart0[countRx++] = temp;
				countRx = 0;
				flag_RxFinish = TRUE;
			}
			break;
		}
			
		default:
		{
			rxUart0[countRx++] = temp;
			lastdata = temp;
		}
		break;
	}
}

/**********************************************************************************************
* 功能说明：串口1中断接收函数
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
ISR(USART1_RX_vect)
{ 
    volatile static uint8_t temp, countRx, NinthData;
		
	NinthData = (UCSR1B & (1<<RXB81));    //提取第9位数据，即RXB8的值
	temp = UDR1;    //若是先接收缓存数据，会改变UCSR1A和UCSR1B的值，因此在接收之前先保存好寄存器的值 
		
	if((NinthData != 0) && (temp == 0x1B))    //数据是地址且为0x1B, 开始响应
	{
		UCSR1A&=~(1<<MPCM1);    //清零MPCM开始接收数据
		countRx = 0;
		return;
	}
		
	if(NinthData == 0)    //如果是数据
	{
		rxUart1[countRx++] = temp;  
	}
		
	if(countRx == 17)    //一个完整的数据帧是17个字节
	{
		UCSR1A |= (1<<MPCM1);    //读取最后一个数据，置位MPCM等待下次被寻址
		flag_NewData = TRUE;
	}
}  

/**********************************************************************************************
* 功能说明：定时器3溢出中断函数
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
ISR(TIMER3_OVF_vect)
{
	TCNT3H=0xFE;
    TCNT3L=0xA6;
	    
	if(flag_LedON == BEGIN) tLedCount++;    //LED开始定时熄灭
	    
	if(clearFlag_YGD == BEGIN) tCountYGD++;
	    
	if(clearFlag_Forward_DD == BEGIN) tCountFD++;
	    
	if(clearFlag_Rear_DD == BEGIN) tCountRD++;
}

/**********************************************************************************************
* 函 数 名 ：DealRxData
* 功能说明：处理从串口0接收的数据(射频模块传上来的数据)
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
void Deal_RLM_Data(void)
{
	if(DecodeProtocol(rxUart0, EpcData))    //对RLM的数据进行校验，只有通过校验的数据才能进行下一步的处理
    {
		RuleCheck(EpcData);
	}
	else
	{
		return;    //不做任何处理
	}
}

/***************************************************************************************************************
* 函 数 名 ：DecodeProtocol
* 功能说明：对机感发送过来的数据进行校验 是否是完整的数据帧以及数据帧中是否包含Epc数据
* 输入参数：p_rxUart0 --> rxUart0   p_EpcData --> EpcData
* 返 回 值 ：1 or 0
***************************************************************************************************************/
uint8_t DecodeProtocol(const uint8_t* p_rxUart0, uint8_t* p_EpcData)
{
    uint8_t i;
	const uint8_t* p = p_rxUart0;
	
	if((*p++ == 0xAA) && (*(p+*p) == 0x55))    //完整的数据帧
	{
		if((p_rxUart0[5]==0xAA) && (p_rxUart0[6] == 0xBB))    //是AABB数据？
		{
			for(i= 0; i< 12; i++)
	        {	
		        p_EpcData[i]= p_rxUart0[i+6];    //将串口缓存区的有效数据放到EpcData数组中   
	        }
			return 1;
		}
	}
	return 0;
}

/************************************************************************************************
* 函 数 名 ：RuleCheck
* 功能说明：对Epc数据进行规则检查 通过 --> 通过串口0向上发送数据  不通过 --> 忽略
* 输入参数：p_EpcData --> EpcData
* 返 回 值 ：void
************************************************************************************************/
void RuleCheck(const uint8_t* p_EpcData)
{
	if(p_EpcData[6] == 0x01)    //如果是预告点
	{
		clearFlag_YGD = BEGIN;    //开启预告点计时清0
		tCountYGD = 0;     
		/*检查预告点，正向预告点 or 反向预告点*/             
		check_preportdata(EpcData, storeRedirDD, storeFdirYGD, storeFdirYGD2);
	}
		 
	if(p_EpcData[6] == 0x02)    //如果是地感
	{
		/*检查地感，正向 or 反向  上报 or 忽略*/
		if(check_portdata(EpcData, storeFdirYGD, storeFdirYGD2, storeRedirDD, storeLastportdata)) 
		{
			FrameProcess(FrameData, EpcData, 0x11, sizeof(EpcData));       
			MPCM_USART1_TransmitFrame(FrameData, 0xC1);    //发送给WirelessCom_1 --> 通信板第一个CPU
		}
	}
}

/*********************************************************************************************************************************************************
* 函 数 名 ：check_preportdata
* 功能说明：预告点的检查规则 与反向定点中的数据进行比较
                    匹配 --> 忽略  不匹配 --> 存入正向预告点
* 输入参数：p_EpcData --> EpcData  p_storeRedirDD --> storeRedirDD  p_storeFdirYGD --> storeFdirYGD  p_storeFdirYGD2 --> storeFdirYGD2
* 返 回 值 ：void
*********************************************************************************************************************************************************/
void check_preportdata(uint8_t* p_EpcData, uint8_t* p_storeRedirDD, uint8_t* p_storeFdirYGD, uint8_t* p_storeFdirYGD2)
{
	uint8_t i;  
	uint8_t epc_Code = p_EpcData[10];
	uint8_t epc_Distance = p_EpcData[8];
	/*判断是否是正向预告点，如果是正向的预告点，与反向定点信号机编码不同或距离不同*/
	if( (p_storeRedirDD[10] != epc_Code) || (p_storeRedirDD[8] != epc_Distance) )	  
	{   		                                                                
		/*是正向预告点，判断是否是重复的点，若重复，忽略，不重复存入storeFdirYGD*/	
		if( ( (p_storeFdirYGD[10] != epc_Code) || (p_storeFdirYGD[8] != epc_Distance) ) && ( (p_storeFdirYGD2[10] != epc_Code) || (p_storeFdirYGD2[8] != epc_Distance) ) )
		{
			for(i= 0; i<12; i++)
		    {
				p_storeFdirYGD2[i] = p_storeFdirYGD[i]; 
			    p_storeFdirYGD[i] = p_EpcData[i];
			}
		}
		else
		{
			p_EpcData[10] = 0;    //过反向信号灯后，车转向，第一个信号灯会因为新旧数据的判断而被忽略不报
			p_EpcData[8] = 0;		
		}
	}
	/*是反向的预告点*/
	else                            
	{
		p_EpcData[10] = 0;	   //过反向信号灯后，车转向，第一个信号灯会因为新旧数据的判断而被忽略不报   
		p_EpcData[8] = 0;	
	}	
}

/*********************************************************************************************************************************************************
* 函 数 名 ：check_portdata
* 功能说明：地感的检查规则 与正向预告点中的数据进行比较，
                    匹配且不重复 --> 规则通过 
					不匹配 --> 存入反向定点
* 输入参数：p_EpcData --> EpcData  p_storeFdirYGD --> storeFdirYGD  p_storeFdirYGD2 --> storeFdirYGD2  
                    p_storeRedirDD --> storeRedirDD   p_storeLastportdata --> storeLastportdata
* 返 回 值 ：通过-->1 or 不通过-->0
*********************************************************************************************************************************************************/
uint8_t check_portdata(uint8_t* p_EpcData, uint8_t* p_storeFdirYGD, uint8_t* p_storeFdirYGD2, uint8_t* p_storeRedirDD, uint8_t* p_storeLastportdata)
{
	uint8_t temp = 0; 
	uint8_t i;
	uint8_t epc_Code = p_EpcData[10];
	uint8_t epc_Distance = p_EpcData[8]; 
	uint8_t epc_Led = p_EpcData[9]; 
	
	/*判断是否是正向定点*/
	if( (p_storeFdirYGD[10] == epc_Code) && (p_storeFdirYGD[8] == epc_Distance) )
	{ 
    /*判断是否是相同数据，如果不是，存入storeLastportdata*/
		if( (p_storeLastportdata[10] != epc_Code) || (p_storeLastportdata[8] != epc_Distance) || (p_storeLastportdata[9] != epc_Led) || (p_storeLastportdata[10] != epc_Code))    //同一个卡号只发送一次  //灯码相同是否重复收码
        {
	        for(i= 0; i< 12; i++)
		    {
		        p_storeLastportdata[i] = p_EpcData[i];
		    }
		    temp = 1;	 //上报定点
			clearFlag_Forward_DD = BEGIN;   	 //开启计时，3MIN清除上次储存定点
			tCountFD = 0;
			//clearYGDsave = BEGIN;	 //开启计时，1s清除上次储存预告点和反向码
			//clearflag1 = 0; 	
		}
		/*是相同数据，忽略*/
		else    
		{
		}		  
    //CheckFeed();         
	}
		
	/*判断是否是正向定点，与YGD2匹配*/
	else if( (p_storeFdirYGD2[10] == epc_Code) && (p_storeFdirYGD2[8] == epc_Distance))
	{
	    if( (p_storeLastportdata[10] !=epc_Code ) || (p_storeLastportdata[8] !=epc_Distance) || (p_storeLastportdata[9] !=epc_Led) || (p_storeLastportdata[10] != epc_Code))	    //同一个卡号只发送一次
		{
	        for(i= 0; i< 12; i++)
		    {
		        p_storeLastportdata[i]= p_EpcData[i];
		    }
		    temp= 1;	
			clearFlag_Forward_DD = BEGIN;   	 //开启计时，3MIN清除上次储存定点
			tCountFD = 0;
		}
		else                                                   //与上次上报卡号相同，不在上报//加7S清数据
		{
		}		
	//CheckFeed();      
	}
	/*判断为反向定点*/
	else
	{	
		/*判断是否重复，若重复，存入反向定点*/
		if( (p_storeRedirDD[10] != epc_Code) || (p_storeRedirDD[8] != epc_Distance) )	 //同一个点作为反向，只存储1次
		{
			for( i= 0; i< 12; i++)
			{
				p_storeRedirDD[i] = p_EpcData[i];
    		}		
		}
		clearFlag_Rear_DD = BEGIN;
		tCountRD = 0;                         
	}
	
	return temp;	
} 

/************************************************************************************************
* 函 数 名 ：Time_up_Clear
* 功能说明：定时时间到，按规则清除存储的Epc数据
* 输入参数：void
* 返 回 值 ：void
************************************************************************************************/
void Time_up_Clear(void)
{
	/*收到预告点后开始定时清存储的点*/
	if(tCountYGD >= CLEAR_GAP0)    //计时1min后清0
	{
		clearFlag_YGD = END;
		tCountYGD = 0;
		
		memset(storeFdirYGD, 0, sizeof(storeFdirYGD));
		memset(storeFdirYGD2, 0, sizeof(storeFdirYGD2));
		memset(storeLastportdata, 0, sizeof(storeLastportdata));
	}
		
	/*收到正向码后开始定时清存储的点*/
	if(tCountFD >= CLEAR_GAP1)    //计时1s后清0
	{
		clearFlag_Rear_DD = END; 
		tCountRD = 0;
		clearFlag_YGD = END; 
		tCountYGD = 0;
			
		memset(storeRedirDD, 0, sizeof(storeRedirDD));
		memset(storeFdirYGD, 0, sizeof(storeFdirYGD));
		memset(storeFdirYGD2, 0, sizeof(storeFdirYGD2));
		if(tCountFD >= CLEAR_GAP2)    //计时3min后清0
		{
			clearFlag_Forward_DD = END;
			tCountFD = 0;
				
			memset(storeLastportdata, 0, sizeof(storeLastportdata));
		}
	} 
		
	/*收到反向码后开始定时清存储的点*/
	if(tCountRD >= CLEAR_GAP3)    //计时7s后清0
	{
		memset(storeRedirDD, 0, sizeof(storeRedirDD));
		if(tCountRD >= CLEAR_GAP4)    //计时1min后清0
		{
			clearFlag_Rear_DD = END;
			tCountRD = 0;
				
			memset(storeFdirYGD, 0, sizeof(storeFdirYGD));
			memset(storeFdirYGD2, 0, sizeof(storeFdirYGD2));
			memset(storeLastportdata, 0, sizeof(storeLastportdata));
		}
	}
}

/**********************************************************************************************
* 函 数 名  ：FrameProcess
* 功能说明：将源数组按照规则组成一帧数据
* 输入参数：dstArray -- 目的数组  srcArry -- 源数组  type -- 帧类型  len -- 源数组的长度
* 返 回 值 ：void
**********************************************************************************************/
void FrameProcess(uint8_t* dstArray, const uint8_t* srcArray, uint8_t type, uint8_t len)
{
	uint8_t* p_dstArray = dstArray;
	uint16_t checkValue = 0;
		
	p_dstArray[0] = 0xAA;    //SOF 
	p_dstArray[1] = type;
	p_dstArray[2] = len;
	p_dstArray += 3;
	
	while(len--)   
	{
		*p_dstArray++ = *srcArray++;
	}
		
	checkValue = Check_CRC16(dstArray, (3+dstArray[2]));
	*p_dstArray++ = (uint8_t)(checkValue>>8);    //CRC
	*p_dstArray = (uint8_t)(checkValue);    //CRC
}

/**********************************************************************************************
* 函 数 名  ：Time3_Init
* 功能说明：定时器3初始化 TCNTn = 65536 - (FOSC/Prescaler)*Timing(s)
* 输入参数：void
* 返 回 值 ：void
**********************************************************************************************/
void Timer3_Init(void)
{
	ETIFR=0;    //定时器3的中断标志寄存器清零
	
	/*65536-(11059200/64)*0.002=65190=FEA6*/
	TCNT3H = 0xFE;
	TCNT3L = 0xA6;
	
	TCCR3B|=(1<<CS31)|(1<<CS30);   //64分频
	ETIMSK=(1<<TOIE3);    //定时器3溢出中断使能
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
	PORTA=0x08;    //开始亮绿灯
	DDRD &=~((1<<0)|(1<<1)); PORTD |= (1<<0)|(1<<1);
	USART0_Init(115200,0);
	USART1_Init(115200,1);
	Timer3_Init();
	//SEI();    //该语句在studio6.2中不被识别
	sei();
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
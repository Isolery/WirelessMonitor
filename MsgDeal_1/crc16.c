/*
****************************************************************************************************************
*
*	模块名称 : crc16
*	文件名称 : crc16.c
*	说       明 : CRC-16/MODBUS校验方式，采用了查表的方式，牺牲空间换取时间
                    http://www.metools.info/    这个网站可以在线校验

* 当前版本：1.0
* 作       者：刘雪文
* 完成日期：2020年5月20日
*	
* 修改内容：
                    
****************************************************************************************************************
*/
#include <avr/io.h>
#include "crc16.h"
#include "predefine.h"

/**********************************************************************************************
* 函 数 名  ：Check_CRC16
* 功能说明：校验函数 
* 输入参数：pucFrame --> 需要校验的数据首地址    len --> 校验长度
* 返 回 值 ：uint16_t --> 返回两个字节的校验码
**********************************************************************************************/
uint16_t Check_CRC16(uint8_t* pucFrame, uint8_t len)
{
    uint8_t CRCH = 0xFF;
		uint8_t CRCL = 0xFF;

    uint16_t index;

    while(len--)
    {
        index = CRCL ^ *(pucFrame++);
        CRCL = (uint8_t)(CRCH ^ CRC_16H[index]);
        CRCH = CRC_16L[index];
    }

    return (uint16_t)(CRCH<< 8 | CRCL);
}
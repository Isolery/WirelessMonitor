/*
*********************************************************************************************************
*
*	模块名称 : uart(For ATmega1280)
*	文件名称 : uart.h
*	说       明 : 头文件

* 当前版本：1.0
* 作       者：刘雪文
* 完成日期：2020年5月20日
*	
*********************************************************************************************************
*/
#ifndef  __USART_H_
#define  __USART_H_
#include "predefine.h"

extern void USART0_Init(uint32_t baudRate, uint8_t MPCMn);
extern void USART0_TransmitByte(uint8_t data);
extern void USART0_TransmitString(const char* cString);
extern void USART0_TransmitArray(const uint8_t* Array, uint8_t len);
extern void USART0_TransmitFrame(const uint8_t* Frame);
extern void MPCM_USART0_TransmitByte(uint8_t data, uint8_t addr);
extern void MPCM_USART0_TransmitString(const char* cString, uint8_t addr);
extern void MPCM_USART0_TransmitArray(const uint8_t* Array, uint8_t len, uint8_t addr);
extern void MPCM_USART0_TransmitFrame(const uint8_t* Frame, uint8_t addr);

extern void USART1_Init(uint32_t baudRate, uint8_t MPCMn);
extern void USART1_TransmitByte(uint8_t data);
extern void USART1_TransmitString(const char* cString);
extern void USART1_TransmitArray(const uint8_t* Array, uint8_t len);
extern void USART1_TransmitFrame(const uint8_t* Frame);
extern void MPCM_USART1_TransmitByte(uint8_t data, uint8_t addr);
extern void MPCM_USART1_TransmitString(const char* cString, uint8_t addr);
extern void MPCM_USART1_TransmitArray(const uint8_t* Array, uint8_t len, uint8_t addr);
extern void MPCM_USART1_TransmitFrame(const uint8_t* Frame, uint8_t addr);
extern void MPCM_USART1_TransmitMonitorFrame(const uint8_t* Frame);

extern void USART2_Init(uint32_t baudRate);
extern void USART2_TransmitByte(uint8_t data);
extern void USART2_TransmitString(const char* cString);
extern void USART2_TransmitArray(const uint8_t* Array, uint8_t len);

extern void USART3_Init(uint32_t baudRate);
extern void USART3_TransmitByte(uint8_t data);
extern void USART3_TransmitString(const char* cString);
extern void USART3_TransmitArray(const uint8_t* Array, uint8_t len);

#endif
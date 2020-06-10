#ifndef  __USART_H_
#define  __USART_H_
#include "predefine.h"

extern void USART0_Init(uint32_t baudRate, uint8_t MPCMn);
extern void USART0_TransmitByte(uint8_t data);
extern void USART0_TransmitString(const uint8_t* cString);
extern void USART0_TransmitArray(const uint8_t* Array, uint8_t len);
extern void USART0_TransmitFrame(const uint8_t* Frame);
extern void MPCM_USART0_TransmitByte(uint8_t data, uint8_t addr);
extern void MPCM_USART0_TransmitString(const uint8_t* cString, uint8_t addr);
extern void MPCM_USART0_TransmitArray(const uint8_t* Array, uint8_t len, uint8_t addr);
extern void MPCM_USART0_TransmitFrame(const uint8_t* Frame, uint8_t addr);

extern void USART1_Init(uint32_t baudRate, uint8_t MPCMn);
extern void USART1_TransmitByte(uint8_t data);
extern void USART1_TransmitString(const uint8_t* cString);
extern void USART1_TransmitArray(const uint8_t* Array, uint8_t len);
extern void USART1_TransmitFrame(const uint8_t* Frame);
extern void MPCM_USART1_TransmitByte(uint8_t data, uint8_t addr);
extern void MPCM_USART1_TransmitString(const uint8_t* cString, uint8_t addr);
extern void MPCM_USART1_TransmitArray(const uint8_t* Array, uint8_t len, uint8_t addr);
extern void MPCM_USART1_TransmitFrame(const uint8_t* Frame, uint8_t addr);

void MPCM_USART1_TransmitMonitorFrame(const uint8_t* Frame);

#endif
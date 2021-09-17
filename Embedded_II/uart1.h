/*
 * uart1.h
 *
 *  Created on: Apr 5, 2020
 *      Author: Furediiii
 */

#ifndef UART1_H_
#define UART1_H_
//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart1();
void enableEmptyTxIntUart1();
void enableRxIntUart1();
bool isParityError();
void clearParityError();
void enableParityBitUart1();
void disableParityBitUart1();
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart1(char c);
char getcUart1();
bool isUart1Empty();
bool isUart1Busy();
//bool isUart1Clear();
bool isTxIntUart1();
bool isRxIntUart1();

#endif /* Uart1_H_ */

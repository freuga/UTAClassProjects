/*
 * buffer.c
 *
 *  Created on: May 6, 2020
 *      Author: aguin
 */
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "uart0.h"
#include "convert.h"
#include "tm4c123gh6pm.h"


#define BUFFER_SIZE 1000

char buffer[BUFFER_SIZE];
uint32_t writeIndex = 0;
uint32_t readIndex = 0;

void initBuffer()
{
    memset(buffer, 0, BUFFER_SIZE);
    UART0_IM_R |= UART_IM_TXIM;
    UART0_ICR_R |= UART_ICR_TXIC;
    NVIC_EN0_R |= (1<<5);
}
void writeBuffer(char *str)
{
    uint32_t length;
    uint32_t i = 0;

    length = c_strlen(str);
    while(i < length)
    {
        buffer[writeIndex] = str[i];
        writeIndex += 1;
        writeIndex = writeIndex % BUFFER_SIZE;
        i++;
    }
}
void bufferIsr()
{
    if(readIndex != writeIndex)
    {
        putcUart0(buffer[readIndex]);
        while(isUart0Busy());
        readIndex += 1;
        readIndex = readIndex % BUFFER_SIZE;
    }
    UART0_ICR_R |= UART_ICR_TXIC;
}

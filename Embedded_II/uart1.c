/*
 * uart1.c
 *
 *  Created on: Apr 5, 2020
 *      Author: aguin
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart1.h"

// PortC masks
#define UART_TX_MASK 32 //PC5
#define UART_RX_MASK 16 //PC4

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
// Set baud rate as function of instruction cycle frequency
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    UART1_IBRD_R = divisorTimes128 >> 7;                 // set integer value to floor(r)
    UART1_FBRD_R = ((divisorTimes128 + 1)) >> 1 & 63;    // set fractional value to round(fract(r)*64)
}
// Initialize UART1
void initUart1()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;    // UART1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;    // port C
    _delay_cycles(3);

    // Configure UART1 pins
    GPIO_PORTC_DIR_R |= UART_TX_MASK;                   // enable output on UART1 TX pin
    GPIO_PORTC_DIR_R &= ~UART_RX_MASK;                   // enable input on UART1 RX pin
    GPIO_PORTC_DR2R_R |= UART_TX_MASK;                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R |= UART_TX_MASK | UART_RX_MASK;    // enable digital on UART1 pins
    GPIO_PORTC_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;  // use peripheral to drive PC4, PC5
    GPIO_PORTC_PCTL_R &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M); // clear bits 0-7
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_U1RX | GPIO_PCTL_PC5_U1TX;
                                                        // select UART1 to drive pins PC4 and PC5: default, added for clarity

    // Configure UART1 to 115200 baud, 8N1 format
    UART1_CTL_R = 0;                                    // turn-off UART1 to allow safe programming
    UART1_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (40 MHz)
//    UART1_IBRD_R = 21;                                  // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
//    UART1_FBRD_R = 45;                                  // round(fract(r)*64)=45
    setUart1BaudRate(38400, 40e6);
    UART1_LCRH_R = UART_LCRH_WLEN_8 |UART_LCRH_SPS | UART_LCRH_PEN;    // configure for 8N1 w/ 16-level FIFO
    UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // enable TX, RX, and module
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart1(char c)
{
    while (UART1_FR_R & UART_FR_TXFF);               // wait if uart1 tx fifo full
    UART1_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart1(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart1(str[i++]);
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart1()
{
//    while (UART1_FR_R & UART_FR_RXFE);               // wait if uart1 rx fifo empty
    return UART1_DR_R & 0xFF;                        // get character from fifo
}
bool isParityError()
{
    return ((UART1_RSR_R & UART_RSR_PE));
}
void clearParityError()
{
    UART1_ICR_R |= UART_ICR_PEIC;
    UART1_ECR_R |= UART_ECR_DATA_M;
}
void enableParityBitUart1()
{
    UART1_LCRH_R |= UART_LCRH_EPS;
}
void disableParityBitUart1()
{
    UART1_LCRH_R &= ~(UART_LCRH_EPS);
}
//enables transition from not empty -> empty interrupt
void enableEmptyTxIntUart1()
{
    UART1_ICR_R = 0xFFFFFFFF;
    UART1_CTL_R |= UART_CTL_EOT;
    UART1_IM_R |= UART_IM_TXIM;
    NVIC_EN0_R |= (1 << 6);
}
void enableRxIntUart1()
{
    UART1_ICR_R = 0xFFFFFFFF;
    UART1_IM_R |= UART_IM_RXIM;
    NVIC_EN0_R |= (1 << 6);
}
// check if uart fifo is empty
bool isUart1Empty()
{
    if(UART1_FR_R & UART_FR_TXFE)
        return true;
    else
        return false;
}
// Checks if shift register is empty
bool isUart1Busy()
{
    if(UART1_FR_R & UART_FR_BUSY)
        return true;
    else
        return false;
}
bool isTxIntUart1()
{
    if(UART1_MIS_R & UART_MIS_TXMIS)
        return true;
    else
        return false;
}
bool isRxIntUart1()
{
    if(UART1_MIS_R & UART_MIS_RXMIS)
        return true;
    else
        return false;
}


/**
 * Freddy Aguinaga
 * Embedded II - RS 485 communication
 * main.c
 *
 * Channels:
 * 0: [I] Push Button
 * 1: [O] LED 1
 * 2: [O] LED 2
 * 3: [O] LED 3
 * 4: [O] LED 4
 * 5: [O] Buzzer
 *
 *
 */

//Libraries
#include <stdbool.h>
#include <stdio.h>
#include "gpio.h"
#include "uart0.h"
#include "uart1.h"
#include "convert.h"
#include "timer.h"
#include "wait.h"
#include "eeprom.h"
#include "buffer.h"
#include "tm4c123gh6pm.h"
#include "rs485Table.h"

//Define board LEDs and push button
#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3
#define PUSH_BUTTON PORTF,4

//Define Output Channels
#define BUZZER  PORTB,3
#define LED_1   PORTB,4
#define LED_2   PORTB,5
#define LED_3   PORTB,6
#define LED_4   PORTB,7

//Data Enable for Transmissions
#define DATAENABLE PORTA,2

// Main constants
#define MAX_ARGS 20
#define MAX_CHARS 255
#define MAX_ACTIVITIES 5

//keeps track of commands that rely on setting time(ms) ON/OFF
typedef struct _activity
{
    bool active;
    bool select;                // chooses between val 1(0) or val 2(1)
    uint8_t cmd;                // cmd determines what variables are used
    uint8_t chan;               // channel used
    uint8_t val1;               // value for first half of control signal
//    uint8_t val2;               // value for second half of control signal (if used)
//    uint16_t Time1;             // Val1  period
//    uint16_t Time2;             // Val2  Period
//    uint16_t currentTime;       // Current ON/OFF time incremented
    uint32_t remainingTime;     // Will decrement to show the remaining time for this activity
}activity;
enum commands
{
    PULSE           = 0x02,
    SET             = 0x00,
    DATA_REQ        = 0x20,
    DATA_REP        = 0x21,
    ACKNOWLEDGE     = 0x70,
    POLL_REQ        = 0x78,
    POLL_RESP       = 0x79,
    SET_ADD         = 0x7A,
    RESET           = 0x7F,
    ERROR_CH        = 0x11,
    ERROR_CMD       = 0x10
};

bool newLine = true;
uint8_t activities = 0;
activity actTable[MAX_ACTIVITIES];
void doActivity(uint8_t actIndex)
{
    switch(actTable[actIndex].cmd)
    {
    case PULSE:
        switch(actTable[actIndex].chan)
        {
        case 1: setPinValue(LED_1,0);   actTable[actIndex].active = false; break;
        case 2: setPinValue(LED_2,0);   actTable[actIndex].active = false; break;
        case 3: setPinValue(LED_3,0);   actTable[actIndex].active = false; break;
        case 4: setPinValue(LED_4,0);   actTable[actIndex].active = false; break;
        case 5: setPinValue(BUZZER,0);  actTable[actIndex].active = false; break;
        }
    break;
    }
}
// Find an empty spot and set the new
bool setActivity(uint8_t cmd, uint8_t chan, uint8_t* data, uint8_t size)
{
    int i;
    switch(cmd)
    {
    case PULSE:
        for(i=0; i<MAX_ACTIVITIES; i++)
        {
            if(actTable[i].active == false)
            {
                actTable[i].active = true;
                actTable[i].chan = chan;
                actTable[i].cmd = cmd;
//                actTable[i].val1 = data[0];
                actTable[i].val1 = 1;
                actTable[i].remainingTime = arrayTo32(&data[1], 2);
                return true;
            }
        }
//    case SQUARE:
    }
    return false;
}
// turn on/off timer for activity monitoring
void setActivityTimer(bool on)
{
    if(on)
    {
        TIMER0_IMR_R    |= TIMER_IMR_TATOIM;            // turn on timeout interrupts for timer1
        TIMER0_CTL_R    |= TIMER_CTL_TAEN;              // turn on timer1 for activities
    }
    else
    {
        TIMER0_CTL_R    &= ~(TIMER_CTL_TAEN);           // turn off timeout interrupts
        TIMER0_IMR_R    &= ~(TIMER_IMR_TATOIM);         // disable interrupts
        TIMER0_TAILR_R   = 400000;                      // reset timer
    }
}
//counts down all the backoffs
void countDown()
{
    //if all valid commands in Tx Buffer have backoffs=0, turn off timer
    if(!decrementBackoffs())
        stopTimer(countDown);
}
void turnOffLeds()
{
    setPinValue(RED_LED, 0);
    setPinValue(BLUE_LED,0);
    setPinValue(GREEN_LED,0);
}
//blinks Led with an on-time of 100ms
void flash(PORT port, uint8_t pin)
{
    turnOffLeds();
    setPinValue(port, pin, 1);
    restartTimer(turnOffLeds);
}
void softwareReset()
{
    NVIC_APINT_R = NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
}
void printHelp()
{
    writeBuffer("#########################  HELP  ##########################\n\r");
    writeBuffer("PULSE:     Address : Channel : Value : Duration[10ms]\n\r");
    writeBuffer("SET:       Address : Channel : Value\n\r");
    writeBuffer("DATAREQ:   Address : Channel\n\r");
    writeBuffer("SA:        Address : New Address\n\r");
    writeBuffer("Poll:\n\r");
    writeBuffer("Reset:\n\r");
    writeBuffer("############################################################\n\r");
}
void printDataReport(uint8_t channel, uint8_t* data)
{
    char channelStr[2];
    char dataStr[2];

    intToStr(channelStr,channel);
    intToStr(dataStr, data[0]);
    writeBuffer("Channel ");
    switch(channel)
    {
    case 0: writeBuffer("BUTTON 1:"); break;
    case 1: writeBuffer("LED 1: ");   break;
    case 2: writeBuffer("LED 2: ");   break;
    case 3: writeBuffer("LED 3: ");   break;
    case 4: writeBuffer("LED 4: ");   break;
    case 5: writeBuffer("Buzzer: ");  break;
    }
    if(channel == 0)
    {
        if(data[0] > 0)
            writeBuffer("Released");
        else
            writeBuffer("Pressed");
    }
    else if(channel < 6)
    {
        if(data[0] > 0)
            writeBuffer("ON");
        else
            writeBuffer("OFF");
    }
    else
        writeBuffer(dataStr);
    writeBuffer("\n\r");
    newLine = true;
}
printQueueMessage(bool successful)
{
    char id[3];
    intToStr(id, getId());
    writeBuffer("Queuing msg ");
    writeBuffer(id);
    writeBuffer(" ");
    if(successful)
        writeBuffer("successful.\n\r");
    else
        writeBuffer("failed.\n\r");
}
void printError(uint8_t node, bool isCmd, uint8_t type, uint8_t channel)
{
    char nodeStr[5];
    char typeStr[5];
    char chanStr[5];
    intToStr(nodeStr, node);
    writeBuffer("Node ");
    writeBuffer(nodeStr);
    intToStrHex(typeStr, type);
    writeBuffer(": command ");
    switch(type)
    {
    case PULSE:     writeBuffer("PULSE"); break;
    case SET:       writeBuffer("SET"); break;
    case DATA_REQ:  writeBuffer("DATA_REQ"); break;
    }
//    writeBuffer(typeStr);
    if(!(isCmd))
    {
        intToStr(chanStr, channel);
        writeBuffer(": channel ");
        writeBuffer(chanStr);
    }
    writeBuffer(" is not supported\n\r");

}
void runRxCommand(uint8_t index)
{
    char id[2];
    int cmdIndex = index;
    if(cmdIndex == -1)
        return;
    uint8_t cmdSrc      = getRxCommandSrc(cmdIndex);
    uint8_t cmdId       = getRxCommandId(cmdIndex);
    uint8_t cmdType     = getRxCommandType(cmdIndex);
    uint8_t cmdChannel  = getRxCommandChannel(cmdIndex);
    uint8_t cmdDataSize = getRxCommandDataSize(cmdIndex);
    uint8_t* cmdData    = getRxCommandData(cmdIndex);
    uint8_t value;
    uint8_t temp;

    //if ack was requested, send ack to sender
    if(cmdType & 0x80)
    {
        queueMessage(cmdSrc, ACKNOWLEDGE, cmdChannel, &cmdId, 1);
        cmdType = (cmdType & 0x7F);
    }
    //Table of commands;
    switch(cmdType)
    {
    case PULSE:
        switch(cmdChannel)
        {
        case 1:
            setPinValue(LED_1,1);
            setActivity(cmdType, cmdChannel, cmdData, cmdDataSize);
            setActivityTimer(true);
            break;
        case 2:
            setPinValue(LED_2,1);
            setActivity(cmdType, cmdChannel, cmdData, cmdDataSize);
            setActivityTimer(true);
            break;
        case 3:
            setPinValue(LED_3,1);
            setActivity(cmdType, cmdChannel, cmdData, cmdDataSize);
            setActivityTimer(true);
            break;
        case 4:
            setPinValue(LED_4,1);
            setActivity(cmdType, cmdChannel, cmdData, cmdDataSize);
            setActivityTimer(true);
            break;
        case 5:
            setPinValue(BUZZER,1);
            setActivity(cmdType, cmdChannel, cmdData, cmdDataSize);
            setActivityTimer(true);
            break;
        default:
            writeBuffer("Error: Received channel not supported.\n\r");
            temp = PULSE;
            queueMessage(cmdSrc, ERROR_CH, cmdChannel, &temp, 1);
            break;
        }
        break;
    case SET:
        switch(cmdChannel)  // check what is supported by set
        {
        case 1:
            if(cmdData[0] > 0)
                setPinValue(LED_1, 1);
            else
                setPinValue(LED_1, 0);
            break;
        case 2:
            if(cmdData[0] > 0)
                setPinValue(LED_2, 1);
            else
                setPinValue(LED_2, 0);
            break;
        case 3:
            if(cmdData[0] > 0)
                setPinValue(LED_3, 1);
            else
                setPinValue(LED_3, 0);
            break;
        case 4:
            if(cmdData[0] > 0)
                setPinValue(LED_4, 1);
            else
                setPinValue(LED_4, 0);
            break;
        case 5:
            if(cmdData[0] > 0)
                setPinValue(BUZZER, 1);
            else
                setPinValue(BUZZER, 0);
            break;
        default:
            writeBuffer("Error: Received channel not supported.\n\r");
            temp = SET;
            queueMessage(cmdSrc, ERROR_CH, cmdChannel, &temp, 1);
            break;
        }
        break;
    case DATA_REQ: // queue up a data response with data of channel requested
        switch(cmdChannel)
        {
        case 0:
            value = getPinValue(PUSH_BUTTON);
            queueMessage(cmdSrc, DATA_REP, cmdChannel, &value, 1);
            break;
        case 1:
            value = getPinValue(LED_1);
            queueMessage(cmdSrc, DATA_REP, cmdChannel, &value, 1);
            break;
        case 2:
            value = getPinValue(LED_2);
            queueMessage(cmdSrc, DATA_REP, cmdChannel, &value, 1);
            break;
        case 3:
            value = getPinValue(LED_3);
            queueMessage(cmdSrc, DATA_REP, cmdChannel, &value, 1);
            break;
        case 4:
            value = getPinValue(LED_4);
            queueMessage(cmdSrc, DATA_REP, cmdChannel, &value, 1);
            break;
        case 5:
            value = getPinValue(BUZZER);
            queueMessage(cmdSrc, DATA_REP, cmdChannel, &value, 1);
            break;
        default:
            writeBuffer("Error: Received channel not supported.\n\r");
            temp = DATA_REQ;
            queueMessage(cmdSrc, ERROR_CH, cmdChannel, &temp, 1);
            break;
        }
        break;
    case DATA_REP:  // Received data from a data req or unsolicited.
        printDataReport(cmdChannel, cmdData); // Temporarily here, because I just wanna see what comes in
        break;
    case POLL_REQ:
        value = getAddress();
        queueMessage(cmdSrc, POLL_RESP, 0, &value, 1);
        break;
    case POLL_RESP:
        intToStr(id, cmdData[0]);
        writeBuffer("Node at: ");
        writeBuffer(id);
        writeBuffer("\n\r");
        break;
    case SET_ADD:
        writeEeprom(0, cmdData[0]);
        setAddress(cmdData[0]);
        break;
    case ACKNOWLEDGE:
        ackCommand(cmdData[0]);
        break;
    case RESET:
        softwareReset();
        break;
    case ERROR_CMD:
        printError(cmdSrc, true, cmdData[0], cmdChannel);
        break;
    case ERROR_CH:
        printError(cmdSrc, false, cmdData[0], cmdChannel);
        break;
    default:
        writeBuffer("Error: Received Command not supported.\n\r");
        queueMessage(cmdSrc, ERROR_CMD, cmdChannel, cmdType, 1);
        break;
    }
    finishedRxCommand(cmdIndex);
}
//Uart1 interrupt when tx buffer is emptied or Rx buffer receives a byte
void txEmptyIsr()
{
    //somehow we still to grab more Tx bits slipping into Rx buffer
    if(getPinValue(DATAENABLE) == 1)
    {
        if(UART1_FR_R & UART_FR_RXFF)
            getcUart1();
        UART1_ICR_R |= UART_ICR_RXIC;
    }
    //if we are currently transmitting
    if(isTxIntUart1())
    {
        if(!(isUart1Busy()))
        {
            UART1_ICR_R |= UART_ICR_TXIC;
            //if we are starting a new message, print it out
            if(getPhase() == 1)
            {
                printTxMessage(getIndex());
                enableParityBitUart1();
            }
            //if not a destination byte, turn off parity
            if(getPhase() == 2)
                disableParityBitUart1();
            //send value while data enable is on
            if(getPinValue(DATAENABLE))
                sendMessage();
            //When a complete message is sent
            if(getPhase() == 0)
            {
                restartTimer(countDown);
                disableParityBitUart1();
                while(isUart1Busy());
                setPinValue(DATAENABLE, 0);

                if(getTransmissions(getIndex()) == 0 && getTxAck(getIndex()))
                    setPinValue(RED_LED, 1);
                else
                    flash(RED_LED);

                //grab last tx byte slipping into the rx buffer
                if(UART1_FR_R & UART_FR_RXFF)
                    getcUart1();
                UART1_ICR_R |= UART_ICR_RXIC;
                UART1_ICR_R |= UART_ICR_TXIC;
            }
            waitMicrosecond(200);
            //grab tx bytes slipping into the Rx buffer
            if(UART1_FR_R & UART_FR_RXFF)
                getcUart1();
            UART1_ICR_R |= UART_ICR_RXIC;
        }
    }
    // receive bytes till we get a complete packet
    if(isRxIntUart1())
    {
        if(getPinValue(DATAENABLE) == 0)
        {
            if(receiveMessage())
            {
                runRxCommand(getIndexRx());
                flash(GREEN_LED);
            }
        }
        // get rid of bytes that we are not ready to receive
        else
        {
            if(UART1_FR_R & UART_FR_RXFF)
                getcUart1();
        }
        UART1_ICR_R |= UART_ICR_RXIC;
    }
    // check on activities if any are active
    if(TIMER0_MIS_R & TIMER_MIS_TATOMIS)
    {
        int i;
        bool oneActive = false;
        for(i=0; i<MAX_ACTIVITIES; i++)
        {
            if(actTable[i].remainingTime > 0)
            {
                oneActive = true;
                actTable[i].remainingTime -= 1;
            }
            else
            {
                doActivity(i);
            }
        }
        if(!oneActive)
        {
            setActivityTimer(false);
        }
        TIMER0_ICR_R |= TIMER_ICR_TATOCINT;
    }
}
void processCommand(char**cmd, uint8_t wordCount)
{
    int idx;
    uint8_t add;
    uint8_t channel;
    uint8_t value;
    bool queuedMsg = false;

    if(cmd == 0 || wordCount == 0)
    {
        return;
    }
    if(c_strcmp(cmd[0], "pulse") == 0 && wordCount == 5)
    {
        uint16_t duration;
        uint8_t bytes[3];
        //contains add, chan, value, duration(10ms)
        if(cmd[1][0] == '0' && cmd[1][1] == 'x')    // save address appropriately if in hex or decimal
            add = strHexToUint8(&cmd[1][2]);
        else
            add = strToInt(cmd[1]);
        if(cmd[2][0] == '0' && cmd[2][1] == 'x')    // save channel appropriately if in hex or decimal
            channel = strHexToUint8(&cmd[2][2]);
        else
            channel = strToInt(cmd[2]);
        bytes[0] = strToInt(cmd[3]);
        duration = strToInt(cmd[4]);
        bytes[1] = (duration >> 8);
        bytes[2] = (duration & 0x00FF);

        idx = queueMessage(add, PULSE, channel, bytes, 3);
        if(idx == -1)
        {
            printQueueMessage(false);
            return;
        }
        queuedMsg = true;
    }
    else if(c_strcmp(cmd[0], "set") == 0 && wordCount == 4)
    {
        if(cmd[1][0] == '0' && cmd[1][1] == 'x')    // save address appropriately if in hex or decimal
            add = strHexToUint8(&cmd[1][2]);
        else
            add = (uint8_t) strToInt(cmd[1]);
        if(cmd[2][0] == '0' && cmd[2][1] == 'x')    // save channel appropriately if in hex or decimal
            channel = strHexToUint8(&cmd[2][2]);
        else
            channel = strToInt(cmd[2]);
        value = strToInt(cmd[3]);

        idx = queueMessage(add, SET, channel, &value, 1);
        if(idx == -1)
        {
            printQueueMessage(false);
            return;
        }
        queuedMsg = true;
    }
    else if(c_strcmp(cmd[0], "get") == 0 && wordCount == 3)
    {
        if(cmd[1][0] == '0' && cmd[1][1] == 'x')
            add = strHexToUint8(&cmd[1][2]);
        else
            add = strToInt(cmd[1]);
        if(cmd[2][0] == '0' && cmd[2][1] == 'x')
            channel = strHexToUint8(&cmd[2][2]);
        else
            channel = strToInt(cmd[2]);
        value = 0;
        idx = queueMessage(add, DATA_REQ, channel, &value, 1);
        if(idx == -1)
        {
            printQueueMessage(false);
            return;
        }
        queuedMsg = true;
    }
    else if(c_strcmp(cmd[0], "sa") == 0 && wordCount == 3)
    {
        uint8_t newAdd;

        if(cmd[1][0] == '0' && cmd[1][1] == 'x')
            add = strHexToUint8(&cmd[1][2]);
        else
            add = strToInt(cmd[1]);
        if(cmd[2][0] == '0' && cmd[2][1] == 'x')
            newAdd = strHexToUint8(&cmd[2][2]);
        else
            newAdd = strToInt(cmd[2]);
        if(newAdd == 0xFF)
        {
            writeBuffer("ERROR: Invalid Address");
            return;
        }
        idx = queueMessage(add, SET_ADD, 0, &newAdd, 1);
        if(idx == -1)
        {
            printQueueMessage(false);
            return;
        }
        queuedMsg = true;
    }
    else if(c_strcmp(cmd[0], "test") == 0 && wordCount < 3)
    {
        int i;
        channel = 1;
        if(wordCount == 1)
        {
            value = 1;
            writeBuffer("\n\r######## Testing send message #######\n\r");
            for(i=0; i<5; i++)
            {
                while(isUart0Busy());
                idx = queueMessage(0, SET, channel, &value, 1);
                if(idx == -1)
                {
                    printQueueMessage(false);
                    writeBuffer("\r\n");
                    break;
                }
                printQueueMessage(true);
                channel++;
            }
        }
        else if(wordCount == 2)
        {
            value = 0;
            writeBuffer("\n\r######## Testing send message #######\n\r");
            for(i=0; i<5; i++)
            {
                while(isUart0Busy());
                idx = queueMessage(0, SET, channel, &value, 1);
                if(idx == -1)
                {
                    printQueueMessage(false);
                    writeBuffer("\r\n");
                    break;
                }
                printQueueMessage(true);
                channel++;
            }
        }
    }
    else if(c_strcmp(cmd[0], "poll") == 0 && wordCount == 1)
    {
        channel = 0;
        value = 0;
        idx = queueMessage(0xFF, POLL_REQ, channel, &value, 1);
        if(idx == -1)
        {
            printQueueMessage(false);
            return;
        }
        queuedMsg = true;
    }
    else if(c_strcmp(cmd[0], "random") == 0 && wordCount == 2)
    {
        if(c_strcmp(cmd[1], "on") == 0)
        {
            writeBuffer("Random set to ON.");
            setRandom(true);
        }
        else if(c_strcmp(cmd[1], "off") == 0)
        {
            writeBuffer("Random set to OFF.");
            setRandom(false);
        }
        else
        {
            writeBuffer("Random: invalid argument");
        }
        writeBuffer("\n\r");
    }
    else if(c_strcmp(cmd[0], "ack") == 0 && wordCount == 2)
    {
        if(c_strcmp(cmd[1], "on") == 0)
        {
            writeBuffer("Ack set to ON.");
            setAck(true);
        }
        else if(c_strcmp(cmd[1], "off") == 0)
        {
            writeBuffer("Ack set to OFF.");
            setAck(false);
        }
        else
        {
            writeBuffer("ERROR Ack: invalid argument");
        }
        writeBuffer("\n\r");
    }
    else if(c_strcmp(cmd[0], "address") == 0 && wordCount == 1)
    {
        char str[10];
        intToStr(str, getAddress());
        writeBuffer("My Address: ");
        writeBuffer(str);
        writeBuffer("\n\r");
    }
    else if(c_strcmp(cmd[0] , "help") == 0 && wordCount == 1)
    {
        printHelp();
    }
    else if(c_strcmp(cmd[0], "reset") == 0 && wordCount < 3)
    {
        // Reset our board
        if(wordCount == 1)
            softwareReset();

        //Reset node with destination address
        if(cmd[1][0] == '0' && cmd[1][1] == 'x')
            add = strHexToUint8(&cmd[1][2]);
        else
            add = strToInt(cmd[1]);
        value = 0;
        idx = queueMessage(add, RESET, 0, &value, 1);
        if(idx == -1)
        {
            printQueueMessage(false);
            return;
        }
        queuedMsg = true;
    }
    else
    {
        writeBuffer("ERROR: Invalid Command");
        writeBuffer("\n\r");
    }
    //if we queued something up, check if we need to "prime the pump" and send the first byte of a message
    if(queuedMsg)
    {
        printQueueMessage(true);

        if(!(isUart1Busy()) && isUart1Empty() && getPhase() == 0)
        {
            printTxMessage(idx);
            setPinValue(DATAENABLE, 1);
            enableParityBitUart1();
            waitMicrosecond(10);
            primePump(idx);
        }
    }
}
void initChannels()
{
    enablePort(PORTB);

    selectPinPushPullOutput(LED_1);
    selectPinPushPullOutput(LED_2);
    selectPinPushPullOutput(LED_3);
    selectPinPushPullOutput(LED_4);
    selectPinPushPullOutput(BUZZER);
}
//Initialize hardware: Set led pins and clock
void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    _delay_cycles(3);

    //clocking Timer 1 peripheral
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R0;

    enablePort(PORTA);
    enablePort(PORTF);

    selectPinPushPullOutput(DATAENABLE);
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinDigitalInput(PUSH_BUTTON);
    enablePinPullup(PUSH_BUTTON);

    TIMER0_CTL_R    &= ~TIMER_CTL_TAEN;             // Turn off timer
    TIMER0_CFG_R    |= TIMER_CFG_32_BIT_TIMER;      // Timer with A to make 16 bit timer
    TIMER0_TAMR_R   |= TIMER_TAMR_TAMR_PERIOD;      // Periodic timer
    TIMER0_TAILR_R   = 400000;                      // Interrupts at 10ms rate
    TIMER0_ICR_R    |= TIMER_ICR_TATOCINT;          // clear any interrupts
    NVIC_EN0_R      |= (1 << 19);                   // Turn on interrupts for Timer1

}
int main(void)
{
    initHw();
    initChannels();
    initEeprom();
    initTimer();
    initUart0();
    initUart1();
    setUart0BaudRate(115200, 40e6);
    setUart1BaudRate(38400, 40e6);

    enableEmptyTxIntUart1();
    enableRxIntUart1();
    setPinValue(DATAENABLE, 0);
    setActivityTimer(false);

    startPeriodicTimerMs(countDown, 1);
    stopTimer(countDown);
    startOneshotTimerMs(turnOffLeds, 10);
    stopTimer(turnOffLeds);

    uint32_t eepromAdd;
    char command[MAX_CHARS];
    char* words[MAX_ARGS];
    uint8_t terminal_index = 0;

    eepromAdd = readEeprom(0);
    if(eepromAdd == 0xFFFFFFFF)
        setAddress(0);
    else
        setAddress(eepromAdd & 0xFF);

    flash(GREEN_LED);
    writeBuffer("\n\r");
    writeBuffer("CSE 43242: Embedded II - RS485 Communications\n\r");
    writeBuffer("Ready.");
    writeBuffer("\n\n\n\n\n\r");
    writeBuffer("\n\n\n\n\n\r");

    while(1)
    {
        if(newLine && isUart0Empty() && getPhase() == 0)
        {
            if(!(isUart0Busy()))
            {
//                writeBuffer("fxa4521> ");
//                newLine = false;
            }
        }
        // Put terminal processing here
        if (kbhitUart0())
        {
            //Get the next character from Uart0 and set into command.
            uint8_t status = getsUart0(command, &terminal_index, MAX_CHARS);

            //if enter button or max characters were entered, process the command string for valid commands and args
            if(status == 0)
            {
                newLine = true;
                uint8_t wordCount;
                wordCount = parseStr(command, words, terminal_index, MAX_ARGS);
////               For Debugging Purposes
//                uint8_t w_idx = 0;
//                char index_num[MAX_ARGS];
//                for(w_idx; w_idx < wordCount; w_idx++)
//                {
//                    intToStr(index_num, w_idx);
//                    writeBuffer(index_num);
//                    putcUart0(' ');
//                    writeBuffer(words[w_idx]);
//                    writeBuffer("\r\n");
//                }
                processCommand(words, wordCount);
                terminal_index = 0;
                words[0] = 0;
            }
        }
        if(!(isUart1Busy()) && getPhase() == 0)
        {
            sendMessage();
            if(getPhase() == 1)
            {
                printTxMessage(getIndex());
                newLine = false;
                enableParityBitUart1();
                setPinValue(DATAENABLE, 1);
                sendMessage();
            }
        }
        if(isUart0Empty())
        {
            bufferIsr();
        }
    }
	return 0;
}

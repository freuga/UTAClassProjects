/*
 * devices.c
 *
 *  Created on: May 2, 2020
 *      Author: aguin
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "tm4c123gh6pm.h"
#include "adc0.h"
#include "convert.h"
#include "uart0.h"
#include "eth0.h"
#include "gpio.h"

#define MAX_TOPIC_LENGTH 50
#define MAX_TOPICS 20
#define MAX_MSG_LENGTH 100

#define NUM_INPUT_DEVICES 2
#define NUM_OUTPUT_DEVICES 2
#define MAX_PACKET_SIZE 1522

#define BLUELED PORTF,2
#define PUSHBUTTON PORTF,4

enum Devices
{
    //inputs
    TEMP_SENSOR,
    PUSH_BUTTON,
//    RTC_SET,


    //outputs
//    RTC_ALARM,
    BLUE_LED,
    CMD
};
typedef struct _topicEntry
{
    bool valid;
    char name[MAX_TOPIC_LENGTH];
    char value[MAX_MSG_LENGTH];
}topicEntry;

uint32_t deviceData[NUM_INPUT_DEVICES];
//char* outputData[NUM_OUTPUT_DEVICES];
topicEntry topicTable[MAX_TOPICS];
time_t utcTime;
struct tm *myTime;

void clockIsr()
{
    if(HIB_RIS_R & HIB_RIS_RTCALT0)
    {
        putsUart0("Squaaa\n\r");
    }
    HIB_IC_R = HIB_IC_RTCALT0;
}
//initialize all the devices we are gonna support here
void initDevices()
{
    enablePort(PORTF);
    _delay_cycles(4);
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;
    _delay_cycles(16);
    SYSCTL_RCGCHIB_R |= SYSCTL_RCGCHIB_R0;
    _delay_cycles(16);

    //configure the wide timer
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    WTIMER0_CFG_R = TIMER_CFG_16_BIT;                 // configure as 32-bit timer A
    WTIMER0_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    WTIMER0_TAILR_R = 10000000;                       // set load value (4 Hz rate)
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    WTIMER0_IMR_R |= TIMER_IMR_TATOIM;                // turn-on interrupt
    NVIC_EN2_R |= 1 << (INT_WTIMER0A-80);             // turn-on interrupt 86 (WTIMER0A)

    //INPUTS
    //configure the ADC to read from temp sensor
    initAdc0Ss3();
    setAdc0Ss3Log2AverageCount(1);
    ADC0_ACTSS_R &= ~(ADC_ACTSS_ASEN3);
    ADC0_SSCTL3_R |= ADC_SSCTL3_TS0;
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;

    //configure Push button
    selectPinDigitalInput(PUSHBUTTON);
    enablePinPullup(PUSHBUTTON);


    // OUTPUTS
    // Set BLUE LED as output
    selectPinPushPullOutput(BLUELED);

//    // get local time
//    time(&utcTime);
//    myTime = localtime(&utcTime);
//    HIB_CTL_R |= HIB_CTL_RTCEN | HIB_CTL_CLK32EN;
//    while(!(HIB_CTL_R & HIB_CTL_WRC));
//    HIB_RTCLD_R = mktime(myTime);
//    while(!(HIB_CTL_R & HIB_CTL_WRC));
//    HIB_RTCM0_R = 5;
//    while(!(HIB_CTL_R & HIB_CTL_WRC));
//    HIB_IM_R = HIB_IM_RTCALT0;
//    NVIC_EN1_R |= 1 << (INT_HIBERNATE - 48);


}
bool isTopicEntry(char* topic1, char* topic2)
{
    int i;
    int maxLength;
    int temp, temp2;
    int difference;

    temp = strlen(topic1);
    temp2 = strlen(topic2);

    if(topic1 <= 0 && topic2 <= 0)
        return false;
    if((temp - temp2) > 0)
    {
        if(topic2[temp2-1] != '#')
        {
            return false;
        }
        else
            maxLength = temp2;
    }
    else if((temp2 - temp) > 0)
    {
        if(topic1[temp-1] != '#')
        {
            return false;
        }
        else
            maxLength = temp;
    }
    else
        maxLength = temp;
    //compare strings to see if they are the same
    for(i=0; i<maxLength; i++)
    {
        difference = topic1[i] - topic2[i];
        if( difference != 0)
        {
            if(topic1[i] == '#')
            {
                if(i==0)
                {
                    if(topic1[i+1] == 0)
                        return true;
                }
                else
                {
                    if(topic1[i-1] == '/' && topic1[i+1] == 0)
                        return true;
                }
            }
            if(topic2[i] == '#')
            {
                if(i==0)
                {
                    if(topic2[i+1] == 0)
                        return true;
                }
                else
                {
                    if(topic2[i-1] == '/' && topic2[i+1] == 0)
                        return true;
                }
            }
            return false;
        }
    }
    return true;
}
int getTopicEntry(char* topic)
{
    int i;
    for(i=0; i<MAX_TOPICS; i++)
    {
        if(topicTable[i].valid)
        {
            if(strcmp(topicTable[i].name, topic) == 0)
                return i;
        }
    }
    return -1;
}
void doDeviceAction(uint8_t device, char* message)
{
    if(getTcpState() != ESTABLISHED)
        return;
    uint8_t packet[MAX_PACKET_SIZE];
    switch(device)
    {
    case TEMP_SENSOR:
        sendMqttPublish(packet, 0, 0, "rb/temp", message);
        break;
    case PUSH_BUTTON:
        sendMqttPublish(packet, 0, 0, "rb/pb", message);
        break;
    case BLUE_LED:
        if(strcmp(message, "on") == 0)
            setPinValue(BLUELED, 1);
        else if(strcmp(message, "off") == 0)
            setPinValue(BLUELED, 0);
        break;
    case CMD:
        putsUart0("Received MQTT message: ");
        putsUart0(message);
        putsUart0("\n\r");
        break;
    default:
        putsUart0("Unsupported Topic|message\n\r");
    }
}
//this is checking isr, that will be called every .1s
void checkInputs()
{
    int data;
    int newTemp;
    char str[10];

    //reading temperature sensor
    data = readAdc0Ss3();
    newTemp = 147.5 - ((75 * 3.3 * data)/4096);
    intToStr(str, newTemp);
//    putsUart0(str);
//    putsUart0("\n\r");
    if(c_abs(newTemp-deviceData[TEMP_SENSOR]) > 1 )
    {
        doDeviceAction(TEMP_SENSOR, str);
        deviceData[TEMP_SENSOR] = newTemp;
        _delay_cycles(16);
    }

    //push button reading
    data = getPinValue(PUSHBUTTON);
    if(data != deviceData[PUSH_BUTTON])
    {
        deviceData[PUSH_BUTTON] = data;
        if(data > 0)
            doDeviceAction(PUSH_BUTTON, "Button Released");
        else
            doDeviceAction(PUSH_BUTTON, "Button Pressed");
    }
}
void checkOutputs(char* topic, char* message)
{
    if(strcmp(topic, "rb/led/blue") == 0)
    {
//        outputData[BLUE_LED - NUM_INPUT_DEVICES] = message;
        doDeviceAction(BLUE_LED, message);
    }
    else if(strcmp(topic, "rb/cmd") == 0)
    {
//        outputData[CMD - NUM_INPUT_DEVICES] = message;
        doDeviceAction(CMD, message);
    }
    else if(strcmp(topic, "rb/udp") == 0)
    {

    }

}
void printOutputs()
{
    char str[10];
    int i;
    bool inTable = false;

    putsUart0("[ Description | Topic ]\n\r");

    // Red Board Blue Led
    for(i=0; i<MAX_TOPICS && !inTable; i++)
    {
        if(topicTable[i].valid)
            inTable = isTopicEntry("rb/led/blue", topicTable[i].name);
    }
    if(inTable)
        strcpy(str, "Enabled");
    else
        strcpy(str, "Disabled");
    putsUart0("Internal Blue LED:\t");
    putsUart0(str);
    putsUart0(":rb/led/blue");
    putsUart0("\n\r");

    // Command Line Interface
    memset(str, 0, 10);
    inTable = false;
    for(i=0; i<MAX_TOPICS && !inTable; i++)
    {
        if(topicTable[i].valid)
            inTable = isTopicEntry("rb/cmd", topicTable[i].name);
    }
    if(inTable)
        strcpy(str, "Enabled");
    else
        strcpy(str, "Disabled");
    putsUart0("Command Line Interface:\t");
    putsUart0(str);
    putsUart0(":rb/cmd");
    putsUart0("\n\r");
}
void printInputs()
{
    char str[10];
    intToStr(str, deviceData[TEMP_SENSOR]);
    putsUart0("Temp Sensor:\t");
    putsUart0(str);
    putsUart0("\t: rb/temp");
    putsUart0("\n\r");

    memset(str, 0, 10);
    intToStr(str, deviceData[PUSH_BUTTON]);
    putsUart0("Push Button:\t");
    putsUart0(str);
    putsUart0("\t: rb/pb");
    putsUart0("\n\r");

    memset(str, 0, 10);
    putsUart0("UDP Port:\t");
    putsUart0("N/A");
    putsUart0("\t: rb/pb");
    putsUart0("\n\r");
}
//void printTime()
//{
//
//}
void saveTopic(char* topic)
{
    int i;
    int open = -1;
    for(i=0; i<MAX_TOPICS; i++)
    {
        if(topicTable[i].valid)
        {
            if(strcmp(topicTable[i].name, topic) == 0)
                return;
        }
        else if(topicTable[i].valid == false)
            open = i;
    }
    if(open >= 0)
    {
        strcpy(topicTable[open].name, topic);
        topicTable[open].valid = true;
        memset(topicTable[open].value, 0, MAX_MSG_LENGTH);
    }
    else
        putsUart0("ERROR: No space to save topic.\n\r");
}
void removeTopic(char* topic)
{
    int i;
    for(i=0; i<MAX_TOPICS; i++)
    {
        if(topicTable[i].valid)
        {
            if(strcmp(topicTable[i].name, topic) == 0)
            {
                topicTable[i].valid = false;
            }
        }
    }
}
void removeAllTopics()
{
    int i;
    for(i=0; i<MAX_TOPICS; i++)
        topicTable[i].valid = false;
}
void printTopics()
{
    int i;
    int count = 0;
    char strCount[3];
    for(i=0; i<MAX_TOPICS; i++)
    {
        if(topicTable[i].valid == true)
        {
            intToStr(strCount,++count);
            putsUart0(strCount);
            putsUart0(": ");
            putsUart0(topicTable[i].name);
            putsUart0("\n\r");
        }
    }
    if(count == 0)
    {
        putsUart0("There are no subscriptions at this moment.\n\r");
    }
}
void updateTopic(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    uint32_t remainingLength=0;
    uint16_t messageLength=0;
    uint16_t topicLength=0;
    int topicEntryIndex;
    char topic[MAX_TOPIC_LENGTH];
    uint8_t *ptr;

    ptr = mqtt->end;
    remainingLength = getRemainingLength(&ptr);

    topicLength = *(uint16_t*)ptr;      // get two full bytes(topic length)
    topicLength = htons(topicLength);
    memset(topic, 0, MAX_TOPIC_LENGTH);
    ptr += 2;                           // jump over topic size bytes
    messageLength = remainingLength - 2 - topicLength;
    strncpy(topic, ptr, topicLength);
    ptr += topicLength;                 // jump over topic and message Id bytes

    if(mqtt->ctlHeader & 0x06)
    {
        ptr += 2;                       // if qos > 0 skip over message id
        messageLength -= 2;             // remove 2 message id bytes from message length since its not qos 1|2
    }

    if(messageLength > MAX_MSG_LENGTH)
        messageLength = MAX_MSG_LENGTH;
//    topicEntryIndex = getTopicEntry(topic);
    for(topicEntryIndex = 0; topicEntryIndex < MAX_TOPICS; topicEntryIndex++)
    {
        if(topicTable[topicEntryIndex].valid)
        {
            if(isTopicEntry(topic, topicTable[topicEntryIndex].name))
            {
                memset(topicTable[topicEntryIndex].value, 0, MAX_MSG_LENGTH);
                strncpy(topicTable[topicEntryIndex].value, ptr, messageLength);

                checkOutputs(topic, topicTable[topicEntryIndex].value);
            }
        }
    }
//    if(topicEntryIndex < 0)
//        return;
//
//    if(messageLength > MAX_MSG_LENGTH)
//        messageLength = MAX_MSG_LENGTH;
//    memset(topicTable[topicEntryIndex].value, 0, MAX_MSG_LENGTH);
//    strncpy(topicTable[topicEntryIndex].value, ptr, messageLength);
//
//    checkOutputs(topicTable[topicEntryIndex].name, topicTable[topicEntryIndex].value);
}

/*
 * rs485Table.c
 *
 *  Created on: Mar 26, 2020
 *      Author: Freddy Aguinaga
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "rs485Table.h"
#include "uart0.h"
#include "uart1.h"
#include "buffer.h"
#include "convert.h"
#include "timer.h"
#include "gpio.h"

#define MAX_QUEUE_SIZE 10
#define MAX_TRANSMISSIONS 5
enum Phases
{
    DEST_ADD    = 1,
    SRC_ADD     = 2,
    SEQ         = 3,
    COMMAND     = 4,
    CHANNEL     = 5,
    SIZE        = 6,
    DATA        = 7,
    CHECKSUM    = 8
};

//Will format the infomation for transmissions.
//Size is 8bytes + N-1 bytes(data)
typedef struct _rxEntry
{
    bool valid;                     // entry is complete and ready to be executed
    uint8_t destAdd;                // Destination address: used to verify we are the target machine
    uint8_t srcAdd;                 // Source address
    uint8_t seqId;                  // sequence id of message
    uint8_t cmd;                    // command
    uint8_t chan;                   // channel
    uint8_t size;                   // size of data arguments
    uint8_t checksum;               // checksum of message
    uint8_t data[100];              // data for arguments
}rxEntry;
typedef struct _txEntry
{
    bool valid;                     // ready status for transmission
    uint32_t backoff;               // delayed transmission in ms
    uint8_t ttg;                    // Transmissions left to attempt to successfully send
    uint8_t destAdd;                // Destination Address
    uint8_t srcAdd;                 // Source Address
    uint8_t seqId;                  // Sequence Id
    uint8_t cmd;                    // Command being executed
    uint8_t chan;                   // Channel (Sub-Address)
    uint8_t size;                   // Data Size
    uint8_t checksum;               // Checksum
    uint8_t data[100];              // Data for arguments
}txEntry;

uint8_t hwAddress = 0;              // Device's own address

//txTable Globals
txEntry txTable[MAX_QUEUE_SIZE];
int index = 0;                      // Current spot in the Tx table being sent/prepared
uint8_t txRow = 0;                  // Determines which Tx entry is being saved in
uint8_t phase = 0;                  // Determines the item in Tx entry is being sent
uint8_t sequenceId = 0;             // Sequence id for messages sent
bool ackReq = false;                // Ack required
bool random = false;                // rng generation of backoff value

//rxTable Globals
rxEntry rxTable[MAX_QUEUE_SIZE];
int indexRx = 0;
uint8_t phaseRx = 1;

//prints a transmission message for a given entry in the Tx Table
void printTxMessage(uint8_t index)
{
    char id[2];
    char attempt[2];

    intToStr(id ,txTable[index].seqId);
    if(txTable[index].cmd & 0x80)
        intToStr(attempt, MAX_TRANSMISSIONS-txTable[index].ttg + 1);
    else
        intToStr(attempt, 1);

    writeBuffer("Transmitting Msg ");
    writeBuffer(id);
    writeBuffer(", Attempt ");
    writeBuffer(attempt);
    writeBuffer(": ");
}
void incId()
{
    sequenceId++;
}
uint8_t getAddress()
{
    return hwAddress;
}
int getRxCommandIndex()
{
    int i = indexRx;
    int temp = indexRx;
    if(i == -1)
    {
        i = 0;
        temp = 0;
    }
    //Cycle through the table starting at the row after the current row
    do
    {
        if(rxTable[i].valid == true)
            return i;
        i = (i+1) % MAX_QUEUE_SIZE;
    }while(i != temp);

    return -1;
}
uint8_t getRxCommandSrc(uint8_t idx)
{
    return rxTable[idx].srcAdd;
}
uint8_t getRxCommandId(uint8_t idx)
{
    return rxTable[idx].seqId;
}
uint8_t getRxCommandType(uint8_t idx)
{
    return rxTable[idx].cmd;
}
uint8_t getRxCommandChannel(uint8_t idx)
{
    return rxTable[idx].chan;
}
uint8_t getRxCommandDataSize(uint8_t idx)
{
    return rxTable[idx].size;
}
uint8_t* getRxCommandData(uint8_t idx)
{
    return rxTable[idx].data;
}
uint8_t getTransmissions(uint8_t idx)
{
    return txTable[idx].ttg;
}
int getIndex()
{
    return index;
}
int getIndexRx()
{
    return indexRx;
}
uint8_t getTxRow()
{
    return txRow;
}
uint8_t getPhase()
{
    return phase;
}
uint8_t getPhaseRx()
{
    return phaseRx;
}
uint8_t getId()
{
    return sequenceId;
}
bool getAck()
{
    return ackReq;
}
bool getTxAck(uint8_t idx)
{
    if(txTable[idx].cmd & 0x80)
        return true;
    return false;
}
void setAddress(uint8_t add)
{
    hwAddress = add;
}
void setAck(bool t)
{
    ackReq = t;
}
void setRandom(bool s)
{
    random = s;
}
void primePump(uint8_t entryIndex)
{
    phase = 1;
    index = entryIndex;
    sendMessage();
}
// finds next valid message to be sent out
int findNextMsg()
{
    int i = txRow;
    do
    {
        if(txTable[i].valid == true && txTable[i].backoff == 0)
            return i;
    }while((i+=1) % MAX_QUEUE_SIZE != txRow);
    return -1;
}
// returns the first invalid entry's index in the txTable
int findEntrySpotTx()
{
    int i = txRow;

    //Cycle through the table starting at the row after the current row
    do
    {
        if(txTable[i].valid == false)
            return i;
    }while(((i += 1) % MAX_QUEUE_SIZE) != txRow);

    return -1;
}
int findEntrySpotRx()
{
    int i = indexRx;
    if(indexRx == -1)
    {
        indexRx = 0;
        i = 0;
    }
    //Cycle through the table starting at the row after the current row
    do
    {
        if(rxTable[i].valid == false)
            return i;
        i = (i+1) % MAX_QUEUE_SIZE;
    }while(i != indexRx);

    return -1;
}
// returns true if it is able to successfully place a new command into the queue, false otherwise
int queueMessage(uint8_t destination, uint8_t command, uint8_t channel, uint8_t cmdData[], uint8_t dataSize)
{
    uint32_t i;
    int idx = findEntrySpotTx();

    if(idx == -1)
        return -1;
    incId();
    //Computing checksum while filling in the fields of new txEntry
    txEntry* currentEntry = &txTable[idx];
    currentEntry->checksum = 0;
    currentEntry->checksum += currentEntry->destAdd = destination;
    currentEntry->checksum += currentEntry->srcAdd = hwAddress;
    currentEntry->checksum += currentEntry->seqId = sequenceId;
    currentEntry->checksum += currentEntry->cmd = (command & 0x7F);
    currentEntry->checksum += currentEntry->chan = channel;
    currentEntry->checksum += currentEntry->size = dataSize;
    if(ackReq && ((currentEntry->cmd & 0x70) == 0))
    {
        currentEntry->ttg = MAX_TRANSMISSIONS;  // set for max transmissions
        currentEntry->cmd += 0x80;              // mark ack required in 8th bit of cmd
        currentEntry->checksum += 0x80;         // take into account the ack req bit
}
    else
    {
        currentEntry->ttg = 1;
    }
    for(i=0; i<dataSize; i++)
        currentEntry->checksum += currentEntry->data[i] = cmdData[i];

    // Negate sum to complete 1s complement of modulo-256 checksum
    currentEntry->checksum = ~(currentEntry->checksum);
    currentEntry->backoff = 0;
    currentEntry->valid = true;

    return idx;
}

// Will check if a message is available to send and send one byte depending
// on what phase of the message sending stage we are in at the moment.
void sendMessage()
{
    char byteStr[3];
    char id[3];
    static uint8_t dataIndex;

    switch(phase)
    {
    case 0:    //check if a new message needs to be set up to be sent
        index = findNextMsg();
        if(index == -1)
            return;

        dataIndex = 0;
        break;
        //Uart1 in each case
    case DEST_ADD:
        enableParityBitUart1();
        intToStrHex(byteStr, txTable[index].destAdd);
        putcUart1(txTable[index].destAdd);
        break;
    case SRC_ADD:
        disableParityBitUart1();
        intToStrHex(byteStr, txTable[index].srcAdd);
        putcUart1(txTable[index].srcAdd);
        break;
    case SEQ:
        intToStrHex(byteStr, txTable[index].seqId);
        putcUart1(txTable[index].seqId);
        break;
    case COMMAND:
        intToStrHex(byteStr, txTable[index].cmd);
        putcUart1(txTable[index].cmd);
        break;
    case CHANNEL:
        intToStrHex(byteStr, txTable[index].chan);
        putcUart1(txTable[index].chan);
        break;
    case SIZE:
        intToStrHex(byteStr, txTable[index].size);
        putcUart1(txTable[index].size);
        break;
    case DATA:
        intToStrHex(byteStr, txTable[index].data[dataIndex]);
        putcUart1(txTable[index].data[dataIndex]);
        break;
    case CHECKSUM:
        intToStrHex(byteStr, txTable[index].checksum);
        putcUart1(txTable[index].checksum);
        break;
    default:
        writeBuffer("Error: invalid Tx phase");
        break;
    }
    if(phase > 0 && phase < 9)
    {
        writeBuffer(byteStr);
        writeBuffer(" ");
    }
    if(phase == 7 && dataIndex < txTable[index].size-1)
    {
        dataIndex++;
    }
    else if(phase == 8)
    {
        dataIndex = 0;
        phase = 0;
        txTable[index].ttg = txTable[index].ttg-1;
        if(txTable[index].ttg == 0)
        {
            txTable[index].valid = false;
            if(txTable[index].cmd & 0x80)
            {
                writeBuffer("\n\rFailed: #");
                intToStr(id, txTable[index].seqId);
                writeBuffer(id);
            }
        }
        else
        {
            txTable[index].backoff = calcBackoff(MAX_TRANSMISSIONS - txTable[index].ttg);
        }
        writeBuffer("\n\r");
    }
    else
    {
        dataIndex = 0;
        phase++;
    }
}
/* Returns period for backoff depending on attempt in ms
 *  n = attempt
 *  T = fixed period
 */
uint32_t calcBackoff(uint32_t attempt)
{
    uint32_t varied = 0;
    uint32_t fixed = 500;

    //(2^n)*T part
    for(varied = 1; attempt>0; attempt--)
        varied *= 2;
    varied = fixed*varied;

    if(random)
    {
        // backoff = number between 0 - (T + (2^n)*T)
        return (random32() % varied) + fixed;
    }
    else
    {
        // backoff = T + (2^n)*T
        return fixed + varied;
    }
}
bool decrementBackoffs()
{
    int i;
    bool active = false;
    for(i = 0; i<MAX_QUEUE_SIZE; i++)
    {
        if(txTable[i].valid == true && txTable[i].backoff > 0)
        {
            txTable[i].backoff -= 1;
            active = true;
        }
    }
    return active;
}
//receives a byte at a time and will return true when a complete message is received and correct
bool receiveMessage()
{
    static uint8_t dataIndexRx;
    static uint8_t checksumRx;

    uint8_t data;
    char str[3];
    data = getcUart1();
    //first character must be
    if(isParityError())
    {
        clearParityError();
        phaseRx = 1;
//        writeBuffer("*");
        if(phaseRx != DEST_ADD)
        {
            return false;
        }
    }
    else if(phaseRx == DEST_ADD)
        return false;

    switch(phaseRx)
    {
    case DEST_ADD:
        indexRx = findEntrySpotRx();
        if(indexRx == -1)
            return false;
        if(data != hwAddress && data != 0xFF)
            return false;
        checksumRx = rxTable[indexRx].destAdd = data;
        dataIndexRx = 0;
        break;
    case SRC_ADD:
        checksumRx += rxTable[indexRx].srcAdd = data;
        break;
    case SEQ:
        checksumRx += rxTable[indexRx].seqId = data;
        break;
    case COMMAND:
        checksumRx += rxTable[indexRx].cmd = data;
        break;
    case CHANNEL:
        checksumRx += rxTable[indexRx].chan = data;
        break;
    case SIZE:
        checksumRx +=  rxTable[indexRx].size = data;
        break;
    case DATA:
        checksumRx += rxTable[indexRx].data[dataIndexRx] = data;
        break;
    case CHECKSUM:
        rxTable[indexRx].checksum = data;
        break;
    default:
        writeBuffer("Error: Invalid Rx Phase");
        break;
    }
    //Remove comment lines to debug receiving bits
    intToStrHex(str,data);
    writeBuffer(str);
    writeBuffer(" ");
    if(phaseRx == DATA && dataIndexRx < rxTable[indexRx].size-1)
    {
        dataIndexRx++;
    }
    else if(phaseRx == CHECKSUM)
    {
        phaseRx = DEST_ADD;
        checksumRx = ~(checksumRx);

        //Set a visual mark whether we received the message correctly
        if(checksumRx == rxTable[indexRx].checksum)
        {
//            writeBuffer("[m]");
            rxTable[indexRx].valid = true;
        }
        else
        {
//            writeBuffer("[i]");
        }
        checksumRx = 0;
        writeBuffer("\n\r");

        return rxTable[indexRx].valid;
    }
    else
    {
        phaseRx++;
        dataIndexRx = 0;
    }
    return false;
}
void finishedRxCommand(uint8_t idx)
{
    rxTable[idx].valid = false;
}
void ackCommand(uint8_t ackdSeq)
{
    uint8_t i;

    for(i=0; i<MAX_QUEUE_SIZE; i++)
    {
        if(txTable[i].seqId == ackdSeq)
        {
            txTable[i].valid = false;
            break;
        }
    }
}

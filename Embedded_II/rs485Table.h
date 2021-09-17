/*
 * rs485Table.h
 *
 *  Created on: Mar 26, 2020
 *      Author: aguin
 */

#ifndef RS485TABLE_H_
#define RS485TABLE_H_

void printTxMessage(uint8_t index);
void incId();
uint8_t getAddress();
int getIndex();
int getIndexRx();
int getRxCommandIndex();
uint8_t getRxCommandSrc(uint8_t idx);
uint8_t getRxCommandId(uint8_t idx);
uint8_t getRxCommandType(uint8_t idx);
uint8_t getRxCommandChannel(uint8_t idx);
uint8_t getRxCommandDataSize(uint8_t idx);
uint8_t* getRxCommandData(uint8_t idx);
uint8_t getTransmissions(uint8_t idx);
uint8_t getId();
uint8_t getTxRow();
uint8_t getPhase();
uint8_t getPhaseRx();
bool getTxAck(uint8_t idx);
bool getAck();
void setAddress(uint8_t add);
void setAck(bool t);
void setRandom(bool s);
void primePump(uint8_t entryIndex);
int findNextMsg();
int findEntrySpotTx();
int findEntrySpotRx();
int queueMessage(uint8_t destination, uint8_t command, uint8_t channel, uint8_t cmdData[], uint8_t dataSize);
void sendMessage();
uint32_t calcBackoff(uint32_t attempt);
bool decrementBackoffs();
bool receiveMessage();
void ackCommand(uint8_t ackdSeq);
void finishedRxCommand(uint8_t idx);

#endif /* RS485TABLE_H_ */

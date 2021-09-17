/*
 * devices.h
 *
 *  Created on: May 2, 2020
 *      Author: aguin
 */

#ifndef DEVICES_H_
#define DEVICES_H_

void initDevices();
void doDeviceAction(uint8_t device, char* message);
void checkInputs();
void checkOutputs(char* topic, char* message);
void printInputs();
void printOutputs();
//void printTime();
bool isTopicEntry(char* topic1, char* topic2);
int getTopicEntry(char* topic);
void saveTopic(char* topic);
void removeTopic(char* topic);
void removeAllTopics();
void updateTopic(uint8_t packet[]);
void printTopics();

#endif /* DEVICES_H_ */

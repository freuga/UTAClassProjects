/*
 * convert.h
 *
 *  Created on: Mar 22, 2020
 *      Author: aguin
 */

#ifndef CONVERT_H_
#define CONVERT_H_

int c_strcmp(char* str1, char* str2);
int c_strlen(char* str);
int strToInt(char* number);
void intToStr(char* str, int number);
void intToStrHex(char* str, int number);
uint8_t strHexToUint8(char* str);
uint32_t arrayTo32(uint8_t* data, uint8_t size);
uint32_t c_abs(int number);

#endif /* CONVERT_H_ */

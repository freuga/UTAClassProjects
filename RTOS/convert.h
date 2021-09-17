/*
 * convert.h
 *
 *  Created on: Mar 25, 2020
 *      Author: aguin
 */

#ifndef CONVERT_H_
#define CONVERT_H_
int c_strcmp(char* str1, char* str2);
int c_strlen(char* str);
int c_strcpy(char* destStr, char* srcStr);
int strToInt(char* number);
bool isDelim(char c, char* delim);
bool isNumber(char* str);
void intToStr(char* str, int number);
void intToStrHex(char* str, int number);
void u32ToStr(char destStr[], uint32_t number);
void u32ToStrHex(char* destStr, uint32_t number);
uint8_t strHexToUint8(char* str);
uint32_t arrayTo32(uint8_t* data, uint8_t size);
#endif /* CONVERT_H_ */

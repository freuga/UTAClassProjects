/*
 * convert.c
 *
 *  Created on: Mar 25, 2020
 *      Author: aguin
 */
//#include<stdio.h>
#include<stdint.h>
#include<stdbool.h>

//compares two strings and returns the difference of last character or first differing character
int c_strcmp(char* str1, char* str2)
{
    int sum = 0;
    while(*str1 != '\0' && *str2 != '\0')
    {
        sum = *str1 - *str2;
        if(sum != 0)
            break;
        str2++;
        str1++;
    }
    return *str1-*str2;
}
//returns number of characters copied from srcStr to destStr, destStr size must be equal or greater than srcStr
int c_strcpy(char destStr[], char srcStr[])
{
    int i=0;
    while(srcStr[i] != '\0')
    {
        destStr[i] = srcStr[i];
        i++;
    }
    destStr[i] = '\0';
    return i;
}
//returns length of string argument
int c_strlen(char* str)
{
    int i=0;
    while(*str != 0)
    {
        str++;
        i++;
    }
    return i;
}
bool isDelim(char c, char* delim)
{
    while(*delim != '\0')
    {
        if(c == *delim)
            return true;
        delim++;
    }
    return false;
}
bool isNumber(char* str)
{
    if(str == 0)
        return false;

    while(*str != '\0')
    {
        if(*str >= '0' && *str <= '9')
            str++;
        else
            return false;
    }
    return true;
}
int strToInt(char* number)
{
    int sum = 0;
    int index = c_strlen(number)-1;                           //start from right to left(LSB)
    int placeholder = 1;                                    // start from one's place
    while(number[index] && index >= 0)
    {
        if(number[index] < '0' || number[index] > '9')      //if an invalid character is entered, return error
        {
            return -1;
        }
        sum += (number[index]-'0')*placeholder;              // add to sum.
        placeholder = placeholder*10;                       // shift to left in significance
        index--;
    }
    return sum;
}
void intToStr(char str[], int number)
{
   int digit = 0;
   int index = 0;
   int temp = number;

   if(number == 0)
   {
       index = 1;
   }
   else
   {
       //Get length of number
       while(temp)
       {
           temp = temp/10;
           index++;
       }
   }
   //Starting from MSD and moving to LSD
   str[index] = 0;
   while(index)
   {
       index--;
       digit = number%10;
       str[index] = digit+48;
       number = number/10;
   }
}
// converts 4 byte number to a string
void u32ToStr(char destStr[], uint32_t number)
{
    uint32_t i, placeholder;
    i = 0;
    placeholder = 1000000000;
    if(number == 0)
    {
        destStr[0] = '0';
        destStr[1] = '\0';
        return;
    }
    while(!(number/placeholder) && placeholder > 0)
    {
        placeholder /= 10;
    }
    //biggest 32 bit number will only reach 10 digits
    while(placeholder > 0)
    {
        //get current number at the placeholder & convert to character
        destStr[i] = number/placeholder + '0';
        number %= placeholder;
        placeholder /= 10;
        i++;
    }
    destStr[i] = '\0';
}
// number is a 4 byte number, destStr must be 9 elements long
// converts unsigned int to hex as a string
void u32ToStrHex(char* destStr, uint32_t number)
{
    uint32_t i, c_num;
    i = 8;
    while(i > 0)
    {
        i--;
        c_num = number % 0x10;
        if(c_num < 10)
            c_num += '0';
        else
        {
            c_num -= 10;
            c_num += 'A';
        }
        number = number / 0x10;
        destStr[i] = c_num;
    }
    destStr[8] = '\0';
}
//Converts int number to hex format for a byte as a string
void intToStrHex(char* str, uint8_t number)
{
    uint8_t high = number/16;
    uint8_t low = number%16;
    if(low < 10)
        str[1] = low + '0';
    else
        str[1] = low + 'A' - 10;

    if(high < 10)
        str[0] = high + '0';
    else
        str[0] = high + 'A' - 10;
    str[2] = '\0';
}
uint8_t strHexToUint8(char* str)
{
    int i=0;
    uint8_t placeholder = 1;

    uint8_t length = c_strlen(str);
    uint8_t sum = 0;
    char c;
    //start from LSD and work to the MSD summing up in base 16
    for(i=length-1; i>=0; i--)
    {
        c = str[i];
        //Convert lowercase hex numbers to uppercase
        if(c >= 'a' && c<='f')
        {
            c = c - ('a'-'A');
        }
        if(c >='0' && c <= '9')
        {
            sum += (c-'0')*placeholder;
        }
        else if(c>='A' &&  c<='F')
        {
            sum += (c-'A'+10)*placeholder;
        }
        placeholder = placeholder*16;
    }
    return sum;
}
uint32_t arrayTo32(uint8_t* data, uint8_t size)
{
    if(size > 4)
        return 0;
    int i;
    uint32_t total = 0;
    uint32_t placeholder = 1;
    for(i=size-1; i>=0; i--)
    {
        total += (data[i] & 0x0F)*placeholder;
        total += (data[i] & 0xF0)*placeholder;
        placeholder = placeholder << 8;
    }
    return total;

}

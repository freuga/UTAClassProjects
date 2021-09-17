/*
 * convert.c
 *
 *  Created on: Mar 25, 2020
 *      Author: aguin
 */
#include<stdio.h>
#include<stdint.h>

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
uint32_t c_abs(int number)
{
    if(number < 0)
        return number * -1;
    return number;
}

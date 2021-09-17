#include <stdint.h>
#include <stdbool.h>
#include "mpu.h"
#include "tm4c123gh6pm.h"
#include "gpio.h"
#include "uart0.h"
#include "convert.h"

extern uint32_t getPSP(void);
extern uint32_t getMSP(void);
extern uint32_t getControl(void);
extern void setPSP(uint32_t add);
extern void setControl(uint32_t val);

// peripheral definitions
#define PRIVILEGE PORTE,5
#define RED_LED PORTE,1
#define ORANGE_LED PORTE,2
#define YELLOW_LED PORTE,3
#define GREEN_LED PORTE,4
#define BLUE_LED PORTF,2
#define BUTTON_0 PORTA,2
#define BUTTON_1 PORTA,3
#define BUTTON_2 PORTA,4
#define BUTTON_3 PORTA,5
#define BUTTON_4 PORTA,6
#define BUTTON_5 PORTA,7

int pid;
void setPrivilegeMode(bool on)
{
    if(on)
    {
        setPinValue(PRIVILEGE, 1);
        setPinValue(PRIVILEGE, 0);
    }
    else
    {
        setControl(0x3);
    }
}
void incrementPid()
{
    // if currently unprivileged, temporarily grant privileged access to increment pid
    if(getControl() & 0x1)
    {
        setPrivilegeMode(true);
        pid++;
        setPrivilegeMode(false);
    }
    else
        pid++;
}
void print4ByteHex(uint32_t temp)
{
    char hexStr[9];
    u32ToStrHex(hexStr, temp);
    putsUart0("0x");
    putsUart0(hexStr);
}
void printByteHex(uint8_t temp)
{
    char hexStr[9];
    intToStrHex(hexStr, temp);
    putsUart0("0x");
    putsUart0(hexStr);
}

void FaultISR(void)
{
    char pidNum[2];
    intToStr(pidNum, pid);
    putsUart0("hard fault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

    putsUart0("PSP:\t");
    print4ByteHex(getPSP());
    putsUart0("\n\rMSP:\t");
    print4ByteHex(getMSP());
    putsUart0("\n\rHFF:\t");
    print4ByteHex(getHFaultFlags());
    putsUart0("\n\r");

    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_BUSA;
}
// ISRS
void MpuISR(void)
{
    char pidNum[2];

    clearMemPending();
    intToStr(pidNum, pid);
    putsUart0("Mpu fault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

    // Displaying Stack pointer positions and Error information
    putsUart0("PSP:\t");
    print4ByteHex(getPSP());
    putsUart0("\n\rMSP:\t");
    print4ByteHex(getMSP());
    putsUart0("\n\rMFF:\t");
    printByteHex(getMemFaultFlags());
    putsUart0("\n\r");

    //Register Dump
    int i;
    uint32_t* psp = (uint32_t*) getPSP();
    for(i=0; i<8; i++)
    {
        switch(i)
        {
        case 0: putsUart0("R0:\t");  break;
        case 1: putsUart0("R1:\t");  break;
        case 2: putsUart0("R2:\t");  break;
        case 3: putsUart0("R3:\t");  break;
        case 4: putsUart0("R12:\t"); break;
        case 5: putsUart0("LR:\t");  break;
        case 6: putsUart0("PC:\t");  break;
        case 7: putsUart0("XPSR:\t");break;
        }
        print4ByteHex(*(psp + i));
        putsUart0("\n\r");
    }
    //displaying instruction causing violation
    putsUart0("Intruction ");
    print4ByteHex(*(psp+6));
    putsUart0(" accessed ");
    print4ByteHex(getMemAccessAddr());
    putsUart0("\n\r");

    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}
void BusISR(void)
{
    char pidNum[2];
    intToStr(pidNum, pid);
    putsUart0("Bus fault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

}
void UsageISR(void)
{
    char pidNum[2];
    intToStr(pidNum, pid);
    putsUart0("Usage fault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

}
void SVCallISR(void)
{
    char pidNum[2];
    intToStr(pidNum, pid);
    putsUart0("SVCallfault in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");
}
void PendsvISR(void)
{
    char pidNum[2];
    intToStr(pidNum, pid);
    putsUart0("PendSV in process ");
    putsUart0(pidNum);
    putsUart0("\n\r");

    // check DERR & IERR bits in Fault Status register
    if(getMemFaultFlags() & 0x3)
    {
        putsUart0("Called by MPU\n\r");
    }
}
void SystickISR(void)
{
    char pidNum[2];
    intToStr(pidNum, pid);
    putsUart0("SysTick in process ");
    putsUart0(pidNum);
}
void PrivilegeISR(void)
{
    setControl(0x2);
    GPIO_PORTE_ICR_R |= 0xFF;
}
void testISRs(uint8_t type)
{
    /*
     * 0: Usage fault
     * 1: bus fault
     * 2: hard fault
     * 3: mpu fault
     */

    uint32_t* p;
    switch(type)
    {
    case 0:
        if(getControl() & 0x1)
        {
            setPrivilegeMode(true);
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGEP;
            setPrivilegeMode(false);
        }
        else
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_USAGEP;
        break;
    case 1:
        if(getControl() & 0x1)
        {
            setPrivilegeMode(true);
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_BUSP;
            setPrivilegeMode(false);
        }
        else
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_BUSP;
        break;
    case 2:
        if(getControl() & 0x1)
        {
            setPrivilegeMode(true);
            NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_MEM);
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEMP;
            NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEM;
            setPrivilegeMode(false);
        }
        else
        {
            NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_MEM);
            setPrivilegeMode(false);
            p = (uint32_t*) 0x20000FFC;
            *p = 0;
            setPrivilegeMode(true);
            NVIC_SYS_HND_CTRL_R |= (NVIC_SYS_HND_CTRL_MEM);
        }
        break;
    case 3:
        //not protected
        p = (uint32_t*) 0x20007FFC;
        *p = 0;
        //protected
        p = (uint32_t*) 0x20000FFC;
        *p = 0;
        break;
    }
}
void isCommand(char** fields, uint8_t num_fields)
{
    char* command = fields[0];
    if(c_strcmp(command, "reboot") == 0)
    {
        if(num_fields > 1)
            putsUart0("reboot: invalid argument\n\r");
        else
        {
            NVIC_APINT_R = NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
        }
    }
    else if(c_strcmp(command, "ps") == 0)
    {
        if(num_fields > 1)
            putsUart0("ps: invalid argument\n\r");
        else
        {
            putsUart0("PS called\n\r");
        }
    }
    else if(c_strcmp(command, "ipcs") == 0)
    {
        if(num_fields > 1)
            putsUart0("ipcs: invalid argument\n\r");
        else
        {
            putsUart0("IPCS called\n\r");
        }
    }
    else if(c_strcmp(command, "kill") == 0)
    {
        if(num_fields != 2 || (num_fields == 2 && !isNumber(fields[1])) )
            putsUart0("kill: invalid argument\n\r");
        else
        {
            if(isNumber(fields[1]))
            {
                putsUart0("pid ");
                putsUart0(fields[1]);
                putsUart0(" killed");
                putsUart0("\n\r");
            }
        }
    }
    else if(c_strcmp(command, "pi") == 0)
    {
        if(num_fields != 2)
            putsUart0("pi: invalid argument\n\r");
        else
        {
            if(c_strcmp(fields[1], "on") == 0)
            {
                setPrivilegeMode(true);
                putsUart0("pi ON\n\r");
            }
            else if(c_strcmp(fields[1], "off") == 0)
            {
                setPrivilegeMode(false);
                putsUart0("pi OFF\n\r");
            }
            else
            {
                putsUart0("pi: invalid argument\n\r");
            }
        }
    }
    else if(c_strcmp(command, "preempt") == 0)
    {
        if(num_fields != 2)
            putsUart0("preempt: invalid argument");
        else
        {
            if(c_strcmp(fields[1], "on") == 0)
            {
                putsUart0("preempt ON\n\r");
            }
            else if(c_strcmp(fields[1], "off") == 0)
            {
                putsUart0("preempt OFF\n\r");
            }
            else
            {
                putsUart0("preempt: invalid argument\n\r");
            }
        }
    }
    else if(c_strcmp(command, "sched") == 0)
    {
        if(num_fields != 2)
            putsUart0("sched: invalid argument\n\r");
        else
        {
            if(c_strcmp(fields[1], "prio") == 0)
            {
                putsUart0("sched prio\n\r");
            }
            else if(c_strcmp(fields[1], "rr") == 0)
            {
                putsUart0("sched rr\n\r");
            }
            else
            {
                putsUart0("sched: invalid argument\n\r");
            }
        }
    }
    else if(c_strcmp(command, "pidof") == 0)
    {
        if(num_fields != 2)
            putsUart0("pidof: invalid argument\n\r");
        else
        {
            putsUart0(fields[1]);
            putsUart0(" launched.");
            putsUart0("\n\r");
        }
    }
    else if(c_strcmp(command, "run") == 0)
    {
        if(num_fields != 2)
            putsUart0("run: invalid argument\n\r");
        else
        {
            setPinValue(RED_LED, getPinValue(RED_LED)^1);
        }
    }
    else if(c_strcmp(command, "test") == 0)
    {
        if(num_fields == 2)
        {
            if(isNumber(fields[1]))
            {
                testISRs((uint8_t) strToInt(fields[1]));
            }
            else if(c_strcmp(fields[1], "hw") == 0)
            {
                testHw();
            }
        }
        else
        {
            putsUart0("test: Invalid argument\n\r");
        }
    }
    else
    {
        putsUart0("Invalid Command\n\r");
        return;
    }
    incrementPid();
}
// returns the number of fields
uint32_t parseFields(char* buffer, char** fields)
{
    char* delim = " ,";
    uint8_t bufferSize;
    bufferSize = c_strlen(buffer);
    uint8_t c_index = 0;
    uint8_t f_index = 0;

    if(bufferSize == 0)
        return 0;

    //check first index
    if(isDelim(buffer[c_index], delim))
    {
        buffer[c_index] = '\0';
    }
    else
    {
        fields[f_index] = buffer;
        f_index++;
    }
    c_index++;
    //everything after, if prefixed with delim, it is a new token. Max fields 5
    while(c_index < bufferSize && f_index < 5)
    {
        if(isDelim(buffer[c_index], delim))
        {
            buffer[c_index] = '\0';
        }
        else
        {
            if(buffer[c_index-1] == '\0')
            {
                fields[f_index] = &buffer[c_index];
                f_index++;
            }
        }
        c_index++;
    }
    return f_index;
}
void testHw()
{

    while(getPinValue(BUTTON_0));

    setPinValue(RED_LED, 1);
    setPinValue(ORANGE_LED, 1);
    setPinValue(GREEN_LED, 1);
    setPinValue(YELLOW_LED, 1);
    setPinValue(BLUE_LED, 1);

    while(getPinValue(BUTTON_1));

    setPinValue(RED_LED, 0);
    setPinValue(ORANGE_LED, 0);
    setPinValue(GREEN_LED, 0);
    setPinValue(YELLOW_LED, 0);
    setPinValue(BLUE_LED, 0);

    while(getPinValue(BUTTON_2));

    setPinValue(RED_LED, 1);
    setPinValue(ORANGE_LED, 1);
    setPinValue(GREEN_LED, 1);
    setPinValue(YELLOW_LED, 1);
    setPinValue(BLUE_LED, 1);

    while(getPinValue(BUTTON_3));

    setPinValue(RED_LED, 0);
    setPinValue(ORANGE_LED, 0);
    setPinValue(GREEN_LED, 0);
    setPinValue(YELLOW_LED, 0);
    setPinValue(BLUE_LED, 0);

    while(getPinValue(BUTTON_4));

    setPinValue(RED_LED, 1);
    setPinValue(ORANGE_LED, 1);
    setPinValue(GREEN_LED, 1);
    setPinValue(YELLOW_LED, 1);
    setPinValue(BLUE_LED, 1);

    while(getPinValue(BUTTON_5));

    setPinValue(RED_LED, 0);
    setPinValue(ORANGE_LED, 0);
    setPinValue(GREEN_LED, 0);
    setPinValue(YELLOW_LED, 0);
    setPinValue(BLUE_LED, 0);
}
void testUart0()
{
//    char number[3];
    char commandLine[80];               // had to make constant cause error due to variable?
    char* fields[5];                    // ^^^^
    uint8_t numFields = 0;
    uint8_t index = 0;

    //run endless loop for commands
    while(1)
    {
        while(!getsUart0(commandLine, &index));
        putsUart0("\n\r");
        numFields = parseFields(commandLine, fields);
        if(numFields > 0)
        {
            isCommand(fields, numFields);
        }
//        for(i=0; i<numFields; i++)
//        {
//            intToStr(number, i);
//            putsUart0(number);
//            putsUart0(": ");
//            putsUart0(fields[i]);
//            putsUart0("\n\r");
//        }
        commandLine[0] = '\0';
        numFields = 0;
    }
}

void initHw()
{
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);
    _delay_cycles(3);


    enablePort(PORTA);
    enablePort(PORTE);
    enablePort(PORTF);

    selectPinPushPullOutput(RED_LED);       // RED LED
    selectPinPushPullOutput(ORANGE_LED);    // ORANGE LED
    selectPinPushPullOutput(YELLOW_LED);    // YELLOW LED
    selectPinPushPullOutput(GREEN_LED);     // GREEN LED
    selectPinPushPullOutput(BLUE_LED);      // BLUE LED(ON BOARD)
    selectPinPushPullOutput(PRIVILEGE);     // PRIVILEGE

    enablePinInterrupt(PRIVILEGE);
    selectPinInterruptFallingEdge(PRIVILEGE);
    NVIC_EN0_R |= (1 << 4);

    enablePinPullup(BUTTON_0);
    enablePinPullup(BUTTON_1);
    enablePinPullup(BUTTON_2);
    enablePinPullup(BUTTON_3);
    enablePinPullup(BUTTON_4);
    enablePinPullup(BUTTON_5);

    selectPinDigitalInput(BUTTON_0);
    selectPinDigitalInput(BUTTON_1);
    selectPinDigitalInput(BUTTON_2);
    selectPinDigitalInput(BUTTON_3);
    selectPinDigitalInput(BUTTON_4);
    selectPinDigitalInput(BUTTON_5);

}
/**
 * main.c
 */
int main(void)
{
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);

    setPSP(0x20008000);                             //setting PSP at top of SRAM
    setControl(0x2);                                // setting ASP for PSP

    initMpu();                                      // setting up memory protection regions
    enableSysFaults();                              // enabling system faults


    testUart0();
    while(1)
    {

    }
	return 0;
}

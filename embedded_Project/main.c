#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>
#include "tm4c123gh6pm.h"

//bit band stuff
#define GPI     (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 0*4)))
#define GPO     (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 5*4)))
#define SENSOR  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 3*4)))
#define SPEAKER (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define PANIC_BUTTON (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 0*4)))

//Motor pins bit band
#define RED    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define WHITE   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define YELLOW  (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 6*4)))
#define BLUE    (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))

//Port F Button and LEDs
#define PANIC_BUTTON_MASK 1
#define BUTTON_MASK 16
#define RED_LED_MASK 2
#define GREEN_LED_MASK 8

//MOTOR pins
#define RED_MASK 64     //PC6
#define WHITE_MASK 32   //PC5
#define YELLOW_MASK 64  //PB6
#define BLUE_MASK 128   //PC7

//Speaker pin
#define SPEAKER_MASK 16 // pin PC4 for speaker output

//Port B pins for IR sensor & light sensor
#define SENSOR_MASK 8   // pin PB3 mask for status mechanism
#define GPI_MASK 1      //pin PB0 input from IR receiver
#define GPO_MASK 32     // pin PB5 output for IR receiver

#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

//NOTE: Timing values may differ from original excel sheet
//      41th and 42th element captures are ignored
//      44th element last of preamble
const uint32_t preamble_LD[] =
{
     54000, 17400, 9000, 12160, 6220,   //5
     7820, 10080, 16780, 10080, 16460,  //10
     28000, 16520, 9520, 8360, 10000,   //15
     7860, 10080, 7820, 10080, 7420,    //20
     10080, 7820, 10100, 7800, 10080,   //25
     7840, 10560, 7340, 10080, 7840,    //30
     10060, 7840, 18040, 7920, 10080,   //35
     7820, 10080, 7820, 10500, 7000,    //40
     7820, 12000, 8000, 7200, 7350,     //45
     21200, 8140, 8140, 16480, 19320,   //50
     10640, 6900, 10400, 7100           //54

};
const uint8_t preamble_size = 54; // size of preamble
uint8_t local = 1; //Local button default cannot unlock/lock
uint8_t panic = 0;

// Beeper variables
uint8_t beepEnable = 0; // beeper enable option
int status = 0;

// IR variables
uint8_t w_index = 1;     //current location in buffer password
uint8_t password_length = 16; //
uint8_t setPass = 0;    //not currently setting password

uint8_t phase = 0; //phase counter
uint16_t code = 0; //controller button code value
uint32_t prev = 0; //previous load value
uint32_t curr = 0; //current load value
void initHw()
{
    // 40MHz Clock, PLL enabled
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    //Clocks for GPIO ports and Timer
    SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOA;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCEEPROM_R = 1;
    SYSCTL_RCGCHIB_R |= 1;

    // Set Ports to APB
    SYSCTL_GPIOHBCTL_R = 0;

    //enable board LEDS and button
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= PANIC_BUTTON_MASK;
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= PANIC_BUTTON_MASK | BUTTON_MASK | GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R |= PANIC_BUTTON_MASK | BUTTON_MASK; // enable internal pull-up for push button
    GPIO_PORTF_IS_R &= ~BUTTON_MASK;
    GPIO_PORTF_IBE_R &= ~BUTTON_MASK;
    GPIO_PORTF_IEV_R &= ~BUTTON_MASK;
    GPIO_PORTF_ICR_R |= BUTTON_MASK;
    GPIO_PORTF_IM_R |= BUTTON_MASK;

    NVIC_EN0_R |= (1 << 30);    // enable port F interrupts

    if(SENSOR)
    {
       RED_LED = 0;
       GREEN_LED = 1;
       status = 0;
    }
    else
    {
        RED_LED = 1;
        GREEN_LED = 0;
        status = 1;
    }

    //enable speaker output pin PC4
    GPIO_PORTC_DIR_R |= SPEAKER_MASK | RED_MASK | BLUE_MASK | WHITE_MASK;   // set speaker pin as output
    GPIO_PORTC_DEN_R |= SPEAKER_MASK | RED_MASK | BLUE_MASK | WHITE_MASK;   // enable digital
    GPIO_PORTC_DR2R_R |= SPEAKER_MASK;  // limit current to 2mA

    //enable GPI interrupt (Falling edge) and GPO pins
    GPIO_PORTB_DIR_R |= GPO_MASK | YELLOW_MASK;
    GPIO_PORTB_DIR_R &= ~SENSOR_MASK;
    GPIO_PORTB_DEN_R |= GPI_MASK | GPO_MASK | SENSOR_MASK | YELLOW_MASK;
    GPIO_PORTB_PUR_R |= SENSOR_MASK;
    GPIO_PORTB_IS_R &= ~GPI_MASK;
    GPIO_PORTB_IBE_R &= ~GPI_MASK;
    GPIO_PORTB_IEV_R &= ~GPI_MASK;
    GPIO_PORTB_ICR_R |= GPI_MASK;
    GPIO_PORTB_IM_R |= GPI_MASK;

    // Configure pins for Tx and Rx
    GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                     // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
    delay4Cycles();                                  // wait 4 clock cycles
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    //Configure periodic timer and leave off
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;    //Turn off timer
    TIMER1_CFG_R |= TIMER_CFG_32_BIT_TIMER;  //Timer with A&B to make 32 bit timer
    TIMER1_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT; //one shot timer
    TIMER1_TAILR_R = 40000000; // default value
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;   // turn on interrupts for timer1


    //Enable RTC
    HIB_CTL_R |= HIB_CTL_CLK32EN;   // Turn on RTC
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_IC_R |= 0x11;                                  // Clear interrupt
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_CTL_R |= HIB_CTL_RTCEN;
    while(!(HIB_CTL_R & HIB_CTL_WRC));              // wait until it is not busy
    HIB_RTCLD_R = 0;                                // Load seconds in
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_RTCM0_R = 4094967296;                       // Just a really high number that it shouldn't reach
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_RTCSS_R = 0;                                // set subseconds match
    NVIC_EN1_R |= (0x1 << 11);
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_IM_R |= HIB_IM_WC | HIB_IM_RTCALT0;    //turn on interrupts

    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);  //Wait until EEPROM is not powering up/working
    if(EEPROM_EESUPP_R & (EEPROM_EESUPP_PRETRY || EEPROM_EESUPP_ERETRY))
    {
        assert(0);
    }
    SYSCTL_SREEPROM_R = 1;  //reset EEPROM
    SYSCTL_SREEPROM_R = 0;  //clear reset

//
}

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}
int strcmp(char* str1, char* str2)
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
int strlen(char*str1)

{
    int length = 0;
    while(*str1 != '\0')
    {
        str1++;
        length++;
    }
    return length;
}
// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}
// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{

    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}
// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
      putcUart0(str[i]);
}
void getsUart0(char* str, uint8_t maxChars)
{
    char letter;
    uint8_t count=0;
    while(count < maxChars)
    {
        letter = getcUart0();
        // return if enter is pressed
        if(letter == 13)
        {
            break;
        }
        // Checks if letter is a backspace ascii number
        else if(letter == 8 || letter == 127)
        {
            if(count > 0)
            {
                count--;
                putsUart0("\b \b");
            }
        }
        //if character is printable, add to string
        else if(letter >= ' ')
        {
            //if character is uppercase letter, convert to lower case
            if(letter >= 'A' && letter <= 'Z')
            {
                letter += 32;
            }
            putcUart0(letter);
            str[count] = letter;
            count++;
        }
    }
    // if the count == maxChars, end with null terminator and return
        str[count] = '\0';
}
void print(char* str)
{
    putsUart0(str);
    putcUart0(10);
    putcUart0(13);
}
uint8_t parseStr(char* str, char** words, uint8_t maxChars, uint8_t maxWords)
{
    if(str == NULL || words == NULL)
    {
        print("ERROR: parseStr invalid argument");
        return 0;
    }
    int i;
    int w_index = 1;
    words[0] = &str[0]; // first word is the first argument
    for(i = 0; i < maxChars && str[i] != '\0'; i++)
    {
        if(w_index >= maxWords)
        {
            break;
        }
        if(str[i] == ' ')
        {
            str[i] = '\0';
            if(str[i+1] != 0 && str[i+1] != '\0' && str[i+1] != ' ')
            {
                words[w_index] = &str[i+1];
                w_index++;
            }
        }
    }
    return w_index;
}
void beep()
{
    if(beepEnable)
    {
          int milliseconds = 50;
          while(milliseconds > 0)
          {
              SPEAKER ^= 1;
              waitMicrosecond(1000); // wait 1ms
              milliseconds--;
          }
          SPEAKER = 0;
    }
}
int lock()
{
    //if it is already locked, then remain locked
    if(!SENSOR)
    {
        status = 1;
        return status;
    }
    //if no leads are powered, Red is default
    if(!RED && !WHITE && !YELLOW && !BLUE)
    {
        RED = 1;
    }
    int i;
    for(i = 0; i < 120; i++)
    {
        waitMicrosecond(6000); // step per microsecond
        if(RED)
        {
            RED = 0;
            YELLOW = 1;
        }
        else if(YELLOW)
        {
            YELLOW = 0;
            WHITE = 1;
        }
        else if(WHITE)
        {
            WHITE = 0;
            BLUE = 1;
        }
        else if(BLUE)
        {
            BLUE = 0;
            RED = 1;
        }
        else
            print("Motor error: No motor leads are powered.");
    }
    //check if motor is still unlocked after trying to lock
    if(SENSOR)
    {
        GREEN_LED = 0;
        RED_LED = 1;
        status = -1; // jammed
    }

    else
    {
        GREEN_LED = 0;
        RED_LED = 1;
        status = 1; // locked
    }
    return status;
}
int unlock()
{
    //check if it is unlocked
    if(SENSOR)
    {
        status = 0;
        return status;
    }
    int i;
    if(!RED && !WHITE && !YELLOW && !BLUE)
    {
        RED = 1;
    }
    for(i = 0; i < 120; i++)
    {
        waitMicrosecond(6000); // step per microsecond
        if(RED)
        {
            RED = 0;
            BLUE = 1;
        }
        else if(YELLOW)
        {
            YELLOW = 0;
            RED = 1;
        }
        else if(WHITE)
        {
            WHITE = 0;
            YELLOW = 1;
        }
        else if(BLUE)
        {
            BLUE = 0;
            WHITE = 1;
        }
        else
            print("Motor error: No motor leads are powered, Resetting to 'RED'");
    }
    //if motor is still locked
    if(!SENSOR)
    {
        GREEN_LED = 0;
        RED_LED = 1;
        status = -1; // jammed
    }
    else
    {
        GREEN_LED = 1;
        RED_LED = 0;
        status = 0; // unlocked
    }
    return status;
}
// temporarily enable beeper to create 1kHz signal for 1ms
void scream()
{
    panic = 1;
    int enabled_before = beepEnable;
    beepEnable = 1;
    while(PANIC_BUTTON)
    {
        beep();
    }
    beepEnable = enabled_before;
    panic = 0;
}
void edgeMode()
{
    GPIO_PORTB_IM_R |= GPI_MASK;        // turn on edge triggering
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;    //turn off timer1A
    NVIC_EN0_R &= ~(1 << 21);           // turn off interrupts for timer1A
    TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;  //turn off timeout interrupts in timer1A
    NVIC_EN0_R |= 1 << 1;               //turn on interrupts in port B

    GPO = 0;                            // GPO Output starts low
    phase = 0;                          // reset phase counter
    curr = 0;                           // reset current load value
    prev = 0;                           // reset previous load value

}
void timerMode()
{
    TIMER1_IMR_R |= TIMER_IMR_TATOIM; // turn on countdown interrupt
    NVIC_EN0_R |= (1 << 21); // turn on interrupt for timer1A
    NVIC_EN0_R &= ~(1 << 1); //turn off edge triggering in portB
    GPIO_PORTB_IM_R &= ~GPI_MASK; // turn off edge
    TIMER1_ICR_R |= 1;              //clear timeout interrupt in timer1
    TIMER1_CTL_R |= TIMER_CTL_TAEN; // turn on timer1A
}

void edgeISR()
{
    if(!panic)
    {
        curr = preamble_LD[0];
        TIMER1_TAILR_R = curr; // value to check first bit of data stream of 2.7ms
        timerMode();    //enable timer mode to toggle in between bits
    }
    GPIO_PORTB_ICR_R |= GPI_MASK; // clearing interrupt at GPI pin
}
// Beeper interrupt function to toggle output at 1kHz
void beepISR()
{
    SPEAKER ^= 1;        //toggle beeper output to create sound
    TIMER2_ICR_R |= 1;   //clear timeout interrupt in timer2
}

void timerISR()
{
    GPO ^= 1;   //Toggle the bit every time.
    phase++;
    //verify that the signal is from our special remote
    if(phase < preamble_size)
    {
        code = 0;
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;    //turn off timer1
        if(GPI == GPO && (phase < 41 || phase > 42)) // return if signal does not match ID signal components.
        {
            edgeMode();                     //reset to edge triggering mode
            TIMER1_ICR_R |= 1;              //clear flag to leave interrupt
            return;                         //exit interrupt
        }
        prev = curr;                        //set the previous half time
        curr = preamble_LD[phase];          //set current half time
        TIMER1_TAILR_R = prev + curr;       //add the previous and current half time
        TIMER1_CTL_R |= TIMER_CTL_TAEN;     //turn on timer1
    }
    //When signal has been verified and in the data bits section
    else if(phase < 70)
    {
        code += GPI;                        // add the current value of signal (H/L) to create code
        if(phase != 69)                     //dont shift if it is the last bit
            code = code << 1;               // shift to left since we are getting the data from MSB to LSB
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;    //turn off timer1
        TIMER1_TAILR_R = 17600;             //Set constant T for reading data bits
        TIMER1_CTL_R |= TIMER_CTL_TAEN;     //turn on timer1
    }
    //After data bits, we add
    else
    {
        uint16_t up = 43350;
        uint16_t down = 43349;
        uint16_t left = 42666;
        uint16_t right = 42665;
        uint16_t center = 42662;
        uint16_t play = 38486;
        uint16_t back = 42661;

        //check if code is a valid button code.
        if(code != up && code != down && code != left && code != right && code != center && code != play && code != back)
        {
            putsUart0("\n\rERROR: Invalid button code");
            TIMER1_ICR_R |= 1;
            return;
        }
        beep();

        //Delay for my siblings cause they can't press buttons correctly
        //waitMicrosecond(10000);

        //Save buttons that have matching code in current index and verified to be same sequence so far
        static int buffer[15];

        //Saves button pressed in current word location in a EEPROM block
        if(setPass)
        {
            //max password length is 16(0-15). All other inputs are ignored
            if(w_index < 16)
            {
                EEPROM_EEOFFSET_R = w_index;
                EEPROM_EERDWR_R = code;
                w_index++;
                password_length++;
            }
        }
        /*Process:
         * w_index start from word 1, and checks each of the 32 blocks available
         * for a matching code in the current word index. Then the matching block
         * needs to be checked if the previous input buttons in the buffer match
         * the block's previous code values. From there it compares the w_index or
         * current word index to the length of the matching block's password to see
         * if it is the smallest password with a matching sequence. Match_block provides
         * unique passwords to be verified more quickly by saving the previous block that
         * had a matching word.
         */
        else if(w_index < 16)
        {
            //Every time a button is pressed, we start searching for code match from block 0.
            static int match_block = 0;
            //possible variable determines whether there is a chance of a valid
            int possible = 0;
            //i is used to store block number since EEPROM was proving to be unreliable.
            int i = 0;

            EEPROM_EEBLOCK_R = 0;
            while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
            //Cycle through all 32 blocks including block 31(panic block)
            while(i <= 31)
            {
                //Move to current word index of block to check following input code against password code.
                EEPROM_EEOFFSET_R = w_index;
                while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
                if(code == EEPROM_EERDWR_R)
                {
                    if(match_block != EEPROM_EEBLOCK_R) //if code matches current column, but isn't the same block, check previous part of the sequence for matching values
                    {
                        //mismatch shows if a block does not match entirely with the current sequence (buffer)
                        int mismatch = 0;
                        int index=1;
                        while(index < w_index)  //check previous values
                        {
                            //temp used to hold word value to prevent spontaneous changes.
                            EEPROM_EEOFFSET_R = index;
                            int temp = EEPROM_EERDWR_R;
                            if(buffer[index-1] == temp)
                            {
                                mismatch = 0;
                            }
                            else
                            {
                                mismatch = 1;
                                break;
                            }
                            index++;
                        }
                        //if a block does not match entirely, do not verify as a possible match, and continue with search.
                        if(mismatch == 1)
                        {
                            goto MISMATCHED;
                        }
                    }
                    //After verifying the block matched the current sequence, save button in code sequence buffer.
                    buffer[w_index-1] = code;
                    //Set possibility of matching password to 1/true
                    possible = 1;
                    //Check the overall password length in current block to see if we have done the complete sequence
                    EEPROM_EEOFFSET_R = 0;
                    if(w_index >= EEPROM_EERDWR_R)
                    {
                        //if any blocks 0-30 are matched completely, lock/unlock
                        if(EEPROM_EEBLOCK_R < 31)
                        {
                            if(!SENSOR)
                            {
                                unlock();
                            }
                            else if(SENSOR)
                            {
                                lock();
                            }
                        }
                        //block 31 is reserved for panic code sequence
                        else
                        {
                            scream();
                        }
                        //Reset to first word of every block
                        //first value in buffer is 0 to reset because no button pressed will be zero.
                        w_index = 1;
                        possible = 0;
                        buffer[0] = 0;
                    }
                }
                MISMATCHED:

                //iterate through all 32 blocks
                i++;
                while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
                EEPROM_EEBLOCK_R = i;
                while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
            }
            //if there is a possibility of a matching password, increment to next word index
            if(possible)
            {
                w_index++;
            }
            //if no possibility is present, start at the beginning with new sequence.
            else
            {
                buffer[0] = 0;
                w_index = 1;
            }
        }
        //After analyzing the signal, return to edge mode to detect the next button press from IR remote
        edgeMode();
    }
    TIMER1_ICR_R |= 1;                      //clear timeout interrupt in timer1
}
char intToChar(int number)
{
    if(number >= 0 && number <= 9)
    {
        return number+48;
    }
    return 0;
}
int numberSize(int number)
{
    int count = 0;
    if(number == 0)
        count++;
    while(number)
    {
        number = number/10;
        count++;
    }
    return count;
}
void intToStr(char* str, int number)
{
   int digit = 0;
   int index = numberSize(number);
   str[index] = 0;
   while(index)
   {
       index--;
       digit = number%10;
       str[index] = intToChar(digit);
       number = number/10;
   }
}
int strToInt(char* number)
{
    int sum = 0;
    int index = strlen(number)-1;   //start from right to left(LSB)
    int placeholder = 1;    // start from one's place
    while(number[index] && index >= 0)
    {
        if(number[index] < '0' || number[index] > '9')  //if an invalid character is entered, return error
        {
            return -1;
        }
        sum += (number[index]-48)*placeholder;  // add to sum.
        placeholder = placeholder*10; // shift to left in significance.
        index--;
    }
    return sum;
}
//Convert strings of hours and minutes to seconds in int data type
int timeToSecs(char* hrs, char* mins)
{

    int sum = 0;
    int index = strlen(hrs)-1;  // get the LSB index
    if(index > 1)
    {
        return -1;
    }
    int placeholder=1;
    while(hrs[index] && index >= 0)
    {
        if(hrs[index] < '0' || hrs[index] > '9')// check if character is a valid number
        {
            return -1;
        }
        sum += (hrs[index]-48)*3600*placeholder; //convert from ascii value to int and add to sum of seconds
        index--;
        placeholder = placeholder*10;
    }
    index = strlen(mins)-1; // get the LSB index
    if(index > 1)
    {
        return -1;
    }
    placeholder = 1;
    while(mins[index] && index >= 0)
    {
        if(mins[index] < '0' || mins[index] > '9')    //Check if character is a valid number
        {
            return -1;
        }
        sum += (mins[index]-48)*placeholder*60; //convert from ascii value to seconds from minutes and add to sum
        index--;
        placeholder = placeholder*10;
    }
    if(sum > 86400)
    {
        return -1;
    }
    return sum;
}
void displayBlock(uint8_t P)
{
    //Print contents of block P
    EEPROM_EEBLOCK_R = P;
    putsUart0("\n\r");
    int i=0;
    while(i < 16)
    {
        EEPROM_EEOFFSET_R = i;
        int number = EEPROM_EERDWR_R;
        char num[3];
        intToStr(num,number);
        putsUart0(num);
        putcUart0(' ');
        i++;
    }
}
void clearBlock(uint8_t P)
{
    EEPROM_EEBLOCK_R = P;
    EEPROM_EEOFFSET_R = 0;
    int i=0;
    while(i < 16)
    {
        EEPROM_EEOFFSET_R = i;
        EEPROM_EERDWR_R = 0;
        i = i+1;
        while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    }
}
void setPassword(uint8_t P)
{

    clearBlock(P);          //Clear password block set up for password recording.
    password_length = 0;    //Contains the length of password of current block/Contains default max password length
    w_index = 1;            //EEPROM index for storing button codes

    putsUart0("\n\rSetting password ..");
    setPass = 1;                            //Enable password-setting
    waitMicrosecond(10000000);              //Set password values for 10 seconds
    setPass = 0;                            //Disable password-setting

    EEPROM_EEOFFSET_R = 1;                  //go to word 1, in block P to see if any button was pressed
    if(EEPROM_EERDWR_R == 0)                //if nothing is pressed, cancel and leave block empty.
    {
        putsUart0("Cancelled.");
        w_index = 1;                        //reset default value for w_index;
        password_length = 15;               //reset the maximum value as default password_length
        return;
    }
    EEPROM_EEOFFSET_R = 0;                  //get ready to store password length at word 0
    EEPROM_EERDWR_R = password_length;
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
    putsUart0("Password set.");             //else, set password length at word 0.

    w_index = 1;            //reset default value for w_index;
    password_length = 15;   //reset the maximum value as default password_length
}
void displayTime(int secs)
{
    //Printing Time
    int buffer = 0;

    int hrs = secs/3600;    // convert to 24 hour scale
    buffer = hrs*3600;      // get complete hours from total seconds
    secs -= buffer;         // remove hours from total seconds

    int mins = secs/60;     // convert remaining to minutes
    buffer = mins*60;       // get total complete minutes
    secs -= buffer;         // remove minutes from seconds
                            // remainder is seconds
    char hours[3];
    char minutes[3];
    char seconds[3];

    intToStr(seconds, secs);
    intToStr(minutes, mins);
    intToStr(hours, hrs);
    print("");
    putsUart0("Hours\t");
    print(hours);
    putsUart0("Minutes\t");
    print(minutes);
    putsUart0("Seconds\t");
    putsUart0(seconds);
}
void buttonPress()
{
    waitMicrosecond(100);
    //unlock/lock with physical button on board, unless it is "panicking" and local is enabled.
    if(local && !panic)
    {
        beep();
        if(!SENSOR)
        {
            unlock();
        }
        else if(SENSOR)
        {
            lock();
        }
    }
    GPIO_PORTF_ICR_R |= BUTTON_MASK;
}
//prints parsed words as a debugging tool
void printWords(char** words, uint8_t maxWords)
{
    int i=0;
    while(i < maxWords)
    {
        if(words[i] == NULL)
            return;
        char index[255];
        intToStr(index, i);
        putsUart0(index);
        putsUart0(" ");
        putsUart0(words[i]);
        i++;
    }
    putsUart0("\n\r");
}
void execute_cmd(char** words, uint8_t arg_count)
{
    beep();
    if(strcmp(words[0], "lock") == 0 && arg_count == 1)
    {
        lock();
    }
    else if(strcmp(words[0], "unlock") == 0 && arg_count == 1)
    {
        unlock();
    }
    else if(strcmp(words[0], "status") == 0 && arg_count == 1)
    {
        putsUart0("\n\r");
        if(status == 1)
        {
            putsUart0("Locked");
        }
        else if(status == 0)
        {
            putsUart0("Unlocked");
        }
        else if(status == -1)
        {
            putsUart0("Jammed");
        }
        else
        {
            putsUart0("ERROR: Unknown status");
        }
    }
    else if(strcmp(words[0],"password") == 0 && arg_count == 2)
    {
        int P = strToInt(words[1]);

        //check if input password index is not a number
        if(P == -1)
        {
            putsUart0("\n\r");
            putsUart0("ERROR: Argument must be valid number(0-30).");
        }
        //Password must be between the range 0-30
        else if(P < 0 && P > 30)
        {
            putsUart0("\n\r");
            putsUart0("ERROR: Argument is not within valid range(0-30).");
        }
        setPassword(P);
        //displayBlock(P);
    }
    else if(strcmp(words[0],"time") == 0 && arg_count == 3)
    {
        int time = timeToSecs(words[1],words[2]);
        if(time == -1)
        {
            putsUart0("\n\rError: Invalid time value");
            return;
        }
        while(!(HIB_CTL_R & HIB_CTL_WRC));
        HIB_RTCLD_R = time;
    }
    else if(strcmp(words[0],"autolock") == 0 && arg_count >= 2 && arg_count <= 3)
    {
        if(strcmp(words[1], "disable") == 0 && arg_count == 2)
        {
            while(!(HIB_CTL_R & HIB_CTL_WRC));
            HIB_IM_R &= ~HIB_IM_RTCALT0;            //turn off interrupts
        }
        else if(strcmp(words[1],"disable") == 0 && arg_count == 3)
        {
            putsUart0("\n\rError: Invalid argument");
        }
        else
        {
            //clear any previous interrupts
            while(!(HIB_CTL_R & HIB_CTL_WRC));
            HIB_IC_R |= 1;
            //turn on interrupts for matching times
            while(!(HIB_CTL_R & HIB_CTL_WRC));
            HIB_IM_R |= HIB_IM_WC | HIB_IM_RTCALT0;

            //convert hours and minutes to seconds to be put into RTCM0 register as the matching value
            int time = timeToSecs(words[1], words[2]);
            if(time == -1)
            {
                putsUart0("\n\rError: Invalid time value");
                return;
            }
            while(!(HIB_CTL_R & HIB_CTL_WRC));
            HIB_RTCM0_R = time;
        }

    }
    else if(strcmp(words[0],"gettime") == 0 && arg_count == 1)
    {
        int time = HIB_RTCC_R;
        displayTime(time);
    }
    else if(strcmp(words[0],"beeper") == 0 && arg_count == 2)
    {
        if(strcmp(words[1],"on") == 0)
        {
            beepEnable = 1;
        }
        else if(strcmp(words[1],"off") == 0)
        {
            beepEnable = 0;
        }
        else
        {
            goto ERROR;
        }
    }
    else if(strcmp(words[0],"local") == 0 && arg_count == 2)
    {
        if(strcmp(words[1],"on") == 0)
        {
            local = 1;
        }
        else if(strcmp(words[1],"off") == 0)
        {
            local = 0;
        }
        else
        {
            goto ERROR;
        }
    }
    else if(strcmp(words[0],"led") == 0 && arg_count == 2)
    {
        if(strcmp(words[1],"on") == 0)
        {
            if(SENSOR)
            {
               RED_LED = 0;
               GREEN_LED = 1;
            }
            else
            {
                RED_LED = 1;
                GREEN_LED = 0;
            }
        }
        else if(strcmp(words[1],"off") == 0)
        {
            RED_LED = 0;
            GREEN_LED = 0;
        }
        else
        {
            goto ERROR;
        }
    }
    else if(strcmp(words[0],"panic") == 0 && arg_count == 1)
    {
        setPassword(31);
    }
    else if (strcmp(words[0],"erase") == 0 && arg_count == 2)
    {
        int P = strToInt(words[1]);
        //check if input password index is not a number
        if(P == -1)
        {
            putsUart0("\n\r");
            putsUart0("ERROR: Argument must be valid number(0-30).");
        }
        //Password must be between the range 0-30
        else if(P < 0 && P > 31)
        {
            putsUart0("\n\r");
            putsUart0("ERROR: Argument is not within valid range(0-30).");
        }
        clearBlock(P);
        //displayBlock(P);
    }
    else if(strcmp(words[0],"clear") == 0 && arg_count == 1)
    {
        int block = 0;
        while(block < 32)
        {
            clearBlock(block);
            block++;
        }
        putsUart0("\n\rAll passwords cleared");
    }
    else
    {
        ERROR:
        putsUart0("\n\rINVALID COMMAND");
    }
}
void wc(void)
{
    //Checks if RTC was interrupted by match in RTCM0. AKA matching times
    if(HIB_RIS_R & HIB_RIS_RTCALT0)
    {
        if(SENSOR)
        {
            lock();
        }
    }
    else
    {
        if(HIB_RTCC_R >= 86400) //reset if it goes over 24 hours worth of seconds
        {
            while(!(HIB_CTL_R & HIB_CTL_WRC));
            HIB_RTCLD_R = 0;
        }
    }
    //clear both wrc and time match interrupt.
    while(!(HIB_CTL_R & HIB_CTL_WRC));
    HIB_IC_R |= 0x11;
}
/**
 * main.c
 */
int main(void)
{

    initHw();
    edgeMode();
    beepEnable = 1;
    beep();
    RED_LED = 0;
    GREEN_LED = 1;
    waitMicrosecond(1000000);
    RED_LED = 0;
    GREEN_LED = 0;
    waitMicrosecond(1000000);
    RED_LED = 0;
    GREEN_LED = 1;
    if(SENSOR)
    {
       RED_LED = 0;
       GREEN_LED = 1;
       status = 0;
    }
    else
    {
        RED_LED = 1;
        GREEN_LED = 0;
        status = 1;
    }


    //cmd line code
    const uint8_t max_chars = 100;
    char str[max_chars];
    char* words[4];
    uint8_t num_words=0;

    while(1)
    {
        //Command line output
        putsUart0("\r\n> ");
        str[0] = '\0';
        getsUart0(str, max_chars);      // get new cmd
        num_words = parseStr(str, words, max_chars, 4); //parse the words up to 4 words due to max number of arguments

        //printWords(words, num_words);
        execute_cmd(words, num_words);
    }
    return 0;
}


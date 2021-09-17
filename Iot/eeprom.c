// EEPROM functions
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target uC:       TM4C123GH6PM
// System Clock:    -

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "eeprom.h"

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initEeprom()
{
    SYSCTL_RCGCEEPROM_R = 1;
    _delay_cycles(3);
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

void writeEeprom(uint16_t add, uint32_t data)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    EEPROM_EERDWR_R = data;
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

uint32_t readEeprom(uint16_t add)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    return EEPROM_EERDWR_R;
}
uint8_t validEeprom(uint16_t add)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    if(0xFFFFFFFF == EEPROM_EERDWR_R)
        return 0;
    return 1;
}

/*
 *  word 0      = DHCP mode
 *  word 10-13  = IP address
 *  word 20-23  = Subnet Mask
 *  word 30-33  = Gateway Address
 *  word N/A    = DNS server
 */

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"

/*
 * Protected memory regions will be assigned as shown below
 * Region 0: BG                         0x0000.0000
 * Region 1: Flash                      0x0000.0000
 * Region 2: Peripheral                 0x4000.0000
 * Region 3: Peripheral Bit-Band        0x4200.0000
 * Region 4: SRAM block 0-7             0x2000.0000
 * Region 5: SRAM block 8-15            0x2000.2000
 * Region 6: SRAM block 16-23           0x2000.4000
 * Region 7: SRAM block 24-31           0x2000.6000
 */

void initMpu()
{
    // begin defining Default memory protection: p(RW) u(RW)
    NVIC_MPU_NUMBER_R = 0;                                                                              // Region 0: default
    NVIC_MPU_BASE_R = 0x00000000;
    NVIC_MPU_ATTR_R = (3 << 24) | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE |
            NVIC_MPU_ATTR_XN | (31 << 1) | 1;                                                           // set to full access, cacheable, shareable, 32KB, enabled

    // begin defining flash memory protection: p(XRW) u(XRW)
    NVIC_MPU_NUMBER_R = 1;                                                                              // Region 1: Flash
    NVIC_MPU_BASE_R = 0x00000000;
    NVIC_MPU_ATTR_R = (3 << 24) | NVIC_MPU_ATTR_CACHEABLE | (17 << 1) | 1;                              // Set to full access, cacheable, 256kB, enabled

    // begin defining Peripheral area
    NVIC_MPU_NUMBER_R = 2;
    NVIC_MPU_BASE_R = 0x40000000;                                                                       // Region 2: Peripheral address
    NVIC_MPU_ATTR_R = (3 << 24) | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_BUFFRABLE |                   // set to full access, never execute, shareable, bufferable, 1MB, enabled
            NVIC_MPU_ATTR_XN | (19 << 1) | 1;

    // begin defining Peripheral bit-band area
    NVIC_MPU_NUMBER_R = 3;
    NVIC_MPU_BASE_R = 0x42000000;                                                                       // Region 3: Peripheral bitband address
    NVIC_MPU_ATTR_R = (3 << 24) | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_BUFFRABLE |                   // Set to full access, never execute, shareable, bufferable, >1MB, enabled
            NVIC_MPU_ATTR_XN | (24 << 1) | 1;

    // Begin defining SRAM block 0-7
    NVIC_MPU_NUMBER_R = 4;
    NVIC_MPU_BASE_R = 0x20000000;                                                                       // Region 4: SRAM block 0-7
    NVIC_MPU_ATTR_R = (1 << 24) | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE |
            NVIC_MPU_ATTR_XN | (12 << 1) | 1;                                                           // Set to p(RW) & u(no access), shareable, cacheable, 8KB, enabled

    // Begin defining SRAM block 8-15
    NVIC_MPU_NUMBER_R = 5;
    NVIC_MPU_BASE_R = 0x20002000;                                                                       // Region 5: SRAM block 8-15
    NVIC_MPU_ATTR_R = (1 << 24) | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE |
            NVIC_MPU_ATTR_XN | (12 << 1) | 1;                                                           // Set to p(RW) & u(no access), shareable, cacheable, 8KB, enabled

    // Begin defining SRAM block 16-23
    NVIC_MPU_NUMBER_R = 6;
    NVIC_MPU_BASE_R = 0x20004000;                                                                       // Region 6: SRAM block 16-23
    NVIC_MPU_ATTR_R = (1 << 24) | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE |
            NVIC_MPU_ATTR_XN | (12 << 1) | 1;                                                           // Set to p(RW) & u(no access), shareable, cacheable, 8KB, enabled

    // Begin defining SRAM block 24-31
    NVIC_MPU_NUMBER_R = 7;
    NVIC_MPU_BASE_R = 0x20006000;                                                                       // Region 7: SRAM block 24-31
    NVIC_MPU_ATTR_R = (1 << 24) | NVIC_MPU_ATTR_SHAREABLE | NVIC_MPU_ATTR_CACHEABLE |
            NVIC_MPU_ATTR_XN | (12 << 1) | 1;                                                // Set to p(RW) & u(no access), shareable, cacheable, 8KB, enabled

    _delay_cycles(4);
    NVIC_MPU_CTRL_R = 1;                                                                                // turn on MPU.
}
void enableSysFaults()
{
    //enable memory, usage, bus fault ISRS
    NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEM | NVIC_SYS_HND_CTRL_BUS | NVIC_SYS_HND_CTRL_USAGE;
}
void clearMemPending()
{
    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_MEMP;
}
void clearMemFault()
{
    NVIC_FAULT_STAT_R |= 0xFF;
}
//returns the memory fault flags in the Configurable Fault Status Register
uint8_t getMemFaultFlags()
{
    return (uint8_t)NVIC_FAULT_STAT_R & 0xFF;
}
//returns address of the data/instruction trying to be accessed
uint32_t getMemAccessAddr()
{
    if(NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_MMARV)
    {
        return NVIC_MM_ADDR_R;
    }
    return 0;
}
// returns hard fault status flags
uint32_t getHFaultFlags()
{
    return NVIC_HFAULT_STAT_R;
}
// returns a pointer to the top of a region in the heap memory
void* getHeapRegionAddr(uint8_t region)
{
    if(region > 3 && region < 32)
        return (void*) (0x20000400 + (0x400 * region));
    else
        return 0;   // no address retrieval in RTOS regions 0-3
}
// privileged instruction to activate memory block permissions for user threads
void enableMemBlocks(uint32_t heapAllocated)
{

    //setting the new permission blocks for Heap section
    NVIC_MPU_NUMBER_R = 4;
    NVIC_MPU_ATTR_R &= ~NVIC_MPU_ATTR_SRD_M;
    NVIC_MPU_ATTR_R |= (heapAllocated & 0x000000FF) << 8;

    NVIC_MPU_NUMBER_R = 5;
    NVIC_MPU_ATTR_R &= ~NVIC_MPU_ATTR_SRD_M;
    NVIC_MPU_ATTR_R |= (heapAllocated & 0x0000FF00);

    NVIC_MPU_NUMBER_R = 6;
    NVIC_MPU_ATTR_R &= ~NVIC_MPU_ATTR_SRD_M;
    NVIC_MPU_ATTR_R |= (heapAllocated & 0x00FF0000) >> 8;

    NVIC_MPU_NUMBER_R = 7;
    NVIC_MPU_ATTR_R &= ~NVIC_MPU_ATTR_SRD_M;
    NVIC_MPU_ATTR_R |= (heapAllocated & 0xFF000000) >> 16;
}



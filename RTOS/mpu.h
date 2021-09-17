/*
 * mpu.h
 *
 *  Created on: Oct 5, 2020
 *      Author: aguin
 */

#ifndef MPU_H_
#define MPU_H_

void initMpu();
void enableSysFaults();
void clearMemPending();
void clearMemFault();
void* getHeapRegionAddr(uint8_t region);
void enableMemBlocks(uint32_t heapAllocated);
//void findRegion(void* addr);
uint8_t getMemFaultFlags();
uint32_t getMemAccessAddr();
uint32_t getHFaultFlags();

#endif /* MPU_H_ */

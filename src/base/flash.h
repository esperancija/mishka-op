/*
 * flash.h
 *
 *  Created on: Oct 27, 2011
 *      Author: user2
 */

#ifndef FLASH_H_
#define FLASH_H_

#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

#include "main.h"

#define PAGE_SIZE		2048
#define MAX_PAGE_NUMBER 127 //because of 2k page size (assume 256k chip)


void erasePage(void* addr);
void writeFlash(void* Src, void* Dst, int Len);

//write koefs structure into flash memory with time stamp
void setKoefs(volatile Koefs * koefs);
//read koefs structure from flash memory
uint8_t getKoefs(volatile Koefs * koefs);


#endif /* FLASH_H_ */

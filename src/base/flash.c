/*
 * flash.c
 *
 *  Created on: Oct 27, 2011
 *      Author: Alexander Khoryakov
 */

#include "main.h"
#include "flash.h"
#include "debug.h"

#include "process.h"


#include "debug-uart.h"


const Koefs* flashKoefs = 	(Koefs*)(FLASH_BASE+PAGE_SIZE*(MAX_PAGE_NUMBER-2)); //use two pages
const Koefs* flashKoefsCopy =	(Koefs*)(FLASH_BASE+PAGE_SIZE*(MAX_PAGE_NUMBER-1)); //use two pages

int16_t correctKoefsCopy=PAGE_SIZE/sizeof(Koefs)-1; //start from end

uint32_t getKoefsCRC(volatile Koefs* src){
uint8_t length = sizeof(Koefs)/4 - 1; //excluding crc
uint32_t* srcDW = (uint32_t*) src;

RCC->AHBENR |= RCC_AHBENR_CRCEN;

	//reset hardware module
		CRC->CR = 1;
	//delay for hardware ready
		asm("NOP");asm("NOP");asm("NOP");

//try prevent compatibility	bugs
   CRC->DR = 0x8383;

	while(length--)
	    CRC->DR=*(srcDW++);

		return CRC->DR;
}

void setKoefsDefault(volatile Koefs * koefs){
	koefs->steerRatio = 43;
	koefs->steerActuatorDelay = 210;
}

void writeFlash(void* Src, void* Dst, int Len)
{
  uint16_t* SrcW = (uint16_t*)Src;
  uint16_t* DstW = (uint16_t*)Dst;


  /* (1) Wait till no operation is on going */
  /* (2) Check that the Flash is unlocked */
  /* (3) Perform unlock sequence */
  while ((FLASH->SR &FLASH_SR_BSY) != 0);
  if ((FLASH->CR &FLASH_CR_LOCK) != 0){
	  FLASH->KEYR = FLASH_FKEY1;
	  FLASH->KEYR = FLASH_FKEY2;
  }

  FLASH->CR |= FLASH_CR_PG; /* Program the flash */


while (Len > 0){
//	if (SrcW%2)
//		DEBUG_MSG(BDL,("WARNING odd address!!!"))

    *DstW = *SrcW;
    while ((FLASH->SR & FLASH_SR_BSY) != 0 );

    if (*DstW != *SrcW )
      goto EndPrg;

    DstW++;
    SrcW++;
    Len-=2;
}


EndPrg:


if ((FLASH->SR &FLASH_SR_EOP) != 0){
	FLASH->SR =FLASH_SR_EOP;
}else{
	/* Manage the error cases */
}

  FLASH->CR &= ~FLASH_CR_PG; /* Reset the flag back !!!! */
//Lock the flash back
  FLASH->CR |= FLASH_CR_LOCK;
}

void erasePage(void* addr)
{
	  FLASH->KEYR = FLASH_KEY1;
	  FLASH->KEYR = FLASH_KEY2;
	//erase page
	  FLASH->CR |= FLASH_CR_PER;
	//write address
	  FLASH->AR = (uint32_t)addr;

	//start operation
	  FLASH->CR|= FLASH_CR_STRT;
	//wait
	  while ((FLASH->SR & FLASH_SR_BSY) != 0 );
	//reset bit
	  FLASH->CR &= ~FLASH_CR_PER;
	//Lock the flash back
	  FLASH->CR |= FLASH_CR_LOCK;
}



//save koefs in eeprom
void setKoefs(volatile Koefs * koefs){

koefs->crc = getKoefsCRC(koefs);

	if ((correctKoefsCopy >= (PAGE_SIZE/sizeof(Koefs)-1))){ //end space on page
		correctKoefsCopy=0;
		erasePage((void*)flashKoefs);
		writeFlash((void*)koefs, (void*)flashKoefs, sizeof(Koefs));
		erasePage((void*)flashKoefsCopy);
		writeFlash((void*)koefs, (void*)flashKoefsCopy, sizeof(Koefs));
	}else{ //write
		correctKoefsCopy++;
		writeFlash((void*)koefs, (void*)(flashKoefs+correctKoefsCopy), sizeof(Koefs));
		writeFlash((void*)koefs, (void*)(flashKoefsCopy+correctKoefsCopy), sizeof(Koefs));
	}
}

//get koefs from eeprom
uint8_t getKoefs(volatile Koefs * koefs){

uint16_t i;
uint16_t* SrcW = (uint16_t*)flashKoefs;
uint16_t* DstW = (uint16_t*)koefs;

//looking for right copy
		while (correctKoefsCopy > -1){
			SrcW = (uint16_t*)flashKoefs + correctKoefsCopy*sizeof(Koefs)/2;
			for(i=0;i<sizeof(Koefs)/2;i++)
				*(DstW+i) = *(SrcW+i);
			if (koefs->crc == getKoefsCRC(koefs)) //copy is OK
				return 1;
			else{ //try second copy
				SrcW = (uint16_t*)flashKoefsCopy + correctKoefsCopy*sizeof(Koefs)/2; //jump to next page
				for(i=0;i<sizeof(Koefs)/2;i++)
					*(DstW+i) = *(SrcW+i);
				if (koefs->crc == getKoefsCRC(koefs)) //second copy is OK
				{
					//write to first copy from second copy
					setKoefs(koefs);
					return 1;
				}
				else
					correctKoefsCopy--; //try next copy
			}
		};

		setKoefsDefault(&murchik.koefs);

		correctKoefsCopy = PAGE_SIZE/sizeof(Koefs);//erase pages & write new values
		setKoefs(koefs);
	return 0;
}


uint32_t getCRC(uint8_t *data, uint32_t size){
	uint32_t crc = 0, i;
	for (i = 0; i < size; i++){
		//DEBUG_MSG(BDL,("add crc %x at %d", *data, i))
		crc += *(data++);
	}
	return crc;
}





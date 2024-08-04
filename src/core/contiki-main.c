
#include <stdint.h>
#include "main.h"
#include "stm32f0xx.h"

#include <sys/process.h>
#include <sys/procinit.h>
#include <etimer.h>
#include <sys/autostart.h>
#include <clock.h>

//#include "bootloader_config.h"

unsigned int idle_count = 0;

//-Wl,-section-start=.boot=0x08000010; -Wl,-section-start=.text=0x08001800; -Wl,-section-start=.boot_vector=0x08000000;

//.section  .boot_vector,"ax",%progbits
//.type  bootVectors, %object
//bootVectors:
//	.word	_eram
//	.word	Reset_Handler
//	.word	NMI_Handler
//	.word	HardFault_Handler
//.size  bootVectors, .-bootVectors

//__attribute__ ((section (".boot_vector"), used))  const uint32_t vect[] = {
//		0x2000A000,
//		0x08000010,
//		0x08000010,
//		0x08000010
//};

int main()// Reset_Handler
{
	IWDG->KR = 0xAAAA; //reset
	IWDG->KR = 0x5555; //enable access
	IWDG->RLR = 0xfff; //set reload register to max
	IWDG->PR = 0x02;//0x07;// set divider
	IWDG->KR = 0xCCCC; //enable


  	clock_init();
	  process_init();
	  ctimer_init();
	  process_start(&etimer_process, NULL);
	  autostart_start(autostart_processes);

	  while(1) {
	    do {
	    } while(process_run() > 0);
	    idle_count++;

//		switch(murchik.showState){
//			case showNormalMenuState:
//				showNormalMenu();
//				break;
//			case showOPMenu1State:
//			case showOPMenu2State:
//				showOPMenu(murchik.showState);
//				break;
//		}

	//reset IWDG
	   IWDG->KR = 0xAAAA;
	    /* Idle! */
	    /* Stop processor clock */
	    /* asm("wfi"::); */
	  }
	  return 0;
}





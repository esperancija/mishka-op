/**
 * \file
 *         main mishka file
 * \author
 *         Alexander Khoryakov (starostarf@gmail.com)
 */


#include "contiki.h"
#include "main.h"

#include "debug-uart.h"
#include "steer.h"


/*---------------------------------------------------------------------------*/
PROCESS(menu_process, "Menu process");

extern Car murchik = {};


/* We require the processes to be started automatically */
AUTOSTART_PROCESSES(
		&menu_process,
		&steer_process
);

/* Implementation of the display process */
PROCESS_THREAD(menu_process, ev, data) {


static struct etimer timer;

	PROCESS_BEGIN();

	initDebug();
	DEBUG_MSG(CDL,(COL_GREEN"Start firmware from %s %s"COL_END, __DATE__, __TIME__));

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

		DEBUG_MSG(BDL,("Start menu process"));

		//init Ds control
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		GPIOA->MODER |= GPIO_MODER_MODER7_0;
		GPIOA->BSRR = GPIO_BSRR_BR_7;

		while (1) {
			etimer_set(&timer, CLOCK_SECOND/10);
			// and wait until the vent we receive is the one we're waiting for
			PROCESS_YIELD();

			if (murchik.isEcoMode == 0){
				GPIOA->BSRR = GPIO_BSRR_BS_7;
				RED_ON;
			}else{
				RED_OFF;
				GPIOA->BSRR = GPIO_BSRR_BR_7;
			}
		}
		PROCESS_END();
}




#ifndef __STEEP__
#define __STEER__
#include "main.h"

PROCESS_NAME(steer_process);

#define TARGET_LEVEL		3115 //in adc to get 2.5V MUST be > 2048
#define MAX_MOMENT			(8*(4095-TARGET_LEVEL)/10)

#define FS_RELAY_SETUP 	do {GPIOA->MODER &= ~GPIO_MODER_MODER8;\
									GPIOA->MODER |= GPIO_MODER_MODER8_0;} while (0)
#define FS_RELAY_ON			(GPIOA->BSRR = GPIO_BSRR_BS_8)
#define FS_RELAY_OFF		(GPIOA->BSRR = GPIO_BSRR_BR_8)

#define GREEN_ON 			(GPIOB->BSRR |= GPIO_BSRR_BS_12)
#define GREEN_OFF 			(GPIOB->BSRR |= GPIO_BSRR_BR_12)
#define RED_ON 				(GPIOB->BSRR |= GPIO_BSRR_BS_13)
#define RED_OFF 			(GPIOB->BSRR |= GPIO_BSRR_BR_13)

#define IS_BUT_PRESS 		(GPIOB->IDR & GPIO_IDR_14)

#define OP_ACTIVE_TIMEOUT	20 //in 1/50 sec

process_event_t calc_data_event;

enum PidReset {normalPid, resetPid};

#endif

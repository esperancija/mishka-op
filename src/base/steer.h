

#ifndef __STEEP__
#define __STEER__
#include "main.h"

//PROCESS_NAME(steer_process);

#define DMA_CH_NUM	3
#define TENZO1_CH	1
#define TENZO2_CH	2
#define SBI_CH		0


#define TARGET_LEVEL		3115 //in adc to get 2.5V MUST be > 2048
#define MAX_MOMENT			(10*(4095-TARGET_LEVEL)/10)//(9*(4095-TARGET_LEVEL)/10)//(8*(4095-TARGET_LEVEL)/10) MODER12

#define FS_RELAY_SETUP 	do {GPIOA->MODER &= ~GPIO_MODER_MODER8;\
									GPIOA->MODER |= GPIO_MODER_MODER8_0;} while (0)

#define FS_RELAY_ON		do {GPIOA->MODER &= ~GPIO_MODER_MODER8;\
								GPIOA->MODER |= GPIO_MODER_MODER8_0;\
								GPIOA->BSRR = GPIO_BSRR_BS_8;} while (0)

#define FS_RELAY_OFF		(GPIOA->MODER &= ~GPIO_MODER_MODER8)//(GPIOA->BSRR = GPIO_BSRR_BR_8; )
#define FS_RELAY_STATE		(GPIOA->IDR & GPIO_IDR_8)

#define GREEN_ON 			(GPIOB->BSRR |= GPIO_BSRR_BS_13)
#define GREEN_OFF 			(GPIOB->BSRR |= GPIO_BSRR_BR_13)
#define RED_ON 				(GPIOB->BSRR |= GPIO_BSRR_BS_12)
#define RED_OFF 			(GPIOB->BSRR |= GPIO_BSRR_BR_12)
#define RED_TOGLE			(GPIOB->ODR ^= GPIO_ODR_12)

#define BUT_SETUP			(GPIOB->PUPDR |= GPIO_PUPDR_PUPDR14_1)
#define IS_BUT_PRESS 		((GPIOB->IDR & GPIO_IDR_14) == 0)

#define OP_ACTIVE_TIMEOUT	20 //in 1/50 sec

#define MOMENT_DIFF_LIMIT	1000//155//105//150 //150 //limit moment change per step

process_event_t calc_data_event;

enum PidReset {normalPid, resetPid};

void initLeds (void);
void doSteerControl(void);
void startSteerControl(void);

#endif

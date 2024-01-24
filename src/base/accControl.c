
#include "accControl.h"
#include "stm32f0xx.h"
#include "contiki-conf.h"
#include "main.h"

#include "can.h"

uint8_t getAccKey(void){
	if (murchik.accControlAdc > (BTN_NOKEY_LVL+BTN_UP_LVL)/2)
		return noKey;
	else if ((murchik.accControlAdc <= (BTN_NOKEY_LVL+BTN_UP_LVL)/2) &&
			(murchik.accControlAdc > (BTN_UP_LVL+BTN_MISHKA_LVL)/2))
		return upKey;
	else if ((murchik.accControlAdc <= (BTN_UP_LVL+BTN_MISHKA_LVL)/2) &&
			(murchik.accControlAdc > (BTN_MISHKA_LVL+BTN_DOWN_LVL)/2))
		return mishkaKey;
	else if ((murchik.accControlAdc <= (BTN_MISHKA_LVL+BTN_DOWN_LVL)/2) &&
			(murchik.accControlAdc > (BTN_DOWN_LVL+BTN_CANCEL_LVL)/2))
		return downKey;
	else if ((murchik.accControlAdc <= (BTN_DOWN_LVL+BTN_CANCEL_LVL)/2) &&
			(murchik.accControlAdc > (BTN_CANCEL_LVL+BTN_ACC_LVL)/2))
		return cancelKey;
	else if (murchik.accControlAdc > BTN_ACC_LVL/2)
		return accOnKey;
	else
		return lkasOnKey;
}

//BTN_MISHKA_LVL

void accCtlLevelTimerCb(void * p){
	GPIOB->MODER &= ~(GPIO_MODER_MODER0);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0);
    GPIOB->BSRR = GPIO_BSRR_BR_0;
}

void setAccCtlLevel(uint16_t level){
	TIM3->CCR3 = level - 1;
	GPIOB->MODER &= ~(GPIO_MODER_MODER0);
	GPIOB->MODER |= (GPIO_MODER_MODER0_1); //TIM3 CH3 alternate output
	//ctimer_set(&accCtlLevelTimer, CLOCK_SECOND/20, accCtlLevelTimerCb, NULL);
}

//void DMA1_Channel1_IRQHandler(void) {
//static uint8_t oldState, newState, delay, cnt, butCnt;
//	//clear pending flag of interrupt
//	DMA1->IFCR |= DMA_IFCR_CGIF1;
//
////openpilot control
//	newState = (murchik.accControlAdc/ACC_ADC_DIV < 5)?1:0;
//	if ((newState != oldState) && (newState == 0) && (delay == 0) && (murchik.gear != 0)){
//		murchik.isOPActive ^= 1;
//		delay = 10;
//	}
//
//	if (delay){
//		//set acc to current speed
//		if (murchik.isOPActive){
//			if (delay == 9)
//				setAccCtlLevel(ACCS_ON_LVL);
//			else if (delay == 4)
//				setAccCtlLevel(ACCS_DEC_LVL);
//			else
//				accCtlLevelTimerCb(NULL);
//		}
//		delay --;
//	}
//	oldState = newState;
//
//	//send openPilot data
//	if (murchik.isOPActive){
//		sendPacket(STEER_TEST_ID, 127 | 0x300, (murchik.koefs.steerRatio*5) | ((murchik.koefs.steerActuatorDelay) << 8));
//	}else {
//		sendPacket(STEER_TEST_ID, 127 , (murchik.koefs.steerRatio*5) | ((murchik.koefs.steerActuatorDelay) << 8));
//	}
//
//
////control part
////	if (murchik.steerButton){
////		if ((butCnt < 20)){
////			butCnt ++;
////		}else if (butCnt < 0xff){
////			if (murchik.showState == showNormalMenuState){
////				murchik.showState = showOPMenu1State;
////			}else {
////				murchik.showState = showNormalMenuState;
////			}
////			butCnt = 0xff;
////		}
////	}else{
////		if (butCnt < 20){
////			if (murchik.showState == showOPMenu1State)
////				murchik.showState = showOPMenu2State;
////			else if (murchik.showState == showOPMenu2State)
////				murchik.showState = showOPMenu1State;
////		}else
////			butCnt = 0xff;
////	}
//
//	if (((murchik.showState == showOPMenu1State) || (murchik.showState == showOPMenu2State)) && getAccKey())
//		murchik.key = getAccKey();
//	else
//		murchik.key = getKey();
//
//	if (murchik.key == noKey){
//		switch(murchik.oldKey){
//			case upKey:
//				if (murchik.showState == showOPMenu1State)
//					murchik.koefs.steerRatio ++;
//				else
//					murchik.koefs.steerActuatorDelay ++;
//
//				//setAccCtlLevel(7000);
//				//murchik.accSetSpeed = SET_UP_SPEED;
//				break;
//			case centerKey:
//				if (murchik.showState == showOPMenu1State)
//					murchik.koefs.steerRatio --;
//				else
//					murchik.koefs.steerActuatorDelay --;
//				//murchik.accSetSpeed = SET_DOWN_SPEED;
//				break;
//			case downKey:
//				if (murchik.showState == showNormalMenuState)
//					murchik.showState = showOPMenu1State;
//				else if (murchik.showState == showOPMenu1State)
//					murchik.showState = showOPMenu2State;
//				else if (murchik.showState == showOPMenu2State){
//					murchik.showState = showNormalMenuState;
//					setNeedSetKoefs();
//				}
//				break;
//		}
//	}
//	murchik.oldKey = murchik.key;
//
//
//
////acc speed control
//	if ((murchik.accSetSpeed > 0)){
////		if ((murchik.accSpeed + 5) < murchik.accSetSpeed){
////			setAccCtlLevel(ACCS_INC_LVL);
////			//cnt = 0;
////		}else if ((murchik.accSpeed - 5) > murchik.accSetSpeed){
////			setAccCtlLevel(ACCS_DEC_LVL);
////			//cnt = 0;
////		}else
//		if (((cnt % 2) == 0) && (murchik.accSpeed != murchik.accSetSpeed)){
//			if (murchik.accSpeed < murchik.accSetSpeed)
//				setAccCtlLevel(ACCS_INC_LVL);
//			else
//				setAccCtlLevel(ACCS_DEC_LVL);
//		}else{
//			accCtlLevelTimerCb(NULL);
//		}
//		cnt++;
//
//		if ((murchik.accSpeed == murchik.accSetSpeed) || (cnt > 200)){
//			cnt = 0;
//			murchik.accSetSpeed = 0;
//			accCtlLevelTimerCb(NULL);
//		}
//	}
//}


void initAccControl(void){

//set up PWM
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

//	//TIM3 CH3 alternate output
	GPIOB->MODER &= ~(GPIO_MODER_MODER0);
    GPIOB->MODER |= (GPIO_MODER_MODER0_0);
    GPIOB->BSRR = GPIO_BSRR_BS_0;

    GPIOB->AFR[0] |= 0x0001;

	/* (1) Set prescaler to 3, so APBCLK/4 i.e 12MHz */
	/* (2) Set ARR = 12000 -1 */
	/* (3) Set CCRx = ARR, as timer clock is 12MHz, an event occurs each 1 ms */
	/* (4) Select  mode on OC1 (OC1M = 001),
	disable preload register on OC1 (OC1PE = 0, reset value) */
	/* (5) Select active high polarity on OC1 (CC1P = 0, reset value),
	enable the output on OC1 (CC1E = 1)*/
	/* (6) Enable output (MOE = 1)*/
	/* (7) Enable counter */
	TIM3->PSC = 0;//((MCK)/48-1);
	TIM3->ARR = 10000;//3000;

	TIM3->CCR3 = 1 - 1; /* (3) */
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1;// | TIM_CCMR2_OC3M_0; /* (4) */
	TIM3->CCER |= TIM_CCER_CC3E; /* (5)*/
	//TIM2->BDTR |= TIM_BDTR_MOE; /* (6) */
	TIM3->CR1 |= TIM_CR1_CEN; /* (7) */
}

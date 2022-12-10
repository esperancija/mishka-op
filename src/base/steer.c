
#include "steer.h"
#include "debug-uart.h"

#include "can.h"
#include <stdlib.h>     /* abs */

PROCESS(steer_process, "Steer process");

uint8_t debugMsg[500];
process_event_t calc_data_event;
int16_t momentAdd;

uint16_t mDiffLimit = MOMENT_DIFF_LIMIT;

static int32_t sum, P, I, D, res;

#define MAX_K_KF		32768

#define KALMAN_KOEF 10000
#define KALMAN(z, x) ((KALMAN_KOEF*z+(MAX_K_KF-KALMAN_KOEF)*x)/MAX_K_KF)


#define KALMAN_S_KOEF 	27000 //20000//26000 //27000
#define KALMAN_S(z, x) ((KALMAN_S_KOEF*z+(MAX_K_KF-KALMAN_S_KOEF)*x)/MAX_K_KF)

int16_t limitMoment(int16_t moment, int16_t value){
	if (moment >= value)
		moment = value;
	else if (moment <= -value)
		moment = -value;
	return moment;
}

/*------------------------------------------------------------DMA interupt handler-*/
void DMA1_Channel1_IRQHandler(void) {

uint16_t value1, value2;

	//clear pending flag of interrupt
	DMA1->IFCR |= DMA_IFCR_CGIF1;

	murchik.steerSensor1 = KALMAN(murchik.steerSensor1, murchik.instantData.rawData[0]);
	murchik.steerSensor2 = KALMAN(murchik.steerSensor2, murchik.instantData.rawData[1]);

	murchik.steerWheelMoment = murchik.steerSensor1 - murchik.steerSensor2;

/*******************************************************************************************/
	if ((murchik.currentState == controlState)){
		momentAdd = murchik.steerTargetMoment;
	}else if (murchik.currentState == offState)
		momentAdd = 0;

	limitMoment(momentAdd, MAX_MOMENT);

	value1 = murchik.steerSensor1+momentAdd;
	value2 = murchik.steerSensor2-momentAdd;

	//check overflow
	if ((value1 < 4095) && (value2 < 4095)){
		DAC->DHR12R1 = value1;
		DAC->DHR12R2 = value2;
	}else{
		DAC->DHR12R1 = murchik.steerSensor1;
		DAC->DHR12R2 = murchik.steerSensor2;
	}
}

void initLeds (void){

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	GPIOB->MODER |= (GPIO_MODER_MODER12_0 |
						GPIO_MODER_MODER13_0 );

	GPIOB->MODER &= ~GPIO_MODER_MODER14;
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR14_0;
}

void initADC(void){

	//Enabling clock for ADC & DMA
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	RCC->AHBENR |= RCC_AHBENR_DMA2EN;
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	//calibrate ADC
	 if ((ADC1->CR & ADC_CR_ADEN) == 0) {
		  ADC1->CR |= ADC_CR_ADCAL;
		  while ((ADC1->CR & ADC_CR_ADCAL));
		  while ((ADC1->CR & ADC_CR_ADEN) == 0) {
			  	  ADC1->CR |= ADC_CR_ADEN;
		  }
		  while (((ADC1->ISR & ADC_ISR_ADRDY) == 0));
	 }

	 //set max channel sample time
	   ADC1->SMPR = ADC_SMPR1_SMPR_1 | ADC_SMPR1_SMPR_2;

#define DMA_CH_NUM	2
   //select channels
	   ADC1->CHSELR =
			   ADC_CHSELR_CHSEL6 + ADC_CHSELR_CHSEL3; //6,3

	//set pin on porta 3,6 as analog input
	   GPIOA->MODER &= ~(GPIO_MODER_MODER3 + GPIO_MODER_MODER6);

	//use DMA continuous conversions
	 ADC1->CFGR1 = ADC_CFGR1_DMAEN + ADC_CFGR1_DMACFG + ADC_CFGR1_OVRMOD +
			 ADC_CFGR1_EXTEN_0 + // rising edge
			 ADC_CFGR1_EXTSEL_2; //TIM15_TRGO (4)

	//set DMA
	    DMA1_Channel1->CNDTR = DMA_CH_NUM;
		DMA1_Channel1->CPAR = ADC1_DR_Address;
		DMA1_Channel1->CMAR = (uint32_t)murchik.instantData.rawData;
	   //set 16 bit block data size memory increment circular & interrupt
	   DMA1_Channel1->CCR = DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC |
			   	    DMA_CCR_TCIE | DMA_CCR_EN | DMA_CCR_CIRC;
		 DMA1_Channel1->CCR |= DMA_CCR_PL;//highest priority

	   		RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	 	   //use hardware timer 15 to trigger ADC 35 times per second
	   		TIM15->PSC = ((MCK)/1000000-1);
	   		//max timer value
			TIM15->ARR =  10-1;

	   		//allow timer working & reset on overflowing
	   		TIM15->CR1 = TIM_CR1_CEN | TIM_CR1_ARPE;
	   		//turn on TRGO signal for ADC -----------------------------------------
	   		TIM15->CR2 = TIM_CR2_MMS_1; //update event

	   //first start
	   ADC1->CR |= ADC_CR_ADSTART;     //start conversions


	   NVIC_SetPriority(DMA1_Channel1_IRQn, 5);
	   NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}



void initSteerControl(void){

//***********************DAC
	/* (1) Enable the peripheral clock of the DAC */
	/* (2) Enable DMA transfer on DAC ch1 and ch2,
	       enable interrupt on DMA underrun DAC ch1 and ch2,
	       enable the DAC ch1 and ch2,
	       select TIM6 as trigger by keeping 000 in TSEL1
	       select TIM7 as trigger by writing 010 in TSEL2 */
	RCC->APB1ENR |= RCC_APB1ENR_DACEN; /* (1) */
	DAC->CR |=
//				DAC_CR_TSEL2_1 | DAC_CR_DMAUDRIE2 | DAC_CR_DMAEN2
//		         | DAC_CR_TEN2 |
	         DAC_CR_EN2 |
//		         DAC_CR_DMAUDRIE1 | DAC_CR_DMAEN1 | DAC_CR_BOFF1
//		         | DAC_CR_TEN1 |
	         DAC_CR_EN1; /* (2) */
	DAC->DHR12R1 = 0;//DAC_OUT1_VALUE; /* Initialize the DAC value on ch1 */
	DAC->DHR12R2 = 0;//DAC_OUT2_VALUE; /* Initialize the DAC value on ch2 */

//*************************ADC
	initADC();

	FS_RELAY_SETUP;
	FS_RELAY_OFF;
}

#define CORRECT_POINT_NUM	5
#define POINT_DATA_DIVIDER	10

//multiply moment according wheel position
int16_t correctMoment(int16_t m, int16_t angle, uint16_t* x, uint16_t* y){
	uint8_t i;

	angle = (angle>=0)?angle:-angle;
	if (angle <= x[0])
		return m*y[0]/POINT_DATA_DIVIDER;
	else if (angle >= x[CORRECT_POINT_NUM-1])
		return m*y[CORRECT_POINT_NUM-1]/POINT_DATA_DIVIDER;
	else{ //make interpolation
		i=1;
		while ((angle > x[i]) && (i < CORRECT_POINT_NUM-1))
			i++;
		if (i >= CORRECT_POINT_NUM-1)
			return m;

		return m*(y[i]+(x[i] - angle)*(y[i-1] - y[i])/(x[i]-x[i-1]))/POINT_DATA_DIVIDER;
	}
	return m;
}



int16_t filter(int16_t sample){

}


#define PID_P 	800//700 //800//600//500//800//900//750//500//
uint16_t pidPAngle[] = {0,   7, 15, 30, 70}; //in degree
uint16_t pidPData[] =  {70, 40, 30,  25, 20};
//uint16_t pidPData[] =  {60, 40, 30,  30, 30};  //in 1/10
//uint16_t pidPData[] =  {40, 25, 10,  5, 3};  //in 1/10
//uint16_t pidPData[] =  {60, 25, 20,  5, 3};  //in 1/10

#define PID_I	35//75//55 //60 //55//42//60//75//65//50//45//50//15//10//50
uint16_t pidIAngle[] = {0,   7, 15, 30, 70}; //in degree
//uint16_t pidIData[] =  {7, 7, 5,  5, 5};  //in 1/10
uint16_t pidIData[] =  {20, 7, 1,  1, 1};  //in 1/10

//uint16_t pidIData[] =  {10, 10, 7,  5, 3};  //in 1/10
//uint16_t pidIData[] =  {15, 12, 10,  10, 10};  //in 1/10

#define PID_D	9000//11000//15000 //9000//10000//12000//9000//10000//1500//10//2000//500//
//uint16_t pidDAngle[] = {0,  45, 60, 70, 90}; //in degree
//uint16_t pidDData[] =  {15, 10,   6,  6,  6};  //in 1/10

uint16_t pidDAngle[] = {0,   7, 15, 30, 70}; //in degree
uint16_t pidDData[] =  {15, 15, 13,  12, 10};  //in 1/10


#define PID_NF			1//15//10 //30//80//20//10//5//10//14 //negative feedback to limit steering speed
//uint16_t pidNFAngle[] = {0,  20, 45, 60, 90}; //in degree
//uint16_t pidNFData[] =  {15, 9,   3,  3,  3};  //in 1/10
uint16_t pidNFAngle[] = {0, 5, 10, 20, 90}; //in degree
uint16_t pidNFData[] =  {50, 200,  160,  10, 10};  //in 1/10


#define PID_I_DROP_ADD	400//300 //to prevent increase I component moment at driver action
#define LIMIT_NFB		200

#define DIFF_AVRG	3

int16_t makePID(int16_t diff, uint8_t isReset){

static int16_t prevData, prevMoment, dArr[8], diffArr[DIFF_AVRG];
static uint8_t dIndex, diffIndex;
uint16_t pidP, i;


if (isReset == resetPid){
	sum = 0;
	prevData = 0;
	return 0;
}

//make some averaging
diffArr[(diffIndex++)%DIFF_AVRG] = diff;
diff = 0;
for (i=0;i<DIFF_AVRG;i++)
	diff += diffArr[i];
diff /= DIFF_AVRG;

if (abs(murchik.steerWheelMoment) < PID_I_DROP_ADD)
	sum += diff;

//limit sum
if (sum >= 100*MAX_MOMENT/PID_I)
	sum = 100*MAX_MOMENT/PID_I;
if (sum <= (-100*MAX_MOMENT/PID_I))
	sum = (-100*MAX_MOMENT/PID_I);

//P = PID_P*diff/100;
//I = PID_I*sum/100;
//D = PID_D*(diff-prevData)/100;



//P = (PID_P)*diff/100;
pidP = correctMoment(PID_P, murchik.steerPosition/2, pidPAngle, pidPData);
P = (pidP)*diff/100;


I = PID_I*sum/100;
//I = correctMoment(PID_I, murchik.steerPosition/2, pidIAngle, pidIData)*sum/100;

//D = PID_D*(diff-prevData)/100;
D = correctMoment(PID_D, murchik.steerPosition/2, pidDAngle, pidDData)*(diff-prevData)/100;

dArr[(dIndex++) % 8] = D;
D=0;
for (i=0;i<8;i++)
	D+=dArr[i];

res = P+I+D/8;


//res += res*murchik.speed/30000;

//res = iir(res);

//limit moment change
if (abs(res - prevMoment) > mDiffLimit){
	if ((res - prevMoment) > 0)
		res = prevMoment + mDiffLimit;
	else
		res = prevMoment - mDiffLimit;
}

if (res >= MAX_MOMENT)
	res = MAX_MOMENT;
if (res <= (-MAX_MOMENT))
	res = (-MAX_MOMENT);

prevData = diff;
prevMoment = res;

return res;
}

static uint32_t steerActionTimer = 0;

//moment to make shake action at working LDW & RCTA
#define MOMENT_ADD				(350 + murchik.speed/50)

PROCESS_THREAD(steer_process, ev, data) {

static struct etimer timer;
static int16_t newMoment, prevMoment, prevAngle, newDiffAngle;
//static uint16_t kalmanKoefs = KALMAN_S_KOEF;
static uint8_t direction, delayCnt;

static uint16_t momentDelayCnt;

	PROCESS_BEGIN();

	DEBUG_MSG(BDL,("Start steer process"));

	calc_data_event = process_alloc_event();

	initCanBus();
	initSteerControl();
	initLeds();

	murchik.currentState = activeState;

	etimer_set(&timer, CLOCK_SECOND/50);
	while (1) {
		//etimer_set(&timer, CLOCK_SECOND/50); //timeout to turn off control
		//etimer_set(&timer, CLOCK_SECOND/3); //timeout to turn off control
		PROCESS_YIELD();

		//DEBUG_MSG(2,("ev = %d, state %d", ev, murchik.currentState))

		if ((IS_BUT_PRESS) || (murchik.opData & opActive)){
			murchik.currentState = controlState;
			GREEN_ON;
			FS_RELAY_ON;
		}else{
			murchik.currentState = activeState;
			GREEN_OFF;
		}

		if (ev == PROCESS_EVENT_TIMER){
			etimer_set(&timer, CLOCK_SECOND/50);

			if (murchik.currentState == activeState){
				if (murchik.ldwState == LDW_LEFT_ACTIVE){
					FS_RELAY_ON;
					if (steerActionTimer % 2)
						momentAdd = MOMENT_ADD;
					else
						momentAdd = MOMENT_ADD/2;
				}else if (murchik.ldwState == LDW_RIGHT_ACTIVE){
					FS_RELAY_ON;
					if (steerActionTimer % 2)
						momentAdd = -MOMENT_ADD;
					else
						momentAdd = -MOMENT_ADD/2;
				}else if (murchik.rctaState){
					FS_RELAY_ON;
					if ((steerActionTimer) % 2){
						momentAdd = MOMENT_ADD/4;
					}else{
						momentAdd = -MOMENT_ADD/4;
					}
				}else{
					FS_RELAY_OFF;
					momentAdd = 0;
				}
			}
			steerActionTimer++;

			if (murchik.opActiveTimer > 0)
				murchik.opActiveTimer--;
			else
				murchik.opData &= ~opActive;

		}

#define KALMAN_AD_KOEF 20000
#define KALMAN_AD2_KOEF 32700

#if (CONTROL_MODE == ANGLE_CONTROL)
		else if (ev == calc_data_event){ //got new steer position value ~100Hz
			if (etimer_expired(&timer))
				etimer_set(&timer, CLOCK_SECOND/50);

			if (abs(murchik.steerPosition - murchik.steerTargetAngle) > 50){
				momentDelayCnt = 100;
			}else{
				if (momentDelayCnt > 0)
					momentDelayCnt--;
			}

			if ((murchik.currentState == controlState) &&
					((murchik.speed > 700) || (IS_BUT_PRESS)) //){
						&& (abs(murchik.steerWheelMoment) < 2000) ){
						//&& (momentDelayCnt == 0)){
						//&& (abs(murchik.steerPosition - murchik.steerTargetAngle) < 50)){

				newMoment = makePID((murchik.steerPosition - murchik.steerTargetAngle), normalPid);

//				if (murchik.steerPosition/2 < 20){
//					newDiffAngle = ((KALMAN_AD_KOEF*(murchik.steerPosition - murchik.steerTargetAngle)+
//							(MAX_K_KF-KALMAN_AD_KOEF)*newDiffAngle)/MAX_K_KF);
//				}else {
//					newDiffAngle = ((KALMAN_AD2_KOEF*(murchik.steerPosition - murchik.steerTargetAngle)+
//							(MAX_K_KF-KALMAN_AD2_KOEF)*newDiffAngle)/MAX_K_KF);
//				}
//				newMoment = makePID(newDiffAngle, normalPid);

				int16_t tempMom = correctMoment(PID_NF, murchik.steerPosition/2, pidNFAngle, pidNFData)*
						(murchik.steerSensor1 - murchik.steerSensor2)/100;

				newMoment += (tempMom > LIMIT_NFB)?LIMIT_NFB:tempMom;

			//to make moment at wheel more smoothly
				murchik.steerTargetMoment = ((KALMAN_S_KOEF*murchik.steerTargetMoment+
													(MAX_K_KF-KALMAN_S_KOEF)*newMoment)/MAX_K_KF);

				if ((mDiffLimit < MOMENT_DIFF_LIMIT) && ((delayCnt++) % 2))
					mDiffLimit++;

				direction <<= 1;
				direction |= (murchik.steerTargetMoment > prevMoment)?1:0;

				dbg_send_bytes(debugMsg,
						snprintf((char*)debugMsg, sizeof(debugMsg),
							"P %d I %d D %d sum %d res %d flt %d str %d angle %d %d dir %d \r\n",
							P, I, D, sum, res, murchik.steerTargetMoment,(murchik.steerSensor1 - murchik.steerSensor2),
							murchik.steerPosition, murchik.steerTargetAngle, direction
						));

//				//if find change direction of moment make steer tighter
//				if (((direction == 0xf0) || (direction == 0x0f)) && (murchik.steerPosition/2 > 25)){
//					if (mDiffLimit > 50)
//						mDiffLimit -= 50;
//				}

//				if (mDiffLimit > abs(murchik.steerPosition - prevAngle)){
//					mDiffLimit -= abs(murchik.steerPosition - prevAngle);
//				}

				//limit value
				if (mDiffLimit > MOMENT_DIFF_LIMIT)
					mDiffLimit = MOMENT_DIFF_LIMIT;

				prevMoment = murchik.steerTargetMoment;

			}else{
				makePID(0, resetPid);//reset internal variables
				murchik.steerTargetMoment = 0;
			}
			prevAngle = murchik.steerPosition;
		}
#endif
	}
	PROCESS_END();
}


#include "steer.h"
#include "debug-uart.h"

#include "accControl.h"

#include "can.h"
#include <stdlib.h>     /* abs */

//PROCESS(steer_process, "Steer process");

uint8_t debugMsg[500];
process_event_t calc_data_event;
int16_t momentAdd;

uint16_t mDiffLimit = MOMENT_DIFF_LIMIT;

static int32_t sum, P, I, D, res;

#define MAX_K_KF		32768

#define KALMAN_KOEF 1000
#define KALMAN(z, x) ((KALMAN_KOEF*z+(MAX_K_KF-KALMAN_KOEF)*x)/MAX_K_KF)


#define KALMAN_S_KOEF 	21000//20000//17000//11000//10000//27000 //20000//26000 //27000
#define KALMAN_S(z, x) ((KALMAN_S_KOEF*z+(MAX_K_KF-KALMAN_S_KOEF)*x)/MAX_K_KF)

#define KALMAN_SBI_KOEF 	32000//20000//17000//11000//10000//27000 //20000//26000 //27000
#define KALMAN_SBI(z, x) ((KALMAN_SBI_KOEF*z+(MAX_K_KF-KALMAN_SBI_KOEF)*x)/MAX_K_KF)

int16_t limitMoment(int16_t moment, int16_t value){
	if (moment >= value)
		moment = value;
	else if (moment <= -value)
		moment = -value;
	return moment;
}


void TIM16_IRQHandler(void){
	if (TIM16->SR & TIM_SR_UIF){
		if (murchik.flags & runMomentCalcFlag){
			//process_post_synch(&steer_process, calc_data_event, 0);
			doSteerControl();
			murchik.flags &= ~runMomentCalcFlag;
		}
	    TIM16->SR &= ~TIM_SR_UIF;
	}
}

/*------------------------------------------------------------DMA interupt handler-*/
void DMA1_Channel1_IRQHandler(void) {  //2.5 uS

uint16_t value1, value2;
static int16_t oldVal1, oldVal2;

//DBG_ON;

	//clear pending flag of interrupt
	DMA1->IFCR |= DMA_IFCR_CGIF1;

	murchik.steerSensor1 = KALMAN(murchik.steerSensor1, murchik.instantData.rawData[TENZO1_CH]);
	murchik.steerSensor2 = KALMAN(murchik.steerSensor2, murchik.instantData.rawData[TENZO2_CH]);
	murchik.steerWheelMoment = murchik.steerSensor1 - murchik.steerSensor2;

	murchik.accControlAdc = KALMAN_SBI(murchik.accControlAdc, murchik.instantData.rawData[SBI_CH]);

/*******************************************************************************************/
	if ((murchik.currentState == controlState)){
		momentAdd = murchik.steerTargetMoment;
	}else if (murchik.currentState == offState)
		momentAdd = 0;

	limitMoment(momentAdd, MAX_MOMENT);

	value1 = murchik.steerSensor1+momentAdd;
	value2 = murchik.steerSensor2-momentAdd;

#define MAX_ALOW_MOMENT	4093	//must be lower than 4096
	//check overflow
	if ((murchik.steerSensor1 < MAX_ALOW_MOMENT) && (murchik.steerSensor2 < MAX_ALOW_MOMENT) &&
			(value1 < MAX_ALOW_MOMENT) && (value2 < MAX_ALOW_MOMENT)){
		DAC->DHR12R1 = value1;
		DAC->DHR12R2 = value2;

		oldVal1 = value1;
		oldVal2 = value2;
	}else{
		DAC->DHR12R1 = oldVal1;//murchik.steerSensor1;
		DAC->DHR12R2 = oldVal2;//murchik.steerSensor2;

		RED_ON;
	}

//DBG_OFF;

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


   //select channels
	   ADC1->CHSELR =
			   ADC_CHSELR_CHSEL6 + ADC_CHSELR_CHSEL3 + ADC_CHSELR_CHSEL0; //6,3,0

	//set pin on porta 3,6 as analog input
	   GPIOA->MODER &= ~(GPIO_MODER_MODER3 + GPIO_MODER_MODER6 + GPIO_MODER_MODER0);

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
//int16_t correctMoment(int16_t m, int16_t angle, uint16_t* x, uint16_t* y){
//	uint8_t i;
//	int16_t res, absAngle;
//
//	absAngle = (angle>=0)?angle:-angle;
//
//	if (absAngle <= x[0])
//		res =  m*y[0]/POINT_DATA_DIVIDER;
//	else if (absAngle >= x[CORRECT_POINT_NUM-1])
//		res = m*y[CORRECT_POINT_NUM-1]/POINT_DATA_DIVIDER;
//	else{ //make interpolation
//		i=1;
//		while ((absAngle > x[i]) && (i < CORRECT_POINT_NUM-1))
//			i++;
//		if (i > CORRECT_POINT_NUM-1)
//			return m;
//
//		res =  m*(y[i]+(x[i] - absAngle)*(y[i-1] - y[i])/(x[i]-x[i-1]))/POINT_DATA_DIVIDER;
//	}
//	if (angle>=0)
//		return res;
//	else
//		return -res;
//}

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
		if (i > CORRECT_POINT_NUM-1)
			return m;

		return m*(y[i]+(x[i] - angle)*(y[i-1] - y[i])/(x[i]-x[i-1]))/POINT_DATA_DIVIDER;
	}
	return m;
}


#define PID_P 	600//700 //800//600//500//800//900//750//500//
uint16_t pidPAngle[] = {0,   7, 15, 30, 70}; //in degree
//uint16_t pidPData[] =  {90, 50, 50,  50, 50};

//uint16_t pidPData[] =  {40, 40, 30,  25, 20};
uint16_t pidPData[] =  {40, 60, 45,  37, 30};

//uint16_t pidPData[] =  {60, 40, 30,  30, 30};  //in 1/10
//uint16_t pidPData[] =  {40, 25, 10,  5, 3};  //in 1/10
//uint16_t pidPData[] =  {60, 25, 20,  5, 3};  //in 1/10

#define PID_I	125//125//200//250//70 //60 //55//42//60//75//65//50//45//50//15//10//50
//uint16_t pidIAngle[] = {0,   7, 15, 30, 70}; //in degree
uint16_t pidIAngle[] = {0,   3, 7, 30, 70}; //in degree
uint16_t pidIData[] =  {10, 10, 10,  5, 5};  //in 1/10
//uint16_t pidIData[] =  {20, 7, 1,  1, 1};  //in 1/10

//uint16_t pidIData[] =  {10, 10, 7,  5, 3};  //in 1/10
//uint16_t pidIData[] =  {15, 12, 10,  10, 10};  //in 1/10

#define PID_D	13000//15000 //9000//10000//12000//9000//10000//1500//10//2000//500//
//uint16_t pidDAngle[] = {0,  45, 60, 70, 90}; //in degree
//uint16_t pidDData[] =  {15, 10,   6,  6,  6};  //in 1/10

uint16_t pidDAngle[] = {0,   7, 15, 30, 70}; //in degree
uint16_t pidDData[] =  {15, 15, 13,  12, 10};  //in 1/10


#define PID_NF			10//20//15//80//15//10 //30//80//20//10//5//10//14 //negative feedback to limit steering speed
//uint16_t pidNFAngle[] = {0,  20, 45, 60, 90}; //in degree
//uint16_t pidNFData[] =  {15, 9,   3,  3,  3};  //in 1/10
uint16_t pidNFAngle[] = {0, 5, 10, 20, 90}; //in degree
uint16_t pidNFData[] =  {70, 10,  0,  0, 0};  //in 1/10


//28.12.22 NF[0] 20->30
//KALMAN_S 11000->17000

#define PID_I_DROP_ADD	2000//400//300 //to prevent increase I component moment at driver action
#define LIMIT_NFB		800

#define DIFF_AVRG	3

int16_t makePID(int16_t diff, uint8_t isReset){

static int32_t prevData, prevMoment, dArr[16], diffArr[DIFF_AVRG];
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


//I = PID_I*sum/100;
I = correctMoment(PID_I, murchik.steerPosition/2, pidIAngle, pidIData)*sum/100;
//I = correctMoment(PID_I, murchik.steerPosition/2, pidIAngle, pidIData)*sum/100;

//D = PID_D*(diff-prevData)/100;
D = correctMoment(PID_D, murchik.steerPosition/2, pidDAngle, pidDData)*(diff-prevData)/100;

#define D_SMOOTH	2
dArr[(dIndex++) % D_SMOOTH] = D;
D=0;
for (i=0;i<D_SMOOTH;i++)
	D+=dArr[i];



res = P+I+D/D_SMOOTH;//+I+D/8;


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
prevMoment = res;//filterDiff;

//res-=N/5;

return res;
}

static uint32_t steerActionTimer = 0;

//moment to make shake action at working LDW & RCTA
#define MOMENT_ADD				(350 + murchik.speed/50)

#if (1)
void startSteerControl(void){

	DEBUG_MSG(BDL,("Start steer process"));

	//calc_data_event = process_alloc_event();

	DEBUG_MSG(BDL,(COL_ORANGE"Init CAN bus ..."COL_END));
	initCanBus();
	DEBUG_MSG(BDL,(COL_GREEN"OK"COL_END));

	DEBUG_MSG(BDL,(COL_ORANGE"Init control ..."COL_END));
	initSteerControl();
	initAccControl();
	DEBUG_MSG(BDL,(COL_GREEN"OK"COL_END));

	murchik.currentState = activeState;

	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
   //use hardware timer 15 to trigger ADC 35 times per second
	TIM16->PSC = ((MCK)/100000-1);
	//max timer value
	TIM16->ARR =  10-1;

	//allow timer working & reset on overflowing
	TIM16->CR1 = TIM_CR1_CEN;// | TIM_CR1_UDIS;
	TIM16->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM16_IRQn, 11);
	NVIC_EnableIRQ(TIM16_IRQn);
}

void doSteerControl(void){

static int16_t newMoment, prevSetAngle;
static uint32_t cnt;

		//if (ev == PROCESS_EVENT_TIMER){
		//	etimer_set(&timer, CLOCK_SECOND/50);

		if ((cnt++)%2){ //to make 50 Hz
			if (murchik.currentState == activeState){
#ifdef STEER_SHAKE
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
#else
				FS_RELAY_OFF;
				momentAdd = 0;
#endif
			}
			steerActionTimer++;

			if (murchik.opActiveTimer > 0)
				murchik.opActiveTimer--;
			else
				murchik.opData &= ~opActive;
		}

#ifdef SBI_TEST
		if ((murchik.opData & opActive)){
#else
		if ((IS_BUT_PRESS) || (murchik.opData & opActive)){
#endif
			murchik.currentState = controlState;
			FS_RELAY_ON;
		}else{
			murchik.currentState = activeState;
		}

//#if (CONTROL_MODE == ANGLE_CONTROL)
//		else if (ev == calc_data_event){ //got new steer position value ~100Hz


			//if (etimer_expired(&timer))
			//	etimer_set(&timer, CLOCK_SECOND/50);

			if ((murchik.currentState == controlState) &&
					((murchik.speed > 700) || (IS_BUT_PRESS)) ){

DBG_ON;
				newMoment = makePID((murchik.steerPosition - ((murchik.steerTargetAngle+prevSetAngle)/2)), normalPid);
DBG_OFF;
				if (PID_NF){
					int16_t tempMom = correctMoment(PID_NF, murchik.steerPosition/2, pidNFAngle, pidNFData)*
							(murchik.steerSensor1 - murchik.steerSensor2)/100;
					newMoment += (tempMom > LIMIT_NFB)?LIMIT_NFB:tempMom;
				}

			//to make moment at wheel more smoothly
				murchik.steerTargetMoment = ((KALMAN_S_KOEF*murchik.steerTargetMoment+
													(MAX_K_KF-KALMAN_S_KOEF)*newMoment)/MAX_K_KF);

				prevSetAngle = murchik.steerTargetAngle;

				//limit value
				if (mDiffLimit > MOMENT_DIFF_LIMIT)
					mDiffLimit = MOMENT_DIFF_LIMIT;

			}else{
				makePID(0, resetPid);//reset internal variables
				murchik.steerTargetMoment = 0;
			}
//DBG_OFF;
}

#else
PROCESS_THREAD(steer_process, ev, data) {

static struct etimer timer;
static int16_t newMoment, prevSetAngle;

	PROCESS_BEGIN();

	DEBUG_MSG(BDL,("Start steer process"));

	calc_data_event = process_alloc_event();

	DEBUG_MSG(BDL,(COL_ORANGE"Init CAN bus ..."COL_END));
	initCanBus();
	DEBUG_MSG(BDL,(COL_GREEN"OK"COL_END));

	DEBUG_MSG(BDL,(COL_ORANGE"Init control ..."COL_END));
	initSteerControl();
	initAccControl();
	DEBUG_MSG(BDL,(COL_GREEN"OK"COL_END));

	murchik.currentState = activeState;

	etimer_set(&timer, CLOCK_SECOND/50);
	while (1) {
		//etimer_set(&timer, CLOCK_SECOND/50); //timeout to turn off control
		//etimer_set(&timer, CLOCK_SECOND/3); //timeout to turn off control
		PROCESS_YIELD();

		//DEBUG_MSG(2,("ev = %d, state %d", ev, murchik.currentState))

#ifdef SBI_TEST
		if ((murchik.opData & opActive)){
#else
		if ((IS_BUT_PRESS) || (murchik.opData & opActive)){
#endif
			murchik.currentState = controlState;
			FS_RELAY_ON;
		}else{
			murchik.currentState = activeState;
		}

		if (ev == PROCESS_EVENT_TIMER){
			etimer_set(&timer, CLOCK_SECOND/50);

			if (murchik.currentState == activeState){

#ifdef STEER_SHAKE
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
#else
				FS_RELAY_OFF;
				momentAdd = 0;
#endif
			}
			steerActionTimer++;

			if (murchik.opActiveTimer > 0)
				murchik.opActiveTimer--;
			else
				murchik.opData &= ~opActive;

		}

#if (CONTROL_MODE == ANGLE_CONTROL)
		else if (ev == calc_data_event){ //got new steer position value ~100Hz

DBG_ON;
			if (etimer_expired(&timer))
				etimer_set(&timer, CLOCK_SECOND/50);

			if ((murchik.currentState == controlState) &&
					((murchik.speed > 700) || (IS_BUT_PRESS)) ){

				newMoment = makePID((murchik.steerPosition - ((murchik.steerTargetAngle+prevSetAngle)/2)), normalPid);

				if (PID_NF){
					int16_t tempMom = correctMoment(PID_NF, murchik.steerPosition/2, pidNFAngle, pidNFData)*
							(murchik.steerSensor1 - murchik.steerSensor2)/100;
					newMoment += (tempMom > LIMIT_NFB)?LIMIT_NFB:tempMom;
				}

			//to make moment at wheel more smoothly
				murchik.steerTargetMoment = ((KALMAN_S_KOEF*murchik.steerTargetMoment+
													(MAX_K_KF-KALMAN_S_KOEF)*newMoment)/MAX_K_KF);

				prevSetAngle = murchik.steerTargetAngle;

				//limit value
				if (mDiffLimit > MOMENT_DIFF_LIMIT)
					mDiffLimit = MOMENT_DIFF_LIMIT;

			}else{
				makePID(0, resetPid);//reset internal variables
				murchik.steerTargetMoment = 0;
			}
DBG_OFF;
		}
#endif
	}
	PROCESS_END();
}
#endif

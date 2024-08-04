
#ifndef _main_h_
#define	_main_h_

#include "stm32f0xx.h"
#include "contiki.h"

//#pragma GCC diagnostic ignored "-Wswitch"
//#pragma GCC diagnostic ignored "-Wcomment"
//#pragma GCC diagnostic ignored "-Wstrict-aliasing"

#define MOMENT_CONTROL	0
#define ANGLE_CONTROL	1

#define CONTROL_MODE ANGLE_CONTROL

#define BDL	2
#define CDL	10
#define DEBUG_LEVEL	2

#define ADC1_DR_Address   ((uint32_t)0x40012440)

typedef struct
{
	uint8_t type;
	uint8_t* addr;
	uint8_t height;
	uint8_t width;
	uint8_t shift;
}Font;

enum enumFontType 	{standartFont, normalFont, additionalFont, changeFont, bigFont, veryBigFont};

enum ShowState {showNormalMenuState, showOPMenu1State, showOPMenu2State};


typedef struct{
	int8_t 	temp; //in C
	uint8_t pres; // in 1/10 atm
}WheelData;

typedef struct{
	uint8_t state;
	uint8_t timeout;
	WheelData FL;
	WheelData RL;
	WheelData RR;
	WheelData FR;
}TPMSData;

/**
 * structure to save in flash koefs
 */
typedef struct
{
	uint8_t steerRatio;
	uint8_t steerActuatorDelay;
	uint16_t reserved;
	//all params MUST be aligned according to settings.c
	uint32_t 	crc;
}Koefs;

#define LDW_LEFT_LEN	5
typedef struct{
	int16_t left[LDW_LEFT_LEN];	//in 0.05m
	int16_t right;  //in 0.05m
	uint16_t width; //in 0.01 m
	uint8_t isLeft;
	uint8_t leftIndex;
	uint8_t isRight;
}LDWData;

/**
 * working variables
 */
typedef struct
{
	uint16_t	rawData[6];	///array from ADC
}InstantData;

///main structure
typedef struct
{
///state of device
	uint8_t 		currentState;
///working instantData
	InstantData 	instantData;

	LDWData	ldwData;
	uint8_t ldwState;

	uint16_t speed;//from CAN
	uint16_t speedFL;//from CAN
	uint16_t speedRL;//from CAN
	uint16_t speedFR;//from CAN
	uint16_t speedRR;//from CAN

	uint16_t 	breaking;
	uint16_t 	isBrake;
	uint8_t		steerButton;

	int16_t steerSensor1;
	int16_t steerSensor2;

	uint16_t accControlAdc; //steer button input
	uint16_t sbo; //steer button output

	int16_t steerPosition;		//from CAN
	uint16_t steerSpeed;		//from CAN
	int16_t steerMoment;		//from CAN
	int16_t	steerTargetAngle;	//calculate needed angle
	uint16_t steerTargetTime;	//in read ldw data period 1/10s
	int16_t steerTargetMoment; //in percent settings value
	int16_t steerWheelMoment;	//in percent
	uint8_t opData;

	int8_t 		ATTemp;
	int8_t 		engTemp;
	uint8_t 	gear;
	uint16_t 	fuel;
	uint32_t 	LPer100;
	int16_t 	accel;
	int16_t 	accAccel;
	int16_t 	accAccel2;
	uint8_t		isOPActive;

	uint8_t		accActive;
	uint8_t 	isGLock;
	uint8_t 	isEcoMode;
	uint8_t 	bswState;
	uint8_t 	rctaState;
	uint8_t		lightState;//02 - low beam, 06 - high beam

	uint8_t		accDistance;
	uint8_t		accSpeed;
	uint8_t 	accSetSpeed;
	int8_t		accTest1; //steerActuatorDelay
	uint8_t		accTest2;
	uint32_t		ahbTest1;
	uint32_t		ahbTest2;
	uint8_t		accTest5;

	TPMSData	tpmsData;
	int16_t 	testData;
	uint16_t opActiveTimer;

	uint8_t		key;
	uint8_t		oldKey;
	uint8_t 	showState;
	Koefs koefs;

}Car;

enum Key			{noKey = 0, lkasOnKey, cancelKey, accOnKey, upKey, downKey, mishkaKey};

enum State {
	offState, 		//no control
	activeState, 	//steer shake by ldw warning
	controlState, 	//steer control by external data
				lastState
};

enum OPState {
	opActive = 1, opLeftLine = 2, opRightLine = 4
};

extern Car murchik;

#define DS_INIT	GPIOA->MODER |= GPIO_MODER_MODER7_0; GPIOA->BSRR = GPIO_BSRR_BR_7;
#define DS_ON 	GPIOA->BSRR = GPIO_BSRR_BS_7
#define DS_OFF  GPIOA->BSRR = GPIO_BSRR_BR_7

//#define SBI_TEST

#define STEER_SHAKE
//#define USE_LCD

Car murchik;

#endif



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

	uint16_t rpm;	//from CAN
	uint16_t speed; //in 1/100 from CAN

	int16_t steerSensor1;
	int16_t steerSensor2;

	int16_t steerPosition;		//from CAN
	uint16_t steerSpeed;		//from CAN
	int16_t steerMoment;		//from CAN
	int16_t	steerTargetAngle;	//calculate needed angle
	uint16_t steerTargetTime;	//in read ldw data period 1/10s
	int16_t steerTargetMoment; //in percent settings value
	int16_t steerWheelMoment;	//in percent
	uint8_t opData;
	int8_t ATTemp;
	uint8_t gear;
	uint16_t fuel;
	uint32_t LPer100;
	int16_t accel;
	uint8_t isGLock;
	uint8_t isEcoMode;
	uint8_t bswState;
	uint8_t rctaState;

	int16_t testData;

	uint16_t opActiveTimer;

}Car;

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

#endif


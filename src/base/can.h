

#ifndef __CAN__
#define __CAN__

#include "main.h"

#define STEERING_WHEEL_MOMENT_ID	0x2f1 //C+D
#define STEERING_WHEEL_POS_ID		0x236	//A+B
#define LDW_STATE_ID				0x222
#define ECO_BUT_ID					0x390
#define BSW_STATE_ID				0x30e

#define STEER_CONTROL_ID			0x3b6
#define STEER_TEST_ID				0x3b8

#define SPEED_1_ID					0x214

#define LDW_LEFT_ACTIVE				0x15
#define LDW_RIGHT_ACTIVE			0x11

void initCanBus(void);

#endif

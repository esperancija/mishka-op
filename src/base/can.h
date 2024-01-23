

#ifndef __CAN__
#define __CAN__

#include "main.h"

#define LDW_REQ_ID 					0x718//0x71a
#define LDW_ANSWER_ID 				0x719//0x71b

#define TPMS_REQ_ID 				0x600
#define TPMS_ANSWER_ID 				0x500

//can.c(0407) 71,67s: id=0x600 2-21-23-0-0-0-0-0
#define TPMS_FL_REQ					0x00232102//0x23
#define TPMS_RL_REQ					0x00242102//0x24
#define TPMS_RR_REQ					0x00252102//0x25
#define TPMS_FR_REQ					0x00262102//0x26

#define TPMS_FL_PID					0x2123
#define TPMS_RL_PID					0x2124
#define TPMS_RR_PID					0x2125
#define TPMS_FR_PID					0x2126

#define ENGINE_RPM_ID				0x308
#define SPEED_1_ID					0x200
#define SPEED_2_ID					0x208
#define SPEED_ID					0x214
#define ATTEMP_ID					0x418
#define GEAR_ID						0x218
#define FUEL_FLOW_ID				0x608
#define STEERING_WHEEL_MOMENT_ID	0x2f1 //C+D
#define STEERING_WHEEL_POS_ID		0x236	//A+B
#define LDW_STATE_ID				0x222
#define ECO_BUT_ID					0x390
#define BSW_STATE_ID				0x30e
#define ACCEL_PEDAL_ID				0x210
#define BRAKE_STATE_ID				0x415
#define BUTTON_STATE_ID				0x425
#define LIGHT_STATE_ID				0x424

#define LONG_CONTROL_ID				0x22a
#define ACC_STATE_ID				0x22b

#define STEER_CONTROL_ID			0x3b6
#define STEER_TEST_ID				0x3b8

#define TEST_AHB1_ID				0x22e
#define TEST_AHB2_ID				0x366

#define LDW_LEFT_ACTIVE				0x15
#define LDW_RIGHT_ACTIVE			0x11

typedef union{
	uint64_t 	f;
	uint32_t 	d[2];
	uint8_t		t[8];
}CanData;

typedef struct{
	uint8_t len;
	uint16_t id;
	CanData data;
}CanPacket;

void initCanBus(void);
uint8_t sendPacket(uint16_t pid, uint32_t data0, uint32_t data1);

#endif

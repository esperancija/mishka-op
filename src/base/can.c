
#include "stdint.h"
#include "can.h"
#include "stm32f0xx.h"
#include "main.h"
#include "debug-uart.h"
#include "steer.h"

uint16_t lastId = 0;

void sendConfirm(uint16_t pid);

uint16_t packetNumber = 0;
#define MAX_PACKET 500
volatile CanPacket packets[MAX_PACKET];

void CEC_IRQHandler(void){
	//uint8_t debugMsg[500];
uint16_t cId = CAN->sFIFOMailBox[0].RIR >> 21;

if (cId == STEERING_WHEEL_POS_ID){
	murchik.steerPosition = (((CAN->sFIFOMailBox[0].RDLR & 0xff) << 8) +
									((CAN->sFIFOMailBox[0].RDLR >> 8) & 0xff)) - 0x1000;//0x0ffd;//0x1000;
	murchik.steerSpeed = ((CAN->sFIFOMailBox[0].RDLR >> 16) & 0xffff)-0x1000;
	process_post(&steer_process, calc_data_event, 0);
//}else if (cId == STEER_TEST_ID){
//	murchik.testData = ((CAN->sFIFOMailBox[0].RDLR) & 0xff) - 127;

//	dbg_send_bytes(debugMsg,
//					snprintf((char*)debugMsg, sizeof(debugMsg), "id=0x%x test=%d, from 0x%x \r\n",
//							CAN->sFIFOMailBox[0].RIR >> 21,
//							murchik.testData,
//							CAN->sFIFOMailBox[0].RDLR
//							));
}else if (cId == LDW_STATE_ID){
	murchik.ldwState = (((((CAN->sFIFOMailBox[0].RDHR) >> 0) & 0xff)));
}else if (cId == ECO_BUT_ID){
	murchik.isEcoMode = (((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff));
}else if (cId == BSW_STATE_ID){
	murchik.bswState = ((CAN->sFIFOMailBox[0].RDLR >> 0) & 0xff);
	murchik.rctaState = ((CAN->sFIFOMailBox[0].RDHR >> 24) & 0xff);
}else if (cId == SPEED_ID){
	murchik.speed = 10*(((CAN->sFIFOMailBox[0].RDLR & 0xff) << 8) + ((CAN->sFIFOMailBox[0].RDLR >> 8) & 0xff))/12;
}else if (cId == SPEED_1_ID){
	//C+D
	murchik.speedFL = ((CAN->sFIFOMailBox[0].RDLR >> 8) & 0xff00) + ((CAN->sFIFOMailBox[0].RDLR >> 24) & 0xff);
	//E+F
	murchik.speedFR = ((CAN->sFIFOMailBox[0].RDHR & 0xff) << 8) + ((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff);
	//G+H
	murchik.speedRL = ((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff00) + ((CAN->sFIFOMailBox[0].RDHR >> 24) & 0xff);
}else if (cId == SPEED_2_ID){
	//E+F
	murchik.speedRR = ((CAN->sFIFOMailBox[0].RDHR & 0xff) << 8) + ((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff);
	//G+H
	//murchik.speed = ((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff00) + ((CAN->sFIFOMailBox[0].RDHR >> 24) & 0xff);
	//murchik.speed = 10*(((CAN->sFIFOMailBox[0].RDLR & 0xff) << 8) + ((CAN->sFIFOMailBox[0].RDLR >> 8) & 0xff))/12;
}else if (cId == ATTEMP_ID){
	murchik.ATTemp = ((CAN->sFIFOMailBox[0].RDLR >> 16) & 0xff) - 50;
}else if (cId == GEAR_ID){
	murchik.gear = ((CAN->sFIFOMailBox[0].RDLR >> 16) & 0x0f);
	murchik.isGLock = ((CAN->sFIFOMailBox[0].RDLR >> 30) & 0xff);
}else if (cId == BRAKE_STATE_ID){
	murchik.breaking = ((CAN->sFIFOMailBox[0].RDHR) & 0xff);
	murchik.isBrake = ((CAN->sFIFOMailBox[0].RDHR) & 0xff);
}else if (cId == BUTTON_STATE_ID){
	murchik.steerButton = ((CAN->sFIFOMailBox[0].RDHR >> 16) & 0x40?1:0);
}else if (cId == FUEL_FLOW_ID){
	//murchik.fuel = (((CAN->sFIFOMailBox[0].RDLR >> 16) & 0xff00) + (CAN->sFIFOMailBox[0].RDHR & 0xff))/100;
	murchik.fuel = (256*((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff) + ((CAN->sFIFOMailBox[0].RDHR >> 16) & 0xff))/5;
	murchik.engTemp = ((CAN->sFIFOMailBox[0].RDLR >> 0) & 0xff) - 40;
}else if (cId == STEER_CONTROL_ID){
	murchik.opActiveTimer = OP_ACTIVE_TIMEOUT;
	murchik.steerTargetAngle = ((CAN->sFIFOMailBox[0].RDLR >> 16) & 0x7ff) - 1024;
	murchik.steerMoment = ((CAN->sFIFOMailBox[0].RDLR) & 0x7ff) - 1024;

	murchik.opData = ((CAN->sFIFOMailBox[0].RDLR >> 11) & 0x1f);
	murchik.accTest1 = ((CAN->sFIFOMailBox[0].RDHR) & 0xff);
	murchik.accTest2 = ((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff);
#if (CONTROL_MODE == MOMENT_CONTROL)
	murchik.steerTargetMoment = ((CAN->sFIFOMailBox[0].RDLR) & 0x7ff) - 1024;
#endif

}else if (cId == ACCEL_PEDAL_ID){
	murchik.accel = 100*((CAN->sFIFOMailBox[0].RDLR >> 16) & 0xff)/255;
}else if (cId == LIGHT_STATE_ID){
	murchik.lightState = ((CAN->sFIFOMailBox[0].RDHR >> 16) & 0x07);
}else if (cId == TEST_AHB2_ID){
	murchik.ahbTest1 = ((CAN->sFIFOMailBox[0].RDLR));
	murchik.ahbTest2 = ((CAN->sFIFOMailBox[0].RDHR));
}else if (cId == LONG_CONTROL_ID){
	murchik.accAccel2 = (((CAN->sFIFOMailBox[0].RDHR & 0x0f) << 8) + ((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff));
	murchik.accAccel = (((CAN->sFIFOMailBox[0].RDLR & 0x0f) >> 8) + ((CAN->sFIFOMailBox[0].RDLR >> 24) & 0xff));
	murchik.accActive = CAN->sFIFOMailBox[0].RDLR & 0x03;
}else if (cId == ACC_STATE_ID){
	murchik.accDistance =	(CAN->sFIFOMailBox[0].RDLR>>2) & 0x03;
	//SG_ SET_SPEED : 8|7@1+ (1,20) [0|200] "kph" XXX
	murchik.accSpeed = ((CAN->sFIFOMailBox[0].RDLR >> 8) & 0x7f) + 20;

	//murchik.accActive = (CAN->sFIFOMailBox[0].RDLR >> 4) & 0x01;

//	murchik.accTest1 = (CAN->sFIFOMailBox[0].RDLR) & 0xff;
//	murchik.accTest2 = (CAN->sFIFOMailBox[0].RDLR >> 8) & 0xff;
//	murchik.accTest3 = (CAN->sFIFOMailBox[0].RDLR >> 16) & 0xff;
//	murchik.accTest4 = (CAN->sFIFOMailBox[0].RDLR >> 24) & 0xff;
//	murchik.accTest5 = (CAN->sFIFOMailBox[0].RDHR >> 0) & 0xff;
}else if ((cId == TPMS_ANSWER_ID)){
		if ((CAN->sFIFOMailBox[0].RDLR & 0x30) == 0x10){
			sendConfirm(TPMS_REQ_ID);
		}

		packets[packetNumber].id = cId;
		packets[packetNumber].len = CAN->sFIFOMailBox[0].RDTR;
		packets[packetNumber].data.d[0] = CAN->sFIFOMailBox[0].RDLR;
		packets[packetNumber].data.d[1] = CAN->sFIFOMailBox[0].RDHR;
		packetNumber++;
		packetNumber %= MAX_PACKET;
}

CAN->RF0R |= CAN_RF0R_RFOM0; /* release FIFO */
}

void initCanBus(void){
	RCC->APB1ENR |= RCC_APB1ENR_CANEN;

	GPIOB->MODER &= ~(GPIO_MODER_MODER8 + GPIO_MODER_MODER9);
	GPIOB->MODER |= GPIO_MODER_MODER8_1 + GPIO_MODER_MODER9_1;
	GPIOB->AFR[1] |= 0x00000044;


	/* (1) Enter CAN init mode to write the configuration */
	/* (2) Wait the init mode entering */
	/* (3) Exit sleep mode */
	/* (4) Loopback mode, set timing to 1Mb/s: BS1 = 4, BS2 = 3,
	       prescaler = 6 */
	/* (5) Leave init mode */
	/* (6) Wait the init mode leaving */
	/* (7) Enter filter init mode, (16-bit + mask, filter 0 for FIFO 0) */
	/* (8) Acivate filter 0 */
	/* (9) Set the Id and the mask (all bits of standard id care */
	/* (10) Leave filter init */
	/* (11) Set FIFO0 message pending IT enable */
	CAN->MCR |= CAN_MCR_INRQ; /* (1) */
	while ((CAN->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) /* (2) */
	{
	  /* add time out here for a robust application */
	}
	CAN->MCR &=~ CAN_MCR_SLEEP; /* (3) */

#define BS1	3
#define BS2	2
	//CAN->BTR |= CAN_BTR_LBKM | BS2 << 20 | BS1 << 16 | ((MCK/(500000*(BS1+BS2+3)))-1); /* (4) */
	CAN->BTR |=  BS2 << 20 | BS1 << 16 | ((MCK/(500000*(BS1+BS2+3)))-1); /* (4) */
	CAN->MCR &=~ CAN_MCR_INRQ; /* (5) */
		while ( (CAN->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) /* (6) */
	{
	  /* add time out here for a robust application */
	}

	CAN->FMR |= CAN_FMR_FINIT; /* (7) */

//	CAN->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 |  CAN_FA1R_FACT2
//			|  CAN_FA1R_FACT3 |  CAN_FA1R_FACT4 | CAN_FA1R_FACT5 | CAN_FA1R_FACT6 | CAN_FA1R_FACT7;
//
//	CAN->sFilterRegister[0].FR1 = ECO_BUT_ID << 5 | 0x7fe << 21; /* (9) */
//	CAN->sFilterRegister[1].FR1 = STEERING_WHEEL_POS_ID << 5 | 0x7ff << 21; /* (9) */
//	CAN->sFilterRegister[2].FR1 = LDW_STATE_ID << 5 | 0x7ff << 21; /* (9) */
//	CAN->sFilterRegister[3].FR1 = STEER_CONTROL_ID << 5 | 0x7ff << 21; /* (9) */
//	CAN->sFilterRegister[4].FR1 = BSW_STATE_ID << 5 | 0x7ff << 21;
//	CAN->sFilterRegister[5].FR1 = SPEED_1_ID << 5 | 0x7ff << 21;
//	CAN->sFilterRegister[6].FR1 = STEER_TEST_ID << 5 | 0x7ff << 21;

	CAN->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 |  CAN_FA1R_FACT2
			|  CAN_FA1R_FACT3 |  CAN_FA1R_FACT4 |  CAN_FA1R_FACT5
			|  CAN_FA1R_FACT6 |  CAN_FA1R_FACT7 |  CAN_FA1R_FACT8
			| CAN_FA1R_FACT9 | CAN_FA1R_FACT10
			| CAN_FA1R_FACT11
			| CAN_FA1R_FACT12
			| CAN_FA1R_FACT13
			; /* (8) */

	CAN->sFilterRegister[0].FR1 = STEERING_WHEEL_POS_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[1].FR1 = SPEED_1_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[2].FR1 = SPEED_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[3].FR1 = ATTEMP_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[4].FR1 = GEAR_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[5].FR1 = LDW_STATE_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[6].FR1 = FUEL_FLOW_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[7].FR1 = BSW_STATE_ID << 5 | 0x7ff << 21;
	CAN->sFilterRegister[8].FR1 = ACCEL_PEDAL_ID << 5 | 0x7ff << 21;
	CAN->sFilterRegister[9].FR1 = BRAKE_STATE_ID << 5 | 0x7cf << 21;
	CAN->sFilterRegister[10].FR1 = TPMS_ANSWER_ID << 5 | 0x7ff << 21;
	CAN->sFilterRegister[11].FR1 = STEER_CONTROL_ID << 5 | 0x7f0 << 21;
	CAN->sFilterRegister[12].FR1 = LONG_CONTROL_ID << 5 | 0x7f0 << 21;
	CAN->sFilterRegister[13].FR1 = LIGHT_STATE_ID << 5 | 0x7f0 << 21;


	CAN->FMR &=~ CAN_FMR_FINIT; /* (10) */
	CAN->IER |= (CAN_IER_FMPIE0 | CAN_IER_FMPIE1); /* (11) */
	NVIC_SetPriority(CEC_IRQn, 10);
	NVIC_EnableIRQ(CEC_IRQn);
}

uint8_t sendPacket(uint16_t pid, uint32_t data0, uint32_t data1){

uint32_t i = 1000000;
static uint8_t errCnt;

	while (((CAN->TSR & CAN_TSR_TME0) != CAN_TSR_TME0) && (--i));

	if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0){ /* (1) */
		CAN->sTxMailBox[0].TDTR = 8; /* (2) */
		//CAN->sTxMailBox[0].TDLR = 2 | 0x1A<<8 | 0x90<<16; /* (3) */
		//09-18-84-00-04-00-04-00
		CAN->sTxMailBox[0].TDLR = data0; /* (3) */
		CAN->sTxMailBox[0].TDHR = data1;
		//CAN->sTxMailBox[0].TDLR = 2 | 0x21<<8 | j<<16; /* (3) */
		//CAN->sTxMailBox[0].TDLR = 2 | 0x21<<8 | usingPids[j]<<16;
		//CAN->sTxMailBox[0].TDHR = 0;
		CAN->sTxMailBox[0].TIR = (uint32_t)(pid << 21
							  | CAN_TI0R_TXRQ); /* (4) */

		//DEBUG_MSG(2,("send OK CAN status 0x%x", CAN->ESR))
		//CAN->TSR |= CAN_TSR_TME0;

		errCnt = 0;
	}else{
		errCnt ++;
		DEBUG_MSG(BDL,(COL_RED"send ERROR CAN ESR 0x%x, TSR 0x%x"COL_END, CAN->ESR, CAN->TSR))
		initCanBus();
		CAN->TSR |= CAN_TSR_TME0;
	}
	return errCnt;
}

void sendConfirm(uint16_t pid){
	if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0){ /* (1) */
		CAN->sTxMailBox[0].TDTR = 8; /* (2) */
		//CAN->sTxMailBox[0].TDLR = 2 | 0x1A<<8 | 0x90<<16; /* (3) */
		CAN->sTxMailBox[0].TDLR = 0x30;//2 | 0x21<<8 | LDW_REQ_PID<<16; /* (3) */
		//CAN->sTxMailBox[0].TDLR = 2 | 0x21<<8 | j<<16; /* (3) */
		//CAN->sTxMailBox[0].TDLR = 2 | 0x21<<8 | usingPids[j]<<16;
		CAN->sTxMailBox[0].TDHR = 0;
		CAN->sTxMailBox[0].TIR = (uint32_t)(pid << 21
							  | CAN_TI0R_TXRQ); /* (4) */

//		packets[packetNumber].id = LDW_REQ_ID;
//		packets[packetNumber].len = 8;
//		packets[packetNumber].data.d[0] = CAN->sTxMailBox[0].TDLR;
//		packets[packetNumber].data.d[1] = 0;
//		packetNumber++;
	}
}

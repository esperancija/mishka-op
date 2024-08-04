
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
	//process_post_synch(&steer_process, calc_data_event, 0);
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
#ifndef USE_LCD
}else if (cId == SPEED_ID){
	murchik.speed = 10*(((CAN->sFIFOMailBox[0].RDLR & 0xff) << 8) + ((CAN->sFIFOMailBox[0].RDLR >> 8) & 0xff))/12;
#else
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
	murchik.speed = ((CAN->sFIFOMailBox[0].RDHR >> 8) & 0xff00) + ((CAN->sFIFOMailBox[0].RDHR >> 24) & 0xff);
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
#endif
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
			|  CAN_FA1R_FACT6 |  CAN_FA1R_FACT7
#ifdef USE_LCD
			|  CAN_FA1R_FACT8
			| CAN_FA1R_FACT9 | CAN_FA1R_FACT10
			| CAN_FA1R_FACT11
			| CAN_FA1R_FACT12
			| CAN_FA1R_FACT13
#endif
			; /* (8) */

	CAN->sFilterRegister[0].FR1 = STEERING_WHEEL_POS_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[1].FR1 = SPEED_1_ID << 5 | 0x7ff << 21; /* (9) */
	//CAN->sFilterRegister[2].FR1 = SPEED_2_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[2].FR1 = SPEED_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[3].FR1 = ATTEMP_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[4].FR1 = GEAR_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[5].FR1 = LDW_STATE_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[6].FR1 = FUEL_FLOW_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[7].FR1 = STEER_CONTROL_ID << 5 | 0x7f0 << 21;
#ifdef USE_LCD
	CAN->sFilterRegister[8].FR1 = BSW_STATE_ID << 5 | 0x7ff << 21;
	CAN->sFilterRegister[9].FR1 = ACCEL_PEDAL_ID << 5 | 0x7ff << 21;
	CAN->sFilterRegister[10].FR1 = BRAKE_STATE_ID << 5 | 0x7cf << 21;
	CAN->sFilterRegister[11].FR1 = TPMS_ANSWER_ID << 5 | 0x7ff << 21;
	CAN->sFilterRegister[12].FR1 = LONG_CONTROL_ID << 5 | 0x7f0 << 21;
	CAN->sFilterRegister[13].FR1 = LIGHT_STATE_ID << 5 | 0x7f0 << 21;
#endif

	CAN->FMR &=~ CAN_FMR_FINIT; /* (10) */
	CAN->IER |= (CAN_IER_FMPIE0 | CAN_IER_FMPIE1); /* (11) */
	NVIC_SetPriority(CEC_IRQn, 10);
	NVIC_EnableIRQ(CEC_IRQn);
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

uint8_t sendPacket(uint16_t pid, uint32_t data0, uint32_t data1){

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
		return 1;
	}else{
		//DEBUG_MSG(2,(COL_BLUE"CAN status 0x%x"COL_END, CAN->ESR))
		//CAN->TSR |= CAN_TSR_TME0;
	}
	return 0;
}


void parseTPMSData(uint32_t data0, uint32_t data1){


static PidData tpmsPidData;
static uint16_t curPid = 0, dataIndex;//, pidNumder;

//convert into byte array
uint8_t t[8], i;
for (i=0;i<4;i++){
	t[i] 	= (data0 >> (8*i)) & 0xff;
	t[i+4] 	= (data1 >> (8*i)) & 0xff;
}



//	if ((packets[i].id == TPMS_REQ_ID) &&
//		(packets[i].data.t[0] == 2) && //len ==2
//		(packets[i].data.t[1] == 0x21)){ //first byte of PID
//
//		curPid = packets[i].data.t[2];
//
//	//			DEBUG_MSG(BDL,("Set new PID %d", curPid))
//				if (pidData[curPid].len > 0)
//					curPid = 0;
//
//		if (pidData[curPid].len != pidData[curPid].lenWrited)
//			pidData[curPid].len = 0;
//		i++;
//		continue;
//	}


	DEBUG_MSG(BDL,("Start switch  %d len %d/%d pid 0x%x",
			t[0] >> 4, tpmsPidData.len, tpmsPidData.lenWrited, curPid))

	DEBUG_MSG(BDL,( "%x-%x-%x-%x-%x-%x-%x-%x \r\n",
			(t[0]) & 0xff,
			(t[1]) & 0xff,
			(t[2]) & 0xff,
			(t[3]) & 0xff,
			(t[4]) & 0xff,
			(t[5]) & 0xff,
			(t[6]) & 0xff,
			(t[7]) & 0xff))

//	if ((tpmsPidData.len != tpmsPidData.lenWrited) || (tpmsPidData.len == 0)){
		switch (t[0] >> 4){
			case 0:
				DEBUG_MSG(BDL,("case 0 data %d", t[0]))
				tpmsPidData.len = t[0] & 0x3f;
				memcpy(tpmsPidData.data, &t[1], tpmsPidData.len);
				tpmsPidData.lenWrited = tpmsPidData.len;
				//curPid = 0;
				break;
			case 1:
				tpmsPidData.len = t[1];
				tpmsPidData.lenWrited = 0;
				curPid = ((t[2] & 0x3f) << 8) | t[3];

				DEBUG_MSG(BDL,("case 1 pid 0x%x, data 0x%x, len %d",
						curPid, t[0],t[1]))

				//sendConfirm(TPMS_REQ_ID);

				if (tpmsPidData.len >= 6+7){
					memcpy(tpmsPidData.data, &t[2], 6);
					tpmsPidData.lenWrited += 6;
				}else{
					memcpy(tpmsPidData.data, &t[2], tpmsPidData.len);
					tpmsPidData.lenWrited += tpmsPidData.len;
				}
				break;
			case 2:
//						DEBUG_MSG(BDL,("case 2 pid %d data %x len %d",
//								curPid, packets[i].data.t[0], pidData[curPid].len))
				dataIndex = t[0] & 0x0f;
				if ((dataIndex == 0) || (dataIndex > (MAX_PID_DATA_LEN - 6)/7))
					break;
				if (tpmsPidData.len > 0){
					if ((tpmsPidData.len > 6+7*dataIndex)){
						memcpy(&tpmsPidData.data[6+7*(dataIndex-1)], &t[1], 7);
						tpmsPidData.lenWrited += 7;
					}else{
						memcpy(&tpmsPidData.data[6+7*(dataIndex-1)], &t[1],
								tpmsPidData.len - (6+7*(dataIndex - 1)));
						tpmsPidData.lenWrited += tpmsPidData.len - (6+7*(dataIndex - 1));
						//curPid = 0;
					}
				}
				break;
			case 3:
				DEBUG_MSG(BDL,("case 3 pid %d data %d",t[0]))
				break;
		}
//	}


	if (tpmsPidData.len == tpmsPidData.lenWrited){
		DEBUG_MSG(BDL,(
						"New data: pid=0x%x len=%02d/%02d \r\n"
						"%02x %02x %02x %02x | %02x %02x %02x %02x ||"
						"%02x %02x %02x %02x ||| %02x %02x %02x %02x \r\n"
						"%02x %02x %02x %02x | %02x %02x %02x %02x ||"
						"%02x %02x %02x %02x ||| %02x %02x %02x %02x "
						"\r\n",
						curPid,
						tpmsPidData.lenWrited, tpmsPidData.len,
						tpmsPidData.data[0], tpmsPidData.data[1], tpmsPidData.data[2], tpmsPidData.data[3],
						tpmsPidData.data[4], tpmsPidData.data[5], tpmsPidData.data[6], tpmsPidData.data[7],
						tpmsPidData.data[8], tpmsPidData.data[9], tpmsPidData.data[10], tpmsPidData.data[11],
						tpmsPidData.data[12], tpmsPidData.data[13], tpmsPidData.data[14], tpmsPidData.data[15],
						tpmsPidData.data[16], tpmsPidData.data[17], tpmsPidData.data[18], tpmsPidData.data[19],
						tpmsPidData.data[20], tpmsPidData.data[21], tpmsPidData.data[22], tpmsPidData.data[23],
						tpmsPidData.data[24], tpmsPidData.data[25], tpmsPidData.data[26], tpmsPidData.data[27],
						tpmsPidData.data[28], tpmsPidData.data[29], tpmsPidData.data[30], tpmsPidData.data[31]
						  ));

		switch (curPid){
			case TPMS_FL_PID:
				murchik.tpmsData.FL.pres = tpmsPidData.data[10]*1379/10133;
				murchik.tpmsData.FL.temp = tpmsPidData.data[11]-50;
				break;
			case TPMS_RL_PID:
				murchik.tpmsData.RL.pres = tpmsPidData.data[10]*1379/10133;
				murchik.tpmsData.RL.temp = tpmsPidData.data[11]-50;
				break;
			case TPMS_RR_PID:
				murchik.tpmsData.RR.pres = tpmsPidData.data[10]*1379/10133;
				murchik.tpmsData.RR.temp = tpmsPidData.data[11]-50;
				break;
			case TPMS_FR_PID:
				murchik.tpmsData.FR.pres = tpmsPidData.data[10]*1379/10133;
				murchik.tpmsData.FR.temp = tpmsPidData.data[11]-50;
				break;
		}
		curPid = 0;
	}
}


uint8_t processCanPackets(Car * murchik){

static uint16_t i;
//static uint8_t j;
char debugMsg[1000];
//char* writePointer = debugMsg;

	NVIC_DisableIRQ(CEC_IRQn);

#if (MODE == 0)

		dbg_send_bytes((const unsigned char*)"\033[2J:",5);
		//up
		dbg_send_bytes((const unsigned char*)"\033[20A",6);
		//left
		dbg_send_bytes((const unsigned char*)"\033[100D",6);


#define PID_NUM 256
PidData pidData[PID_NUM];
uint16_t curPid = 0, index;//, pidNumder;
uint16_t savePacketNumber = packetNumber;


//process new pid data
	memset(pidData, 0, sizeof(pidData));
	//memset(&murchik->ldwData, 0, sizeof(LDWData));

//	if (packetNumber)
//		STATUS_LOAD_ON_PHY;

	DEBUG_MSG(BDL,("Got new %d packets",packetNumber))
	//STATUS_LOAD_OFF_PHY;
	i=0;
	while (i<packetNumber){
		DEBUG_MSG(BDL,( "id=0x%x %x-%x-%x-%x-%x-%x-%x-%x \r\n",
									packets[i].id,
									(packets[i].data.t[0]) & 0xff,
									(packets[i].data.t[1]) & 0xff,
									(packets[i].data.t[2]) & 0xff,
									(packets[i].data.t[3]) & 0xff,
									(packets[i].data.t[4]) & 0xff,
									(packets[i].data.t[5]) & 0xff,
									(packets[i].data.t[6]) & 0xff,
									(packets[i].data.t[7]) & 0xff))

//		DEBUG_MSG(BDL,("Start %d %d len %d/%d pid %d",
//				i, packets[i].data.t[0], packets[i].data.t[0], packets[i].data.t[1], packets[i].id ))

			if ((packets[i].id == TPMS_REQ_ID) &&
				(packets[i].data.t[0] == 2) && //len ==2
				(packets[i].data.t[1] == 0x21)){ //first byte of PID

			curPid = packets[i].data.t[2];

//			DEBUG_MSG(BDL,("Set new PID %d", curPid))
					if (pidData[curPid].len > 0)
						curPid = 0;

			if (pidData[curPid].len != pidData[curPid].lenWrited)
				pidData[curPid].len = 0;
			i++;
			continue;
		}

		if (packets[i].id == TPMS_ANSWER_ID){
//					DEBUG_MSG(BDL,("Start switch  %d len %d/%d pid %d",
//							packets[i].data.t[0] >> 4, pidData[curPid].len, pidData[curPid].lenWrited, curPid))
			if ((pidData[curPid].len != pidData[curPid].lenWrited) || (pidData[curPid].len == 0)){
				switch (packets[i].data.t[0] >> 4){
					case 0:
						DEBUG_MSG(BDL,("case 0 pid %d data %d", curPid, packets[i].data.t[0]))
						pidData[curPid].len = packets[i].data.t[0] & 0x3f;
						memcpy(pidData[curPid].data, (char*)&packets[i].data.t[1], pidData[curPid].len);
						pidData[curPid].lenWrited = pidData[curPid].len;
						//curPid = 0;
						break;
					case 1:
						DEBUG_MSG(BDL,("case 1 pid %d, data %d, len %d", curPid,
								packets[i].data.t[0],
								packets[i].data.t[1]))
						pidData[curPid].len = packets[i].data.t[1];
						pidData[curPid].lenWrited = 0;
						if (pidData[curPid].len >= 6+7){
							memcpy(pidData[curPid].data, (char*)&packets[i].data.t[2], 6);
							pidData[curPid].lenWrited += 6;
							//sendConfirm();
						}else{
							memcpy(pidData[curPid].data, (char*)&packets[i].data.t[2], pidData[curPid].len);
							pidData[curPid].lenWrited += pidData[curPid].len;
							//curPid = 0;
						}
						break;
					case 2:
//						DEBUG_MSG(BDL,("case 2 pid %d data %x len %d",
//								curPid, packets[i].data.t[0], pidData[curPid].len))
						index = packets[i].data.t[0] & 0x0f;
						if ((index == 0) || (index > (MAX_PID_DATA_LEN - 6)/7))
							break;
						if (pidData[curPid].len > 0){
							if ((pidData[curPid].len > 6+7*index)){
								memcpy(&pidData[curPid].data[6+7*(index-1)], (char*)&packets[i].data.t[1], 7);
								pidData[curPid].lenWrited += 7;
							}else{
								memcpy(&pidData[curPid].data[6+7*(index-1)], (char*)&packets[i].data.t[1],
											pidData[curPid].len - (6+7*(index - 1)));
								pidData[curPid].lenWrited += pidData[curPid].len - (6+7*(index - 1));
								//curPid = 0;
							}
						}
						break;
					case 3:
						DEBUG_MSG(BDL,("case 3 pid %d data %d", curPid, packets[i].data.t[0]))
						break;
				}
			}
		}
		if (curPid == TPMS_FL_REQ){
			DEBUG_MSG(BDL,("find FL data pid %d data %d", curPid, packets[i].data.t[0]))
//			murchik->ldwData.right = (pidData[curPid].data[2]-0x80);
//			murchik->ldwData.left[(++murchik->ldwData.leftIndex)%LDW_LEFT_LEN] =  (pidData[curPid].data[3]-0x80);
//			murchik->ldwData.width = 2*pidData[curPid].data[4];
//			murchik->ldwData.isLeft = (pidData[curPid].data[5] & 0x02)?1:0;
//			murchik->ldwData.isRight = (pidData[curPid].data[5] & 0x01)?1:0;
		}
//uint16_t temp;

//		if ((curPid == LDW_REQ_PID) && (pidData[curPid].lenWrited == pidData[curPid].len)){
//			temp = 256*(pidData[curPid].data[10] & 0x07) +
//					pidData[curPid].data[11];
//			if (temp)
//				murchik->fcmData.numTarget = temp;
//			temp = 256*(pidData[curPid].data[15] & 0x07) +
//					pidData[curPid].data[16];
//			if (temp)
//				murchik->fcmData.relSpeed = temp;
////			murchik->fcmData.relDist =  (256*(pidData[curPid].data[23] & 0x07) +
////					pidData[curPid].data[24])/10;
//			murchik->fcmData.relDist = pidData[curPid].data[20];
//			murchik->fcmData.laterOffset = (int8_t)pidData[curPid].data[20];
//			murchik->fcmData.curveRadius = pidData[curPid].data[10];
//		}
		i++;
	}

	//if (j == 0)
//show new pid data
	for (index = 1; index < PID_NUM; index ++){
		if ((pidData[index].len == 0) || (pidData[index].data[0] == 0x7f))
			continue;
		dbg_send_bytes((uint8_t *)debugMsg,
				snprintf((char*)debugMsg, sizeof(debugMsg),
						"pid=0x%x len=%02d/%02d \t\t "
						"%02x %02x %02x %02x | %02x %02x %02x %02x ||"
						"%02x %02x %02x %02x ||| %02x %02x %02x %02x \r\n"
						"%02x %02x %02x %02x | %02x %02x %02x %02x ||"
						"%02x %02x %02x %02x ||| %02x %02x %02x %02x "
						"\r\n",
						index,
						pidData[index].lenWrited, pidData[index].len,
						pidData[index].data[0], pidData[index].data[1], pidData[index].data[2], pidData[index].data[3],
						pidData[index].data[4], pidData[index].data[5], pidData[index].data[6], pidData[index].data[7],
                        pidData[index].data[8], pidData[index].data[9], pidData[index].data[10], pidData[index].data[11],
						pidData[index].data[12], pidData[index].data[13], pidData[index].data[14], pidData[index].data[15],
						pidData[index].data[16], pidData[index].data[17], pidData[index].data[18], pidData[index].data[19],
						pidData[index].data[20], pidData[index].data[21], pidData[index].data[22], pidData[index].data[23],
						pidData[index].data[24], pidData[index].data[25], pidData[index].data[26], pidData[index].data[27],
						pidData[index].data[28], pidData[index].data[29], pidData[index].data[30], pidData[index].data[31]
						   )
		);
		curPid = 0;
		pidData[curPid].len = 0;
	}

#elif (MODE == 1)
	//	j++;
	//	j%=usingPidsSize;
	//if (j == 0){
		dbg_send_bytes((const unsigned char*)"\033[2J:",5);
		//up
		dbg_send_bytes((const unsigned char*)"\033[20A",6);
		//left
		dbg_send_bytes((const unsigned char*)"\033[100D",6);
	//}
	DEBUG_MSG(BDL,("Got new %d id's",lastId));
		for (i=0;i<lastId;i++){
			if ((i % 4) == 0){
				dbg_send_bytes("\r\n", 4);
				//left
				dbg_send_bytes("\033[500D", 6);
			}
//			if (idArr[i].isUpdate == 0xff)
//				dbg_send_bytes(COL_YELLOW,7);
			if (idArr[i].isUpdate)
				dbg_send_bytes(COL_RED,7);
			dbg_send_bytes(debugMsg,
					snprintf(debugMsg, sizeof(debugMsg), "0x%03x/%x/%x=%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x \t",
							idArr[i].id, idArr[i].len, idArr[i].isUpdate,
								idArr[i].data[0], idArr[i].data[1], idArr[i].data[2], idArr[i].data[3],
								idArr[i].data[4], idArr[i].data[5], idArr[i].data[6], idArr[i].data[7])
			);


//	writePointer +=	snprintf(writePointer, sizeof(debugMsg), "0x%03x/%x/%x=%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x | ",
//							idArr[i].id, idArr[i].len, idArr[i].isUpdate,
//								idArr[i].data[0], idArr[i].data[1], idArr[i].data[2], idArr[i].data[3],
//								idArr[i].data[4], idArr[i].data[5], idArr[i].data[6], idArr[i].data[7]
//			);
			dbg_send_bytes(COL_END,4);
			idArr[i].isUpdate = 0;
		}
		dbg_send_bytes(debugMsg,writePointer - debugMsg);
		dbg_send_bytes("\r\n", 4);
#elif (MODE == 2)
#endif
	NVIC_EnableIRQ(CEC_IRQn);

//if (getKeys()){

#if (MODE == 0)
	/* (1) check mailbox 0 is empty */
	/* (2) fill data length = 1 */
	/* (3) fill 8-bit data */
	/* (4) fill Id field and request a transmission */
//	if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0){ /* (1) */
//		CAN->sTxMailBox[0].TDTR = 8; /* (2) */
//		//CAN->sTxMailBox[0].TDLR = 2 | 0x1A<<8 | 0x90<<16; /* (3) */
//		CAN->sTxMailBox[0].TDLR = 2 | 0x21<<8 | LDW_REQ_PID<<16; /* (3) */
//		//CAN->sTxMailBox[0].TDLR = 2 | 0x21<<8 | j<<16; /* (3) */
//		//CAN->sTxMailBox[0].TDLR = 2 | 0x21<<8 | usingPids[j]<<16;
//		CAN->sTxMailBox[0].TDHR = 0;
//		CAN->sTxMailBox[0].TIR = (uint32_t)(LDW_REQ_ID << 21
//							  | CAN_TI0R_TXRQ); /* (4) */
//
		packetNumber = 0;


//		//make manual loopback
//		packets[packetNumber].id = LDW_REQ_ID;
//		packets[packetNumber].len = 8;
//		packets[packetNumber].data.d[0] = CAN->sTxMailBox[0].TDLR;
//		packets[packetNumber].data.d[1] = 0;
//		packetNumber++;
//	}
	//STATUS_LINK_OFF_PHY;

	if (savePacketNumber){
//		for (i=0;i<savePacketNumber;i++)
//			//if (packets[i].id == LDW_STATE_ID)
//			  DEBUG_MSG(BDL,("%db 0x%x %x %x %x %x %x %x %x %x",
//					  packets[i].len, packets[i].id,
//					  packets[i].data.t[0], packets[i].data.t[1],
//					  packets[i].data.t[2], packets[i].data.t[3],
//					  packets[i].data.t[4], packets[i].data.t[5],
//					  packets[i].data.t[6], packets[i].data.t[7]
//			  ))

		return 1;
	}else
		return 0;
#else
//	for (i=0;i<packetNumber;i++)
//		  DEBUG_MSG(BDL,("%db \t 0x%x %x %x %x %x %x %x %x %x", steer_process
//				  packets[i].len, packets[i].id,
//				  packets[i].data.t[0], packets[i].data.t[1],
//				  packets[i].data.t[2], packets[i].data.t[3],
//				  packets[i].data.t[4], packets[i].data.t[5],
//				  packets[i].data.t[6], packets[i].data.t[7]
//		  ))
      packetNumber = 0;
#endif
}


uint16_t getPacketNumber(volatile CanPacket ** pPackets){
	*pPackets = (CanPacket *)&packets;
	return packetNumber;
}

void setPacketNumber(uint16_t p){
	packetNumber= p;
}

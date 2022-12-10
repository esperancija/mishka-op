
#include "stdint.h"
#include "can.h"
#include "stm32f0xx.h"
#include "main.h"
#include "debug-uart.h"
#include "steer.h"

uint16_t lastId = 0;

void CEC_IRQHandler(void){
	uint8_t debugMsg[500];
uint16_t cId = CAN->sFIFOMailBox[0].RIR >> 21;

if (cId == STEERING_WHEEL_POS_ID){
	murchik.steerPosition = (((CAN->sFIFOMailBox[0].RDLR & 0xff) << 8) +
									((CAN->sFIFOMailBox[0].RDLR >> 8) & 0xff)) - 0x1000;//0x0ffd;//0x1000;
	murchik.steerSpeed = ((CAN->sFIFOMailBox[0].RDLR >> 16) & 0xffff)-0x1000;
	process_post(&steer_process, calc_data_event, 0);
}else if (cId == STEER_TEST_ID){
	murchik.testData = ((CAN->sFIFOMailBox[0].RDLR) & 0xff) - 127;

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
}else if (cId == SPEED_1_ID){
	murchik.speed = 10*(((CAN->sFIFOMailBox[0].RDLR & 0xff) << 8) + ((CAN->sFIFOMailBox[0].RDLR >> 8) & 0xff))/12;
}else if (cId == STEER_CONTROL_ID){
	murchik.opActiveTimer = OP_ACTIVE_TIMEOUT;

	//murchik.opData = ((CAN->sFIFOMailBox[0].RDHR) & 0xff);
	murchik.opData = ((CAN->sFIFOMailBox[0].RDLR >> 11) & 0x1f);

	murchik.steerTargetAngle = ((CAN->sFIFOMailBox[0].RDLR >> 16) & 0x7ff) - 1024;
#if (CONTROL_MODE == MOMENT_CONTROL)
	murchik.steerTargetMoment = ((CAN->sFIFOMailBox[0].RDLR) & 0x7ff) - 1024;
#endif

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

	CAN->FA1R |= CAN_FA1R_FACT0 | CAN_FA1R_FACT1 |  CAN_FA1R_FACT2
			|  CAN_FA1R_FACT3 |  CAN_FA1R_FACT4 | CAN_FA1R_FACT5 | CAN_FA1R_FACT6;

	CAN->sFilterRegister[0].FR1 = ECO_BUT_ID << 5 | 0x7fe << 21; /* (9) */
	CAN->sFilterRegister[1].FR1 = STEERING_WHEEL_POS_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[2].FR1 = LDW_STATE_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[3].FR1 = STEER_CONTROL_ID << 5 | 0x7ff << 21; /* (9) */
	CAN->sFilterRegister[4].FR1 = BSW_STATE_ID << 5 | 0x7ff << 21;
	CAN->sFilterRegister[5].FR1 = SPEED_1_ID << 5 | 0x7ff << 21;
	CAN->sFilterRegister[6].FR1 = STEER_TEST_ID << 5 | 0x7ff << 21;


	CAN->FMR &=~ CAN_FMR_FINIT; /* (10) */
	CAN->IER |= (CAN_IER_FMPIE0 | CAN_IER_FMPIE1); /* (11) */
	NVIC_SetPriority(CEC_IRQn, 10);
	NVIC_EnableIRQ(CEC_IRQn);
}

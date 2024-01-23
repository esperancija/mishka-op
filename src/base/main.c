/**
 * \file
 *         main mishka file
 * \author
 *         Alexander Khoryakov (starostarf@gmail.com)
 */


#include "contiki.h"
#include "main.h"

#include "debug-uart.h"
#include "steer.h"

#include "can.h"
#include "core.h"

#include "flash.h"

#include "accControl.h"


/*---------------------------------------------------------------------------*/
PROCESS(menu_process, "Menu process");

Car murchik = {};

static uint8_t isNeedSetKoefs;

/* We require the processes to be started automatically */
AUTOSTART_PROCESSES(
		&menu_process
		,&steer_process
);

/* Implementation of the display process */
PROCESS_THREAD(menu_process, ev, data) {

static struct etimer timer;

static uint32_t statusCnt;
static uint8_t onState, res;
static uint8_t oldSteerKey;
	   uint8_t steerKey;

#ifdef USE_LCD
static uint8_t oldShowState = 0xff;
uint8_t buf[200];
#endif

	PROCESS_BEGIN();

	initDebug();
	DEBUG_MSG(CDL,(COL_GREEN"Start firmware from %s %s"COL_END, __DATE__, __TIME__));
	initLeds();


	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

		DEBUG_MSG(BDL,("Start menu process"));

		//init Ds control
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		DS_INIT;
		BUT_SETUP;

		LCD_init();

		for (res=0;res<3;res++){
			RED_ON;
			delay_ms(100);
			RED_OFF;
			delay_ms(100);
		}

		while (1) {
			etimer_set(&timer, CLOCK_SECOND/10);

			//reset IWDG
			IWDG->KR = 0xAAAA;

			if (murchik.isEcoMode == 0){
				DS_ON;
			}else{
				DS_OFF;
			}

			if ((++statusCnt % 1) == 0){
				DEBUG_MSG(CDL,(COL_YELLOW"SBI=%d(%d) %d/%d RELAY=%d BUTTON=%d steer=%d DAC=%d/%d targetMoment=%d,"
						" state=%d koefs=%d-%d speed=%d, opActive=%d"COL_END,
						murchik.accControlAdc, getAccKey(), murchik.steerSensor1, murchik.steerSensor2,
						FS_RELAY_STATE, IS_BUT_PRESS, murchik.steerPosition,
						DAC->DHR12R1, DAC->DHR12R2, murchik.steerTargetMoment, murchik.currentState,
						murchik.koefs.steerRatio, murchik.koefs.steerActuatorDelay, murchik.speedFL, murchik.opData & opActive));
			}
			//setAccCtlLevel(9999);

#ifdef SBI_TEST
			steerKey = (IS_BUT_PRESS)?lkasOnKey:noKey;
#else
			steerKey = getAccKey();
#endif

#define ACC_ADC_LIMIT 2000
			//if ((steerKey == lkasOnKey) && (oldSteerKey == noKey)){
			if (((steerKey == mishkaKey) || (steerKey == lkasOnKey)) && (oldSteerKey == noKey)){
				onState ^= 1;
			}
			oldSteerKey = steerKey;

			if (onState)
				res = sendPacket(STEER_TEST_ID, 0x0300, (murchik.koefs.steerRatio*5) | ((murchik.koefs.steerActuatorDelay) << 8));
			else
				res = sendPacket(STEER_TEST_ID, 0x0000, (murchik.koefs.steerRatio*5) | ((murchik.koefs.steerActuatorDelay) << 8));

			if (res > 3)
				RED_TOGLE;
			else{
				if ((FS_RELAY_STATE) && (murchik.currentState != controlState))
					RED_ON;
				else
					RED_OFF;
			}


#ifdef USE_LCD
			if (murchik.showState != oldShowState){
				LCD_fillScreen(BLACK);
			}

			oldShowState = murchik.showState;
#endif

			if (isNeedSetKoefs){
				setKoefs(&murchik.koefs);
				isNeedSetKoefs = 0;
			}

			if (murchik.koefs.steerRatio == 0)
				murchik.koefs.steerRatio = 43;
			if (murchik.koefs.steerActuatorDelay == 0)
				murchik.koefs.steerActuatorDelay = 210;

//			switch(murchik.showState){
//				case showNormalMenuState:
//					showNormalMenu();
//					break;
//				case showOPMenu1State:
//				case showOPMenu2State:
//					showOPMenu(murchik.showState);
//					break;
//			}

#if (0) //send TPMS Request
			px++;
#define TPMS_REQ_DELAY	100
			switch (px % TPMS_REQ_DELAY){
				case 1:
					i = sendPacket(TPMS_REQ_ID, TPMS_FL_REQ, 0);
					DEBUG_MSG(BDL,(COL_ORANGE"send 0x%x FL to 0x%x, res %d", TPMS_FL_REQ, TPMS_REQ_ID, i));
					break;
				case 6:
					i = sendPacket(TPMS_REQ_ID, TPMS_FR_REQ, 0);
					DEBUG_MSG(BDL,(COL_ORANGE"send 0x%x FR to 0x%x, res %d", TPMS_FR_REQ, TPMS_REQ_ID, i));
					break;
				case 11:
					i = sendPacket(TPMS_REQ_ID, TPMS_RL_REQ, 0);
					DEBUG_MSG(BDL,(COL_ORANGE"send 0x%x RL to 0x%x, res %d", TPMS_RL_REQ, TPMS_REQ_ID, i));
					break;
				case 16:
					i = sendPacket(TPMS_REQ_ID, TPMS_RR_REQ, 0);
					DEBUG_MSG(BDL,(COL_ORANGE"send 0x%x RR to 0x%x, res %d", TPMS_RR_REQ, TPMS_REQ_ID, i));
					break;
				case 5:
				case 10:
				case 15:
				case 20:
					i=0;
					volatile CanPacket * pPackets;
					uint16_t pNum = getPacketNumber(&pPackets);
					while (i<pNum){
						DEBUG_MSG(BDL,(COL_ORANGE"process %d packet from %d 0x%x"COL_END, i,pNum, pPackets))
						parseTPMSData(pPackets[i].data.d[0], pPackets[i].data.d[1]);
						i++;
					}
					setPacketNumber(0);
					initCanBus();
					//break;
				default:
					//if (isOPActive){
//					if (murchik.isOPActive){
//						sendPacket(STEER_TEST_ID, 127 | 0x300, 0);
//					}else {
//						sendPacket(STEER_TEST_ID, 127 , 0);
//					}
					break;
			}
#endif

#if (0)
			switch ((cnt++/20) % 10){
			case 0:
				murchik.steerTargetAngle=0;
				break;
			case 1:
				murchik.steerTargetAngle=10;
				break;
			case 2:
				murchik.steerTargetAngle=20;
				break;
			case 3:
				murchik.steerTargetAngle=30;
				break;
			case 4:
				murchik.steerTargetAngle=90;
				break;
			case 5:
				murchik.steerTargetAngle=-90;
				break;
			case 6:
				murchik.steerTargetAngle=0;
				break;
			case 7:
				murchik.steerTargetAngle=-30;
				break;
			case 8:
				murchik.steerTargetAngle=30;
				break;
			case 9:
				murchik.steerTargetAngle=-20;
				break;

			}
#endif
			// and wait until the vent we receive is the one we're waiting for
			PROCESS_YIELD();
		}
		PROCESS_END();
}


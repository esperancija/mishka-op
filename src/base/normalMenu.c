
#include "main.h"
#include "normalMenu.h"
#include "icons.h"
#include "text.h"
#include "graph.h"
#include "debug-uart.h"
#include <stdlib.h>

#ifdef USE_LCD

static uint16_t i;
uint8_t debugMsg[500];

//static uint16_t datat[100];

static uint8_t gearVal[4];
static uint8_t consum[8];
static uint8_t engTemp[4];
static uint16_t engTempX;
static uint8_t transmTemp[4];
static uint16_t transmTempX;
static uint8_t FLTemp[8];
static uint8_t FLPres[8];
static uint8_t FRTemp[8];
static uint8_t FRPres[8];
static uint8_t RLTemp[8];
static uint8_t RLPres[8];
static uint8_t RRTemp[8];
static uint8_t RRPres[8];

static uint8_t steerDiff[8];
static uint8_t steerAngle[8];

//static uint8_t fDiff[16];
//static uint8_t rDiff[16];

static uint8_t accel[4];
//static uint8_t breaking[4];
static uint8_t steerPos[4];

static uint8_t test1[4];
static uint8_t test2[4];
//static uint8_t test3[4];
//static uint8_t test4[4];
//static uint8_t test5[4];

static uint16_t steerPX, setSteerPX;
static uint16_t steerPY, setSteerPY;

void showNormalMenu(void){

uint16_t lineColor = WHITE;

			//LCD_fillScreen(BLACK);
	//		LCD_setCursor(px,px);
	//	    LCD_setTextColor(WHITE);
	//	    LCD_setTextSize(1);
	//		LCD_writeString(test);

	#define ENGINE_ICON_X	220
	#define ENGINE_ICON_Y	0
	        LCD_icon(ENGINE_ICON_X,ENGINE_ICON_Y, EngTempIcon, BLUE_G);

	#define TRANSM_ICON_X	280
	#define TRANSM_ICON_Y	0
	        LCD_icon(TRANSM_ICON_X,TRANSM_ICON_Y, TransmTempIcon, BLUE_G);

	//calc  LPer100
	#define DATA_SIZE 5
	static uint16_t dataBuf[DATA_SIZE];
	static uint32_t index;
				dataBuf[(index++)%DATA_SIZE] = murchik.fuel;
				murchik.LPer100 = 0;
				for (i=0; i<DATA_SIZE; i++){
					murchik.LPer100 += dataBuf[i];
				}
				//murchik.LPer100 = ((murchik.LPer100/DATA_SIZE)*40)*100/murchik.speed;
				//murchik.LPer100 = (murchik.LPer100/DATA_SIZE)*288/murchik.speed/5;
				murchik.LPer100 = murchik.LPer100*255/murchik.speed/(DATA_SIZE/2);

	//show BSW state
				if (murchik.bswState & 0x08)
					LCD_icon(1,1, BSWIcon, GREEN);
				else
					LCD_icon(1,1, BSWIcon, DDGRAY);
	//show OP state
				lineColor = DGRAY;//BLACK;
				if (murchik.opData & opActive){
					LCD_icon(40,1, OPIcon, GREEN);
					lineColor = DGRAY;
				}else if (murchik.steerButton)
					LCD_icon(40,1, OPIcon, ORANGE);
				else if (murchik.isOPActive)
					LCD_icon(40,1, OPIcon, DGRAY);
				else
					LCD_icon(40,1, OPIcon, DDGRAY);

				//show steed diff
				LCD_setCursor(40,40);
				LCD_setTextColor(BLACK);
				LCD_writeString(steerDiff);
				LCD_SetFont(additionalFont);
				LCD_setTextColor(lineColor);
				snprintf((char*)steerDiff,sizeof(steerDiff),"%d.%c",
						(murchik.steerPosition - murchik.steerTargetAngle)/2,
						((murchik.steerPosition - murchik.steerTargetAngle)%2)?'5':'0');
				LCD_setCursor(40,40);
				LCD_writeString(steerDiff);

				//show steer angle
				LCD_setCursor(80,40);
				LCD_setTextColor(BLACK);
				LCD_writeString(steerAngle);
				LCD_SetFont(additionalFont);
				LCD_setTextColor(OLIVE);
				snprintf((char*)steerAngle,sizeof(steerAngle),"%d.%c",
						murchik.steerPosition/2, (murchik.steerPosition%2)?'5':'0');
				LCD_setCursor(80,40);
				LCD_writeString(steerAngle);

	//show accState
				if (murchik.accActive & 0x01)
					LCD_icon(80,1, AccIcon, DGRAY);
				else if (murchik.accActive & 0x02)
					LCD_icon(80,1, AccIcon, GREEN);
				else
					LCD_icon(80,1, AccIcon, DDGRAY);

//murchik.ATTemp = 125;
//murchik.engTemp = 125;

	//show eng Temp
	#define	ENG_TEMP_X	ENGINE_ICON_X+12
	#define	ENG_TEMP_Y	35
				LCD_SetFont(bigFont);
				LCD_setTextColor(BLACK);
				LCD_setCursor(engTempX,ENG_TEMP_Y);
				LCD_writeString(engTemp);
				if (murchik.engTemp >= 0)
					snprintf((char*)engTemp,sizeof(engTemp),"%d", murchik.engTemp);
				else
					snprintf((char*)engTemp,sizeof(engTemp),"/%d", abs(murchik.engTemp));
				engTempX = ENG_TEMP_X-getStringLen(engTemp)/2;
				LCD_setCursor(engTempX,ENG_TEMP_Y);
				LCD_setTextColor(OLIVE);
				LCD_writeString(engTemp);

	//show transTemp
	#define	TRANSM_TEMP_X	TRANSM_ICON_X+12
	#define	TRANSM_TEMP_Y	35
				LCD_SetFont(bigFont);
				LCD_setTextColor(BLACK);
				LCD_setCursor(transmTempX,TRANSM_TEMP_Y);
				LCD_writeString(transmTemp);
				if (murchik.ATTemp >= 0)
					snprintf((char*)transmTemp,sizeof(transmTemp),"%d", murchik.ATTemp);
				else
					snprintf((char*)transmTemp,sizeof(transmTemp),"/%d", abs(murchik.ATTemp));
				transmTempX = TRANSM_TEMP_X-getStringLen(transmTemp)/2;
				LCD_setCursor(transmTempX,TRANSM_TEMP_Y);
				if (murchik.ATTemp < 100)
					LCD_setTextColor(OLIVE);
				else
					LCD_setTextColor(RED);
				LCD_writeString(transmTemp);

	//show OP state
	#define CAR_ICON_X	145
	#define CAR_ICON_Y	50
	#define CARL_XSSH	15 //car line X start shift
	#define CARL_YSSH	30 //car line Y start shift
	#define CARL_XESH	0 //car line X end shift
	#define CARL_YESH	45 //car line Y end shift
	#define CARL_WIDTH	5

        LCD_icon(CAR_ICON_X,CAR_ICON_Y, CarIcon, DDGRAY);
        for (i=0;i<CARL_WIDTH;i++){
        	if (murchik.opData & opLeftLine)
        		lineColor = WHITE;
        	else
        		lineColor = DDGRAY;
        	LCD_drawLine(CAR_ICON_X-CARL_XSSH-i, CAR_ICON_Y+CARL_YSSH,
        		CAR_ICON_X+CARL_XESH-i, CAR_ICON_Y-CARL_YESH, lineColor);

        	if (murchik.opData & opRightLine)
					lineColor = WHITE;
				else
					lineColor = DDGRAY;
            LCD_drawLine(CAR_ICON_X+ICON_W+CARL_XSSH+i, CAR_ICON_Y+CARL_YSSH,
            		CAR_ICON_X+ICON_W-CARL_XESH+i, CAR_ICON_Y-CARL_YESH, lineColor);
        }
        //show head light
        if (murchik.lightState == 0x06)
        	lineColor = BLUE;
        else
        	lineColor = BLACK;
        for (i=0;i<3;i++){
        	LCD_drawLine(CAR_ICON_X+5, CAR_ICON_Y-5,
        	         CAR_ICON_X+7+i*3, CAR_ICON_Y-30-5, lineColor);

        	LCD_drawLine(CAR_ICON_X+32-5, CAR_ICON_Y-5,
        	         CAR_ICON_X+32-7-i*3, CAR_ICON_Y-30-5, lineColor);
        }

    //show steer position
        LCD_fillCircle(steerPX, steerPY, 5, BLACK);
        steerPX = 37 - murchik.steerPosition/30;
        steerPY = 75 + abs(murchik.steerPosition/70);
        LCD_fillCircle(steerPX, steerPY, 5, WHITE);

        LCD_fillCircle(setSteerPX, setSteerPY, 5, BLACK);
        setSteerPX = 37 - murchik.steerTargetAngle/30;
        setSteerPY = 68 + abs(murchik.steerTargetAngle/70);
        LCD_fillCircle(setSteerPX, setSteerPY, 5, OLIVE);

	//showGear
	#define GEAR_X	20
	#define GEAR_Y	100
	uint16_t	gearLockColor;
				switch (murchik.isGLock){
					case 1:
						gearLockColor = RED;
						break;
					case 2:
					case 3:
						gearLockColor = ORANGE;
						break;
					default:
						gearLockColor = GREEN;
						break;
				}
				LCD_SetFont(veryBigFont);
				LCD_setTextColor(BLACK);
				LCD_setCursor(GEAR_X,GEAR_Y);
				LCD_writeString(gearVal);
				if (murchik.gear <= 6)
					itoa(murchik.gear & 0x07, (char*)gearVal, 10);
				else
					*gearVal = 0;
				LCD_setCursor(GEAR_X,GEAR_Y);
				LCD_setTextColor(gearLockColor);
				LCD_writeString(gearVal);



				LCD_drawCircle(GEAR_X+34/2,GEAR_Y+25, 32, gearLockColor);
				LCD_drawCircle(GEAR_X+34/2,GEAR_Y+25, 33, gearLockColor);
				LCD_drawCircle(GEAR_X+34/2,GEAR_Y+25, 34, gearLockColor);
				LCD_drawCircle(GEAR_X+34/2,GEAR_Y+25, 35, gearLockColor);
				LCD_drawCircle(GEAR_X+34/2,GEAR_Y+25, 36, gearLockColor);


	//show LPER100
	#define LPER100_X	120
	#define LPER100_Y	100
				LCD_SetFont(veryBigFont);
				LCD_setTextColor(BLACK);
				LCD_setCursor(LPER100_X,LPER100_Y);
				LCD_writeString(consum);
				if (murchik.LPer100 < 1000)
					snprintf((char*)consum,sizeof(consum),"%d/%d", murchik.LPer100/10, murchik.LPer100%10);
				else
					snprintf((char*)consum,sizeof(consum),"...");
				//itoa((px/2+6)%100, consum, 10);
				LCD_setTextColor(GREEN);
				LCD_setCursor(LPER100_X,LPER100_Y);
				LCD_writeString(consum);

//	//show diffs
	int16_t frontDiff = (murchik.speedFL - murchik.speedFR);///((murchik.speedFL + murchik.speedFR)/10);
//				//frontDiff -= murchik.steerPosition/64;
//				LCD_setCursor(180,65);
//				LCD_setTextColor(BLACK);
//				LCD_SetFont(additionalFont);
//				LCD_writeString(fDiff);
//				LCD_setTextColor(RED);
//				//if (frontDiff >= 0)
//					snprintf((char*)fDiff,sizeof(fDiff),"%d %d", frontDiff, murchik.steerPosition);
//	//			else
//	//				snprintf((char*)fDiff,sizeof(fDiff),"/%d", abs(frontDiff));
//
//				LCD_setCursor(180,65);
//				LCD_writeString(fDiff);
//
	int16_t rearDiff = (murchik.speedRL - murchik.speedRR);///((murchik.speedRL + murchik.speedRR)/10);
//				//rearDiff -= murchik.steerPosition/70;
//				LCD_setCursor(180,200);
//				LCD_setTextColor(BLACK);
//				LCD_writeString(rDiff);
//				//itoa((px+10)%150, FLTemp, 10);
//				LCD_setTextColor(RED);
//	//			if (rearDiff >= 0)
//				snprintf((char*)rDiff,sizeof(rDiff),"%d %d", rearDiff, murchik.steerPosition);
//	//			else
//	//				snprintf((char*)rDiff,sizeof(rDiff),"/%d", abs(rearDiff));
//				LCD_setCursor(180,200);
//				LCD_writeString(rDiff);


	//wheels
	#define DIFF_LIMIT	10//30

	#define FLW_X		210
	#define FLW_TEXT_X	(FLW_X-45) //55
	#define FLW_Y		163//35
	#define RRW_X		250
	#define RRW_TEXT_X	283
	#define RRW_Y		200
				if (frontDiff > DIFF_LIMIT){
					if (murchik.accel <= 1)
						LCD_icon(FLW_X,FLW_Y, WheelIconLeft, RED);
					else
						LCD_icon(FLW_X,FLW_Y, WheelIconLeft, GREEN);
				}else
					LCD_icon(FLW_X,FLW_Y, WheelIconLeft, DDGRAY);

				LCD_SetFont(additionalFont);
				LCD_setCursor(FLW_TEXT_X,FLW_Y);
				LCD_setTextColor(BLACK);
				LCD_writeString(FLTemp);
				LCD_setCursor(FLW_TEXT_X,FLW_Y);
				LCD_setTextColor(GREENYELLOW);
				snprintf((char*)FLTemp,sizeof(FLTemp),"%d C", murchik.tpmsData.FL.temp);
				LCD_writeString(FLTemp);

				LCD_SetFont(additionalFont);
				LCD_setCursor(FLW_TEXT_X,FLW_Y+15);
				LCD_setTextColor(BLACK);
				LCD_writeString(FLPres);
				LCD_setCursor(FLW_TEXT_X,FLW_Y+15);
				LCD_setTextColor(GREENYELLOW);
				snprintf((char*)FLPres,sizeof(FLPres),"%d.%d b", murchik.tpmsData.FL.pres/10, murchik.tpmsData.FL.pres%10);
				LCD_writeString(FLPres);

				if (frontDiff < -DIFF_LIMIT){
					if (murchik.accel <= 1)
						LCD_icon(RRW_X,FLW_Y, WheelIconLeft, RED);
					else
						LCD_icon(RRW_X,FLW_Y, WheelIconLeft, GREEN);
				}else
					LCD_icon(RRW_X,FLW_Y, WheelIconRight, DDGRAY);

				LCD_setCursor(RRW_TEXT_X,FLW_Y);
				LCD_setTextColor(BLACK);
				LCD_writeString(FRTemp);
				//itoa((px+10)%150, FLTemp, 10);
				LCD_SetFont(additionalFont);
				LCD_setTextColor(GREENYELLOW);
				snprintf((char*)FRTemp,sizeof(FRTemp),"%d C", murchik.tpmsData.FR.temp);
				LCD_setCursor(RRW_TEXT_X,FLW_Y);
				LCD_writeString(FRTemp);

				LCD_SetFont(additionalFont);
				LCD_setCursor(RRW_TEXT_X,FLW_Y+15);
				LCD_setTextColor(BLACK);
				LCD_writeString(FRPres);
				LCD_setCursor(RRW_TEXT_X,FLW_Y+15);
				LCD_setTextColor(GREENYELLOW);
				snprintf((char*)FRPres,sizeof(FRPres),"%d.%d b", murchik.tpmsData.FR.pres/10, murchik.tpmsData.FR.pres%10);
				LCD_writeString(FRPres);


				if (rearDiff > DIFF_LIMIT){
					if (murchik.accel <= 1)
						LCD_icon(FLW_X,RRW_Y, WheelIconLeft, RED);
					else
						LCD_icon(FLW_X,RRW_Y, WheelIconLeft, GREEN);
				}else
					LCD_icon(FLW_X,RRW_Y, WheelIconLeft, DDGRAY);

				LCD_setCursor(FLW_TEXT_X,RRW_Y);
				LCD_setTextColor(BLACK);
				LCD_writeString(RLTemp);
				LCD_SetFont(additionalFont);
				LCD_setTextColor(GREENYELLOW);
				snprintf((char*)RLTemp,sizeof(RLTemp),"%d C", murchik.tpmsData.RL.temp);
				LCD_setCursor(FLW_TEXT_X,RRW_Y);
				LCD_writeString(RLTemp);

				LCD_SetFont(additionalFont);
				LCD_setCursor(FLW_TEXT_X,RRW_Y+15);
				LCD_setTextColor(BLACK);
				LCD_writeString(RLPres);
				LCD_setCursor(FLW_TEXT_X,RRW_Y+15);
				LCD_setTextColor(GREENYELLOW);
				snprintf((char*)RLPres,sizeof(RLPres),"%d.%d b", murchik.tpmsData.RL.pres/10, murchik.tpmsData.RL.pres%10);
				LCD_writeString(RLPres);


				if (frontDiff < -DIFF_LIMIT){
					if (murchik.accel <= 1)
						LCD_icon(RRW_X,RRW_Y, WheelIconRight, RED);
					else
						LCD_icon(RRW_X,RRW_Y, WheelIconRight, GREEN);
				}else
					LCD_icon(RRW_X,RRW_Y, WheelIconRight, DDGRAY);
				LCD_setCursor(RRW_TEXT_X,RRW_Y);
				LCD_setTextColor(BLACK);
				LCD_writeString(RRTemp);
				LCD_SetFont(additionalFont);
				LCD_setTextColor(GREENYELLOW);
				snprintf((char*)RRTemp,sizeof(RRTemp),"%d C", murchik.tpmsData.RR.temp);
				LCD_setCursor(RRW_TEXT_X,RRW_Y);
				LCD_writeString(RRTemp);

				LCD_SetFont(additionalFont);
				LCD_setCursor(RRW_TEXT_X,RRW_Y+15);
				LCD_setTextColor(BLACK);
				LCD_writeString(RRPres);
				LCD_setCursor(RRW_TEXT_X,RRW_Y+15);
				LCD_setTextColor(GREENYELLOW);
				snprintf((char*)RRPres,sizeof(RRPres),"%d.%d b", murchik.tpmsData.RR.pres/10, murchik.tpmsData.RR.pres%10);
				LCD_writeString(RRPres);



//show pedal position
			LCD_SetFont(bigFont);
			LCD_setCursor(1,200);
			LCD_setTextColor(BLACK);
			LCD_writeString(accel);
			//itoa((px+10)%150, FLTemp, 10);

			if (murchik.accAccel == 0){
				if (murchik.accel){
					LCD_setTextColor(YELLOW);
					snprintf((char*)accel,sizeof(accel),"%d", murchik.accel);
				}else{
					LCD_setTextColor(DDGRAY);
					snprintf((char*)accel,sizeof(accel),"..");
				}
			}else if (murchik.accAccel < 0x7f){
				LCD_setTextColor(GREEN);
				snprintf((char*)accel,sizeof(accel),"%d", murchik.accAccel);
			}else{
				LCD_setTextColor(RED);
				snprintf((char*)accel,sizeof(accel),".%d", 0xff - murchik.accAccel);
			}

			LCD_setCursor(1,200);
			LCD_writeString(accel);

	//show bsw state
				LCD_SetFont(bigFont);
				LCD_setCursor(50,200);
				LCD_setTextColor(BLACK);
				LCD_writeString(steerPos);
				LCD_setTextColor(DCYAN);
				snprintf((char*)steerPos,sizeof(steerPos),"%c%d",(murchik.accTest1<0)?'.':'/', murchik.accTest1);//murchik.accSetSpeed);//murchik.accAccel2);
				LCD_setCursor(50,200);
				LCD_writeString(steerPos);

#define TEST_DATA_Y	160
#define TEST_COLOR WHITE
			//show test state
				LCD_SetFont(bigFont);
				LCD_setCursor(0,TEST_DATA_Y);
				LCD_setTextColor(BLACK);
				LCD_writeString(test1);
				LCD_setTextColor(TEST_COLOR);
				snprintf((char*)test1,sizeof(test1),"%d", murchik.steerMoment); //accTest1);
				LCD_setCursor(0,TEST_DATA_Y);
				LCD_writeString(test1);
			//show test state
				LCD_SetFont(bigFont);
				LCD_setCursor(70,TEST_DATA_Y);
				LCD_setTextColor(BLACK);
				LCD_writeString(test2);
				LCD_setTextColor(TEST_COLOR);
				snprintf((char*)test2,sizeof(test2),"%d",murchik.accTest2*2);
				//snprintf((char*)test2,sizeof(test2),"%d", murchik.accControlAdc/ACC_ADC_DIV); //accTest1);
				LCD_setCursor(70,TEST_DATA_Y);
				LCD_writeString(test2);

//				LCD_SetFont(bigFont);
//				LCD_setCursor(120,TEST_DATA_Y);
//				LCD_setTextColor(BLACK);
//				LCD_writeString(test3);
//				LCD_setTextColor(TEST_COLOR);
//				snprintf((char*)test3,sizeof(test3),"%d", murchik.accTest3);
//				LCD_setCursor(120,TEST_DATA_Y);
//				LCD_writeString(test3);
//
//				LCD_SetFont(bigFont);
//				LCD_setCursor(180,TEST_DATA_Y);
//				LCD_setTextColor(BLACK);
//				LCD_writeString(test4);
//				LCD_setTextColor(TEST_COLOR);
//				snprintf((char*)test4,sizeof(test4),"%d", murchik.accTest4);
//				LCD_setCursor(180,TEST_DATA_Y);
//				LCD_writeString(test4);
//
//				LCD_SetFont(bigFont);
//				LCD_setCursor(240,TEST_DATA_Y);
//				LCD_setTextColor(BLACK);
//				LCD_writeString(test5);
//				LCD_setTextColor(TEST_COLOR);
//				snprintf((char*)test5,sizeof(test5),"%d", murchik.accTest5);
//				LCD_setCursor(240,TEST_DATA_Y);
//				LCD_writeString(test5);

	////show breaking info
	//			LCD_SetFont(bigFont);
	//			LCD_setCursor(1,210);
	//			LCD_setTextColor(BLACK);
	//			LCD_writeString(breaking);
	//			//itoa((px+10)%150, FLTemp, 10);
	//			LCD_setTextColor(BRRED);
	//			snprintf((char*)breaking,sizeof(breaking),"%d", murchik.breaking);
	//			LCD_setCursor(1,210);
	//			LCD_writeString(breaking);
	//
	//			if (murchik.isBrake)
	//				LCD_fillCircle(20, 190, 20, RED);
	//			else
	//				LCD_fillCircle(20, 190, 20, BLACK);

}

#endif

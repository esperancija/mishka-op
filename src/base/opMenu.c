

#include "main.h"
#include "normalMenu.h"
#include "icons.h"
#include "text.h"
#include "graph.h"
#include "debug-uart.h"
#include "stdlib.h"

#define STR_BUF_SIZE	6
void showValue(uint16_t x, uint16_t y, uint8_t* buff, int16_t value, uint16_t color, char* comment){
	LCD_SetFont(bigFont);
	LCD_setTextColor(BLACK);
	LCD_setCursor(x,y);
	LCD_writeString(buff);
	if (value >= 0)
		snprintf((char*)buff,STR_BUF_SIZE,"%d", value);
	else
		snprintf((char*)buff,STR_BUF_SIZE,".%d", abs(value));
	LCD_setCursor(x,y);
	LCD_setTextColor(color);
	LCD_writeString(buff);

	LCD_SetFont(additionalFont);
	LCD_setCursor(x+70,y+10);
	LCD_writeString((uint8_t*) comment);
}

#define STR_HEX_BUF_SIZE 16
void showHexValue(uint16_t x, uint16_t y, uint8_t* buff, int16_t value, uint16_t color, char* comment){
	LCD_SetFont(bigFont);
	LCD_setTextColor(BLACK);
	LCD_setCursor(x,y);
	LCD_writeString(buff);
		snprintf((char*)buff,STR_BUF_SIZE,"%x", value);
	LCD_setCursor(x,y);
	LCD_setTextColor(color);
	LCD_writeString(buff);

	LCD_SetFont(additionalFont);
	LCD_setCursor(x+70,y+10);
	LCD_writeString((uint8_t*) comment);
}



void showOPMenu(uint8_t showState){
//show steer Position
//static uint8_t steerPos[STR_BUF_SIZE];
//showValue(10, 10, steerPos, murchik.steerPosition/2, GREEN, "REAL ANGLE");
//
////show steer target Position
//static uint8_t steerTPos[STR_BUF_SIZE];
//showValue(10, 50, steerTPos, murchik.steerTargetAngle/2, OLIVE, "SET ANGLE");

////show steer target moment
//static uint8_t steerMoment[STR_BUF_SIZE];
//showValue(10, 90, steerMoment, murchik.opData & 0x01, RED, "ACTIVE");


//	static uint8_t steerPos[STR_HEX_BUF_SIZE];
//	showHexValue(10, 10, steerPos, murchik.ahbTest1, GREEN, "1");
//
//
//	static uint8_t steerTPos[STR_HEX_BUF_SIZE];
//	showHexValue(10, 50, steerTPos, murchik.ahbTest2, OLIVE, "2");


//show steerActuatorDelay
static uint8_t sadState[STR_BUF_SIZE];
if (showState == showOPMenu2State){
	showValue(10, 200, sadState, murchik.koefs.steerActuatorDelay*2, GREEN, "STEER ACTUATOR DELAY");
}else{
	showValue(10, 200, sadState, murchik.koefs.steerActuatorDelay*2, DGRAY, "STEER ACTUATOR DELAY");
}

//show steerRatio
static uint8_t steerTestData[STR_BUF_SIZE];
if (showState == showOPMenu1State)
	showValue(10, 160, steerTestData, murchik.koefs.steerRatio*5, GREEN, "STEER RATIO");
else {
	showValue(10, 160, steerTestData, murchik.koefs.steerRatio*5, DGRAY, "STEER RATIO");
}
//showValue(10, 200, steerTestData, murchik.key, ORANGE, "Button adc");

//show steer diff
static uint8_t steerDiff[STR_BUF_SIZE];
showValue(200, 10, steerDiff,  murchik.steerPosition/2 - murchik.steerTargetAngle/2, BLUE, "DIFF");

//show eps moment
//static uint8_t steerEpsMoment[STR_BUF_SIZE];
//showValue(200, 160, steerEpsMoment, murchik.epsMoment, DGRAY, "1");
//
//static uint8_t steerEpsMoment2[STR_BUF_SIZE];
//showValue(200, 200, steerEpsMoment, murchik.epsMoment2, DGRAY, "2");



}

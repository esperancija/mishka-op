

#include "main.h"
#include "normalMenu.h"
//#include "icons.h"
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

//show steerActuatorDelay
static uint8_t sadState[STR_BUF_SIZE];
if (showState == showOPMenu1State){
	showValue(10, 200, sadState, murchik.koefs.steerActuatorDelay*2, GREEN, "STEER ACTUATOR DELAY");
}else{
	showValue(10, 200, sadState, murchik.koefs.steerActuatorDelay*2, DGRAY, "STEER ACTUATOR DELAY");
}

//show steerRatio
static uint8_t steerTestData[STR_BUF_SIZE];
if (showState == showOPMenu2State)
	showValue(10, 160, steerTestData, murchik.koefs.steerRatio*5, GREEN, "STEER RATIO");
else {
	showValue(10, 160, steerTestData, murchik.koefs.steerRatio*5, DGRAY, "STEER RATIO");
}
//showValue(10, 200, steerTestData, murchik.key, ORANGE, "Button adc");

//show steer diff
static uint8_t steerDiff[STR_BUF_SIZE];
showValue(200, 10, steerDiff,  murchik.steerPosition/2 - murchik.steerTargetAngle/2, BLUE, "DIFF");

}

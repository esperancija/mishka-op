#include "text.h"

#include "main.h"
#include "mainFont.h"
#include "additionalFont.h"

#include "changeFont.h"

#include "droidDigits18x32.h"
#include "bigFont.h"

Font font;

u8 _cp437 = 0;

u16 cursorX = 0,
    cursorY = 0;

u8 textSize = 1;

u8 wrap = 1;

u16 textColor   = RED,
    textBgColor = TRANSPARENT_COLOR;

inline static void LCD_drawChar(u16 x0, u16 y0, unsigned char c, u16 color, u16 bg, uint8_t size) {
    u16 scaledWidth       = (u16) (size * 6),
        doubleScaledWidth = scaledWidth * size;

    u16 x1 = (u16) (x0 + scaledWidth - 1),
        y1 = (u16) (y0 + 8 * size - 1);

    u16 doubleSize = size * size;
    u16 count      = (u16) (48 * doubleSize);

    u16 charPixels[count];

    u16 mx, my;
    s8  i, j, sx, sy;
    u8  line;
    u16 pixelColor;

    if (x0 >= LCD_getWidth() || y0 >= LCD_getHeight() || x1 < 0 || y1 < 0) return;

    if (!_cp437 && (c >= 176)) c++; // Handle 'classic' charset behavior

    u16 characterNumber = (u16) (c * 5);

    if (bg == TRANSPARENT_COLOR) {
        LCD_readPixels(x0, y0, x1, y1, charPixels);
    }

    LCD_setAddressWindowToWrite(x0, y0, x1, y1);

    for (i = 0; i < 6; i++) {
        line = (u8) (i < 5 ? pgm_read_byte(fontOrigin + characterNumber + i) : 0x0);
        my   = (u16) (i * size);


        for (j = 0; j < 8; j++, line >>= 1) {
            mx = (u16) (j * doubleScaledWidth);

            pixelColor = line & 0x1 ? color : bg;

            if (pixelColor == TRANSPARENT_COLOR) continue;

            for (sx = 0; sx < size; ++sx) {
                for (sy = 0; sy < size; ++sy) {
                    charPixels[mx + my + sy * scaledWidth + sx] = pixelColor;
                }
            }
        }
    }

    LCD_setSpi16();
    dmaSendData16(charPixels, count);
    LCD_setSpi8();
}

inline void LCD_write(unsigned char c) {
    if (c == '\n') {
        cursorY += textSize * 8;
        cursorX = 0;
    } else if (c == '\r') {
        cursorX = 0;
    } else {
        if (wrap && ((cursorX + textSize * 6) >= LCD_getWidth())) { // Heading off edge?
            cursorX = 0;            // Reset x to zero
            cursorY += textSize * 8; // Advance y one line
        }
        LCD_drawChar(cursorX, cursorY, c, textColor, textBgColor, textSize);
        cursorX += textSize * 6;
    }
}

//void LCD_writeString(unsigned char *s) {
//    while (*(s)) LCD_write(*s++);
//}

void LCD_setCursor(u16 x, u16 y) {
    cursorX = x;
    cursorY = y;
}

void LCD_setTextSize(u8 size) {
    textSize = size;
}

void LCD_setTextColor(u16 color) {
    textColor = color;
}

void LCD_setTextBgColor(u16 color) {
    textBgColor = color;
}

u16 LCD_getCursorX() {
    return cursorX;
}

u16 LCD_getCursorY() {
    return cursorY;
}



void LCD_SetFont(uint8_t newFont)
{
//newFont = bigFont;
	switch (newFont){
	case veryBigFont:
		font.type = veryBigFont;
		font.addr = (uint8_t*)Droid_Sans34x56;//(uint8_t*)Times_New_Roman38x51;//Droid_Sans38x48;
		font.height = 56;//51;//48;
		font.width = 34;
		font.shift = 46;
		break;
	case bigFont:
		font.type = bigFont;
		font.addr = (uint8_t*)Droid_Sans18x32;//Droid_Sans_Mono7x13;//Droid_Sans12x13;
		font.height = 32;
		font.width = 18;
		font.shift = 46;
		break;
	case changeFont: //this font must have identical width of symbols
		font.type = changeFont;
		font.addr = (uint8_t*)Droid_Sans13x17;
		font.height = 17;
		font.width =  13;
		font.shift = 42;
		break;
	case additionalFont: //lower case for bold strings
		font.type = additionalFont;
		font.addr = (uint8_t*)Droid_Sans16x14;
		font.height = 14;
		font.width = 16;
		font.shift = 32;
		break;
	default:
	case normalFont: //lower case for very small names
		font.type = normalFont;
		font.addr = (uint8_t*)Droid_Sans7x8;
		font.height = 8;
		font.width = 7;
		font.shift = 32;
		break;
	}
}



uint8_t LCD_showChar(u16 x, u16 y, unsigned char charToWrite, u16 color, u8 isVisible){

//void GLCD_WriteChar(char charToWrite)
//{
uint8_t heightBytes, charWidth;
char* curCharData;

//to right show lower symbols space
	if (charToWrite >= font.shift)
		charToWrite -= font.shift;

	//use MikroElektronika GLCD Font Creator 1.2.0.0 type
		heightBytes = (font.height/8)+((font.height%8)?1:0);
		curCharData = (char*)font.addr+(font.width*heightBytes+1)*charToWrite;
		//curCharData = font.addr+(font.width*font.height/8+1)*charToWrite;
		charWidth = (*curCharData);

	uint16_t charPixels[font.width*font.height];
	uint32_t i,j;
		if (isVisible){
			curCharData++;

			for (i=0;i<font.width*font.height;i++)
				charPixels[i] = BLACK;

			for (j=0; j<font.height; j++){
				for (i=0; i<font.width; i++){
					if (curCharData[j/8+i*heightBytes] & (1<<(j%8)))
						charPixels[j*font.width + i] = color;
				}
			}
			LCD_setAddressWindowToWrite(x, y, x+font.width-1, y+font.height-1);
			dmaSendData16(charPixels, font.width*font.height);


		}
		return charWidth;
}

uint8_t getStringLen(uint8_t *s){
	uint8_t len=0;
	while (*s){
		len += LCD_showChar(0,0,*s,0,0);
		s++;
	}
	return len;

}

void LCD_writeString(unsigned char *s) {
	//LCD_fillRect(cursorX, cursorY, font.width*3+3, font.height, BLACK);
    while (*(s)){
    	cursorX += LCD_showChar(cursorX, cursorY, *s++, textColor, 1)+1;
    	LCD_drawFastVLine(cursorX-1,cursorY, font.height, BLACK);
    }
}

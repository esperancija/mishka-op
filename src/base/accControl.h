

#ifndef __ACCCONTROL__
#define __ACCCONTROL__

#include "main.h"

void setAccCtlLevel(uint16_t level);
void initAccControl(void);
uint8_t getAccKey(void);

#if (1)
#define BTN_ACC_LVL		736//675//750
#define BTN_CANCEL_LVL	1364//1200//1400
#define BTN_DOWN_LVL	2025 //1940//1600//2010
#define BTN_MISHKA_LVL	2106
#define BTN_UP_LVL		2482//2000//2570
#define BTN_NOKEY_LVL	2980//2300
#else
#define BTN_ACC_LVL		736//675//750
#define BTN_CANCEL_LVL	1044//1200//1400
#define BTN_DOWN_LVL	1636 //1940//1600//2010
#define BTN_MISHKA_LVL	2106
#define BTN_UP_LVL		2324//2000//2570
#define BTN_NOKEY_LVL	2980//2300
#endif

#define ACCS_ON_LVL		7000
#define ACCS_INC_LVL	350//2675
#define ACCS_DEC_LVL	1100//1000//2000

//acc 750
//cancel 1400
//+ 2570
//- 2010



#endif

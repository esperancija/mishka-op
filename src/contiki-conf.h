#ifndef __CONTIKI_CONF_H__CDBB4VIH3I__
#define __CONTIKI_CONF_H__CDBB4VIH3I__

#include <stdint.h>

//#define MCK 36000000
//#define SETUP_PLLMULL RCC_CFGR_PLLMULL9 // 4Mhz base

#define MCK 48000000
#define SETUP_PLLMULL RCC_CFGR_PLLMULL12 // 4Mhz base

#define CCIF
#define CLIF

#define WITH_UIP 0
#define WITH_ASCII 1

#define CLOCK_CONF_SECOND 1000

#define CLOCK_SECOND CLOCK_CONF_SECOND

/* These names are deprecated, use C99 names. */
typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int8_t s8_t;
typedef int16_t s16_t;
typedef int32_t s32_t;

typedef unsigned int clock_time_t;
typedef unsigned int uip_stats_t;

#ifndef BV
#define BV(x) (1<<(x))
#endif

#define WITH_CONTIKI 1

#endif /* __CONTIKI_CONF_H__CDBB4VIH3I__ */

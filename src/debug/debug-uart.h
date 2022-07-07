#ifndef __DEBUG_UART_H__1V2039076V__
#define __DEBUG_UART_H__1V2039076V__

#include <string.h>
#include "debug.h"
#include <stdint.h>

#ifndef dbg_setup_uart
#define dbg_setup_uart dbg_setup_uart_default
#endif

#define COL_END "\033[0m"
#define COL_RED "\033[31;1m"
#define COL_GREEN "\033[32;1m"
#define COL_YELLOW "\033[33;1m"
#define COL_BLUE "\033[34;1m"
#define COL_MAG "\033[35;1m"
#define COL_CYAN "\033[36;1m"
#define COL_ORANGE "\033[38;5;208m"
#define COL_ACK "\033[38;5;154m"
#define COL_REC "\033[38;5;133m"
#define COL_MQTT "\033[38;5;228m"
#define COL_NOTIF "\033[38;5;48m"
#define COL_GRPW "\033[38;5;211m"


void initDebug(void);
void initDebugRAM(void);

void sendDebugRAM(uint8_t * str);

void wifi_debug_print_msg(int level,
                      char *file, int line, char *text );

#if (DEBUG_LEVEL < CDL)
#define BUFFER_LEN 256//128
#else
#define BUFFER_LEN 256//128
#endif

#define DEBUG_MSG( level, args )                    \
	if (DEBUG_LEVEL <= level){		\
wifi_debug_print_msg(level, (strrchr(__FILE__, 0x5c) +1), __LINE__, (char*)debug_fmt args );}

void print_hexdump_log(uint8_t level, const char *name, const unsigned char *buf, uint8_t length);

void
dbg_setup_uart();

void
dbg_set_input_handler(void (*handler)(const char *inp, unsigned int len));

unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len);

void
dbg_putchar(const char ch);

void
dbg_blocking_putchar(const char ch);

void my_debug(void *ctx, int level, const char *str);

void
dbg_drain();


#endif /* __DEBUG_UART_H__1V2039076V__ */

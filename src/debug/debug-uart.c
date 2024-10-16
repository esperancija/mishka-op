#include <debug-uart.h>
#include <string.h>
//#include <stm32f0x_map.h>
//#include <stm32f0x_dma.h>
//#include <gpio.h>

#include "stm32f0xx.h"
#include "main.h"
#include <stdint.h>
#include <stdarg.h>
#include "mini-printf.h"
#include "contiki-conf.h"
#include "clock.h"
//#include "bootloader/bootloader_config.h"
//#include "rtc.h"

#ifndef DBG_UART
#define DBG_UART USART3
#endif

#ifndef DBG_DMA_NO
#define DBG_DMA_NO 1
#endif

#ifndef DBG_DMA_CHANNEL_NO
#define DBG_DMA_CHANNEL_NO 4
#endif


#define _DBG_DMA_NAME(x) DMA##x
#define DBG_DMA_NAME(x) _DBG_DMA_NAME(x)
#define DBG_DMA DBG_DMA_NAME(DBG_DMA_NO)

#define _DMA_CHANNEL_NAME(x,c) DMA ## x ## _Channel ## c
#define DMA_CHANNEL_NAME(x,c) _DMA_CHANNEL_NAME(x,c)
//#define DBG_DMA_CHANNEL  DMA_CHANNEL_NAME(DBG_DMA_NO, DBG_DMA_CHANNEL_NO)
#define DBG_DMA_CHANNEL DMA1_Channel4

#define _DBG_DMA_CHANNEL_IFCR_CGIF(c) DMA_IFCR_CGIF ## c
#define _XDBG_DMA_CHANNEL_IFCR_CGIF(c) _DBG_DMA_CHANNEL_IFCR_CGIF(c)
#define DBG_DMA_CHANNEL_IFCR_CGIF \
_XDBG_DMA_CHANNEL_IFCR_CGIF(DBG_DMA_CHANNEL_NO)


#ifndef DBG_XMIT_BUFFER_LEN
#define DBG_XMIT_BUFFER_LEN 3000
#endif


static unsigned char xmit_buffer[DBG_XMIT_BUFFER_LEN];
#define XMIT_BUFFER_END &xmit_buffer[DBG_XMIT_BUFFER_LEN]

uint8_t uart_buffer[BUFFER_LEN] = {};

void initDebug(void){
	//init USART3 as debug port
	  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	  RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_DMA1EN;

	//remap DMA request
	  DMA1_CSELR->CSELR |= 0xA000;//ch 4 => 0x0A (USART3_tx)

	  DBG_UART->BRR = MCK/115200/1;//417;//625;//0x1a1;
	  //DBG_UART->BRR = MCK/921600/1;//417;//625;//0x1a1;
	  DBG_UART->CR1 |= USART_CR1_UE;
	  DBG_UART->CR3 = USART_CR3_DMAT;

	  DBG_UART->CR1 |= (USART_CR1_TE);
	//pin setup
		GPIOB->MODER 	&= ~(GPIO_MODER_MODER10);
		GPIOB->MODER 	|= (GPIO_MODER_MODER10_1);
		GPIOB->OSPEEDR 	|= (GPIO_OSPEEDER_OSPEEDR10);
		GPIOB->AFR[1] 	|= 0x400;

	NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
	NVIC_SetPriority(DMA1_Channel4_5_IRQn, 12);
}

int
debug_usart(char *fmt, ...)
{
	int ret;
	va_list va;
	va_start(va, fmt);
	ret = mini_vsnprintf((char*)uart_buffer, BUFFER_LEN, fmt, va);
	va_end(va);

//add /n
	ret = strlen((char*)uart_buffer);
	uart_buffer[ret] = 0x0d;
	uart_buffer[ret+1] = 0x0a;
	uart_buffer[ret+2] = 0;

	dbg_send_bytes((const unsigned char*)uart_buffer, strlen((char*)uart_buffer));
	dbg_drain();
	return ret;
}

int debug_dump(char *fmt, ...)
{
	int ret;
	va_list va;
	va_start(va, fmt);
	ret = mini_vsnprintf((char*)uart_buffer, BUFFER_LEN, fmt, va);
	va_end(va);

	dbg_send_bytes((const unsigned char*)uart_buffer, strlen((char*)uart_buffer));
	dbg_drain();
	return ret;
}

void
	print_hexdump_log(uint8_t level, const char *name, const unsigned char *buf, uint8_t length) {

	if (level >= DEBUG_LEVEL){
		  debug_dump("%s: (%u bytes): ", name, length);
	    while (length--)
	    	debug_dump("%02X ", *buf++);

	    uart_buffer[0] = 0x0d;
	    uart_buffer[1] = 0x0a;
	    uart_buffer[2] = 0;
		dbg_send_bytes((const unsigned char*)uart_buffer, strlen((char*)uart_buffer));
		dbg_drain();
	}
}

void my_debug(void *ctx, int level, const char *str)
{
    debug_usart("%s", str);
    //cloudPublishDebug("%s", str);
}

void wifi_debug_print_msg(int level,
                      char *file, int line, char *text )
{

    char str[256];
    int maxlen = sizeof( str ) - 1;

		snprintf( str, maxlen, "%s(%04d) %d,%ds: %s", file, line,
				clock_time()/CLOCK_SECOND,((clock_time())%CLOCK_SECOND)/(CLOCK_SECOND/100), text );
		str[maxlen] = '\0';
		my_debug( NULL, level, str );

		//malloc in debug_fmt
		//free(text);

#ifdef DEBUG_ESP
#if (HWVER != LPT)

    if (level > 5){
    	snprintf( str, maxlen, "debug:%s(%04d) %s\r\n", file, line, text);
    	sendUSART((uint8_t *)str, strlen(str)+1, commandSendMode);
    }
#endif

#endif

    //setTracePoint(file,line);

}

/* Valid data in head to tail-1 */
/* Read position */
static unsigned char * volatile xmit_buffer_head = xmit_buffer;

/* Write position */
static unsigned char * volatile xmit_buffer_tail = xmit_buffer;

/* xmit_buffer_head == xmit_buffer_tail means empty so we can only store
   DBG_XMIT_BUFFER_LEN-1 characters */

volatile unsigned char dma_running = 0;
static unsigned char * volatile dma_end;

static void
update_dma(void)
{
  if (xmit_buffer_tail == xmit_buffer_head) return;
  DBG_DMA_CHANNEL->CCR = (DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE);

  DBG_DMA_CHANNEL->CPAR = (uint32_t)&DBG_UART->TDR;
  DBG_DMA_CHANNEL->CMAR = (uint32_t)xmit_buffer_head;
  if (xmit_buffer_head < xmit_buffer_tail) {
    DBG_DMA_CHANNEL->CNDTR = xmit_buffer_tail - xmit_buffer_head;
    dma_end = xmit_buffer_tail;
  } else {
    DBG_DMA_CHANNEL->CNDTR =  XMIT_BUFFER_END - xmit_buffer_head;
    dma_end = xmit_buffer;
  }

  DBG_DMA_CHANNEL->CCR |= DMA_CCR_EN;
}

void
DMA1_Channel4_5_IRQHandler()
{
#if (0)
debugRamSend("From boot loader");
#else
  DBG_DMA->IFCR = DBG_DMA_CHANNEL_IFCR_CGIF;
  xmit_buffer_head = dma_end;
  if (xmit_buffer_tail == xmit_buffer_head) {
    dma_running = 0;
    return;
  }
  update_dma();
#endif
}



unsigned int
dbg_send_bytes(const unsigned char *seq, unsigned int len)
{
  /* Since each of the pointers should be read atomically
     there's no need to disable interrupts */
  unsigned char *head = xmit_buffer_head;
  unsigned char *tail = xmit_buffer_tail;
  if (tail >= head) {
    /* Free space wraps */
    unsigned int xfer_len = XMIT_BUFFER_END - tail;
    unsigned int freeBytes = DBG_XMIT_BUFFER_LEN - (tail - head) - 1;
    if (len > freeBytes) len = freeBytes;
    if (xfer_len < len) {
      memcpy(tail, seq, xfer_len);
      seq += xfer_len;
      xfer_len = len - xfer_len;
      memcpy(xmit_buffer, seq, xfer_len);
      tail = xmit_buffer + xfer_len;
    } else {
      memcpy(tail, seq, len);
      tail += len;
      if (tail == XMIT_BUFFER_END) tail = xmit_buffer;
    }
  } else {
    /* Free space continuous */
    unsigned int freeBytes = (head - tail) - 1;
    if (len > freeBytes) len = freeBytes;
    memcpy(tail, seq, len);
    tail += len;
  }
  xmit_buffer_tail = tail;
  if (!dma_running) {
    dma_running = 1;
    update_dma();
  }
  return len;
}

static unsigned char dbg_write_overrun = 0;

void
dbg_putchar(const char ch)
{
  if (dbg_write_overrun) {
    if (dbg_send_bytes((const unsigned char*)"^",1) != 1) return;
  }
  dbg_write_overrun = 0;
  if (dbg_send_bytes((const unsigned char*)&ch,1) != 1) {
    dbg_write_overrun = 1;
  }
}

void
dbg_blocking_putchar(const char ch)
{
  if (dbg_write_overrun) {
    while (dbg_send_bytes((const unsigned char*)"^",1) != 1);
  }
  dbg_write_overrun = 0;
  while (dbg_send_bytes((const unsigned char*)&ch,1) != 1);
}

void
dbg_drain()
{
uint32_t c=60000;
  while((xmit_buffer_tail != xmit_buffer_head) && (c--));
}

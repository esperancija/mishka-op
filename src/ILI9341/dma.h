#ifndef TEST1_DMA_H
#define TEST1_DMA_H

#include "control.h"
#include "stm32f0xx.h"

//#define dmaWait() while(SPI_I2S_GetFlagStatus(SPI_MASTER,SPI_I2S_FLAG_BSY) == SET);
#define dmaWait()  while(GLCD_DMA->CCR & DMA_CCR_EN);

void dmaInit(void);

void dmaSendCmd(u8 cmd);
void dmaSendCmdCont(u8 cmd);

void dmaReceiveDataCont8(u8 *data);

void dmaSendData8(u8 *data, u32 n);
void dmaSendData16(u16 *data, u32 n);

void dmaSendDataCont8(u8 *data, u32 n);
void dmaSendDataCont16(u16 *data, u32 n);

void dmaSendDataBuf16();
void dmaSendDataContBuf16(u16 *data, u32 n);

void dmaFill16(u16 color, u32 n);

#endif //TEST1_DMA_H

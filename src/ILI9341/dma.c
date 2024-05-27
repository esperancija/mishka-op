#include "core.h"

//DMA_InitTypeDef dmaStructure;

#define DMA_BUF_SIZE 2048
u16 dmaBufIndex = 0;
u16 dmaBuffer[DMA_BUF_SIZE];

void dmaInit(void) {
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 9);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
}

//<editor-fold desc="Dma init options and start">

inline static void dmaReceive8(u8 *data, u32 n) {
    //dmaStructure.DMA_MemoryBaseAddr = (u32) data;
    //dmaStructure.DMA_BufferSize     = n;

//    dmaStructure.DMA_Mode               = DMA_Mode_Normal;
//    dmaStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//    dmaStructure.DMA_DIR                = DMA_DIR_PeripheralSRC;
//    dmaStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
//    dmaStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;

	LCD_setSpi8();
	GLCD_RX_DMA->CNDTR = n;//only for start
	GLCD_RX_DMA->CPAR = (uint32_t)&(SPI_MASTER->DR);
	GLCD_RX_DMA->CMAR = (uint32_t)data;
   //set 8 bit block data size memory increment direction & interrupt
	GLCD_RX_DMA->CCR = DMA_CCR_MINC | //DMA_CCR_DIR |
				DMA_CCR_TCIE | DMA_CCR_EN;

    //dmaStartRx();
}

inline static void dmaSend8(u8 *data, u32 n) {
//    DMA_StructInit(&dmaStructure);
//    dmaStructure.DMA_PeripheralBaseAddr = (u32) &(SPI_MASTER->DR);
//    dmaStructure.DMA_Priority           = DMA_Priority_Medium;
//
//    dmaStructure.DMA_MemoryBaseAddr = (u32) data;
//    dmaStructure.DMA_BufferSize     = n;
//
//    dmaStructure.DMA_Mode               = DMA_Mode_Normal;
//    dmaStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//    dmaStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
//    dmaStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
//    dmaStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//
//    dmaStartTx();

	LCD_setSpi8();
	dmaWait();
	GLCD_DMA->CNDTR = n;//only for start
	GLCD_DMA->CPAR = (uint32_t)&(SPI_MASTER->DR);
	GLCD_DMA->CMAR = (uint32_t)data;
   //set 8 bit block data size memory increment direction & interrupt
	GLCD_DMA->CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_DIR |
				DMA_CCR_TCIE | DMA_CCR_EN;
}

inline static void dmaSendCircular16(u16 *data, u32 n) {
//    DMA_StructInit(&dmaStructure);
//    dmaStructure.DMA_PeripheralBaseAddr = (u32) &(SPI_MASTER->DR);
//    dmaStructure.DMA_Priority           = DMA_Priority_Medium;
//
//    dmaStructure.DMA_MemoryBaseAddr = (u32) data;
//    dmaStructure.DMA_BufferSize     = n;
//
//    dmaStructure.DMA_Mode               = DMA_Mode_Circular;
//    dmaStructure.DMA_MemoryInc          = DMA_MemoryInc_Disable;
//    dmaStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
//    dmaStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
//    dmaStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//
//    dmaStartTx();

	LCD_setSpi16();
	dmaWait();
	GLCD_DMA->CNDTR = n;
	GLCD_DMA->CPAR = (uint32_t)&(SPI_MASTER->DR);
	GLCD_DMA->CMAR = (uint32_t)data;
   //set 8 bit block data size memory increment direction & interrupt
	GLCD_DMA->CCR = DMA_CCR_CIRC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
						DMA_CCR_DIR |
							DMA_CCR_TCIE | DMA_CCR_EN;
}

inline static void dmaSend16(u16 *data, u32 n) {
//    DMA_StructInit(&dmaStructure);
//    dmaStructure.DMA_PeripheralBaseAddr = (u32) &(SPI_MASTER->DR);
//    dmaStructure.DMA_Priority           = DMA_Priority_Medium;
//
//    dmaStructure.DMA_MemoryBaseAddr = (u32) data;
//    dmaStructure.DMA_BufferSize     = n;
//
//    dmaStructure.DMA_Mode               = DMA_Mode_Normal;
//    dmaStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
//    dmaStructure.DMA_DIR                = DMA_DIR_PeripheralDST;
//    dmaStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
//    dmaStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//
//    dmaStartTx();

	LCD_setSpi16();
	dmaWait();
	GLCD_DMA->CNDTR = n;
	GLCD_DMA->CPAR = (uint32_t)&(SPI_MASTER->DR);
	GLCD_DMA->CMAR = (uint32_t)data;
   //set 8 bit block data size memory increment direction & interrupt
	GLCD_DMA->CCR = DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 |
						DMA_CCR_DIR |
							DMA_CCR_TCIE | DMA_CCR_EN;
}

//</editor-fold>

//<editor-fold desc="DMA send receive functions">
//5 for (SPI_CR1_BR_1 + SPI_CR1_BR_0);
#define BASE_DELAY 1//30//16

inline void dmaSendCmd(u8 cmd) {
    TFT_CS_RESET;
    TFT_DC_RESET;
    dmaSend8(&cmd, 1);
    dmaWait();
    delay_us(BASE_DELAY);
    TFT_CS_SET;
}

inline void dmaSendCmdCont(u8 cmd) {
    TFT_DC_RESET;
    dmaSend8(&cmd, 1);
    dmaWait();
    delay_us(BASE_DELAY);
}

inline void dmaReceiveDataCont8(u8 *data) {
    u8 dummy = 0xFF;
    dmaSend8(&dummy, 1);
    dmaReceive8(data, 1);
    //dmaWait();
    delay_us(BASE_DELAY);
}

inline void dmaSendData8(u8 *data, u32 n) {
    TFT_CS_RESET;
    TFT_DC_SET;
    dmaSend8(data, n);
    dmaWait();
    delay_us(BASE_DELAY);

    TFT_CS_SET;
}

inline void dmaSendDataCont8(u8 *data, u32 n) {
    TFT_DC_SET;
    dmaSend8(data, n);
    dmaWait();
    delay_us(BASE_DELAY);
}

inline void dmaSendData16(u16 *data, u32 n) {
    TFT_CS_RESET;
    TFT_DC_SET;
    dmaSend16(data, n);
    dmaWait();
    delay_us(BASE_DELAY*2);
    TFT_CS_SET;
}

inline void dmaSendDataCont16(u16 *data, u32 n) {
    TFT_DC_SET;
    dmaSend16(data, n);
    dmaWait();
    delay_us(BASE_DELAY*2);
}

inline static void dmaSendDataCircular16(u16 *data, u32 n) {
    TFT_DC_SET;
    dmaSendCircular16(data, n);
    dmaWait();
    delay_us(BASE_DELAY*3);
}

//</editor-fold>

inline void dmaFill16(u16 color, u32 n) {
    TFT_CS_RESET;
    dmaSendCmdCont(LCD_GRAM);
    while (n != 0) {
        u16 ts = (u16) (n > UINT16_MAX ? UINT16_MAX : n);
        dmaSendDataCircular16(&color, ts);
        n -= ts;
    }
    TFT_CS_SET;
}

//<editor-fold desc="IRQ handlers">

void DMA1_Channel2_3_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF2){
    	//DMA1->IFCR |= DMA_IFCR_CTEIF2;
    	DMA1->IFCR |= DMA_IFCR_CGIF2;
    	GLCD_RX_DMA->CCR &= ~DMA_CCR_EN;
    }

    if (DMA1->ISR & DMA_ISR_TCIF3){
    	//DMA1->IFCR |= DMA_IFCR_CTEIF3;
    	DMA1->IFCR |= DMA_IFCR_CGIF3;
    	GLCD_DMA->CCR &= ~DMA_CCR_EN;
    }
}

//void DMA1_Channel3_IRQHandler(void) {
//    if (DMA_GetITStatus(DMA1_IT_TC3) == SET) {
//        DMA_Cmd(DMA1_Channel3, DISABLE);
//        DMA_ClearITPendingBit(DMA1_IT_TC3);
//    }
//}

//</editor-fold>

#include "core.h"

static u16 screen_width  = LCD_PIXEL_WIDTH,
    screen_height = LCD_PIXEL_HEIGHT;

//<editor-fold desc="Init commands">

static const uint8_t init_commands[] = {
        // Power control A
        6, LCD_POWERA, 0x39, 0x2C, 0x00, 0x34, 0x02,
        // Power control B
        //4, LCD_POWERB, 0x00, 0xC1, 0x30,
        4, LCD_POWERB, 0x00, 0xC9, 0x30,
        // Driver timing control A
        //4, LCD_DTCA, 0x85, 0x00, 0x78,
        4, LCD_DTCA, 0x85, 0x10, 0x7A,
        // Driver timing control B
        3, LCD_DTCB, 0x00, 0x00,
        // Power on sequence control
        5, LCD_POWER_SEQ, 0x64, 0x03, 0x12, 0x81,
        // Pump ratio control
        2, LCD_PRC, 0x20,
        // Power control 1
        //2, LCD_POWER1, 0x23,
        2, LCD_POWER1, 0x1B,
        // Power control 2
        //2, LCD_POWER2, 0x10,
        2, LCD_POWER2, 0x00,
        // VCOM control 1
        //3, LCD_VCOM1, 0x3E, 0x28,
        3, LCD_VCOM1, 0x30, 0x30,
        // VCOM cotnrol 2
        //2, LCD_VCOM2, 0x86,
        2, LCD_VCOM2, 0xB7,
        // Memory access control
        //2, LCD_MAC, 0x48,
        2, LCD_MAC, 0x08,
        // Pixel format set
        2, LCD_PIXEL_FORMAT, 0x55,
        // Frame rate control
        //3, LCD_FRMCTR1, 0x00, 0x18,
        3, LCD_FRMCTR1, 0x00, 0x1A,
        // Display function control
        //4, LCD_DFC, 0x08, 0x82, 0x27,
        3, LCD_DFC, 0x0A, 0xA2,
        // 3Gamma function disable
        2, LCD_3GAMMA_EN, 0x00,
        // Gamma curve selected
        2, LCD_GAMMA, 0x01,
        // Set positive gamma
        //16, LCD_PGAMMA, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
        16, LCD_PGAMMA, 0x0F, 0x2A, 0x28, 0x08, 0x0E, 0x08, 0x54, 0xA9, 0x43, 0x0A, 0x0F, 0x00, 0x00, 0x00, 0x00,
        //16, LCD_NGAMMA, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
        16, LCD_NGAMMA, 0x00, 0x15, 0x17, 0x07, 0x11, 0x06, 0x2B, 0x56, 0x3C, 0x05, 0x10, 0x0F, 0x3F, 0x3F, 0x0F,

        5, LCD_PAGE_ADDR, 0x00, 0x00, 0x01, 0x3f,
        2, LCD_COLUMN_ADDR, 0x00, 0x00, 0x00, 0xef,
        0
};

//</editor-fold>

//<editor-fold desc="LCD initialization functions">

static void LCD_pinsInit() {
//    SPI_InitTypeDef  spiStructure;
//    GPIO_InitTypeDef gpioStructure;
//
//    RCC_PCLK2Config(RCC_HCLK_Div2);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2ENR_AFIOEN, ENABLE);
//    RCC_APB2PeriphClockCmd(SPI_MASTER_GPIO_CLK | SPI_MASTER_CLK, ENABLE);

	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	//enable clock for SPI unit
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;


//set PINS mode as output
    GPIOB->MODER |= (GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER15_0);

//set SPI pins as alternate & output
    GPIOB->MODER |= (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4 |
    						GPIO_OSPEEDER_OSPEEDR5);

//set clock polarity and phase, set master
    GLCD_SPI->CR1 = SPI_CR1_MSTR + SPI_CR1_CPOL +
    						SPI_CR1_CPHA + SPI_CR1_SSM + SPI_CR1_SSI;
//set baud rate
    GLCD_SPI->CR1 |= (SPI_CR1_BR_0); // sck/ 2-6MHz 3-3Mhz
//enable DMA
    GLCD_SPI->CR2 |= (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);// | SPI_CR2_FRXTH);// | (SPI_CR2_DS_0 + SPI_CR2_DS_1));
//enable interface
    GLCD_SPI->CR1 |= SPI_CR1_SPE;
}

void LCD_reset() {
    TFT_RST_RESET;
    delay_ms(10);
    TFT_RST_SET;
    delay_ms(50);
}

void LCD_exitStandby() {
    dmaSendCmd(LCD_SLEEP_OUT);
    delay_ms(150);
    dmaSendCmd(LCD_DISPLAY_ON);
}

static void LCD_configure() {
    u8 count;
    u8 *address = (u8 *) init_commands;

    TFT_CS_RESET;
    while (1) {
        count = *(address++);
        if (count-- == 0) break;
        dmaSendCmdCont(*(address++));
        dmaSendDataCont8(address, count);
        address += count;
    }
    TFT_CS_SET;

    LCD_setOrientation(0xa8);
}

void LCD_init() {
    LCD_pinsInit();
    dmaInit();

    LCD_reset();
    LCD_configure();
    LCD_exitStandby();

    TFT_LED_SET;
}

//</editor-fold>

//<editor-fold desc="LCD common operations">

void LCD_setOrientation(u8 o) {
//    if (o == ORIENTATION_LANDSCAPE || o == ORIENTATION_LANDSCAPE_MIRROR) {
//        screen_height = LCD_PIXEL_WIDTH;
//        screen_width  = LCD_PIXEL_HEIGHT;
//    } else {
        screen_height = LCD_PIXEL_HEIGHT;
        screen_width  = LCD_PIXEL_WIDTH;
//    }
    TFT_CS_RESET;
    dmaSendCmdCont(LCD_MAC);
    dmaSendDataCont8(&o, 1);
    TFT_CS_SET;
}

void LCD_setBrightness(u8 bri) {
    TFT_CS_RESET;
    dmaSendCmdCont(LCD_WDB);
    dmaSendDataCont8(&bri, 1);
    TFT_CS_SET;
}

inline void LCD_setAddressWindow(u16 x1, u16 y1, u16 x2, u16 y2) {
    u16 pointData[2];

    TFT_CS_RESET;
    dmaSendCmdCont(LCD_COLUMN_ADDR);
    pointData[0] = x1;
    pointData[1] = x2;
    LCD_setSpi16();
    dmaSendDataCont16(pointData, 2);
    LCD_setSpi8();

    dmaSendCmdCont(LCD_PAGE_ADDR);
    pointData[0] = y1;
    pointData[1] = y2;
    LCD_setSpi16();
    dmaSendDataCont16(pointData, 2);
    LCD_setSpi8();
    TFT_CS_SET;
}

inline u16 LCD_getWidth() {
    return screen_width;
}

inline u16 LCD_getHeight() {
    return screen_height;
}

//</editor-fold>

//<editor-fold desc="SPI functions">

inline void LCD_setSpi8(void) {
    SPI_MASTER->CR1 &= ~SPI_CR1_SPE; // DISABLE SPI
    //SPI_MASTER->CR1 &= ~SPI_CR1_DFF; // SPI 8
    SPI_MASTER->CR2 &= ~SPI_CR2_DS; // SPI 8
    SPI_MASTER->CR2 |= (SPI_CR2_DS_0 | SPI_CR2_DS_1 | SPI_CR2_DS_2); // SPI 8
    SPI_MASTER->CR1 |= SPI_CR1_SPE;  // ENABLE SPI
}

inline void LCD_setSpi16(void) {
    SPI_MASTER->CR1 &= ~SPI_CR1_SPE; // DISABLE SPI
    //SPI_MASTER->CR1 |= SPI_CR1_DFF;  // SPI 16
    SPI_MASTER->CR2 |= SPI_CR2_DS; // SPI 16
    SPI_MASTER->CR1 |= SPI_CR1_SPE;  // ENABLE SPI
}

// </editor-fold>

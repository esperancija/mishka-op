#ifndef TEST1_CONFIG_H
#define TEST1_CONFIG_H

#define u8 uint8_t
#define u16 uint16_t
#define u32 uint32_t

#define s8 int8_t
#define s16 int16_t
#define s32 int32_t

// <editor-fold desc="Defaults">

#define TFT_LED_PIN     GPIO_Pin_14
#define TFT_RESET_PIN   GPIO_Pin_15
#define TFT_DC_PIN      GPIO_Pin_7
#define TFT_CS_PIN      GPIO_Pin_6

#define SPI_MASTER                   SPI1
#define SPI_MASTER_CLK               RCC_APB2Periph_SPI1
#define SPI_MASTER_GPIO              GPIOB
#define SPI_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOB
#define SPI_MASTER_PIN_NSS           GPIO_Pin_2
#define SPI_MASTER_PIN_SCK           GPIO_Pin_3
#define SPI_MASTER_PIN_MISO          GPIO_Pin_4
#define SPI_MASTER_PIN_MOSI          GPIO_Pin_5

// </editor-fold>

// <editor-fold desc="Colors">

#define BLACK           0x0000      /*   0,   0,   0 */
#define NAVY            0x000F      /*   0,   0, 128 */
#define DGREEN          0x03E0      /*   0, 128,   0 */
#define DCYAN           0x03EF      /*   0, 128, 128 */
#define MAROON          0x7800      /* 128,   0,   0 */
#define PURPLE          0x780F      /* 128,   0, 128 */
#define OLIVE           0x7BE0      /* 128, 128,   0 */
#define LGRAY           0xC618      /* 192, 192, 192 */
#define DGRAY           0x7BEF      /* 128, 128, 128 */
#define DDGRAY          0x39E7      /* 64, 64, 64 */
#define BLUE            0x001F      /*   0,   0, 255 */
#define BLUE_G          0x01EF      /*   0,   128, 128 */
#define GREEN           0x07E0      /*   0, 255,   0 */
#define CYAN            0x07FF      /*   0, 255, 255 */
#define RED             0xF800      /* 255,   0,   0 */
#define LRED            0xF8EF      /* 255,   0,   0 */
#define MAGENTA         0xF81F      /* 255,   0, 255 */
#define YELLOW          0xFFE0      /* 255, 255,   0 */
#define WHITE           0xFFFF      /* 255, 255, 255 */
#define ORANGE          0xFD20      /* 255, 165,   0 */
#define GREENYELLOW     0xAFE5      /* 173, 255,  47 */
#define BROWN                 0XBC40 //
#define BRRED                 0XFC07 //


// </editor-fold>

#endif //TEST1_CONFIG_H

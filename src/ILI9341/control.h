#ifndef TEST2_CONTROL_H
#define TEST2_CONTROL_H

#include "config.h"
#include "commands.h"

// <editor-fold desc="Functions">
#define TFT_DC_SET      (GPIOB->BSRR = GPIO_BSRR_BS_7)//GPIO_SetBits(GPIOA, TFT_DC_PIN);
#define TFT_DC_RESET    (GPIOB->BSRR = GPIO_BSRR_BR_7)//GPIO_ResetBits(GPIOA, TFT_DC_PIN);

#define TFT_RST_SET     (GPIOB->BSRR = GPIO_BSRR_BS_15)//GPIO_SetBits(GPIOA, TFT_RESET_PIN);
#define TFT_RST_RESET   (GPIOB->BSRR = GPIO_BSRR_BR_15)//GPIO_ResetBits(GPIOA, TFT_RESET_PIN);

#define TFT_CS_SET      (GPIOB->BSRR = GPIO_BSRR_BS_6)//GPIO_SetBits(GPIOA, TFT_CS_PIN);
#define TFT_CS_RESET    (GPIOB->BSRR = GPIO_BSRR_BR_6)//GPIO_ResetBits(GPIOA, TFT_CS_PIN);

#define TFT_LED_SET      //(GPIOB->BSRR = GPIO_BSRR_BS_14)//GPIO_SetBits(GPIOA, TFT_LED_PIN);
#define TFT_LED_RESET    //(GPIOB->BSRR = GPIO_BSRR_BR_14)//GPIO_ResetBits(GPIOA, TFT_LED_PIN);
// </editor-fold>

#endif //TEST2_CONTROL_H

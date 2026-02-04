#ifndef __ST7735_H
#define __ST7735_H

#include "stm32f4xx_hal.h"

#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160
#define X_OFFSET 2
#define Y_OFFSET 1

#define BLACK   0x0000
#define RED    0x001F
#define BLUE     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

/*
 * LCD's instructions set.
 */
#define ST7735_SWRESET  0x01 // reset lcd controller
#define ST7735_SLPOUT   0x11 // wake from sleep mode
#define ST7735_COLMOD   0x3A // set color format
#define ST7735_MADCTL   0x36 // mem access control
#define ST7735_CASET    0x2A // set column address (X range)
#define ST7735_RASET    0x2B // set row address (Y range)
#define ST7735_RAMWR    0x2C // await incoming data
#define ST7735_DISPON   0x29 // turn display on

#define CS_LOW()   HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_SET)

#define DC_LOW()   HAL_GPIO_WritePin(ST7735_DC_PORT, ST7735_DC_PIN, GPIO_PIN_RESET)
#define DC_HIGH()  HAL_GPIO_WritePin(ST7735_DC_PORT, ST7735_DC_PIN, GPIO_PIN_SET)

#define RST_LOW()  HAL_GPIO_WritePin(ST7735_RST_PORT, ST7735_RST_PIN, GPIO_PIN_RESET)
#define RST_HIGH() HAL_GPIO_WritePin(ST7735_RST_PORT, ST7735_RST_PIN, GPIO_PIN_SET)

extern SPI_HandleTypeDef hspi1;

/*
 * Ports LCD<->F411E
 */
#define ST7735_CS_PORT   GPIOC
#define ST7735_CS_PIN    GPIO_PIN_4

#define ST7735_DC_PORT   GPIOC
#define ST7735_DC_PIN    GPIO_PIN_5

#define ST7735_RST_PORT  GPIOB
#define ST7735_RST_PIN   GPIO_PIN_0

void ST7735_Init(void);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_DrawLine(uint16_t x0, uint16_t y0,
                     uint16_t x1, uint16_t y1,
                     uint16_t color);
void ST7735_FillRect(uint16_t x, uint16_t y,
                     uint16_t w, uint16_t h,
                     uint16_t color);
void ST7735_DrawColumns(float* values,
					uint16_t values_size,
					uint8_t mode);
#endif

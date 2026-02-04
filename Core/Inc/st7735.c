#include "st7735.h"
#include <stdlib.h>

#define MAX_VAL_DB 90 // values to normalize to for column drawing depending on the mode
#define MAX_VAL_LIN 10

static void ST7735_WriteCommand(uint8_t cmd)
{
    DC_LOW();
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    CS_HIGH();
}

static void ST7735_WriteData(uint8_t *buff, size_t buff_size)
{
    DC_HIGH();
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, buff, buff_size, HAL_MAX_DELAY);
    CS_HIGH();
}

static void ST7735_Reset(void)
{
    RST_LOW();
    HAL_Delay(10);
    RST_HIGH();
    HAL_Delay(10);
}

static void ST7735_SetAddressWindow(uint16_t x0, uint16_t y0,
                                    uint16_t x1, uint16_t y1)
{
	/*
	 * Setting up address window of LCD (where to draw and range of drawing).
	 */
    uint8_t data[4];

    ST7735_WriteCommand(ST7735_CASET);
    data[0] = 0;
    data[1] = x0 + X_OFFSET;
    data[2] = 0;
    data[3] = x1 + X_OFFSET;
    ST7735_WriteData(data, 4);

    ST7735_WriteCommand(ST7735_RASET);
    data[1] = y0 + Y_OFFSET;
    data[3] = y1 + Y_OFFSET;
    ST7735_WriteData(data, 4);

    ST7735_WriteCommand(ST7735_RAMWR);
}


void ST7735_Init(void)
{
	/*
	 * Initialize LCD.
	 */
    ST7735_Reset();

    ST7735_WriteCommand(ST7735_SWRESET);
    HAL_Delay(150);

    ST7735_WriteCommand(ST7735_SLPOUT);
    HAL_Delay(150);

    uint8_t colorMode = 0x05; // RGB565
    ST7735_WriteCommand(ST7735_COLMOD);
    ST7735_WriteData(&colorMode, 1);

    uint8_t madctl = 0xC8; // orientation
    ST7735_WriteCommand(ST7735_MADCTL);
    ST7735_WriteData(&madctl, 1);

    ST7735_WriteCommand(ST7735_DISPON);
    HAL_Delay(100);

    ST7735_FillScreen(BLACK);
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
	/*
	 * Used to draw 1 pixel of LCD.
	 */
    if (x >= ST7735_WIDTH || y >= ST7735_HEIGHT) return;

    uint8_t data[2] = { color >> 8, color & 0xFF };

    ST7735_SetAddressWindow(x, y, x, y);
    ST7735_WriteData(data, 2);
}

void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRect(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_FillRect(uint16_t x, uint16_t y,
                     uint16_t w, uint16_t h,
                     uint16_t color)
{
	/*
	 * Function drawing a rectangle with (x,y) corner, width w, height h.
	 */
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;

    if ((x + w - 1) >= ST7735_WIDTH)  w = ST7735_WIDTH  - x;
    if ((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

    ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);

    uint8_t data[2] = { color >> 8, color & 0xFF };

    DC_HIGH();
    CS_LOW();
    for (uint32_t i = 0; i < w * h; i++)
        HAL_SPI_Transmit(&hspi1, data, 2, HAL_MAX_DELAY);
    CS_HIGH();
}

void ST7735_DrawLine(uint16_t x0, uint16_t y0,
                     uint16_t x1, uint16_t y1,
                     uint16_t color)
{
	/*
	 * Function drawing a line from A(x0,y0) to B(x1,y1) of color.
	 */
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (1)
    {
        ST7735_DrawPixel(x0, y0, color);
        if (x0 == x1 && y0 == y1) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

void ST7735_DrawScale_vertical(){
	//to_do
}
void ST7735_DrawScale_horizontal(){
	//to_do
}

void ST7735_DrawColumns(float* values, uint16_t values_size, uint8_t modeDb)
{
	/*
	 * Function drawing values_size columns on LCD of heights normalized to maximum of maxVal.
	 */
	float maxVal = modeDb ? MAX_VAL_DB : MAX_VAL_LIN;
    if (values_size == 0) return;
    uint16_t h = ST7735_HEIGHT / values_size;
    if (h == 0) h = 1;
    //ST7735_FillScreen(BLACK); //<- slower and worse-looking than double FillRect

    /*
    float maxVal = 100;
    for (uint16_t i = 1; i < values_size; i++) {
        if (values[i] > maxVal) {
            maxVal = values[i];
        }
    }
    */

    uint16_t x = 0;
    uint16_t y = 0;
    uint16_t w = 0;
    for (uint16_t i = 0; i < values_size; i++) {
    	if(values[i] < maxVal){
    		w = (uint16_t)(values[i] / maxVal * ((float)ST7735_WIDTH)/1.2);
    	}
    	else {
    		w = ST7735_WIDTH;
    	}
        x = ST7735_WIDTH - w;
        ST7735_FillRect(0, y, ST7735_WIDTH, h, BLACK);
        ST7735_FillRect(x, y, w, h, GREEN);
        y += h;
    }
}

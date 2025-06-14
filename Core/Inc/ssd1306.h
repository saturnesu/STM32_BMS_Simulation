#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

// I2C Port and Address
#define SSD1306_I2C_PORT        hi2c1
#define SSD1306_I2C_ADDR        (0x3C << 1)

// Display resolution
#define SSD1306_WIDTH           128
#define SSD1306_HEIGHT          64

// Color enumeration
typedef enum {
    Black = 0x00,
    White = 0x01
} SSD1306_COLOR;

// Public function prototypes
void SSD1306_Init(I2C_HandleTypeDef* hi2c);
void SSD1306_Clear(void);
void SSD1306_UpdateScreen(void);
void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);
void SSD1306_WriteChar(char ch);
void SSD1306_WriteString(const char* str);
void SSD1306_SetCursor(uint8_t x, uint8_t y);

#endif

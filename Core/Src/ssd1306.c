#include "ssd1306.h"
#include "font5x7.h"

static I2C_HandleTypeDef* i2cHandle;

static uint8_t buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static uint8_t cursorX = 0;
static uint8_t cursorY = 0;

static void SSD1306_WriteCommand(uint8_t command) {
    uint8_t data[2] = {0x00, command};
    HAL_I2C_Master_Transmit(i2cHandle, SSD1306_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}

void SSD1306_Init(I2C_HandleTypeDef* hi2c) {
    i2cHandle = hi2c;

    HAL_Delay(100);

    SSD1306_WriteCommand(0xAE); // Display off
    SSD1306_WriteCommand(0x20); // Set Memory Addressing Mode
    SSD1306_WriteCommand(0x00); // Horizontal addressing mode
    SSD1306_WriteCommand(0xB0); // Page Start Address
    SSD1306_WriteCommand(0xC8); // COM Output Scan Direction
    SSD1306_WriteCommand(0x00); // Set lower column
    SSD1306_WriteCommand(0x10); // Set higher column
    SSD1306_WriteCommand(0x40); // Start line
    SSD1306_WriteCommand(0x81); // Set contrast
    SSD1306_WriteCommand(0xFF);
    SSD1306_WriteCommand(0xA1); // Segment re-map
    SSD1306_WriteCommand(0xA6); // Normal display
    SSD1306_WriteCommand(0xA8); // Set multiplex ratio
    SSD1306_WriteCommand(0x3F);
    SSD1306_WriteCommand(0xA4); // Output follows RAM content
    SSD1306_WriteCommand(0xD3); // Display offset
    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(0xD5); // Display clock
    SSD1306_WriteCommand(0xF0);
    SSD1306_WriteCommand(0xD9); // Pre-charge
    SSD1306_WriteCommand(0x22);
    SSD1306_WriteCommand(0xDA); // COM pins
    SSD1306_WriteCommand(0x12);
    SSD1306_WriteCommand(0xDB); // VCOM detect
    SSD1306_WriteCommand(0x20);
    SSD1306_WriteCommand(0x8D); // Charge pump
    SSD1306_WriteCommand(0x14);
    SSD1306_WriteCommand(0xAF); // Display ON

    SSD1306_Clear();
    SSD1306_UpdateScreen();
}

void SSD1306_Clear(void) {
    memset(buffer, 0x00, sizeof(buffer));
}

void SSD1306_UpdateScreen(void) {
    for (uint8_t page = 0; page < 8; page++) {
        SSD1306_WriteCommand(0xB0 + page);
        SSD1306_WriteCommand(0x00);
        SSD1306_WriteCommand(0x10);
        uint8_t data[SSD1306_WIDTH + 1];
        data[0] = 0x40; // Co = 0, D/C# = 1
        memcpy(&data[1], &buffer[SSD1306_WIDTH * page], SSD1306_WIDTH);
        HAL_I2C_Master_Transmit(i2cHandle, SSD1306_I2C_ADDR, data, SSD1306_WIDTH + 1, HAL_MAX_DELAY);
    }
}

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    if (color)
        buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y % 8));
    else
        buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}

void SSD1306_SetCursor(uint8_t x, uint8_t y) {
    cursorX = x;
    cursorY = y;
}

void SSD1306_WriteChar(char ch) {
    if (ch < 32 || ch > 126) ch = '?';

    for (uint8_t i = 0; i < 5; i++) {
        uint8_t line = font5x7[ch - 32][i];
        for (uint8_t j = 0; j < 7; j++) {
            SSD1306_DrawPixel(cursorX + i, cursorY + j, (line >> j) & 0x01);
        }
    }

    cursorX += 6;
    if (cursorX + 5 >= SSD1306_WIDTH) {
        cursorX = 0;
        cursorY += 8;
    }
}

void SSD1306_WriteString(const char* str) {
    while (*str) {
        SSD1306_WriteChar(*str++);
    }
}

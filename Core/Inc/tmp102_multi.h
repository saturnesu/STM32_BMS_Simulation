#ifndef TMP102_MULTI_H
#define TMP102_MULTI_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

// TMP102 I2C addresses (shifted for HAL)
#define TMP102_ADDR_CELL1 (0x48 << 1) // = 0x90
#define TMP102_ADDR_CELL2 (0x49 << 1) // = 0x92
#define TMP102_ADDR_CELL3 (0x4A << 1) // = 0x94

// Responds Temperature in Celsius or -999.0f in fault
HAL_StatusTypeDef TMP102_ReadTemperature(I2C_HandleTypeDef* hi2c, uint8_t address, float* temperature_C);

#endif // TMP102_MULTI_H

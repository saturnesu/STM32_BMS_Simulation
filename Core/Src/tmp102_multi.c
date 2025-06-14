#include "tmp102_multi.h"
#include <stdio.h>

HAL_StatusTypeDef TMP102_ReadTemperature(I2C_HandleTypeDef* hi2c, uint8_t address, float* temperature_C)
{
    uint8_t temp_raw[2];

    if (HAL_I2C_Mem_Read(hi2c, address, 0x00, I2C_MEMADD_SIZE_8BIT, temp_raw, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    int16_t temp = ((temp_raw[0] << 4) | (temp_raw[1] >> 4));
    if (temp & 0x800) // negative temperature
        temp |= 0xF000;

    *temperature_C = temp * 0.0625f;
    return HAL_OK;
}

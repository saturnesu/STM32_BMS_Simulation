#include "max17043.h"

HAL_StatusTypeDef MAX17043_ReadSOC(I2C_HandleTypeDef *hi2c, float *soc) {
    uint8_t reg = 0x04;
    uint8_t data[2];

    if (HAL_I2C_Master_Transmit(hi2c, MAX17043_I2C_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(hi2c, MAX17043_I2C_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    *soc = data[0] + data[1] / 256.0f;
    return HAL_OK;
}

HAL_StatusTypeDef MAX17043_ReadVoltage(I2C_HandleTypeDef *hi2c, float *voltage_V) {
    uint8_t reg = 0x02;
    uint8_t data[2];

    if (HAL_I2C_Master_Transmit(hi2c, MAX17043_I2C_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    if (HAL_I2C_Master_Receive(hi2c, MAX17043_I2C_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    uint16_t raw = ((uint16_t)data[0] << 8) | data[1];
    raw >>= 4;  // Only top 12 bits are used

    *voltage_V = raw * 1.25f / 1000.0f; // in Volts
    return HAL_OK;
}

HAL_StatusTypeDef MAX17043_QuickStart(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3] = {0x06, 0x40, 0x00}; // Reg 0x06: Mode, QuickStart bit = 1
    return HAL_I2C_Master_Transmit(hi2c, MAX17043_I2C_ADDR, data, 3, HAL_MAX_DELAY);
}

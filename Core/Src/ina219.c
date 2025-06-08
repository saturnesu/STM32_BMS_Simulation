#include "ina219.h"
#include "stdio.h"

#define INA219_REG_BUS_VOLTAGE 0x02

HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c)
{
    // Doesn't need anything for basic init — default config
    return HAL_OK;
}

HAL_StatusTypeDef INA219_ReadVoltage(I2C_HandleTypeDef *hi2c, float *bus_voltage_V)
{
    uint8_t reg = INA219_REG_BUS_VOLTAGE;
    uint8_t data[2];

    if (HAL_I2C_Mem_Read(hi2c, INA219_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    uint16_t raw = (data[0] << 8) | data[1];
    raw >>= 3;  // 13-bit value. First 3 bits are not used.

    *bus_voltage_V = raw * 4.0f / 1000.0f; // 4mV per bit

    return HAL_OK;
}

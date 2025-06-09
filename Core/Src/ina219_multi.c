#include "ina219_multi.h"
#include "stdio.h"

#define INA219_REG_BUS_VOLTAGE  0x02
#define INA219_REG_CURRENT      0x04


HAL_StatusTypeDef INA219_ReadAll(I2C_HandleTypeDef *hi2c, uint8_t address, CellData *data)
{
    uint8_t buf[2];
    uint16_t raw;

    // Read voltage register
    if (HAL_I2C_Mem_Read(hi2c, address, INA219_REG_BUS_VOLTAGE, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    raw = (buf[0] << 8) | buf[1]; 			// Combine the 2 bytes into one 16-bit word
    raw >>= 3;								// The 3 LSB not used
    data->voltage_V = raw * 4.0f / 1000.0f; // Every bit is 4mV, so we divide it with 1000

    // Read current register
    if (HAL_I2C_Mem_Read(hi2c, address, INA219_REG_CURRENT, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    raw = (buf[0] << 8) | buf[1];				// Combine the 2 bytes into one 16-bit word
    data->current_mA = (float)((int16_t)raw);  	// The current is 16-bit signed, so we make it float in mA

    // Calculate power in Watts
    data->power_W = data->voltage_V * (data->current_mA / 1000.0f); // Conversion of mA to A

    return HAL_OK;
}

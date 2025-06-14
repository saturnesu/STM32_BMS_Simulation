#include "max17043.h"
#include "math.h"

// Registers
#define REG_VCELL     0x02
#define REG_SOC       0x04
#define REG_MODE      0x06
#define REG_VERSION   0x08
#define REG_CONFIG    0x0C
#define REG_COMMAND   0xFE

// TCA9548A address (default 0x70)
#define TCA9548A_ADDR         (0x70 << 1)

// Channel selection on TCA9548A
HAL_StatusTypeDef MAX17043_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel) {
    if (channel > 7) return HAL_ERROR;
    uint8_t data = 1 << channel;
    return HAL_I2C_Master_Transmit(hi2c, TCA9548A_ADDR, &data, 1, HAL_MAX_DELAY);
}

// Read battery voltage in Volts
HAL_StatusTypeDef MAX17043_ReadVoltage(I2C_HandleTypeDef *hi2c, float *voltage) {
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2c, MAX17043_ADDR, REG_VCELL, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    uint16_t raw = (buf[0] << 4) | (buf[1] >> 4);
    *voltage = raw * 1.25f / 1000.0f;  // Result in Volts

    return HAL_OK;
}

// Read battery SoC in percentage
HAL_StatusTypeDef MAX17043_ReadSOC(I2C_HandleTypeDef *hi2c, float *soc) {
    uint8_t buf[2];
    if (HAL_I2C_Mem_Read(hi2c, MAX17043_ADDR, REG_SOC, I2C_MEMADD_SIZE_8BIT, buf, 2, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    *soc = buf[0] + buf[1] / 256.0f;
    return HAL_OK;
}

// Issue a quick start (recalculates SOC)
HAL_StatusTypeDef MAX17043_QuickStart(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2] = {0x40, 0x00};
    return HAL_I2C_Mem_Write(hi2c, MAX17043_ADDR, REG_MODE, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

// Issue a power-on reset
HAL_StatusTypeDef MAX17043_Reset(I2C_HandleTypeDef *hi2c) {
    uint8_t data[2] = {0x54, 0x00};  // Reset command
    return HAL_I2C_Mem_Write(hi2c, MAX17043_ADDR, REG_COMMAND, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

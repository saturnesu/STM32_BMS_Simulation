#ifndef __MAX17043_H
#define __MAX17043_H

#include "stm32f4xx_hal.h"

// MAX17043 I2C address (fixed)
#define MAX17043_ADDR         (0x36 << 1)  // 7-bit left-shifted

// Function prototypes
HAL_StatusTypeDef MAX17043_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel);
HAL_StatusTypeDef MAX17043_ReadVoltage(I2C_HandleTypeDef *hi2c, float *voltage);
HAL_StatusTypeDef MAX17043_ReadSOC(I2C_HandleTypeDef *hi2c, float *soc);
HAL_StatusTypeDef MAX17043_QuickStart(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MAX17043_Reset(I2C_HandleTypeDef *hi2c);

#endif

#ifndef INC_MAX17043_H_
#define INC_MAX17043_H_

#include "stm32f4xx_hal.h"

#define MAX17043_I2C_ADDR (0x36 << 1)  // 7-bit left-shifted for STM32 HAL

HAL_StatusTypeDef MAX17043_ReadVoltage(I2C_HandleTypeDef *hi2c, float *voltage_V);
HAL_StatusTypeDef MAX17043_ReadSOC(I2C_HandleTypeDef *hi2c, float *soc);
HAL_StatusTypeDef MAX17043_QuickStart(I2C_HandleTypeDef *hi2c);

#endif /* INC_MAX17043_H_ */

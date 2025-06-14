#ifndef __TCA9548A_H
#define __TCA9548A_H

#include "stm32f4xx_hal.h"

#define TCA9548A_ADDR  (0x70 << 1)  // Default I2C address of multiplexer

HAL_StatusTypeDef TCA9548A_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel);

#endif

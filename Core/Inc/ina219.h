#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stm32f4xx_hal.h"

// I2C default address (0x40 << 1)
#define INA219_ADDRESS 0x80

HAL_StatusTypeDef INA219_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef INA219_ReadVoltage(I2C_HandleTypeDef *hi2c, float *bus_voltage_V);
HAL_StatusTypeDef INA219_ReadCurrent(I2C_HandleTypeDef *hi2c, float *current_mA);


#endif /* INC_INA219_H_ */

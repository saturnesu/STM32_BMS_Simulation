#ifndef INC_INA219_MULTI_H_
#define INC_INA219_MULTI_H_

#include "stm32f4xx_hal.h"

// INA219 I2C addresses (shifted for HAL)
#define INA219_ADDR_CELL1 (0x40 << 1)
#define INA219_ADDR_CELL2 (0x42 << 1)
#define INA219_ADDR_CELL3 (0x43 << 1)

// Define structure for 1 cell
typedef struct
{
    uint8_t address;     // I2C address of this INA219
    float voltage_V;     // Bus voltage in volts
    float current_mA;    // Current in milliamps
    float power_W;       // Computed power (V * A)
}
CellData;

HAL_StatusTypeDef INA219_ReadAll(I2C_HandleTypeDef *hi2c, uint8_t address, CellData *data);

#endif /* INC_INA219_MULTI_H_ */

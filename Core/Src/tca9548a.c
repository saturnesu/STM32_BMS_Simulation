#include "tca9548a.h"

HAL_StatusTypeDef TCA9548A_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel) {
    if (channel > 7) return HAL_ERROR;
    uint8_t data = 1 << channel;
    return HAL_I2C_Master_Transmit(hi2c, TCA9548A_ADDR, &data, 1, HAL_MAX_DELAY);
}

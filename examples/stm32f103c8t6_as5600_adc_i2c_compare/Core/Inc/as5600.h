#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define AS5600_ADDR          (0x36U << 1)
#define AS5600_REG_STATUS    0x0BU
#define AS5600_REG_RAW_ANGLE 0x0CU
#define AS5600_REG_ANGLE     0x0EU

#define AS5600_I2C_TIMEOUT_MS 100U

HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw);
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle);
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);
float AS5600_RawToDegree(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */

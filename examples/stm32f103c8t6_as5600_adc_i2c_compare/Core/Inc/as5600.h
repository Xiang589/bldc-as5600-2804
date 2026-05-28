#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* AS5600 的 7 位 I2C 地址为 0x36。
 * HAL I2C 接口要求传入左移 1 位后的地址，因此定义为 (0x36U << 1)。 */
#define AS5600_ADDR          (0x36U << 1)
/* 状态寄存器：可用于判断磁铁是否存在以及磁场强弱状态。 */
#define AS5600_REG_STATUS    0x0BU
#define AS5600_REG_AGC       0x1AU
#define AS5600_REG_MAGNITUDE 0x1BU
/* 原始角度寄存器：输出未经零点/最大角配置处理的 12 位角度值。 */
#define AS5600_REG_RAW_ANGLE 0x0CU
/* 角度寄存器：输出经过 AS5600 内部映射处理后的 12 位角度值。 */
#define AS5600_REG_ANGLE     0x0EU

/* HAL I2C 阻塞读取的超时时间，单位毫秒。 */
#define AS5600_I2C_TIMEOUT_MS 100U

#define AS5600_STATUS_MH     0x08U
#define AS5600_STATUS_ML     0x10U
#define AS5600_STATUS_MD     0x20U

/**
 * @brief  读取 AS5600 的 RAW_ANGLE 原始角度寄存器。
 * @param  hi2c I2C 句柄指针（例如 &hi2c1）。
 * @param  raw  输出参数，返回 12 位原始角度计数值。
 * @retval HAL 状态码（HAL_OK 表示读取成功）。
 * @note   raw 取值范围 0~4095，可进一步换算为 0~360 度。
 */
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw);
/**
 * @brief  读取 AS5600 的 ANGLE 角度寄存器（映射后角度）。
 * @param  hi2c  I2C 句柄指针（例如 &hi2c1）。
 * @param  angle 输出参数，返回 12 位映射后角度计数值。
 * @retval HAL 状态码（HAL_OK 表示读取成功）。
 * @note   与 RAW_ANGLE 的区别是该值可受芯片内部零点/量程配置影响。
 */
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle);
/**
 * @brief  读取 AS5600 状态寄存器 STATUS。
 * @param  hi2c   I2C 句柄指针（例如 &hi2c1）。
 * @param  status 输出参数，返回 8 位状态值。
 * @retval HAL 状态码（HAL_OK 表示读取成功）。
 * @note   本驱动不解析具体 bit，后续可在上层按需求扩展状态位解析。
 */
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);
HAL_StatusTypeDef AS5600_ReadAgc(I2C_HandleTypeDef *hi2c, uint8_t *agc);
HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *magnitude);
/**
 * @brief  将 12 位角度计数换算为角度值（度）。
 * @param  raw 原始计数，典型范围 0~4095。
 * @retval 对应角度，单位度，范围约 [0, 360)。
 * @note   换算公式：degree = raw / 4096 * 360。
 */
float AS5600_RawToDegree(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */

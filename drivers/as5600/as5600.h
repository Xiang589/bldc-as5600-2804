#ifndef AS5600_H
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* AS5600 的 7 位 I2C 地址是 0x36。
 * STM32 HAL 的 HAL_I2C_Mem_Read/Write 传入的是左移 1 位后的地址格式，
 * 所以这里写成 (0x36U << 1)。 */
#define AS5600_ADDR          (0x36U << 1)
/* 状态寄存器：可用于判断磁铁检测状态、磁场过强/过弱等。 */
#define AS5600_REG_STATUS    0x0BU
/* 原始角度寄存器：未经零点/量程配置映射处理的 12 位角度值。 */
#define AS5600_REG_RAW_ANGLE 0x0CU
/* 映射后角度寄存器：经过 AS5600 内部零点/最大角配置处理后的角度值。 */
#define AS5600_REG_ANGLE     0x0EU

/* HAL I2C 阻塞读的超时时间（毫秒）。 */
#define AS5600_I2C_TIMEOUT_MS 100U

/**
 * @brief  读取 AS5600 原始角度（RAW_ANGLE）。
 * @param  hi2c I2C 外设句柄（如 &hi2c1）。
 * @param  raw  输出参数，返回原始 12 位角度计数。
 * @retval HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT。
 * @note   raw 范围为 0~4095，对应机械角 0~360 度（不含 360）。
 */
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw);
/**
 * @brief  读取 AS5600 映射后角度（ANGLE）。
 * @param  hi2c  I2C 外设句柄（如 &hi2c1）。
 * @param  angle 输出参数，返回映射后的 12 位角度计数。
 * @retval HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT。
 * @note   angle 范围为 0~4095；与 RAW_ANGLE 的区别在于其经过芯片内部配置映射。
 */
HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle);
/**
 * @brief  读取 AS5600 状态寄存器（STATUS）。
 * @param  hi2c   I2C 外设句柄（如 &hi2c1）。
 * @param  status 输出参数，返回 8 位状态值。
 * @retval HAL_OK/HAL_ERROR/HAL_BUSY/HAL_TIMEOUT。
 * @note   当前驱动仅返回原始 status 值，位域解析可按项目需要在上层扩展。
 */
HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status);
/**
 * @brief  将 AS5600 12 位原始计数换算为角度（度）。
 * @param  raw 原始计数值，通常为 0~4095。
 * @retval 浮点角度值，单位：度，范围约为 [0, 360)。
 * @note   公式：degree = raw / 4096 * 360。
 */
float AS5600_RawToDegree(uint16_t raw);

#ifdef __cplusplus
}
#endif

#endif /* AS5600_H */

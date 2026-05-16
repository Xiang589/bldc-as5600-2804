#include "as5600.h"

/*
 * AS5600 STM32 HAL 驱动（核心读取逻辑）
 *
 * 本文件演示如何通过 HAL 的 I2C 内存读接口访问 AS5600 寄存器。
 * AS5600 输出角度为 12 位，一圈对应 0~4095。
 * 注意：I2C 外设初始化（GPIO、时钟、速率）由 CubeMX 生成代码完成，
 * 本驱动只负责寄存器读取与角度换算。
 */

/* 内部静态函数：读取 8 位寄存器。
 * 该函数仅在当前 .c 文件内使用，不对外暴露。典型用途是读取 STATUS。 */
static HAL_StatusTypeDef AS5600_ReadU8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data)
{
  /* 基本入参检查：防止句柄或输出指针为空导致非法访问。 */
  if ((hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  /* HAL_I2C_Mem_Read 参数说明：
   * hi2c: 使用哪个 I2C 外设句柄
   * AS5600_ADDR: 目标器件地址
   * reg: 目标寄存器地址
   * I2C_MEMADD_SIZE_8BIT: 寄存器地址宽度为 8 位
   * data: 接收缓冲区
   * 1U: 读取 1 字节
   * AS5600_I2C_TIMEOUT_MS: 阻塞等待超时时间（ms） */
  return HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1U, AS5600_I2C_TIMEOUT_MS);
}

/* 内部静态函数：读取 16 位寄存器。
 * 主要用于 RAW_ANGLE/ANGLE 这类 2 字节角度寄存器读取。 */
static HAL_StatusTypeDef AS5600_ReadU16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *data)
{
  uint8_t buf[2] = {0};
  HAL_StatusTypeDef ret;

  /* 空指针保护，避免后续通信或写回结果时发生异常。 */
  if ((hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  /* 角度寄存器由高低两个字节组成，因此读取长度是 2 字节。 */
  ret = HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2U, AS5600_I2C_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    /* 直接透传 HAL 返回值，方便上层区分通信成功/失败类型。 */
    return ret;
  }

  /* AS5600 角度寄存器格式：buf[0] 高字节，buf[1] 低字节。 */
  *data = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  /* 有效位只有低 12 位，屏蔽高 4 位以保证范围在 0~4095。 */
  *data &= 0x0FFFU;

  return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw)
{
  /* RAW_ANGLE：返回未经芯片内部零点/量程映射处理的原始角度计数（0~4095）。 */
  return AS5600_ReadU16(hi2c, AS5600_REG_RAW_ANGLE, raw);
}

HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle)
{
  /* ANGLE：返回经过 AS5600 内部配置映射后的角度计数。 */
  return AS5600_ReadU16(hi2c, AS5600_REG_ANGLE, angle);
}

HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
  /* STATUS 可用于判断磁铁状态（是否检测到、磁场过强/过弱等）。
   * 当前函数只返回原始状态字节，不做 bit 解析（可作为后续扩展点）。 */
  return AS5600_ReadU8(hi2c, AS5600_REG_STATUS, status);
}

float AS5600_RawToDegree(uint16_t raw)
{
  /* 12 位分辨率：一圈 4096 个计数。
   * 换算公式：degree = raw / 4096 * 360。
   * raw & 0x0FFFU 用于保证只使用有效低 12 位。 */
  return ((float)(raw & 0x0FFFU) * 360.0f) / 4096.0f;
}

#include "as5600.h"

/*
 * AS5600 STM32 HAL 驱动示例
 *
 * 目标：给初学者演示“如何通过 HAL I2C 读取磁编码器寄存器”。
 * 本驱动不负责 I2C 初始化，I2C1 的初始化由 CubeMX 生成的 MX_I2C1_Init() 完成。
 * AS5600 的角度是 12 位，一圈对应 0~4095 计数。
 */

/* 仅文件内可见的工具函数：读取 8 位寄存器（例如 STATUS）。 */
static HAL_StatusTypeDef AS5600_ReadU8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data)
{
  /* 参数检查：防止空指针导致运行时错误。 */
  if ((hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  /* 阻塞式寄存器读取：
   * hi2c: I2C 外设句柄
   * AS5600_ADDR: AS5600 设备地址
   * reg: 目标寄存器地址
   * I2C_MEMADD_SIZE_8BIT: 寄存器地址宽度为 8 位
   * data: 接收缓冲区
   * 1U: 读取 1 字节
   * AS5600_I2C_TIMEOUT_MS: 超时时间（毫秒） */
  return HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1U, AS5600_I2C_TIMEOUT_MS);
}

/* 仅文件内可见的工具函数：读取 16 位寄存器（角度寄存器）。 */
static HAL_StatusTypeDef AS5600_ReadU16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *data)
{
  uint8_t buf[2] = {0};
  HAL_StatusTypeDef ret;

  /* 先做防御式检查，避免传入无效指针。 */
  if ((hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  /* AS5600 角度寄存器由高字节+低字节组成，因此读取长度是 2。 */
  ret = HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2U, AS5600_I2C_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    /* 将 HAL 状态原样返回给上层，便于定位 I2C 通信问题。 */
    return ret;
  }

  /* 按“高字节在前”拼接 16 位数值：buf[0] 高字节，buf[1] 低字节。 */
  *data = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  /* AS5600 角度有效位是 12 位，掩码后得到 0~4095。 */
  *data &= 0x0FFFU;

  return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw)
{
  /* RAW_ANGLE：未经内部映射处理的原始角度数据。 */
  return AS5600_ReadU16(hi2c, AS5600_REG_RAW_ANGLE, raw);
}

HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle)
{
  /* ANGLE：经过芯片内部零点/量程配置映射后的角度数据。 */
  return AS5600_ReadU16(hi2c, AS5600_REG_ANGLE, angle);
}

HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
  /* STATUS 可反映磁铁状态，当前仅返回原始字节，后续可再增加 bit 解析。 */
  return AS5600_ReadU8(hi2c, AS5600_REG_STATUS, status);
}

HAL_StatusTypeDef AS5600_ReadAgc(I2C_HandleTypeDef *hi2c, uint8_t *agc)
{
  return AS5600_ReadU8(hi2c, AS5600_REG_AGC, agc);
}

HAL_StatusTypeDef AS5600_ReadMagnitude(I2C_HandleTypeDef *hi2c, uint16_t *magnitude)
{
  return AS5600_ReadU16(hi2c, AS5600_REG_MAGNITUDE, magnitude);
}

float AS5600_RawToDegree(uint16_t raw)
{
  /* 角度换算：12 位计数一圈共 4096 份 -> degree = raw / 4096 * 360。 */
  return ((float)(raw & 0x0FFFU) * 360.0f) / 4096.0f;
}

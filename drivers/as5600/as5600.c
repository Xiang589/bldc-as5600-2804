#include "as5600.h"

static HAL_StatusTypeDef AS5600_ReadU8(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *data)
{
  if ((hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1U, AS5600_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef AS5600_ReadU16(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *data)
{
  uint8_t buf[2] = {0};
  HAL_StatusTypeDef ret;

  if ((hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  ret = HAL_I2C_Mem_Read(hi2c, AS5600_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2U, AS5600_I2C_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  *data = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
  *data &= 0x0FFFU;

  return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *raw)
{
  return AS5600_ReadU16(hi2c, AS5600_REG_RAW_ANGLE, raw);
}

HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle)
{
  return AS5600_ReadU16(hi2c, AS5600_REG_ANGLE, angle);
}

HAL_StatusTypeDef AS5600_ReadStatus(I2C_HandleTypeDef *hi2c, uint8_t *status)
{
  return AS5600_ReadU8(hi2c, AS5600_REG_STATUS, status);
}

float AS5600_RawToDegree(uint16_t raw)
{
  return ((float)(raw & 0x0FFFU) * 360.0f) / 4096.0f;
}

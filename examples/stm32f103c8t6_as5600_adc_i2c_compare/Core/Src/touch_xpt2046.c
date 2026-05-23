#include "touch_xpt2046.h"

#include "lcd_ili9341.h"
#include "spi.h"
#include <stdio.h>

#define TOUCH_DEBUG_PRINT 0U
#define TOUCH_SPI_TIMEOUT_MS 100U
#define XPT_CMD_X 0xD0U
#define XPT_CMD_Y 0x90U

static TouchCalibration_t g_cal;

static HAL_StatusTypeDef Touch_Read12(uint8_t cmd, uint16_t *raw)
{
  HAL_StatusTypeDef ret;
  uint8_t tx[3] = {cmd, 0x00U, 0x00U};
  uint8_t rx[3] = {0};

  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  ret = LCD_WaitForReady(TOUCH_SPI_TIMEOUT_MS);
  if (ret != HAL_OK)
  {
    return ret;
  }

  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET);
  ret = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3U, TOUCH_SPI_TIMEOUT_MS);
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);

  if (ret != HAL_OK)
  {
    return ret;
  }

  *raw = (uint16_t)(((rx[1] << 8U) | rx[2]) >> 3U);
  return HAL_OK;
}

void Touch_LoadDefaultCalibration(void)
{
  g_cal.ax = 0;
  g_cal.bx = (int32_t)((239.0f / 3640.0f) * 65536.0f);
  g_cal.cx = (int32_t)(-260.0f * (239.0f / 3640.0f) * 65536.0f);
  g_cal.ay = (int32_t)(-(319.0f / 3600.0f) * 65536.0f);
  g_cal.by = 0;
  g_cal.cy = (int32_t)((319.0f + 250.0f * (319.0f / 3600.0f)) * 65536.0f);
}

void Touch_SetCalibration(const TouchCalibration_t *cal)
{
  if (cal != NULL)
  {
    g_cal = *cal;
  }
}

void Touch_GetCalibration(TouchCalibration_t *cal)
{
  if (cal != NULL)
  {
    *cal = g_cal;
  }
}

void Touch_Init(void)
{
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);
  Touch_LoadDefaultCalibration();
}

uint8_t Touch_IsPressed(void)
{
  return (HAL_GPIO_ReadPin(TP_PEN_GPIO_Port, TP_PEN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

HAL_StatusTypeDef Touch_ReadRawStatus(uint16_t *raw_x, uint16_t *raw_y)
{
  HAL_StatusTypeDef ret;
  uint16_t rx;
  uint16_t ry;

  if ((raw_x == NULL) || (raw_y == NULL))
  {
    return HAL_ERROR;
  }

  if (Touch_IsPressed() == 0U)
  {
    return HAL_BUSY;
  }

  ret = Touch_Read12(XPT_CMD_X, &rx);
  if (ret != HAL_OK)
  {
    return ret;
  }

  ret = Touch_Read12(XPT_CMD_Y, &ry);
  if (ret != HAL_OK)
  {
    return ret;
  }

  *raw_x = rx;
  *raw_y = ry;
  return HAL_OK;
}

uint8_t Touch_ReadRaw(uint16_t *raw_x, uint16_t *raw_y)
{
  return (Touch_ReadRawStatus(raw_x, raw_y) == HAL_OK) ? 1U : 0U;
}

uint8_t Touch_MapRawToPoint(uint16_t raw_x,
                            uint16_t raw_y,
                            const TouchCalibration_t *cal,
                            uint16_t *x,
                            uint16_t *y)
{
  const TouchCalibration_t *cc = (cal == NULL) ? &g_cal : cal;
  int64_t xq;
  int64_t yq;
  int32_t sx;
  int32_t sy;

  if ((x == NULL) || (y == NULL))
  {
    return 0U;
  }

  xq = (int64_t)cc->ax * raw_x + (int64_t)cc->bx * raw_y + (int64_t)cc->cx;
  yq = (int64_t)cc->ay * raw_x + (int64_t)cc->by * raw_y + (int64_t)cc->cy;
  sx = (int32_t)(xq >> 16);
  sy = (int32_t)(yq >> 16);

  if (sx < 0)
  {
    sx = 0;
  }
  if (sy < 0)
  {
    sy = 0;
  }
  if (sx > ((int32_t)LCD_WIDTH - 1))
  {
    sx = (int32_t)LCD_WIDTH - 1;
  }
  if (sy > ((int32_t)LCD_HEIGHT - 1))
  {
    sy = (int32_t)LCD_HEIGHT - 1;
  }

  *x = (uint16_t)sx;
  *y = (uint16_t)sy;
  return 1U;
}

HAL_StatusTypeDef Touch_ReadPointStatus(uint16_t *x, uint16_t *y)
{
  uint16_t rx;
  uint16_t ry;
  HAL_StatusTypeDef ret;

  if ((x == NULL) || (y == NULL))
  {
    return HAL_ERROR;
  }

  ret = Touch_ReadRawStatus(&rx, &ry);
  if (ret != HAL_OK)
  {
    return ret;
  }

  if (Touch_MapRawToPoint(rx, ry, &g_cal, x, y) == 0U)
  {
    return HAL_ERROR;
  }

#if TOUCH_DEBUG_PRINT
  printf("[TOUCH] raw_x=%u raw_y=%u x=%u y=%u\r\n", rx, ry, *x, *y);
#endif

  return HAL_OK;
}

uint8_t Touch_ReadPoint(uint16_t *x, uint16_t *y)
{
  return (Touch_ReadPointStatus(x, y) == HAL_OK) ? 1U : 0U;
}

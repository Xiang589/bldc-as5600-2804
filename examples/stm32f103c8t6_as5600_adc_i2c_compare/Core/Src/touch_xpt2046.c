#include "touch_xpt2046.h"

#include "lcd_ili9341.h"
#include "spi.h"
#include <stdio.h>

#define XPT_CMD_X 0xD0U
#define XPT_CMD_Y 0x90U

static TouchCalibration_t g_cal;

static uint16_t Touch_Read12(uint8_t cmd)
{
  uint8_t tx[3] = {cmd, 0x00U, 0x00U};
  uint8_t rx[3] = {0};
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3U, 100U);
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);
  return (uint16_t)(((rx[1] << 8U) | rx[2]) >> 3U);
}

void Touch_LoadDefaultCalibration(void)
{
  g_cal.ax = 0;
  g_cal.bx = (int32_t)((239.0f / 3640.0f) * 65536.0f);  /* raw_y -> x */
  g_cal.cx = (int32_t)(-260.0f * (239.0f / 3640.0f) * 65536.0f);
  g_cal.ay = (int32_t)(-(319.0f / 3600.0f) * 65536.0f); /* raw_x -> inverted y */
  g_cal.by = 0;
  g_cal.cy = (int32_t)((319.0f + 250.0f * (319.0f / 3600.0f)) * 65536.0f);
}

void Touch_SetCalibration(const TouchCalibration_t *cal) { if (cal) g_cal = *cal; }
void Touch_GetCalibration(TouchCalibration_t *cal) { if (cal) *cal = g_cal; }

void Touch_Init(void)
{
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);
  Touch_LoadDefaultCalibration();
}

uint8_t Touch_IsPressed(void)
{
  return (HAL_GPIO_ReadPin(TP_PEN_GPIO_Port, TP_PEN_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

uint8_t Touch_ReadRaw(uint16_t *raw_x, uint16_t *raw_y)
{
  if ((raw_x == NULL) || (raw_y == NULL) || (Touch_IsPressed() == 0U)) return 0U;
  *raw_x = Touch_Read12(XPT_CMD_X);
  *raw_y = Touch_Read12(XPT_CMD_Y);
  return 1U;
}

uint8_t Touch_MapRawToPoint(uint16_t raw_x, uint16_t raw_y, const TouchCalibration_t *cal, uint16_t *x, uint16_t *y)
{
  const TouchCalibration_t *cc = (cal == NULL) ? &g_cal : cal;
  int64_t xq, yq;
  int32_t sx, sy;
  if ((x == NULL) || (y == NULL)) return 0U;

  xq = (int64_t)cc->ax * raw_x + (int64_t)cc->bx * raw_y + (int64_t)cc->cx;
  yq = (int64_t)cc->ay * raw_x + (int64_t)cc->by * raw_y + (int64_t)cc->cy;
  sx = (int32_t)(xq >> 16);
  sy = (int32_t)(yq >> 16);

  if (sx < 0) sx = 0;
  if (sy < 0) sy = 0;
  if (sx > ((int32_t)LCD_WIDTH - 1)) sx = (int32_t)LCD_WIDTH - 1;
  if (sy > ((int32_t)LCD_HEIGHT - 1)) sy = (int32_t)LCD_HEIGHT - 1;

  *x = (uint16_t)sx;
  *y = (uint16_t)sy;
  return 1U;
}

uint8_t Touch_ReadPoint(uint16_t *x, uint16_t *y)
{
  uint16_t rx, ry;
  if ((x == NULL) || (y == NULL)) return 0U;
  if (Touch_ReadRaw(&rx, &ry) == 0U) return 0U;
  if (Touch_MapRawToPoint(rx, ry, &g_cal, x, y) == 0U) return 0U;
  printf("[TOUCH] raw_x=%u raw_y=%u x=%u y=%u\r\n", rx, ry, *x, *y);
  return 1U;
}

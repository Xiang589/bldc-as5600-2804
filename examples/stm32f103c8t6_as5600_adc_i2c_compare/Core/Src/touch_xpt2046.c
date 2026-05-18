#include "touch_xpt2046.h"

#include "lcd_ili9341.h"
#include "spi.h"
#include <stdio.h>

#define XPT_CMD_X 0xD0U
#define XPT_CMD_Y 0x90U

#define TOUCH_RAW_X_MIN_DEFAULT 250U
#define TOUCH_RAW_X_MAX_DEFAULT 3850U
#define TOUCH_RAW_Y_MIN_DEFAULT 260U
#define TOUCH_RAW_Y_MAX_DEFAULT 3900U
#define TOUCH_SWAP_XY_DEFAULT   1U
#define TOUCH_INVERT_X_DEFAULT  0U
#define TOUCH_INVERT_Y_DEFAULT  1U

static TouchCalibration_t g_cal;

static uint16_t Touch_Map(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_max)
{
  if (in_max <= in_min)
  {
    return 0U;
  }
  if (value < in_min) value = in_min;
  if (value > in_max) value = in_max;
  return (uint16_t)(((uint32_t)(value - in_min) * out_max) / (uint32_t)(in_max - in_min));
}

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
  g_cal.raw_x_min = TOUCH_RAW_X_MIN_DEFAULT;
  g_cal.raw_x_max = TOUCH_RAW_X_MAX_DEFAULT;
  g_cal.raw_y_min = TOUCH_RAW_Y_MIN_DEFAULT;
  g_cal.raw_y_max = TOUCH_RAW_Y_MAX_DEFAULT;
  g_cal.swap_xy = TOUCH_SWAP_XY_DEFAULT;
  g_cal.invert_x = TOUCH_INVERT_X_DEFAULT;
  g_cal.invert_y = TOUCH_INVERT_Y_DEFAULT;
}

void Touch_SetCalibration(const TouchCalibration_t *cal)
{
  if (cal == NULL)
  {
    return;
  }
  g_cal = *cal;
}

void Touch_GetCalibration(TouchCalibration_t *cal)
{
  if (cal == NULL)
  {
    return;
  }
  *cal = g_cal;
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

uint8_t Touch_ReadRaw(uint16_t *raw_x, uint16_t *raw_y)
{
  if ((raw_x == NULL) || (raw_y == NULL) || (Touch_IsPressed() == 0U)) return 0U;
  *raw_x = Touch_Read12(XPT_CMD_X);
  *raw_y = Touch_Read12(XPT_CMD_Y);
  return 1U;
}

uint8_t Touch_MapRawToPoint(uint16_t raw_x, uint16_t raw_y,
                            const TouchCalibration_t *cal,
                            uint16_t *x, uint16_t *y)
{
  uint16_t sx = 0U, sy = 0U;
  const TouchCalibration_t *cc = (cal == NULL) ? &g_cal : cal;
  if ((x == NULL) || (y == NULL)) return 0U;
  if (cc->swap_xy != 0U)
  {
    sx = Touch_Map(raw_y, cc->raw_y_min, cc->raw_y_max, LCD_WIDTH - 1U);
    sy = Touch_Map(raw_x, cc->raw_x_min, cc->raw_x_max, LCD_HEIGHT - 1U);
  }
  else
  {
    sx = Touch_Map(raw_x, cc->raw_x_min, cc->raw_x_max, LCD_WIDTH - 1U);
    sy = Touch_Map(raw_y, cc->raw_y_min, cc->raw_y_max, LCD_HEIGHT - 1U);
  }
  if (cc->invert_x != 0U) sx = (LCD_WIDTH - 1U) - sx;
  if (cc->invert_y != 0U) sy = (LCD_HEIGHT - 1U) - sy;
  *x = sx; *y = sy;
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

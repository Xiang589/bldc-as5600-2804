#include "touch_xpt2046.h"

#include "lcd_ili9341.h"
#include "spi.h"

#define XPT_CMD_X 0xD0U
#define XPT_CMD_Y 0x90U

#define TOUCH_RAW_X_MIN 250U
#define TOUCH_RAW_X_MAX 3850U
#define TOUCH_RAW_Y_MIN 260U
#define TOUCH_RAW_Y_MAX 3900U

#define TOUCH_SWAP_XY   1U
#define TOUCH_INVERT_X  0U
#define TOUCH_INVERT_Y  1U


static uint16_t Touch_Map(uint16_t value, uint16_t in_min, uint16_t in_max, uint16_t out_max)
{
  if (value < in_min) value = in_min;
  if (value > in_max) value = in_max;
  return (uint16_t)(((uint32_t)(value - in_min) * out_max) / (uint32_t)(in_max - in_min));
}

static uint16_t Touch_Read12(uint8_t cmd)
{
  uint8_t tx[3] = {cmd, 0x00U, 0x00U};
  uint8_t rx[3] = {0};
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 3U, 100U);
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);
  return (uint16_t)(((rx[1] << 8U) | rx[2]) >> 3U);
}

void Touch_Init(void)
{
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_SET);
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

uint8_t Touch_ReadPoint(uint16_t *x, uint16_t *y)
{
  uint16_t rx, ry;
  if ((x == NULL) || (y == NULL)) return 0U;
  if (Touch_ReadRaw(&rx, &ry) == 0U) return 0U;

  uint16_t sx = 0U;
  uint16_t sy = 0U;

#if TOUCH_SWAP_XY
  sx = Touch_Map(ry, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, LCD_WIDTH - 1U);
  sy = Touch_Map(rx, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, LCD_HEIGHT - 1U);
#else
  sx = Touch_Map(rx, TOUCH_RAW_X_MIN, TOUCH_RAW_X_MAX, LCD_WIDTH - 1U);
  sy = Touch_Map(ry, TOUCH_RAW_Y_MIN, TOUCH_RAW_Y_MAX, LCD_HEIGHT - 1U);
#endif
#if TOUCH_INVERT_X
  sx = (LCD_WIDTH - 1U) - sx;
#endif
#if TOUCH_INVERT_Y
  sy = (LCD_HEIGHT - 1U) - sy;
#endif

  *x = sx; *y = sy;
  return 1U;
}

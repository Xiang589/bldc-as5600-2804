#include "lcd_ili9341.h"

#include "spi.h"

#define ILI9341_SWRESET 0x01
#define ILI9341_SLPOUT  0x11
#define ILI9341_DISPON  0x29
#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

static void LCD_Select(uint8_t sel) { HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, sel ? GPIO_PIN_RESET : GPIO_PIN_SET); }
static void LCD_DC(uint8_t dc) { HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, dc ? GPIO_PIN_SET : GPIO_PIN_RESET); }

static void LCD_WriteCmd(uint8_t cmd) { LCD_DC(0U); LCD_Select(1U); HAL_SPI_Transmit(&hspi1, &cmd, 1U, 100U); LCD_Select(0U); }
static void LCD_WriteData(const uint8_t *data, uint16_t len) { LCD_DC(1U); LCD_Select(1U); HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 100U); LCD_Select(0U); }

static void LCD_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t data[4];
  LCD_WriteCmd(ILI9341_CASET);
  data[0] = (uint8_t)(x0 >> 8); data[1] = (uint8_t)x0; data[2] = (uint8_t)(x1 >> 8); data[3] = (uint8_t)x1;
  LCD_WriteData(data, 4U);

  LCD_WriteCmd(ILI9341_PASET);
  data[0] = (uint8_t)(y0 >> 8); data[1] = (uint8_t)y0; data[2] = (uint8_t)(y1 >> 8); data[3] = (uint8_t)y1;
  LCD_WriteData(data, 4U);

  LCD_WriteCmd(ILI9341_RAMWR);
}

static void LCD_WriteColorBurst(uint16_t color, uint32_t count)
{
  uint8_t buf[64];
  uint8_t hi = (uint8_t)(color >> 8);
  uint8_t lo = (uint8_t)color;
  for (uint32_t i = 0; i < sizeof(buf); i += 2U) { buf[i] = hi; buf[i + 1U] = lo; }

  LCD_DC(1U);
  LCD_Select(1U);
  while (count > 0U)
  {
    uint16_t px = (count > (sizeof(buf) / 2U)) ? (sizeof(buf) / 2U) : (uint16_t)count;
    HAL_SPI_Transmit(&hspi1, buf, px * 2U, 100U);
    count -= px;
  }
  LCD_Select(0U);
}

void LCD_Init(void)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);
  HAL_Delay(10U);
  HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);
  HAL_Delay(120U);

  LCD_WriteCmd(ILI9341_SWRESET); HAL_Delay(5U);
  LCD_WriteCmd(ILI9341_SLPOUT); HAL_Delay(120U);

  { uint8_t pixfmt = 0x55U; LCD_WriteCmd(ILI9341_PIXFMT); LCD_WriteData(&pixfmt, 1U); }
  { uint8_t madctl = 0x48U; LCD_WriteCmd(ILI9341_MADCTL); LCD_WriteData(&madctl, 1U); }

  LCD_WriteCmd(ILI9341_DISPON); HAL_Delay(20U);
}

void LCD_SetBacklight(uint8_t on) { HAL_GPIO_WritePin(LCD_BLK_GPIO_Port, LCD_BLK_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET); }

void LCD_FillScreen(uint16_t color) { LCD_FillRect(0U, 0U, LCD_WIDTH, LCD_HEIGHT, color); }

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if ((x >= LCD_WIDTH) || (y >= LCD_HEIGHT) || (w == 0U) || (h == 0U)) return;
  if ((x + w) > LCD_WIDTH) w = LCD_WIDTH - x;
  if ((y + h) > LCD_HEIGHT) h = LCD_HEIGHT - y;
  LCD_SetWindow(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U));
  LCD_WriteColorBurst(color, (uint32_t)w * h);
}

void LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  if ((w < 2U) || (h < 2U)) { LCD_FillRect(x, y, w, h, color); return; }
  LCD_FillRect(x, y, w, 1U, color);
  LCD_FillRect(x, (uint16_t)(y + h - 1U), w, 1U, color);
  LCD_FillRect(x, y, 1U, h, color);
  LCD_FillRect((uint16_t)(x + w - 1U), y, 1U, h, color);
}

static uint8_t FontRow(char c, uint8_t r)
{
  switch (c)
  {
    case 'A': { static const uint8_t g[7]={0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}; return g[r]; }
    case 'D': { static const uint8_t g[7]={0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}; return g[r]; }
    case 'M': { static const uint8_t g[7]={0x11,0x1B,0x15,0x15,0x11,0x11,0x11}; return g[r]; }
    case 'O': { static const uint8_t g[7]={0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[r]; }
    case 'R': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}; return g[r]; }
    case 'T': { static const uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x04}; return g[r]; }
    case 'S': { static const uint8_t g[7]={0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}; return g[r]; }
    case 'P': { static const uint8_t g[7]={0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}; return g[r]; }
    case 'U': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x11,0x0E}; return g[r]; }
    case 'Y': { static const uint8_t g[7]={0x11,0x11,0x0A,0x04,0x04,0x04,0x04}; return g[r]; }
    case 'N': { static const uint8_t g[7]={0x11,0x19,0x15,0x13,0x11,0x11,0x11}; return g[r]; }
    case 'E': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}; return g[r]; }
    case 'I': { static const uint8_t g[7]={0x1F,0x04,0x04,0x04,0x04,0x04,0x1F}; return g[r]; }
    case 'V': { static const uint8_t g[7]={0x11,0x11,0x11,0x11,0x11,0x0A,0x04}; return g[r]; }
    case 'L': { static const uint8_t g[7]={0x10,0x10,0x10,0x10,0x10,0x10,0x1F}; return g[r]; }
    case ':': { static const uint8_t g[7]={0x00,0x04,0x00,0x00,0x04,0x00,0x00}; return g[r]; }
    case '+': { static const uint8_t g[7]={0x00,0x04,0x04,0x1F,0x04,0x04,0x00}; return g[r]; }
    case '-': { static const uint8_t g[7]={0x00,0x00,0x00,0x1F,0x00,0x00,0x00}; return g[r]; }
    case '%': { static const uint8_t g[7]={0x19,0x19,0x02,0x04,0x08,0x13,0x13}; return g[r]; }
    case '0': { static const uint8_t g[7]={0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}; return g[r]; }
    case '1': { static const uint8_t g[7]={0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}; return g[r]; }
    case '2': { static const uint8_t g[7]={0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}; return g[r]; }
    case '3': { static const uint8_t g[7]={0x1E,0x01,0x01,0x0E,0x01,0x01,0x1E}; return g[r]; }
    case '4': { static const uint8_t g[7]={0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}; return g[r]; }
    case '5': { static const uint8_t g[7]={0x1F,0x10,0x10,0x1E,0x01,0x01,0x1E}; return g[r]; }
    case '6': { static const uint8_t g[7]={0x0E,0x10,0x10,0x1E,0x11,0x11,0x0E}; return g[r]; }
    case '7': { static const uint8_t g[7]={0x1F,0x01,0x02,0x04,0x08,0x08,0x08}; return g[r]; }
    case '8': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}; return g[r]; }
    case '9': { static const uint8_t g[7]={0x0E,0x11,0x11,0x0F,0x01,0x01,0x0E}; return g[r]; }
    default:  { static const uint8_t g[7]={0x00,0x00,0x00,0x00,0x00,0x00,0x00}; return g[r]; }
  }
}

static void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg)
{
  if ((c >= 'a') && (c <= 'z')) c = (char)(c - 'a' + 'A');
  for (uint8_t row = 0; row < 7U; ++row)
  {
    uint8_t bits = FontRow(c, row);
    for (uint8_t col = 0; col < 5U; ++col)
    {
      LCD_FillRect((uint16_t)(x + col), (uint16_t)(y + row), 1U, 1U, (bits & (1U << (4U - col))) ? color : bg);
    }
    LCD_FillRect((uint16_t)(x + 5U), (uint16_t)(y + row), 1U, 1U, bg);
  }
  LCD_FillRect(x, (uint16_t)(y + 7U), 6U, 1U, bg);
}

void LCD_DrawText(uint16_t x, uint16_t y, const char *text, uint16_t color, uint16_t bg_color)
{
  if (text == NULL) return;
  uint16_t cx = x;
  while (*text != '\0')
  {
    LCD_DrawChar(cx, y, *text, color, bg_color);
    cx = (uint16_t)(cx + 6U);
    ++text;
  }
}

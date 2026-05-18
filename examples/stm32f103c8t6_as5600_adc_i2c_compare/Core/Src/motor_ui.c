#include "motor_ui.h"

#include <stdio.h>

#include "as5600.h"
#include "i2c.h"
#include "lcd_ili9341.h"
#include "motor_control.h"
#include "touch_xpt2046.h"

#define C_BG    0x0000U
#define C_FG    0xFFFFU
#define C_BTN   0x07E0U
#define C_STOP  0xF800U

#define DUTY_STEP 0.05f

typedef struct { uint16_t x,y,w,h; const char *label; uint16_t color; } UiButton;

static UiButton g_btn_start = {20, 180, 90, 44, "START", C_BTN};
static UiButton g_btn_stop  = {130,180, 90, 44, "STOP",  C_STOP};
static UiButton g_btn_plus  = {20, 236, 90, 44, "DUTY+", C_BTN};
static UiButton g_btn_minus = {130,236, 90, 44, "DUTY-", C_BTN};

static uint32_t g_last_draw = 0U;
static uint8_t g_touch_latch = 0U;
static uint8_t g_touch_pen = 0U;
static uint8_t g_touch_has_xy = 0U;
static uint16_t g_touch_x = 0U;
static uint16_t g_touch_y = 0U;

static void Ui_DrawButton(const UiButton *b)
{
  LCD_FillRect(b->x, b->y, b->w, b->h, b->color);
  LCD_DrawRect(b->x, b->y, b->w, b->h, C_FG);
  LCD_DrawText((uint16_t)(b->x + 14U), (uint16_t)(b->y + 16U), b->label, C_FG, b->color);
}

static uint8_t Ui_Hit(const UiButton *b, uint16_t x, uint16_t y)
{ return (x >= b->x) && (x < (b->x + b->w)) && (y >= b->y) && (y < (b->y + b->h)); }

static void Ui_DrawStatus(void)
{
  char line[32];
  uint16_t raw = 0U;
  char touch_line[32];

  LCD_FillRect(10U, 34U, 220U, 132U, C_BG);
  LCD_DrawText(10U, 36U, "Motor:", C_FG, C_BG);
  const uint8_t running = MotorControl_IsRunning();
  LCD_DrawText(82U, 36U, running ? "RUN" : "STOP", running ? C_BTN : C_STOP, C_BG);

  snprintf(line, sizeof(line), "Duty: %u%%", (unsigned int)(MotorControl_GetDuty() * 100.0f));
  LCD_DrawText(10U, 60U, line, C_FG, C_BG);

  if (AS5600_ReadRawAngle(&hi2c1, &raw) == HAL_OK)
  {
    snprintf(line, sizeof(line), "AS5600: %u", raw);
  }
  else
  {
    snprintf(line, sizeof(line), "AS5600: ERR");
  }
  LCD_DrawText(10U, 84U, line, C_FG, C_BG);

  if (g_touch_pen == 0U)
  {
    snprintf(touch_line, sizeof(touch_line), "Touch: ---");
  }
  else if (g_touch_has_xy == 0U)
  {
    snprintf(touch_line, sizeof(touch_line), "Touch: PEN");
  }
  else
  {
    snprintf(touch_line, sizeof(touch_line), "Touch: %u,%u", g_touch_x, g_touch_y);
  }
  LCD_DrawText(10U, 108U, touch_line, C_FG, C_BG);
}

void MotorUi_Init(void)
{
  LCD_Init();
  LCD_SetBacklight(1U);
  Touch_Init();

  LCD_FillScreen(C_BG);
  LCD_DrawText(40U, 8U, "AS5600 MOTOR", C_FG, C_BG);
  LCD_DrawRect(4U, 4U, 232U, 24U, C_FG);

  Ui_DrawButton(&g_btn_start);
  Ui_DrawButton(&g_btn_stop);
  Ui_DrawButton(&g_btn_plus);
  Ui_DrawButton(&g_btn_minus);
  Ui_DrawStatus();
}

void MotorUi_Update(uint32_t now)
{
  uint16_t x, y;
  (void)now;

  if (Touch_IsPressed() != 0U)
  {
    g_touch_pen = 1U;
    if (g_touch_latch == 0U)
    {
      printf("[TOUCH] PEN low\r\n");
      if (Touch_ReadPoint(&x, &y) != 0U)
      {
        g_touch_has_xy = 1U;
        g_touch_x = x;
        g_touch_y = y;
        printf("[TOUCH] x=%u y=%u\r\n", x, y);
        if (Ui_Hit(&g_btn_start, x, y)) { printf("[TOUCH] START\r\n"); MotorControl_Start(); }
        else if (Ui_Hit(&g_btn_stop, x, y)) { printf("[TOUCH] STOP\r\n"); MotorControl_Stop(); }
        else if (Ui_Hit(&g_btn_plus, x, y)) { printf("[TOUCH] DUTY+\r\n"); MotorControl_SetDuty(MotorControl_GetDuty() + DUTY_STEP); }
        else if (Ui_Hit(&g_btn_minus, x, y)) { printf("[TOUCH] DUTY-\r\n"); MotorControl_SetDuty(MotorControl_GetDuty() - DUTY_STEP); }
      }
      else
      {
        g_touch_has_xy = 0U;
      }
      g_touch_latch = 1U;
    }
  }
  else
  {
    g_touch_latch = 0U;
    g_touch_pen = 0U;
    g_touch_has_xy = 0U;
  }

  if ((now - g_last_draw) >= 250U)
  {
    Ui_DrawStatus();
    g_last_draw = now;
  }
}

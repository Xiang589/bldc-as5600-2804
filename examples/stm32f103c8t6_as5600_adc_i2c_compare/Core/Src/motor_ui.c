#include "motor_ui.h"

#include <stdio.h>

#include "as5600.h"
#include "i2c.h"
#include "lcd_ili9341.h"
#include "motor_control.h"
#include "touch_xpt2046.h"

#define ENABLE_TOUCH_CALIBRATION 1U

#define C_BG    0x0000U
#define C_FG    0xFFFFU
#define C_BTN   0x07E0U
#define C_STOP  0xF800U

#define DUTY_STEP 0.05f

typedef struct { uint16_t x,y,w,h; const char *label; uint16_t color; } UiButton;
typedef struct { const char *name; uint16_t sx; uint16_t sy; uint16_t rx; uint16_t ry; } CalPoint;

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

#if ENABLE_TOUCH_CALIBRATION
static CalPoint g_cal_points[5] = {
  {"LT", 20U, 20U, 0U, 0U},
  {"RT", 220U, 20U, 0U, 0U},
  {"LB", 20U, 300U, 0U, 0U},
  {"RB", 220U, 300U, 0U, 0U},
  {"CT", 120U, 160U, 0U, 0U},
};
static uint8_t g_cal_index = 0U;
static uint8_t g_cal_done = 0U;
static uint8_t g_cal_wait_release = 0U;
#endif

static void Ui_DrawButton(const UiButton *b)
{
  LCD_FillRect(b->x, b->y, b->w, b->h, b->color);
  LCD_DrawRect(b->x, b->y, b->w, b->h, C_FG);
  LCD_DrawText((uint16_t)(b->x + 14U), (uint16_t)(b->y + 16U), b->label, C_FG, b->color);
}

static uint8_t Ui_Hit(const UiButton *b, uint16_t x, uint16_t y)
{ return (x >= b->x) && (x < (b->x + b->w)) && (y >= b->y) && (y < (b->y + b->h)); }

#if ENABLE_TOUCH_CALIBRATION
static void Cal_DrawCross(uint16_t x, uint16_t y)
{
  LCD_FillRect((uint16_t)(x - 10U), y, 21U, 1U, C_FG);
  LCD_FillRect(x, (uint16_t)(y - 10U), 1U, 21U, C_FG);
}

static void Cal_DrawPointScreen(uint8_t idx)
{
  char line[24];
  LCD_FillScreen(C_BG);
  snprintf(line, sizeof(line), "CAL %u/5", (unsigned int)(idx + 1U));
  LCD_DrawText(10U, 8U, line, C_FG, C_BG);
  LCD_DrawText(10U, 24U, "PRESS CROSS", C_FG, C_BG);
  Cal_DrawCross(g_cal_points[idx].sx, g_cal_points[idx].sy);
}

static void Cal_PrintSuggestion(void)
{
  const CalPoint *lt = &g_cal_points[0];
  const CalPoint *rt = &g_cal_points[1];
  const CalPoint *lb = &g_cal_points[2];
  const CalPoint *rb = &g_cal_points[3];

  uint8_t swap = 0U;
  uint8_t inv_x = 0U;
  uint8_t inv_y = 0U;

  uint32_t dx_from_rawx = (lt->rx > rt->rx) ? (lt->rx - rt->rx) : (rt->rx - lt->rx);
  uint32_t dx_from_rawy = (lt->ry > rt->ry) ? (lt->ry - rt->ry) : (rt->ry - lt->ry);
  swap = (dx_from_rawy > dx_from_rawx) ? 1U : 0U;

  if (swap == 0U)
  {
    inv_x = (rt->rx < lt->rx) ? 1U : 0U;
    inv_y = (lb->ry < lt->ry) ? 1U : 0U;
  }
  else
  {
    inv_x = (rt->ry < lt->ry) ? 1U : 0U;
    inv_y = (lb->rx < lt->rx) ? 1U : 0U;
  }

  uint16_t raw_x_min = (lt->rx + rt->rx) / 2U;
  uint16_t raw_x_max = (lb->rx + rb->rx) / 2U;
  uint16_t raw_y_min = (lt->ry + lb->ry) / 2U;
  uint16_t raw_y_max = (rt->ry + rb->ry) / 2U;

  if (raw_x_min > raw_x_max) { uint16_t t = raw_x_min; raw_x_min = raw_x_max; raw_x_max = t; }
  if (raw_y_min > raw_y_max) { uint16_t t = raw_y_min; raw_y_min = raw_y_max; raw_y_max = t; }

  printf("#define TOUCH_SWAP_XY   %uU\r\n", swap);
  printf("#define TOUCH_INVERT_X  %uU\r\n", inv_x);
  printf("#define TOUCH_INVERT_Y  %uU\r\n", inv_y);
  printf("#define TOUCH_RAW_X_MIN %uU\r\n", raw_x_min);
  printf("#define TOUCH_RAW_X_MAX %uU\r\n", raw_x_max);
  printf("#define TOUCH_RAW_Y_MIN %uU\r\n", raw_y_min);
  printf("#define TOUCH_RAW_Y_MAX %uU\r\n", raw_y_max);
}
#endif

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

#if ENABLE_TOUCH_CALIBRATION
  g_cal_index = 0U;
  g_cal_done = 0U;
  g_cal_wait_release = 0U;
  Cal_DrawPointScreen(g_cal_index);
#else
  LCD_FillScreen(C_BG);
  LCD_DrawText(40U, 8U, "AS5600 MOTOR", C_FG, C_BG);
  LCD_DrawRect(4U, 4U, 232U, 24U, C_FG);

  Ui_DrawButton(&g_btn_start);
  Ui_DrawButton(&g_btn_stop);
  Ui_DrawButton(&g_btn_plus);
  Ui_DrawButton(&g_btn_minus);
  Ui_DrawStatus();
#endif
}

void MotorUi_Update(uint32_t now)
{
  uint16_t x, y;
  (void)now;

#if ENABLE_TOUCH_CALIBRATION
  if (g_cal_done != 0U)
  {
    return;
  }

  if (g_cal_wait_release != 0U)
  {
    if (Touch_IsPressed() == 0U)
    {
      g_cal_wait_release = 0U;
      g_cal_index++;
      if (g_cal_index >= 5U)
      {
        g_cal_done = 1U;
        LCD_FillScreen(C_BG);
        LCD_DrawText(10U, 8U, "CAL DONE", C_FG, C_BG);
        printf("[CAL] DONE\r\n");
        for (uint8_t i = 0U; i < 5U; ++i)
        {
          printf("[CAL] %s raw_x=%u raw_y=%u\r\n", g_cal_points[i].name, g_cal_points[i].rx, g_cal_points[i].ry);
        }
        Cal_PrintSuggestion();
      }
      else
      {
        Cal_DrawPointScreen(g_cal_index);
      }
    }
    return;
  }

  if (Touch_IsPressed() != 0U)
  {
    uint32_t sum_x = 0U, sum_y = 0U;
    uint16_t rx = 0U, ry = 0U;
    uint8_t valid = 0U;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
      if (Touch_ReadRaw(&rx, &ry) != 0U)
      {
        sum_x += rx;
        sum_y += ry;
        valid++;
      }
      HAL_Delay(5U);
    }
    if (valid > 0U)
    {
      g_cal_points[g_cal_index].rx = (uint16_t)(sum_x / valid);
      g_cal_points[g_cal_index].ry = (uint16_t)(sum_y / valid);
      printf("[CAL] %s screen_x=%u screen_y=%u raw_x=%u raw_y=%u\r\n",
             g_cal_points[g_cal_index].name,
             g_cal_points[g_cal_index].sx,
             g_cal_points[g_cal_index].sy,
             g_cal_points[g_cal_index].rx,
             g_cal_points[g_cal_index].ry);
      g_cal_wait_release = 1U;
    }
  }
  return;
#else
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
#endif
}

#include "motor_ui.h"

#include <stdio.h>

#include "as5600.h"
#include "i2c.h"
#include "lcd_ili9341.h"
#include "motor_driver.h"
#include "touch_xpt2046.h"

#define C_BG    0x0000U
#define C_FG    0xFFFFU
#define C_BTN   0x07E0U
#define C_STOP  0xF800U

#define DUTY_MIN 0.10f
#define DUTY_MAX 0.60f
#define DUTY_STEP 0.05f

typedef struct { uint16_t x,y,w,h; const char *label; uint16_t color; } UiButton;

static UiButton g_btn_start = {20, 180, 90, 44, "START", C_BTN};
static UiButton g_btn_stop  = {130,180, 90, 44, "STOP",  C_STOP};
static UiButton g_btn_plus  = {20, 236, 90, 44, "DUTY+", C_BTN};
static UiButton g_btn_minus = {130,236, 90, 44, "DUTY-", C_BTN};

static uint8_t g_running = 0U;
static float g_duty = 0.20f;
static uint32_t g_last_draw = 0U;
static uint8_t g_touch_latch = 0U;
static uint8_t g_comm_step = 0U;
static uint32_t g_last_step_tick = 0U;

static void Ui_DrawButton(const UiButton *b)
{
  LCD_FillRect(b->x, b->y, b->w, b->h, b->color);
  LCD_DrawRect(b->x, b->y, b->w, b->h, C_FG);
  LCD_DrawText((uint16_t)(b->x + 14U), (uint16_t)(b->y + 16U), b->label, C_FG, b->color);
}

static uint8_t Ui_Hit(const UiButton *b, uint16_t x, uint16_t y)
{ return (x >= b->x) && (x < (b->x + b->w)) && (y >= b->y) && (y < (b->y + b->h)); }

static void Ui_ApplyRunPattern(void)
{
  const float d = g_duty;
  switch (g_comm_step)
  {
    case 0U: MotorDriver_SetPwmDuty(d, 0.0f, 0.0f); break;
    case 1U: MotorDriver_SetPwmDuty(0.0f, d, 0.0f); break;
    default: MotorDriver_SetPwmDuty(0.0f, 0.0f, d); break;
  }
  g_comm_step = (uint8_t)((g_comm_step + 1U) % 3U);
}

static void Ui_StopMotor(void)
{
  MotorDriver_SetAllPwmZero();
  MotorDriver_Disable();
  g_running = 0U;
}

static void Ui_StartMotor(void)
{
  g_comm_step = 0U;
  MotorDriver_SetAllPwmZero();
  MotorDriver_Enable();
  g_running = 1U;
  Ui_ApplyRunPattern();
  g_last_step_tick = HAL_GetTick();
}

static void Ui_DrawStatus(void)
{
  char line[32];
  uint16_t raw = 0U;

  LCD_FillRect(10U, 34U, 220U, 132U, C_BG);
  LCD_DrawText(10U, 36U, "Motor:", C_FG, C_BG);
  LCD_DrawText(82U, 36U, g_running ? "RUN" : "STOP", g_running ? C_BTN : C_STOP, C_BG);

  snprintf(line, sizeof(line), "Duty: %u%%", (unsigned int)(g_duty * 100.0f));
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
}

void MotorUi_Init(void)
{
  Ui_StopMotor();
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
  if ((g_running != 0U) && ((now - g_last_step_tick) >= 120U))
  {
    Ui_ApplyRunPattern();
    g_last_step_tick = now;
  }

  if (Touch_IsPressed() != 0U)
  {
    if ((g_touch_latch == 0U) && (Touch_ReadPoint(&x, &y) != 0U))
    {
      g_touch_latch = 1U;
      if (Ui_Hit(&g_btn_start, x, y)) Ui_StartMotor();
      else if (Ui_Hit(&g_btn_stop, x, y)) Ui_StopMotor();
      else if (Ui_Hit(&g_btn_plus, x, y)) { g_duty += DUTY_STEP; if (g_duty > DUTY_MAX) g_duty = DUTY_MAX; }
      else if (Ui_Hit(&g_btn_minus, x, y)) { g_duty -= DUTY_STEP; if (g_duty < DUTY_MIN) g_duty = DUTY_MIN; }
    }
  }
  else
  {
    g_touch_latch = 0U;
  }

  if ((now - g_last_draw) >= 250U)
  {
    Ui_DrawStatus();
    g_last_draw = now;
  }
}

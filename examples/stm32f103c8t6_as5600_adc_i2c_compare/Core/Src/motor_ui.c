#include "motor_ui.h"

#include <stdio.h>

#include "motor_feedback.h"
#include "lcd_ili9341.h"
#include "motor_control.h"
#include "touch_cal_storage.h"
#include "touch_xpt2046.h"

#define ENABLE_TOUCH_CALIBRATION 0U
#define TOUCH_DEBUG_PRINT 0U
#define C_BG 0x0000U
#define C_FG 0xFFFFU
#define C_BTN 0x07E0U
#define C_STOP 0xF800U
#define C_CAL 0x001FU
#define TOUCH_SAMPLE_PERIOD_MS 25U
#define UI_STATUS_PERIOD_MS 500U

typedef struct {
  uint16_t x;
  uint16_t y;
  uint16_t w;
  uint16_t h;
  const char *label;
  uint16_t color;
} UiButton;

typedef struct {
  const char *name;
  uint16_t sx;
  uint16_t sy;
  uint16_t rx;
  uint16_t ry;
} CalPoint;

typedef enum {
  UI_MODE_MAIN = 0,
  UI_MODE_SET = 1,
  UI_MODE_CAL_CONFIRM = 2,
  UI_MODE_CALIBRATION = 3,
  UI_MODE_CAL_DONE = 4
} UiMode_t;

static UiButton g_btn_start = {20, 176, 90, 48, "START", C_BTN};
static UiButton g_btn_stop = {130, 176, 90, 48, "STOP", C_STOP};
static UiButton g_btn_dir = {20, 244, 90, 48, "DIR", C_BTN};
static UiButton g_btn_set = {130, 244, 90, 48, "SET", C_BTN};
static UiButton g_btn_cal = {180, 6, 50, 24, "CAL", C_CAL};
static UiButton g_btn_back_top = {180, 6, 50, 24, "<", C_CAL};
static UiButton g_btn_spd_minus = {20, 120, 90, 48, "SPD-", C_BTN};
static UiButton g_btn_spd_plus = {130, 120, 90, 48, "SPD+", C_BTN};
static UiButton g_btn_duty_minus = {20, 190, 90, 48, "DUTY-", C_BTN};
static UiButton g_btn_duty_plus = {130, 190, 90, 48, "DUTY+", C_BTN};
static UiButton g_btn_mode = {20, 260, 90, 40, "MODE", C_BTN};
static UiButton g_btn_back = {130, 260, 90, 40, "BACK", C_BTN};
static UiButton g_btn_yes = {30, 180, 80, 44, "YES", C_BTN};
static UiButton g_btn_no = {130, 180, 80, 44, "NO", C_STOP};

static UiMode_t g_ui_mode = UI_MODE_MAIN;
static uint32_t g_last_draw = 0U;
static uint32_t g_mode_tick = 0U;
static uint32_t g_touch_sample_tick = 0U;
static uint8_t g_touch_latch = 0U;
static uint8_t g_touch_pen = 0U;
static uint8_t g_touch_has_xy = 0U;
static uint8_t g_cal_wait_release = 0U;
static uint8_t g_wait_release_before_cal = 0U;
static uint8_t g_has_flash_cal = 0U;
static uint8_t g_last_hit_btn = 0U;
static uint8_t g_hit_stable_count = 0U;
static uint16_t g_touch_x = 0U;
static uint16_t g_touch_y = 0U;
static TouchCalibration_t g_prev_cal;
static CalPoint g_cal_points[5] = {
  {"LT", 20, 20, 0, 0},
  {"RT", 220, 20, 0, 0},
  {"LB", 20, 300, 0, 0},
  {"RB", 220, 300, 0, 0},
  {"CT", 120, 160, 0, 0},
};
static uint8_t g_cal_index = 0U;

static void Ui_DrawSetStatus(void);

static uint8_t Ui_Hit(const UiButton *b, uint16_t x, uint16_t y)
{
  return (x >= b->x) && (x < (b->x + b->w)) && (y >= b->y) && (y < (b->y + b->h));
}

static uint8_t Ui_ButtonId(uint16_t x, uint16_t y)
{
  if (g_ui_mode == UI_MODE_MAIN)
  {
    if (Ui_Hit(&g_btn_cal, x, y)) return 1U;
    if (Ui_Hit(&g_btn_start, x, y)) return 2U;
    if (Ui_Hit(&g_btn_stop, x, y)) return 3U;
    if (Ui_Hit(&g_btn_dir, x, y)) return 4U;
    if (Ui_Hit(&g_btn_set, x, y)) return 5U;
    return 0U;
  }

  if (g_ui_mode == UI_MODE_SET)
  {
    if (Ui_Hit(&g_btn_back_top, x, y)) return 11U;
    if (Ui_Hit(&g_btn_spd_minus, x, y)) return 12U;
    if (Ui_Hit(&g_btn_spd_plus, x, y)) return 13U;
    if (Ui_Hit(&g_btn_duty_minus, x, y)) return 14U;
    if (Ui_Hit(&g_btn_duty_plus, x, y)) return 15U;
    if (Ui_Hit(&g_btn_mode, x, y)) return 16U;
    if (Ui_Hit(&g_btn_back, x, y)) return 17U;
  }
  return 0U;
}

static void Ui_DrawButton(const UiButton *b)
{
  LCD_FillRect(b->x, b->y, b->w, b->h, b->color);
  LCD_DrawRect(b->x, b->y, b->w, b->h, C_FG);
  LCD_DrawText((uint16_t)(b->x + 10U), (uint16_t)(b->y + 6U), b->label, C_FG, b->color);
}

static void Cal_DrawCross(uint16_t x, uint16_t y)
{
  LCD_FillRect((uint16_t)(x - 10U), y, 21U, 1U, C_FG);
  LCD_FillRect(x, (uint16_t)(y - 10U), 1U, 21U, C_FG);
}

static void Cal_DrawPointScreen(uint8_t i)
{
  char line[24];
  LCD_FillScreen(C_BG);
  snprintf(line, sizeof(line), "CAL %u/5", (unsigned int)(i + 1U));
  LCD_DrawText(10U, 8U, line, C_FG, C_BG);
  LCD_DrawText(10U, 24U, "PRESS CROSS", C_FG, C_BG);
  Cal_DrawCross(g_cal_points[i].sx, g_cal_points[i].sy);
}

static void Ui_DrawMainScreen(void)
{
  LCD_FillScreen(C_BG);
  LCD_DrawText(10U, 8U, "AS5600 OPEN LOOP", C_FG, C_BG);
  LCD_DrawRect(4U, 4U, 232U, 24U, C_FG);
  Ui_DrawButton(&g_btn_cal);
  Ui_DrawButton(&g_btn_start);
  Ui_DrawButton(&g_btn_stop);
  Ui_DrawButton(&g_btn_dir);
  Ui_DrawButton(&g_btn_set);
}

static void Ui_DrawSetScreen(void)
{
  LCD_FillScreen(C_BG);
  LCD_DrawText(10U, 8U, "OPEN LOOP SET", C_FG, C_BG);
  LCD_DrawRect(4U, 4U, 232U, 24U, C_FG);
  Ui_DrawButton(&g_btn_back_top);
  Ui_DrawSetStatus();
  Ui_DrawButton(&g_btn_spd_minus);
  Ui_DrawButton(&g_btn_spd_plus);
  Ui_DrawButton(&g_btn_duty_minus);
  Ui_DrawButton(&g_btn_duty_plus);
  Ui_DrawButton(&g_btn_mode);
  Ui_DrawButton(&g_btn_back);
}

static void Ui_DrawSetStatus(void)
{
  char line[32];

  LCD_FillRect(10U, 40U, 220U, 60U, C_BG);

  if (MotorControl_GetMode() == MOTOR_MODE_SPEED_CLOSED_LOOP)
  {
    int32_t tr = MotorControl_GetTargetRpmX10();
    snprintf(line, sizeof(line), "Tgt: %ld.%ldrpm", (long)(tr / 10), (long)(tr % 10));
  }
  else
  {
    snprintf(line, sizeof(line), "Spd: L%u %ums/%ums",
             (unsigned int)MotorControl_GetSpeedLevel(),
             (unsigned int)MotorControl_GetTargetPeriodMs(),
             (unsigned int)MotorControl_GetCurrentPeriodMs());
  }
  LCD_DrawText(10U, 48U, line, C_FG, C_BG);

  snprintf(line, sizeof(line), "Mode: %s",
           MotorControl_GetMode() == MOTOR_MODE_SPEED_CLOSED_LOOP ? "CLSPD" : "OPEN");
  LCD_DrawText(10U, 72U, line, C_FG, C_BG);

  snprintf(line, sizeof(line), "Duty: %u%%", (unsigned int)(MotorControl_GetDuty() * 100.0f));
  LCD_DrawText(10U, 96U, line, C_FG, C_BG);
}

static void Ui_DrawConfirmScreen(void)
{
  LCD_FillScreen(C_BG);
  LCD_DrawText(20U, 40U, "RECALIBRATE?", C_FG, C_BG);
  Ui_DrawButton(&g_btn_yes);
  Ui_DrawButton(&g_btn_no);
}

static void Ui_DrawStatus(void)
{
  char line[32];
  char touch_line[32];

  LCD_FillRect(10U, 34U, 220U, 132U, C_BG);
  LCD_DrawText(10U, 36U, "State:", C_FG, C_BG);
  LCD_DrawText(82U, 36U,
               MotorControl_IsRunning()
                 ? (MotorControl_GetMode() == MOTOR_MODE_SPEED_CLOSED_LOOP ? "RUN CL" : "RUN OPEN")
                 : (MotorControl_GetMode() == MOTOR_MODE_SPEED_CLOSED_LOOP ? "STOP CL" : "STOP OPEN"),
               MotorControl_IsRunning() ? C_BTN : C_STOP,
               C_BG);

  LCD_DrawText(10U, 60U, "Dir:", C_FG, C_BG);
  LCD_DrawText(82U, 60U, MotorControl_GetDirection() == MOTOR_DIR_FWD ? "FWD" : "REV", C_FG, C_BG);

  if (MotorControl_GetMode() == MOTOR_MODE_SPEED_CLOSED_LOOP)
  {
    int32_t tr = MotorControl_GetTargetRpmX10();
    snprintf(line, sizeof(line), "Tgt: %ld.%ldrpm", (long)(tr / 10), (long)(tr % 10));
  }
  else
  {
    snprintf(line, sizeof(line), "Spd: L%u %ums/%ums",
             (unsigned int)MotorControl_GetSpeedLevel(),
             (unsigned int)MotorControl_GetTargetPeriodMs(),
             (unsigned int)MotorControl_GetCurrentPeriodMs());
  }
  LCD_DrawText(10U, 84U, line, C_FG, C_BG);

  snprintf(line, sizeof(line), "Duty: %u%%", (unsigned int)(MotorControl_GetDuty() * 100.0f));
  LCD_DrawText(10U, 108U, line, C_FG, C_BG);

  if (MotorFeedback_IsAngleValid() != 0U)
  {
    int32_t angle_x100 = MotorFeedback_GetAngleX100();
    snprintf(line, sizeof(line), "Angle: %ld.%02ld",
             (long)(angle_x100 / 100), (long)(angle_x100 % 100));
  }
  else
  {
    snprintf(line, sizeof(line), "Angle: ERR");
  }
  LCD_DrawText(10U, 132U, line, C_FG, C_BG);

  if (MotorFeedback_IsSpeedValid() != 0U)
  {
    int32_t rpm_x10 = MotorFeedback_GetRpmX10();
    int32_t rpm_abs = rpm_x10;
    if (rpm_abs < 0) rpm_abs = -rpm_abs;
    snprintf(line, sizeof(line), "RPM: %s%ld.%ld",
             (rpm_x10 < 0) ? "-" : "",
             (long)(rpm_abs / 10),
             (long)(rpm_abs % 10));
  }
  else
  {
    snprintf(line, sizeof(line), "RPM: ERR");
  }
  LCD_DrawText(10U, 156U, line, C_FG, C_BG);

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
  if (g_ui_mode == UI_MODE_MAIN)
  {
    (void)touch_line;
  }
}

static TouchCalibration_t Cal_Build(void)
{
  TouchCalibration_t cal = {0};
  const CalPoint *lt = &g_cal_points[0];
  const CalPoint *rt = &g_cal_points[1];
  const CalPoint *lb = &g_cal_points[2];

  const float x0 = (float)lt->rx, y0 = (float)lt->ry;
  const float x1 = (float)rt->rx, y1 = (float)rt->ry;
  const float x2 = (float)lb->rx, y2 = (float)lb->ry;
  const float sx0 = 20.0f, sy0 = 20.0f;
  const float sx1 = 220.0f, sy1 = 20.0f;
  const float sx2 = 20.0f, sy2 = 300.0f;

  const float denom = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
  if ((denom > -1e-6f) && (denom < 1e-6f))
  {
    return cal;
  }

  cal.ax = (int32_t)((((sx1 - sx0) * (y2 - y0) - (sx2 - sx0) * (y1 - y0)) / denom) * 65536.0f);
  cal.bx = (int32_t)((((x1 - x0) * (sx2 - sx0) - (x2 - x0) * (sx1 - sx0)) / denom) * 65536.0f);
  cal.cx = (int32_t)((sx0 - ((float)cal.ax / 65536.0f) * x0 - ((float)cal.bx / 65536.0f) * y0) * 65536.0f);
  cal.ay = (int32_t)((((sy1 - sy0) * (y2 - y0) - (sy2 - sy0) * (y1 - y0)) / denom) * 65536.0f);
  cal.by = (int32_t)((((x1 - x0) * (sy2 - sy0) - (x2 - x0) * (sy1 - sy0)) / denom) * 65536.0f);
  cal.cy = (int32_t)((sy0 - ((float)cal.ay / 65536.0f) * x0 - ((float)cal.by / 65536.0f) * y0) * 65536.0f);
  return cal;
}

static uint8_t Cal_Validate(const TouchCalibration_t *cal)
{
  uint16_t rbx = 0U, rby = 0U, cx = 0U, cy = 0U;
  int32_t erbx, erby, ecx, ecy;

  if (cal == NULL)
  {
    return 0U;
  }

  if ((cal->ax == 0) && (cal->bx == 0) && (cal->ay == 0) && (cal->by == 0))
  {
    return 0U;
  }

  if (Touch_MapRawToPoint(g_cal_points[3].rx, g_cal_points[3].ry, cal, &rbx, &rby) == 0U)
  {
    return 0U;
  }
  if (Touch_MapRawToPoint(g_cal_points[4].rx, g_cal_points[4].ry, cal, &cx, &cy) == 0U)
  {
    return 0U;
  }

  erbx = (int32_t)rbx - 220;
  erby = (int32_t)rby - 300;
  ecx = (int32_t)cx - 120;
  ecy = (int32_t)cy - 160;
  if (erbx < 0) erbx = -erbx;
  if (erby < 0) erby = -erby;
  if (ecx < 0) ecx = -ecx;
  if (ecy < 0) ecy = -ecy;

  return ((erbx <= 40) && (erby <= 40) && (ecx <= 40) && (ecy <= 40)) ? 1U : 0U;
}

static void Ui_HandleConfirmTouch(uint32_t now)
{
  uint16_t x, y;
  (void)now;

  if (Touch_IsPressed() != 0U)
  {
    if ((g_touch_latch == 0U) && (Touch_ReadPoint(&x, &y) != 0U))
    {
      g_touch_latch = 1U;
      if (Ui_Hit(&g_btn_yes, x, y))
      {
        MotorControl_Stop();
        g_cal_index = 0U;
        g_cal_wait_release = 0U;
        g_wait_release_before_cal = 1U;
        g_ui_mode = UI_MODE_CALIBRATION;
        LCD_FillScreen(C_BG);
        LCD_DrawText(10U, 8U, "RELEASE TOUCH", C_FG, C_BG);
      }
      else if (Ui_Hit(&g_btn_no, x, y))
      {
        g_ui_mode = UI_MODE_MAIN;
        Ui_DrawMainScreen();
        Ui_DrawStatus();
      }
    }
  }
  else
  {
    g_touch_latch = 0U;
  }
}

static void Ui_HandleCalibration(uint32_t now)
{
  if (g_wait_release_before_cal != 0U)
  {
    if (Touch_IsPressed() == 0U)
    {
      g_wait_release_before_cal = 0U;
      Cal_DrawPointScreen(g_cal_index);
    }
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
        TouchCalibration_t cal = Cal_Build();
        if (Cal_Validate(&cal) != 0U)
        {
          TouchCalibration_t old;
          Touch_GetCalibration(&old);
          Touch_SetCalibration(&cal);
          MotorControl_Stop();
          if (TouchCalStorage_Save(&cal) != 0U)
          {
            g_prev_cal = cal;
            g_has_flash_cal = 1U;
            printf("[CAL] DONE\r\n[CAL] SAVED\r\n");
            LCD_FillScreen(C_BG);
            LCD_DrawText(10U, 8U, "CAL SAVED", C_FG, C_BG);
          }
          else
          {
            printf("[CAL] SAVE ERR\r\n");
            Touch_SetCalibration(&old);
            LCD_FillScreen(C_BG);
            LCD_DrawText(10U, 8U, "CAL SAVE ERR", C_FG, C_BG);
          }
        }
        else
        {
          printf("[CAL] INVALID\r\n");
          if (g_has_flash_cal != 0U)
          {
            Touch_SetCalibration(&g_prev_cal);
          }
          else
          {
            Touch_LoadDefaultCalibration();
          }
          LCD_FillScreen(C_BG);
          LCD_DrawText(10U, 8U, "CAL INVALID", C_FG, C_BG);
        }
        g_mode_tick = now;
        g_ui_mode = UI_MODE_CAL_DONE;
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
    uint32_t sx = 0U, sy = 0U;
    uint16_t rx = 0U, ry = 0U;
    uint8_t n = 0U;
    for (uint8_t i = 0U; i < 8U; i++)
    {
      if (Touch_ReadRaw(&rx, &ry) != 0U)
      {
        sx += rx;
        sy += ry;
        n++;
      }
      HAL_Delay(5U);
    }

    if (n != 0U)
    {
      g_cal_points[g_cal_index].rx = (uint16_t)(sx / n);
      g_cal_points[g_cal_index].ry = (uint16_t)(sy / n);
      printf("[CAL] %s screen_x=%u screen_y=%u raw_x=%u raw_y=%u\r\n",
             g_cal_points[g_cal_index].name,
             g_cal_points[g_cal_index].sx,
             g_cal_points[g_cal_index].sy,
             g_cal_points[g_cal_index].rx,
             g_cal_points[g_cal_index].ry);
      g_cal_wait_release = 1U;
    }
  }
}

static void Ui_HandleMainTouch(uint32_t now)
{
  uint16_t x, y;

  if (Touch_IsPressed() != 0U)
  {
    g_touch_pen = 1U;
#if TOUCH_DEBUG_PRINT
    if ((g_touch_latch == 0U) && (g_touch_pen == 0U))
    {
      printf("[TOUCH] PEN low\r\n");
    }
#endif

    if ((now - g_touch_sample_tick) >= TOUCH_SAMPLE_PERIOD_MS)
    {
      uint8_t hit;
      g_touch_sample_tick = now;
      if (Touch_ReadPoint(&x, &y) == 0U)
      {
        g_touch_has_xy = 0U;
        return;
      }

      g_touch_has_xy = 1U;
      g_touch_x = x;
      g_touch_y = y;
#if TOUCH_DEBUG_PRINT
      printf("[TOUCH] x=%u y=%u\r\n", x, y);
#endif

      hit = Ui_ButtonId(x, y);
      if (hit != 0U)
      {
        if (hit == g_last_hit_btn)
        {
          if (g_hit_stable_count < 255U)
          {
            g_hit_stable_count++;
          }
        }
        else
        {
          g_last_hit_btn = hit;
          g_hit_stable_count = 1U;
        }
      }
      else
      {
        g_last_hit_btn = 0U;
        g_hit_stable_count = 0U;
      }

      if ((g_touch_latch == 0U) && (g_hit_stable_count >= 2U))
      {
        g_touch_latch = 1U;
        if (hit == 1U)
        {
#if TOUCH_DEBUG_PRINT
          printf("[TOUCH] CAL\r\n");
#endif
          g_ui_mode = UI_MODE_CAL_CONFIRM;
          Ui_DrawConfirmScreen();
        }
        else if (hit == 2U)
        {
#if TOUCH_DEBUG_PRINT
          printf("[TOUCH] START\r\n");
#endif
          MotorControl_Start();
        }
        else if (hit == 3U)
        {
#if TOUCH_DEBUG_PRINT
          printf("[TOUCH] STOP\r\n");
#endif
          MotorControl_Stop();
        }
        else if (hit == 4U)
        {
          MotorControl_ToggleDirection();
        }
        else if (hit == 5U)
        {
          g_ui_mode = UI_MODE_SET;
          Ui_DrawSetScreen();
        }
      }
    }
  }
  else
  {
    g_touch_latch = 0U;
    g_touch_pen = 0U;
    g_touch_has_xy = 0U;
    g_last_hit_btn = 0U;
    g_hit_stable_count = 0U;
  }
}


static void Ui_HandleSetTouch(uint32_t now)
{
  uint16_t x, y;
  if (Touch_IsPressed() != 0U)
  {
    g_touch_pen = 1U;
    if ((now - g_touch_sample_tick) >= TOUCH_SAMPLE_PERIOD_MS)
    {
      g_touch_sample_tick = now;
      if (Touch_ReadPoint(&x, &y) == 0U)
      {
        g_touch_has_xy = 0U;
        return;
      }
      g_touch_has_xy = 1U;
      g_touch_x = x;
      g_touch_y = y;
      uint8_t hit = Ui_ButtonId(x, y);
      if ((g_touch_latch == 0U) && (hit != 0U))
      {
        g_touch_latch = 1U;
        if (hit == 11U || hit == 17U)
        {
          g_ui_mode = UI_MODE_MAIN;
          Ui_DrawMainScreen();
          Ui_DrawStatus();
        }
        else if (hit == 16U)
        {
          MotorControl_ToggleMode();
          Ui_DrawSetStatus();
        }
        else if (hit == 12U)
        {
          MotorControl_SpeedDown();
          Ui_DrawSetStatus();
        }
        else if (hit == 13U)
        {
          MotorControl_SpeedUp();
          Ui_DrawSetStatus();
        }
        else if (hit == 14U)
        {
          MotorControl_DutyDown();
          Ui_DrawSetStatus();
        }
        else if (hit == 15U)
        {
          MotorControl_DutyUp();
          Ui_DrawSetStatus();
        }
      }
    }
  }
  else
  {
    g_touch_latch = 0U;
    g_touch_pen = 0U;
    g_touch_has_xy = 0U;
  }
}

void MotorUi_Init(void)
{
  TouchCalibration_t cal;
  LCD_Init();
  LCD_SetBacklight(1U);
  Touch_Init();
  MotorFeedback_Init();

  if (TouchCalStorage_Load(&cal) != 0U)
  {
    Touch_SetCalibration(&cal);
    g_prev_cal = cal;
    g_has_flash_cal = 1U;
    printf("[CAL] loaded from flash\r\n");
    g_ui_mode = UI_MODE_MAIN;
    Ui_DrawMainScreen();
    Ui_DrawStatus();
  }
  else
  {
    Touch_LoadDefaultCalibration();
    printf("[CAL] use defaults\r\n");
    g_ui_mode = UI_MODE_CALIBRATION;
    g_cal_index = 0U;
    g_cal_wait_release = 0U;
    g_wait_release_before_cal = 1U;
    LCD_FillScreen(C_BG);
    LCD_DrawText(10U, 8U, "NO CAL IN FLASH", C_FG, C_BG);
    LCD_DrawText(10U, 24U, "RELEASE THEN CAL", C_FG, C_BG);
  }
}

void MotorUi_Update(uint32_t now)
{
  MotorFeedback_Update(now);

  if (g_ui_mode == UI_MODE_CAL_DONE)
  {
    if ((now - g_mode_tick) >= 1200U)
    {
      g_ui_mode = UI_MODE_MAIN;
      Ui_DrawMainScreen();
      Ui_DrawStatus();
    }
    return;
  }

  if (g_ui_mode == UI_MODE_CAL_CONFIRM)
  {
    Ui_HandleConfirmTouch(now);
    return;
  }

  if (g_ui_mode == UI_MODE_CALIBRATION)
  {
    Ui_HandleCalibration(now);
    return;
  }

  if (g_ui_mode == UI_MODE_SET)
  {
    Ui_HandleSetTouch(now);
  }
  else
  {
    Ui_HandleMainTouch(now);
  }
  if ((now - g_last_draw) >= UI_STATUS_PERIOD_MS)
  {
    if (g_ui_mode == UI_MODE_SET)
    {
      Ui_DrawSetStatus();
    }
    else
    {
      Ui_DrawStatus();
    }
    g_last_draw = now;
  }
}

#include "motor_feedback.h"

#include "as5600.h"
#include "i2c.h"

#define MOTOR_FEEDBACK_ANGLE_PERIOD_MS 20U
#define MOTOR_FEEDBACK_SPEED_PERIOD_MS 200U

static uint8_t g_angle_valid = 0U;
static uint8_t g_speed_valid = 0U;
static uint8_t g_has_prev = 0U;
static uint16_t g_raw_angle = 0U;
static int32_t g_angle_x100 = 0;
static int32_t g_rpm_x10 = 0;
static int32_t g_delta_raw = 0;
static int32_t g_total_raw_turns = 0;
static uint16_t g_prev_raw = 0U;
static uint32_t g_prev_tick = 0U;
static uint32_t g_last_angle_tick = 0U;

void MotorFeedback_Init(void)
{
  g_angle_valid = 0U;
  g_speed_valid = 0U;
  g_has_prev = 0U;
  g_raw_angle = 0U;
  g_angle_x100 = 0;
  g_rpm_x10 = 0;
  g_delta_raw = 0;
  g_total_raw_turns = 0;
  g_prev_raw = 0U;
  g_prev_tick = HAL_GetTick();
  g_last_angle_tick = g_prev_tick;
}

void MotorFeedback_Update(uint32_t now)
{
  uint16_t raw = 0U;

  if ((now - g_last_angle_tick) < MOTOR_FEEDBACK_ANGLE_PERIOD_MS)
  {
    return;
  }

  if (AS5600_ReadRawAngle(&hi2c1, &raw) != HAL_OK)
  {
    g_angle_valid = 0U;
    g_speed_valid = 0U;
    g_has_prev = 0U;
    g_prev_tick = now;
    return;
  }

  g_last_angle_tick = now;
  g_raw_angle = raw;
  g_angle_x100 = ((int32_t)raw * 36000) / 4096;
  g_angle_valid = 1U;

  if ((now - g_prev_tick) < MOTOR_FEEDBACK_SPEED_PERIOD_MS)
  {
    return;
  }

  if (g_has_prev == 0U)
  {
    g_prev_raw = raw;
    g_prev_tick = now;
    g_has_prev = 1U;
    g_speed_valid = 0U;
    g_delta_raw = 0;
    return;
  }

  {
    uint32_t dt_ms = now - g_prev_tick;
    int32_t delta_raw = (int32_t)raw - (int32_t)g_prev_raw;

    if (delta_raw > 2048)
    {
      delta_raw -= 4096;
    }
    else if (delta_raw < -2048)
    {
      delta_raw += 4096;
    }

    g_delta_raw = delta_raw;
    g_total_raw_turns += delta_raw;

    if (dt_ms > 0U)
    {
      int64_t rpm_x10 = ((int64_t)delta_raw * 600000LL) / (4096LL * (int64_t)dt_ms);
      g_rpm_x10 = (int32_t)rpm_x10;
      g_speed_valid = 1U;
    }
    else
    {
      g_speed_valid = 0U;
    }
  }

  g_prev_raw = raw;
  g_prev_tick = now;
}

uint8_t MotorFeedback_IsAngleValid(void) { return g_angle_valid; }
uint8_t MotorFeedback_IsSpeedValid(void) { return g_speed_valid; }
uint16_t MotorFeedback_GetRawAngle(void) { return g_raw_angle; }
int32_t MotorFeedback_GetAngleX100(void) { return g_angle_x100; }
int32_t MotorFeedback_GetRpmX10(void) { return g_rpm_x10; }
int32_t MotorFeedback_GetDeltaRaw(void) { return g_delta_raw; }
int32_t MotorFeedback_GetTotalRawTurns(void) { return g_total_raw_turns; }

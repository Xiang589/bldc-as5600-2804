#include "motor_feedback.h"

#include "as5600.h"
#include "i2c.h"

#define MOTOR_FEEDBACK_ANGLE_PERIOD_MS 20U
#define MOTOR_FEEDBACK_SPEED_PERIOD_MS 200U
#define MOTOR_FEEDBACK_I2C_RECOVERY_ERROR_THRESHOLD 5U
#define MOTOR_FEEDBACK_I2C_RECOVERY_PERIOD_MS 500U
#define MOTOR_FEEDBACK_I2C_RECOVERY_DELAY_MS 2U

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
static uint32_t g_last_ok_tick = 0U;
static uint32_t g_last_error_tick = 0U;
static uint32_t g_update_count = 0U;
static uint32_t g_error_count = 0U;
static uint32_t g_consecutive_error_count = 0U;
static uint32_t g_last_recovery_tick = 0U;
static HAL_StatusTypeDef g_last_hal_status = HAL_OK;
static MotorFeedbackSnapshot_t g_snapshot;

static uint32_t MotorFeedback_EnterCritical(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void MotorFeedback_ExitCritical(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static void MotorFeedback_UpdateSnapshot(void)
{
  g_snapshot.angle_valid = g_angle_valid;
  g_snapshot.speed_valid = g_speed_valid;
  g_snapshot.raw_angle = g_raw_angle;
  g_snapshot.angle_x100 = g_angle_x100;
  g_snapshot.rpm_x10 = g_rpm_x10;
  g_snapshot.delta_raw = g_delta_raw;
  g_snapshot.total_raw_turns = g_total_raw_turns;
  g_snapshot.last_ok_tick = g_last_ok_tick;
  g_snapshot.last_error_tick = g_last_error_tick;
  g_snapshot.update_count = g_update_count;
  g_snapshot.error_count = g_error_count;
  g_snapshot.consecutive_error_count = g_consecutive_error_count;
  g_snapshot.last_hal_status = g_last_hal_status;
}

static void MotorFeedback_TryRecoverI2c(uint32_t now)
{
  if (g_consecutive_error_count < MOTOR_FEEDBACK_I2C_RECOVERY_ERROR_THRESHOLD)
  {
    return;
  }

  if ((g_last_recovery_tick != 0U) &&
      ((now - g_last_recovery_tick) < MOTOR_FEEDBACK_I2C_RECOVERY_PERIOD_MS))
  {
    return;
  }

  g_last_recovery_tick = now;
  (void)HAL_I2C_DeInit(&hi2c1);
  HAL_Delay(MOTOR_FEEDBACK_I2C_RECOVERY_DELAY_MS);
  MX_I2C1_Init();
}

void MotorFeedback_Init(void)
{
  uint32_t primask;

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
  g_last_ok_tick = 0U;
  g_last_error_tick = 0U;
  g_update_count = 0U;
  g_error_count = 0U;
  g_consecutive_error_count = 0U;
  g_last_recovery_tick = 0U;
  g_last_hal_status = HAL_OK;

  primask = MotorFeedback_EnterCritical();
  MotorFeedback_UpdateSnapshot();
  MotorFeedback_ExitCritical(primask);
}

void MotorFeedback_Update(uint32_t now)
{
  uint16_t raw = 0U;
  HAL_StatusTypeDef status;

  if ((now - g_last_angle_tick) < MOTOR_FEEDBACK_ANGLE_PERIOD_MS)
  {
    return;
  }

  status = AS5600_ReadRawAngle(&hi2c1, &raw);
  if (status != HAL_OK)
  {
    uint32_t primask;

    primask = MotorFeedback_EnterCritical();
    g_angle_valid = 0U;
    g_speed_valid = 0U;
    g_has_prev = 0U;
    g_last_angle_tick = now;
    g_prev_tick = now;
    g_last_error_tick = now;
    g_error_count++;
    g_consecutive_error_count++;
    g_last_hal_status = status;
    MotorFeedback_UpdateSnapshot();
    MotorFeedback_ExitCritical(primask);
    MotorFeedback_TryRecoverI2c(now);
    return;
  }

  {
    uint8_t next_has_prev = g_has_prev;
    uint8_t next_speed_valid = g_speed_valid;
    uint16_t next_prev_raw = g_prev_raw;
    uint32_t next_prev_tick = g_prev_tick;
    int32_t next_rpm_x10 = g_rpm_x10;
    int32_t next_delta_raw = g_delta_raw;
    int32_t next_total_raw_turns = g_total_raw_turns;

    if (g_has_prev == 0U)
    {
      next_prev_raw = raw;
      next_prev_tick = now;
      next_has_prev = 1U;
      next_speed_valid = 0U;
      next_delta_raw = 0;
    }
    else if ((now - g_prev_tick) >= MOTOR_FEEDBACK_SPEED_PERIOD_MS)
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

      next_delta_raw = delta_raw;
      next_total_raw_turns += delta_raw;

      if (dt_ms > 0U)
      {
        int64_t rpm_x10 = ((int64_t)delta_raw * 600000LL) / (4096LL * (int64_t)dt_ms);
        next_rpm_x10 = (int32_t)rpm_x10;
        next_speed_valid = 1U;
      }
      else
      {
        next_speed_valid = 0U;
      }

      next_prev_raw = raw;
      next_prev_tick = now;
    }

    {
      uint32_t primask = MotorFeedback_EnterCritical();
      g_last_angle_tick = now;
      g_raw_angle = raw;
      g_angle_x100 = ((int32_t)raw * 36000) / 4096;
      g_angle_valid = 1U;
      g_speed_valid = next_speed_valid;
      g_has_prev = next_has_prev;
      g_rpm_x10 = next_rpm_x10;
      g_delta_raw = next_delta_raw;
      g_total_raw_turns = next_total_raw_turns;
      g_prev_raw = next_prev_raw;
      g_prev_tick = next_prev_tick;
      g_last_ok_tick = now;
      g_update_count++;
      g_consecutive_error_count = 0U;
      g_last_hal_status = HAL_OK;
      MotorFeedback_UpdateSnapshot();
      MotorFeedback_ExitCritical(primask);
    }
  }
}

void MotorFeedback_GetSnapshot(MotorFeedbackSnapshot_t *snapshot)
{
  uint32_t primask;

  if (snapshot == NULL)
  {
    return;
  }

  primask = MotorFeedback_EnterCritical();
  *snapshot = g_snapshot;
  MotorFeedback_ExitCritical(primask);
}

uint8_t MotorFeedback_IsAngleValid(void)
{
  MotorFeedbackSnapshot_t snapshot;
  MotorFeedback_GetSnapshot(&snapshot);
  return snapshot.angle_valid;
}

uint8_t MotorFeedback_IsSpeedValid(void)
{
  MotorFeedbackSnapshot_t snapshot;
  MotorFeedback_GetSnapshot(&snapshot);
  return snapshot.speed_valid;
}

uint16_t MotorFeedback_GetRawAngle(void)
{
  MotorFeedbackSnapshot_t snapshot;
  MotorFeedback_GetSnapshot(&snapshot);
  return snapshot.raw_angle;
}

int32_t MotorFeedback_GetAngleX100(void)
{
  MotorFeedbackSnapshot_t snapshot;
  MotorFeedback_GetSnapshot(&snapshot);
  return snapshot.angle_x100;
}

int32_t MotorFeedback_GetRpmX10(void)
{
  MotorFeedbackSnapshot_t snapshot;
  MotorFeedback_GetSnapshot(&snapshot);
  return snapshot.rpm_x10;
}

int32_t MotorFeedback_GetDeltaRaw(void)
{
  MotorFeedbackSnapshot_t snapshot;
  MotorFeedback_GetSnapshot(&snapshot);
  return snapshot.delta_raw;
}

int32_t MotorFeedback_GetTotalRawTurns(void)
{
  MotorFeedbackSnapshot_t snapshot;
  MotorFeedback_GetSnapshot(&snapshot);
  return snapshot.total_raw_turns;
}

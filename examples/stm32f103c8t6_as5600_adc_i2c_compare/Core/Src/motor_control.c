#include "motor_control.h"

#include <stdint.h>

#include "motor_driver.h"

#define MOTOR_DUTY_MIN               0.10f
#define MOTOR_DUTY_MAX               0.60f
#define MOTOR_DUTY_DEFAULT           0.20f
#define MOTOR_DUTY_STEP              0.05f
#define MOTOR_SPEED_LEVEL_MIN        1U
#define MOTOR_SPEED_LEVEL_MAX        6U
#define MOTOR_SPEED_LEVEL_DEFAULT    3U
#define MOTOR_UPDATE_PERIOD_MS       1U
#define MOTOR_RAMP_PERIOD_MS         200U
#define MOTOR_LUT_SIZE               256U
#define MOTOR_LUT_PHASE_B_OFFSET     85U
#define MOTOR_LUT_PHASE_C_OFFSET     171U
#define MOTOR_CENTER_DUTY            0.50f
#define MOTOR_AMPLITUDE_MIN          0.05f
#define MOTOR_AMPLITUDE_MAX          0.20f

static const uint16_t kSpeedPeriodMs[MOTOR_SPEED_LEVEL_MAX] = {80U, 50U, 30U, 20U, 15U, 10U};

static const int16_t kSineLut[MOTOR_LUT_SIZE] = {
       0,    804,   1608,   2410,   3212,   4011,   4808,   5602,
    6393,   7179,   7962,   8739,   9512,  10278,  11039,  11793,
   12539,  13279,  14010,  14732,  15446,  16151,  16846,  17530,
   18204,  18868,  19519,  20159,  20787,  21403,  22005,  22594,
   23170,  23731,  24279,  24811,  25329,  25832,  26319,  26790,
   27245,  27683,  28105,  28510,  28898,  29268,  29621,  29956,
   30273,  30571,  30852,  31113,  31356,  31580,  31785,  31971,
   32137,  32285,  32412,  32521,  32609,  32678,  32728,  32757,
   32767,  32757,  32728,  32678,  32609,  32521,  32412,  32285,
   32137,  31971,  31785,  31580,  31356,  31113,  30852,  30571,
   30273,  29956,  29621,  29268,  28898,  28510,  28105,  27683,
   27245,  26790,  26319,  25832,  25329,  24811,  24279,  23731,
   23170,  22594,  22005,  21403,  20787,  20159,  19519,  18868,
   18204,  17530,  16846,  16151,  15446,  14732,  14010,  13279,
   12539,  11793,  11039,  10278,   9512,   8739,   7962,   7179,
    6393,   5602,   4808,   4011,   3212,   2410,   1608,    804,
       0,   -804,  -1608,  -2410,  -3212,  -4011,  -4808,  -5602,
   -6393,  -7179,  -7962,  -8739,  -9512, -10278, -11039, -11793,
  -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530,
  -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
  -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790,
  -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
  -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971,
  -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
  -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
  -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
  -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683,
  -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
  -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868,
  -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
  -12539, -11793, -11039, -10278,  -9512,  -8739,  -7962,  -7179,
   -6393,  -5602,  -4808,  -4011,  -3212,  -2410,  -1608,   -804
};

static uint8_t g_running = 0U;
static float g_duty = MOTOR_DUTY_DEFAULT;
static MotorDirection_t g_direction = MOTOR_DIR_FWD;
static uint8_t g_target_speed_level = MOTOR_SPEED_LEVEL_DEFAULT;
static uint8_t g_current_speed_level = MOTOR_SPEED_LEVEL_MIN;
static uint8_t g_theta_index = 0U;
static uint32_t g_last_update_tick = 0U;
static uint32_t g_last_ramp_tick = 0U;

static float MotorControl_ClampDuty(float duty)
{
  if (duty < MOTOR_DUTY_MIN) return MOTOR_DUTY_MIN;
  if (duty > MOTOR_DUTY_MAX) return MOTOR_DUTY_MAX;
  return duty;
}

static float MotorControl_ClampPhaseDuty(float duty)
{
  if (duty < 0.0f) return 0.0f;
  if (duty > 1.0f) return 1.0f;
  return duty;
}

static uint8_t MotorControl_ClampSpeedLevel(uint8_t level)
{
  if (level < MOTOR_SPEED_LEVEL_MIN) return MOTOR_SPEED_LEVEL_MIN;
  if (level > MOTOR_SPEED_LEVEL_MAX) return MOTOR_SPEED_LEVEL_MAX;
  return level;
}

static float MotorControl_DutyToAmplitude(float duty)
{
  float amplitude = duty * 0.33f;
  if (amplitude < MOTOR_AMPLITUDE_MIN) amplitude = MOTOR_AMPLITUDE_MIN;
  if (amplitude > MOTOR_AMPLITUDE_MAX) amplitude = MOTOR_AMPLITUDE_MAX;
  return amplitude;
}

static void MotorControl_ApplyOpenLoopPwm(void)
{
  const float amplitude = MotorControl_DutyToAmplitude(g_duty);
  const int16_t sa = kSineLut[g_theta_index];
  const int16_t sb = kSineLut[(uint8_t)(g_theta_index + MOTOR_LUT_PHASE_B_OFFSET)];
  const int16_t sc = kSineLut[(uint8_t)(g_theta_index + MOTOR_LUT_PHASE_C_OFFSET)];

  const float du = MotorControl_ClampPhaseDuty(MOTOR_CENTER_DUTY + amplitude * ((float)sa / 32767.0f));
  const float dv = MotorControl_ClampPhaseDuty(MOTOR_CENTER_DUTY + amplitude * ((float)sb / 32767.0f));
  const float dw = MotorControl_ClampPhaseDuty(MOTOR_CENTER_DUTY + amplitude * ((float)sc / 32767.0f));

  MotorDriver_SetPwmDuty(du, dv, dw);
}

void MotorControl_Init(void)
{
  g_running = 0U;
  g_duty = MOTOR_DUTY_DEFAULT;
  g_direction = MOTOR_DIR_FWD;
  g_target_speed_level = MOTOR_SPEED_LEVEL_DEFAULT;
  g_current_speed_level = MOTOR_SPEED_LEVEL_MIN;
  g_theta_index = 0U;
  g_last_update_tick = HAL_GetTick();
  g_last_ramp_tick = g_last_update_tick;

  MotorDriver_SetAllPwmZero();
  MotorDriver_Disable();
}

void MotorControl_Start(void)
{
  if (g_running != 0U) return;

  g_current_speed_level = MOTOR_SPEED_LEVEL_MIN;
  g_theta_index = 0U;
  MotorDriver_SetAllPwmZero();
  MotorDriver_Enable();
  g_running = 1U;
  g_last_update_tick = HAL_GetTick();
  g_last_ramp_tick = g_last_update_tick;
  MotorControl_ApplyOpenLoopPwm();
}

void MotorControl_Stop(void)
{
  MotorDriver_SetAllPwmZero();
  MotorDriver_Disable();
  g_running = 0U;
  g_current_speed_level = MOTOR_SPEED_LEVEL_MIN;
  g_last_update_tick = HAL_GetTick();
  g_last_ramp_tick = g_last_update_tick;
}

void MotorControl_Update(uint32_t now)
{
  if (g_running == 0U) return;

  if ((now - g_last_ramp_tick) >= MOTOR_RAMP_PERIOD_MS)
  {
    if (g_current_speed_level < g_target_speed_level) g_current_speed_level++;
    else if (g_current_speed_level > g_target_speed_level) g_current_speed_level--;
    g_last_ramp_tick = now;
  }

  if ((now - g_last_update_tick) >= MOTOR_UPDATE_PERIOD_MS)
  {
    const uint16_t period = kSpeedPeriodMs[g_current_speed_level - 1U];
    const uint8_t step = (uint8_t)((MOTOR_LUT_SIZE + (period / 2U)) / period);
    if (g_direction == MOTOR_DIR_FWD) g_theta_index = (uint8_t)(g_theta_index + step);
    else g_theta_index = (uint8_t)(g_theta_index - step);

    MotorControl_ApplyOpenLoopPwm();
    g_last_update_tick = now;
  }
}

uint8_t MotorControl_IsRunning(void) { return g_running; }

void MotorControl_SetDirection(MotorDirection_t dir)
{
  if (dir != MOTOR_DIR_REV) dir = MOTOR_DIR_FWD;
  g_direction = dir;
}

MotorDirection_t MotorControl_GetDirection(void) { return g_direction; }

void MotorControl_ToggleDirection(void)
{
  if (g_running != 0U) MotorControl_Stop();
  g_direction = (g_direction == MOTOR_DIR_FWD) ? MOTOR_DIR_REV : MOTOR_DIR_FWD;
}

void MotorControl_SetSpeedLevel(uint8_t level) { g_target_speed_level = MotorControl_ClampSpeedLevel(level); }
uint8_t MotorControl_GetSpeedLevel(void) { return g_target_speed_level; }
void MotorControl_SpeedUp(void) { MotorControl_SetSpeedLevel((uint8_t)(g_target_speed_level + 1U)); }
void MotorControl_SpeedDown(void) { MotorControl_SetSpeedLevel((uint8_t)(g_target_speed_level - 1U)); }
uint16_t MotorControl_GetStepPeriodMs(void) { return kSpeedPeriodMs[g_target_speed_level - 1U]; }

void MotorControl_SetDuty(float duty) { g_duty = MotorControl_ClampDuty(duty); }
float MotorControl_GetDuty(void) { return g_duty; }
void MotorControl_DutyUp(void) { MotorControl_SetDuty(g_duty + MOTOR_DUTY_STEP); }
void MotorControl_DutyDown(void) { MotorControl_SetDuty(g_duty - MOTOR_DUTY_STEP); }

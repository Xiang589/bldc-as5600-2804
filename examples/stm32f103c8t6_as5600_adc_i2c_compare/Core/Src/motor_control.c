#include "motor_control.h"

#include <stdint.h>

#include "motor_driver.h"
#include "motor_feedback.h"

#define MOTOR_DUTY_MIN               0.10f
#define MOTOR_DUTY_MAX               0.60f
#define MOTOR_DUTY_DEFAULT           0.20f
#define MOTOR_DUTY_STEP              0.05f
#define MOTOR_SPEED_LEVEL_MIN        1U
#define MOTOR_SPEED_LEVEL_MAX        6U
#define MOTOR_SPEED_LEVEL_DEFAULT    3U
#define MOTOR_UPDATE_DT_MAX_MS       20U
#define MOTOR_RAMP_PERIOD_MS         200U
#define MOTOR_LUT_SIZE               256U
#define MOTOR_LUT_PHASE_B_OFFSET     85U
#define MOTOR_LUT_PHASE_C_OFFSET     171U
#define MOTOR_CENTER_DUTY_PERMYRIAD 5000U
#define MOTOR_FULL_DUTY_PERMYRIAD    10000U
#define MOTOR_AMPLITUDE_MIN          0.05f
#define MOTOR_AMPLITUDE_MAX          0.20f
#define MOTOR_PHASE_FRAC_BITS        16U
#define MOTOR_PHASE_SCALE            (1UL << MOTOR_PHASE_FRAC_BITS)
#define MOTOR_PHASE_FULL             (MOTOR_LUT_SIZE * MOTOR_PHASE_SCALE)
#define MOTOR_CL_PERIOD_MS            100U
#define MOTOR_CL_KP_NUM               1
#define MOTOR_CL_KP_DEN               20
#define MOTOR_CL_KI_NUM               1
#define MOTOR_CL_KI_DEN               200
#define MOTOR_CL_PERIOD_MIN_MS        8U
#define MOTOR_CL_PERIOD_MAX_MS        120U
#define MOTOR_CL_FEEDBACK_TIMEOUT_MS  500U
#define MOTOR_CL_STARTUP_GRACE_MS    2000U

static const uint16_t kSpeedPeriodMs[MOTOR_SPEED_LEVEL_MAX] = {
  300U,
  200U,
  120U,
  80U,
  50U,
  30U
};
static const int32_t kTargetRpmX10[MOTOR_SPEED_LEVEL_MAX] = {50, 100, 200, 300, 450, 600};

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
static uint16_t g_duty_permyriad = (uint16_t)(MOTOR_DUTY_DEFAULT * 10000.0f);
static MotorDirection_t g_direction = MOTOR_DIR_FWD;
static uint8_t g_target_speed_level = MOTOR_SPEED_LEVEL_DEFAULT;
static uint16_t g_target_period_ms = 30U;
static uint16_t g_current_period_ms = 80U;
static uint32_t g_phase_q16 = 0U;
static uint32_t g_last_update_tick = 0U;
static uint32_t g_last_ramp_tick = 0U;
static MotorControlMode_t g_mode = MOTOR_MODE_OPEN_LOOP;
static int32_t g_cl_integrator = 0;
static uint32_t g_last_cl_tick = 0U;
static uint16_t g_cl_period_ms = 80U;
static uint32_t g_feedback_lost_tick = 0U;
static uint32_t g_start_tick = 0U;
static uint8_t g_control_ready = 0U;

static float MotorControl_ClampDuty(float duty)
{
  if (duty < MOTOR_DUTY_MIN) return MOTOR_DUTY_MIN;
  if (duty > MOTOR_DUTY_MAX) return MOTOR_DUTY_MAX;
  return duty;
}

static uint8_t MotorControl_ClampSpeedLevel(uint8_t level)
{
  if (level < MOTOR_SPEED_LEVEL_MIN) return MOTOR_SPEED_LEVEL_MIN;
  if (level > MOTOR_SPEED_LEVEL_MAX) return MOTOR_SPEED_LEVEL_MAX;
  return level;
}

static uint16_t MotorControl_DutyToAmplitudePermyriad(uint16_t duty_permyriad)
{
  uint32_t amplitude = ((uint32_t)duty_permyriad * 33U) / 100U;
  if (amplitude < 500U) amplitude = 500U;
  if (amplitude > 2000U) amplitude = 2000U;
  return (uint16_t)amplitude;
}

static uint16_t MotorControl_ClampPermyriad(int32_t duty)
{
  if (duty < 0) return 0U;
  if (duty > (int32_t)MOTOR_FULL_DUTY_PERMYRIAD) return MOTOR_FULL_DUTY_PERMYRIAD;
  return (uint16_t)duty;
}

static void MotorControl_ApplyOpenLoopPwm(void)
{
  const uint8_t index = (uint8_t)((g_phase_q16 >> MOTOR_PHASE_FRAC_BITS) & 0xFFU);
  const uint16_t amplitude_permyriad = MotorControl_DutyToAmplitudePermyriad(g_duty_permyriad);
  const int16_t sa = kSineLut[index];
  const int16_t sb = kSineLut[(uint8_t)(index + MOTOR_LUT_PHASE_B_OFFSET)];
  const int16_t sc = kSineLut[(uint8_t)(index + MOTOR_LUT_PHASE_C_OFFSET)];

  const int32_t du = (int32_t)MOTOR_CENTER_DUTY_PERMYRIAD + ((int32_t)amplitude_permyriad * (int32_t)sa) / 32767;
  const int32_t dv = (int32_t)MOTOR_CENTER_DUTY_PERMYRIAD + ((int32_t)amplitude_permyriad * (int32_t)sb) / 32767;
  const int32_t dw = (int32_t)MOTOR_CENTER_DUTY_PERMYRIAD + ((int32_t)amplitude_permyriad * (int32_t)sc) / 32767;

  MotorDriver_SetPwmDutyPermyriad(MotorControl_ClampPermyriad(du),
                                  MotorControl_ClampPermyriad(dv),
                                  MotorControl_ClampPermyriad(dw));
}

void MotorControl_Init(void)
{
  g_running = 0U;
  g_duty = MOTOR_DUTY_DEFAULT;
  g_duty_permyriad = (uint16_t)(MOTOR_DUTY_DEFAULT * 10000.0f);
  g_direction = MOTOR_DIR_FWD;
  g_target_speed_level = MOTOR_SPEED_LEVEL_DEFAULT;
  g_target_period_ms = kSpeedPeriodMs[g_target_speed_level - 1U];
  g_current_period_ms = kSpeedPeriodMs[MOTOR_SPEED_LEVEL_MIN - 1U];
  g_phase_q16 = 0U;
  g_last_update_tick = HAL_GetTick();
  g_last_ramp_tick = g_last_update_tick;
  g_mode = MOTOR_MODE_OPEN_LOOP;
  g_cl_integrator = 0;
  g_last_cl_tick = g_last_update_tick;
  g_cl_period_ms = kSpeedPeriodMs[MOTOR_SPEED_LEVEL_MIN - 1U];
  g_feedback_lost_tick = 0U;
  g_start_tick = 0U;

  MotorDriver_SetAllPwmZero();
  MotorDriver_Disable();
  g_control_ready = 1U;
}

void MotorControl_Start(void)
{
  if (g_running != 0U) return;

  if ((g_mode == MOTOR_MODE_SPEED_CLOSED_LOOP) && (MotorFeedback_IsAngleValid() == 0U))
  {
    return;
  }

  g_current_period_ms = kSpeedPeriodMs[MOTOR_SPEED_LEVEL_MIN - 1U];
  g_cl_period_ms = g_current_period_ms;
  g_cl_integrator = 0;
  g_phase_q16 = 0U;
  MotorDriver_SetAllPwmZero();
  MotorDriver_Enable();
  g_running = 1U;
  g_last_update_tick = HAL_GetTick();
  g_last_ramp_tick = g_last_update_tick;
  g_last_cl_tick = g_last_update_tick;
  g_feedback_lost_tick = 0U;
  g_start_tick = g_last_update_tick;
  MotorControl_ApplyOpenLoopPwm();
}

void MotorControl_Stop(void)
{
  MotorDriver_SetAllPwmZero();
  MotorDriver_Disable();
  g_running = 0U;
  g_current_period_ms = kSpeedPeriodMs[MOTOR_SPEED_LEVEL_MIN - 1U];
  g_cl_period_ms = g_current_period_ms;
  g_cl_integrator = 0;
  g_last_update_tick = HAL_GetTick();
  g_last_ramp_tick = g_last_update_tick;
  g_last_cl_tick = g_last_update_tick;
  g_feedback_lost_tick = 0U;
  g_start_tick = g_last_update_tick;
}

void MotorControl_Update(uint32_t now)
{
  uint32_t dt_ms;

  if (g_running == 0U) return;

  dt_ms = now - g_last_update_tick;
  if (dt_ms == 0U) return;
  if (dt_ms > MOTOR_UPDATE_DT_MAX_MS) dt_ms = MOTOR_UPDATE_DT_MAX_MS;

  if (g_mode == MOTOR_MODE_SPEED_CLOSED_LOOP)
  {
    if (MotorFeedback_IsSpeedValid() == 0U)
    {
      if ((now - g_start_tick) < MOTOR_CL_STARTUP_GRACE_MS)
      {
        /* allow startup without valid speed feedback yet */
      }
      else
      {
        if (g_feedback_lost_tick == 0U) g_feedback_lost_tick = now;
        else if ((now - g_feedback_lost_tick) >= MOTOR_CL_FEEDBACK_TIMEOUT_MS)
        {
          MotorControl_Stop();
          return;
        }
      }
    }
    else
    {
      g_feedback_lost_tick = 0U;
      if ((now - g_last_cl_tick) >= MOTOR_CL_PERIOD_MS)
      {
        int32_t target = kTargetRpmX10[g_target_speed_level - 1U];
        int32_t actual = MotorFeedback_GetRpmX10();
        int32_t err;
        int32_t adjust;
        int32_t np;
        if (actual < 0) actual = -actual;
        err = target - actual;
        g_cl_integrator += err;
        if (g_cl_integrator > 5000) g_cl_integrator = 5000;
        if (g_cl_integrator < -5000) g_cl_integrator = -5000;
        adjust = (err * MOTOR_CL_KP_NUM) / MOTOR_CL_KP_DEN +
                 (g_cl_integrator * MOTOR_CL_KI_NUM) / MOTOR_CL_KI_DEN;
        np = (int32_t)g_target_period_ms - adjust;
        if (np < (int32_t)MOTOR_CL_PERIOD_MIN_MS) np = MOTOR_CL_PERIOD_MIN_MS;
        if (np > (int32_t)MOTOR_CL_PERIOD_MAX_MS) np = MOTOR_CL_PERIOD_MAX_MS;
        g_cl_period_ms = (uint16_t)np;
        g_current_period_ms = g_cl_period_ms;
        g_last_cl_tick = now;
      }
    }
  }
  else
  {
    if ((now - g_last_ramp_tick) >= MOTOR_RAMP_PERIOD_MS)
    {
      if (g_current_period_ms > g_target_period_ms) g_current_period_ms--;
      else if (g_current_period_ms < g_target_period_ms) g_current_period_ms++;
      g_last_ramp_tick = now;
    }
  }

  {
    uint32_t delta_q16 = (uint32_t)(((uint64_t)MOTOR_PHASE_FULL * dt_ms) / g_current_period_ms);
    if (g_direction == MOTOR_DIR_FWD)
    {
      g_phase_q16 += delta_q16;
      while (g_phase_q16 >= MOTOR_PHASE_FULL) g_phase_q16 -= MOTOR_PHASE_FULL;
    }
    else
    {
      delta_q16 %= MOTOR_PHASE_FULL;
      if (g_phase_q16 >= delta_q16) g_phase_q16 -= delta_q16;
      else g_phase_q16 = MOTOR_PHASE_FULL - (delta_q16 - g_phase_q16);
    }
  }

  MotorControl_ApplyOpenLoopPwm();
  g_last_update_tick = now;
}


void MotorControl_Tick1ms(void)
{
  if (g_control_ready == 0U)
  {
    return;
  }

  MotorControl_Update(HAL_GetTick());
}

uint8_t MotorControl_IsRunning(void) { return g_running; }

void MotorControl_SetDirection(MotorDirection_t dir)
{
  if (dir != MOTOR_DIR_REV) dir = MOTOR_DIR_FWD;
  if ((g_running != 0U) && (dir != g_direction))
  {
    MotorControl_Stop();
  }
  g_direction = dir;
}

MotorDirection_t MotorControl_GetDirection(void) { return g_direction; }

void MotorControl_ToggleDirection(void)
{
  if (g_running != 0U) MotorControl_Stop();
  g_direction = (g_direction == MOTOR_DIR_FWD) ? MOTOR_DIR_REV : MOTOR_DIR_FWD;
}

void MotorControl_SetSpeedLevel(uint8_t level)
{
  g_target_speed_level = MotorControl_ClampSpeedLevel(level);
  g_target_period_ms = kSpeedPeriodMs[g_target_speed_level - 1U];
}
uint8_t MotorControl_GetSpeedLevel(void) { return g_target_speed_level; }
void MotorControl_SpeedUp(void) { MotorControl_SetSpeedLevel((uint8_t)(g_target_speed_level + 1U)); }
void MotorControl_SpeedDown(void) { MotorControl_SetSpeedLevel((uint8_t)(g_target_speed_level - 1U)); }
uint16_t MotorControl_GetStepPeriodMs(void) { return kSpeedPeriodMs[g_target_speed_level - 1U]; }

void MotorControl_SetDuty(float duty)
{
  g_duty = MotorControl_ClampDuty(duty);
  g_duty_permyriad = (uint16_t)(g_duty * 10000.0f);
}
float MotorControl_GetDuty(void) { return g_duty; }
void MotorControl_DutyUp(void) { MotorControl_SetDuty(g_duty + MOTOR_DUTY_STEP); }
void MotorControl_DutyDown(void) { MotorControl_SetDuty(g_duty - MOTOR_DUTY_STEP); }

uint16_t MotorControl_GetTargetPeriodMs(void) { return g_target_period_ms; }
uint16_t MotorControl_GetCurrentPeriodMs(void) { return g_current_period_ms; }
uint32_t MotorControl_GetPhaseQ16(void) { return g_phase_q16; }
uint8_t MotorControl_GetPhaseIndex(void)
{
  return (uint8_t)((g_phase_q16 >> MOTOR_PHASE_FRAC_BITS) & 0xFFU);
}

void MotorControl_SetMode(MotorControlMode_t mode)
{
  if (mode != MOTOR_MODE_SPEED_CLOSED_LOOP)
  {
    mode = MOTOR_MODE_OPEN_LOOP;
  }

  if ((g_running != 0U) && (mode != g_mode))
  {
    MotorControl_Stop();
  }

  g_mode = mode;
}

void MotorControl_ToggleMode(void)
{
  if (g_mode == MOTOR_MODE_OPEN_LOOP)
  {
    MotorControl_SetMode(MOTOR_MODE_SPEED_CLOSED_LOOP);
  }
  else
  {
    MotorControl_SetMode(MOTOR_MODE_OPEN_LOOP);
  }
}

MotorControlMode_t MotorControl_GetMode(void)
{
  return g_mode;
}

int32_t MotorControl_GetTargetRpmX10(void) { return kTargetRpmX10[g_target_speed_level - 1U]; }

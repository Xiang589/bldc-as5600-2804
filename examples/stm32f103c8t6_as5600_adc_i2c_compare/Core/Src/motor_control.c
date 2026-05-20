#include "motor_control.h"

#include "motor_driver.h"

#define MOTOR_DUTY_MIN          0.10f
#define MOTOR_DUTY_MAX          0.60f
#define MOTOR_DUTY_DEFAULT      0.20f
#define MOTOR_PATTERN_PERIOD_MS 120U

static uint8_t g_running = 0U;
static float g_duty = MOTOR_DUTY_DEFAULT;
static uint16_t g_duty_permyriad = 2000U;
static uint8_t g_comm_step = 0U;
static uint32_t g_last_step_tick = 0U;

static float MotorControl_ClampDuty(float duty)
{
  if (duty < MOTOR_DUTY_MIN)
  {
    return MOTOR_DUTY_MIN;
  }
  if (duty > MOTOR_DUTY_MAX)
  {
    return MOTOR_DUTY_MAX;
  }
  return duty;
}


static uint16_t MotorControl_DutyFloatToPermyriad(float duty)
{
  const float clamped = MotorControl_ClampDuty(duty);
  return (uint16_t)(clamped * 10000.0f + 0.5f);
}

static void MotorControl_ApplyPatternStep(void)
{
  const uint16_t d = g_duty_permyriad;
  switch (g_comm_step)
  {
    case 0U: MotorDriver_SetPwmDutyPermyriad(d, 0U, 0U); break;
    case 1U: MotorDriver_SetPwmDutyPermyriad(0U, d, 0U); break;
    default: MotorDriver_SetPwmDutyPermyriad(0U, 0U, d); break;
  }
  g_comm_step = (uint8_t)((g_comm_step + 1U) % 3U);
}

void MotorControl_Init(void)
{
  g_running = 0U;
  g_duty = MOTOR_DUTY_DEFAULT;
  g_duty_permyriad = MotorControl_DutyFloatToPermyriad(MOTOR_DUTY_DEFAULT);
  g_comm_step = 0U;
  g_last_step_tick = HAL_GetTick();

  MotorDriver_SetAllPwmZero();
  MotorDriver_Disable();
}

void MotorControl_Start(void)
{
  if (g_running != 0U)
  {
    return;
  }
  g_comm_step = 0U;
  MotorDriver_SetAllPwmZero();
  MotorDriver_Enable();
  g_running = 1U;
  MotorControl_ApplyPatternStep();
  g_last_step_tick = HAL_GetTick();
}

void MotorControl_Stop(void)
{
  MotorDriver_SetAllPwmZero();
  MotorDriver_Disable();
  g_running = 0U;
}

void MotorControl_SetDuty(float duty)
{
  g_duty = MotorControl_ClampDuty(duty);
  g_duty_permyriad = MotorControl_DutyFloatToPermyriad(g_duty);
}

float MotorControl_GetDuty(void)
{
  return g_duty;
}

uint8_t MotorControl_IsRunning(void)
{
  return g_running;
}

void MotorControl_Update(uint32_t now)
{
  if (g_running == 0U)
  {
    return;
  }

  if ((now - g_last_step_tick) >= MOTOR_PATTERN_PERIOD_MS)
  {
    MotorControl_ApplyPatternStep();
    g_last_step_tick = now;
  }
}

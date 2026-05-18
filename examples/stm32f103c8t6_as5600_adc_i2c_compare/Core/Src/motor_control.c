#include "motor_control.h"

#include "motor_driver.h"

#define MOTOR_DUTY_MIN          0.10f
#define MOTOR_DUTY_MAX          0.60f
#define MOTOR_DUTY_DEFAULT      0.20f
#define MOTOR_PATTERN_PERIOD_MS 20U

static uint8_t g_running = 0U;
static float g_duty = MOTOR_DUTY_DEFAULT;
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

static const float kMotorPatternTable[6][3] = {
  {0.60f, 0.40f, 0.50f},
  {0.60f, 0.50f, 0.40f},
  {0.50f, 0.60f, 0.40f},
  {0.40f, 0.60f, 0.50f},
  {0.40f, 0.50f, 0.60f},
  {0.50f, 0.40f, 0.60f},
};

static float MotorControl_ClampPhaseDuty(float duty)
{
  if (duty < 0.0f)
  {
    return 0.0f;
  }
  if (duty > 1.0f)
  {
    return 1.0f;
  }
  return duty;
}

static void MotorControl_ApplyPatternStep(void)
{
  const float gain = g_duty / MOTOR_DUTY_DEFAULT;
  const float du = MotorControl_ClampPhaseDuty(kMotorPatternTable[g_comm_step][0] * gain);
  const float dv = MotorControl_ClampPhaseDuty(kMotorPatternTable[g_comm_step][1] * gain);
  const float dw = MotorControl_ClampPhaseDuty(kMotorPatternTable[g_comm_step][2] * gain);

  MotorDriver_SetPwmDuty(du, dv, dw);
  g_comm_step = (uint8_t)((g_comm_step + 1U) % 6U);
}

void MotorControl_Init(void)
{
  g_running = 0U;
  g_duty = MOTOR_DUTY_DEFAULT;
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

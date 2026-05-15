#include "motor_driver.h"

#include "main.h"
#include "tim.h"

enum
{
  MOTOR_TIM_CH_U = TIM_CHANNEL_1,
  MOTOR_TIM_CH_V = TIM_CHANNEL_2,
  MOTOR_TIM_CH_W = TIM_CHANNEL_3
};

static float ClampDuty(float duty)
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

static uint32_t DutyToCompare(float duty)
{
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  float clamped = ClampDuty(duty);
  return (uint32_t)((float)arr * clamped);
}

void MotorDriver_SetPwmDuty(float duty_u, float duty_v, float duty_w)
{
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_TIM_CH_U, DutyToCompare(duty_u));
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_TIM_CH_V, DutyToCompare(duty_v));
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_TIM_CH_W, DutyToCompare(duty_w));
}

void MotorDriver_SetAllPwmZero(void)
{
  MotorDriver_SetPwmDuty(0.0f, 0.0f, 0.0f);
}

/*
 * MotorDriver_Enable() 仅负责拉高 FOCMini EN。
 * 调用前必须确认 TIM1_CH1/CH2/CH3 的 PWM 占空比处于安全值。
 * 当前上电默认由 MotorDriver_Init() 保证：PWM = 0，EN = Low。
 * 本函数不会自动设置占空比，也不会让电机自动旋转。
 */
void MotorDriver_Enable(void)
{
  HAL_GPIO_WritePin(FOCMINI_EN_GPIO_Port, FOCMINI_EN_Pin, GPIO_PIN_SET);
}

void MotorDriver_Disable(void)
{
  MotorDriver_SetAllPwmZero();
  HAL_GPIO_WritePin(FOCMINI_EN_GPIO_Port, FOCMINI_EN_Pin, GPIO_PIN_RESET);
}

void MotorDriver_Init(void)
{
  MotorDriver_Disable();

  (void)HAL_TIM_PWM_Start(&htim1, MOTOR_TIM_CH_U);
  (void)HAL_TIM_PWM_Start(&htim1, MOTOR_TIM_CH_V);
  (void)HAL_TIM_PWM_Start(&htim1, MOTOR_TIM_CH_W);

  MotorDriver_SetAllPwmZero();
}

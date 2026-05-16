#include "motor_driver.h"

#include "main.h"
#include "tim.h"

/* 电机驱动安全骨架说明：
 * - 当前仅用于三相 PWM 与使能引脚的基础联调，不是正式 FOC 控制器。
 * - TIM1 CH1/CH2/CH3 分别对应 U/V/W 三相 PWM 输出。
 * - FOCMINI_EN 是驱动板使能，上电默认应保持关闭。 */
enum
{
  MOTOR_TIM_CH_U = TIM_CHANNEL_1,
  MOTOR_TIM_CH_V = TIM_CHANNEL_2,
  MOTOR_TIM_CH_W = TIM_CHANNEL_3
};

static float ClampDuty(float duty)
{
  /* 占空比限幅到 0.0~1.0，避免错误输入导致异常 PWM 输出。 */
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
  /* 读取 TIM1 的 ARR（自动重装载值），ARR 决定一个 PWM 周期的计数上限。 */
  uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
  float clamped = ClampDuty(duty);
  /* 将 0.0~1.0 的 duty 转成 CCR 比较值，近似满足 CCR/ARR ~= duty。 */
  return (uint32_t)((float)arr * clamped);
}

void MotorDriver_SetPwmDuty(float duty_u, float duty_v, float duty_w)
{
  /* 分别设置 U/V/W 三相 CCR，对应 TIM1 CH1/CH2/CH3。 */
  /* __HAL_TIM_SET_COMPARE 会写入 CCR 寄存器，改变对应通道占空比。 */
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_TIM_CH_U, DutyToCompare(duty_u));
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_TIM_CH_V, DutyToCompare(duty_v));
  __HAL_TIM_SET_COMPARE(&htim1, MOTOR_TIM_CH_W, DutyToCompare(duty_w));
}

void MotorDriver_SetAllPwmZero(void)
{
  /* 安全清零：三相全部输出 0 占空比。 */
  MotorDriver_SetPwmDuty(0.0f, 0.0f, 0.0f);
}

/*
 * MotorDriver_Enable() 仅负责拉高 FOCMINI_EN。
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
  /* 更安全的关断顺序：先把 PWM 清零，再关闭 EN。 */
  MotorDriver_SetAllPwmZero();
  HAL_GPIO_WritePin(FOCMINI_EN_GPIO_Port, FOCMINI_EN_Pin, GPIO_PIN_RESET);
}

void MotorDriver_Init(void)
{
  /* 上电初始化先关驱动使能，避免未配置完成就输出到电机。 */
  MotorDriver_Disable();

  /* 启动 TIM1 三路 PWM 通道（U/V/W 三相）。 */
  (void)HAL_TIM_PWM_Start(&htim1, MOTOR_TIM_CH_U);
  (void)HAL_TIM_PWM_Start(&htim1, MOTOR_TIM_CH_V);
  (void)HAL_TIM_PWM_Start(&htim1, MOTOR_TIM_CH_W);

  /* 再次清零占空比，确保初始化结束时仍处于安全输出。 */
  MotorDriver_SetAllPwmZero();
}

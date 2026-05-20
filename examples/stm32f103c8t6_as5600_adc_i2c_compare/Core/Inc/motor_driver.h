#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 本模块是当前阶段的三相 PWM 安全测试骨架，便于外设联调与上电验证；
 * 它不是完整 FOC（磁场定向控制）算法实现。 */

/* 初始化流程：先关使能，再启动 PWM，再清零占空比，确保上电默认安全。 */
void MotorDriver_Init(void);
/* 仅拉高驱动板 EN 引脚，不会自动修改三相占空比。 */
void MotorDriver_Enable(void);
/* 先清零三相 PWM，再拉低 EN，属于更安全的关断顺序。 */
void MotorDriver_Disable(void);
/* 将 U/V/W 三相占空比全部设为 0，用于快速进入安全输出。 */
void MotorDriver_SetAllPwmZero(void);
/* 分别设置 U/V/W 三相占空比，输入建议范围 0.0~1.0。 */
void MotorDriver_SetPwmDuty(float duty_u, float duty_v, float duty_w);
/* 分别设置 U/V/W 三相占空比，输入范围 0~10000（万分比）。 */
void MotorDriver_SetPwmDutyPermyriad(uint16_t u, uint16_t v, uint16_t w);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */

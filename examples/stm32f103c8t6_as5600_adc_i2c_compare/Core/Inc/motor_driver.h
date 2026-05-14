#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

void MotorDriver_Init(void);
void MotorDriver_Enable(void);
void MotorDriver_Disable(void);
void MotorDriver_SetAllPwmZero(void);
void MotorDriver_SetPwmDuty(float duty_u, float duty_v, float duty_w);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DRIVER_H */

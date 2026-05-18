#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void MotorControl_Init(void);
void MotorControl_Start(void);
void MotorControl_Stop(void);
void MotorControl_SetDuty(float duty);
float MotorControl_GetDuty(void);
uint8_t MotorControl_IsRunning(void);
void MotorControl_Update(uint32_t now);

#ifdef __cplusplus
}
#endif

#endif

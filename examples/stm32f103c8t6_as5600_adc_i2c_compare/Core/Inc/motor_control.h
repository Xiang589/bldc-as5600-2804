#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  MOTOR_DIR_FWD = 0,
  MOTOR_DIR_REV = 1
} MotorDirection_t;

typedef enum {
  MOTOR_MODE_OPEN_LOOP = 0,
  MOTOR_MODE_SPEED_CLOSED_LOOP = 1
} MotorControlMode_t;

void MotorControl_Init(void);
void MotorControl_Start(void);
void MotorControl_Stop(void);
void MotorControl_Update(uint32_t now);

uint8_t MotorControl_IsRunning(void);

void MotorControl_SetDirection(MotorDirection_t dir);
MotorDirection_t MotorControl_GetDirection(void);
void MotorControl_ToggleDirection(void);

void MotorControl_SetSpeedLevel(uint8_t level);
uint8_t MotorControl_GetSpeedLevel(void);
void MotorControl_SpeedUp(void);
void MotorControl_SpeedDown(void);
uint16_t MotorControl_GetStepPeriodMs(void);
uint16_t MotorControl_GetTargetPeriodMs(void);
uint16_t MotorControl_GetCurrentPeriodMs(void);
uint32_t MotorControl_GetPhaseQ16(void);
uint8_t MotorControl_GetPhaseIndex(void);

void MotorControl_SetDuty(float duty);
float MotorControl_GetDuty(void);
void MotorControl_DutyUp(void);
void MotorControl_DutyDown(void);

void MotorControl_SetMode(MotorControlMode_t mode);
MotorControlMode_t MotorControl_GetMode(void);

#ifdef __cplusplus
}
#endif

#endif

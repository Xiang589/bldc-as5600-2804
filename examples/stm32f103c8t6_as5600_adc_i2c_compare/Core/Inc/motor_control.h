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
  MOTOR_MODE_SPEED_CLOSED_LOOP = 1,
  MOTOR_MODE_FOC_VOLTAGE = 2,
  MOTOR_MODE_FOC_VELOCITY = 3,
  MOTOR_MODE_FOC_POSITION = 4
} MotorControlMode_t;

typedef enum {
  MOTOR_STATE_STOPPED = 0,
  MOTOR_STATE_STARTUP = 1,
  MOTOR_STATE_RUNNING_OPEN_LOOP = 2,
  MOTOR_STATE_RUNNING_CLOSED_LOOP = 3,
  MOTOR_STATE_RUNNING_FOC_VOLTAGE = 4,
  MOTOR_STATE_RUNNING_FOC_VELOCITY = 5,
  MOTOR_STATE_RUNNING_FOC_POSITION = 6,
  MOTOR_STATE_CALIBRATION = 7,
  MOTOR_STATE_FAULT = 8
} MotorControlState_t;

typedef enum {
  MOTOR_STOP_REASON_NONE = 0,
  MOTOR_STOP_REASON_USER = 1,
  MOTOR_STOP_REASON_DIRECTION_CHANGED = 2,
  MOTOR_STOP_REASON_MODE_CHANGED = 3,
  MOTOR_STOP_REASON_FEEDBACK_LOST = 4,
  MOTOR_STOP_REASON_START_DENIED_NO_ANGLE = 5,
  MOTOR_STOP_REASON_START_DENIED_SENSOR_DIAG = 6,
  MOTOR_STOP_REASON_START_DENIED_FOC_NOT_CALIBRATED = 7,
  MOTOR_STOP_REASON_FOC_CALIBRATION_FAILED = 8
} MotorStopReason_t;

typedef enum {
  MOTOR_FAULT_NONE = 0,
  MOTOR_FAULT_FEEDBACK_LOST = 1,
  MOTOR_FAULT_STARTUP_FEEDBACK_TIMEOUT = 2,
  MOTOR_FAULT_INVALID_STATE = 3,
  MOTOR_FAULT_SENSOR_DIAG = 4,
  MOTOR_FAULT_ANGLE_STALE = 5,
  MOTOR_FAULT_FOC_CALIBRATION_FAILED = 6
} MotorFault_t;

void MotorControl_Init(void);
void MotorControl_Start(void);
void MotorControl_Stop(void);
void MotorControl_Update(uint32_t now);
void MotorControl_Tick1ms(void);

uint8_t MotorControl_IsRunning(void);
MotorControlState_t MotorControl_GetState(void);
MotorStopReason_t MotorControl_GetStopReason(void);
MotorFault_t MotorControl_GetFault(void);
void MotorControl_ClearFault(void);

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
int32_t MotorControl_GetSpeedPidErrorX10(void);
int32_t MotorControl_GetSpeedPidOutputMs(void);
int32_t MotorControl_GetSpeedPidIntegrator(void);
uint32_t MotorControl_GetPhaseQ16(void);
uint8_t MotorControl_GetPhaseIndex(void);

void MotorControl_SetDuty(float duty);
float MotorControl_GetDuty(void);
uint16_t MotorControl_GetModulationAmplitudePermyriad(void);
void MotorControl_DutyUp(void);
void MotorControl_DutyDown(void);

void MotorControl_SetMode(MotorControlMode_t mode);
MotorControlMode_t MotorControl_GetMode(void);
void MotorControl_ToggleMode(void);
/* Sets the speed target magnitude; direction is selected separately. */
void MotorControl_SetTargetRpmX10(int32_t rpm_x10);
int32_t MotorControl_GetTargetRpmX10(void);
/* Sets the FOC position target relative to the current software zero. */
void MotorControl_SetTargetPositionDegX10(int32_t deg_x10);
int32_t MotorControl_GetTargetPositionDegX10(void);
int32_t MotorControl_GetPositionDegX10(void);
int32_t MotorControl_GetFocUqMv(void);
int32_t MotorControl_GetFocTargetVelocityRpmX10(void);
int32_t MotorControl_GetFocVelocityErrorX10(void);
int32_t MotorControl_GetFocPositionErrorDegX10(void);
int32_t MotorControl_GetFocVoltageTargetMv(void);
uint8_t MotorControl_IsFocZeroCalibrated(void);
/* Runs the low-voltage d-axis alignment flow and captures FOC electrical zero. */
uint8_t MotorControl_CalibrateFocZero(void);

#ifdef __cplusplus
}
#endif

#endif

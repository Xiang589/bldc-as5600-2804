#ifndef MOTOR_COMMAND_H
#define MOTOR_COMMAND_H

#include <stdint.h>

#include "comm_protocol.h"
#include "motor_control.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  MOTOR_COMMAND_OK = 0,
  MOTOR_COMMAND_ERR_ARG,
  MOTOR_COMMAND_ERR_DISABLED,
  MOTOR_COMMAND_ERR_ESTOP,
  MOTOR_COMMAND_ERR_UNSUPPORTED,
  MOTOR_COMMAND_ERR_STATE
} MotorCommandResult_t;

typedef struct {
  uint8_t enabled;
  uint8_t estop_latched;
  CommProtocolMode_t mode;
  int32_t target_milli;
  int32_t voltage_limit_mv;
  int32_t velocity_limit_mrad_s;
  MotorControlState_t state;
  MotorStopReason_t stop_reason;
  MotorFault_t fault;
  uint8_t foc_zero_calibrated;
  int32_t angle_mrad;
  int32_t velocity_mrad_s;
  uint16_t raw_angle;
  uint8_t magnet_detected;
  uint8_t magnet_too_weak;
  uint8_t magnet_too_strong;
  int32_t foc_uq_mv;
} MotorCommandStatus_t;

void MotorCommand_Init(void);
MotorCommandResult_t MotorCommand_SetEnable(uint8_t enabled);
MotorCommandResult_t MotorCommand_SetMode(CommProtocolMode_t mode);
MotorCommandResult_t MotorCommand_SetTargetMilli(int32_t target_milli);
MotorCommandResult_t MotorCommand_SetVoltageMv(int32_t voltage_mv);
MotorCommandResult_t MotorCommand_SetVelocityMradS(int32_t velocity_mrad_s);
MotorCommandResult_t MotorCommand_SetPositionMrad(int32_t position_mrad);
MotorCommandResult_t MotorCommand_SetLimits(int32_t voltage_limit_mv,
                                            int32_t velocity_limit_mrad_s);
MotorCommandResult_t MotorCommand_Stop(void);
MotorCommandResult_t MotorCommand_EStop(void);
MotorCommandResult_t MotorCommand_ClearOrCalibrateZero(void);
void MotorCommand_HandleCommTimeout(void);
void MotorCommand_GetStatus(MotorCommandStatus_t *status);
uint8_t MotorCommand_IsEnabled(void);
const char *MotorCommand_ResultCode(MotorCommandResult_t result);
const char *MotorCommand_ResultMessage(MotorCommandResult_t result);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_COMMAND_H */

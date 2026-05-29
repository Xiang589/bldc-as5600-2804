#include "motor_command.h"

#include "FreeRTOS.h"
#include "motor_control_config.h"
#include "motor_feedback.h"
#include "task.h"

#define MOTOR_COMMAND_DEFAULT_VOLTAGE_LIMIT_MV MOTOR_FOC_VOLTAGE_LIMIT_MV
#define MOTOR_COMMAND_MAX_VOLTAGE_LIMIT_MV     MOTOR_FOC_VOLTAGE_LIMIT_MV
#define MOTOR_COMMAND_DEFAULT_VELOCITY_LIMIT_MRAD_S 12566
#define MOTOR_COMMAND_MAX_VELOCITY_LIMIT_MRAD_S     12566
#define MOTOR_COMMAND_PI_MILLI 3142
#define MOTOR_COMMAND_TWO_PI_MILLI 6283
#define MOTOR_COMMAND_POSITION_LIMIT_MRAD MOTOR_COMMAND_TWO_PI_MILLI

static uint8_t g_enabled = 0U;
static uint8_t g_estop_latched = 0U;
static CommProtocolMode_t g_mode = COMM_MODE_IDLE;
static int32_t g_target_milli = 0;
static int32_t g_voltage_limit_mv = MOTOR_COMMAND_DEFAULT_VOLTAGE_LIMIT_MV;
static int32_t g_velocity_limit_mrad_s = MOTOR_COMMAND_DEFAULT_VELOCITY_LIMIT_MRAD_S;
static MotorStopReason_t g_last_error_stop_reason = MOTOR_STOP_REASON_NONE;
static MotorFault_t g_last_error_fault = MOTOR_FAULT_NONE;

static MotorCommandResult_t MotorCommand_ApplyVoltageMv(int32_t voltage_mv,
                                                        uint8_t use_foc_voltage);

static int32_t MotorCommand_AbsI32(int32_t value)
{
  return (value < 0) ? -value : value;
}

static uint8_t MotorCommand_IsWithinSignedLimit(int32_t value, int32_t limit)
{
  return ((value >= -limit) && (value <= limit)) ? 1U : 0U;
}

static int32_t MotorCommand_MradS_ToRpmX10(int32_t velocity_mrad_s)
{
  int64_t value = (int64_t)velocity_mrad_s * 600LL;
  if (value >= 0)
  {
    value += MOTOR_COMMAND_PI_MILLI;
  }
  else
  {
    value -= MOTOR_COMMAND_PI_MILLI;
  }
  return (int32_t)(value / MOTOR_COMMAND_TWO_PI_MILLI);
}

static int32_t MotorCommand_RpmX10_ToMradS(int32_t rpm_x10)
{
  return (int32_t)(((int64_t)rpm_x10 * MOTOR_COMMAND_TWO_PI_MILLI) / 600LL);
}

static int32_t MotorCommand_Mrad_ToDegX10(int32_t position_mrad)
{
  return (int32_t)(((int64_t)position_mrad * 1800LL) / MOTOR_COMMAND_PI_MILLI);
}

static int32_t MotorCommand_DegX100_ToMrad(int32_t angle_x100)
{
  return (int32_t)(((int64_t)angle_x100 * MOTOR_COMMAND_PI_MILLI) / 18000LL);
}

static void MotorCommand_SetDirectionForSignedValue(int32_t value)
{
  MotorControl_SetDirection((value < 0) ? MOTOR_DIR_REV : MOTOR_DIR_FWD);
}

static void MotorCommand_ClearLastStateError(void)
{
  g_last_error_stop_reason = MOTOR_STOP_REASON_NONE;
  g_last_error_fault = MOTOR_FAULT_NONE;
}

static MotorCommandResult_t MotorCommand_RecordStateError(void)
{
  g_last_error_stop_reason = MotorControl_GetStopReason();
  g_last_error_fault = MotorControl_GetFault();
  return MOTOR_COMMAND_ERR_STATE;
}

static MotorCommandResult_t MotorCommand_StartAndVerify(void)
{
  MotorControl_Start();
  if (MotorControl_IsRunning() == 0U)
  {
    return MotorCommand_RecordStateError();
  }

  MotorCommand_ClearLastStateError();
  return MOTOR_COMMAND_OK;
}

static void MotorCommand_ApplyControlMode(CommProtocolMode_t mode)
{
  switch (mode)
  {
    case COMM_MODE_OPEN:
      MotorControl_SetMode(MOTOR_MODE_OPEN_LOOP);
      break;
    case COMM_MODE_VEL:
      MotorControl_SetMode(MOTOR_MODE_FOC_VELOCITY);
      break;
    case COMM_MODE_POS:
      MotorControl_SetMode(MOTOR_MODE_FOC_POSITION);
      break;
    case COMM_MODE_IDLE:
    default:
      MotorControl_Stop();
      break;
  }
}

static MotorCommandResult_t MotorCommand_CheckCanOutput(void)
{
  if (g_estop_latched != 0U)
  {
    return MOTOR_COMMAND_ERR_ESTOP;
  }
  if (g_enabled == 0U)
  {
    return MOTOR_COMMAND_ERR_DISABLED;
  }
  return MOTOR_COMMAND_OK;
}

void MotorCommand_Init(void)
{
  taskENTER_CRITICAL();
  g_enabled = 0U;
  g_estop_latched = 0U;
  g_mode = COMM_MODE_IDLE;
  g_target_milli = 0;
  g_voltage_limit_mv = MOTOR_COMMAND_DEFAULT_VOLTAGE_LIMIT_MV;
  g_velocity_limit_mrad_s = MOTOR_COMMAND_DEFAULT_VELOCITY_LIMIT_MRAD_S;
  MotorCommand_ClearLastStateError();
  MotorControl_Stop();
  taskEXIT_CRITICAL();
}

MotorCommandResult_t MotorCommand_SetEnable(uint8_t enabled)
{
  MotorCommandResult_t result = MOTOR_COMMAND_OK;

  taskENTER_CRITICAL();
  if (enabled == 0U)
  {
    g_enabled = 0U;
    g_target_milli = 0;
    MotorControl_Stop();
    MotorCommand_ClearLastStateError();
    taskEXIT_CRITICAL();
    return MOTOR_COMMAND_OK;
  }

  if (MotorControl_GetFault() != MOTOR_FAULT_NONE)
  {
    result = MotorCommand_RecordStateError();
    taskEXIT_CRITICAL();
    return result;
  }

  g_enabled = 1U;
  g_estop_latched = 0U;
  MotorCommand_ClearLastStateError();
  taskEXIT_CRITICAL();
  return MOTOR_COMMAND_OK;
}

MotorCommandResult_t MotorCommand_SetMode(CommProtocolMode_t mode)
{
  taskENTER_CRITICAL();
  g_mode = mode;
  if (mode == COMM_MODE_IDLE)
  {
    g_target_milli = 0;
    MotorControl_Stop();
    MotorCommand_ClearLastStateError();
    taskEXIT_CRITICAL();
    return MOTOR_COMMAND_OK;
  }

  MotorCommand_ApplyControlMode(mode);
  if (mode == COMM_MODE_POS)
  {
    g_target_milli = (int32_t)(((int64_t)MotorControl_GetPositionDegX10() *
                                MOTOR_COMMAND_PI_MILLI) / 1800LL);
    MotorControl_SetTargetPositionDegX10(MotorControl_GetPositionDegX10());
  }
  else
  {
    g_target_milli = 0;
    if (mode == COMM_MODE_VEL)
    {
      MotorControl_SetTargetRpmX10(0);
    }
  }

  MotorCommand_ClearLastStateError();
  taskEXIT_CRITICAL();
  return MOTOR_COMMAND_OK;
}

MotorCommandResult_t MotorCommand_SetTargetMilli(int32_t target_milli)
{
  CommProtocolMode_t mode;

  taskENTER_CRITICAL();
  mode = g_mode;
  taskEXIT_CRITICAL();

  switch (mode)
  {
    case COMM_MODE_OPEN:
      return MotorCommand_ApplyVoltageMv(target_milli, 0U);
    case COMM_MODE_VEL:
      return MotorCommand_SetVelocityMradS(target_milli);
    case COMM_MODE_POS:
      return MotorCommand_SetPositionMrad(target_milli);
    case COMM_MODE_IDLE:
    default:
      return MOTOR_COMMAND_ERR_STATE;
  }
}

static MotorCommandResult_t MotorCommand_ApplyVoltageMv(int32_t voltage_mv,
                                                        uint8_t use_foc_voltage)
{
  int32_t limited_mv;
  float duty;
  MotorCommandResult_t check;

  taskENTER_CRITICAL();
  check = MotorCommand_CheckCanOutput();
  if (check != MOTOR_COMMAND_OK)
  {
    taskEXIT_CRITICAL();
    return check;
  }

  if (MotorCommand_IsWithinSignedLimit(voltage_mv, g_voltage_limit_mv) == 0U)
  {
    taskEXIT_CRITICAL();
    return MOTOR_COMMAND_ERR_RANGE;
  }

  g_mode = COMM_MODE_OPEN;
  g_target_milli = voltage_mv;

  if (voltage_mv == 0)
  {
    MotorControl_Stop();
    MotorCommand_ClearLastStateError();
    taskEXIT_CRITICAL();
    return MOTOR_COMMAND_OK;
  }

  MotorCommand_SetDirectionForSignedValue(voltage_mv);
  limited_mv = MotorCommand_AbsI32(voltage_mv);
  duty = ((float)limited_mv * 100.0f) / (33.0f * (float)MOTOR_FOC_SUPPLY_MV);
  MotorControl_SetDuty(duty);
  MotorControl_SetMode((use_foc_voltage != 0U) ? MOTOR_MODE_FOC_VOLTAGE
                                                : MOTOR_MODE_OPEN_LOOP);
  check = MotorCommand_StartAndVerify();
  taskEXIT_CRITICAL();
  return check;
}

MotorCommandResult_t MotorCommand_SetVoltageMv(int32_t voltage_mv)
{
  return MotorCommand_ApplyVoltageMv(voltage_mv, 1U);
}

MotorCommandResult_t MotorCommand_SetVelocityMradS(int32_t velocity_mrad_s)
{
  int32_t limited_velocity;
  int32_t target_rpm_x10;
  MotorCommandResult_t check;

  taskENTER_CRITICAL();
  check = MotorCommand_CheckCanOutput();
  if (check != MOTOR_COMMAND_OK)
  {
    taskEXIT_CRITICAL();
    return check;
  }

  if (MotorCommand_IsWithinSignedLimit(velocity_mrad_s, g_velocity_limit_mrad_s) == 0U)
  {
    taskEXIT_CRITICAL();
    return MOTOR_COMMAND_ERR_RANGE;
  }

  limited_velocity = velocity_mrad_s;
  g_mode = COMM_MODE_VEL;
  g_target_milli = limited_velocity;

  if (limited_velocity == 0)
  {
    MotorControl_Stop();
    MotorCommand_ClearLastStateError();
    taskEXIT_CRITICAL();
    return MOTOR_COMMAND_OK;
  }

  MotorCommand_SetDirectionForSignedValue(limited_velocity);
  target_rpm_x10 = MotorCommand_MradS_ToRpmX10(MotorCommand_AbsI32(limited_velocity));
  MotorControl_SetTargetRpmX10(target_rpm_x10);
  MotorControl_SetMode(MOTOR_MODE_FOC_VELOCITY);
  check = MotorCommand_StartAndVerify();
  taskEXIT_CRITICAL();
  return check;
}

MotorCommandResult_t MotorCommand_SetPositionMrad(int32_t position_mrad)
{
  MotorCommandResult_t check;

  taskENTER_CRITICAL();
  check = MotorCommand_CheckCanOutput();
  if (check != MOTOR_COMMAND_OK)
  {
    taskEXIT_CRITICAL();
    return check;
  }

  if (MotorCommand_IsWithinSignedLimit(position_mrad, MOTOR_COMMAND_POSITION_LIMIT_MRAD) == 0U)
  {
    taskEXIT_CRITICAL();
    return MOTOR_COMMAND_ERR_RANGE;
  }

  g_mode = COMM_MODE_POS;
  g_target_milli = position_mrad;
  MotorControl_SetTargetPositionDegX10(MotorCommand_Mrad_ToDegX10(position_mrad));
  MotorControl_SetMode(MOTOR_MODE_FOC_POSITION);
  check = MotorCommand_StartAndVerify();
  taskEXIT_CRITICAL();
  return check;
}

MotorCommandResult_t MotorCommand_SetLimits(int32_t voltage_limit_mv,
                                            int32_t velocity_limit_mrad_s)
{
  if ((voltage_limit_mv < 0) || (velocity_limit_mrad_s < 0))
  {
    return MOTOR_COMMAND_ERR_ARG;
  }

  if ((voltage_limit_mv > MOTOR_COMMAND_MAX_VOLTAGE_LIMIT_MV) ||
      (velocity_limit_mrad_s > MOTOR_COMMAND_MAX_VELOCITY_LIMIT_MRAD_S))
  {
    return MOTOR_COMMAND_ERR_RANGE;
  }

  taskENTER_CRITICAL();
  g_voltage_limit_mv = voltage_limit_mv;
  g_velocity_limit_mrad_s = velocity_limit_mrad_s;
  MotorCommand_ClearLastStateError();
  taskEXIT_CRITICAL();
  return MOTOR_COMMAND_OK;
}

MotorCommandResult_t MotorCommand_Stop(void)
{
  taskENTER_CRITICAL();
  g_enabled = 0U;
  g_target_milli = 0;
  MotorControl_Stop();
  MotorCommand_ClearLastStateError();
  taskEXIT_CRITICAL();
  return MOTOR_COMMAND_OK;
}

MotorCommandResult_t MotorCommand_EStop(void)
{
  taskENTER_CRITICAL();
  g_enabled = 0U;
  g_estop_latched = 1U;
  g_target_milli = 0;
  MotorControl_Stop();
  MotorCommand_ClearLastStateError();
  taskEXIT_CRITICAL();
  return MOTOR_COMMAND_OK;
}

MotorCommandResult_t MotorCommand_ClearOrCalibrateZero(void)
{
  uint8_t calibrated;

  /*
   * FOC zero calibration deliberately waits for rotor settling, so it cannot
   * run inside a hard critical section. Suspending task switches keeps the
   * ControlTask/UI from touching motor_control while TIM2 HAL tick continues.
   */
  vTaskSuspendAll();
  calibrated = MotorControl_CalibrateFocZero();
  (void)xTaskResumeAll();

  if (calibrated == 0U)
  {
    return MotorCommand_RecordStateError();
  }
  MotorCommand_ClearLastStateError();
  return MOTOR_COMMAND_OK;
}

void MotorCommand_HandleCommTimeout(void)
{
  taskENTER_CRITICAL();
  g_enabled = 0U;
  g_target_milli = 0;
  MotorControl_Stop();
  MotorCommand_ClearLastStateError();
  taskEXIT_CRITICAL();
}

void MotorCommand_GetStatus(MotorCommandStatus_t *status)
{
  MotorFeedbackSnapshot_t feedback;

  if (status == 0)
  {
    return;
  }

  MotorFeedback_GetSnapshot(&feedback);
  taskENTER_CRITICAL();
  status->enabled = g_enabled;
  status->estop_latched = g_estop_latched;
  status->mode = g_mode;
  status->target_milli = g_target_milli;
  status->voltage_limit_mv = g_voltage_limit_mv;
  status->velocity_limit_mrad_s = g_velocity_limit_mrad_s;
  status->state = MotorControl_GetState();
  status->stop_reason = MotorControl_GetStopReason();
  status->fault = MotorControl_GetFault();
  status->foc_zero_calibrated = MotorControl_IsFocZeroCalibrated();
  status->angle_mrad = (feedback.angle_valid != 0U)
                         ? MotorCommand_DegX100_ToMrad(feedback.angle_x100)
                         : 0;
  status->velocity_mrad_s = (feedback.speed_valid != 0U)
                              ? MotorCommand_RpmX10_ToMradS(feedback.rpm_x10)
                              : 0;
  status->raw_angle = feedback.raw_angle;
  status->magnet_detected = feedback.magnet_detected;
  status->magnet_too_weak = feedback.magnet_too_weak;
  status->magnet_too_strong = feedback.magnet_too_strong;
  status->foc_uq_mv = MotorControl_GetFocUqMv();
  taskEXIT_CRITICAL();
}

uint8_t MotorCommand_IsEnabled(void)
{
  uint8_t enabled;

  taskENTER_CRITICAL();
  enabled = g_enabled;
  taskEXIT_CRITICAL();
  return enabled;
}

MotorStopReason_t MotorCommand_GetLastErrorStopReason(void)
{
  MotorStopReason_t reason;

  taskENTER_CRITICAL();
  reason = g_last_error_stop_reason;
  taskEXIT_CRITICAL();
  return reason;
}

MotorFault_t MotorCommand_GetLastErrorFault(void)
{
  MotorFault_t fault;

  taskENTER_CRITICAL();
  fault = g_last_error_fault;
  taskEXIT_CRITICAL();
  return fault;
}

const char *MotorCommand_ResultCode(MotorCommandResult_t result)
{
  switch (result)
  {
    case MOTOR_COMMAND_ERR_ARG: return "ARG";
    case MOTOR_COMMAND_ERR_DISABLED: return "DISABLED";
    case MOTOR_COMMAND_ERR_ESTOP: return "ESTOP";
    case MOTOR_COMMAND_ERR_UNSUPPORTED: return "UNSUPPORTED";
    case MOTOR_COMMAND_ERR_RANGE: return "RANGE";
    case MOTOR_COMMAND_ERR_STATE: return "STATE";
    case MOTOR_COMMAND_OK:
    default:
      return "OK";
  }
}

const char *MotorCommand_ResultMessage(MotorCommandResult_t result)
{
  switch (result)
  {
    case MOTOR_COMMAND_ERR_ARG: return "bad argument";
    case MOTOR_COMMAND_ERR_DISABLED: return "motor disabled";
    case MOTOR_COMMAND_ERR_ESTOP: return "estop latched";
    case MOTOR_COMMAND_ERR_UNSUPPORTED: return "unsupported";
    case MOTOR_COMMAND_ERR_RANGE: return "out of range";
    case MOTOR_COMMAND_ERR_STATE: return "invalid state";
    case MOTOR_COMMAND_OK:
    default:
      return "ok";
  }
}

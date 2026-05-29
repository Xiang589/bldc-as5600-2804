#include "comm_task.h"

#include <string.h>

#include "cmsis_os.h"
#include "comm_protocol.h"
#include "comm_uart_dma.h"
#include "main.h"
#include "motor_command.h"

#define COMM_TASK_POLL_MS 2U
#define COMM_TASK_WATCHDOG_MS 1000U
#define COMM_TASK_TX_RETRY_DELAY_MS 1U
#define COMM_TASK_STREAM_MIN_MS 20
#define COMM_TASK_STREAM_MAX_MS 5000

static uint32_t g_last_control_tick = 0U;
static uint32_t g_stream_period_ms = 0U;
static uint32_t g_last_stream_tick = 0U;

static void CommTask_AppendStr(char *line, size_t line_size, size_t *pos, const char *text)
{
  while ((*text != '\0') && ((*pos + 1U) < line_size))
  {
    line[*pos] = *text;
    (*pos)++;
    text++;
  }
  if (line_size > 0U)
  {
    line[*pos] = '\0';
  }
}

static void CommTask_AppendU32(char *line, size_t line_size, size_t *pos, uint32_t value)
{
  char tmp[10];
  uint8_t count = 0U;

  do
  {
    tmp[count++] = (char)('0' + (value % 10U));
    value /= 10U;
  } while ((value != 0U) && (count < sizeof(tmp)));

  while (count > 0U)
  {
    count--;
    if ((*pos + 1U) >= line_size)
    {
      break;
    }
    line[*pos] = tmp[count];
    (*pos)++;
  }
  if (line_size > 0U)
  {
    line[*pos] = '\0';
  }
}

static void CommTask_AppendI32(char *line, size_t line_size, size_t *pos, int32_t value)
{
  uint32_t magnitude;

  if (value < 0)
  {
    CommTask_AppendStr(line, line_size, pos, "-");
    magnitude = (uint32_t)(-(value + 1)) + 1U;
  }
  else
  {
    magnitude = (uint32_t)value;
  }
  CommTask_AppendU32(line, line_size, pos, magnitude);
}

static void CommTask_AppendFieldI32(char *line,
                                    size_t line_size,
                                    size_t *pos,
                                    const char *name,
                                    int32_t value)
{
  CommTask_AppendStr(line, line_size, pos, " ");
  CommTask_AppendStr(line, line_size, pos, name);
  CommTask_AppendStr(line, line_size, pos, "=");
  CommTask_AppendI32(line, line_size, pos, value);
}

static void CommTask_AppendFieldU32(char *line,
                                    size_t line_size,
                                    size_t *pos,
                                    const char *name,
                                    uint32_t value)
{
  CommTask_AppendStr(line, line_size, pos, " ");
  CommTask_AppendStr(line, line_size, pos, name);
  CommTask_AppendStr(line, line_size, pos, "=");
  CommTask_AppendU32(line, line_size, pos, value);
}

static void CommTask_FinishLine(char *line, size_t line_size, size_t *pos)
{
  CommTask_AppendStr(line, line_size, pos, "\r\n");
}

static void CommTask_SendReliable(const char *text)
{
  size_t len = strlen(text);
  while (CommUartDma_Transmit(text, len) == 0U)
  {
    osDelay(COMM_TASK_TX_RETRY_DELAY_MS);
  }
}

static void CommTask_SendBestEffort(const char *text)
{
  (void)CommUartDma_Transmit(text, strlen(text));
}

static void CommTask_SendError(const char *code, const char *message)
{
  char line[96];
  size_t pos = 0U;

  CommTask_AppendStr(line, sizeof(line), &pos, "ERR ");
  CommTask_AppendStr(line, sizeof(line), &pos, code);
  CommTask_AppendStr(line, sizeof(line), &pos, " ");
  CommTask_AppendStr(line, sizeof(line), &pos, message);
  CommTask_FinishLine(line, sizeof(line), &pos);
  CommTask_SendReliable(line);
}

static void CommTask_SendMotorResult(CommCommandType_t type, MotorCommandResult_t result)
{
  char line[96];

  if (result != MOTOR_COMMAND_OK)
  {
    CommTask_SendError(MotorCommand_ResultCode(result),
                       MotorCommand_ResultMessage(result));
    return;
  }

  size_t pos = 0U;
  CommTask_AppendStr(line, sizeof(line), &pos, "OK ");
  CommTask_AppendStr(line, sizeof(line), &pos, CommProtocol_CommandName(type));
  CommTask_FinishLine(line, sizeof(line), &pos);
  CommTask_SendReliable(line);
}

static void CommTask_FormatStatus(char *line, size_t line_size, const char *prefix)
{
  MotorCommandStatus_t status;
  size_t pos = 0U;
  MotorCommand_GetStatus(&status);

  CommTask_AppendStr(line, line_size, &pos, prefix);
  CommTask_AppendFieldU32(line, line_size, &pos, "en", status.enabled);
  CommTask_AppendStr(line, line_size, &pos, " mode=");
  CommTask_AppendStr(line, line_size, &pos, CommProtocol_ModeName(status.mode));
  CommTask_AppendFieldI32(line, line_size, &pos, "target", status.target_milli);
  CommTask_AppendFieldI32(line, line_size, &pos, "angle", status.angle_mrad);
  CommTask_AppendFieldI32(line, line_size, &pos, "vel", status.velocity_mrad_s);
  CommTask_AppendFieldU32(line, line_size, &pos, "raw", status.raw_angle);
  CommTask_AppendFieldU32(line, line_size, &pos, "md", status.magnet_detected);
  CommTask_AppendFieldU32(line, line_size, &pos, "ml", status.magnet_too_weak);
  CommTask_AppendFieldU32(line, line_size, &pos, "mh", status.magnet_too_strong);
  CommTask_AppendFieldI32(line, line_size, &pos, "vlim", status.voltage_limit_mv);
  CommTask_AppendFieldI32(line, line_size, &pos, "wlim", status.velocity_limit_mrad_s);
  CommTask_AppendFieldI32(line, line_size, &pos, "uq", status.foc_uq_mv);
  CommTask_AppendFieldU32(line, line_size, &pos, "z", status.foc_zero_calibrated);
  CommTask_FinishLine(line, line_size, &pos);
}

static uint8_t CommTask_IsControlCommand(CommCommandType_t type,
                                         MotorCommandResult_t result)
{
  if (result != MOTOR_COMMAND_OK)
  {
    return 0U;
  }

  switch (type)
  {
    case COMM_CMD_ENABLE:
    case COMM_CMD_MODE:
    case COMM_CMD_TARGET:
    case COMM_CMD_VOLT:
    case COMM_CMD_VEL:
    case COMM_CMD_POS:
    case COMM_CMD_LIMIT:
    case COMM_CMD_STOP:
    case COMM_CMD_ESTOP:
    case COMM_CMD_ZERO:
    case COMM_CMD_KEEPALIVE:
      return 1U;
    default:
      return 0U;
  }
}

static MotorCommandResult_t CommTask_ExecuteCommand(const CommProtocolCommand_t *command)
{
  MotorCommandResult_t result = MOTOR_COMMAND_OK;
  char line[192];

  switch (command->type)
  {
    case COMM_CMD_PING:
      CommTask_SendReliable("OK PING\r\n");
      break;
    case COMM_CMD_HELP:
      CommTask_SendReliable("OK HELP PING HELP ID? STATUS? ENABLE MODE TARGET VOLT VEL POS LIMIT PIDV PIDP STREAM STOP ESTOP ZERO KEEPALIVE\r\n");
      break;
    case COMM_CMD_ID:
      CommTask_SendReliable("OK ID stm32-as5600-bldc 0.1\r\n");
      break;
    case COMM_CMD_STATUS:
      CommTask_FormatStatus(line, sizeof(line), "OK STATUS");
      CommTask_SendReliable(line);
      break;
    case COMM_CMD_ENABLE:
      result = MotorCommand_SetEnable((uint8_t)command->value[0]);
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_MODE:
      result = MotorCommand_SetMode(command->mode);
      if (result == MOTOR_COMMAND_OK)
      {
        size_t pos = 0U;
        CommTask_AppendStr(line, sizeof(line), &pos, "OK MODE ");
        CommTask_AppendStr(line, sizeof(line), &pos,
                           CommProtocol_ModeName(command->mode));
        CommTask_FinishLine(line, sizeof(line), &pos);
        CommTask_SendReliable(line);
      }
      else
      {
        CommTask_SendMotorResult(command->type, result);
      }
      break;
    case COMM_CMD_TARGET:
      result = MotorCommand_SetTargetMilli(command->value[0]);
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_VOLT:
      result = MotorCommand_SetVoltageMv(command->value[0]);
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_VEL:
      result = MotorCommand_SetVelocityMradS(command->value[0]);
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_POS:
      result = MotorCommand_SetPositionMrad(command->value[0]);
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_LIMIT:
      result = MotorCommand_SetLimits(command->value[0], command->value[1]);
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_PIDV:
    case COMM_CMD_PIDP:
      result = MOTOR_COMMAND_ERR_UNSUPPORTED;
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_STREAM:
      if ((command->value[0] != 0) &&
          ((command->value[0] < COMM_TASK_STREAM_MIN_MS) ||
           (command->value[0] > COMM_TASK_STREAM_MAX_MS)))
      {
        result = MOTOR_COMMAND_ERR_ARG;
        CommTask_SendMotorResult(command->type, result);
      }
      else
      {
        g_stream_period_ms = (uint32_t)command->value[0];
        g_last_stream_tick = HAL_GetTick();
        CommTask_SendMotorResult(command->type, result);
      }
      break;
    case COMM_CMD_STOP:
      result = MotorCommand_Stop();
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_ESTOP:
      result = MotorCommand_EStop();
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_ZERO:
      result = MotorCommand_ClearOrCalibrateZero();
      CommTask_SendMotorResult(command->type, result);
      break;
    case COMM_CMD_KEEPALIVE:
      CommTask_SendReliable("OK KEEPALIVE\r\n");
      break;
    case COMM_CMD_NONE:
    default:
      result = MOTOR_COMMAND_ERR_ARG;
      CommTask_SendMotorResult(command->type, result);
      break;
  }

  if (CommTask_IsControlCommand(command->type, result) != 0U)
  {
    g_last_control_tick = HAL_GetTick();
  }

  return result;
}

static void CommTask_ProcessLine(char *line)
{
  CommProtocolCommand_t command;
  CommParseResult_t parse_result = CommProtocol_ParseLine(line, &command);

  switch (parse_result)
  {
    case COMM_PARSE_OK:
      (void)CommTask_ExecuteCommand(&command);
      break;
    case COMM_PARSE_EMPTY:
      break;
    case COMM_PARSE_UNKNOWN:
      CommTask_SendError("CMD", "unknown command");
      break;
    case COMM_PARSE_ARG:
    default:
      CommTask_SendError("ARG", "bad argument");
      break;
  }
}

static void CommTask_CheckWatchdog(uint32_t now)
{
  if ((MotorCommand_IsEnabled() != 0U) &&
      ((now - g_last_control_tick) >= COMM_TASK_WATCHDOG_MS))
  {
    MotorCommand_HandleCommTimeout();
    CommTask_SendReliable("EVT COMM_TIMEOUT\r\n");
    g_last_control_tick = now;
  }
}

static void CommTask_CheckStream(uint32_t now)
{
  char line[192];

  if (g_stream_period_ms == 0U)
  {
    return;
  }

  if ((now - g_last_stream_tick) < g_stream_period_ms)
  {
    return;
  }

  g_last_stream_tick = now;
  CommTask_FormatStatus(line, sizeof(line), "EVT STATUS");
  CommTask_SendBestEffort(line);
}

void CommTask_Run(void)
{
  char line[COMM_PROTOCOL_MAX_LINE_LEN + 1U];
  uint16_t line_len = 0U;
  uint8_t line_overflow = 0U;
  uint8_t byte;

  MotorCommand_Init();
  if (CommUartDma_Init() == 0U)
  {
    CommTask_SendError("UART", "dma init failed");
  }
  else
  {
    CommTask_SendReliable("EVT READY\r\n");
  }
  g_last_control_tick = HAL_GetTick();

  for (;;)
  {
    CommUartDma_PollRx();
    while (CommUartDma_GetByte(&byte) != 0U)
    {
      if ((byte == '\n') || (byte == '\r'))
      {
        if (line_overflow != 0U)
        {
          CommTask_SendError("ARG", "line too long");
          line_overflow = 0U;
          line_len = 0U;
        }
        else if (line_len > 0U)
        {
          line[line_len] = '\0';
          CommTask_ProcessLine(line);
          line_len = 0U;
        }
      }
      else if (line_overflow == 0U)
      {
        if (line_len >= COMM_PROTOCOL_MAX_LINE_LEN)
        {
          line_overflow = 1U;
          line_len = 0U;
        }
        else
        {
          line[line_len++] = (char)byte;
        }
      }
    }

    {
      uint32_t now = HAL_GetTick();
      CommTask_CheckWatchdog(now);
      CommTask_CheckStream(now);
    }
    osDelay(COMM_TASK_POLL_MS);
  }
}

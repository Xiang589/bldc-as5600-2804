#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COMM_PROTOCOL_MAX_LINE_LEN 128U

typedef enum {
  COMM_PARSE_OK = 0,
  COMM_PARSE_EMPTY,
  COMM_PARSE_UNKNOWN,
  COMM_PARSE_ARG
} CommParseResult_t;

typedef enum {
  COMM_CMD_NONE = 0,
  COMM_CMD_PING,
  COMM_CMD_HELP,
  COMM_CMD_ID,
  COMM_CMD_STATUS,
  COMM_CMD_ENABLE,
  COMM_CMD_MODE,
  COMM_CMD_TARGET,
  COMM_CMD_VOLT,
  COMM_CMD_VEL,
  COMM_CMD_POS,
  COMM_CMD_LIMIT,
  COMM_CMD_PIDV,
  COMM_CMD_PIDP,
  COMM_CMD_STREAM,
  COMM_CMD_STOP,
  COMM_CMD_ESTOP,
  COMM_CMD_ZERO,
  COMM_CMD_KEEPALIVE
} CommCommandType_t;

typedef enum {
  COMM_MODE_IDLE = 0,
  COMM_MODE_OPEN,
  COMM_MODE_VEL,
  COMM_MODE_POS
} CommProtocolMode_t;

typedef struct {
  CommCommandType_t type;
  CommProtocolMode_t mode;
  int32_t value[5];
  uint8_t value_count;
} CommProtocolCommand_t;

CommParseResult_t CommProtocol_ParseLine(char *line, CommProtocolCommand_t *command);
const char *CommProtocol_CommandName(CommCommandType_t type);
const char *CommProtocol_ModeName(CommProtocolMode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* COMM_PROTOCOL_H */

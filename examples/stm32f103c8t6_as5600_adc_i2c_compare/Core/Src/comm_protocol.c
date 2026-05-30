#include "comm_protocol.h"

#include <stddef.h>

#define COMM_PROTOCOL_MAX_TOKENS 8U

static uint8_t CommProtocol_IsSpace(char ch)
{
  return ((ch == ' ') || (ch == '\t')) ? 1U : 0U;
}

static char CommProtocol_ToUpper(char ch)
{
  if ((ch >= 'a') && (ch <= 'z'))
  {
    return (char)(ch - ('a' - 'A'));
  }
  return ch;
}

static uint8_t CommProtocol_Equals(const char *a, const char *b)
{
  while ((*a != '\0') && (*b != '\0'))
  {
    if (CommProtocol_ToUpper(*a) != CommProtocol_ToUpper(*b))
    {
      return 0U;
    }
    a++;
    b++;
  }
  return ((*a == '\0') && (*b == '\0')) ? 1U : 0U;
}

static uint8_t CommProtocol_ParseInt(const char *text, int32_t *out)
{
  int32_t sign = 1;
  int32_t value = 0;
  uint8_t digits = 0U;

  if ((text == NULL) || (out == NULL) || (*text == '\0'))
  {
    return 0U;
  }

  if (*text == '-')
  {
    sign = -1;
    text++;
  }
  else if (*text == '+')
  {
    text++;
  }

  while (*text != '\0')
  {
    if ((*text < '0') || (*text > '9'))
    {
      return 0U;
    }
    value = (value * 10) + (int32_t)(*text - '0');
    digits = 1U;
    text++;
  }

  if (digits == 0U)
  {
    return 0U;
  }

  *out = value * sign;
  return 1U;
}

static uint8_t CommProtocol_ParseMilli(const char *text, int32_t *out)
{
  int32_t sign = 1;
  int32_t whole = 0;
  int32_t frac = 0;
  uint8_t digits = 0U;
  uint8_t frac_digits = 0U;

  if ((text == NULL) || (out == NULL) || (*text == '\0'))
  {
    return 0U;
  }

  if (*text == '-')
  {
    sign = -1;
    text++;
  }
  else if (*text == '+')
  {
    text++;
  }

  while ((*text >= '0') && (*text <= '9'))
  {
    whole = (whole * 10) + (int32_t)(*text - '0');
    digits = 1U;
    text++;
  }

  if (*text == '.')
  {
    text++;
    while ((*text >= '0') && (*text <= '9'))
    {
      if (frac_digits < 3U)
      {
        frac = (frac * 10) + (int32_t)(*text - '0');
        frac_digits++;
      }
      text++;
      digits = 1U;
    }
  }

  if ((*text != '\0') || (digits == 0U))
  {
    return 0U;
  }

  while (frac_digits < 3U)
  {
    frac *= 10;
    frac_digits++;
  }

  *out = ((whole * 1000) + frac) * sign;
  return 1U;
}

static uint8_t CommProtocol_Tokenize(char *line, char *tokens[], uint8_t max_tokens)
{
  uint8_t count = 0U;

  while (*line != '\0')
  {
    while (CommProtocol_IsSpace(*line) != 0U)
    {
      line++;
    }

    if (*line == '\0')
    {
      break;
    }

    if (count >= max_tokens)
    {
      return count;
    }

    tokens[count++] = line;
    while ((*line != '\0') && (CommProtocol_IsSpace(*line) == 0U))
    {
      line++;
    }
    if (*line != '\0')
    {
      *line = '\0';
      line++;
    }
  }

  return count;
}

static uint8_t CommProtocol_ParseMode(const char *text, CommProtocolMode_t *mode)
{
  if (CommProtocol_Equals(text, "IDLE") != 0U)
  {
    *mode = COMM_MODE_IDLE;
    return 1U;
  }
  if (CommProtocol_Equals(text, "OPEN") != 0U)
  {
    *mode = COMM_MODE_OPEN;
    return 1U;
  }
  if (CommProtocol_Equals(text, "VEL") != 0U)
  {
    *mode = COMM_MODE_VEL;
    return 1U;
  }
  if (CommProtocol_Equals(text, "POS") != 0U)
  {
    *mode = COMM_MODE_POS;
    return 1U;
  }
  return 0U;
}

static CommParseResult_t CommProtocol_ParseMilliArgs(char *tokens[],
                                                     uint8_t token_count,
                                                     uint8_t first_arg,
                                                     uint8_t min_args,
                                                     uint8_t max_args,
                                                     CommProtocolCommand_t *command)
{
  uint8_t count;
  uint8_t i;

  if (token_count < first_arg)
  {
    return COMM_PARSE_ARG;
  }

  count = (uint8_t)(token_count - first_arg);
  if ((count < min_args) || (count > max_args))
  {
    return COMM_PARSE_ARG;
  }

  for (i = 0U; i < count; i++)
  {
    if (CommProtocol_ParseMilli(tokens[first_arg + i], &command->value[i]) == 0U)
    {
      return COMM_PARSE_ARG;
    }
  }
  command->value_count = count;
  return COMM_PARSE_OK;
}

CommParseResult_t CommProtocol_ParseLine(char *line, CommProtocolCommand_t *command)
{
  char *tokens[COMM_PROTOCOL_MAX_TOKENS];
  uint8_t token_count;
  int32_t ivalue;

  if ((line == NULL) || (command == NULL))
  {
    return COMM_PARSE_ARG;
  }

  command->type = COMM_CMD_NONE;
  command->mode = COMM_MODE_IDLE;
  command->value_count = 0U;
  for (uint8_t i = 0U; i < 5U; i++)
  {
    command->value[i] = 0;
  }

  token_count = CommProtocol_Tokenize(line, tokens, COMM_PROTOCOL_MAX_TOKENS);
  if (token_count == 0U)
  {
    return COMM_PARSE_EMPTY;
  }

  if (CommProtocol_Equals(tokens[0], "PING") != 0U)
  {
    command->type = COMM_CMD_PING;
    return (token_count == 1U) ? COMM_PARSE_OK : COMM_PARSE_ARG;
  }
  if (CommProtocol_Equals(tokens[0], "HELP") != 0U)
  {
    command->type = COMM_CMD_HELP;
    return (token_count == 1U) ? COMM_PARSE_OK : COMM_PARSE_ARG;
  }
  if (CommProtocol_Equals(tokens[0], "ID?") != 0U)
  {
    command->type = COMM_CMD_ID;
    return (token_count == 1U) ? COMM_PARSE_OK : COMM_PARSE_ARG;
  }
  if (CommProtocol_Equals(tokens[0], "STATUS?") != 0U)
  {
    command->type = COMM_CMD_STATUS;
    return (token_count == 1U) ? COMM_PARSE_OK : COMM_PARSE_ARG;
  }
  if (CommProtocol_Equals(tokens[0], "STOP") != 0U)
  {
    command->type = COMM_CMD_STOP;
    return (token_count == 1U) ? COMM_PARSE_OK : COMM_PARSE_ARG;
  }
  if (CommProtocol_Equals(tokens[0], "ESTOP") != 0U)
  {
    command->type = COMM_CMD_ESTOP;
    return (token_count == 1U) ? COMM_PARSE_OK : COMM_PARSE_ARG;
  }
  if (CommProtocol_Equals(tokens[0], "ZERO") != 0U)
  {
    command->type = COMM_CMD_ZERO;
    return (token_count == 1U) ? COMM_PARSE_OK : COMM_PARSE_ARG;
  }
  if (CommProtocol_Equals(tokens[0], "KEEPALIVE") != 0U)
  {
    command->type = COMM_CMD_KEEPALIVE;
    return (token_count == 1U) ? COMM_PARSE_OK : COMM_PARSE_ARG;
  }

  if (CommProtocol_Equals(tokens[0], "ENABLE") != 0U)
  {
    command->type = COMM_CMD_ENABLE;
    if ((token_count != 2U) || (CommProtocol_ParseInt(tokens[1], &ivalue) == 0U) ||
        ((ivalue != 0) && (ivalue != 1)))
    {
      return COMM_PARSE_ARG;
    }
    command->value[0] = ivalue;
    command->value_count = 1U;
    return COMM_PARSE_OK;
  }

  if (CommProtocol_Equals(tokens[0], "MODE") != 0U)
  {
    command->type = COMM_CMD_MODE;
    if ((token_count != 2U) || (CommProtocol_ParseMode(tokens[1], &command->mode) == 0U))
    {
      return COMM_PARSE_ARG;
    }
    return COMM_PARSE_OK;
  }

  if (CommProtocol_Equals(tokens[0], "TARGET") != 0U)
  {
    command->type = COMM_CMD_TARGET;
    return CommProtocol_ParseMilliArgs(tokens, token_count, 1U, 1U, 1U, command);
  }
  if (CommProtocol_Equals(tokens[0], "VOLT") != 0U)
  {
    command->type = COMM_CMD_VOLT;
    return CommProtocol_ParseMilliArgs(tokens, token_count, 1U, 1U, 1U, command);
  }
  if (CommProtocol_Equals(tokens[0], "VEL") != 0U)
  {
    command->type = COMM_CMD_VEL;
    return CommProtocol_ParseMilliArgs(tokens, token_count, 1U, 1U, 1U, command);
  }
  if (CommProtocol_Equals(tokens[0], "POS") != 0U)
  {
    command->type = COMM_CMD_POS;
    return CommProtocol_ParseMilliArgs(tokens, token_count, 1U, 1U, 1U, command);
  }
  if (CommProtocol_Equals(tokens[0], "LIMIT") != 0U)
  {
    command->type = COMM_CMD_LIMIT;
    return CommProtocol_ParseMilliArgs(tokens, token_count, 1U, 2U, 2U, command);
  }
  if (CommProtocol_Equals(tokens[0], "PIDV") != 0U)
  {
    command->type = COMM_CMD_PIDV;
    return CommProtocol_ParseMilliArgs(tokens, token_count, 1U, 3U, 5U, command);
  }
  if (CommProtocol_Equals(tokens[0], "PIDP") != 0U)
  {
    command->type = COMM_CMD_PIDP;
    return CommProtocol_ParseMilliArgs(tokens, token_count, 1U, 1U, 3U, command);
  }
  if (CommProtocol_Equals(tokens[0], "STREAM") != 0U)
  {
    command->type = COMM_CMD_STREAM;
    if ((token_count != 2U) || (CommProtocol_ParseInt(tokens[1], &ivalue) == 0U))
    {
      return COMM_PARSE_ARG;
    }
    command->value[0] = ivalue;
    command->value_count = 1U;
    return COMM_PARSE_OK;
  }

  return COMM_PARSE_UNKNOWN;
}

const char *CommProtocol_CommandName(CommCommandType_t type)
{
  switch (type)
  {
    case COMM_CMD_PING: return "PING";
    case COMM_CMD_HELP: return "HELP";
    case COMM_CMD_ID: return "ID";
    case COMM_CMD_STATUS: return "STATUS";
    case COMM_CMD_ENABLE: return "ENABLE";
    case COMM_CMD_MODE: return "MODE";
    case COMM_CMD_TARGET: return "TARGET";
    case COMM_CMD_VOLT: return "VOLT";
    case COMM_CMD_VEL: return "VEL";
    case COMM_CMD_POS: return "POS";
    case COMM_CMD_LIMIT: return "LIMIT";
    case COMM_CMD_PIDV: return "PIDV";
    case COMM_CMD_PIDP: return "PIDP";
    case COMM_CMD_STREAM: return "STREAM";
    case COMM_CMD_STOP: return "STOP";
    case COMM_CMD_ESTOP: return "ESTOP";
    case COMM_CMD_ZERO: return "ZERO";
    case COMM_CMD_KEEPALIVE: return "KEEPALIVE";
    default: return "NONE";
  }
}

const char *CommProtocol_ModeName(CommProtocolMode_t mode)
{
  switch (mode)
  {
    case COMM_MODE_OPEN: return "OPEN";
    case COMM_MODE_VEL: return "VEL";
    case COMM_MODE_POS: return "POS";
    case COMM_MODE_IDLE:
    default:
      return "IDLE";
  }
}

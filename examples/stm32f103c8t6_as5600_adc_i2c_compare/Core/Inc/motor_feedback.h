#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void MotorFeedback_Init(void);
void MotorFeedback_Update(uint32_t now);

typedef struct {
  uint8_t angle_valid;
  uint8_t speed_valid;
  uint16_t raw_angle;
  int32_t angle_x100;
  int32_t rpm_x10;
  int32_t delta_raw;
  int32_t total_raw_turns;
  uint32_t last_ok_tick;
  uint32_t last_error_tick;
  uint32_t update_count;
  uint32_t error_count;
  uint32_t consecutive_error_count;
  HAL_StatusTypeDef last_hal_status;
} MotorFeedbackSnapshot_t;

void MotorFeedback_GetSnapshot(MotorFeedbackSnapshot_t *snapshot);

uint8_t MotorFeedback_IsAngleValid(void);
uint8_t MotorFeedback_IsSpeedValid(void);

uint16_t MotorFeedback_GetRawAngle(void);
int32_t MotorFeedback_GetAngleX100(void);
int32_t MotorFeedback_GetRpmX10(void);
int32_t MotorFeedback_GetDeltaRaw(void);
int32_t MotorFeedback_GetTotalRawTurns(void);

#ifdef __cplusplus
}
#endif

#endif

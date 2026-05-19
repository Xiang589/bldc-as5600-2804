#ifndef MOTOR_FEEDBACK_H
#define MOTOR_FEEDBACK_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void MotorFeedback_Init(void);
void MotorFeedback_Update(uint32_t now);

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

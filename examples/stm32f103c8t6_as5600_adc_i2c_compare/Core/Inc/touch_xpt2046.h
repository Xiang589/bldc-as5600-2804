#ifndef TOUCH_XPT2046_H
#define TOUCH_XPT2046_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  int32_t ax;
  int32_t bx;
  int32_t cx;
  int32_t ay;
  int32_t by;
  int32_t cy;
} TouchCalibration_t;

void Touch_Init(void);
uint8_t Touch_IsPressed(void);
HAL_StatusTypeDef Touch_ReadRawStatus(uint16_t *raw_x, uint16_t *raw_y);
HAL_StatusTypeDef Touch_ReadPointStatus(uint16_t *x, uint16_t *y);
uint8_t Touch_ReadRaw(uint16_t *raw_x, uint16_t *raw_y);
uint8_t Touch_ReadPoint(uint16_t *x, uint16_t *y);
void Touch_SetCalibration(const TouchCalibration_t *cal);
void Touch_GetCalibration(TouchCalibration_t *cal);
void Touch_LoadDefaultCalibration(void);
uint8_t Touch_MapRawToPoint(uint16_t raw_x, uint16_t raw_y,
                            const TouchCalibration_t *cal,
                            uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif

#endif

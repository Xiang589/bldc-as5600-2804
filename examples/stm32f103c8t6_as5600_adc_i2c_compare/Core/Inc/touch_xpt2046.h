#ifndef TOUCH_XPT2046_H
#define TOUCH_XPT2046_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint16_t raw_x_min;
  uint16_t raw_x_max;
  uint16_t raw_y_min;
  uint16_t raw_y_max;
  uint8_t swap_xy;
  uint8_t invert_x;
  uint8_t invert_y;
} TouchCalibration_t;

void Touch_Init(void);
uint8_t Touch_IsPressed(void);
uint8_t Touch_ReadRaw(uint16_t *raw_x, uint16_t *raw_y);
uint8_t Touch_ReadPoint(uint16_t *x, uint16_t *y);
void Touch_SetCalibration(const TouchCalibration_t *cal);
void Touch_GetCalibration(TouchCalibration_t *cal);
void Touch_LoadDefaultCalibration(void);

#ifdef __cplusplus
}
#endif

#endif

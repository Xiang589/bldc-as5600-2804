#ifndef TOUCH_XPT2046_H
#define TOUCH_XPT2046_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Touch_Init(void);
uint8_t Touch_IsPressed(void);
uint8_t Touch_ReadRaw(uint16_t *raw_x, uint16_t *raw_y);
uint8_t Touch_ReadPoint(uint16_t *x, uint16_t *y);

#ifdef __cplusplus
}
#endif

#endif

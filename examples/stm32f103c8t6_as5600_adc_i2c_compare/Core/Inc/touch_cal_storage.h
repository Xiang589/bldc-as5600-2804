#ifndef TOUCH_CAL_STORAGE_H
#define TOUCH_CAL_STORAGE_H

#include "touch_xpt2046.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t TouchCalStorage_Load(TouchCalibration_t *cal);
uint8_t TouchCalStorage_Save(const TouchCalibration_t *cal);

#ifdef __cplusplus
}
#endif

#endif

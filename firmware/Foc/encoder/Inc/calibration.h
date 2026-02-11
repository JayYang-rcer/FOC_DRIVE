#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "as5047p.h"
#ifdef __cplusplus
extern "C" {
#endif
void calibrate_mt_encoder(float vd_set, float pos);
#ifdef __cplusplus
}
#endif
#endif


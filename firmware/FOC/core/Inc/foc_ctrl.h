#ifndef __FOC_CTRL_H
#define __FOC_CTRL_H
#include "foc_cfg.h"
#include "adc.h"

void FocVolt(float vd_ref, float vq_ref, float pos);
void Encoder_Idle(void);
void FocHandle(void);
extern volatile int spd_set;
#endif

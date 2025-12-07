#ifndef __FOC_CTRL_H
#define __FOC_CTRL_H

#include "foc_cfg.h"
#include "adc.h"

void FocVolt(float vd_ref, float vq_ref, float pos);

void Encoder_Idle(void);

void FocHandle(void);

void FocCurrent(float id_set, float iq_set, float pos);

_RAM_FUNC void HfiVolt(float vd, float vq, float pos);

_RAM_FUNC void HfiCurrent(float id_set, float iq_set, float pos);

#endif

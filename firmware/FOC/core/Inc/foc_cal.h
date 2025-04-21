#ifndef __FOC_CAL_H
#define __FOC_CAL_H

#include "util.h"
#include "foc_cfg.h"

void SinCosVal(foc_param_t *foc);
void Clarke(foc_param_t *foc);
void Park(foc_param_t *foc);
void InvPark(foc_param_t *foc);
void InvClarke(foc_param_t *foc);
int SvpwmSector(foc_param_t *foc);
int svpwm(foc_param_t *foc);

#endif

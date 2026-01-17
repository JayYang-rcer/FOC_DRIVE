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

/*********************************** smo *******************************/
void SmoInit(smo_param_t *smo);

float SmoViewer(foc_param_t *foc, smo_param_t *smo);

void SmoParamInit(smo_param_t *smo);
/*********************************** smo *******************************/

/*********************************** hfi *******************************/
float HfiInjectSign(float inject_U);
float HfiPllAngle(pll_t *pll, hfi_param_t *hfi);
void HfiAngleCalc(foc_param_t *foc, hfi_param_t *hfi);
void IdqToIdqF(foc_param_t *foc, hfi_param_t *hfi);
void IdqToIdqH(foc_param_t *foc, hfi_param_t *hfi);
bool HfiNsIdentify(hfi_param_t *hfi, foc_param_t *foc);
int16_t non_flux_observer(non_flux_t* flux, foc_param_t* foc, motor_cfg_t *motor);
/*********************************** hfi *******************************/

extern smo_param_t smo_param;
extern volatile float smo_angle;

#endif

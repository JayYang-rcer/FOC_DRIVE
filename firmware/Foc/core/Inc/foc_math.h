#ifndef __FOC_CAL_H
#define __FOC_CAL_H

#include "util.h"
#include "foc_cfg.h"

#ifdef __cplusplus
extern "C" {
#endif
void SinCosVal(FocParam_t *foc);

void Clarke(FocParam_t *foc);

void Park(FocParam_t *foc);

void InvPark(FocParam_t *foc);

void InvClarke(FocParam_t *foc);

int SvpwmSector(FocParam_t *foc);

int svpwm(FocParam_t *foc);

/*********************************** smo *******************************/
void SmoInit(SmoParam_t *smo);

float SmoViewer(FocParam_t *foc, SmoParam_t *smo);

void SmoParamInit(SmoParam_t *smo);
/*********************************** smo *******************************/

/*********************************** hfi *******************************/
float   HfiInjectSign(float inject_U);
float   HfiPllAngle(pll_t *pll, HfiParam_t *hfi);
void    HfiAngleCalc(FocParam_t *foc, HfiParam_t *hfi);
void    IdqToIdqF(FocParam_t *foc, HfiParam_t *hfi);
void    IdqToIdqH(FocParam_t *foc, HfiParam_t *hfi);
bool    HfiNsIdentify(HfiParam_t *hfi, FocParam_t *foc);
int16_t non_flux_observer(void);
/*********************************** hfi *******************************/
#ifdef __cplusplus
}
#endif

extern SmoParam_t     smo_param;
extern volatile float smo_angle;

#endif

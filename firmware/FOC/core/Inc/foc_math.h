#ifndef __FOC_CAL_H
#define __FOC_CAL_H

#include "util.h"
#include "foc_cfg.h"

typedef struct smo_param_t
{
    float A;
    float B;
    float ksw; //滑膜系数
    float ialpha_view; //alpha轴电流观测值
    float ibeta_view; //beta轴电流观测值
    float ialpha_view_last; //上次观测值
    float ibeta_view_last; //上次观测值
    float valpah;   //观测值
    float vbeta;

    float Ealpha;   //alpha轴拓展反电动势
    float Ebeta;    //beta轴拓展反电动势
}smo_param_t;


void SinCosVal(foc_param_t *foc);
void Clarke(foc_param_t *foc);
void Park(foc_param_t *foc);
void InvPark(foc_param_t *foc);
void InvClarke(foc_param_t *foc);
int SvpwmSector(foc_param_t *foc);
int svpwm(foc_param_t *foc);
void SmoInit(smo_param_t *smo);
float SmoViewer(foc_param_t *foc, smo_param_t *smo);
void SmoParamInit(smo_param_t *smo);

extern smo_param_t smo;
extern volatile float pll_angle;
extern float ualpha;
extern float ubeta;


#endif

#ifndef __FOC_CTRL_H
#define __FOC_CTRL_H
#include "foc_cfg.h"
#include "adc.h"

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

void FocVolt(float vd_ref, float vq_ref, float pos);
void Encoder_Idle(void);
void FocHandle(void);
extern volatile int spd_set;
extern smo_param_t smo;
extern float ualpha;
extern float ubeta;
#endif

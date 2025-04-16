#ifndef _MOC_SPD_H_
#define _MOC_SPD_H_

#include "stdint.h"

typedef struct pll_t
{
    uint16_t kp;
    uint16_t ki;
    uint16_t loop_hz;
    float ref;
    float fbk;
    float out_value;
    float angle_out;
    float i_term;
    float p_term;
}pll_t;

float MovingAverageCotrl(float ref, float fbk);
float PllSpeedCtrl(pll_t *pll, float angle);
void LowPassFilter(float *in,float hz);

#endif

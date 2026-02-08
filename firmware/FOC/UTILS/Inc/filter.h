#ifndef _MOC_SPD_H_
#define _MOC_SPD_H_

#include "stdint.h"

typedef struct pll_t {
    float kp;
    float ki;
    float loop_hz;
    float ref;
    float fbk;
    float error;
    float out_value;
    float angle_out;

    float i_term;
    float p_term;
    float i_term_limit;
} pll_t;

typedef struct lpf_t {
    float in_last;
    float trust;
} lpf_t;


typedef struct MovingAverage_t {
    void *buffer;
    uint8_t size;
    uint8_t index;
} MovingAverage_t;

#ifdef __cplusplus
extern "C" {
#endif
float PllSpeedCtrl(pll_t *pll, float angle);
void LowPassFilter(float *data, lpf_t *lpf);
void MoveAverageFilter(MovingAverage_t *filter, float *data);
#ifdef __cplusplus
}
#endif

#endif

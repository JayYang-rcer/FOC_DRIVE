#include "filter.h"
#include "util.h"
#include "main.h"

float PllSpeedCtrl(pll_t *pll, float angle)
{
	pll->ref = sin_f32(angle)*cos_f32(pll->angle_out);
	pll->fbk = sin_f32(pll->angle_out)*cos_f32(angle);

    pll->p_term = (pll->ref - pll->fbk)*pll->kp;
    pll->i_term += (angle - pll->angle_out)*pll->ki/pll->loop_hz;
	pll->out_value = pll->p_term + pll->i_term;

    pll->angle_out += pll->out_value/pll->loop_hz;
	WRAP_0_2PI(pll->angle_out)

	pll->out_value*=60.f/M_2PI;
	return pll->out_value;
}


void LowPassFilter(float *data, lpf_t* lpf)
{
    *data = *data * lpf->trust + (1 - lpf->trust)*lpf->in_last;
    lpf->in_last = *data;
}


/*
 * @brief: 滑动均值滤波器
 * @param: buffer: 滑动窗口缓冲区地址
 * @param: data: 输入数据
 * @param: size: 滑动窗口大小
 */
__RAM_FUNC void MoveAverageFilter(MovingAverage_t* filter, float *data)
{
    float sum = 0;                           // 窗口内数据的和
    // 将新数据存入缓冲区
    ((float*)filter->buffer)[filter->index] = *data;

    // 更新索引（循环缓冲区）
    filter->index = (filter->index + 1) % filter->size;

    // 计算窗口内数据的和
    for (int i = 0; i < filter->size; i++) {
        sum += ((float*)filter->buffer)[i];
    }

    // 返回平均值
    *data = sum / filter->size;
}
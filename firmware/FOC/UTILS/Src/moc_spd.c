#include "moc_spd.h"
#include "util.h"

//float speed_moving_average_filter(float input, float *buffer, int size)
//{
//    float sum = 0;                           // 窗口内数据的和
//    static int speed_index;
//    // 将新数据存入缓冲区
//    buffer[speed_index] = input;
//
//    // 更新索引（循环缓冲区）
//    speed_index = (speed_index + 1) % size;
//
//    // 计算窗口内数据的和
//    for (int i = 0; i < size; i++) {
//        sum += buffer[i];
//    }
//
//    // 返回平均值
//    return sum / size;
//}


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

void PllLpf(float *in,float hz)
{
    static float in_last;
    *in = *in * hz + in_last * (1.f - hz);
    in_last = *in;
}



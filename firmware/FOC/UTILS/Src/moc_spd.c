#include "moc_spd.h"
#include "util.h"

// volatile uint32_t pll_kp = 621,pll_hz=5000;
// volatile uint32_t pll_ki = 39;
float PllSpeedCtrl(pll_t *pll, float angle)
{
    static float angle_out,i_term,p_term;
	pll->ref = sin_f32(angle)*cos_f32(angle_out);
	pll->fbk = sin_f32(angle_out)*cos_f32(angle);
	
	p_term = (pll->ref - pll->fbk)*pll->kp;
	i_term += (angle - angle_out)*pll->ki/pll->loop_hz;
	pll->out_value = p_term + i_term;
	
	angle_out += pll->out_value/pll->loop_hz;
	WRAP_0_2PI(angle_out)

	pll->out_value*=60.f/M_2PI;
	return pll->out_value;
}

void LowPassFilter(float *in,float hz)
{
    static float in_last;
    *in = *in * hz + in_last * (1.f - hz);
    in_last = *in;
}

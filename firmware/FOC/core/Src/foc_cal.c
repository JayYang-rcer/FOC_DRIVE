#include "foc_cal.h"
#include "adc.h"
#include "main.h"
#include "vofa.h"

_RAM_FUNC void SinCosVal(foc_param_t *foc)
{
    foc->sin_val = sin_f32(foc->theta);
    foc->cos_val = cos_f32(foc->theta);
}

_RAM_FUNC void Clarke(foc_param_t *foc)
{
    foc->i_alpha = foc->i_a;
    foc->i_beta = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
}

_RAM_FUNC void Park(foc_param_t *foc)
{
    foc->i_d =  foc->i_alpha * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q = -foc->i_alpha * foc->sin_val + foc->i_beta * foc->cos_val;
}

_RAM_FUNC void InvPark(foc_param_t *foc)
{
    // foc->v_alpha = foc->v_d * foc->cos_val - foc->v_q * foc->sin_val;
    // foc->v_beta  = foc->v_q * foc->cos_val + foc->v_d * foc->sin_val;
    foc->v_alpha = foc->v_d * foc->cos_val - foc->v_q * foc->sin_val;
    foc->v_beta  = foc->v_q * foc->cos_val + foc->v_d * foc->sin_val;
}

_RAM_FUNC void InvClarke(foc_param_t *foc)
{
    foc->v_a = foc->v_alpha;
    foc->v_b = -0.5f * foc->v_alpha + SQRT3_BY_2 * foc->v_beta;
    foc->v_c = -0.5f * foc->v_alpha - SQRT3_BY_2 * foc->v_beta;
}

/**
* @brief:      svpwm_sector(foc_para_t *foc)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    SVPWM扇区判断
*/
// _RAM_FUNC int SvpwmSector(foc_para_t *foc)
_RAM_FUNC int SvpwmSector(foc_param_t *foc)
{
    float TS = 1.f;
    float ta = 0.0f, tb = 0.0f, tc = 0.0f;
    float k = (TS*SQRT3)/foc->vbus;
    float tx,ty;
    float temp=0;

    float U1 = foc->v_beta;
    float U2 = (SQRT3 *foc->v_alpha - foc->v_beta) * 0.5f;
    float U3 = -(SQRT3 *foc->v_alpha + foc->v_beta) * 0.5f;
    // InvClarke(foc);

    int a = (U1 > 0.0f) ? 1 : 0;
    int b = (U2 > 0.0f) ? 1 : 0;
    int c = (U3 > 0.0f) ? 1 : 0;
    int sextant = (int)((c << 2) + (b << 1) + a);
    int sector=0;

    switch (sextant)
    {
        case 1:
        {
            sector = 2;
            tx = -U2 * k; 
            ty = -U3 * k;
            break;
        }
        
        case 2:
        {
            sector = 6;
            tx = -U3*k;
            ty = -U1*k;
            break;
        }
        
        case 3:
        {
            sector = 1;
            tx = U2*k;
            ty = U1*k;
            break;
        }        

        case 4:
        {
            sector = 4;
            tx = -U1*k;
            ty = -U2*k;
            break;
        }        

        case 5:
        {
            sector = 3;
            tx = U1*k;
            ty = U3*k;
            break;
        }       

        case 6:
        {
            sector = 5;
            tx = U3*k;
            ty = U2*k;
            break;
        }

        default:
            break;
    }   

    //过调制处理,会导致电机无法达到最大理论转速,后续如果需要进一步优化性能，考虑在互补PWM的死去上做处理，增大有效的电平时间
    if(tx + ty > TS)
    {
        temp = tx+ty;
        tx=tx/temp*TS;
        ty=ty/temp*TS;
    }

    ta = (TS + tx + ty) * 0.25f;
    tb = ta - tx * 0.5f;
    tc = tb - ty * 0.5f;
    
    switch (sector)
    {
        case 1: foc->dtc_a = ta; foc->dtc_b = tb; foc->dtc_c = tc; break;
        case 2: foc->dtc_a = tb; foc->dtc_b = ta; foc->dtc_c = tc; break;
        case 3: foc->dtc_a = tc; foc->dtc_b = ta; foc->dtc_c = tb; break;
        case 4: foc->dtc_a = tc; foc->dtc_b = tb; foc->dtc_c = ta; break;
        case 5: foc->dtc_a = tb; foc->dtc_b = tc; foc->dtc_c = ta; break;
        case 6: foc->dtc_a = ta; foc->dtc_b = tc; foc->dtc_c = tb; break;
        default:
            break;
    }
	
	// if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid = foc->dtc_a >= 0.0f && foc->dtc_a <= 1.0f &&
                       foc->dtc_b >= 0.0f && foc->dtc_b <= 1.0f &&
                       foc->dtc_c >= 0.0f && foc->dtc_c <= 1.0f;

    return result_valid ? 0 : -1;
}


#include "foc_cfg.h"
#include "adc.h"
#include "tim.h"
#include "util.h"

#define Dead_Time 80
#define PWM_ARR() __HAL_TIM_GET_AUTORELOAD(&htim8)
#define SET_DTC_A(value)     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, value)
#define SET_DTC_B(value)     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, value)
#define SET_DTC_C(value)     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, value)

#define BATTERY_CELL 3

foc_adc_t mc_adc;
foc_param_t foc;
motor_cfg_t motor_cfg;
motor_ctrl_t motor_ctrl;
pid_para_t id_pid,iq_pid;
pid_para_t speed_pid;
pid_para_t pos_pid;
pll_t pll_spd;

void MotorPidInit(void)
{
    //id_pid.kp = motor_cfg.ls/1000000*7800*motor_cfg.pn/60*M_2PI;    // kp = 0.00000648(H) * 5000*7(erpm/min)(带宽) / 60(s/min) * 2PI
    //id_pid.ki = motor_cfg.rs/1000*7800*motor_cfg.pn/60*M_2PI/20000;      // ki = 0.03765 * 5000*7 / 60 * 2PI / 32000(电流环频率)


	id_pid.kp = motor_cfg.ls/1000000*20000/20*M_2PI;    // kp = 0.00000648(H) * 5000*7(erpm/min)(带宽) / 60(s/min) * 2PI
    id_pid.ki = (motor_cfg.rs/1000)/(motor_cfg.ls/1000000)/20000;      // ki = 0.03765 * 5000*7 / 60 * 2PI / 32000(电流环频率)
    id_pid.out_max = (BATTERY_CELL*4)*ONE_BY_SQRT3;
    id_pid.out_min = -(BATTERY_CELL*4)*ONE_BY_SQRT3;
    id_pid.i_term_max = (BATTERY_CELL*4)*ONE_BY_SQRT3;
    id_pid.i_term_min = -(BATTERY_CELL*4)*ONE_BY_SQRT3;

    iq_pid.kp = motor_cfg.ls/1000000*20000/20*M_2PI;
    iq_pid.ki = (motor_cfg.rs/1000)/(motor_cfg.ls/1000000)/20000;
    iq_pid.out_max = (BATTERY_CELL*4)*ONE_BY_SQRT3;
    iq_pid.out_min = -(BATTERY_CELL*4)*ONE_BY_SQRT3;
    iq_pid.i_term_max = (BATTERY_CELL*4)*ONE_BY_SQRT3;
    iq_pid.i_term_min = -(BATTERY_CELL*4)*ONE_BY_SQRT3;

    speed_pid.kp = 0.0088f;
    speed_pid.ki = 0.006f;
    speed_pid.out_max = 15.f;
    speed_pid.out_min = -15.f;
    speed_pid.i_term_max = 15.f;
    speed_pid.i_term_min = -15.f;
    speed_pid.lpf_error = 0.1f;
	speed_pid.lpf_d = 0.1f;
    speed_pid.deadband = 1.f;

#if USE_VOLT_POS
    pos_pid.lpf_d = 0.1f;
    pos_pid.kp = 0.035f;
    pos_pid.kd = 0.0002f;
    pos_pid.out_max = 0.8f;
    pos_pid.out_min = -0.8f;
#else
    pos_pid.kp = 3.5f;
    pos_pid.kd = 0.005f;
    pos_pid.out_max = 300.f;
    pos_pid.out_min = -300.f;
#endif

    //u3最高最速，KV700 * 11.1/sqrt(3) = 5037
    pll_spd.loop_hz = 10000;
    pll_spd.kp = 5037.f/60.f*M_2PI * 0.707f * 2.f;
	//pll_spd.kp = 4200.f/60.f*M_2PI * 10.f * 2.f;
    pll_spd.ki = (5037.f/60.f*M_2PI) * (5037.f/60.f*M_2PI) / pll_spd.loop_hz;
}


void MotorParaInit(void)
{
    motor_cfg.flux = 1.221f; //mWb
    motor_cfg.pn = 7;
    motor_cfg.rs = 37.65f; //mOhm
    motor_cfg.ls = 6.48f; //uH

    MotorPidInit();
}

void GetCurrentOffset(foc_adc_t *mc_adc)
{
    float sum_ia=0,sum_ib=0, sum_ic=0,sum_vbus=0;
    float sum_va=0,sum_vb=0, sum_vc=0;
    for(int i=0; i<1000; i++)
    {
        HAL_Delay(1);
		sum_ia += (float)(ADC1->JDR3);
        sum_ib += (float)(ADC1->JDR2);
        sum_ic += (float)(ADC1->JDR1);
        sum_va += (float)(ADC2->JDR3);
        sum_vb += (float)(ADC2->JDR2);
        sum_vc += (float)(ADC2->JDR1);
    }

	mc_adc->ia_offset = sum_ia / 1000.0f;
    mc_adc->ib_offset = sum_ib / 1000.0f;
    mc_adc->ic_offset = sum_ic / 1000.0f;
    mc_adc->va_offset = sum_va / 1000.0f;
    mc_adc->vb_offset = sum_vb / 1000.0f;
    mc_adc->vc_offset = sum_vc / 1000.0f;
}

void FocPwmStart(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
}

void FocPwmStop(void)
{
    SET_DTC_A(PWM_ARR());
    SET_DTC_B(PWM_ARR());
    SET_DTC_C(PWM_ARR());
}

_RAM_FUNC void FocPwmRun(foc_param_t *foc)
{
    SET_DTC_A((uint16_t)(foc->dtc_a * PWM_ARR()));
    SET_DTC_B((uint16_t)(foc->dtc_b * PWM_ARR()));
    SET_DTC_C((uint16_t)(foc->dtc_c * PWM_ARR()));
}

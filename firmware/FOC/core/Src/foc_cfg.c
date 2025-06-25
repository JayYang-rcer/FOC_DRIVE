#include "foc_cfg.h"
#include "adc.h"
#include "tim.h"
#include "util.h"

#define Dead_Time 0
#define PWM_ARR() __HAL_TIM_GET_AUTORELOAD(&htim8)

#define BATTERY_CELL 3
#define CURRENT_LOOP_RATE 20000 //电流环频率

foc_adc_t mc_adc;
foc_param_t foc_param;
motor_cfg_t motor_cfg;
motor_ctrl_t motor_ctrl = {0};
pid_para_t id_pid,iq_pid;
pid_para_t speed_pid;
pid_para_t pos_pid;
pll_t pll_spd;

/********************smo param********************/
smo_param_t smo_param;
pll_t pll_smo;
/**********************************************************/

/********************hfi param********************/
hfi_param_t hfi_param = {
        .inject_U = 0.8f,
};

pll_t pll_hfi = {
        .loop_hz = 20000, //20khz
        .kp = 4000,
        .ki = 250000,
        .i_term_limit = 500
};
/**********************************************************/

void MotorPidInit(void)
{
//    id_pid.kp = motor_cfg.ls/1000000*8000*motor_cfg.pn/60*M_2PI/5;    // kp = 0.00000648(H) * 5000*7(erpm/min)(带宽) / 60(s/min) * 2PI
//    id_pid.ki = motor_cfg.rs/1000*8000*motor_cfg.pn/60*M_2PI/CURRENT_LOOP_RATE/5;      // ki = 0.03765 * 5000*7 / 60 * 2PI / 32000(电流环频率)
    id_pid.kp = motor_cfg.ls/1000000*8000*motor_cfg.pn/60*M_2PI;    // kp = 0.00000648(H) * 5000*7(erpm/min)(带宽) / 60(s/min) * 2PI
    id_pid.ki = motor_cfg.rs/1000*8000*motor_cfg.pn/60*M_2PI/CURRENT_LOOP_RATE;      // ki = 0.03765 * 5000*7 / 60 * 2PI / 32000(电流环频率)
    iq_pid.kp = motor_cfg.ls/1000000*8000*motor_cfg.pn/60*M_2PI;    // kp = 0.00000648(H) * 5000*7(erpm/min)(带宽) / 60(s/min) * 2PI
    iq_pid.ki = motor_cfg.rs/1000*8000*motor_cfg.pn/60*M_2PI/CURRENT_LOOP_RATE;      // ki = 0.03765 * 5000*7 / 60 * 2PI / 32000(电流环频率)


//	id_pid.kp = motor_cfg.ls/1000000*CURRENT_LOOP_RATE/20*M_2PI;    // kp = 0.00000648(H) * 5000*7(erpm/min)(带宽) / 60(s/min) * 2PI
//    id_pid.ki = (motor_cfg.rs/1000)/(motor_cfg.ls/1000000)/CURRENT_LOOP_RATE;      // ki = 0.03765 * 5000*7 / 60 * 2PI / 32000(电流环频率)
//    iq_pid.kp = motor_cfg.ls/1000000*CURRENT_LOOP_RATE/20*M_2PI;
//    iq_pid.ki = (motor_cfg.rs/1000)/(motor_cfg.ls/1000000)/CURRENT_LOOP_RATE;

    id_pid.out_max = (BATTERY_CELL*4)*ONE_BY_SQRT3;
    id_pid.out_min = -(BATTERY_CELL*4)*ONE_BY_SQRT3;
    id_pid.i_term_max = (BATTERY_CELL*4)*ONE_BY_SQRT3;
    id_pid.i_term_min = -(BATTERY_CELL*4)*ONE_BY_SQRT3;

    iq_pid.out_max = (BATTERY_CELL*4)*ONE_BY_SQRT3;
    iq_pid.out_min = -(BATTERY_CELL*4)*ONE_BY_SQRT3;
    iq_pid.i_term_max = (BATTERY_CELL*4)*ONE_BY_SQRT3;
    iq_pid.i_term_min = -(BATTERY_CELL*4)*ONE_BY_SQRT3;

/******************使用电压环进行位置闭环的速度pid参数*********************
    pos_pid.lpf_d = 0.1f;
    pos_pid.kp = 0.035f;
    pos_pid.kd = 0.0002f;
    pos_pid.out_max = 0.8f;
    pos_pid.out_min = -0.8f;
*********************************************************************/
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
    motor_cfg.rs = 42.2333f; //mOhm
    motor_cfg.ls = 6.3f; //uH

    MotorPidInit();
}


bool GetCurrentOffset(foc_adc_t *_adc)
{
    float sum_ia=0,sum_ib=0, sum_ic=0;
    for(int i=0; i<1000; i++)
    {
        HAL_Delay(1);
		sum_ia += (float)(ADC1->JDR3);
        sum_ib += (float)(ADC1->JDR2);
        sum_ic += (float)(ADC1->JDR1);
    }

	_adc->ia_offset = sum_ia / 1000.0f;
    _adc->ib_offset = sum_ib / 1000.0f;
    _adc->ic_offset = sum_ic / 1000.0f;

    return true;
}

void FocPwmStart(bool A, bool AN, bool B, bool BN, bool C, bool CN)
{
    if(A) HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    else HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    if(AN) HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    else HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
    if(B) HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    else HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    if(BN) HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    else HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    if(C) HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    else HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    if(CN) HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    else HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
}

void FocPwmStop(void)
{
    SET_DTC_A(0);
    SET_DTC_B(0);
    SET_DTC_C(0);
}


void MotorCtrlReset(motor_ctrl_t* motor)
{
    motor->iq_set=0;
    motor->id_set=0;
    motor->vd_set=0;
    motor->vq_set=0;
    motor->speed_set=0;
    motor->pos_set=0;
    motor->epos_acc=0;
}

_RAM_FUNC void FocPwmRun(foc_param_t *foc)
{
    SET_DTC_A((uint16_t)(foc->dtc_a * PWM_ARR() + Dead_Time));
    SET_DTC_B((uint16_t)(foc->dtc_b * PWM_ARR() + Dead_Time));
    SET_DTC_C((uint16_t)(foc->dtc_c * PWM_ARR() + Dead_Time));
}

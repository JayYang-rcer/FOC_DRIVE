#include "foc_cfg.h"
#include "adc.h"
#include "tim.h"
#include "util.h"

#define CURRENT_LOOP_RATE 20000.0f // 电流环频率

FocAdcValue_t mc_adc;
FocParam_t    foc_param;
MotorCfg_t    motor_cfg;
MotorCtrl_t   motor_ctrl = {0};
pi_para_t     id_pi, iq_pi;
pi_para_t     hfi_id_pi, hfi_iq_pi;
pid_para_t    speed_pid, HfiSpeed_pid;
pid_para_t    pos_pid;
pll_t         pll_spd;

/********************smo param********************/
SmoParam_t smo_param;
pll_t      pll_smo;
/**********************************************************/

/********************hfi param********************/
HfiParam_t hfi_param = {.omega_e = 0.1f, .inject_U = 1.2f};

pll_t pll_hfi = {
    .kp           = 1200,
    .ki           = 250000,
    .loop_hz      = 20000, // 20khz
    .i_term_limit = 1000};
/**********************************************************/

/********************non linear flux param********************/
NonFlux_t nonFlux = {
    .Gamma = 10000,
    .Ts    = 1 / 20000.f,
};

pll_t pll_flux = {
    .kp           = 1000,
    .ki           = 180000,
    .loop_hz      = 20000, // 20khz
    .i_term_limit = 10000};
/**********************************************************/
/* kp = 0.00000648(H) * 5000*7(erpm/min)(带宽) / 60(s/min) * 2PI
 * ki = 0.03765 * 5000*7 / 60 * 2PI / 20000(电流环频率)
 */
void MotorPidInit(void)
{
    // id_pid.kp = motor_cfg.ls/1000000*8000*motor_cfg.pn/60*M_2PI/5;
    // id_pid.ki = motor_cfg.rs/1000*8000*motor_cfg.pn/60*M_2PI/CURRENT_LOOP_RATE/5;

    id_pi.kp    = motor_cfg.ls / 1000000.0f * 11000.0f * motor_cfg.pn * RPM_TO_RADS * 3.f;
    id_pi.ki    = motor_cfg.rs / 1000.0f * 11000.0f * motor_cfg.pn * RPM_TO_RADS / CURRENT_LOOP_RATE;
    id_pi.kAnti = 0.5f;
    iq_pi.kp    = motor_cfg.ls / 1000000.0f * 11000.0f * motor_cfg.pn * RPM_TO_RADS * 3.f;
    iq_pi.ki    = motor_cfg.rs / 1000.0f * 11000.0f * motor_cfg.pn * RPM_TO_RADS / CURRENT_LOOP_RATE;
    iq_pi.kAnti = 0.5f;

    id_pi.out_max    = (BATTERY_CELL * 4.0f) * ONE_BY_SQRT3;
    id_pi.out_min    = -(BATTERY_CELL * 4.0f) * ONE_BY_SQRT3;
    id_pi.i_term_max = (BATTERY_CELL * 4.0f) * ONE_BY_SQRT3;
    id_pi.i_term_min = -(BATTERY_CELL * 4.0f) * ONE_BY_SQRT3;

    iq_pi.out_max    = (BATTERY_CELL * 4.0f) * ONE_BY_SQRT3;
    iq_pi.out_min    = -(BATTERY_CELL * 4.0f) * ONE_BY_SQRT3;
    iq_pi.i_term_max = (BATTERY_CELL * 4.0f) * ONE_BY_SQRT3;
    iq_pi.i_term_min = -(BATTERY_CELL * 4.0f) * ONE_BY_SQRT3;

    hfi_id_pi.kp    = motor_cfg.ls / 1000000 * 10000 * motor_cfg.pn / 60 * M_2PI;
    hfi_id_pi.ki    = motor_cfg.rs / 1000 * 10000 * motor_cfg.pn / 60 * M_2PI / CURRENT_LOOP_RATE;
    hfi_id_pi.kAnti = 0.5f;
    hfi_iq_pi.kp    = motor_cfg.ls / 1000000 * 10000 * motor_cfg.pn / 60 * M_2PI;
    hfi_iq_pi.ki    = motor_cfg.rs / 1000 * 10000 * motor_cfg.pn / 60 * M_2PI / CURRENT_LOOP_RATE;
    hfi_iq_pi.kAnti = 0.5f;

    hfi_id_pi.out_max    = (BATTERY_CELL * 4) * ONE_BY_SQRT3;
    hfi_id_pi.out_min    = -(BATTERY_CELL * 4) * ONE_BY_SQRT3;
    hfi_id_pi.i_term_max = (BATTERY_CELL * 4) * ONE_BY_SQRT3;
    hfi_id_pi.i_term_min = -(BATTERY_CELL * 4) * ONE_BY_SQRT3;

    hfi_iq_pi.out_max    = (BATTERY_CELL * 4) * ONE_BY_SQRT3;
    hfi_iq_pi.out_min    = -(BATTERY_CELL * 4) * ONE_BY_SQRT3;
    hfi_iq_pi.i_term_max = (BATTERY_CELL * 4) * ONE_BY_SQRT3;
    hfi_iq_pi.i_term_min = -(BATTERY_CELL * 4) * ONE_BY_SQRT3;

    /******************使用电压环进行位置闭环的速度pid参数*********************
    pos_pid.lpf_d = 0.1f;
    pos_pid.kp = 0.035f;
    pos_pid.kd = 0.0002f;
    pos_pid.out_max = 0.8f;
    pos_pid.out_min = -0.8f;
    *********************************************************************/
    speed_pid.kp         = 0.001f;
    speed_pid.ki         = 0.002f;
    speed_pid.out_max    = 15.f;
    speed_pid.out_min    = -15.f;
    speed_pid.i_term_max = 15.f;
    speed_pid.i_term_min = -15.f;
    speed_pid.lpf_error  = 0.1f;
    speed_pid.lpf_d      = 0.1f;
    speed_pid.deadband   = 1.f;

    HfiSpeed_pid.kp         = 0.004f;
    HfiSpeed_pid.ki         = 0.003f;
    HfiSpeed_pid.out_max    = 15.f;
    HfiSpeed_pid.out_min    = -15.f;
    HfiSpeed_pid.i_term_max = 15.f;
    HfiSpeed_pid.i_term_min = -15.f;
    HfiSpeed_pid.lpf_error  = 0.1f;
    HfiSpeed_pid.lpf_d      = 0.1f;
    HfiSpeed_pid.deadband   = 1.f;

#if USE_VOLT_POS
    pos_pid.lpf_d   = 0.1f;
    pos_pid.kp      = 0.035f;
    pos_pid.kd      = 0.0002f;
    pos_pid.out_max = 0.8f;
    pos_pid.out_min = -0.8f;
#else
    pos_pid.kp      = 3.5f;
    pos_pid.kd      = 0.005f;
    pos_pid.out_max = 300.f;
    pos_pid.out_min = -300.f;
#endif

    // u3最高最速，KV700 * 11.1/sqrt(3) = 5037
    pll_spd.loop_hz = 10000;
    pll_spd.kp      = 10800.f / 60.f * M_2PI * 0.707f * 2.f;
    // pll_spd.kp = 4200.f/60.f*M_2PI * 10.f * 2.f;
    pll_spd.ki = (10800.f / 60.f * M_2PI) * (10800.f / 60.f * M_2PI) / pll_spd.loop_hz;
}

void MotorParaInit(void)
{
    motor_cfg.flux = 1.221f; // mWb
    motor_cfg.pn   = 7;
    motor_cfg.rs   = 37.5333f; // mOhm
    motor_cfg.ls   = 6.3f;     // uH

    MotorPidInit();
}

void CurrentSampInit(void)
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 3999);
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
}

void FocPwmStart(bool A, bool AN, bool B, bool BN, bool C, bool CN)
{
    if (A)
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    else
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
    if (AN)
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
    else
        HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);
    if (B)
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    else
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
    if (BN)
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
    else
        HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
    if (C)
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    else
        HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    if (CN)
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
    else
        HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
}

void FocPwmStop(void)
{
    SET_DTC_A(0);
    SET_DTC_B(0);
    SET_DTC_C(0);
}

void MotorCtrlReset(MotorCtrl_t *motor)
{
    motor->iq_set    = 0;
    motor->id_set    = 0;
    motor->vd_set    = 0;
    motor->vq_set    = 0;
    motor->speed_set = 0;
    motor->pos_set   = 0;
    motor->epos_acc  = 0;
}

#include "foc_cfg.h"
#include "adc.h"
#include "tim.h"

FocParam_t    foc_param;
MotorCfg_t    motor_cfg;
MotorCtrl_t   motor_ctrl = {0};

void MotorPidInit(void)
{
//#if USE_VOLT_POS
//    pos_pid.lpf_d   = 0.1f;
//    pos_pid.kp      = 0.035f;
//    pos_pid.kd      = 0.0002f;
//    pos_pid.out_max = 0.8f;
//    pos_pid.out_min = -0.8f;
//#else
//    pos_pid.kp      = 3.5f;
//    pos_pid.kd      = 0.005f;
//    pos_pid.out_max = 300.f;
//    pos_pid.out_min = -300.f;
//#endif
}

void MotorParaInit(void)
{
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

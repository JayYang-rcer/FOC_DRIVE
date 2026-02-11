#ifndef __FOC_CFG_H
#define __FOC_CFG_H

#include "filter.h"
#include "pid.h"
#include "stdbool.h"

#define SET_DTC_A(value)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, value)
#define SET_DTC_B(value)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, value)
#define SET_DTC_C(value)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, value)

#define R_SENSE           0.001f                          // 采样电阻阻值
#define IOP               20.f                            // 电流采样电阻放大倍数
#define IRATIO            (3.3f / 4096.f) / R_SENSE / IOP // 电流采样,电压转换为电流值的系数
#define VBUS_RATIO        (6.1f * 3.3f) / 4096.0f         // 母线电压采样电压转换为电压值的系数
#define BATTERY_CELL      4.0f

// #define VBUS_RATIO 0.0084723f //母线电压采样电压转换为电压值的系数

// Speed PID parameters
#define SPEED_PID_TIME_HZ 2000

typedef enum {
    FOC_IDLE            = 0, // 空闲
    FOC_VF_CTRL         = 1, // VF强拖
    FOC_VOLT_CTRL       = 2, // 电压控制
    FOC_CURRENT_CTRL    = 3, // 电流控制
    FOC_SPEED_CTRL      = 4, // 速度控制
    FOC_POSITION_CTRL   = 5, // 位置控制
    FOC_SENSORLESS_CTRL = 6, // 无传感器控制
    FOC_HFI             = 7, // 高频注入测试
    FOC_IF_CTRL         = 8, // IF强拖
} FocCtrlMode_e;

typedef union {
    struct {
        float x;
        float y;
    } xy;

    struct {
        float alpha;
        float beta;
    } s;

    struct {
        float d;
        float q;
    } r;
} Vector2Df_t;

typedef struct {
    uint16_t uhU;
    uint16_t uhV;
    uint16_t uhW;
}Vector3D_t;

typedef struct {
    float fU;
    float fV;
    float fW;
} Vector3S_t;

typedef struct {
    Vector2Df_t Is;
    Vector2Df_t Vs;
    Vector2Df_t state;
    float      Gamma; // Non-linear observer gain
    float      Ts;    // Sampling period
    float      theta_e;
    float       omega;
} NonFlux_t;

typedef struct {
    float A;
    float B;
    float ksw;              // 滑膜系数
    float ialpha_view;      // alpha轴电流观测值
    float ibeta_view;       // beta轴电流观测值
    float ialpha_view_last; // 上次观测值
    float ibeta_view_last;  // 上次观测值
    float valpah;           // 观测值
    float vbeta;

    float Ealpha; // alpha轴拓展反电动势
    float Ebeta;  // beta轴拓展反电动势
} SmoParam_t;

typedef struct {
    float   rotor_pos;  // 转子位置
    float   rotor_vel;  // 转子转速
    float   rotor_rev;  // 转子转向
    float   rotor_epos; // 转子电角度
    float   rotor_evel; // 转子电速度
    float   fRefSlope;
    int16_t RefSlope;

    float rs;    // 相电阻
    float ls;    // 相电感
    float flux;  // 磁链
    float jx;    // 转动惯量
    float pn;    // 极对数
    float delta; // 阻尼系数
} MotorCfg_t;

typedef struct {
    bool  foc_init;
    float epos_acc;  // 电角加速度
    float vd_set;    // 直轴电压设定值
    float vq_set;    // 交轴电压设定值
    float id_set;    // 直轴电流设定值
    float iq_set;    // 交轴电流设定值
    float speed_set; // 速度设定值
    float pos_set;   // 位置设置

    uint16_t spd_cnt;
    uint16_t pos_cnt;

    FocCtrlMode_e mode; // 控制模式
} MotorCtrl_t;

typedef struct {
    Vector3S_t current_raw;
    Vector3S_t offset;
    float vbus;      // 母线电压
    float temp;      // 温度
} FocAdcValue_t;

typedef struct foc_param_t {
    float  vbus;
    float  inv_vbus; // 母线电压倒数

    float ibus;
    float i_abs;

    float theta;   // 角度
    float sin_val; // 此角度对应的正弦值
    float cos_val; // 此角度对应的余弦值

    Vector3S_t current;
    Vector3S_t vphase;

    Vector2Df_t idq;
    Vector2Df_t vdq;

    Vector2Df_t iab;
    Vector2Df_t vab;

    int8_t sector;
    float dtc_a; // A 相 PWM 占空比
    float dtc_b; // B 相 PWM 占空比
    float dtc_c; // C 相 PWM 占空比
} FocParam_t;

typedef struct hfi_param_t {
    float theta_e;  // 预测电角度
    float omega_e;  // 预测电角速度
    float inject_U; // 注入电压
    int   sign;

    uint16_t nsd_count;
    float    isum_positive;
    float    isum_negetive;

    Vector2Df_t ab_last;
    Vector2Df_t ab_laster;

    Vector2Df_t ab_h;

    Vector2Df_t idq_h;
    Vector2Df_t idq_h_last; // 上次的dq轴高频电流

    Vector2Df_t idq_f;
    Vector2Df_t idq_f_last; // 上次的dq轴高频电流
    Vector2Df_t idq_f_laster; // 上次的dq轴高频电流
} HfiParam_t;

extern FocAdcValue_t mc_adc;
extern FocParam_t    foc_param;
extern MotorCfg_t    motor_cfg;
extern MotorCtrl_t   motor_ctrl;
extern pi_para_t     id_pi, iq_pi;
extern pi_para_t     hfi_id_pi, hfi_iq_pi;
extern pid_para_t    speed_pid, HfiSpeed_pid;
extern pid_para_t    pos_pid;
extern pll_t         pll_spd;
extern NonFlux_t     nonFlux;
extern pll_t         pll_flux;
/********************smo param********************/
extern pll_t      pll_smo;
extern SmoParam_t smo_param;
/*************************************************/

/******************** hfi param ********************/
extern pll_t      pll_hfi; // 高频注入的PLL
extern HfiParam_t hfi_param;
/*************************************************/

#ifdef __cplusplus
extern "C"{
#endif
bool GetCurrentOffset(FocAdcValue_t *mc_adc);

void FocPwmStart(bool A, bool AN, bool B, bool BN, bool C, bool CN);

void FocPwmStop(void);

void FocPwmRun(FocParam_t *foc);

void MotorParaInit(void);

void CurrentSampInit(void);

void MotorCtrlReset(MotorCtrl_t *motor);
#ifdef __cplusplus
}
#endif

#endif

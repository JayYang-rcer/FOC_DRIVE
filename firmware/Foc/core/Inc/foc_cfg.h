#ifndef __FOC_CFG_H
#define __FOC_CFG_H

#include "pid.h"
#include "stdbool.h"
#include "filter.h"

#define SET_DTC_A(value)     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, value)
#define SET_DTC_B(value)     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, value)
#define SET_DTC_C(value)     __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, value)

#define R_SENSE 0.003f  //采样电阻阻值
#define IOP 20.f //电流采样电阻放大倍数
#define IRATIO (3.3f/4096.f) / R_SENSE / IOP //电流采样,电压转换为电流值的系数
#define VBUS_RATIO (6.1f * 3.3f)/4096.0f //母线电压采样电压转换为电压值的系数
//#define VBUS_RATIO 0.0084723f //母线电压采样电压转换为电压值的系数

// Speed PID parameters
#define SPEED_PID_TIME_HZ 5000

typedef enum FOC_CTRL_MODE {
    FOC_IDLE = 0, //空闲
    FOC_VF_CTRL = 1, //强拖
    FOC_VOLT_CTRL = 2, //电压控制
    FOC_CURRENT_CTRL = 3, //电流控制
    FOC_SPEED_CTRL = 4, //速度控制
    FOC_POSITION_CTRL = 5, //位置控制
    FOC_SENSORLESS_CTRL = 6, //无传感器控制
    FOC_HFI_TEST = 7, //高频注入测试
} FOC_CTRL_MODE;

typedef union {
    struct {
        float x;
        float y;
    }fx;

    struct {
        float alpha;
        float beta;
    }fab;

    struct {
        float d;
        float q;
    }fdq;
}Vector2D_t;

typedef struct {
    Vector2D_t Is;
    Vector2D_t Vs;
    Vector2D_t state;
    float Gamma;       // Non-linear observer gain
    float Ts;          // Sampling period
    float theta_e;
    float omega_e;
}non_flux_t;

typedef struct aplha_beta_t {
    float alpha; //alpha轴
    float beta;  //beta轴
} aplha_beta_t;

typedef struct dq_t {
    float id; //d轴
    float iq; //q轴
} dq_t;

typedef struct smo_param_t {
    float A;
    float B;
    float ksw; //滑膜系数
    float ialpha_view; //alpha轴电流观测值
    float ibeta_view; //beta轴电流观测值
    float ialpha_view_last; //上次观测值
    float ibeta_view_last; //上次观测值
    float valpah;   //观测值
    float vbeta;

    float Ealpha;   //alpha轴拓展反电动势
    float Ebeta;    //beta轴拓展反电动势
} smo_param_t;

typedef struct {
    float rotor_pos;    //转子位置
    float rotor_vel;    //转子转速
    float rotor_rev;    //转子转向
    float rotor_epos;    //转子电角度
    float rotor_evel;    //转子电速度
    float fRefSlope;
    int16_t RefSlope;

    float rs;           //相电阻
    float ls;           //相电感
    float flux;         //磁链
    float jx;           //转动惯量
    float pn;           //极对数
    float delta;        //阻尼系数
} motor_cfg_t;

typedef struct {
    bool foc_init;
    float epos_acc;     //电角加速度
    float vd_set;       //直轴电压设定值
    float vq_set;       //交轴电压设定值
    float id_set;       //直轴电流设定值
    float iq_set;       //交轴电流设定值
    float speed_set;    //速度设定值
    float pos_set;        //位置设置

    uint16_t spd_cnt;
    uint16_t pos_cnt;

    FOC_CTRL_MODE mode; //控制模式
} motor_ctrl_t;

typedef struct {
    float adc_ia;           //A相电流
    float adc_ib;           //B相电流
    float adc_ic;           //C相电流
    float va;           //A相端电压
    float vb;           //B相端电压
    float vc;           //C相端电压

    float ia_offset;    //A相电流偏移
    float ib_offset;    //B相电流偏移
    float ic_offset;    //C相电流偏移
    float vbus;         //母线电压
    float temp;         //温度
} foc_adc_t;

typedef struct foc_param_t {
    int8_t sector;
    float vbus;
    float inv_vbus; // 母线电压倒数

    float ibus;
    float i_abs;

    float theta;   // 角度
    float sin_val; // 此角度对应的正弦值
    float cos_val; // 此角度对应的余弦值

    float i_a; // A 相电流
    float i_b; // B 相电流
    float i_c; // C 相电流

    float v_a; // A 相电压
    float v_b; // B 相电压
    float v_c; // C 相电压

    float i_d; // D 坐标系电流
    float i_q; // Q 坐标系电流

    float v_d; // D 坐标系电压
    float v_q; // Q 坐标系电压

    float i_alpha; // Alpha 坐标系电流
    float i_beta;  // Beta 坐标系电流

    float v_alpha; // Alpha 坐标系电压
    float v_beta;  // Beta 坐标系电压

    float dtc_a; // A 相 PWM 占空比
    float dtc_b; // B 相 PWM 占空比
    float dtc_c; // C 相 PWM 占空比
} foc_param_t;


typedef struct hfi_param_t {
    float theta_e; //预测电角度
    float omega_e; //预测电角速度
    float inject_U; //注入电压
    int sign;

    uint16_t nsd_count;
    float isum_positive;
    float isum_negetive;

    aplha_beta_t ab;
    aplha_beta_t ab_last;
    aplha_beta_t ab_laster;

    aplha_beta_t ab_h;
    aplha_beta_t ab_h_last;
    aplha_beta_t envelope;

    dq_t idq_h;
    dq_t idq_h_last; //上次的dq轴高频电流
    dq_t idq_h_laster; //上次的dq轴高频电流

    dq_t idq_f;
    dq_t idq_f_last;
    dq_t idq_f_laster; //上次的dq轴高频电流
} hfi_param_t;


extern foc_adc_t mc_adc;
extern foc_param_t foc_param;
extern motor_cfg_t motor_cfg;
extern motor_ctrl_t motor_ctrl;
extern pi_para_t id_pi, iq_pi;
extern pid_para_t id_pid, iq_pid;
extern pid_para_t speed_pid;
extern pid_para_t pos_pid;
extern pll_t pll_spd;

/********************smo param********************/
extern pll_t pll_smo;
extern smo_param_t smo_param;
/*************************************************/

/******************** hfi param ********************/
extern pll_t pll_hfi; //高频注入的PLL
extern hfi_param_t hfi_param;
/*************************************************/

bool GetCurrentOffset(foc_adc_t *mc_adc);

void FocPwmStart(bool A, bool AN, bool B, bool BN, bool C, bool CN);

void FocPwmStop(void);

void FocPwmRun(foc_param_t *foc);

void MotorParaInit(void);

void CurrentSampInit(void);

void MotorCtrlReset(motor_ctrl_t *motor);

#endif

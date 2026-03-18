#ifndef __FOC_CFG_H
#define __FOC_CFG_H

#include "stdbool.h"
#include "stdint.h"
#include "util.h"

#define SET_DTC_A(value)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, value)
#define SET_DTC_B(value)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, value)
#define SET_DTC_C(value)  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, value)

#define R_SENSE           0.001f                          // 采样电阻阻值
#define IOP               20.f                            // 电流采样电阻放大倍数
#define IRATIO            (3.3f / 4096.f) / R_SENSE / IOP // 电流采样,电压转换为电流值的系数
#define VBUS_RATIO        (6.1f * 3.3f) / 4096.0f         // 母线电压采样电压转换为电压值的系数
#define BATTERY_CELL      4.0f

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

typedef struct {
    float   rotor_pos;  // 转子位置
    float   rotor_vel;  // 转子转速
    float   rotor_rev;  // 转子转向
    float   rotor_epos; // 转子电角度
    float   rotor_evel; // 转子电速度
    float   fRefSlope;
    int16_t RefSlope;
} MotorCfg_t;

typedef struct
{
    float rs;    // 相电阻
    float ls;    // 相电感
    float flux;  // 磁链
    float jx;    // 转动惯量
    float pn;    // 极对数
    float delta; // 阻尼系数
} MotorParam_t;

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

typedef struct foc_param_t {
    float vbus;
    float i_bus;
    float theta; // 角度

    Vector3S_t current;
    Vector3S_t vphase;

    Vector2Df_t idq;
    Vector2Df_t vdq;

    Vector2Df_t iab;
    Vector2Df_t vab;
} FocParam_t;

extern FocParam_t    foc_param;
extern MotorCfg_t    motor_cfg;
extern MotorCtrl_t   motor_ctrl;

#ifdef __cplusplus
extern "C" {
#endif

void FocPwmStart(bool A, bool AN, bool B, bool BN, bool C, bool CN);
void FocPwmStop(void);
void MotorParaInit(void);
void CurrentSampInit(void);
void ResourceInit(void);
void MotorCtrlReset(MotorCtrl_t *motor);
#ifdef __cplusplus
}
#endif

#endif

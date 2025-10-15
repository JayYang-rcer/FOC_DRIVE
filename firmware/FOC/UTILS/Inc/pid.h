#ifndef __PID_H
#define __PID_H

typedef struct pid_para_t {
    volatile float kp;       // 比例
    volatile float ki;       // 积分
    volatile float kd;       // 微分
    volatile float lpf_d;    // 微分低通滤波频率
    volatile float lpf_error;// 误差低通滤波频率
    volatile float p_term;   // 比例项
    volatile float i_term;   // 积分项
    volatile float d_term;   // 微分项

    volatile float i_term_max;// 积分累加限幅
    volatile float i_term_min;// 积分累加限幅

    volatile float ctrl_period;// 控制周期

    volatile float target_value;// 目标值
    volatile float fback_value; // 实际值

    float error;             // 误差
    volatile float pre_error;// 上一次误差
    volatile float d_error;  // 误差变化率

    volatile float out_min; // 输出限幅
    volatile float out_max; // 输出限幅
    volatile float deadband;// 死区

    volatile float out_value;
} pid_para_t;

typedef struct pi_para_t {
    volatile float kp;    // 比例
    volatile float ki;    // 积分
    volatile float p_term;// 比例项
    volatile float i_term;// 积分项
    volatile float kAnti; // 抗积分饱和系数

    volatile float i_term_max;// 积分累加限幅
    volatile float i_term_min;// 积分累加限幅
    volatile float anti_term; // 抗积分饱和

    volatile float ctrl_period;// 控制周期

    volatile float target_value;// 目标值
    volatile float fback_value; // 实际值

    float error;// 误差

    volatile float out_min;// 输出限幅
    volatile float out_max;// 输出限幅

    volatile float out_raw;
    volatile float out_value;
} pi_para_t;

void pid_para_init(pid_para_t *pid_config);
float SerialPidCtrl(pid_para_t *pid, float target_value, float fdback_value);
float SerialPidCtrlTest(pi_para_t *pi, float target_value, float fdback_value);
float ParallelPidCtrl(pid_para_t *pid, float target_value, float fdback_value);
float IncreatParallePidCtrl(pid_para_t *pid, float target_value, float fdback_value);
float AbsLimit(float a, float abs_max);

#endif

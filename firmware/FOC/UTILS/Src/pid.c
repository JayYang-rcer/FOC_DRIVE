#include "pid.h"
#include "foc_cfg.h"
#include "util.h"

float AbsLimit(float a, float abs_max) {
    if (a > abs_max) a = abs_max;

    if (a < -abs_max) a = -abs_max;
    return a;
}


float IpCtrl() {
    return 0;
}

float IncreatParallePidCtrl(pid_para_t *pid, float target_value, float fdback_value) {

    pid->target_value = target_value;
    pid->fback_value  = fdback_value;
    pid->error        = pid->target_value - pid->fback_value;
    pid->d_error      = pid->error - pid->pre_error;

    // LowPassFilter(&pid->error, pid->lpf_error);
    // LowPassFilter(&d_error, pid->lpf_d);
    if (ABS(pid->error) < pid->deadband) return 0;
    pid->p_term = pid->kp * pid->d_error;
    pid->i_term = AbsLimit(pid->ki * pid->d_error, pid->i_term_max);

    pid->d_term = pid->d_error * pid->kd * SPEED_PID_TIME_HZ;

    if (pid->i_term > pid->i_term_max) pid->i_term = pid->i_term_max;
    else if (pid->i_term < pid->i_term_min)
        pid->i_term = pid->i_term_min;

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;

    if (pid->out_value > pid->out_max) pid->out_value = pid->out_max;
    else if (pid->out_value < pid->out_min)
        pid->out_value = pid->out_min;

    return pid->out_value;
}

float ParallelPidCtrl(pid_para_t *pid, float target_value, float fdback_value) {
    pid->target_value = target_value;
    pid->fback_value  = fdback_value;
    pid->error        = pid->target_value - pid->fback_value;
    pid->d_error      = pid->error - pid->pre_error;

    // LowPassFilter(&pid->error, pid->lpf_error);
    // LowPassFilter(&d_error, pid->lpf_d);
    if (ABS(pid->error) < pid->deadband) return 0;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->error;

    if (pid->i_term > pid->i_term_max) pid->i_term = pid->i_term_max;
    else if (pid->i_term < pid->i_term_min)
        pid->i_term = pid->i_term_min;

    pid->d_term = pid->d_error * pid->kd;

    pid->out_value = pid->p_term + pid->i_term + pid->d_term;

    if (pid->out_value > pid->out_max) pid->out_value = pid->out_max;
    else if (pid->out_value < pid->out_min)
        pid->out_value = pid->out_min;

    return pid->out_value;
}

float SerialPidCtrl(pi_para_t *pi, float target_value, float fdback_value) {
    pi->target_value = target_value;
    pi->fback_value  = fdback_value;
    pi->error        = pi->target_value - pi->fback_value;

    pi->p_term = pi->kp * pi->error;
    pi->i_term += pi->ki * pi->kp * (pi->error - pi->anti_term);

    if (pi->i_term > pi->i_term_max) pi->i_term = pi->i_term_max;
    if (pi->i_term < pi->i_term_min) pi->i_term = pi->i_term_min;

    pi->out_raw   = pi->p_term + pi->i_term;
    pi->out_value = pi->out_raw;

    if (pi->out_value > pi->out_max) pi->out_value = pi->out_max;
    if (pi->out_value < pi->out_min) pi->out_value = pi->out_min;

    pi->anti_term = pi->kAnti * ABS(pi->out_raw - pi->out_value);

    return pi->out_value;
}

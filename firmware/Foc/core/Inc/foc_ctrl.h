#ifndef __FOC_CTRL_H
#define __FOC_CTRL_H
#include "adc.h"
#include "foc_cfg.h"
#include "pid.h"
#include "analog_sense.h"

#ifdef __cplusplus
extern "C" {
#endif
_RAM_FUNC void HfiVolt(float vd, float vq, float pos, float inject);
_RAM_FUNC void HfiCurrent(float id_set, float iq_set, float pos);

void FocVolt(float vd_ref, float vq_ref, float pos);
void FocCurrent(float id_set, float iq_set, float pos);
void FocIFVolt(float id_ref, float pos);
#ifdef __cplusplus
}
#endif

class FocController
{
public:
    // 运行FOC控制
    void Run();

    // 设置控制模式
    void SetMode(FocCtrlMode_e mode);
    void SetCurrentSense(PhaseSenseBase* sense);

    // 设定值设置
    void SetVoltage(float vd, float vq);
    void SetCurrent(float id, float iq);
    void SetSpeed(float rpm);
    void SetPosition(float pos);

    IncrementalPid pid_spd_;
    IncrementalPid pid_pos_;
    PIController pid_id_;
    PIController pid_iq_;
    PhaseSenseBase *sense_{};
private:
    // 内部状态（替代foc_param）
    Vector2Df_t current_ab_{};
    Vector2Df_t current_dq_{};
    Vector2Df_t voltage_ab_{};
    Vector2Df_t voltage_dq_{};
    float angle_{};
    float velocity_{};
    float vbus_{};

    FocCtrlMode_e mode_;
};

#endif

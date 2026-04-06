//
// Created by 28076 on 25-6-14.
//
#ifndef FOC_IDENTITY_H
#define FOC_IDENTITY_H
#include "foc_cfg.h"
#include "stdbool.h"
typedef enum {
    IDENT_SET1 = 0,
    IDENT_SET2 = 1,
    IDENT_GET1 = 2,
    IDENT_GET2,
    IDENT_SUCCESS,
} IDENTIFY_STATUS_RES;

typedef enum {
    L_IDENT_IDLE,
    L_IDENT_ALIGN,       // 1. 转子定向（D轴对齐）
    L_IDENT_PULSE_D,     // 2. 脉冲法测 Ld
    L_IDENT_PULSE_Q,     // 3. 脉冲法测 Lq
    L_IDENT_HFI_D,       // 4. HFI测Ld
    L_IDENT_HFI_Q,       // 5. HFI测Lq
    L_IDENT_FINISH
} IDENT_STATUS_L;

typedef enum {
    FLUX_IDENT_IDLE,
    FLUX_IDENT_ALIGN,    // 1. D轴对齐
    FLUX_IDENT_MEASURE, // 2. 测量反电动势
    FLUX_IDENT_CALC,    // 3. 计算磁链
    FLUX_IDENT_FINISH
} IDENT_STATUS_FLUX;

typedef struct {
    float v_set1; // 直轴电压设定值
    float v_set2; // 交轴电压设定值
    /******************电阻识别参数**********************/
    IDENTIFY_STATUS_RES ident_status_res; // 识别状态
    IDENT_STATUS_L      ident_status_l;
    float               i_value1, i_value2;
    float               res; // 识别结果A
    float               Ld;
    float               Lq;
    /******************磁链识别参数**********************/
    IDENT_STATUS_FLUX   ident_status_flux;
    float               flux; // 磁链辨识结果 (mWb)
    // 多频率测量用的临时变量
    uint8_t             flux_sample_idx;      // 当前频率索引
    uint8_t             flux_measure_cnt;     // 测量计数
    float               flux_vq_sum;          // Vq累加
    float               flux_iq_sum;          // Iq累加
    float               flux_samples[4];      // 多频率点结果
    /******************电阻识别参数**********************/
} foc_identify_t;
#ifdef __cplusplus
extern "C" {
#endif
bool FocIdentifyRes(foc_identify_t *identify, FocParam_t *foc, float Is1, float Is2);
bool FocIdentifyInductance(foc_identify_t *ident, FocParam_t *foc);
bool FocIdentifyFlux(foc_identify_t *ident, FocParam_t *foc);
#ifdef __cplusplus
}
#endif
extern foc_identify_t identify;
#endif
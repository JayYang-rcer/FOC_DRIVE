//
// Created by 28076 on 25-6-14.
//
#ifndef FOC_IDENTITY_H
#define FOC_IDENTITY_H

#include "stdbool.h"
#include "foc_cfg.h"


typedef enum IDENTIFY_STATUS
{
    IDENT_SET = 0,
    IDENT_GET = 1,
    IDENT_SUCCESS,
}IDENTIFY_STATUS_RES;

typedef struct foc_identify_t
{
    float vd_set; //直轴电压设定值
    float vq_set; //交轴电压设定值

    /******************电阻识别参数**********************/
    IDENTIFY_STATUS_RES ident_status_res; //识别状态
    float res_a; //识别结果A
    float res_b; //识别结果B
    float res_c; //识别结果C
    /******************电阻识别参数**********************/
}foc_identify_t;

bool FocIdentifyRes(foc_identify_t* identify, foc_param_t* foc, float Is);
bool FocIdentifyTest(foc_identify_t* identify, foc_param_t* foc, float Is);
extern foc_identify_t identify_res;
#endif

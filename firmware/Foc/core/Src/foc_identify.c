//
// Created by 28076 on 25-6-14.
//
#include "foc_identify.h"
#include "foc_ctrl.h"
#include "filter.h"
#include "math.h"
#include "foc_math.h"

lpf_t identify_lpf_Is = {.in_last = 0.0f, .trust = 0.05f}; // 电流低通滤波器
volatile float Is_sum;
foc_identify_t identify_res = {
        .vd_set = 0.0f,
        .vq_set = 0.0f,
        .ident_status_res = IDENT_SET
};


bool FocIdentifyRes(foc_identify_t *identify, FocParam_t *foc, float Is) {
    switch (identify->ident_status_res) {
        case IDENT_SET: {
            FocPwmStart(true, true, true, true, false, false);
            LowPassFilter(&foc->i_a, &identify_lpf_Is); // 低通滤波处理
            if (foc->i_a > Is) {
                identify->ident_status_res = IDENT_GET;
            } else {
                identify->vd_set += 0.00001f; // 增加直轴电压
                foc->dtc_a = identify->vd_set / 12.2f + 42.5f / 4250.f; // 设置A相DTC
                foc->dtc_b = 0;
                foc->dtc_c = 0;
            }
            break;
        }

        case IDENT_GET: {
            static int cnt = 0;
            if (++cnt < 20000) //等待2000次采样
            {
                LowPassFilter(&foc->i_a, &identify_lpf_Is); // 低通滤波处理
                Is_sum += fabsf(foc->i_a); // 累加电流值
            } else {
                identify->ident_status_res = IDENT_SUCCESS;
                float i_value = Is_sum / 20000.0f; // 计算平均值
                identify->res_a = (identify->vd_set) / i_value * 1000.f * 0.5f; // 计算相电阻
                //清零
                foc->dtc_a = 0;
                foc->dtc_b = 0;
                foc->dtc_c = 0;
                break;
            }
        }

        case IDENT_SUCCESS: {
            return true;
            break;
        }

        default:
            break;
    }

    return false;
}


///**
// * @brief 电机相电阻辨识
// * @param identify 识别结构体指针
// * @param foc FOC参数结构体指针
// * @param U 注入dq轴的电压值
// * @teturn 识别是否成功
// */
//bool FocIdentifyTest(foc_identify_t* identify, FocParam_t* foc, float Is)
//{
//    switch (identify->ident_status_R)
//    {
//        case IDENT_SET:
//        {
//            FocPwmStart(true, true, true, true, true, true);
//            Clarke(foc);
//            Park(foc);
//            LowPassFilter(&foc->i_d, &identify_lpf_Is); // 低通滤波处理
//            if (foc->i_d > Is)
//            {
//                identify->ident_status_R = IDENT_GET;
//            }
//            else
//            {
//                identify->vd_set += 0.00001f; // 增加直轴电压
//                identify->vq_set = 0.0f; // 设置交轴电压为0
//                FocVolt(identify->vd_set, identify->vq_set, 0);
//            }
//            break;
//        }
//
//        case IDENT_GET:
//        {
//            static int cnt=0;
//            if(++cnt < 2000) //等待2000次采样
//            {
//                FocVolt(identify->vd_set, identify->vq_set, 0);
//                Clarke(foc);
//                Park(foc);
//                LowPassFilter(&foc->i_d, &identify_lpf_Is); // 低通滤波处理
//                Is_sum += fabsf(foc->i_d); // 累加电流值
//            }
//            else
//            {
//                identify->ident_status_R = IDENT_SUCCESS;
//                float i_value = Is_sum / 2000.0f; // 计算平均值
//                motor_cfg.rs = (identify->vd_set) / i_value * 1000.f; // 计算相电阻
//                FocPwmStop();
//            }
//            break;
//        }
//
//        case IDENT_SUCCESS:
//        {
//            return true;
//            break;
//        }
//
//        default: break;
//    }
//    return false;
//}

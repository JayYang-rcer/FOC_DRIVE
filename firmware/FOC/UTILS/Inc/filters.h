//
// Created by 28076 on 26-2-23.
//

#ifndef DRIVE_CMAKE_FILTERS_H
#define DRIVE_CMAKE_FILTERS_H

#include "main.h"
#include "util.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

// ========================================================================
// 一阶低通滤波器
// alpha: 滤波系数 (0~1)，越小越平滑
// ========================================================================
class LowPassFilter
{
public:
    explicit LowPassFilter(float alpha = 1.0f) : alpha_(alpha) {}

    // 方式1: 直接调用（推荐，适合中断）
    __RAM_FUNC void Update(float *data)
    {
        *data = *data * alpha_ + (1.0f - alpha_) * last_;
        last_ = *data;
    }

    __RAM_FUNC float Update(float data)
    {
        float raw = data;
        Update(&raw);
        return raw;
    }

    // 设置alpha
    void SetAlpha(float alpha) { alpha_ = alpha; }

    // 复位
    void Reset() { last_ = 0.0f; }

private:
    float alpha_;
    float last_ = 0.0f;
};

// ========================================================================
// Move windows filter
// length: length of window
// ========================================================================
template <int length>
class MeanFilter
{
    static_assert(length >= 2 && length <= 64, "length should be in [2,64]");

public:
    MeanFilter()
    {
        index_ = 0;
        sum_   = 0.0f;
        for (int i = 0; i < length; i++) {
            buffer_[i] = 0.0f;
        }
    }

    __RAM_FUNC float Update(float data)
    {
        sum_ -= buffer_[index_];
        sum_ += data;
        buffer_[index_] = data;
        index_          = (index_ + 1) % length;
        return sum_ / (float)length;
    }

    __RAM_FUNC void Update(float *data)
    {
        *data = Update(*data);
    }

    // 复位
    void Reset()
    {
        index_ = 0;
        sum_   = 0.0f;
        for (int i = 0; i < length; i++) {
            buffer_[i] = 0.0f;
        }
    }

private:
    float   buffer_[length]{};
    float   sum_;
    uint8_t index_;
};

#endif // DRIVE_CMAKE_FILTERS_H

#ifndef __UTIL_H
#define __UTIL_H

#include <float.h>
#include <math.h>
#include <stdint.h>

// Return the sign of the argument. -1.0 if negative, 1.0 if zero or positive.
#define SIGN(x)       (((x) < 0.0) ? -1.0 : 1.0)
// Two-norm of 2D vector
#define SQ(x)         ((x) * (x))
#define NORM2_f(x, y) (sqrtf(SQ(x) + SQ(y)))

#define ABS(x)        ((x) > 0 ? (x) : -(x))

#define min(x, y)     (((x) < (y)) ? (x) : (y)) // 取较小值
#define max(x, y)     (((x) > (y)) ? (x) : (y)) // 取较大值

// #define M_PI (3.14159265358f)         // 圆周率
#define M_2PI         (6.28318530716f) // 2倍圆周率
#define SQRT3         (1.73205080757f) // 3的平方根
#define SQRT3_BY_2    (0.86602540378f) // 3的平方根的一半
#define ONE_BY_SQRT3  (0.57735026919f) // 1除以3的平方根
#define TWO_BY_SQRT3  (1.15470053838f) // 2除以3的平方根
#define RPM_TO_RADS   (M_2PI / 60.0f)
#define RADS_TO_RPM   (60.0f / M_2PI)

#define WRAP_PM_PI(theta)                           \
    theta = (theta > M_PI) ? theta - M_2PI : theta; \
    theta = (theta < -M_PI) ? theta + M_2PI : theta; // Wrap theta to [-pi, pi)
#define WRAP_0_2PI(theta)                            \
    theta = (theta > M_2PI) ? theta - M_2PI : theta; \
    theta = (theta < 0.0f) ? theta + M_2PI : theta; // Wrap theta to [0, 2pi)


#ifdef __cplusplus
extern "C" {
#endif
float sat1_datf(float val, float up, float low);

float fast_atan2(float y, float x);

float sin_f32(float x);

float cos_f32(float x);

uint8_t crc8(const uint8_t *data, const uint32_t size);

uint32_t crc32(const uint8_t *data, uint32_t size);

int uint32_to_data(uint32_t val, uint8_t *data);

int int32_to_data(int32_t val, uint8_t *data);

int uint16_to_data(uint16_t val, uint8_t *data);

int int16_to_data(int16_t val, uint8_t *data);

int float_to_data(float val, uint8_t *data);

uint32_t data_to_uint32(uint8_t *data);

int32_t data_to_int32(uint8_t *data);

uint16_t data_to_uint16(uint8_t *data);

int16_t data_to_int16(uint8_t *data);

float data_to_float(uint8_t *data);
#ifdef __cplusplus
}
#endif
#endif

//
// Created by 28076 on 26-2-10.
//

#ifndef DRIVE_CMAKE_ENCODER_H
#define DRIVE_CMAKE_ENCODER_H
#include "foc_cfg.h"
#include "pid_class.h"
#include "stdint-gcc.h"
#include "tim.h"
#include "util.h"
#include <cassert>
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

enum class EncoderType : uint8_t {
    NONE = 0,
    ABI,     // 增量式编码器
    AS5047P, // AS5047P磁编码器
    AS5600,  // AS5600磁编码器(预留)
    HALL     // Hall传感器
};

enum class As5047pAddress : uint16_t {
    NOP      = 0x0000, // No operation
    ERRFL    = 0x0001, // Error register
    PROG     = 0x0003, // Programming register
    DIAAGC   = 0x3FFC, // Diagnostic and AGC
    MAG      = 0x3FFD, // CORDIC magnitude
    ANGLEUNC = 0x3FFE, // Measured angle without dynamic angle error compensation
    ANGLECOM = 0x3FFF, // Measured angle with dynamic angle error compensation
};

enum class EncoderError : uint16_t {
    NONE,
    NULL_POINTER,
    NOT_INITIALIZED,
    PARERR,  // Parity error
    INVCOMM, // Invalid command error: set to 1 by reading or writing an invalid register address
    FRERR,   // Framing error: is set to 1 when a non-compliant SPI frame is detected
    MAGL,    // Magnetic field strength too low;
    MAGH,    // Magnetic field strength too high
    COF,     // CORDIC overflow
    LF,      // Offset compensation 0:Internal offset loops not ready regulated LF=1: Internal offset loop finished
    TIMEOUT,
};

// SPI传输函数类型
using SpiTransferFunc = uint16_t (*)(uint16_t addr);

// clang-format off
inline const char* ToString(EncoderError err) {
    switch (err) {
    case EncoderError::NONE:             return "OK";
    case EncoderError::NULL_POINTER:     return "NULL_POINTER";
    case EncoderError::NOT_INITIALIZED:  return "NOT_INITIALIZED";
    case EncoderError::PARERR:           return "PARITY_ERROR";
    case EncoderError::INVCOMM:          return  "SPI_TRANSFER_FAILED";
    case EncoderError::FRERR:            return "FRAMING_ERROR";
    case EncoderError::TIMEOUT:          return "TIMEOUT";
    default:                             return "UNKNOWN";
    }
}
// clang-format on

class SpeedPLLMonitor : public PIController
{
public:
    void Init(const Config &cfg) { PiInit(cfg); }

    float GetSpeed(float theta)
    {
        float ref   = sin_f32(theta) * cos_f32(velocity_);
        float fbk   = sin_f32(velocity_) * cos_f32(theta);
        float error = ref - fbk;
        velocity_   = Calculate(error);

        velocity_ *= RADS_TO_RPM;
        return velocity_;
    }

private:
    float velocity_ = 0.0f;
    bool  is_ready_ = false;
};

class AngleProvider
{
public:
    virtual ~AngleProvider() = default;
    [[nodiscard]] inline float GetElectAngle() const { return theta_elect_; }
    [[nodiscard]] inline float GetVelocity() const { return velocity_; }

protected:
    float theta_elect_ = 0.0f;
    float velocity_    = 0.0f;
};

class EncoderBase : public AngleProvider
{
public:
    // Get Angle
    [[nodiscard]] float GetMachineAngle() const { return theta_multi_; }

    void SetElectOffset(float offset) { offset_elect_ = offset; }

    // Get Error status
    [[nodiscard]] EncoderError GetError() const { return error_; }
    [[nodiscard]] bool         HasError() const { return error_ != EncoderError::NONE; }
    [[nodiscard]] bool         IsValid() const { return valid_; }

protected:
    EncoderError error_ = EncoderError::NONE;

    bool     valid_      = false;
    uint8_t  pole_pairs_ = 0;
    uint16_t cpr_        = 0;
    int8_t   dir_        = 1;
    int16_t  rev_        = 0;
    float    factor_     = 0.0f;

    float velocity_    = 0.0f;
    float theta_       = 0.0f;
    float theta_last_  = 0.0f;
    float theta_multi_ = 0.0f;

    float theta_elect_  = 0.0f;
    float offset_elect_ = 0.0f;

    void EncoderDataProc(uint16_t raw_data);

    virtual void ElectAngleWriting(float offset) { offset_elect_ = offset; }
};

class AbiEncoder : public EncoderBase
{
public:
    struct Config {
        uint32_t     cpr; // count per route
        uint8_t      pole_pairs;
        TIM_TypeDef *tim_handle;
    };
    AbiEncoder() = default;

    void Init(const Config &cfg)
    {
        this->pole_pairs_      = cfg.pole_pairs;
        this->cpr_             = cfg.cpr;
        this->factor_          = M_2PI / (float)(cfg.cpr);
        this->tim_handle_      = cfg.tim_handle;
        this->tim_handle_->ARR = cfg.cpr - 1;
    }

    inline void Update() { EncoderDataProc(tim_handle_->CNT); }

private:
    TIM_TypeDef *tim_handle_{};
};

class As5407Encoder : public EncoderBase
{
public:
    struct Config {
        uint32_t        cpr; // count per route
        uint8_t         pole_pairs;
        SpiTransferFunc As5047RawDataGettingFun;
    };

    void Init(const Config &cfg)
    {
        this->pole_pairs_     = cfg.pole_pairs;
        this->cpr_            = cfg.cpr;
        this->factor_         = M_2PI / (float)(cfg.cpr);
        this->RawDataGetting_ = cfg.As5047RawDataGettingFun;
    }
    inline void Update()
    {
        uint16_t received_data = RawDataGetting_((uint16_t)(As5047pAddress::ANGLECOM));
        EncoderDataProc(received_data);
    }

    EncoderError GetErrflErrorInfo()
    {
        uint16_t received_data = RawDataGetting_((uint16_t)(As5047pAddress::ERRFL));
        return (EncoderError(received_data));
    }

    EncoderError GetDiaagcErrorInfo()
    {
        uint16_t received_data = RawDataGetting_((uint16_t)(As5047pAddress::DIAAGC));
        return (EncoderError(received_data));
    }

private:
    SpiTransferFunc RawDataGetting_{};
};

class HallEncoder : public AngleProvider
{
public:
    struct Config {
        uint8_t      pole_pairs;
        TIM_TypeDef *tim_handle;
    };
    explicit HallEncoder(const Config &cfg)
    {
        this->pole_pairs_ = cfg.pole_pairs;
        this->tim_handle_ = cfg.tim_handle;
    }

    void calibration() {}
    void Update() {}

private:
    EncoderError error_ = EncoderError::NONE;
    bool         valid_ = false;
    uint8_t      pole_pairs_;

    float velocity_    = 0.0f;
    float theta_elect_ = 0.0f;

    float        offset_ccw_[6]{0};
    float        offset_cw_[6]{0};
    TIM_TypeDef *tim_handle_;
};

#endif // DRIVE_CMAKE_ENCODER_H

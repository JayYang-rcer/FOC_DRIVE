//
// Created by 28076 on 26-2-10.
//

#ifndef DRIVE_CMAKE_ENCODER_H
#define DRIVE_CMAKE_ENCODER_H
#include "stdint-gcc.h"
#include "util.h"
#include "foc_cfg.h"
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

enum class AS5047P_ADDR : uint16_t {
    NOP      = 0x0000, // No operation
    ERRFL    = 0x0001, // Error register
    PROG     = 0x0003, // Programming register
    DIAAGC   = 0x3FFC, // Diagnostic and AGC
    MAG      = 0x3FFD, // CORDIC magnitude
    ANGLEUNC = 0x3FFE, // Measured angle without dynamic angle error compensation
    ANGLECOM = 0x3FFF, // Measured angle with dynamic angle error compensation
};

enum class AS5047_ERROR : uint16_t {
    PARERR,  // Parity error
    INVCOMM, // Invalid command error: set to 1 by reading or writing an invalid register address
    FRERR,   // Framing error: is set to 1 when a non-compliant SPI frame is detected
};

class AngleGetBase
{
public:
    [[nodiscard]] const float &GetElectAngle() const { return theta_elect_; }
    [[nodiscard]] const float &GetMachineAngle() const { return theta_multi_; }

protected:
    uint8_t  pole_pairs_;
    uint16_t cpr_;
    int8_t   dir_;
    uint16_t rev_;
    float    factor_;

    float theta_;
    float theta_last_;
    float theta_diff_;
    float theta_multi_;
    float offset_machine_;

    float theta_elect_;
    float offset_elect_;

    void EncoderDataProc(uint16_t raw_data);
};

class AbiEncoder : public AngleGetBase
{
public:
    struct Config {
        uint32_t cpr; // count per route
        uint8_t  pole_pairs;
    };
    explicit AbiEncoder(const Config &cfg) : AngleGetBase()
    {
        pole_pairs_ = cfg.pole_pairs;
        cpr_        = cfg.cpr;
        factor_     = M_2PI / (float)(cfg.cpr);
    }

    inline void Update(uint16_t raw_data) { EncoderDataProc(raw_data); }
};

class As5407Encoder : public AngleGetBase
{
public:
    typedef uint16_t (*SpiTransferFunc)(uint16_t);
    struct Config {
        uint32_t        cpr; // count per route
        uint8_t         pole_pairs;
        SpiTransferFunc As5047RawDataGettingFun;
    };

    explicit As5407Encoder(const Config &cfg) : AngleGetBase()
    {
        pole_pairs_     = cfg.pole_pairs;
        cpr_            = cfg.cpr;
        factor_         = M_2PI / (float)(cfg.cpr);
        RawDataGetting_ = cfg.As5047RawDataGettingFun;
    }

    inline void Update()
    {
        uint16_t received_data = RawDataGetting_((uint16_t)(AS5047P_ADDR::ANGLECOM));
        EncoderDataProc(received_data);
    }

    AS5047_ERROR GetErrorInfo()
    {
        uint16_t received_data = RawDataGetting_((uint16_t)(AS5047P_ADDR::ERRFL));
        return (AS5047_ERROR(received_data));
    }

private:
    SpiTransferFunc RawDataGetting_;
};

#endif // DRIVE_CMAKE_ENCODER_H

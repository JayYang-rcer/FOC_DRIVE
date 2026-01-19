#ifndef __AS5047P_H__
#define __AS5047P_H__

#include "spi.h"
#include "foc_cfg.h"

typedef enum AS5047P_ENUM {
    NOP = 0x0000,       //No operation
    ERRFL = 0x0001,     //Error register
    PROG = 0x0003,      //Programming register
    DIAAGC = 0x3FFC,    //Diagnostic and AGC
    MAG = 0x3FFD,       //CORDIC magnitude
    ANGLEUNC = 0x3FFE,  //Measured angle without dynamic angle error compensation
    ANGLECOM = 0x3FFF,  //Measured angle with dynamic angle error compensation
} AS5047P_ENUM;

typedef struct enc_para_t {
    int8_t dir;
    int8_t rev;     //revolution
    uint8_t pn;     //pole number

    uint16_t raw_data;
    uint16_t raw_data_last;
    uint16_t cpr;   //counts per revolution
    uint8_t bit;    //resolution
    uint8_t shift_bit;

    float pos;
    float pos_e;
    float pos_s;        // single turn pos
    float pos_m;        // multi turn pos

    float offset_mpos;
    float offset_epos;

    float pos_last;
    float pos_diff;
    float factor;

    bool enc_init;
} enc_para_t;

extern enc_para_t enc_para;

uint16_t ParityBitCalculate(uint16_t data);

uint16_t SpiReadWriteOneByte(uint16_t addr);

uint16_t As5047pRead(uint16_t addr);

void SpeedMeasure(enc_para_t *enc, MotorCfg_t *motor);

#endif

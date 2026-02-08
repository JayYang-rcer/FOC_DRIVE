//
// Created by 28076 on 25-4-16.
//

#ifndef DRIVE_CMAKE_ENCODER_PROC_H
#define DRIVE_CMAKE_ENCODER_PROC_H

#include "as5047p.h"
#include "foc_cfg.h"
#ifdef __cplusplus
extern "C" {
#endif
void PosCalculate(enc_para_t *enc);

void EncoderInit(void);
#ifdef __cplusplus
}
#endif
#endif //DRIVE_CMAKE_ENCODER_PROC_H

//
// Created by 28076 on 25-4-16.
//

#include "encoder_proc.h"
#include "util.h"
#include "moc_spd.h"


void EncoderInit(void)
{
    enc_para.cpr = 16383;
    enc_para.bit = 14;
    // enc_para.shift_bit = 2;
    enc_para.pn = 7;
    enc_para.dir = 1;   //当矢量角度和编码器角度方向相反时，需要设置为！1
    enc_para.rev = 0;
    enc_para.pos = 0.0f;
    enc_para.pos_e = 0.0f;
    enc_para.pos_s = 0.0f;
    enc_para.pos_m = 0.0f;
    enc_para.offset_mpos = 1.82557607f;
    enc_para.offset_epos = 0.212661743f;
    enc_para.pos_last = 0.0f;
    enc_para.pos_diff = 0.0f;
    enc_para.factor = M_2PI / enc_para.cpr;
}


//计算转子的多圈角度值，还有转子的电角度值，并且要计算好方向
_RAM_FUNC void PosCalculate(enc_para_t* enc)
{
    // read the raw data
    enc->raw_data = As5047pRead(ANGLECOM);
    if(enc->dir == 1)
        enc->pos = (float)enc->raw_data * enc->factor;
    else
        enc->pos = (float)(enc->cpr - enc->raw_data) * enc->factor;
    WRAP_0_2PI(enc->pos);

    enc->pos_diff = enc->pos - enc->pos_last;
    // calculate the single position
    enc->pos_s = enc->pos + enc->offset_mpos;
    WRAP_0_2PI(enc->pos_s);

    // calculate the electrical position
    enc->pos_e = enc->pos*enc->pn - (uint16_t)(enc->pos*enc->pn/M_2PI)*M_2PI + enc->offset_epos;
    WRAP_0_2PI(enc->pos_e);

    // count the revolution
    if(enc->pos_diff > 0.8f*M_2PI)
        enc->rev--;
    else if(enc->pos_diff < -0.8f*M_2PI)
        enc->rev++;

    enc->pos_last = enc->pos;
    enc->pos_m = enc->pos + enc->rev*M_2PI;
}


void GetAngle(motor_cfg_t* motor)
{

}

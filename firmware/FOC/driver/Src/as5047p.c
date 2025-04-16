/*
AS5047p data frame:

command frame: 16 bits
|  15  | 14  | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
parity | w/r |                   addr                                    |

data frame: 16 bits
|  15  |   14  | 13 | 12 | 11 | 10 | 9 | 8 | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
parity | error |                   data                                    |

*/

#include "as5047p.h"
#include "util.h"

#define AS5047P_CS_UP     HAL_GPIO_WritePin(ENC_NSS_GPIO_Port, ENC_NSS_Pin, GPIO_PIN_SET);
#define AS5047P_CS_DOWN   HAL_GPIO_WritePin(ENC_NSS_GPIO_Port, ENC_NSS_Pin, GPIO_PIN_RESET);

enc_para_t enc_para;

/**
 * @brief  Parity bit calculation
 */
_RAM_FUNC uint16_t ParityBitCalculate(uint16_t data)
{
    uint16_t parity_bit_value=0;
		
		while(data != 0)
		{
			parity_bit_value ^= data; 
			data >>=1;
		}
		return (parity_bit_value & 0x1);    
}


_RAM_FUNC uint16_t SpiReadWriteOneByte(uint16_t addr)
{
		AS5047P_CS_DOWN
	
    uint16_t rx_data;
	
    if(HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&addr, (uint8_t *)&rx_data, 1, 1000) != HAL_OK)
    {
        rx_data = 0;
    } 
		
    AS5047P_CS_UP

    return rx_data;
}


//需要添加奇偶检验，包括发送和接受
_RAM_FUNC uint16_t As5047pRead(uint16_t addr)
{
    uint16_t data;
    addr |= 0x4000; // set bit 14 as 1, read command 1, b0100 0000 0000 0000
    if(ParityBitCalculate(addr) == 0)
        addr |= 0x8000; // set bit 15 as 1, b1000 0000 0000 0000
		
    SpiReadWriteOneByte(addr);
    data = SpiReadWriteOneByte(addr);
		
    //3.检查错误及校验
    //偶校验最高位 通过则读取数据
    if((data>>15) == ParityBitCalculate(data & 0x7FFF))
    {
        return (data & 0x3FFF);
    }
    else
    {
        return 0;
    }
}


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
    //enc_para.offset_mpos = 1.00204933;
    //enc_para.offset_epos = 0.731159687f-4.64f;
    enc_para.offset_mpos = 1.82557607;
    enc_para.offset_epos = 0.212661743;
	enc_para.pos_last = 0.0f;
    enc_para.pos_diff = 0.0f;
    enc_para.factor = M_2PI / enc_para.cpr;
}

volatile float spd=0.f;
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


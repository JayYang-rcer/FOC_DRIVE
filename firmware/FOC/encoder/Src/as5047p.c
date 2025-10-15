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

#define AS5047P_CS_UP \
    HAL_GPIO_WritePin(ENC_NSS_GPIO_Port, ENC_NSS_Pin, GPIO_PIN_SET);
#define AS5047P_CS_DOWN \
    HAL_GPIO_WritePin(ENC_NSS_GPIO_Port, ENC_NSS_Pin, GPIO_PIN_RESET);

enc_para_t enc_para;

/**
 * @brief  Parity bit calculation
 */
_RAM_FUNC uint16_t ParityBitCalculate(uint16_t data) {
    uint16_t parity_bit_value = 0;

    while (data != 0) {
        parity_bit_value ^= data;
        data >>= 1;
    }
    return (parity_bit_value & 0x1);
}

_RAM_FUNC uint16_t SpiReadWriteOneByte(uint16_t addr) {
    AS5047P_CS_DOWN

    uint16_t rx_data;

    if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) &addr, (uint8_t *) &rx_data,
                                1, 1000)
        != HAL_OK) {
        rx_data = 0;
    }

    AS5047P_CS_UP

    return rx_data;
}

// 需要添加奇偶检验，包括发送和接受
_RAM_FUNC uint16_t As5047pRead(uint16_t addr) {
    uint16_t data;
    addr |= 0x4000;// set bit 14 as 1, read command 1, b0100 0000 0000 0000
    if (ParityBitCalculate(addr) == 0)
        addr |= 0x8000;// set bit 15 as 1, b1000 0000 0000 0000

    SpiReadWriteOneByte(addr);
    data = SpiReadWriteOneByte(addr);

    // 3.检查错误及校验
    // 偶校验最高位 通过则读取数据
    if ((data >> 15) == ParityBitCalculate(data & 0x7FFF)) {
        return (data & 0x3FFF);
    } else {
        return 0;
    }
}

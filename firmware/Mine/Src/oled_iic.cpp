//
// Created by 28076 on 26-3-13.
//
#include "oled_iic.h"

/**
 * @brief OLED初始化函数
 */
void OLED::Init()
{
    WriteCmd(0xAE); // 显示关闭
    WriteCmd(0x20); // 设置内存寻址模式
    WriteCmd(0x10); // 寻址模式：页寻址模式(复位)
    WriteCmd(0xb0); // 设置页起始地址,0-7
    WriteCmd(0xc8); // 设置COM输出扫描方向
    WriteCmd(0x00); // 设置低列地址
    WriteCmd(0x10); // 设置高列地址
    WriteCmd(0x40); // 设置起始行地址
    WriteCmd(0x81); // 设置对比度控制
    WriteCmd(0xff); // 亮度调节 0x00~0xff
    WriteCmd(0xa1); // 设置段重新映射
    WriteCmd(0xa6); // 设置正常显示
    WriteCmd(0xa8); // 设置复用比例
    WriteCmd(0x3F);
    WriteCmd(0xa4); // 输出遵循RAM内容
    WriteCmd(0xd3); // 设置显示偏移
    WriteCmd(0x00); // 不偏移
    WriteCmd(0xd5); // 设置显示时钟分频
    WriteCmd(0xf0);
    WriteCmd(0xd9); // 设置预充电周期
    WriteCmd(0x22);
    WriteCmd(0xda); // 设置COM硬件配置
    WriteCmd(0x12);
    WriteCmd(0xdb); // 设置VCOMH
    WriteCmd(0x20);
    WriteCmd(0x8d); // 电荷泵设置
    WriteCmd(0x14); // 使能
    WriteCmd(0xaf); // 开启显示

    OLED_ClearRAM();
    OLED_RefreshRAM();
}

/**
 * @brief 硬件层级清屏（直接填充寄存器）
 */
void OLED::OLED_CLS()
{
    for (uint8_t m = 0; m < 8; m++) {
        WriteCmd(0xb0 + m);
        WriteCmd(0x00);
        WriteCmd(0x10);
        for (uint8_t n = 0; n < 128; n++) {
            WriteDat(0x00);
        }
    }
}

/**
 * @brief 唤醒OLED
 */
void OLED::OLED_ON()
{
    WriteCmd(0X8D);
    WriteCmd(0X14);
    WriteCmd(0XAF);
}

/**
 * @brief 休眠OLED
 */
void OLED::OLED_OFF()
{
    WriteCmd(0X8D);
    WriteCmd(0X10);
    WriteCmd(0XAE);
}

/**
 * @brief 将数据缓冲区(buffer_)的内容推送到屏幕
 */
void OLED::OLED_RefreshRAM()
{
        for (uint16_t m = 0; m < SCREEN_ROW / 8; m++) {
            WriteCmd(0xb0 + m);
            WriteCmd(0x00);
            WriteCmd(0x10);
            for (uint16_t n = 0; n < SCREEN_COLUMN; n++) {
                WriteDat(buffer_[m * SCREEN_COLUMN + n]);
            }
        }
//    WriteDatDma(buffer_);
}

/**
 * @brief 清空软件缓冲区
 */
void OLED::OLED_ClearRAM()
{
    for (uint16_t i = 0; i < (SCREEN_ROW * SCREEN_COLUMN / 8); i++) {
        buffer_[i] = 0x00;
    }
}

/**
 * @brief 画点函数（操作缓冲区）
 */
void OLED::OLED_SetPixel(int16_t x, int16_t y, uint8_t set_pixel)
{
    if (x >= 0 && x < SCREEN_COLUMN && y >= 0 && y < SCREEN_ROW) {
        if (set_pixel)
            buffer_[(y / 8) * SCREEN_COLUMN + x] |= (0x01 << (y % 8));
        else
            buffer_[(y / 8) * SCREEN_COLUMN + x] &= ~(0x01 << (y % 8));
    }
}

/**
 * @brief 设置显示模式（正常/反相）
 */
void OLED::OLED_DisplayMode(uint8_t mode)
{
    WriteCmd(mode);
}

/**
 * @brief 对比度（亮度）控制
 */
void OLED::OLED_IntensityControl(uint8_t intensity)
{
    WriteCmd(0x81);
    WriteCmd(intensity);
}

/**
 * @brief 垂直偏移显示
 */
void OLED::OLED_Shift(uint8_t shift_num)
{
    for (uint8_t i = 0; i < shift_num; i++) {
        WriteCmd(0xd3);
        WriteCmd(i);
//        HAL_Delay(10);
    }
}

/**
 * @brief 硬件水平滚动设置
 */
void OLED::OLED_HorizontalShift(uint8_t start_page, uint8_t end_page, uint8_t direction)
{
    WriteCmd(0x2e); // 关闭滚动
    WriteCmd(direction);
    WriteCmd(0x00);
    WriteCmd(start_page);
    WriteCmd(0x05);
    WriteCmd(end_page);
    WriteCmd(0x00);
    WriteCmd(0xff);
    WriteCmd(0x2f); // 开启滚动
}

/**
 * @brief 显示字符串
 */
void OLED::OLED_ShowStr(int16_t x, int16_t y, const char *str, uint8_t TextSize, bool is_invert)
{
    if (x < 0 || x >= SCREEN_COLUMN || y < 0 || y >= SCREEN_ROW || str == nullptr) {
        return;
    }

    int32_t       c      = 0;
    unsigned char j      = 0;
    unsigned char char_w = (TextSize == 1) ? 6 : 8;
    unsigned char char_h = (TextSize == 1) ? 8 : 16;

    while (str[j] != '\0') {
        c = (unsigned char)str[j] - 32;
        if (c < 0 || c > 95) // 仅处理标准ASCII可见字符范围
        {
            j++;
            continue;
        }

        // 自动换行逻辑
        if (x + char_w > SCREEN_COLUMN) {
            x = 0;
            y += char_h;
        }

        // 垂直越界检查
        if (y + char_h > SCREEN_ROW) {
            break;
        }

        if (TextSize == 1) // 6x8 字体
        {
            for (unsigned char m = 0; m < 6; m++) {
                unsigned char byte = F6x8[c][m];
                for (unsigned char n = 0; n < 8; n++) {
                    bool pixel = (byte >> n) & 0x01;
                    OLED_SetPixel(x + m, y + n, is_invert == !pixel);
                }
            }
        } else if (TextSize == 2) // 8x16 字体
        {
            for (unsigned char m = 0; m < 2; m++) // 两个Page
            {
                for (unsigned char n = 0; n < 8; n++) // 宽度8
                {
                    unsigned char byte = F8X16[c][n + m * 8];
                    for (unsigned char i = 0; i < 8; i++) // 高度8
                    {
                        bool pixel = (byte >> i) & 0x01;
                        OLED_SetPixel(x + n, y + i + m * 8, is_invert == !pixel);
                    }
                }
            }
        }

        x += char_w;
        j++;
    }
}
/**
 * @brief 显示中文（GB2312）
 */
void OLED::OLED_ShowChinese(int16_t x, int16_t y, uchar *ch)
{
    if (x < 0 || y < 0)
        return;

    int32_t len    = 0;
    uchar   offset = 2; // GB2312占2字节

    while (ch[len] != '\0') {
        if (x > (SCREEN_COLUMN - 16)) {
            x = 0;
            y += 16;
        }
        if (y > (SCREEN_ROW - 16))
            break;

        for (uchar i = 0; i < sizeof(F16x16_CN) / sizeof(GB2312_CN); i++) {
            // 匹配中文字符索引
            if ((F16x16_CN[i].index[0] == ch[len]) && (F16x16_CN[i].index[1] == ch[len + 1])) {
                for (uint8_t m = 0; m < 2; m++) {
                    for (uint8_t n = 0; n < 16; n++) {
                        for (uint8_t j = 0; j < 8; j++) {
                            OLED_SetPixel(x + n, y + j + m * 8, (F16x16_CN[i].encoder[n + m * 16] >> j) & 0x01);
                        }
                    }
                }
                x += 16;
                len += offset;
                break;
            }
        }
    }
    OLED_RefreshRAM();
}

/**
 * @brief 显示图片
 */
void OLED::OLED_ShowBMP(int16_t x0, int16_t y0, int16_t L, int16_t H, const uchar BMP[])
{
    // 边界检查
    if (x0 < 0 || y0 < 0 || x0 + L > SCREEN_COLUMN || y0 + H > SCREEN_ROW)
        return;

    uchar *p = (uchar *)BMP;
    for (int16_t y = y0; y < y0 + H; y += 8) {
        for (int16_t x = x0; x < x0 + L; x++) {
            for (int16_t i = 0; i < 8; i++) {
                // 只有在图像范围内才绘制
                if ((y + i) < (y0 + H)) {
                    OLED_SetPixel(x, y + i, ((*p) >> i) & 0x01);
                }
            }
            p++;
        }
    }
    OLED_RefreshRAM();
}
#ifndef _OLED_IIC_H
#define _OLED_IIC_H

#include "i2c.h"
#include "oled_front.h" // 确保此文件内含 F6x8, F8X16, F16x16_CN 等字库定义

typedef unsigned char uchar;

/* OLED 硬件地址 */
#define OLED_ADDRESS 0x78

/* 屏幕参数 */
#define SCREEN_COLUMN      (128) // 总列数
#define SCREEN_ROW         (64)  // 总行数
#define SCREEN_PAGE_NUM    (8)   // 屏幕页数 (64/8)

/* OLED 控制命令宏 */
#define LEFT   0x27
#define RIGHT  0x26
#define ON     0xA7 // 反相显示
#define OFF    0xA6 // 正常显示

#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
}
#endif

class OLED
{
public:
    /**
     * @brief 构造函数
     * @param iic 指向硬件 I2C 句柄的指针
     * @param buffer 指向外部定义的全屏缓冲区(需1024字节: 128*64/8)
     */
    OLED(I2C_HandleTypeDef *iic, uchar *buffer) : iic_(iic), buffer_(buffer) {}

    /* --- 系统控制函数 --- */
    void Init();             // 初始化 OLED
    void OLED_ON();          // 唤醒屏幕/开启电荷泵
    void OLED_OFF();         // 屏幕休眠/关闭电荷泵
    void OLED_DisplayMode(uint8_t mode);       // 设置正常/反相显示
    void OLED_IntensityControl(uint8_t intensity); // 亮度/对比度调节(0-255)

    /* --- 显存/缓冲区操作 --- */
    void OLED_RefreshRAM();  // 将缓冲区内容推送到 OLED 硬件
    void OLED_ClearRAM();    // 清空软件缓冲区(buffer_)
    void OLED_CLS();        // 硬件清屏(直接清除寄存器内容)

    /* --- 绘图与显示函数 --- */
    // 画点：set_pixel=1点亮, 0熄灭
    void OLED_SetPixel(int16_t x, int16_t y, uint8_t set_pixel);

    // 显示字符串：TextSize 1(6x8), 2(8x16)
    void OLED_ShowStr(int16_t x, int16_t y, const char *str, uint8_t TextSize, bool is_invert);
    
    // 显示中文：传入中文字符串索引
    void OLED_ShowChinese(int16_t x, int16_t y, uchar *ch);
    
    // 显示图片：x,y为起点，L,H为宽高，BMP为图片数组
    void OLED_ShowBMP(int16_t x0, int16_t y0, int16_t L, int16_t H, const uchar BMP[]);

    /* --- 特效函数 --- */
    void OLED_Shift(uint8_t shift_num); // 垂直偏移
    void OLED_HorizontalShift(uint8_t start_page, uint8_t end_page, uint8_t direction); // 水平滚动

private:
    /* 底层通讯函数 */
    void WriteCmd(uint8_t cmd)
    {
        I2C_WriteByte(0x00, cmd);
    }

    void WriteDat(uint8_t dat)
    {
        I2C_WriteByte(0x40, dat);
    }

    void I2C_WriteByte(uint8_t addr, uint8_t data)
    {
        // 使用 HAL 库进行 I2C 通讯
        HAL_I2C_Mem_Write(iic_, OLED_ADDRESS, addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
    }

    I2C_HandleTypeDef *iic_;   // I2C 句柄
    uchar *buffer_;    // 显存缓冲区指针
};

#endif /* _OLED_H */
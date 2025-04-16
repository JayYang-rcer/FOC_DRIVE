#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include "usbd_cdc_if.h"


#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))

void usb_printf(const char *format, ...);
void VofaStart(void);
void VofaSendData(uint8_t num, float data); 
void VofaSendframetail(void);

#endif /* __VOFA_H__ */









//#include "drv8301.h"
//#include "gpio.h"
//#include "drive_tim.h"
//
///*  1.要先是能ENGATE之后才可以进行spi通信
//    2.8301的SPI通信要先写状态寄存器(检查是否有fault)，再读控制寄存器。读取到的值存放在Drv8301SpiCfg_t结构体中
//    3.有一个任务是定时读取8301的状态寄存器，如果有fault则停止电机，并且通过GPIO读取nFAULT引脚的状态
//    4.8301的控制寄存器的配置要根据具体的电机参数来配置，其配置的结构体是Drv8301SpiCfg_t
//    5.8301的句柄是Drv8301Handle_t，配置其GPIO和SPI的句柄
//*/
//
//
///**
// * @brief DRV8301的句柄配置
// */
//Drv8301Handle_t gDrv8301Handle = {
//    .spiHandle = &hspi1,
//    .csPort = DRV_nCS_GPIO_Port,
//    .csPin = DRV_nCS_Pin,
//    .enGatePort = EN_GATE_GPIO_Port,
//    .enGatePin = EN_GATE_Pin,
//    .nFaultPort = DRV_nFAULT_GPIO_Port,
//    .nFaultPin = DRV_nFAULT_Pin,
//};
//
///**
// * @brief DRV8301的SPI寄存器配置
// */
//Drv8301SpiCfg_t gDrv8301SpiCfg = {
//    .drv_reg_ctrl1.peak_current = kDrv8301_PeakCurrent_1p70A,
//    .drv_reg_ctrl1.gate_rstmode = kDrv8301_GateMode_NormalMode,
//    .drv_reg_ctrl1.pwm_mode = kDrv8301_PWMMode_Six,
//    .drv_reg_ctrl1.ocp_mode = kDrv8301_OcpMode_LatchShutdown,
//    .drv_reg_ctrl1.oc_adj = kDrv8301_VdsLevel_0p511_V,
//
//    .drv_reg_ctrl2.octw_mode = kDrv8301_OctwMode_Both,
//    .drv_reg_ctrl2.gain = kDrv8301_Gain_10, // 0.005R * 10 * 25A = 1.25V, 考虑换电阻阻值高一点的采样电阻。0.4V - 2.9V
//    .drv_reg_ctrl2.dc_cal_ch1 = kDrv8301_DcCalMode_Ch1_Load, // calibration off (normal operation)
//    .drv_reg_ctrl2.dc_cal_ch2 = kDrv8301_DcCalMode_Ch2_Load,
//    .drv_reg_ctrl2.oc_toff = kDrv8301_OcToff_CycleByCycle,
//};
//
//bool Drv8301Init(Drv8301Handle_t *handle)
//{
//    HAL_GPIO_WritePin(handle->enGatePort, handle->enGatePin, GPIO_PIN_SET);
//    return true;
//}
//
//
//uint32_t DrvSendData(Drv8301SpiCfg_t *cfg)
//{
//    uint16_t data[2];
//    data[0] = cfg->drv_reg_ctrl1.peak_current | \
//                cfg->drv_reg_ctrl1.gate_rstmode  | \
//                cfg->drv_reg_ctrl1.pwm_mode | \
//                cfg->drv_reg_ctrl1.ocp_mode | \
//                cfg->drv_reg_ctrl1.oc_adj;
//
//    data[1] = cfg->drv_reg_ctrl2.octw_mode | \
//                cfg->drv_reg_ctrl2.gain | \
//                cfg->drv_reg_ctrl2.dc_cal_ch1 | \
//                cfg->drv_reg_ctrl2.dc_cal_ch2 | \
//                cfg->drv_reg_ctrl2.oc_toff;
//
//		Drv8301WriteReg(kDrv8301_RegNameControl_1, data[0]);
//        Drv8301WriteReg(kDrv8301_RegNameControl_2, data[1]);
//	return 0;
//}
//
//
//uint16_t DrvReadData(Drv8301SpiCfg_t *cfg)
//{
//    uint16_t recieve_value;
//    // Update Status Register 1
//    Drv8301ReadReg(kDrv8301_RegNameStatus_1, &recieve_value);
//    cfg->status_reg_1_value = recieve_value;
//    cfg->drv_reg_status1.FETLC_OC = (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_FETLC_OC_BITS);
//    cfg->drv_reg_status1.FETHC_OC = (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_FETHC_OC_BITS);
//    cfg->drv_reg_status1.FETLB_OC = (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_FETLB_OC_BITS);
//    cfg->drv_reg_status1.FETHB_OC = (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_FETHB_OC_BITS);
//    cfg->drv_reg_status1.FETLA_OC = (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_FETLA_OC_BITS);
//    cfg->drv_reg_status1.OTW =      (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_FETHA_OC_BITS);
//    cfg->drv_reg_status1.OTSD =     (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_OTSD_BITS);
//    cfg->drv_reg_status1.PVDD_UV =  (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_PVDD_UV_BITS);
//    cfg->drv_reg_status1.GVDD_UV =  (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_GVDD_UV_BITS);
//    cfg->drv_reg_status1.FAULT =    (bool)(recieve_value & (uint16_t)DRV8301_STATUS1_FAULT_BITS);
//
//    // Update Status Register 1
//    Drv8301ReadReg(kDrv8301_RegNameStatus_2, &recieve_value);
//    cfg->status_reg_2_value = recieve_value;
//    cfg->drv_reg_status2.GVDD_OV = (bool)(recieve_value & (uint16_t)DRV8301_STATUS2_GVDD_OV_BITS);
//    cfg->drv_reg_status2.DeviceID = (recieve_value & DRV8301_STATUS2_ID_BITS);
//
//    // Update Control Register 1
//    Drv8301ReadReg(kDrv8301_RegNameControl_1,&recieve_value);
//    cfg->ctrl_reg_1_value = recieve_value;
//    cfg->drv_reg_ctrl1.peak_current = (Drv8301GateCurrent_e)(recieve_value & (uint16_t)DRV8301_CTRL1_GATE_CURRENT_BITS);
//    cfg->drv_reg_ctrl1.gate_rstmode = (Drv8301GateReset_e)(recieve_value & (uint16_t)DRV8301_CTRL1_GATE_RESET_BITS);
//    cfg->drv_reg_ctrl1.pwm_mode =     (Drv8301PwmMode_e)(recieve_value & (uint16_t)DRV8301_CTRL1_PWM_MODE_BITS);
//    cfg->drv_reg_ctrl1.ocp_mode =     (Drv8301OcpMode_e)(recieve_value & (uint16_t)DRV8301_CTRL1_OC_MODE_BITS);
//    cfg->drv_reg_ctrl1.oc_adj =       (Drv8301VdsLevel_e)(recieve_value & (uint16_t)DRV8301_CTRL1_OC_ADJ_SET_BITS);
//
//    // Update Control Register 2
//    Drv8301ReadReg(kDrv8301_RegNameControl_2, &recieve_value);
//    cfg->ctrl_reg_2_value = recieve_value;
//    cfg->drv_reg_ctrl2.octw_mode = (Drv8301OctwMode_e)(recieve_value & (uint16_t)DRV8301_CTRL2_OCTW_SET_BITS);
//    cfg->drv_reg_ctrl2.gain =     (Drv8301Gain_e)(recieve_value & (uint16_t)DRV8301_CTRL2_GAIN_BITS);
//    cfg->drv_reg_ctrl2.dc_cal_ch1 = (Drv8301DcCalMode_e)(recieve_value & (uint16_t)DRV8301_CTRL2_DC_CAL_1_BITS);
//    cfg->drv_reg_ctrl2.dc_cal_ch2 = (Drv8301DcCalMode_e)(recieve_value & (uint16_t)DRV8301_CTRL2_DC_CAL_2_BITS);
//    cfg->drv_reg_ctrl2.oc_toff =   (Drv8301OcToff_e)(recieve_value & (uint16_t)DRV8301_CTRL2_OC_TOFF_BITS);
//
//    return 0;
//}
//
//bool Drv8301ReadReg(Drv8301RegName_e regName, uint16_t *data)
//{
//    uint16_t data_t;
//    data_t = kDrv8301_SpiMode_Read | regName;
//    HAL_GPIO_WritePin(gDrv8301Handle.csPort, gDrv8301Handle.csPin, GPIO_PIN_RESET);
//    delay_us_nos(10);
//    HAL_SPI_Transmit(gDrv8301Handle.spiHandle, (uint8_t*)(&data_t), 1, 1000);
//    HAL_GPIO_WritePin(gDrv8301Handle.csPort, gDrv8301Handle.csPin, GPIO_PIN_SET);
//    delay_us_nos(10);
//    HAL_GPIO_WritePin(gDrv8301Handle.csPort, gDrv8301Handle.csPin, GPIO_PIN_RESET);
//    delay_us_nos(10);
//    HAL_SPI_TransmitReceive(gDrv8301Handle.spiHandle, (uint8_t*)&data_t, (uint8_t*)data, 1, 1000);
//    HAL_GPIO_WritePin(gDrv8301Handle.csPort, gDrv8301Handle.csPin, GPIO_PIN_SET);
//    delay_us_nos(10);
//    return true;
//}
//
//uint16_t data_t;
//bool Drv8301WriteReg(Drv8301RegName_e regName, uint16_t data)
//{
//    data_t = kDrv8301_SpiMode_Write | regName | data;
//
//    HAL_GPIO_WritePin(gDrv8301Handle.csPort, gDrv8301Handle.csPin, GPIO_PIN_RESET);
//    delay_us_nos(10);
//    HAL_SPI_Transmit(gDrv8301Handle.spiHandle, (uint8_t*)&data_t, 1, 1000);
//    HAL_GPIO_WritePin(gDrv8301Handle.csPort, gDrv8301Handle.csPin, GPIO_PIN_SET);
//    delay_us_nos(10);
//	return true;
//}

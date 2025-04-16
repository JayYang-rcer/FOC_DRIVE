#ifndef __DRV8301_H
#define __DRV8301_H

#include "spi.h"
#include "stdbool.h"

// **************************************************************************
// the defines

//! \brief Defines the address mask
#define DRV8301_ADDR_MASK               (0x7800)

//! \brief Defines the data mask
#define DRV8301_DATA_MASK               (0x07FF)

//! \brief Defines the R/W mask
#define DRV8301_RW_MASK                 (0x8000)

//! \brief Defines the R/W mask
#define DRV8301_FAULT_TYPE_MASK         (0x07FF)

//! \brief Defines the location of the FETLC_OC (FET Low side, Phase C Over Current) bits in the Status 1 register
#define DRV8301_STATUS1_FETLC_OC_BITS   (1 << 0)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase C Over Current) bits in the Status 1 register
#define DRV8301_STATUS1_FETHC_OC_BITS   (1 << 1)

//! \brief Defines the location of the FETLC_OC (FET Low side, Phase B Over Current) bits in the Status 1 register
#define DRV8301_STATUS1_FETLB_OC_BITS   (1 << 2)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase B Over Current) bits in the Status 1 register
#define DRV8301_STATUS1_FETHB_OC_BITS   (1 << 3)

//! \brief Defines the location of the FETLC_OC (FET Low side, Phase A Over Current) bits in the Status 1 register
#define DRV8301_STATUS1_FETLA_OC_BITS   (1 << 4)

//! \brief Defines the location of the FETLC_OC (FET High side, Phase A Over Current) bits in the Status 1 register
#define DRV8301_STATUS1_FETHA_OC_BITS   (1 << 5)

//! \brief Defines the location of the OTW (Over Temperature Warning) bits in the Status 1 register
#define DRV8301_STATUS1_OTW_BITS        (1 << 6)

//! \brief Defines the location of the OTSD (Over Temperature Shut Down) bits in the Status 1 register
#define DRV8301_STATUS1_OTSD_BITS       (1 << 7)

//! \brief Defines the location of the PVDD_UV (Power supply Vdd, Under Voltage) bits in the Status 1 register
#define DRV8301_STATUS1_PVDD_UV_BITS    (1 << 8)

//! \brief Defines the location of the GVDD_UV (DRV8301 Vdd, Under Voltage) bits in the Status 1 register
#define DRV8301_STATUS1_GVDD_UV_BITS    (1 << 9)

//! \brief Defines the location of the FAULT bits in the Status 1 register
#define DRV8301_STATUS1_FAULT_BITS      (1 << 10)

//! \brief Defines the location of the Device ID bits in the Status 2 register
#define DRV8301_STATUS2_ID_BITS        (15 << 0)

//! \brief Defines the location of the GVDD_OV (DRV8301 Vdd, Over Voltage) bits in the Status 2 register
#define DRV8301_STATUS2_GVDD_OV_BITS    (1 << 7)

//! \brief Defines the location of the GATE_CURRENT bits in the Control 1 register
#define DRV8301_CTRL1_GATE_CURRENT_BITS  (3 << 0)

//! \brief Defines the location of the GATE_RESET bits in the Control 1 register
#define DRV8301_CTRL1_GATE_RESET_BITS    (1 << 2)

//! \brief Defines the location of the PWM_MODE bits in the Control 1 register
#define DRV8301_CTRL1_PWM_MODE_BITS      (1 << 3)

//! \brief Defines the location of the OC_MODE bits in the Control 1 register
#define DRV8301_CTRL1_OC_MODE_BITS       (3 << 4)

//! \brief Defines the location of the OC_ADJ bits in the Control 1 register
#define DRV8301_CTRL1_OC_ADJ_SET_BITS   (31 << 6)

//! \brief Defines the location of the OCTW_SET bits in the Control 2 register
#define DRV8301_CTRL2_OCTW_SET_BITS      (3 << 0)

//! \brief Defines the location of the GAIN bits in the Control 2 register
#define DRV8301_CTRL2_GAIN_BITS          (3 << 2)

//! \brief Defines the location of the DC_CAL_1 bits in the Control 2 register
#define DRV8301_CTRL2_DC_CAL_1_BITS      (1 << 4)

//! \brief Defines the location of the DC_CAL_2 bits in the Control 2 register
#define DRV8301_CTRL2_DC_CAL_2_BITS      (1 << 5)

//! \brief Defines the location of the OC_TOFF bits in the Control 2 register
#define DRV8301_CTRL2_OC_TOFF_BITS       (1 << 6)


/**
 * @brief SPI mode of 8301
 */
typedef enum 
{
  kDrv8301_SpiMode_Read = 1 << 15, // 8301 read mode
  kDrv8301_SpiMode_Write = 0 << 15
}Drv8301SpiMode_e;

/**
 * @brief The Name of Register
 */
typedef enum
{
  kDrv8301_RegNameStatus_1  = 0 << 11,   //!< Status Register 1
  kDrv8301_RegNameStatus_2  = 1 << 11,   //!< Status Register 2
  kDrv8301_RegNameControl_1 = 2 << 11,  //!< Control Register 1
  kDrv8301_RegNameControl_2 = 3 << 11   //!< Control Register 2
}Drv8301RegName_e;

/**
 * @brief Status Register 1 (Address: 0x00) (All Default Values are Zero)
 */
typedef enum
{
    kDrv8301_Fault_NoFault =  (0 << 0), //No Fault
    kDrv8301_Fault_FETLC_OC = (1 << 0), //FET Low side, Phase C Over Current fault
    kDrv8301_Fault_FETHC_OC = (1 << 1), //FET High side, Phase C Over Current fault
    kDrv8301_Fault_FETLB_OC = (1 << 2), //FET Low side, Phase B Over Current fault
    kDrv8301_Fault_FETHB_OC = (1 << 3), //FET High side, Phase B Over Current fault
    kDrv8301_Fault_FETLA_OC = (1 << 4), //FET Low side, Phase A Over Current fault
    kDrv8301_Fault_FETHA_OC = (1 << 5), //FET High side, Phase A Over Current fault
    kDrv8301_Fault_OTW =      (1 << 6), //Over Temperature Warning
    kDrv8301_Fault_OTSD =     (1 << 7), //Over Temperature Shut Down
    kDrv8301_Fault_PVDD_UV =  (1 << 8), //power supply voltage VDD under voltage
    kDrv8301_Fault_GVDD_UV =  (1 << 9), //DRV8301 Vdd Under Voltage fault
    kDrv8301_Fault_FAULT =    (1 << 10),  //Fault
}Drv8301FaultType_e;


/**
 * @brief Table 11. Control Register 1 for Gate Driver Control (Address: 0x02) 
 */
typedef enum
{
    kDrv8301_PeakCurrent_1p70A = (0 << 0), //Peak Current 1.7A
    kDrv8301_PeakCurrent_0p70A = (1 << 0), //Peak Current 0.7A
    kDrv8301_PeakCurrent_0p25A = (2 << 0), //Peak Current 1.4A
}Drv8301GateCurrent_e;

typedef enum
{
    kDrv8301_GateMode_NormalMode = (0 << 2), //Normal Mode
    kDrv8301_GateMode_Reset = (1 << 2), //Reset gate driver latched faults(reverts to 0)
}Drv8301GateReset_e;

typedef enum 
{
    kDrv8301_PWMMode_Six = (0 << 3), //Six PWM inputs
    kDrv8301_PWMMode_Three = (1 << 3), //Three PWM inputs
}Drv8301PwmMode_e;

typedef enum 
{
    kDrv8301_OcpMode_CurrentLimit = (0 << 4), //Current Limit
    kDrv8301_OcpMode_LatchShutdown = (1 << 4), //Latch Shutdown
    kDrv8301_OcpMode_ReportOnly = (2 << 4), //Report Only
    kDrv8301_OcpMode_OCDisable = (3 << 4), //Over Current Disable
}Drv8301OcpMode_e;

//! \brief Enumeration for the Vds level for th over current adjustment
//!
typedef enum 
{
  kDrv8301_VdsLevel_0p060_V =  0 << 6,      //!< Vds = 0.060 V
  kDrv8301_VdsLevel_0p068_V =  1 << 6,      //!< Vds = 0.068 V
  kDrv8301_VdsLevel_0p076_V =  2 << 6,      //!< Vds = 0.076 V
  kDrv8301_VdsLevel_0p086_V =  3 << 6,      //!< Vds = 0.086 V
  kDrv8301_VdsLevel_0p097_V =  4 << 6,      //!< Vds = 0.097 V
  kDrv8301_VdsLevel_0p109_V =  5 << 6,      //!< Vds = 0.109 V
  kDrv8301_VdsLevel_0p123_V =  6 << 6,      //!< Vds = 0.123 V
  kDrv8301_VdsLevel_0p138_V =  7 << 6,      //!< Vds = 0.138 V
  kDrv8301_VdsLevel_0p155_V =  8 << 6,      //!< Vds = 0.155 V
  kDrv8301_VdsLevel_0p175_V =  9 << 6,      //!< Vds = 0.175 V
  kDrv8301_VdsLevel_0p197_V = 10 << 6,      //!< Vds = 0.197 V
  kDrv8301_VdsLevel_0p222_V = 11 << 6,      //!< Vds = 0.222 V
  kDrv8301_VdsLevel_0p250_V = 12 << 6,      //!< Vds = 0.250 V
  kDrv8301_VdsLevel_0p282_V = 13 << 6,      //!< Vds = 0.282 V
  kDrv8301_VdsLevel_0p317_V = 14 << 6,      //!< Vds = 0.317 V
  kDrv8301_VdsLevel_0p358_V = 15 << 6,      //!< Vds = 0.358 V
  kDrv8301_VdsLevel_0p403_V = 16 << 6,      //!< Vds = 0.403 V
  kDrv8301_VdsLevel_0p454_V = 17 << 6,      //!< Vds = 0.454 V
  kDrv8301_VdsLevel_0p511_V = 18 << 6,      //!< Vds = 0.511 V
  kDrv8301_VdsLevel_0p576_V = 19 << 6,      //!< Vds = 0.576 V
  kDrv8301_VdsLevel_0p648_V = 20 << 6,      //!< Vds = 0.648 V
  kDrv8301_VdsLevel_0p730_V = 21 << 6,      //!< Vds = 0.730 V
  kDrv8301_VdsLevel_0p822_V = 22 << 6,      //!< Vds = 0.822 V
  kDrv8301_VdsLevel_0p926_V = 23 << 6,      //!< Vds = 0.926 V
  kDrv8301_VdsLevel_1p043_V = 24 << 6,      //!< Vds = 1.403 V
  kDrv8301_VdsLevel_1p175_V = 25 << 6,      //!< Vds = 1.175 V
  kDrv8301_VdsLevel_1p324_V = 26 << 6,      //!< Vds = 1.324 V
  kDrv8301_VdsLevel_1p491_V = 27 << 6,      //!< Vds = 1.491 V
  kDrv8301_VdsLevel_1p679_V = 28 << 6,      //!< Vds = 1.679 V
  kDrv8301_VdsLevel_1p892_V = 29 << 6,      //!< Vds = 1.892 V
  kDrv8301_VdsLevel_2p131_V = 30 << 6,      //!< Vds = 2.131 V
  kDrv8301_VdsLevel_2p400_V = 31 << 6       //!< Vds = 2.400 V
} Drv8301VdsLevel_e;

/**
 * Table 12. Control Register 2 for Current Shunt Amplifiers and Misc Control (Address: 0x03)
 */
typedef enum 
{
    kDrv8301_OctwMode_Both = (0 << 0), //Both
    kDrv8301_OctwMode_OnlyOt = (1 << 0), //Only Over Current
    kDrv8301_OctwMode_OnlyOc = (2 << 0), //Only Over Temperature
    kDrv8301_OctwMode_None = (3 << 0), //None, OC only reserved
}Drv8301OctwMode_e;

typedef enum 
{
    kDrv8301_Gain_10 = (0 << 2), //10V/V
    kDrv8301_Gain_20 = (1 << 2), //20V/V
    kDrv8301_Gain_40 = (2 << 2), //40V/V
    kDrv8301_Gain_80 = (3 << 2), //80V/V
}Drv8301Gain_e;

typedef enum Drv8301DcCalMode_e
{
  kDrv8301_DcCalMode_Ch1_Load   = (0 << 4),   //!< Shunt amplifier 1 connected to load via input pins
  kDrv8301_DcCalMode_Ch1_NoLoad = (1 << 4),   //!< Shunt amplifier 1 disconnected from load and input pins are shorted
  kDrv8301_DcCalMode_Ch2_Load   = (0 << 5),   //!< Shunt amplifier 2 connected to load via input pins
  kDrv8301_DcCalMode_Ch2_NoLoad = (1 << 5)    //!< Shunt amplifier 2 disconnected from load and input pins are shorted
}Drv8301DcCalMode_e;

typedef enum 
{
    kDrv8301_OcToff_CycleByCycle = (0 << 6), //Shunt Amp
    kDrv8301_OcToff_OfftimeCtrl = (1 << 6), //Shunt Amp Gain
}Drv8301OcToff_e;

typedef struct Drv8301Handle_t
{
    SPI_HandleTypeDef *spiHandle;
    GPIO_TypeDef *csPort;
    uint16_t csPin;
    GPIO_TypeDef *enGatePort;
    uint16_t enGatePin;
    GPIO_TypeDef *nFaultPort;
    uint16_t nFaultPin;
}Drv8301Handle_t;

typedef struct DrvSpiStatus1_t
{
  bool                  FAULT;
  bool                  GVDD_UV;
  bool                  PVDD_UV;
  bool                  OTSD;
  bool                  OTW;
  bool                  FETHA_OC;
  bool                  FETLA_OC;
  bool                  FETHB_OC;
  bool                  FETLB_OC;
  bool                  FETHC_OC;
  bool                  FETLC_OC;
}DrvSpiStatus1_t;


typedef struct DrvSpiStatus2_t
{
  bool                  GVDD_OV;
  uint16_t              DeviceID;
}DrvSpiStatus2_t;

typedef struct DrvSpiCtrl1_t
{
    Drv8301GateCurrent_e peak_current;
    Drv8301GateReset_e gate_rstmode;
    Drv8301PwmMode_e pwm_mode;
    Drv8301OcpMode_e ocp_mode;
    Drv8301VdsLevel_e oc_adj;
}DrvSpiCtrl1_t;


typedef struct DrvSpiCtrl2_t
{
    Drv8301OctwMode_e octw_mode;
    Drv8301Gain_e gain;
    Drv8301DcCalMode_e dc_cal_ch1;
    Drv8301DcCalMode_e dc_cal_ch2;
    Drv8301OcToff_e oc_toff;
}DrvSpiCtrl2_t;


/**
 * @brief Save the SPI configuration of 8301
 */
typedef struct Drv8301SpiCfg_t
{
    DrvSpiStatus1_t drv_reg_status1;
    DrvSpiStatus2_t drv_reg_status2;
    DrvSpiCtrl1_t drv_reg_ctrl1;
    DrvSpiCtrl2_t drv_reg_ctrl2;
    uint16_t status_reg_1_value;
    uint16_t status_reg_2_value;
    uint16_t ctrl_reg_1_value;
    uint16_t ctrl_reg_2_value;
    bool send_cmd;
    bool Recv_cmd;
}Drv8301SpiCfg_t;

extern Drv8301Handle_t gDrv8301Handle;
extern Drv8301SpiCfg_t gDrv8301SpiCfg;

uint32_t DrvSendData(Drv8301SpiCfg_t *drv8301SpiCfg);
uint16_t DrvReadData(Drv8301SpiCfg_t *cfg);
bool Drv8301ReadReg(Drv8301RegName_e regName, uint16_t *data);
bool Drv8301WriteReg(Drv8301RegName_e regName, uint16_t data);
bool Drv8301Init(Drv8301Handle_t *handle);

#endif

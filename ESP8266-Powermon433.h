// ESP8266-PowerMon433, RH_CC110.h
//
// Definitions for Texas Instruments CC110L transceiver.
// http://www.ti.com/lit/ds/symlink/cc110l.pdf
// As used in Anaren CC110L Air Module BoosterPack
// https://www.anaren.com/air/cc110l-air-module-boosterpack-embedded-antenna-module-anaren
// 
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2016 Mike McCauley
// $Id: RH_CC110.h,v 1.4 2016/01/02 01:46:34 mikem Exp mikem $
// 

#ifndef RH_CC110_h
#define RH_CC110_h

// This is the maximum number of interrupts the driver can support
// Most Arduinos can handle 2, Megas can handle more
#define RH_CC110_NUM_INTERRUPTS 3

// Max number of octets the FIFO can hold
#define RH_CC110_FIFO_SIZE 64

// This is the maximum number of bytes that can be carried by the chip
// We use some for headers, keeping fewer for RadioHead messages
#define RH_CC110_MAX_PAYLOAD_LEN RH_CC110_FIFO_SIZE

// The length of the headers we add.
// The headers are inside the chip payload
#define RH_CC110_HEADER_LEN 4

// This is the maximum message length that can be supported by this driver. 
// Can be pre-defined to a smaller size (to save SRAM) prior to including this header
// Here we allow for 1 byte message length, 4 bytes headers, user data 
#ifndef RH_CC110_MAX_MESSAGE_LEN
 #define RH_CC110_MAX_MESSAGE_LEN (RH_CC110_MAX_PAYLOAD_LEN - RH_CC110_HEADER_LEN - 1)
#endif

#define RH_CC110_SPI_READ_MASK  0x80
#define RH_CC110_SPI_BURST_MASK 0x40

// Register definitions from Table 5-22
#define RH_CC110_REG_00_IOCFG2                 0x00
#define RH_CC110_REG_01_IOCFG1                 0x01
#define RH_CC110_REG_02_IOCFG0                 0x02
#define RH_CC110_REG_03_FIFOTHR                0x03
#define RH_CC110_REG_04_SYNC1                  0x04
#define RH_CC110_REG_05_SYNC0                  0x05
#define RH_CC110_REG_06_PKTLEN                 0x06
#define RH_CC110_REG_07_PKTCTRL1               0x07
#define RH_CC110_REG_08_PKTCTRL0               0x08
#define RH_CC110_REG_09_ADDR                   0x09
#define RH_CC110_REG_0A_CHANNR                 0x0a
#define RH_CC110_REG_0B_FSCTRL1                0x0b
#define RH_CC110_REG_0C_FSCTRL0                0x0c
#define RH_CC110_REG_0D_FREQ2                  0x0d
#define RH_CC110_REG_0E_FREQ1                  0x0e
#define RH_CC110_REG_0F_FREQ0                  0x0f
#define RH_CC110_REG_10_MDMCFG4                0x10
#define RH_CC110_REG_11_MDMCFG3                0x11
#define RH_CC110_REG_12_MDMCFG2                0x12
#define RH_CC110_REG_13_MDMCFG1                0x13
#define RH_CC110_REG_14_MDMCFG0                0x14
#define RH_CC110_REG_15_DEVIATN                0x15
#define RH_CC110_REG_16_MCSM2                  0x16
#define RH_CC110_REG_17_MCSM1                  0x17
#define RH_CC110_REG_18_MCSM0                  0x18
#define RH_CC110_REG_19_FOCCFG                 0x19
#define RH_CC110_REG_1A_BSCFG                  0x1a
#define RH_CC110_REG_1B_AGCCTRL2               0x1b
#define RH_CC110_REG_1C_AGCCTRL1               0x1c
#define RH_CC110_REG_1D_AGCCTRL0               0x1d
#define RH_CC110_REG_1E_WOREVT1                0x1e
#define RH_CC110_REG_1F_WOREVT0                0x1f
#define RH_CC110_REG_20_WORCTRL                0x20
#define RH_CC110_REG_21_FREND1                 0x21
#define RH_CC110_REG_22_FREND0                 0x22
#define RH_CC110_REG_23_FSCAL3                 0x23
#define RH_CC110_REG_24_FSCAL2                 0x24
#define RH_CC110_REG_25_FSCAL1                 0x25
#define RH_CC110_REG_26_FSCAL0                 0x26
#define RH_CC110_REG_27_RCCTRL1                0x28
#define RH_CC110_REG_28_RCCTRL0                0x29
#define RH_CC110_REG_29_FSTEST                 0x2a
#define RH_CC110_REG_2A_PTEST                  0x2b
#define RH_CC110_REG_2B_AGCTEST                0x2c
#define RH_CC110_REG_2C_TEST2                  0x2c
#define RH_CC110_REG_2D_TEST1                  0x2d
#define RH_CC110_REG_2E_TEST0                  0x2e

// Single byte read and write version of registers 0x30 to 0x3f. Strobes
// use spiCommand()
#define RH_CC110_STROBE_30_SRES                0x30
#define RH_CC110_STROBE_31_SFSTXON             0x31
#define RH_CC110_STROBE_32_SXOFF               0x32
#define RH_CC110_STROBE_33_SCAL                0x33
#define RH_CC110_STROBE_34_SRX                 0x34
#define RH_CC110_STROBE_35_STX                 0x35
#define RH_CC110_STROBE_36_SIDLE               0x36

#define RH_CC110_STROBE_39_SPWD                0x39
#define RH_CC110_STROBE_3A_SFRX                0x3a
#define RH_CC110_STROBE_3B_SFTX                0x3b

#define RH_CC110_STROBE_3D_SNOP                0x3d


// Burst read from these registers gives more data:
// use spiBurstReadRegister()
#define RH_CC110_REG_30_PARTNUM                0x30
#define RH_CC110_REG_31_VERSION                0x31
#define RH_CC110_REG_32_FREQEST                0x32
#define RH_CC110_REG_33_CRC_REG                0x33
#define RH_CC110_REG_34_RSSI                   0x34
#define RH_CC110_REG_35_MARCSTATE              0x35

#define RH_CC110_REG_38_PKTSTATUS              0x38

#define RH_CC110_REG_3A_TXBYTES                0x3a
#define RH_CC110_REG_3B_RXBYTES                0x3b

// PATABLE, TXFIFO, RXFIFO also support burst
#define RH_CC110_REG_3E_PATABLE                0x3e
#define RH_CC110_REG_3F_FIFO                   0x3f

// Status Byte
#define RH_CC110_STATUS_CHIP_RDY            0x80
#define RH_CC110_STATUS_STATE               0x70
#define RH_CC110_STATUS_IDLE                0x00
#define RH_CC110_STATUS_RX                  0x10
#define RH_CC110_STATUS_TX                  0x20
#define RH_CC110_STATUS_FSTXON              0x30
#define RH_CC110_STATUS_CALIBRATE           0x40
#define RH_CC110_STATUS_SETTLING            0x50
#define RH_CC110_STATUS_RXFIFO_OVERFLOW     0x60
#define RH_CC110_STATUS_TXFIFO_UNDERFLOW    0x70
#define RH_CC110_STATUS_FIFOBYTES_AVAILABLE 0x0f

// Register contents
// Chip Status Byte, read from header, data or command strobe
#define RH_CC110_CHIP_RDY                   0x80
#define RH_CC110_STATE                      0x70
#define RH_CC110_FIFO_BYTES_AVAILABLE       0x0f

// Register bit field definitions
// #define RH_CC110_REG_00_IOCFG2                 0x00
// #define RH_CC110_REG_01_IOCFG1                 0x01
// #define RH_CC110_REG_02_IOCFG0                 0x02
#define RH_CC110_GDO_CFG_RX_FIFO_THR              0x00
#define RH_CC110_GDO_CFG_RX_FIFO_FULL             0x01
#define RH_CC110_GDO_CFG_TX_FIFO_THR              0x02
#define RH_CC110_GDO_CFG_TX_FIFO_EMPTY            0x03
#define RH_CC110_GDO_CFG_RX_FIFO_OVERFLOW         0x04
#define RH_CC110_GDO_CFG_TX_FIFO_UNDEFLOOW        0x05
#define RH_CC110_GDO_CFG_SYNC                     0x06
#define RH_CC110_GDO_CFG_CRC_OK_AUTORESET         0x07
#define RH_CC110_GDO_CFG_CCA                      0x09
#define RH_CC110_GDO_CFG_LOCK_DETECT              0x0a
#define RH_CC110_GDO_CFG_SERIAL_CLOCK             0x0b
#define RH_CC110_GDO_CFG_SYNCHRONOUS_SDO          0x0c
#define RH_CC110_GDO_CFG_SDO                      0x0d
#define RH_CC110_GDO_CFG_CARRIER                  0x0e
#define RH_CC110_GDO_CFG_CRC_OK                   0x0f
#define RH_CC110_GDO_CFG_PA_PD                    0x1b
#define RH_CC110_GDO_CFG_LNA_PD                   0x1c
#define RH_CC110_GDO_CFG_CLK_32K                  0x27
#define RH_CC110_GDO_CFG_CHIP_RDYN                0x29
#define RH_CC110_GDO_CFG_XOSC_STABLE              0x2b
#define RH_CC110_GDO_CFG_HIGH_IMPEDANCE           0x2e
#define RH_CC110_GDO_CFG_0                        0x2f
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_1           0x30
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_1_5         0x31
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_2           0x32
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_3           0x33
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_4           0x34
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_6           0x35
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_8           0x36
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_12          0x37
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_16          0x38
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_24          0x39
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_32          0x3a
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_48          0x3b
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_64          0x3c
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_96          0x3d
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_128         0x3e
#define RH_CC110_GDO_CFG_CLK_XOSC_DIV_192         0x3f

// #define RH_CC110_REG_03_FIFOTHR                0x03
#define RH_CC110_ADC_RETENTION                    0x80

#define RH_CC110_CLOSE_IN_RX                      0x30
#define RH_CC110_CLOSE_IN_RX_0DB                  0x00
#define RH_CC110_CLOSE_IN_RX_6DB                  0x10
#define RH_CC110_CLOSE_IN_RX_12DB                 0x20
#define RH_CC110_CLOSE_IN_RX_18DB                 0x30

#define RH_CC110_FIFO_THR                         0x0f

// #define RH_CC110_REG_04_SYNC1                  0x04
// #define RH_CC110_REG_05_SYNC0                  0x05
// #define RH_CC110_REG_06_PKTLEN                 0x06
// #define RH_CC110_REG_07_PKTCTRL1               0x07
#define RH_CC110_CRC_AUTOFLUSH                    0x08
#define RH_CC110_APPEND_STATUS                    0x04
#define RH_CC110_ADDR_CHK                         0x03
// can or the next 2:
#define RH_CC110_ADDR_CHK_ADDRESS                 0x01
#define RH_CC110_ADDR_CHK_BROADCAST               0x02


// #define RH_CC110_REG_08_PKTCTRL0               0x08
#define RH_CC110_PKT_FORMAT                       0x30
#define RH_CC110_PKT_FORMAT_NORMAL                0x00
#define RH_CC110_PKT_FORMAT_SYNC_SERIAL           0x10
#define RH_CC110_PKT_FORMAT_RANDOM_TX             0x20
#define RH_CC110_PKT_FORMAT_ASYNC_SERIAL          0x30

#define RH_CC110_CRC_EN                           0x04

#define RH_CC110_LENGTH_CONFIG                    0x03
#define RH_CC110_LENGTH_CONFIG_FIXED              0x00
#define RH_CC110_LENGTH_CONFIG_VARIABLE           0x01
#define RH_CC110_LENGTH_CONFIG_INFINITE           0x02

// #define RH_CC110_REG_09_ADDR                   0x09
// #define RH_CC110_REG_0A_CHANNR                 0x0a
// #define RH_CC110_REG_0B_FSCTRL1                0x0b
// #define RH_CC110_REG_0C_FSCTRL0                0x0c
// #define RH_CC110_REG_0D_FREQ2                  0x0d
// #define RH_CC110_REG_0E_FREQ1                  0x0e
// #define RH_CC110_REG_0F_FREQ0                  0x0f
// #define RH_CC110_REG_10_MDMCFG4                0x10
#define RH_CC110_CHANBW_E                         0xc0
#define RH_CC110_CHANBW_M                         0x30
#define RH_CC110_DRATE_E                          0x0f

// #define RH_CC110_REG_11_MDMCFG3                0x11
// #define RH_CC110_REG_12_MDMCFG2                0x12
#define RH_CC110_DEM_DCFILT_OFF                   0x80
#define RH_CC110_MOD_FORMAT                       0x70
#define RH_CC110_MOD_FORMAT_2FSK                  0x00
#define RH_CC110_MOD_FORMAT_GFSK                  0x10
#define RH_CC110_MOD_FORMAT_OOK                   0x30
#define RH_CC110_MOD_FORMAT_4FSK                  0x40
#define RH_CC110_MANCHESTER_EN                    0x08
#define RH_CC110_SYNC_MODE                        0x07
#define RH_CC110_SYNC_MODE_NONE                   0x00
#define RH_CC110_SYNC_MODE_15_16                  0x01
#define RH_CC110_SYNC_MODE_16_16                  0x02
#define RH_CC110_SYNC_MODE_30_32                  0x03
#define RH_CC110_SYNC_MODE_NONE_CARRIER           0x04
#define RH_CC110_SYNC_MODE_15_16_CARRIER          0x05
#define RH_CC110_SYNC_MODE_16_16_CARRIER          0x06
#define RH_CC110_SYNC_MODE_30_32_CARRIER          0x07

// #define RH_CC110_REG_13_MDMCFG1                0x13
#define RH_CC110_NUM_PREAMBLE                     0x70
#define RH_CC110_NUM_PREAMBLE_2                   0x00
#define RH_CC110_NUM_PREAMBLE_3                   0x10
#define RH_CC110_NUM_PREAMBLE_4                   0x20
#define RH_CC110_NUM_PREAMBLE_6                   0x30
#define RH_CC110_NUM_PREAMBLE_8                   0x40
#define RH_CC110_NUM_PREAMBLE_12                  0x50
#define RH_CC110_NUM_PREAMBLE_16                  0x60
#define RH_CC110_NUM_PREAMBLE_24                  0x70

#define RH_CC110_CHANSPC_E                        0x03

// #define RH_CC110_REG_14_MDMCFG0                0x14
// #define RH_CC110_REG_15_DEVIATN                0x15
#define RH_CC110_DEVIATION_E                      0x70
#define RH_CC110_DEVIATION_M                      0x07

// #define RH_CC110_REG_16_MCSM2                  0x16
#define RH_CC110_RX_TIME_RSSI                     0x10

// #define RH_CC110_REG_17_MCSM1                  0x17
#define RH_CC110_CCA_MODE                         0x30
#define RH_CC110_CCA_MODE_ALWAYS                  0x00
#define RH_CC110_CCA_MODE_RSSI                    0x10
#define RH_CC110_CCA_MODE_PACKET                  0x20
#define RH_CC110_CCA_MODE_RSSI_PACKET             0x30
#define RH_CC110_RXOFF_MODE                       0x0c
#define RH_CC110_RXOFF_MODE_IDLE                  0x00
#define RH_CC110_RXOFF_MODE_FSTXON                0x04
#define RH_CC110_RXOFF_MODE_TX                    0x08
#define RH_CC110_RXOFF_MODE_RX                    0x0c
#define RH_CC110_TXOFF_MODE                       0x03
#define RH_CC110_TXOFF_MODE_IDLE                  0x00
#define RH_CC110_TXOFF_MODE_FSTXON                0x01
#define RH_CC110_TXOFF_MODE_TX                    0x02
#define RH_CC110_TXOFF_MODE_RX                    0x03

// #define RH_CC110_REG_18_MCSM0                  0x18
#define RH_CC110_FS_AUTOCAL                       0x30
#define RH_CC110_FS_AUTOCAL_NEVER                 0x00
#define RH_CC110_FS_AUTOCAL_FROM_IDLE             0x10
#define RH_CC110_FS_AUTOCAL_TO_IDLE               0x20
#define RH_CC110_FS_AUTOCAL_TO_IDLE_4             0x30
#define RH_CC110_PO_TIMEOUT                       0x0c
#define RH_CC110_PO_TIMEOUT_1                     0x00
#define RH_CC110_PO_TIMEOUT_16                    0x04
#define RH_CC110_PO_TIMEOUT_64                    0x08
#define RH_CC110_PO_TIMEOUT_256                   0x0c
#define RH_CC110_XOSC_FORCE_ON                    0x01

// #define RH_CC110_REG_19_FOCCFG                 0x19
#define RH_CC110_FOC_BS_CS_GATE                   0x20
#define RH_CC110_FOC_PRE_K                        0x18
#define RH_CC110_FOC_PRE_K_0                      0x00
#define RH_CC110_FOC_PRE_K_1                      0x08
#define RH_CC110_FOC_PRE_K_2                      0x10
#define RH_CC110_FOC_PRE_K_3                      0x18
#define RH_CC110_FOC_POST_K                       0x04
#define RH_CC110_FOC_LIMIT                        0x03
#define RH_CC110_FOC_LIMIT_0                      0x00
#define RH_CC110_FOC_LIMIT_8                      0x01
#define RH_CC110_FOC_LIMIT_4                      0x02
#define RH_CC110_FOC_LIMIT_2                      0x03

// #define RH_CC110_REG_1A_BSCFG                  0x1a
#define RH_CC110_BS_PRE_K                         0xc0
#define RH_CC110_BS_PRE_K_1                       0x00
#define RH_CC110_BS_PRE_K_2                       0x40
#define RH_CC110_BS_PRE_K_3                       0x80
#define RH_CC110_BS_PRE_K_4                       0xc0
#define RH_CC110_BS_PRE_KP                        0x30
#define RH_CC110_BS_PRE_KP_1                      0x00
#define RH_CC110_BS_PRE_KP_2                      0x10
#define RH_CC110_BS_PRE_KP_3                      0x20
#define RH_CC110_BS_PRE_KP_4                      0x30
#define RH_CC110_BS_POST_KI                       0x08
#define RH_CC110_BS_POST_KP                       0x04
#define RH_CC110_BS_LIMIT                         0x03
#define RH_CC110_BS_LIMIT_0                       0x00
#define RH_CC110_BS_LIMIT_3                       0x01
#define RH_CC110_BS_LIMIT_6                       0x02
#define RH_CC110_BS_LIMIT_12                      0x03

// #define RH_CC110_REG_1B_AGCCTRL2               0x1b
#define RH_CC110_MAX_DVA_GAIN                     0xc0
#define RH_CC110_MAX_DVA_GAIN_ALL                 0x00
#define RH_CC110_MAX_DVA_GAIN_ALL_LESS_1          0x40
#define RH_CC110_MAX_DVA_GAIN_ALL_LESS_2          0x80
#define RH_CC110_MAX_DVA_GAIN_ALL_LESS_3          0xc0
#define RH_CC110_MAX_LNA_GAIN                     0x38

#define RH_CC110_MAGN_TARGET                      0x07
#define RH_CC110_MAGN_TARGET_24DB                 0x00
#define RH_CC110_MAGN_TARGET_27DB                 0x01
#define RH_CC110_MAGN_TARGET_30DB                 0x02
#define RH_CC110_MAGN_TARGET_33DB                 0x03
#define RH_CC110_MAGN_TARGET_36DB                 0x04
#define RH_CC110_MAGN_TARGET_38DB                 0x05
#define RH_CC110_MAGN_TARGET_40DB                 0x06
#define RH_CC110_MAGN_TARGET_42DB                 0x07

// #define RH_CC110_REG_1C_AGCCTRL1               0x1c
#define RH_CC110_AGC_LNA_PRIORITY                 0x40
#define RH_CC110_CARRIER_SENSE_REL_THR            0x30
#define RH_CC110_CARRIER_SENSE_REL_THR_0DB        0x00
#define RH_CC110_CARRIER_SENSE_REL_THR_6DB        0x10
#define RH_CC110_CARRIER_SENSE_REL_THR_10DB       0x20
#define RH_CC110_CARRIER_SENSE_REL_THR_14DB       0x30
#define RH_CC110_CARRIER_SENSE_ABS_THR            0x0f

// #define RH_CC110_REG_1D_AGCCTRL0               0x1d
#define RH_CC110_HYST_LEVEL                       0xc0
#define RH_CC110_HYST_LEVEL_NONE                  0x00
#define RH_CC110_HYST_LEVEL_LOW                   0x40
#define RH_CC110_HYST_LEVEL_MEDIUM                0x80
#define RH_CC110_HYST_LEVEL_HIGH                  0xc0
#define RH_CC110_WAIT_TIME                        0x30
#define RH_CC110_WAIT_TIME_8                      0x00
#define RH_CC110_WAIT_TIME_16                     0x10
#define RH_CC110_WAIT_TIME_24                     0x20
#define RH_CC110_WAIT_TIME_32                     0x30
#define RH_CC110_AGC_FREEZE                       0x0c
#define RH_CC110_AGC_FILTER_LENGTH                0x03
#define RH_CC110_AGC_FILTER_LENGTH_8              0x00
#define RH_CC110_AGC_FILTER_LENGTH_16             0x01
#define RH_CC110_AGC_FILTER_LENGTH_32             0x02
#define RH_CC110_AGC_FILTER_LENGTH_64             0x03

// #define RH_CC110_REG_1E_WOREVT1                0x1e
// #define RH_CC110_REG_1F_WOREVT0                0x1f
// #define RH_CC110_REG_20_WORCTRL                0x20
// #define RH_CC110_REG_21_FREND1                 0x21
#define RH_CC110_LNA_CURRENT                      0xc0
#define RH_CC110_LNA2MIX_CURRENT                  0x30
#define RH_CC110_LODIV_BUF_CURRENT_RX             0x0c
#define RH_CC110_MIX_CURRENT                      0x03

// #define RH_CC110_REG_22_FREND0                 0x22
#define RH_CC110_LODIV_BUF_CURRENT_TX             0x30
#define RH_CC110_PA_POWER                         0x07

// #define RH_CC110_REG_23_FSCAL3                 0x23
#define RH_CC110_FSCAL3_7_6                       0xc0
#define RH_CC110_CHP_CURR_CAL_EN                  0x30
#define RH_CC110_FSCAL3_3_0                       0x0f

// #define RH_CC110_REG_24_FSCAL2                 0x24
#define RH_CC110_VCO_CORE_H_EN                    0x20
#define RH_CC110_FSCAL2                           0x1f

// #define RH_CC110_REG_25_FSCAL1                 0x25
#define RH_CC110_FSCAL1                           0x3f

// #define RH_CC110_REG_26_FSCAL0                 0x26
#define RH_CC110_FSCAL0                           0x7f

// #define RH_CC110_REG_27_RCCTRL1                0x28
// #define RH_CC110_REG_28_RCCTRL0                0x29
// #define RH_CC110_REG_29_FSTEST                 0x2a
// #define RH_CC110_REG_2A_PTEST                  0x2b
// #define RH_CC110_REG_2B_AGCTEST                0x2c
// #define RH_CC110_REG_2C_TEST2                  0x2c
// #define RH_CC110_REG_2D_TEST1                  0x2d
// #define RH_CC110_REG_2E_TEST0                  0x2e
#define RH_CC110_TEST0_7_2                        0xfc
#define RH_CC110_VCO_SEL_CAL_EN                   0x02
#define RH_CC110_TEST0_0                          0x01

// #define RH_CC110_REG_30_PARTNUM                0x30
// #define RH_CC110_REG_31_VERSION                0x31
// #define RH_CC110_REG_32_FREQEST                0x32
// #define RH_CC110_REG_33_CRC_REG                0x33
#define RH_CC110_CRC_REG_CRC_OK                   0x80

// #define RH_CC110_REG_34_RSSI                   0x34
// #define RH_CC110_REG_35_MARCSTATE              0x35
#define RH_CC110_MARC_STATE                       0x1f
#define RH_CC110_MARC_STATE_SLEEP                 0x00
#define RH_CC110_MARC_STATE_IDLE                  0x01
#define RH_CC110_MARC_STATE_XOFF                  0x02
#define RH_CC110_MARC_STATE_VCOON_MC              0x03
#define RH_CC110_MARC_STATE_REGON_MC              0x04
#define RH_CC110_MARC_STATE_MANCAL                0x05
#define RH_CC110_MARC_STATE_VCOON                 0x06
#define RH_CC110_MARC_STATE_REGON                 0x07
#define RH_CC110_MARC_STATE_STARTCAL              0x08
#define RH_CC110_MARC_STATE_BWBOOST               0x09
#define RH_CC110_MARC_STATE_FS_LOCK               0x0a
#define RH_CC110_MARC_STATE_IFADCON               0x0b
#define RH_CC110_MARC_STATE_ENDCAL                0x0c
#define RH_CC110_MARC_STATE_RX                    0x0d
#define RH_CC110_MARC_STATE_RX_END                0x0e
#define RH_CC110_MARC_STATE_RX_RST                0x0f
#define RH_CC110_MARC_STATE_TXRX_SWITCH           0x10
#define RH_CC110_MARC_STATE_RXFIFO_OVERFLOW       0x11
#define RH_CC110_MARC_STATE_FSTXON                0x12
#define RH_CC110_MARC_STATE_TX                    0x13
#define RH_CC110_MARC_STATE_TX_END                0x14
#define RH_CC110_MARC_STATE_RXTX_SWITCH           0x15
#define RH_CC110_MARC_STATE_TXFIFO_UNDERFLOW      0x16

// #define RH_CC110_REG_38_PKTSTATUS              0x38
#define RH_CC110_PKTSTATUS_CRC_OK                 0x80
#define RH_CC110_PKTSTATUS_CS                     0x40
#define RH_CC110_PKTSTATUS_CCA                    0x10
#define RH_CC110_PKTSTATUS_SFD                    0x08
#define RH_CC110_PKTSTATUS_GDO2                   0x04
#define RH_CC110_PKTSTATUS_GDO0                   0x01

// #define RH_CC110_REG_3A_TXBYTES                0x3a
#define RH_CC110_TXFIFO_UNDERFLOW                 0x80
#define RH_CC110_NUM_TXBYTES                      0x7f

// #define RH_CC110_REG_3B_RXBYTES                0x3b
#define RH_CC110_RXFIFO_UNDERFLOW                 0x80
#define RH_CC110_NUM_RXBYTES                      0x7f

/////////////////////////////////////////////////////////////////////
/// \class RH_CC110 RH_CC110.h <RH_CC110.h>
/// \brief Send and receive addressed, reliable, acknowledged datagrams by Texas Instruments CC110L and compatible transceivers and modules.
///
/// The TI CC110L is a low cost tranceiver chip capable of 300 to 928MHz and with a wide range of modulation types and speeds.
/// The chip is typically provided on a module that also includes the antenna and coupling hardware
/// and is therefore capable of a more restricted frequency range.
///
/// Supported modules include:
/// - Anaren AIR BoosterPack 430BOOST-CC110L 
///
/// This base class provides basic functions for sending and receiving unaddressed, unreliable datagrams
/// of arbitrary length to 59 octets per packet at a selected data rate and modulation type. 
/// Use one of the Manager classes to get addressing and 
/// acknowledgement reliability, routing, meshes etc.
///
/// Naturally, for any 2 radios to communicate that must be configured to use the same frequency and 
/// data rate, and with identical network addresses.
///
/// Several CC110L modules can be connected to an Arduino, permitting the construction of translators
/// and frequency changers, etc.
///
/// Several GFSK modulation schemes are provided and may be selected by calling setModemConfig(). No FSK or OOK 
/// modulation schemes are provided though the implementor may configure the mnodem characteristics directly 
/// by calling setModemRegisters().
///
/// Implementation based on:
/// http://www.ti.com/lit/ds/symlink/cc110l.pdf
/// and
/// https://www.anaren.com/air/cc110l-air-module-boosterpack-embedded-antenna-module-anaren
///
/// \par Crystal Frequency
///
/// Modules based on the CC110L may contain a crystal oscillator with one of 2 possible frequencies: 26MHz or 27MHz.
/// A number of radio configuration parameters (including carrier frequency and data rates) depend on the
/// crystal oscillator frequency. The chip has no knowledge of the frequency, so it is up to the implementer
/// to tell the driver the oscillator frequency by passing in the appropriate value of is27MHz to the constructor (default 26MHz)
/// or by calling setIs27MHz() before calling init().
/// Failure to correctly set this flag will cause incorrect frequency and modulation
/// characteristics to be used. 
///
/// Caution: it is not easy to determine what the actual crystal frequency is on some modules. For example, 
/// the documentation for the Anaren BoosterPack indictes a 26MHz crystal, but measurements on the devices delivered here
/// indicate a 27MHz crystal is actually installed. TI recommend 27MHz for 
///
/// \par Packet Format
///
/// - 2 octets sync (a configurable network address)
/// - 1 octet message length
/// - 4 to 63 octets of payload consisting of:
///   - 1 octet TO header
///   - 1 octet FROM header
///   - 1 octet ID header
///   - 1 octet FLAGS header
///   - 0 to 59 octets of user message
/// - 2 octets CRC 
///
/// \par Connecting CC110L to Arduino
/// 
/// Warning: the CC110L is a 3.3V part, and exposing it to 5V on any pin will damage it. Ensure you are using a 3.3V 
/// MCU or use level shifters. We tested with Teensy 3.1.
///
/// The electrical connection between a CC110L module and the Arduino or other processor
/// require 3.3V, the 3 x SPI pins (SCK, SDI, SDO), 
/// a Chip Select pin and an Interrupt pin.
/// Examples below assume the Anaren BoosterPack. Caution: the pin numbering on the Anaren BoosterPack
/// is a bit counter-intuitive: the direction of number on J1 is the reverse of J2. Check the pin numbers
/// stencilied on the front of the board to be sure.
///
/// \code
///                 Teensy 3.1   CC110L pin name         Anaren BoosterPack pin
///                 3.3V---------VDD   (3.3V to 7V in)        J1-1
///          SS pin D10----------CSn   (chip select in)       J2-8
///         SCK pin D13----------SCLK  (SPI clock in)         J1-7
///        MOSI pin D11----------MOSI  (SPI Data in)          J2-5
///        MISO pin D12----------MISO  (SPI data out)         J2-4
///                 D2-----------GDO0  (Interrupt output)     J2-9
///                 GND----------GND   (ground in)            J2-10
/// \endcode
/// and use the default RH_CC110 constructor. You can use other pins by passing the appropriate arguments
/// to the RH_CC110 constructor, depending on what your MCU supports.
///
/// \par Example programs
///
/// Several example programs are provided.
///
/// \par Radio operating strategy and defaults
///
/// The radio is enabled at all times and switched between RX, TX and IDLE modes.
/// When RX is enabled (by calling available() or setModeRx()) the radio will stay in RX mode until a 
/// valid CRC correct message addressed to thiis node is received, when it will transition to IDLE.
/// When TX is enabled (by calling send()) it will stay in TX mode until the message has ben sent
/// and waitPacketSent() is called when it wil transition to IDLE 
///(this radio has no 'packet sent' interrupt that could be used, so polling
/// with waitPacketSent() is required
///
/// The modulation schemes supported include the GFSK schemes provided by default in the TI SmartRF Suite.
/// This software allows you to get the correct register values for diferent modulation schemes. All the modulation
/// schemes prvided in the driver are based on the recommended register values given by SmartRF.
/// Other schemes such a 2-FSK, 4-FSK and OOK are suported by the chip, but canned configurations are not provided with this driver. 
/// The implementer may choose to create their own modem configurations and pass them to setModemRegisters().
///
#endif

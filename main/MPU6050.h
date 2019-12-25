// I2Cdev library collection - MPU6050 I2C device class
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// 11/28/2014 by Marton Sebok <sebokmarton@gmail.com>
//
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ...        - ongoing debug release
//     2014-11-28 - ported to PIC18 peripheral library from Arduino code
//     2017-03-11 - tested basic functions on STM32

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Copyright (c) 2014 Marton Sebok

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/*
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/

#ifndef _MPU6050_H_
  #define _MPU6050_H_
  #ifdef __cplusplus
    extern "C"
    {
  #endif
      /* Includes ------------------------------------------------------------------*/
      #include "tracking_main.h"

      #define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
      #define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
      #define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

      #define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
      #define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
      #define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
      #define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
      #define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
      #define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
      #define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
      #define MPU6050_RA_XA_OFFS_L_TC     0x07
      #define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
      #define MPU6050_RA_YA_OFFS_L_TC     0x09
      #define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
      #define MPU6050_RA_ZA_OFFS_L_TC     0x0B
      #define MPU6050_RA_SELF_TEST_X      0x0D //[7:5] XA_TEST[4-2], [4:0] XG_TEST[4-0]
      #define MPU6050_RA_SELF_TEST_Y      0x0E //[7:5] YA_TEST[4-2], [4:0] YG_TEST[4-0]
      #define MPU6050_RA_SELF_TEST_Z      0x0F //[7:5] ZA_TEST[4-2], [4:0] ZG_TEST[4-0]
      #define MPU6050_RA_SELF_TEST_A      0x10 //[5:4] XA_TEST[1-0], [3:2] YA_TEST[1-0], [1:0] ZA_TEST[1-0]
      #define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
      #define MPU6050_RA_XG_OFFS_USRL     0x14
      #define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
      #define MPU6050_RA_YG_OFFS_USRL     0x16
      #define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
      #define MPU6050_RA_ZG_OFFS_USRL     0x18
      #define MPU6050_RA_SMPLRT_DIV       0x19
      #define MPU6050_RA_CONFIG           0x1A
      #define MPU6050_RA_GYRO_CONFIG      0x1B
      #define MPU6050_RA_ACCEL_CONFIG     0x1C
      #define MPU6050_RA_FF_THR           0x1D
      #define MPU6050_RA_FF_DUR           0x1E
      #define MPU6050_RA_MOT_THR          0x1F
      #define MPU6050_RA_MOT_DUR          0x20
      #define MPU6050_RA_ZRMOT_THR        0x21
      #define MPU6050_RA_ZRMOT_DUR        0x22
      #define MPU6050_RA_FIFO_EN          0x23
      #define MPU6050_RA_I2C_MST_CTRL     0x24
      #define MPU6050_RA_I2C_SLV0_ADDR    0x25
      #define MPU6050_RA_I2C_SLV0_REG     0x26
      #define MPU6050_RA_I2C_SLV0_CTRL    0x27
      #define MPU6050_RA_I2C_SLV1_ADDR    0x28
      #define MPU6050_RA_I2C_SLV1_REG     0x29
      #define MPU6050_RA_I2C_SLV1_CTRL    0x2A
      #define MPU6050_RA_I2C_SLV2_ADDR    0x2B
      #define MPU6050_RA_I2C_SLV2_REG     0x2C
      #define MPU6050_RA_I2C_SLV2_CTRL    0x2D
      #define MPU6050_RA_I2C_SLV3_ADDR    0x2E
      #define MPU6050_RA_I2C_SLV3_REG     0x2F
      #define MPU6050_RA_I2C_SLV3_CTRL    0x30
      #define MPU6050_RA_I2C_SLV4_ADDR    0x31
      #define MPU6050_RA_I2C_SLV4_REG     0x32
      #define MPU6050_RA_I2C_SLV4_DO      0x33
      #define MPU6050_RA_I2C_SLV4_CTRL    0x34
      #define MPU6050_RA_I2C_SLV4_DI      0x35
      #define MPU6050_RA_I2C_MST_STATUS   0x36
      #define MPU6050_RA_INT_PIN_CFG      0x37
      #define MPU6050_RA_INT_ENABLE       0x38
      #define MPU6050_RA_DMP_INT_STATUS   0x39
      #define MPU6050_RA_INT_STATUS       0x3A
      #define MPU6050_RA_ACCEL_XOUT_H     0x3B
      #define MPU6050_RA_ACCEL_XOUT_L     0x3C
      #define MPU6050_RA_ACCEL_YOUT_H     0x3D
      #define MPU6050_RA_ACCEL_YOUT_L     0x3E
      #define MPU6050_RA_ACCEL_ZOUT_H     0x3F
      #define MPU6050_RA_ACCEL_ZOUT_L     0x40
      #define MPU6050_RA_TEMP_OUT_H       0x41
      #define MPU6050_RA_TEMP_OUT_L       0x42
      #define MPU6050_RA_GYRO_XOUT_H      0x43
      #define MPU6050_RA_GYRO_XOUT_L      0x44
      #define MPU6050_RA_GYRO_YOUT_H      0x45
      #define MPU6050_RA_GYRO_YOUT_L      0x46
      #define MPU6050_RA_GYRO_ZOUT_H      0x47
      #define MPU6050_RA_GYRO_ZOUT_L      0x48
      #define MPU6050_RA_EXT_SENS_DATA_00 0x49
      #define MPU6050_RA_EXT_SENS_DATA_01 0x4A
      #define MPU6050_RA_EXT_SENS_DATA_02 0x4B
      #define MPU6050_RA_EXT_SENS_DATA_03 0x4C
      #define MPU6050_RA_EXT_SENS_DATA_04 0x4D
      #define MPU6050_RA_EXT_SENS_DATA_05 0x4E
      #define MPU6050_RA_EXT_SENS_DATA_06 0x4F
      #define MPU6050_RA_EXT_SENS_DATA_07 0x50
      #define MPU6050_RA_EXT_SENS_DATA_08 0x51
      #define MPU6050_RA_EXT_SENS_DATA_09 0x52
      #define MPU6050_RA_EXT_SENS_DATA_10 0x53
      #define MPU6050_RA_EXT_SENS_DATA_11 0x54
      #define MPU6050_RA_EXT_SENS_DATA_12 0x55
      #define MPU6050_RA_EXT_SENS_DATA_13 0x56
      #define MPU6050_RA_EXT_SENS_DATA_14 0x57
      #define MPU6050_RA_EXT_SENS_DATA_15 0x58
      #define MPU6050_RA_EXT_SENS_DATA_16 0x59
      #define MPU6050_RA_EXT_SENS_DATA_17 0x5A
      #define MPU6050_RA_EXT_SENS_DATA_18 0x5B
      #define MPU6050_RA_EXT_SENS_DATA_19 0x5C
      #define MPU6050_RA_EXT_SENS_DATA_20 0x5D
      #define MPU6050_RA_EXT_SENS_DATA_21 0x5E
      #define MPU6050_RA_EXT_SENS_DATA_22 0x5F
      #define MPU6050_RA_EXT_SENS_DATA_23 0x60
      #define MPU6050_RA_MOT_DETECT_STATUS    0x61
      #define MPU6050_RA_I2C_SLV0_DO      0x63
      #define MPU6050_RA_I2C_SLV1_DO      0x64
      #define MPU6050_RA_I2C_SLV2_DO      0x65
      #define MPU6050_RA_I2C_SLV3_DO      0x66
      #define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
      #define MPU6050_RA_SIGNAL_PATH_RESET    0x68
      #define MPU6050_RA_MOT_DETECT_CTRL      0x69
      #define MPU6050_RA_USER_CTRL        0x6A
      #define MPU6050_RA_PWR_MGMT_1       0x6B
      #define MPU6050_RA_PWR_MGMT_2       0x6C
      #define MPU6050_RA_BANK_SEL         0x6D
      #define MPU6050_RA_MEM_START_ADDR   0x6E
      #define MPU6050_RA_MEM_R_W          0x6F
      #define MPU6050_RA_DMP_CFG_1        0x70
      #define MPU6050_RA_DMP_CFG_2        0x71
      #define MPU6050_RA_FIFO_COUNTH      0x72
      #define MPU6050_RA_FIFO_COUNTL      0x73
      #define MPU6050_RA_FIFO_R_W         0x74
      #define MPU6050_RA_WHO_AM_I         0x75

      #define MPU6050_SELF_TEST_XA_1_BIT     0x07
      #define MPU6050_SELF_TEST_XA_1_LENGTH  0x03
      #define MPU6050_SELF_TEST_XA_2_BIT     0x05
      #define MPU6050_SELF_TEST_XA_2_LENGTH  0x02
      #define MPU6050_SELF_TEST_YA_1_BIT     0x07
      #define MPU6050_SELF_TEST_YA_1_LENGTH  0x03
      #define MPU6050_SELF_TEST_YA_2_BIT     0x03
      #define MPU6050_SELF_TEST_YA_2_LENGTH  0x02
      #define MPU6050_SELF_TEST_ZA_1_BIT     0x07
      #define MPU6050_SELF_TEST_ZA_1_LENGTH  0x03
      #define MPU6050_SELF_TEST_ZA_2_BIT     0x01
      #define MPU6050_SELF_TEST_ZA_2_LENGTH  0x02

      #define MPU6050_SELF_TEST_XG_1_BIT     0x04
      #define MPU6050_SELF_TEST_XG_1_LENGTH  0x05
      #define MPU6050_SELF_TEST_YG_1_BIT     0x04
      #define MPU6050_SELF_TEST_YG_1_LENGTH  0x05
      #define MPU6050_SELF_TEST_ZG_1_BIT     0x04
      #define MPU6050_SELF_TEST_ZG_1_LENGTH  0x05

      #define MPU6050_TC_PWR_MODE_BIT     7
      #define MPU6050_TC_OFFSET_BIT       6
      #define MPU6050_TC_OFFSET_LENGTH    6
      #define MPU6050_TC_OTP_BNK_VLD_BIT  0

      #define MPU6050_VDDIO_LEVEL_VLOGIC  0
      #define MPU6050_VDDIO_LEVEL_VDD     1

      #define MPU6050_CFG_EXT_SYNC_SET_BIT    5
      #define MPU6050_CFG_EXT_SYNC_SET_LENGTH 3
      #define MPU6050_CFG_DLPF_CFG_BIT    2
      #define MPU6050_CFG_DLPF_CFG_LENGTH 3

      #define MPU6050_EXT_SYNC_DISABLED       0x0
      #define MPU6050_EXT_SYNC_TEMP_OUT_L     0x1
      #define MPU6050_EXT_SYNC_GYRO_XOUT_L    0x2
      #define MPU6050_EXT_SYNC_GYRO_YOUT_L    0x3
      #define MPU6050_EXT_SYNC_GYRO_ZOUT_L    0x4
      #define MPU6050_EXT_SYNC_ACCEL_XOUT_L   0x5
      #define MPU6050_EXT_SYNC_ACCEL_YOUT_L   0x6
      #define MPU6050_EXT_SYNC_ACCEL_ZOUT_L   0x7

      #define MPU6050_DLPF_BW_256         0x00
      #define MPU6050_DLPF_BW_188         0x01
      #define MPU6050_DLPF_BW_98          0x02
      #define MPU6050_DLPF_BW_42          0x03
      #define MPU6050_DLPF_BW_20          0x04
      #define MPU6050_DLPF_BW_10          0x05
      #define MPU6050_DLPF_BW_5           0x06

      #define MPU6050_GCONFIG_FS_SEL_BIT      4
      #define MPU6050_GCONFIG_FS_SEL_LENGTH   2

      #define MPU6050_GYRO_FS_250         0x00
      #define MPU6050_GYRO_FS_500         0x01
      #define MPU6050_GYRO_FS_1000        0x02
      #define MPU6050_GYRO_FS_2000        0x03

      #define MPU6050_ACONFIG_XA_ST_BIT           7
      #define MPU6050_ACONFIG_YA_ST_BIT           6
      #define MPU6050_ACONFIG_ZA_ST_BIT           5
      #define MPU6050_ACONFIG_AFS_SEL_BIT         4
      #define MPU6050_ACONFIG_AFS_SEL_LENGTH      2
      #define MPU6050_ACONFIG_ACCEL_HPF_BIT       2
      #define MPU6050_ACONFIG_ACCEL_HPF_LENGTH    3

      #define MPU6050_ACCEL_FS_2          0x00
      #define MPU6050_ACCEL_FS_4          0x01
      #define MPU6050_ACCEL_FS_8          0x02
      #define MPU6050_ACCEL_FS_16         0x03

      #define MPU6050_DHPF_RESET          0x00
      #define MPU6050_DHPF_5              0x01
      #define MPU6050_DHPF_2P5            0x02
      #define MPU6050_DHPF_1P25           0x03
      #define MPU6050_DHPF_0P63           0x04
      #define MPU6050_DHPF_HOLD           0x07

      #define MPU6050_TEMP_FIFO_EN_BIT    7
      #define MPU6050_XG_FIFO_EN_BIT      6
      #define MPU6050_YG_FIFO_EN_BIT      5
      #define MPU6050_ZG_FIFO_EN_BIT      4
      #define MPU6050_ACCEL_FIFO_EN_BIT   3
      #define MPU6050_SLV2_FIFO_EN_BIT    2
      #define MPU6050_SLV1_FIFO_EN_BIT    1
      #define MPU6050_SLV0_FIFO_EN_BIT    0

      #define MPU6050_MULT_MST_EN_BIT     7
      #define MPU6050_WAIT_FOR_ES_BIT     6
      #define MPU6050_SLV_3_FIFO_EN_BIT   5
      #define MPU6050_I2C_MST_P_NSR_BIT   4
      #define MPU6050_I2C_MST_CLK_BIT     3
      #define MPU6050_I2C_MST_CLK_LENGTH  4

      #define MPU6050_CLOCK_DIV_348       0x0
      #define MPU6050_CLOCK_DIV_333       0x1
      #define MPU6050_CLOCK_DIV_320       0x2
      #define MPU6050_CLOCK_DIV_308       0x3
      #define MPU6050_CLOCK_DIV_296       0x4
      #define MPU6050_CLOCK_DIV_286       0x5
      #define MPU6050_CLOCK_DIV_276       0x6
      #define MPU6050_CLOCK_DIV_267       0x7
      #define MPU6050_CLOCK_DIV_258       0x8
      #define MPU6050_CLOCK_DIV_500       0x9
      #define MPU6050_CLOCK_DIV_471       0xA
      #define MPU6050_CLOCK_DIV_444       0xB
      #define MPU6050_CLOCK_DIV_421       0xC
      #define MPU6050_CLOCK_DIV_400       0xD
      #define MPU6050_CLOCK_DIV_381       0xE
      #define MPU6050_CLOCK_DIV_364       0xF

      #define MPU6050_I2C_SLV_RW_BIT      7
      #define MPU6050_I2C_SLV_ADDR_BIT    6
      #define MPU6050_I2C_SLV_ADDR_LENGTH 7
      #define MPU6050_I2C_SLV_EN_BIT      7
      #define MPU6050_I2C_SLV_BYTE_SW_BIT 6
      #define MPU6050_I2C_SLV_REG_DIS_BIT 5
      #define MPU6050_I2C_SLV_GRP_BIT     4
      #define MPU6050_I2C_SLV_LEN_BIT     3
      #define MPU6050_I2C_SLV_LEN_LENGTH  4

      #define MPU6050_I2C_SLV4_RW_BIT         7
      #define MPU6050_I2C_SLV4_ADDR_BIT       6
      #define MPU6050_I2C_SLV4_ADDR_LENGTH    7
      #define MPU6050_I2C_SLV4_EN_BIT         7
      #define MPU6050_I2C_SLV4_INT_EN_BIT     6
      #define MPU6050_I2C_SLV4_REG_DIS_BIT    5
      #define MPU6050_I2C_SLV4_MST_DLY_BIT    4
      #define MPU6050_I2C_SLV4_MST_DLY_LENGTH 5

      #define MPU6050_MST_PASS_THROUGH_BIT    7
      #define MPU6050_MST_I2C_SLV4_DONE_BIT   6
      #define MPU6050_MST_I2C_LOST_ARB_BIT    5
      #define MPU6050_MST_I2C_SLV4_NACK_BIT   4
      #define MPU6050_MST_I2C_SLV3_NACK_BIT   3
      #define MPU6050_MST_I2C_SLV2_NACK_BIT   2
      #define MPU6050_MST_I2C_SLV1_NACK_BIT   1
      #define MPU6050_MST_I2C_SLV0_NACK_BIT   0

      #define MPU6050_INTCFG_INT_LEVEL_BIT        7
      #define MPU6050_INTCFG_INT_OPEN_BIT         6
      #define MPU6050_INTCFG_LATCH_INT_EN_BIT     5
      #define MPU6050_INTCFG_INT_RD_CLEAR_BIT     4
      #define MPU6050_INTCFG_FSYNC_INT_LEVEL_BIT  3
      #define MPU6050_INTCFG_FSYNC_INT_EN_BIT     2
      #define MPU6050_INTCFG_I2C_BYPASS_EN_BIT    1
      #define MPU6050_INTCFG_CLKOUT_EN_BIT        0

      #define MPU6050_INTMODE_ACTIVEHIGH  0x00
      #define MPU6050_INTMODE_ACTIVELOW   0x01

      #define MPU6050_INTDRV_PUSHPULL     0x00
      #define MPU6050_INTDRV_OPENDRAIN    0x01

      #define MPU6050_INTLATCH_50USPULSE  0x00
      #define MPU6050_INTLATCH_WAITCLEAR  0x01

      #define MPU6050_INTCLEAR_STATUSREAD 0x00
      #define MPU6050_INTCLEAR_ANYREAD    0x01

      #define MPU6050_INTERRUPT_FF_BIT            7
      #define MPU6050_INTERRUPT_MOT_BIT           6
      #define MPU6050_INTERRUPT_ZMOT_BIT          5
      #define MPU6050_INTERRUPT_FIFO_OFLOW_BIT    4
      #define MPU6050_INTERRUPT_I2C_MST_INT_BIT   3
      #define MPU6050_INTERRUPT_PLL_RDY_INT_BIT   2
      #define MPU6050_INTERRUPT_DMP_INT_BIT       1
      #define MPU6050_INTERRUPT_DATA_RDY_BIT      0

      // TODO: figure out what these actually do
      // UMPL source code is not very obivous
      #define MPU6050_DMPINT_5_BIT            5
      #define MPU6050_DMPINT_4_BIT            4
      #define MPU6050_DMPINT_3_BIT            3
      #define MPU6050_DMPINT_2_BIT            2
      #define MPU6050_DMPINT_1_BIT            1
      #define MPU6050_DMPINT_0_BIT            0

      #define MPU6050_MOTION_MOT_XNEG_BIT     7
      #define MPU6050_MOTION_MOT_XPOS_BIT     6
      #define MPU6050_MOTION_MOT_YNEG_BIT     5
      #define MPU6050_MOTION_MOT_YPOS_BIT     4
      #define MPU6050_MOTION_MOT_ZNEG_BIT     3
      #define MPU6050_MOTION_MOT_ZPOS_BIT     2
      #define MPU6050_MOTION_MOT_ZRMOT_BIT    0

      #define MPU6050_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
      #define MPU6050_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
      #define MPU6050_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
      #define MPU6050_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
      #define MPU6050_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
      #define MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

      #define MPU6050_PATHRESET_GYRO_RESET_BIT    2
      #define MPU6050_PATHRESET_ACCEL_RESET_BIT   1
      #define MPU6050_PATHRESET_TEMP_RESET_BIT    0

      #define MPU6050_DETECT_ACCEL_ON_DELAY_BIT       5
      #define MPU6050_DETECT_ACCEL_ON_DELAY_LENGTH    2
      #define MPU6050_DETECT_FF_COUNT_BIT             3
      #define MPU6050_DETECT_FF_COUNT_LENGTH          2
      #define MPU6050_DETECT_MOT_COUNT_BIT            1
      #define MPU6050_DETECT_MOT_COUNT_LENGTH         2

      #define MPU6050_DETECT_DECREMENT_RESET  0x0
      #define MPU6050_DETECT_DECREMENT_1      0x1
      #define MPU6050_DETECT_DECREMENT_2      0x2
      #define MPU6050_DETECT_DECREMENT_4      0x3

      #define MPU6050_USERCTRL_DMP_EN_BIT             7
      #define MPU6050_USERCTRL_FIFO_EN_BIT            6
      #define MPU6050_USERCTRL_I2C_MST_EN_BIT         5
      #define MPU6050_USERCTRL_I2C_IF_DIS_BIT         4
      #define MPU6050_USERCTRL_DMP_RESET_BIT          3
      #define MPU6050_USERCTRL_FIFO_RESET_BIT         2
      #define MPU6050_USERCTRL_I2C_MST_RESET_BIT      1
      #define MPU6050_USERCTRL_SIG_COND_RESET_BIT     0

      #define MPU6050_PWR1_DEVICE_RESET_BIT   7
      #define MPU6050_PWR1_SLEEP_BIT          6
      #define MPU6050_PWR1_CYCLE_BIT          5
      #define MPU6050_PWR1_TEMP_DIS_BIT       3
      #define MPU6050_PWR1_CLKSEL_BIT         2
      #define MPU6050_PWR1_CLKSEL_LENGTH      3

      #define MPU6050_CLOCK_INTERNAL          0x00
      #define MPU6050_CLOCK_PLL_XGYRO         0x01
      #define MPU6050_CLOCK_PLL_YGYRO         0x02
      #define MPU6050_CLOCK_PLL_ZGYRO         0x03
      #define MPU6050_CLOCK_PLL_EXT32K        0x04
      #define MPU6050_CLOCK_PLL_EXT19M        0x05
      #define MPU6050_CLOCK_KEEP_RESET        0x07

      #define MPU6050_PWR2_LP_WAKE_CTRL_BIT       7
      #define MPU6050_PWR2_LP_WAKE_CTRL_LENGTH    2
      #define MPU6050_PWR2_STBY_XA_BIT            5
      #define MPU6050_PWR2_STBY_YA_BIT            4
      #define MPU6050_PWR2_STBY_ZA_BIT            3
      #define MPU6050_PWR2_STBY_XG_BIT            2
      #define MPU6050_PWR2_STBY_YG_BIT            1
      #define MPU6050_PWR2_STBY_ZG_BIT            0

      #define MPU6050_WAKE_FREQ_1P25      0x0
      #define MPU6050_WAKE_FREQ_2P5       0x1
      #define MPU6050_WAKE_FREQ_5         0x2
      #define MPU6050_WAKE_FREQ_10        0x3

      #define MPU6050_BANKSEL_PRFTCH_EN_BIT       6
      #define MPU6050_BANKSEL_CFG_USER_BANK_BIT   5
      #define MPU6050_BANKSEL_MEM_SEL_BIT         4
      #define MPU6050_BANKSEL_MEM_SEL_LENGTH      5

      #define MPU6050_WHO_AM_I_BIT        6
      #define MPU6050_WHO_AM_I_LENGTH     6

      #define MPU6050_DMP_MEMORY_BANKS        8
      #define MPU6050_DMP_MEMORY_BANK_SIZE    256
      #define MPU6050_DMP_MEMORY_CHUNK_SIZE   U8_T(16)

      // note: DMP code memory blocks defined at end of header file

      typedef struct {
         uint8_t devAddr;
          uint8_t buffer[14];
      } MPU6050_t;

      typedef struct  {
           int32_t setXAccelOffset;
           int32_t setYAccelOffset;
           int32_t setZAccelOffset;
           int32_t setXGyroOffset;
           int32_t setYGyroOffset;
           int32_t setZGyroOffset;
      } MPU6050_Set_t;

      void MPU6050(uint8_t address);

      uint8_t MPU6050_initialize(void);
      uint8_t MPU6050_testConnection(void);

      // AUX_VDDIO register
      uint8_t MPU6050_getAuxVDDIOLevel(uint8_t *data);
      uint8_t MPU6050_setAuxVDDIOLevel(uint8_t level);

      // SMPLRT_DIV register
      uint8_t MPU6050_getRate(uint8_t *data);
      uint8_t MPU6050_setRate(uint8_t rate);

      // CONFIG register
      uint8_t MPU6050_getExternalFrameSync(uint8_t *data);
      uint8_t MPU6050_setExternalFrameSync(uint8_t sync);
      uint8_t MPU6050_getDLPFMode(uint8_t *data);
      uint8_t MPU6050_setDLPFMode(uint8_t bandwidth);

      // GYRO_CONFIG register
      uint8_t MPU6050_getFullScaleGyroRange(uint8_t *data);
      uint8_t MPU6050_setFullScaleGyroRange(uint8_t range);

      // SELF_TEST registers
      uint8_t MPU6050_getAccelXSelfTestFactoryTrim(uint8_t *data);
      uint8_t MPU6050_getAccelYSelfTestFactoryTrim(uint8_t *data);
      uint8_t MPU6050_getAccelZSelfTestFactoryTrim(uint8_t *data);

      uint8_t MPU6050_getGyroXSelfTestFactoryTrim(uint8_t *data);
      uint8_t MPU6050_getGyroYSelfTestFactoryTrim(uint8_t *data);
      uint8_t MPU6050_getGyroZSelfTestFactoryTrim(uint8_t *data);

      // ACCEL_CONFIG register
      uint8_t MPU6050_getAccelXSelfTest(uint8_t *data);
      uint8_t MPU6050_setAccelXSelfTest(uint8_t enabled);
      uint8_t MPU6050_getAccelYSelfTest(uint8_t *data);
      uint8_t MPU6050_setAccelYSelfTest(uint8_t enabled);
      uint8_t MPU6050_getAccelZSelfTest(uint8_t *data);
      uint8_t MPU6050_setAccelZSelfTest(uint8_t enabled);
      uint8_t MPU6050_getFullScaleAccelRange(uint8_t *data);
      uint8_t MPU6050_setFullScaleAccelRange(uint8_t range);
      uint8_t MPU6050_getDHPFMode(uint8_t *data);
      uint8_t MPU6050_setDHPFMode(uint8_t mode);

      // FF_THR register
      uint8_t MPU6050_getFreefallDetectionThreshold(uint8_t *data);
      uint8_t MPU6050_setFreefallDetectionThreshold(uint8_t threshold);

      // FF_DUR register
      uint8_t MPU6050_getFreefallDetectionDuration(uint8_t *data);
      uint8_t MPU6050_setFreefallDetectionDuration(uint8_t duration);

      // MOT_THR register
      uint8_t MPU6050_getMotionDetectionThreshold(uint8_t *data);
      uint8_t MPU6050_setMotionDetectionThreshold(uint8_t threshold);

      // MOT_DUR register
      uint8_t MPU6050_getMotionDetectionDuration(uint8_t *data);
      uint8_t MPU6050_setMotionDetectionDuration(uint8_t duration);

      // ZRMOT_THR register
      uint8_t MPU6050_getZeroMotionDetectionThreshold(uint8_t *data);
      uint8_t MPU6050_setZeroMotionDetectionThreshold(uint8_t threshold);

      // ZRMOT_DUR register
      uint8_t MPU6050_getZeroMotionDetectionDuration(uint8_t *data);
      uint8_t MPU6050_setZeroMotionDetectionDuration(uint8_t duration);

      // FIFO_EN register
      uint8_t MPU6050_getTempFIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setTempFIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getXGyroFIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setXGyroFIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getYGyroFIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setYGyroFIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getZGyroFIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setZGyroFIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getAccelFIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setAccelFIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getSlave2FIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setSlave2FIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getSlave1FIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setSlave1FIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getSlave0FIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setSlave0FIFOEnabled(uint8_t enabled);

      // I2C_MST_CTRL register
      uint8_t MPU6050_getMultiMasterEnabled(uint8_t *data);
      uint8_t MPU6050_setMultiMasterEnabled(uint8_t enabled);
      uint8_t MPU6050_getWaitForExternalSensorEnabled(uint8_t *data);
      uint8_t MPU6050_setWaitForExternalSensorEnabled(uint8_t enabled);
      uint8_t MPU6050_getSlave3FIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setSlave3FIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getSlaveReadWriteTransitionEnabled(uint8_t *data);
      uint8_t MPU6050_setSlaveReadWriteTransitionEnabled(uint8_t enabled);
      uint8_t MPU6050_getMasterClockSpeed(uint8_t *data);
      uint8_t MPU6050_setMasterClockSpeed(uint8_t speed);

      // I2C_SLV* registers (Slave 0-3)
      uint8_t MPU6050_getSlaveAddress(uint8_t num, uint8_t *data);
      uint8_t MPU6050_setSlaveAddress(uint8_t num, uint8_t address);
      uint8_t MPU6050_getSlaveRegister(uint8_t num, uint8_t *data);
      uint8_t MPU6050_setSlaveRegister(uint8_t num, uint8_t reg);
      uint8_t MPU6050_getSlaveEnabled(uint8_t num, uint8_t *data);
      uint8_t MPU6050_setSlaveEnabled(uint8_t num, uint8_t enabled);
      uint8_t MPU6050_getSlaveWordByteSwap(uint8_t num, uint8_t *data);
      uint8_t MPU6050_setSlaveWordByteSwap(uint8_t num, uint8_t enabled);
      uint8_t MPU6050_getSlaveWriteMode(uint8_t num, uint8_t *data);
      uint8_t MPU6050_setSlaveWriteMode(uint8_t num, uint8_t mode);
      uint8_t MPU6050_getSlaveWordGroupOffset(uint8_t num, uint8_t *data);
      uint8_t MPU6050_setSlaveWordGroupOffset(uint8_t num, uint8_t enabled);
      uint8_t MPU6050_getSlaveDataLength(uint8_t num, uint8_t *data);
      uint8_t MPU6050_setSlaveDataLength(uint8_t num, uint8_t length);

      // I2C_SLV* registers (Slave 4)
      uint8_t MPU6050_getSlave4Address(uint8_t *data);
      uint8_t MPU6050_setSlave4Address(uint8_t address);
      uint8_t MPU6050_getSlave4Register(uint8_t *data);
      uint8_t MPU6050_setSlave4Register(uint8_t reg);
      uint8_t MPU6050_setSlave4OutputByte(uint8_t data);
      uint8_t MPU6050_getSlave4Enabled(uint8_t *data);
      uint8_t MPU6050_setSlave4Enabled(uint8_t enabled);
      uint8_t MPU6050_getSlave4InterruptEnabled(uint8_t *data);
      uint8_t MPU6050_setSlave4InterruptEnabled(uint8_t enabled);
      uint8_t MPU6050_getSlave4WriteMode(uint8_t *data);
      uint8_t MPU6050_setSlave4WriteMode(uint8_t mode);
      uint8_t MPU6050_getSlave4MasterDelay(uint8_t *data);
      uint8_t MPU6050_setSlave4MasterDelay(uint8_t delay);
      uint8_t MPU6050_getSlate4InputByte(uint8_t *data);

      // I2C_MST_STATUS register
      uint8_t MPU6050_getPassthroughStatus(uint8_t *data);
      uint8_t MPU6050_getSlave4IsDone(uint8_t *data);
      uint8_t MPU6050_getLostArbitration(uint8_t *data);
      uint8_t MPU6050_getSlave4Nack(uint8_t *data);
      uint8_t MPU6050_getSlave3Nack(uint8_t *data);
      uint8_t MPU6050_getSlave2Nack(uint8_t *data);
      uint8_t MPU6050_getSlave1Nack(uint8_t *data);
      uint8_t MPU6050_getSlave0Nack(uint8_t *data);

      // INT_PIN_CFG register
      uint8_t MPU6050_getInterruptMode(uint8_t *data);
      uint8_t MPU6050_setInterruptMode(uint8_t mode);
      uint8_t MPU6050_getInterruptDrive(uint8_t *data);
      uint8_t MPU6050_setInterruptDrive(uint8_t drive);
      uint8_t MPU6050_getInterruptLatch(uint8_t *data);
      uint8_t MPU6050_setInterruptLatch(uint8_t latch);
      uint8_t MPU6050_getInterruptLatchClear(uint8_t *data);
      uint8_t MPU6050_setInterruptLatchClear(uint8_t clear);
      uint8_t MPU6050_getFSyncInterruptLevel(uint8_t *data);
      uint8_t MPU6050_setFSyncInterruptLevel(uint8_t level);
      uint8_t MPU6050_getFSyncInterruptEnabled(uint8_t *data);
      uint8_t MPU6050_setFSyncInterruptEnabled(uint8_t enabled);
      uint8_t MPU6050_getI2CBypassEnabled(uint8_t *data);
      uint8_t MPU6050_setI2CBypassEnabled(uint8_t enabled);
      uint8_t MPU6050_getClockOutputEnabled(uint8_t *data);
      uint8_t MPU6050_setClockOutputEnabled(uint8_t enabled);

      // INT_ENABLE register
      uint8_t MPU6050_getIntEnabled(uint8_t *data);
      uint8_t MPU6050_setIntEnabled(uint8_t enabled);
      uint8_t MPU6050_getIntFreefallEnabled(uint8_t *data);
      uint8_t MPU6050_setIntFreefallEnabled(uint8_t enabled);
      uint8_t MPU6050_getIntMotionEnabled(uint8_t *data);
      uint8_t MPU6050_setIntMotionEnabled(uint8_t enabled);
      uint8_t MPU6050_getIntZeroMotionEnabled(uint8_t *data);
      uint8_t MPU6050_setIntZeroMotionEnabled(uint8_t enabled);
      uint8_t MPU6050_getIntFIFOBufferOverflowEnabled(uint8_t *data);
      uint8_t MPU6050_setIntFIFOBufferOverflowEnabled(uint8_t enabled);
      uint8_t MPU6050_getIntI2CMasterEnabled(uint8_t *data);
      uint8_t MPU6050_setIntI2CMasterEnabled(uint8_t enabled);
      uint8_t MPU6050_getIntDataReadyEnabled(uint8_t *data);
      uint8_t MPU6050_setIntDataReadyEnabled(uint8_t enabled);

      // INT_STATUS register
      uint8_t MPU6050_getIntStatus(uint8_t *data);
      uint8_t MPU6050_getIntFreefallStatus(uint8_t *data);
      uint8_t MPU6050_getIntMotionStatus(uint8_t *data);
      uint8_t MPU6050_getIntZeroMotionStatus(uint8_t *data);
      uint8_t MPU6050_getIntFIFOBufferOverflowStatus(uint8_t *data);
      uint8_t MPU6050_getIntI2CMasterStatus(uint8_t *data);
      uint8_t MPU6050_getIntDataReadyStatus(uint8_t *data);

      // ACCEL_*OUT_* registers
      uint8_t MPU6050_getMotion9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my, int16_t *mz);
      uint8_t MPU6050_getMotion6(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz);
      uint8_t MPU6050_getMotion6_Mod(VectorInt16 *acc, VectorInt16 *gyro, int16_t *temperature);

      uint8_t MPU6050_getAcceleration(int16_t *x, int16_t *y, int16_t *z);
      uint8_t MPU6050_getAccelerationX(int16_t *ax);
      uint8_t MPU6050_getAccelerationY(int16_t *ay);
      uint8_t MPU6050_getAccelerationZ(int16_t *az);

      // TEMP_OUT_* registers
      uint8_t MPU6050_getTemperature(int16_t *data);

      // GYRO_*OUT_* registers
      uint8_t MPU6050_getRotation(int16_t *x, int16_t *y, int16_t *z);
      uint8_t MPU6050_getRotationX(int16_t *data);
      uint8_t MPU6050_getRotationY(int16_t *data);
      uint8_t MPU6050_getRotationZ(int16_t *data);

      // EXT_SENS_DATA_* registers
      uint8_t MPU6050_getExternalSensorByte(int position, uint8_t *data);
      uint8_t MPU6050_getExternalSensorWord(int position, uint16_t *data);
      uint32_t MPU6050_getExternalSensorDWord(int position, uint32_t *data);

      // MOT_DETECT_STATUS register
      uint8_t MPU6050_getMotionStatus(uint8_t *data);
      uint8_t MPU6050_getXNegMotionDetected(uint8_t *data);
      uint8_t MPU6050_getXPosMotionDetected(uint8_t *data);
      uint8_t MPU6050_getYNegMotionDetected(uint8_t *data);
      uint8_t MPU6050_getYPosMotionDetected(uint8_t *data);
      uint8_t MPU6050_getZNegMotionDetected(uint8_t *data);
      uint8_t MPU6050_getZPosMotionDetected(uint8_t *data);
      uint8_t MPU6050_getZeroMotionDetected(uint8_t *data);

      // I2C_SLV*_DO register
      uint8_t MPU6050_setSlaveOutputByte(uint8_t num, uint8_t data);

      // I2C_MST_DELAY_CTRL register
      uint8_t MPU6050_getExternalShadowDelayEnabled(uint8_t *data);
      uint8_t MPU6050_setExternalShadowDelayEnabled(uint8_t enabled);
      uint8_t MPU6050_getSlaveDelayEnabled(uint8_t num, uint8_t *data);
      uint8_t MPU6050_setSlaveDelayEnabled(uint8_t num, uint8_t enabled);

      // SIGNAL_PATH_RESET register
      uint8_t MPU6050_resetGyroscopePath(void);
      uint8_t MPU6050_resetAccelerometerPath(void);
      uint8_t MPU6050_resetTemperaturePath(void);

      // MOT_DETECT_CTRL register
      uint8_t MPU6050_getAccelerometerPowerOnDelay(uint8_t *data);
      uint8_t MPU6050_setAccelerometerPowerOnDelay(uint8_t delay);
      uint8_t MPU6050_getFreefallDetectionCounterDecrement(uint8_t *data);
      uint8_t MPU6050_setFreefallDetectionCounterDecrement(uint8_t decrement);
      uint8_t MPU6050_getMotionDetectionCounterDecrement(uint8_t *data);
      uint8_t MPU6050_setMotionDetectionCounterDecrement(uint8_t decrement);

      // USER_CTRL register
      uint8_t MPU6050_getFIFOEnabled(uint8_t *data);
      uint8_t MPU6050_setFIFOEnabled(uint8_t enabled);
      uint8_t MPU6050_getI2CMasterModeEnabled(uint8_t *data);
      uint8_t MPU6050_setI2CMasterModeEnabled(uint8_t enabled);
      uint8_t MPU6050_switchSPIEnabled(uint8_t enabled);
      uint8_t MPU6050_resetFIFO(void);
      uint8_t MPU6050_resetI2CMaster(void);
      uint8_t MPU6050_resetSensors(void);

      // PWR_MGMT_1 register
      uint8_t MPU6050_reset(void);
      uint8_t MPU6050_getSleepEnabled(uint8_t *data);
      uint8_t MPU6050_setSleepEnabled(uint8_t enabled);
      uint8_t MPU6050_getWakeCycleEnabled(uint8_t *data);
      uint8_t MPU6050_setWakeCycleEnabled(uint8_t enabled);
      uint8_t MPU6050_getTempSensorEnabled(uint8_t *data);
      uint8_t MPU6050_setTempSensorEnabled(uint8_t enabled);
      uint8_t MPU6050_getClockSource(uint8_t *data);
      uint8_t MPU6050_setClockSource(uint8_t source);

      // PWR_MGMT_2 register
      uint8_t MPU6050_getWakeFrequency(uint8_t *data);
      uint8_t MPU6050_setWakeFrequency(uint8_t frequency);
      uint8_t MPU6050_getStandbyXAccelEnabled(uint8_t *data);
      uint8_t MPU6050_setStandbyXAccelEnabled(uint8_t enabled);
      uint8_t MPU6050_getStandbyYAccelEnabled(uint8_t *data);
      uint8_t MPU6050_setStandbyYAccelEnabled(uint8_t enabled);
      uint8_t MPU6050_getStandbyZAccelEnabled(uint8_t *data);
      uint8_t MPU6050_setStandbyZAccelEnabled(uint8_t enabled);
      uint8_t MPU6050_getStandbyXGyroEnabled(uint8_t *data);
      uint8_t MPU6050_setStandbyXGyroEnabled(uint8_t enabled);
      uint8_t MPU6050_getStandbyYGyroEnabled(uint8_t *data);
      uint8_t MPU6050_setStandbyYGyroEnabled(uint8_t enabled);
      uint8_t MPU6050_getStandbyZGyroEnabled(uint8_t *data);
      uint8_t MPU6050_setStandbyZGyroEnabled(uint8_t enabled);

      // FIFO_COUNT_* registers
      uint8_t MPU6050_getFIFOCount(uint16_t *data);

      // FIFO_R_W register
      uint8_t MPU6050_getFIFOByte(uint8_t *data);
      uint8_t MPU6050_setFIFOByte(uint8_t data);
      uint8_t MPU6050_getFIFOBytes(uint8_t *data, uint8_t length);

      // WHO_AM_I register
      uint8_t MPU6050_getDeviceID(uint8_t *id);
      uint8_t MPU6050_setDeviceID(uint8_t id);

      // ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========
      uint8_t MPU6050_getOTPBankValid(uint8_t *data);
      uint8_t MPU6050_setOTPBankValid(uint8_t enabled);

      // XA_OFFS_TC register
      uint8_t MPU6050_getXAccelOffsetTC(int8_t *data);
      uint8_t MPU6050_setXAccelOffsetTC(int8_t offset);

      // YA_OFFS_TC register
      uint8_t MPU6050_getYAccelOffsetTC(int8_t *data);
      uint8_t MPU6050_setYAccelOffsetTC(int8_t offset);

      // ZA_OFFS_TC register
      uint8_t MPU6050_getZAccelOffsetTC(int8_t *data);
      uint8_t MPU6050_setZAccelOffsetTC(int8_t offset);

      // XG_OFFS_TC register
      uint8_t MPU6050_getXGyroOffsetTC(int8_t *data);
      uint8_t MPU6050_setXGyroOffsetTC(int8_t offset);

      // YG_OFFS_TC register
      uint8_t MPU6050_getYGyroOffsetTC(int8_t *data);
      uint8_t MPU6050_setYGyroOffsetTC(int8_t offset);

      // ZG_OFFS_TC register
      uint8_t MPU6050_getZGyroOffsetTC(int8_t *data);
      uint8_t MPU6050_setZGyroOffsetTC(int8_t offset);

      // XG_OFFS_USR* registers
      uint8_t MPU6050_getXGyroOffsetUser(int16_t *data);
      uint8_t MPU6050_setXGyroOffsetUser(int16_t offset);

      // YG_OFFS_USR* register
      uint8_t MPU6050_getYGyroOffsetUser(int16_t *data);
      uint8_t MPU6050_setYGyroOffsetUser(int16_t offset);

      // ZG_OFFS_USR* register
      uint8_t MPU6050_getZGyroOffsetUser(int16_t *data);
      uint8_t MPU6050_setZGyroOffsetUser(int16_t offset);

      // X_FINE_GAIN register
      uint8_t getXFineGain(int8_t *data);
      uint8_t MPU6050_setXFineGain(int8_t gain);

      // Y_FINE_GAIN register
      uint8_t getYFineGain(int8_t *data);
      uint8_t MPU6050_setYFineGain(int8_t gain);

      // Z_FINE_GAIN register
      uint8_t getZFineGain(int8_t *data);
      uint8_t MPU6050_setZFineGain(int8_t gain);

      // XA_OFFS_* registers
      uint8_t MPU6050_getXAccelOffset(int16_t *data);
      uint8_t MPU6050_setXAccelOffset(int16_t offset);

      // YA_OFFS_* register
      uint8_t MPU6050_getYAccelOffset(int16_t *data);
      uint8_t MPU6050_setYAccelOffset(int16_t offset);

      // ZA_OFFS_* register
      uint8_t MPU6050_getZAccelOffset(int16_t *data);
      uint8_t MPU6050_setZAccelOffset(int16_t offset);

      // XG_OFFS_USR* registers
      uint8_t MPU6050_getXGyroOffset(int16_t *data);
      uint8_t MPU6050_setXGyroOffset(int16_t offset);

      // YG_OFFS_USR* register
      uint8_t MPU6050_getYGyroOffset(int16_t *data);
      uint8_t MPU6050_setYGyroOffset(int16_t offset);

      // ZG_OFFS_USR* register
      uint8_t MPU6050_getZGyroOffset(int16_t *data);
      uint8_t MPU6050_setZGyroOffset(int16_t offset);

      // INT_ENABLE register (DMP functions)
      uint8_t MPU6050_getIntPLLReadyEnabled(uint8_t *data);
      uint8_t MPU6050_setIntPLLReadyEnabled(uint8_t enabled);
      uint8_t MPU6050_getIntDMPEnabled(uint8_t *data);
      uint8_t MPU6050_setIntDMPEnabled(uint8_t enabled);

      // DMP_INT_STATUS
      uint8_t MPU6050_getDMPInt5Status(uint8_t *data);
      uint8_t MPU6050_getDMPInt4Status(uint8_t *data);
      uint8_t MPU6050_getDMPInt3Status(uint8_t *data);
      uint8_t MPU6050_getDMPInt2Status(uint8_t *data);
      uint8_t MPU6050_getDMPInt1Status(uint8_t *data);
      uint8_t MPU6050_getDMPInt0Status(uint8_t *data);

      // INT_STATUS register (DMP functions)
      uint8_t MPU6050_getIntPLLReadyStatus(uint8_t *data);
      uint8_t MPU6050_getIntDMPStatus(uint8_t *data);

      // USER_CTRL register (DMP functions)
      uint8_t MPU6050_getDMPEnabled(uint8_t *data);
      uint8_t MPU6050_setDMPEnabled(uint8_t enabled);
      uint8_t MPU6050_resetDMP(void);

      // BANK_SEL register
      uint8_t MPU6050_setMemoryBank(uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank);

      // MEM_START_ADDR register
      uint8_t MPU6050_setMemoryStartAddress(uint8_t address);

      // MEM_R_W register
      uint8_t MPU6050_readMemoryByte(uint8_t *data);
      uint8_t MPU6050_writeMemoryByte(uint8_t data);
      uint8_t MPU6050_readMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address);
      uint8_t MPU6050_writeMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t a_bank, uint8_t a_address, uint8_t verify, uint8_t useProgMem);
      uint8_t MPU6050_writeProgMemoryBlock(uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify);

      uint8_t MPU6050_writeDMPConfigurationSet(uint8_t *data, uint16_t dataSize, uint8_t useProgMem);
      uint8_t MPU6050_writeProgDMPConfigurationSet(uint8_t *data, uint16_t dataSize);

      // DMP_CFG_1 register
      uint8_t MPU6050_getDMPConfig1(uint8_t *data);
      uint8_t MPU6050_setDMPConfig1(uint8_t config);

      // DMP_CFG_2 register
      uint8_t MPU6050_getDMPConfig2(uint8_t *data);
      uint8_t MPU6050_setDMPConfig2(uint8_t config);

  #ifdef __cplusplus
    }
  #endif
#endif /* _MPU6050_H_ */

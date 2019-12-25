// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
// 03/28/2017 by Kamnev Yuriy <kamnev.u1969@gmail.com>
//
// Changelog:
//     2017-03-28 - ported to STM32 using Keil MDK Pack

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2013 Jeff Rowberg
Copyright (c) 2017 Kamnev Yuriy

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

#ifndef SRC_I2CDEVLIB_I2CDEV_H_
#define SRC_I2CDEVLIB_I2CDEV_H_
  #ifdef __cplusplus
    extern "C"
    {
  #endif

      /* Private includes ----------------------------------------------------------*/
      #include "tracking_main.h"

      #define I2CDev_Driver  Driver_I2C1
      #define MPU6050_I2C  I2C1

      /* Master RECEIVER mode -----------------------------*/
      /* --EV7 */
      #define  I2C_EVENT_MASTER_BYTE_RECEIVED (((I2C_SR2_BUSY | I2C_SR2_MSL)<<16) | I2C_SR1_RXNE)
      //((uint32_t)0x00030040)  /* BUSY, MSL and RXNE flags */

      /* Master TRANSMITTER mode --------------------------*/
      /* --EV8 */
      #define I2C_EVENT_MASTER_BYTE_TRANSMITTING (((I2C_SR2_TRA | I2C_SR2_BUSY | I2C_SR2_MSL)<<16) | I2C_SR1_TXE)
      //((uint32_t)0x00070080) /* TRA, BUSY, MSL, TXE flags */
      /* --EV8_2 */
      #define  I2C_EVENT_MASTER_BYTE_TRANSMITTED (((I2C_SR2_TRA | I2C_SR2_BUSY | I2C_SR2_MSL)<<16) | (I2C_SR1_TXE | I2C_SR1_BTF))
      //((uint32_t)0x00070084)  /* TRA, BUSY, MSL, TXE and BTF flags */

      uint8_t I2Cdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
      uint8_t I2Cdev_readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data);
      uint8_t I2Cdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
      uint8_t I2Cdev_readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data);
      uint8_t I2Cdev_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
      uint8_t I2Cdev_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data);
      #define I2Cdev_readBytes I2C_MasterMemoryRead
      //uint8_t I2Cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
      uint8_t I2Cdev_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

      uint8_t I2Cdev_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
      uint8_t I2Cdev_writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);
      uint8_t I2Cdev_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
      uint8_t I2Cdev_writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);
      uint8_t I2Cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
      uint8_t I2Cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
      #define I2Cdev_writeBytes I2C_MasterMemoryWrite//I2C_MasterMemoryDMA_Write//
      //uint8_t I2Cdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

      uint8_t I2Cdev_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

  #ifdef __cplusplus
    }
  #endif
#endif /* SRC_I2CDEVLIB_I2CDEV_H_ */

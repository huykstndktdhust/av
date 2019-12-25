// I2Cdev library collection - MPU6050 I2C device class, 6-axis MotionApps 2.0 implementation
// Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 5/20/2013 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

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

#ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
  #define _MPU6050_6AXIS_MOTIONAPPS20_H_
  #ifdef __cplusplus
    extern "C" {
  #endif
    /* Includes ------------------------------------------------------------------*/
    #include "tracking_main.h"

    // MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
    //#define MPU6050_INCLUDE_DMP_MOTIONAPPS20 move to project define

    // Tom Carpenter's conditional PROGMEM code
    // http://forum.arduino.cc/index.php?topic=129407.0
    #ifdef __AVR__
        #include <avr/pgmspace.h>
    #else
      // Teensy 3.0 library conditional PROGMEM code from Paul Stoffregen
      #ifndef __PGMSPACE_H_
        #define __PGMSPACE_H_ 1
        #include <inttypes.h>

        #define PROGMEM
        #define PGM_P   char *
        #define PSTR(str) (str)
        #define F(x) x

        typedef void prog_void;
        typedef char prog_char;
        typedef unsigned char prog_uchar;
        typedef int8_t prog_int8_t;
        typedef uint8_t prog_uint8_t;
        typedef int16_t prog_int16_t;
        typedef uint16_t prog_uint16_t;
        typedef int32_t prog_int32_t;
        typedef uint32_t prog_uint32_t;

        #define strcpy_P(dest, src) strcpy((dest), (src))
        #define strcat_P(dest, src) strcat((dest), (src))
        #define strcmp_P(a, b) strcmp((a), (b))

        #define pgm_read_byte(addr) (*( unsigned char *)(addr))
        #define pgm_read_word(addr) (*( unsigned short *)(addr))
        #define pgm_read_dword(addr) (*( unsigned long *)(addr))
        #define pgm_read_float(addr) (*( float *)(addr))

        #define pgm_read_byte_near(addr) pgm_read_byte(addr)
        #define pgm_read_word_near(addr) pgm_read_word(addr)
        #define pgm_read_dword_near(addr) pgm_read_dword(addr)
        #define pgm_read_float_near(addr) pgm_read_float(addr)
        #define pgm_read_byte_far(addr) pgm_read_byte(addr)
        #define pgm_read_word_far(addr) pgm_read_word(addr)
        #define pgm_read_dword_far(addr) pgm_read_dword(addr)
        #define pgm_read_float_far(addr) pgm_read_float(addr)
      #endif
    #endif

    /* Source is from the InvenSense MotionApps v2 demo code. Original source is
     * unavailable, unless you happen to be amazing as decompiling binary by
     * hand (in which case, please contact me, and I'm totally serious).
     *
     * Also, I'd like to offer many, many thanks to Noah Zerkin for all of the
     * DMP reverse-engineering he did to help make this bit of wizardry
     * possible.
     */

    // NOTE! Enabling DEBUG adds about 3.3kB to the flash program size.
    // Debug output is now working even on ATMega328P MCUs (e.g. Arduino Uno)
    // after moving string constants to flash memory storage using the F()
    // compiler macro (Arduino IDE 1.0+ required).

    //#define DEBUG
    #ifdef DEBUG
      #define DEBUG_PRINT(x) Serial.print(x)
      #define DEBUG_PRINTF(x, y) Serial.print(x, y)
      #define DEBUG_PRINTLN(x) Serial.println(x)
      #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
    #else
      #define DEBUG_PRINT(x)
      #define DEBUG_PRINTF(x, y)
      #define DEBUG_PRINTLN(x)
      #define DEBUG_PRINTLNF(x, y)
    #endif

    #define MPU6050_DMP_CODE_SIZE       1929    // dmpMemory[]
    #define MPU6050_DMP_CONFIG_SIZE     192     // dmpConfig[]
    #define MPU6050_DMP_UPDATES_SIZE    47      // dmpUpdates[]


    #ifndef MPU6050_DMP_FIFO_RATE_DIVISOR
      #define MPU6050_DMP_FIFO_RATE_DIVISOR 0x01
    #endif

    /* Exported function prototypes -----------------------------------------------*/
    #ifndef MPU6050_INCLUDE_DMP_MOTIONAPPS20
		  #define MPU6050_INCLUDE_DMP_MOTIONAPPS20
		#endif
    // special methods for MotionApps 2.0 implementation
    #ifdef MPU6050_INCLUDE_DMP_MOTIONAPPS20
//    uint8_t *dmpPacketBuffer;
//    uint16_t dmpPacketSize;

      uint8_t MPU6050_dmpInitialize(void);
      uint8_t MPU6050_dmpPacketAvailable(uint8_t *data);
      uint8_t MPU6050_dmpGetAccelI32(int32_t *data, uint8_t *packet);
      uint8_t MPU6050_dmpGetAccelI16(int16_t *data, uint8_t *packet);
      uint8_t MPU6050_dmpGetAccelVi16(VectorInt16 *v, uint8_t *packet);
      uint8_t MPU6050_dmpGetGyroI32(int32_t *data, uint8_t *packet);
      uint8_t MPU6050_dmpGetGyroI16(int16_t *data, uint8_t *packet);
      uint8_t MPU6050_dmpGetGyroVi16(VectorInt16 *v, uint8_t *packet);
      uint8_t MPU6050_dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
      uint8_t MPU6050_dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);

      uint8_t MPU6050_dmpGetGravityI16(int16_t *data, uint8_t *packet) ;
      void MPU6050_dmpGetGravityV_Float(VectorFloat *v, Quaternion *q);
      void MPU6050_dmpGetEuler(float *data, Quaternion *q);
      #ifdef USE_OLD_DMPGETYAWPITCHROLL
        void MPU6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
      #else
        void MPU6050_dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);
      #endif
      uint16_t MPU6050_dmpGetFIFOPacketSize(void);
      uint8_t MPU6050_dmpSetFIFORate(uint8_t fifoRate);
      uint8_t MPU6050_dmpGetFIFORate(uint8_t *data);
      uint8_t MPU6050_dmpGetSampleStepSizeMS(uint8_t *data);
      uint8_t MPU6050_dmpGetSampleFrequency(uint8_t *data);
      uint8_t MPU6050_dmpDecodeTemperature(int32_t tempReg);

      // Register callbacks after a packet of FIFO data is processed
      //uint8_t MPU6050_dmpRegisterFIFORateProcess(inv_obj_func func, int16_t priority);
      //uint8_t MPU6050_dmpUnregisterFIFORateProcess(inv_obj_func func);
      uint8_t MPU6050_dmpRunFIFORateProcesses(uint8_t *data);

      // Setup FIFO for various output
      uint8_t MPU6050_dmpSendQuaternion(uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendGyro(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendAccel(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendLinearAccel(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendLinearAccelInWorld(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendControlData(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendExternalSensorData(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendGravity(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendPacketNumber(uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendQuantizedAccel(uint_fast16_t elements, uint_fast16_t accuracy);
      uint8_t MPU6050_dmpSendEIS(uint_fast16_t elements, uint_fast16_t accuracy);

      // Get Fixed Point data from FIFO

      //uint8_t MPU6050_dmpGetMag(int16_t *data, uint8_t *packet); //MPU6050_INCLUDE_DMP_MOTIONAPPS41
      uint8_t MPU6050_dmpSetLinearAccelFilterCoefficient(float coef);

      uint8_t MPU6050_dmpGetExternalSensorData(int32_t *data, uint16_t size, uint8_t *packet);
      uint8_t MPU6050_dmpGetEIS(int32_t *data, uint8_t *packet);

      // Get Floating Point data from FIFO
      uint8_t MPU6050_dmpGetAccelFloat(float *data, uint8_t *packet);
      //uint8_t MPU6050_dmpGetQuaternionFloat(float *data, uint8_t *packet);
      uint8_t MPU6050_dmpGetQuaternionI32(int32_t *data, uint8_t *packet);
      uint8_t MPU6050_dmpGetQuaternionI16(int16_t *data, uint8_t *packet);
      uint8_t MPU6050_dmpGetQuaternionQ(Quaternion *q, uint8_t *packet);
      uint8_t MPU6050_dmpProcessFIFOPacket(uint8_t *dmpData);
      uint8_t MPU6050_dmpReadAndProcessFIFOPacket(uint8_t numPackets, uint8_t *processed);

      uint8_t MPU6050_dmpSetFIFOProcessedCallback(void (*func) (void));

      uint8_t MPU6050_dmpInitFIFOParam(uint8_t *data);
      uint8_t MPU6050_dmpCloseFIFO(uint8_t *data);
      uint8_t MPU6050_dmpSetGyroDataSource(uint8_t source);
      uint8_t MPU6050_dmpDecodeQuantizedAccel(uint8_t *data);
      uint32_t MPU6050_dmpGetGyroSumOfSquare(uint8_t *data);
      uint32_t MPU6050_dmpGetAccelSumOfSquare(uint8_t *data);
      void MPU6050_dmpOverrideQuaternion(long *q);

      // uint8_t MPU6050_dmpSetFIFOProcessedCallback(void (*func) (void));

    #endif //MPU6050_INCLUDE_DMP_MOTIONAPPS20
  #ifdef __cplusplus
    }
  #endif
#endif /* _MPU6050_6AXIS_MOTIONAPPS20_H_ */

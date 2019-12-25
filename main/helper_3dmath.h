// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-05 - add 3D math helper file to DMP6 example sketch

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

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_
  #ifdef __cplusplus
    extern "C" {
  #endif
      /* Private includes ----------------------------------------------------------*/
      #include "tracking_main.h"

      typedef struct {
           float w;
           float x;
           float y;
           float z;
      } Quaternion;
      typedef struct {
           uint8_t x;
           uint8_t y;
           uint8_t z;
      } VectorUint8;

      typedef struct {
           int8_t x;
           int8_t y;
           int8_t z;
      } VectorInt8;

      typedef struct {
           int16_t x;
           int16_t y;
           int16_t z;
      } VectorInt16;

      typedef struct {
           float x;
           float y;
           float z;
      } VectorFloat;

      /* Exported function prototypes -----------------------------------------------*/
      void QuaternionInit(Quaternion *q1);
      void QuaternionSet(Quaternion *q1, float nw, float nx, float ny, float nz);
      void getProductQ(Quaternion *q, Quaternion q1 ,Quaternion q2);
      void getConjugateQ(Quaternion *q, Quaternion q1);
      void getDegreeRezero(float *drain, float *source , float *mem , uint8_t number);
      void getConjugateQypr(uint8_t ypr);//Quaternion *q, Quaternion q1,
      float getMagnitudeQ(Quaternion *q1);
      void normalizeQ(Quaternion *q1);
      void getNormalizedQ(Quaternion *r, Quaternion *q1);
      void VectorInt16Init(VectorInt16 *v);
      void VectorInt16Set(VectorInt16 *v, int16_t nx, int16_t ny, int16_t nz);
      float getMagnitudeVi16(VectorInt16 *v);
      void normalizeVi16(VectorInt16 *v);
      void getNormalizedVi16(VectorInt16 *r, VectorInt16 *v);
      void rotateVi16(VectorInt16 *r, Quaternion *q);
      void getRotatedVi16(VectorInt16 *r, Quaternion *q);
      void VectorFloatInit(VectorFloat *v);
      void VectorFloatSet(VectorFloat *v, float nx, float ny, float nz);
      float getMagnitudeFloat(VectorFloat *v);
      void normalizeFloat(VectorFloat *v);
      void getNormalizedFloat(VectorFloat *r, VectorFloat *v);
      void rotateFloat(VectorFloat *r, Quaternion *q);
      void getRotatedFloat(VectorFloat *r, Quaternion *q);

  #ifdef __cplusplus
    }
  #endif
#endif /* _HELPER_3DMATH_H_ */

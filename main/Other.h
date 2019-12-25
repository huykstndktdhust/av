/**
  ******************************************************************************
  *   @file Other.h
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.6
  *   @date 12-30-2016
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OTHER_H
  #define __OTHER_H

  #ifdef __cplusplus
  extern "C" {
  #endif

  /* Includes ------------------------------------------------------------------*/
  #include "tracking_main.h"
  #include "ExternVariablesFunctions.h"

  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
  /* Exported function prototypes -----------------------------------------------*/

  int32_t My_Uint32_Abs(int32_t data);
  uint8_t SaveMemoryOffset(VectorInt16 aOffset, VectorInt16 gOffset);
  void ReadMemoryOffset(VectorInt16 *aOffset, VectorInt16 *gOffset);
  void ReadMemoryU16( char * name, u16 *data);
  uint8_t SaveMemoryU16( char * name, u16 data);
  void MPU6050_dmpDataReady(void);
  void IRAM_ATTR gpio_isr_handler(void* arg);
  void MPU6050_Setup(void);
  float abs_f(float data);

  void serialFloatX10PrintDegree(const char* tag, float f, uint8_t name);
  void serialFloatX1000PrintPI(const char* tag, float f, uint8_t name);
  void serialFloatX10Print360(const char* tag, float f, uint8_t name);

  void PrintStringI(const char* tag, char *b);
  void PrintStringE(const char* tag, char *b);

  void PrintI32(const char* tag, int32_t data);
  void PrintU32(const char* tag, uint32_t data);
  void PrintI16(const char* tag, int16_t data);
  void PrintU16Hex(const char* tag, uint16_t data);
  void PrintU16(const char* tag, uint16_t data);
  void PrintCharacter(const char* tag, char b);
  void PrintTeapot(const char* tag);

  void MX_MPU6050_I2C_Init_More(void);
  uint8_t MPU6050_dmpReadLoop(void);
  void MPU6050_dmpInitializeLoop(void);
  void MPU6050_Reset_Loop(void);
  void MPU6050_Init_Loop(void);
  void MPU6050_Connection_Loop(void);

#ifdef __cplusplus
}
#endif
#endif /*__OTHER_H */


/**
  * @}
  */ 

/**
  * @}
  */ 

/*****************************END OF FILE****/

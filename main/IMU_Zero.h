/**
  ******************************************************************************
  *   @file IMU_Zero.h
  *   @editor SangTN@fsoft.com.vn - FPT Company
  *   @version V1.3
  *   @date 12-30-2016
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __IMU_ZERO_H
  #define __IMU_ZERO_H

  #ifdef __cplusplus
  extern "C" {
  #endif

  /* Includes ------------------------------------------------------------------*/
  #include "tracking_main.h"

  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
  /* Exported function prototypes -----------------------------------------------*/
    void ForceHeader (void);
    void GetSmoothed (void);
    void SetOffsets (int32_t *TheOffsets);
    void ShowProgress (void);
    void PullBracketsIn (void);
    void PullBracketsOut (void);
    void SetAveraging (int16_t NewN);
    uint8_t SetupCalib (void);

#ifdef __cplusplus
}
#endif
#endif /*__IMU_ZERO_H */


/**
  * @}
  */ 

/**
  * @}
  */ 

/*****************************END OF FILE****/

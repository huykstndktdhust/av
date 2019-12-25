/**
  ******************************************************************************
  *   @file Delay.h
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.4
  *   @date 19-11-2014
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  *   @source
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
  #define __DELAY_H

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
      void TIM_Delay_Config(void);
      void delay_us( uint16_t us);
      void delay_ms( uint16_t ms);
  #ifdef __cplusplus
    }
  #endif
#endif /*__DELAY_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


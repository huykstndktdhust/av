/**
  ******************************************************************************
  *   @file Library_Mod.h
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.5
  *   @date 09-05-2015
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  *   @source
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIBRARY_MOD_H
  #define __LIBRARY_MOD_H

  #ifdef __cplusplus
   extern "C" {
  #endif

  /* Includes ------------------------------------------------------------------*/
  #include "tracking_main.h"

  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/

  #define I2C_ESP_ERROR_CHECK(x, y) do {                                      \
        x = (y);                                                        \
        if (x != ESP_OK) {                                       \
            _esp_error_check_failed(x, __FILE__, __LINE__,       \
                                    __ASSERT_FUNC, #y);                 \
        }                                                               \
    } while(0);

  /* Exported types ------------------------------------------------------------*/
  /* Exported function prototypes -----------------------------------------------*/
  void LL_IncTick(void);
  uint8_t I2C_MasterMemoryRead(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t* pBuffer);
  uint8_t I2C_MasterMemoryWrite(uint8_t slaveAddr, uint8_t regAddr, uint8_t length, uint8_t* pBuffer);
  esp_err_t i2c_master_init(void);

#ifdef __cplusplus
}
#endif
#endif /*__CONFIG_H */


/**
  * @}
  */ 

/**
  * @}
  */ 

/*****************************END OF FILE****/

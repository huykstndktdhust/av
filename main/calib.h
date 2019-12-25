/**
  ******************************************************************************
  *   @file calib.h
  *   @author LinhNT63@fsoft.com.vn - FPT Company
  *   @version V1.0
  *   @date 08-21-2019
  *   Final editors: LinhNT63@fsoft.com.vn
  *   @date:         08-21-2019
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CALIB_H
#define __CALIB_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "tracking_main.h"
#include "ExternVariablesFunctions.h"

  /* Exported types ------------------------------------------------------------*/
  /* Exported constants --------------------------------------------------------*/
  /* Exported macro ------------------------------------------------------------*/
  /* Exported functions ------------------------------------------------------- */
  /* Exported function prototypes -----------------------------------------------*/

  void calib(VectorInt16* acc_err, VectorInt16* gyr_err)
  {
      int c = 500;
      VectorInt16 acc;
      VectorInt16 gyr;
      int16_t tem;

      VectorInt16Init(&acc);
      VectorInt16Init(&gyr);
      VectorInt16Init(acc_err);
      VectorInt16Init(gyr_err);
      tem = 0;

      delay_ms(5);
      ESP_LOGI_OP("===== FIRST ===== :acc/gyr,",",%d,%d,%d,%d,%d,%d",
                      acc.x,
                      acc.y,
                      acc.z,
                      gyr.x,
                      gyr.y,
                      gyr.z
                      );
      ESP_LOGI_OP("===== FIRST ===== acc_err/gyr_err,",",%d,%d,%d,%d,%d,%d",
                      acc_err->x,
                      acc_err->y,
                      acc_err->z,
                      gyr_err->x,
                      gyr_err->y,
                      gyr_err->z
                      );  
      while (c > 0)
      {
            system_t.error = MPU6050_getMotion6_Mod(&acc, &gyr, &tem);
            ESP_LOGI_OP("acc/gyr,",",%d,%d,%d,%d,%d,%d",
                      acc.x,
                      acc.y,
                      acc.z,
                      gyr.x,
                      gyr.y,
                      gyr.z
                      );
            acc_err->x += (int16_t)(acc.x/500);
            acc_err->y += (int16_t)(acc.y/500);
            acc_err->z += (int16_t)(acc.z/500);
            gyr_err->x += (int16_t)(gyr.x/500);
            gyr_err->y += (int16_t)(gyr.y/500);
            gyr_err->z += (int16_t)(gyr.z/500);
            ESP_LOGI_OP("acc_err/gyr_err,",",%d,%d,%d,%d,%d,%d",
                      acc_err->x,
                      acc_err->y,
                      acc_err->z,
                      gyr_err->x,
                      gyr_err->y,
                      gyr_err->z
                      );
            delay_ms(20);
            c--;
      }

      ESP_LOGI_OP("===== LAST ===== :acc/gyr,",",%d,%d,%d,%d,%d,%d",
                      acc.x,
                      acc.y,
                      acc.z,
                      gyr.x,
                      gyr.y,
                      gyr.z
                      );
      ESP_LOGI_OP("===== LAST ===== acc_err/gyr_err,",",%d,%d,%d,%d,%d,%d",
                      acc_err->x,
                      acc_err->y,
                      acc_err->z,
                      gyr_err->x,
                      gyr_err->y,
                      gyr_err->z
                      );
  }
  
#ifdef __cplusplus
}
#endif
#endif /*__CALIB_H */

/**
  * @}
  */

/**
  * @}
  */

/*****************************END OF FILE****/

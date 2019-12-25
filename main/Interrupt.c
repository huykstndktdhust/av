/**
  ******************************************************************************
  *   @file Interrupt.c
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.5
  *   @date 27-03-2015
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Interrupt.h"
#include "ExternVariablesFunctions.h"

/* Exported functions ------------------------------------------------------- */
inline void TIMEOUT_UserCallback(void)
{
  for(uint8_t i = 0; i < 6; i++)
  {
    delay_ms(50);
    LED_taskMPU6050 ^= 1;
    if(system_t.Reset != 0)
    {
        esp_restart();
    }
  }
  LED_taskMPU6050 = 1;

  for(uint8_t i = 0; i < 15; i++)
  {
    delay_ms(50);
    if(system_t.Reset != 0)
    {
        esp_restart();
    }
  }
}

/*****************************END OF FILE****/

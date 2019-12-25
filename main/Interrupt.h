/**
  ******************************************************************************
  *   @file Interrupt.h
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.5
  *   @date 27-03-2015
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  *   @source
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __INTERRUPT_H
#define __INTERRUPT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "tracking_main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

typedef struct
{
     uint16_t Timer;
     uint16_t Sample10;
     uint16_t Computer;
     uint16_t ComputerMax;
} COUNT_VALUE;

#define COUNT_VALUE_DEFAULTS \
{                            \
                           0,\
                           0,\
                           0,\
                           0,\
}

typedef struct
{
   uint8_t Reset;
//   uint8_t Turning;
//   uint8_t SelectModeInput;
   uint8_t SelectInitOnline;

   uint32_t Tick;
   uint32_t Tick_Main0;
   uint32_t Tick_Main1;
   uint32_t Tick_Printf;
   uint8_t Start;

   uint16_t CountReady;
   uint32_t SetCountLed;
   uint8_t ready;
   float last_val;
   uint8_t error;
   uint8_t ModeOutput;
} SYSTEM_VALUE;

#define SYSTEM_VALUE_DEFAULTS  \
{                              \
                            0, \
                               \
                               \
                            0, \
                               \
                            0, \
                            0, \
                            0, \
                            0, \
                         true, \
                               \
                            0, \
                          100, \
                        false, \
                         0.0f, \
                            0, \
                            0, \
}
typedef struct
{
   uint8_t FlagError;
}ERROR_VALUE;

#define ERROR_VALUE_DEFAULTS  \
{                              \
                            0, \
}
typedef struct
{
   uint16_t  Input;         // Input Signal
   int16_t  VTcount;        // Parameter: upper count
   int16_t  VTPcount;       // Parameter: upper count
   int16_t  VTNcount;       // Parameter: lower count
   int16_t  VTmax;          // Parameter: maximum count
   int16_t  VTmin;          // Parameter: minimum count
   uint8_t  Inverse;        // Parameter: Inverse Signal Output:
   uint8_t  Output;         // Output Signal:
} INPUT_FILTER;

/*~2000ms time sample 10ms*/
#define INPUT_FILTER_DEFAULTS \
{                             \
                           0, \
                           0, \
                           90, \
                           -90, \
                           50, \
                           -50, \
                           0, \
                           1, \
}

#define DIGITAL_INTEGRAL_FILTER(v)                     \
  if(v.Input != 0) {                                   \
    if(v.VTcount < v.VTmax)v.VTcount++;                \
  }                                                    \
  else {                                               \
    if(v.VTcount > v.VTmin)v.VTcount--;                \
  }                                                    \
  if(v.VTcount > v.VTPcount)v.Output = (1^v.Inverse);      \
  else if(v.VTcount < v.VTNcount)v.Output = (0^v.Inverse); \

typedef struct
{
   uint16_t  Input;         // Input Signal
   int16_t  VTcount;        // Parameter: upper count
   int16_t  VTmax;          // Parameter: maximum count
   int16_t  VTmin;          // Parameter: minimum count
   uint8_t  Inverse;        // Parameter: Inverse Signal Output:
   uint8_t  Output;         // Output Signal:
} INPUT_FILTER_V2;

/*~1000ms time sample 10ms --> 100*/
#define INPUT_FILTER_DEFAULTS_V2 \
{                                \
                              0, \
                              0, \
                              90, \
                              -90, \
                              50, \
                              -50, \
                              0, \
                              1, \
}

#define DIGITAL_INTEGRAL_FILTER_V2(v)   \
  if(v.Input != 0) {                    \
    if(v.VTcount < v.VTmax)v.VTcount++; \
    else v.Output = (1^v.Inverse);      \
  }                                     \
  else {                                \
    if(v.VTcount > v.VTmin)v.VTcount--; \
    else v.Output = (0^v.Inverse);      \
  }                                     \

  typedef struct
  {
     uint16_t  Input;         // Input Signal
     uint16_t  VTcount1;        // Parameter: upper count
     uint16_t  VTcount0;        // Parameter: upper count
     uint16_t  VTmax;          // Parameter: maximum count
     uint8_t  Inverse;        // Parameter: Inverse Signal Output:
     uint8_t  Output;         // Output Signal:
  } INPUT_FILTER_V3;
  /*~250ms time sample 10ms*/
  #define INPUT_FILTER_DEFAULTS_V3 \
  {                                \
                                0, \
                                0, \
                                0, \
                                25, \
                                0, \
                                1, \
  }
#define DIGITAL_INTEGRAL_FILTER_V3(v)   \
  if(v.Input != 0) {                    \
    if(v.VTcount1 < v.VTmax){v.VTcount1++; v.VTcount0 = 0;} \
    else v.Output = (1^v.Inverse);      \
  }                                     \
  else {                                \
    if(v.VTcount0 < v.VTmax){v.VTcount0++; v.VTcount1 = 0;} \
    else v.Output = (0^v.Inverse);      \
  }                                     \

    typedef struct
    {
         uint8_t StateOld;
         uint8_t StateNew;
         uint8_t EdgeFlag;
    } EDGE_TRIGGER;

    #define EDGE_TRIGGER_DEFAULTS  \
    {                              \
                                1, \
                                1, \
                                0, \
    }

    #define FALLING_EDGE_TRIGGER_DETECTION(v)                 \
    if((v.StateOld == 1) && (v.StateNew == 0))                \
    {                                                         \
        v.EdgeFlag = 1;                                       \
    }                                                         \
    v.StateOld = v.StateNew;       /*save State*/   \


    #define RISING_EDGE_TRIGGER_DETECTION(v)                  \
    if((v.StateOld == 0) && (v.StateNew == 1))                \
    {                                                         \
        v.EdgeFlag = 1;                                       \
    }                                                         \
    v.StateOld = v.StateNew;                /*save State*/    \

//  //    if(StateButtonOld != StateButtonNew)
//  //    {
//  //        StateButtonOld[Set] = StateButtonNew[Set]; //save State
//  //    }



#define GPIO_Pin_0                 ((uint16_t)0x0001)  /*!< Pin 0 selected */
#define GPIO_Pin_1                 ((uint16_t)0x0002)  /*!< Pin 1 selected */
#define GPIO_Pin_2                 ((uint16_t)0x0004)  /*!< Pin 2 selected */
#define GPIO_Pin_3                 ((uint16_t)0x0008)  /*!< Pin 3 selected */
#define GPIO_Pin_4                 ((uint16_t)0x0010)  /*!< Pin 4 selected */
#define GPIO_Pin_5                 ((uint16_t)0x0020)  /*!< Pin 5 selected */
#define GPIO_Pin_6                 ((uint16_t)0x0040)  /*!< Pin 6 selected */
#define GPIO_Pin_7                 ((uint16_t)0x0080)  /*!< Pin 7 selected */
#define GPIO_Pin_8                 ((uint16_t)0x0100)  /*!< Pin 8 selected */
#define GPIO_Pin_9                 ((uint16_t)0x0200)  /*!< Pin 9 selected */
#define GPIO_Pin_10                ((uint16_t)0x0400)  /*!< Pin 10 selected */
#define GPIO_Pin_11                ((uint16_t)0x0800)  /*!< Pin 11 selected */
#define GPIO_Pin_12                ((uint16_t)0x1000)  /*!< Pin 12 selected */
#define GPIO_Pin_13                ((uint16_t)0x2000)  /*!< Pin 13 selected */
#define GPIO_Pin_14                ((uint16_t)0x4000)  /*!< Pin 14 selected */
#define GPIO_Pin_15                ((uint16_t)0x8000)  /*!< Pin 15 selected */
#define GPIO_Pin_All               ((uint16_t)0xFFFF)  /*!< All pins selected */

/* Exported functions ------------------------------------------------------- */
/* Private function prototypes -----------------------------------------------*/
void TIMEOUT_UserCallback(void);
#ifdef __cplusplus
}
#endif
#endif /*__INTERRUPT_H */

/**
  * @}
  */

/**
  * @}
  */

/*****************************END OF FILE****/

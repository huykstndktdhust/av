/**
  ******************************************************************************
  *   @file ExternVariablesFunctions.h
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.0
  *   @date 05-09-2017
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  *   @source
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXTERNVARIABLESFUNCTIONS_H
  #define __EXTERNVARIABLESFUNCTIONS_H

  #ifdef __cplusplus
    extern "C" {
  #endif
    /* Includes ------------------------------------------------------------------*/

//    #include "tracking_main.h"

    #include "driver/gpio.h"
    #include "driver/uart.h"
    #include "driver/adc.h"
    #include "driver/i2c.h"
    #include "driver/ledc.h"
    #include "driver/periph_ctrl.h"
    #include "driver/timer.h"

    #include "esp_bt.h"
    #include "esp_bt_defs.h"
    #include "esp_bt_main.h"
    #include "esp_err.h"
    #include "esp_gap_ble_api.h"
    #include "esp_gatts_api.h"
    #include "esp_log.h"
    #include "esp_system.h"
    #include "esp_types.h"

    #include "freertos/FreeRTOS.h"
    #include "freertos/FreeRTOSConfig.h"
    #include "freertos/task.h"
    #include "freertos/queue.h"
    #include "freertos/event_groups.h"
    #include "freertos/semphr.h"

    #include "nvs_flash.h"

    #include "sdkconfig.h"

    #include "soc/uart_struct.h"
    #include "soc/timer_group_struct.h"


    #include "esplog_task_uart0.h"
    #include "gatt_s.h"
    #include "logger.h"
    #include "tracking_timer.h"
    #include "tracking_adc.h"
    #include "variables.h"

    //#include "Config.h"
    #include "Delay.h"
    #include "GenericTypeDefs.h"
    #include "helper_3dmath.h"
    #include "IMU_Zero.h"
    #include "Interrupt.h"
    #include "I2Cdev.h"
    #include "Library_Mod.h"
    #include "MPU6050.h"
    #include "MPU6050_6Axis_MotionApps20.h"
    #include "Other.h"

    /* Extern types ------------------------------------------------------------*/
    /* Extern constants --------------------------------------------------------*/
    /* Extern macro ------------------------------------------------------------*/
    /* =========================================================================
       NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
       when using Serial.write(buf, len). The Teapot output uses this method.
       The solution requires a modification to the Arduino USBAPI.h file, which
       is fortunately simple, but annoying. This will be fixed in the next IDE
       release. For more info, see these links:

       http://arduino.cc/forum/index.php/topic,109987.0.html
       http://code.google.com/p/arduino/issues/detail?id=958
     * ========================================================================= */

    // uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
    // quaternion components in a [w, x, y, z] format (not best for parsing
    // on a remote host such as Processing or something though)
    #define OUTPUT_READABLE_QUATERNION

    // uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
    // (in degrees) calculated from the quaternions coming from the FIFO.
    // Note that Euler angles suffer from gimbal lock (for more info, see
    // http://en.wikipedia.org/wiki/Gimbal_lock)
    //#define OUTPUT_READABLE_EULER

    // uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
    // pitch/roll angles (in degrees) calculated from the quaternions coming
    // from the FIFO. Note this also requires gravity vector calculations.
    // Also note that yaw/pitch/roll angles suffer from gimbal lock (for
    // more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
    #define OUTPUT_READABLE_YAWPITCHROLL

    // uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
    // components with gravity removed. This acceleration reference frame is
    // not compensated for orientation, so +X is always +X according to the
    // sensor, just without the effects of gravity. If you want acceleration
    // compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
    //#define OUTPUT_READABLE_REALACCEL

    // uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
    // components with gravity removed and adjusted for the world frame of
    // reference (yaw is relative to initial orientation, since no magnetometer
    // is present in this case). Could be quite handy in some cases.
    //#define OUTPUT_READABLE_WORLDACCEL

    // uncomment "OUTPUT_TEAPOT" if you want output that matches the
    // format used for the InvenSense teapot demo
    //#define OUTPUT_TEAPOT

    // uncomment "USE_INT_PIN" if you want use interrupt pin
    // #define USE_INT_PIN
    // uncomment "USE_SET_HOME" if you want use set home with uart
    #define USE_SET_HOME
    // uncomment "USE_PRINT_X10" if you want use set print float * 10 with uart
    #define USE_PRINT_X10

    #define USE_SET_OFFSET

    #define USE_NOT_SYSTEM_TICK

    //#define WRITE_PROTECTION_ENABLE
    #define USER_TIME_OUT 1000 //20 count a day x30 date x3 month =1800

    /*
     * @brief  type conversion definition
     */
    #define U8_T(x)  (uint8_t)x
    #define U8_P(x)  (uint8_t *)x
    #define U16_T(x) (uint16_t)x
    #define U16_P(x) (uint16_t *)x
    #define U32_T(x) (uint32_t)x
    #define U32_P(x) (uint32_t *)x
    #define I8_T(x)  (int8_t)x
    #define I8_P(x)  (int8_t *)x
    #define I16_T(x) (int16_t)x
    #define I16_P(x) (int16_t *)x
    #define I32_T(x) (int32_t)x
    #define I32_P(x) (int32_t *)x
    #define F32_T(x) (float)x
    #define F32_P(x) (float *)x
    #define D64_T(x) (double)x
    #define D64_P(x) (double *)x

    /*
    * @brief  LowLayer Status structures definition
    */
    typedef enum
    {
     LL_OK       = 0x00U,
     LL_ERROR    = 0x01U,
     LL_BUSY     = 0x02U,
     LL_TIMEOUT  = 0x03U,
     LL_OVER     = 0x04U,
     LL_VERIFY_FAILED = 0x05U
    } LL_StatusTypeDef;

    /** @addtogroup Exported_types
     * @{
     */
    typedef enum
    {
     RESET = 0,
     SET = !RESET
    } FlagStatus, ITStatus;

    typedef enum
    {
     DISABLE = 0,
     ENABLE = !DISABLE
    } FunctionalState;
    #define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

    typedef enum
    {
     ERROR = 0,
     SUCCESS = !ERROR
    } ErrorStatus;

    /** @addtogroup Exported_macros
     * @{
     */
    #define SET_BIT(REG, BIT)     ((REG) |= (BIT))

    #define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))

    #define READ_BIT(REG, BIT)    ((REG) & (BIT))

    #define CLEAR_REG(REG)        ((REG) = (0x0))

    #define WRITE_REG(REG, VAL)   ((REG) = (VAL))

    #define READ_REG(REG)         ((REG))

    #define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

    #define POSITION_VAL(VAL)     (__CLZ(__RBIT(VAL)))

    typedef struct
    {
        float psi;
        float theta;
        float phi;
    } EULER;

    typedef struct
    {
        float yaw;
        float pitch;
        float roll;
    } YPR;

    //  extern constfloat CONST_CURRENT;
    //  extern constfloat CONST_VOLT_BUS;
    //  extern constfloat CONST_100PER93;
    //  extern constfloat CONST_93PER100;
    //  extern constfloat pi;
    //  extern constfloat pi2;
    #define CONST_CURRENT 3.678501986E-3
    #define CONST_VOLT_BUS 22.38595145E-3
    #define CONST_100PER93 1.0752688172f  //100/93.5f
    #define CONST_93PER100 0.93f  //93.5f/100
    #ifndef PI
      #define PI  M_PI //3.14159265358979f
    #endif
    #ifndef PI2
      #define PI2 (2*PI)  //6.28318530717958f  //
    #endif
    #define UART_RX_BUF_SIZE  6
    #define UART_TX_BUF_SIZE  0xFF
    #define I2C_RX_BUF_SIZE  0xFFF
    #define I2C_TX_BUF_SIZE  0xFFF
    #define RAD_TO_DEGREES      (180.0/PI)  //57.2957795f //
    #define DEGREES_TO_RAD      (PI/180.0)  //0.01745329f //

    #define _BV(bit) (1U << bit)

//------------------------------------------------------------------------------//

    extern  uint32_t MemoryToggle;
    extern  u8 LED_GREEN_CALIB;
    extern  u8 MODE_0_GPIO;
    extern  u8 MODE_1_GPIO;
    extern  u8 INT_MPU_GPIO;
    extern  u8 BUTTON_USER0;
    extern  u8 BUTTON_USER1;
    extern  u8 GPIO_CHARGE_CONTROL;
    extern  u8 LED_ADAPTOR;
    extern  u8 LED_RED_BLE_ON;
    extern  u8 LED_BLUE_BLE_NOTIFY;

    extern  u8 Pause_taskMPU6050;
    extern  u8 LED_taskMPU6050;
    // Output
//    extern void LedDebug(uint32_t level);
//    extern void LedDebugToggle(void);
//------------------------------------------------------------------------------//

    #define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
    #define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

    #define I2C_MASTER_SDA_IO 21
    #define I2C_MASTER_SCL_IO 22
    #define I2C_NUM I2C_NUM_0
    #define INT_MPU_IO 37

    #define ACK_CHECK_EN   0x1          /*!< I2C master will check ack from slave*/
    #define ACK_CHECK_DIS  0x0          /*!< I2C master will not check ack from slave */
    #define ACK_VAL    0x0              /*!< I2C ack value */
    #define NACK_VAL   0x1              /*!< I2C nack value */
//------------------------------------------------------------------------------//
    #define PRINT_MUL_10 0
    #define PRINT_FLOAT_TO_4_BYTE 1
    #define PRINT_FLOAT_TO_STRING 2
    #define PRINT_TEAPOT 3

    extern  uint32_t Address;
    typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
    #ifdef WRITE_PROTECTION_ENABLE
        extern FLASH_OBProgramInitTypeDef OBInit;
    #endif
    #ifdef USER_TIME_OUT
        extern  uint16_t g_u16DataTimeOut;
        extern  uint16_t g_u16DataTimeOutCheck;
        extern  TestStatus MemoryProgramStatus;
    #endif

    /* Extern variables Delay.c---------------------------------------------------------*/
    extern  uint64_t CountTimerG1_0;
    /* Extern variables main.c---------------------------------------------------------*/

    extern ERROR_VALUE err_system_t;
    extern SYSTEM_VALUE system_t;

    extern  uint8_t LBRACKET;
    extern  uint8_t RBRACKET;
    extern  uint8_t COMMA;
    extern  uint8_t BLANK;
    extern  uint8_t PERIOD;

    extern  uint8_t iAx;
    extern  uint8_t iAy;
    extern  uint8_t iAz;
    extern  uint8_t iGx;
    extern  uint8_t iGy;
    extern  uint8_t iGz;

    extern  uint16_t usDelay;   // empirical, to hold sampling to 200 Hz
    extern  int16_t NFast;    // the bigger, the better (but slower)
    extern  int16_t NSlow;    // ..
    extern  uint8_t LinesBetweenHeaders;
    extern int16_t LowValue[];
    extern int16_t HighValue[];
    extern int16_t Smoothed[];
    extern int32_t LowOffset[];
    extern int32_t HighOffset[];
    extern int16_t Target[];
    extern  uint8_t LinesOut;
    extern  int16_t N;
    // MPU control/status vars
    extern  uint8_t _ready;  // set true if DMP init was successful
    extern  uint8_t _home;

    extern  uint8_t dmpReady;  // set true if DMP init was successful
    extern  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    extern  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    extern  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    extern  uint16_t fifoCount;     // count of all bytes currently in FIFO
    extern uint8_t fifoBuffer[]; // FIFO storage buffer
    // orientation/motion vars
    extern Quaternion q, hq, tmp;           // [w, x, y, z]         quaternion container
    extern VectorFloat gravity;    // [x, y, z]            gravity vector
    extern VectorInt16 aaReal, aa, aaWorld;         //real acceleration , acceleration and world-frame acceleration

    extern VectorInt8 gOffsetTC;
    extern VectorInt16 gOffset;
    extern VectorUint8 aOffsetTC;
    extern VectorInt16 aOffset;

    extern EULER euler;            // [psi, theta, phi]    Euler angle container
    extern YPR container_gravity;  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    extern YPR degree;             // [yaw, pitch, roll]   yaw/pitch/roll degree
    extern YPR degree_rezero;
    extern YPR degree_mem;
    // packet structure for InvenSense teapot demo
    extern uint8_t teapotPacket[];

    /* Buffer used for transmission */
    extern uint8_t UartTxBuffer[];

    extern INPUT_FILTER_V3 Sw0;
    extern EDGE_TRIGGER EdgeSw0;
    extern INPUT_FILTER_V3 Sw1;
    extern EDGE_TRIGGER EdgeSw1;
    extern INPUT_FILTER_V3 M0;
    extern EDGE_TRIGGER EdgeM0;

    extern  uint8_t ModeOutput;
    extern  uint32_t  MPU_Timeout;
    // MotionApps 2.0 DMP implementation, built using the MPU-6050EVB evaluation board
    // #define MPU6050_INCLUDE_DMP_MOTIONAPPS20 move to project

    /* Extern variables MPU6050.c---------------------------------------------------------*/
    extern  uint8_t *dmpPacketBuffer;
    extern  uint16_t dmpPacketSize;

    // ================================================================
    // ===               INTERRUPT DETECTION ROUTINE                ===
    // ================================================================

    extern  uint8_t mpuInterrupt;     // indicates whether MPU interrupt pin has gone high

    /*==================================================================================================
    *                                      VARIABLES
    ==================================================================================================*/
    extern _notify2app notify2app_gstr;
    extern _motion_tracking_flag motion_tracking_flag_gstr;
    extern _tracking_variables tracking_variables_gtr;
    extern _receive_from_app receive_from_app_gtr;
    extern Quaternion qtn;
    extern VectorInt16 err_acc, err_gyr;

  #ifdef __cplusplus
    }
  #endif
#endif /*__ExternVariablesFunctions_H */


/**
  * @}
  */

/**
  * @}
  */

/*****************************END OF FILE****/

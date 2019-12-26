/**
*   @file name     tracking_main.c
*   Project:       Motion_Tracking
*   @Purpose       main source file
*   Author:        TaiVV2@fsoft.com.vn
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/

/* I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
 Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

 Changelog:
      2016-04-18 - Eliminated a potential infinite loop
      2013-05-08 - added seamless Fastwire support
                 - added note about gyro calibration
      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
      2012-06-20 - improved FIFO overflow handling and simplified read process
      2012-06-19 - completely rearranged DMP initialization code and simplification
      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
      2012-06-05 - add gravity-compensated initial reference frame acceleration output
                 - add 3D math helper file to DMP6 example sketch
                 - add Euler output and Yaw/Pitch/Roll output formats
      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
      2012-05-30 - basic DMP initialization working
*/
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

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "tracking_main.h"
#include "ExternVariablesFunctions.h"
#include "MahonyAHRS.h"

#define MOTION_TRACKING
/*==================================================================================================
*                                      VARIABLES
==================================================================================================*/
_tracking_variables tracking_variables_gtr;
_motion_tracking_flag motion_tracking_flag_gstr;
_receive_from_app receive_from_app_gtr;
VectorInt16 err_gyr;
VectorInt16 err_acc;
_notify2app notify2app_gstr;
Quaternion qtn;
static const char *MAIN_TAG = "main";
ERROR_VALUE err_system_t = ERROR_VALUE_DEFAULTS;
SYSTEM_VALUE system_t = SYSTEM_VALUE_DEFAULTS;

uint32_t Address = 0x00;
#ifdef WRITE_PROTECTION_ENABLE
FLASH_OBProgramInitTypeDef OBInit;
#endif
#ifdef USER_TIME_OUT
uint16_t g_u16DataTimeOut = 0;
uint16_t g_u16DataTimeOutCheck = 0;
TestStatus MemoryProgramStatus = PASSED;
#endif

uint8_t LBRACKET = '[';
uint8_t RBRACKET = ']';
uint8_t COMMA = ',';
uint8_t BLANK = ' ';
uint8_t PERIOD = '.';

uint8_t iAx = 0;
uint8_t iAy = 1;
uint8_t iAz = 2;
uint8_t iGx = 3;
uint8_t iGy = 4;
uint8_t iGz = 5;

uint16_t usDelay = 5000; // empirical, to hold sampling to 200 Hz//3150
int16_t NFast = 200;     // the bigger, the better (but slower)//1000
int16_t NSlow = 1000;    // ..//10000
uint8_t LinesBetweenHeaders = 5;
int16_t LowValue[6];
int16_t HighValue[6];
int16_t Smoothed[6];
int32_t LowOffset[6];
int32_t HighOffset[6];
int16_t Target[6];
uint8_t LinesOut;
int16_t N;
// MPU control/status vars
uint8_t _ready = false; // set true if DMP init was successful
uint8_t _home = false;

uint8_t dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU
uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;      // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer

// orientation/motion vars
Quaternion q, hq, tmp; // [w, x, y, z]         quaternion container
VectorFloat gravity;   // [x, y, z]            gravity vector

VectorInt16 aaReal, aa, aaWorld; //real acceleration , acceleration and world-frame acceleration

VectorInt8 gOffsetTC;
VectorInt16 gOffset;
VectorUint8 aOffsetTC;
VectorInt16 aOffset;

EULER euler;           // [psi, theta, phi]    Euler angle container
YPR container_gravity; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
YPR degree;            // [yaw, pitch, roll]   yaw/pitch/roll degree
YPR degree_rezero;
YPR degree_mem;
// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = {'$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n'};

/* Buffer used for transmission */
uint8_t UartTxBuffer[UART_TX_BUF_SIZE];

INPUT_FILTER_V3 Sw0 = INPUT_FILTER_DEFAULTS_V3;
EDGE_TRIGGER EdgeSw0 = EDGE_TRIGGER_DEFAULTS;
INPUT_FILTER_V3 Sw1 = INPUT_FILTER_DEFAULTS_V3;
EDGE_TRIGGER EdgeSw1 = EDGE_TRIGGER_DEFAULTS;
INPUT_FILTER_V3 M0 = INPUT_FILTER_DEFAULTS_V3;
EDGE_TRIGGER EdgeM0 = EDGE_TRIGGER_DEFAULTS;

//INPUT_FILTER_V3 M1 = INPUT_FILTER_DEFAULTS_V3;
//EDGE_TRIGGER EdgeM1 = EDGE_TRIGGER_DEFAULTS;

VectorInt16 aReal, gReal;
int16_t tReal;

static char *TAG = "app_main.c";

TaskHandle_t handle_taskMPU6050 = NULL;

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
static void process_data_from_app(void);

/*================================================================================================*/
/**
* @function name  task_create
* @brief          using for create a task
*                 if the creation is OK, a started_announcement will display (log)
*                 if the creation is NG, a error announcement will display (log)
* @parameter      TaskFunction_t pvTaskCode, const char * const pcName, const uint32_t usStackDepth,
                  void * const pvParameters, UBaseType_t uxPriority, TaskHandle_t * const pvCreatedTask,
                  const BaseType_t xCoreID
* @return value   none
* @note           none
*/
static void task_create(TaskFunction_t pvTaskCode, const char *const pcName, const uint32_t usStackDepth,
                        void *const pvParameters, UBaseType_t uxPriority, TaskHandle_t *const pvCreatedTask,
                        const BaseType_t xCoreID)
{
  BaseType_t ret;

  ret = xTaskCreatePinnedToCore(pvTaskCode, pcName, usStackDepth, pvParameters, uxPriority,
                                pvCreatedTask, xCoreID);
  if (pdPASS == ret)
  {
    ESP_LOGI_OP(TAG, "%s started", pcName);
  }
  else
  {
    ESP_LOGE_OP(TAG, "%s return error: %d", pcName, ret);
  }
}

/*================================================================================================*/
/**
* @function name  myTaskDelete
* @brief          Delete task
* @parameter      xHandle
* @return value   none
* @note           none
*/
void myTaskDelete(TaskHandle_t xHandle)
{
  TaskHandle_t xTask = xHandle;

  vTaskSuspendAll();

  if (NULL != xHandle)
  {
    xHandle = NULL;
    vTaskDelete(xTask);
  }

  xTaskResumeAll();
}
uint32_t MemoryToggle = 1;
u8 LED_GREEN_CALIB = GPIO_NUM_27;
u8 MODE_0_GPIO = GPIO_NUM_36;
u8 MODE_1_GPIO = GPIO_NUM_39;
u8 INT_MPU_GPIO = GPIO_NUM_23;
u8 BUTTON_USER0 = GPIO_NUM_19;
u8 BUTTON_USER1 = GPIO_NUM_18;
u8 GPIO_CHARGE_CONTROL = GPIO_NUM_5;
u8 LED_ADAPTOR = GPIO_NUM_33;

u8 LED_RED_BLE_ON = GPIO_NUM_25;
u8 LED_BLUE_BLE_NOTIFY = GPIO_NUM_26;
u8 Pause_taskMPU6050 = 0;
u8 LED_taskMPU6050 = 1;
void taskMPU6050_Suspend(void)
{
  if (handle_taskMPU6050 != NULL)
  {
    //vTaskSuspend(handle_taskMPU6050);
    Pause_taskMPU6050 = 1;
  }
}
void taskMPU6050_Resume(void)
{
  if (handle_taskMPU6050 != NULL)
  {
    //vTaskResume(handle_taskMPU6050);
    Pause_taskMPU6050 = 0;
  }
}

void calib(VectorInt16 *acc_err, VectorInt16 *gyr_err)
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
  ESP_LOGI_OP("===== FIRST ===== :acc/gyr,", ",%d,%d,%d,%d,%d,%d",
              acc.x,
              acc.y,
              acc.z,
              gyr.x,
              gyr.y,
              gyr.z);
  ESP_LOGI_OP("===== FIRST ===== acc_err/gyr_err,", ",%d,%d,%d,%d,%d,%d",
              acc_err->x,
              acc_err->y,
              acc_err->z,
              gyr_err->x,
              gyr_err->y,
              gyr_err->z);
  while (c > 0)
  {
    system_t.error = MPU6050_getMotion6_Mod(&acc, &gyr, &tem);
    ESP_LOGI_OP("acc/gyr,", ",%d,%d,%d,%d,%d,%d",
                acc.x,
                acc.y,
                acc.z,
                gyr.x,
                gyr.y,
                gyr.z);
    acc_err->x += (int16_t)(acc.x / 500);
    acc_err->y += (int16_t)(acc.y / 500);
    acc_err->z += (int16_t)(acc.z / 500);
    gyr_err->x += (int16_t)(gyr.x / 500);
    gyr_err->y += (int16_t)(gyr.y / 500);
    gyr_err->z += (int16_t)(gyr.z / 500);
    ESP_LOGI_OP("acc_err/gyr_err,", ",%d,%d,%d,%d,%d,%d",
                acc_err->x,
                acc_err->y,
                acc_err->z,
                gyr_err->x,
                gyr_err->y,
                gyr_err->z);
    delay_ms(20);
    c--;
  }

  ESP_LOGI_OP("===== LAST ===== :acc/gyr,", ",%d,%d,%d,%d,%d,%d",
              acc.x,
              acc.y,
              acc.z,
              gyr.x,
              gyr.y,
              gyr.z);
  ESP_LOGI_OP("===== LAST ===== acc_err/gyr_err,", ",%d,%d,%d,%d,%d,%d",
              acc_err->x,
              acc_err->y,
              acc_err->z,
              gyr_err->x,
              gyr_err->y,
              gyr_err->z);
}

void taskMPU6050(void *param)
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  ESP_ERROR_CHECK(i2c_master_init());

  MPU6050_Setup();

  calib(&err_acc, &err_gyr);

  for (;;)
  {
    delay_ms(5); //Perform an action every 5 ticks //200Hz

    //  Best result is to match with DMP refresh rate
    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 310
    // Now its 0x13, which means DMP is refreshed with 10Hz rate
    system_t.Tick_Main0++;
    if (system_t.Tick_Main0 >= 20)
    {
      system_t.Tick_Main0 = 0;
      MPU6050_dmpReadLoop();
    }

    system_t.error = MPU6050_getMotion6_Mod(&aReal, &gReal, &tReal);

    aReal.x -= err_acc.x;
    aReal.y -= err_acc.y;
    aReal.z -= err_acc.z;
    gReal.x -= err_gyr.x;
    gReal.y -= err_gyr.y;
    gReal.z -= err_gyr.z;

    notify2app_gstr.data.Accelerator_X = aReal.x;
    notify2app_gstr.data.Accelerator_Y = aReal.y;
    notify2app_gstr.data.Accelerator_Z = aReal.z;
    notify2app_gstr.data.Temperature = tReal;

    notify2app_gstr.data.Gyro_X = gReal.x;
    notify2app_gstr.data.Gyro_Y = gReal.y;
    notify2app_gstr.data.Gyro_Z = gReal.z;

    //LOG IT!!!!
    ESP_LOGI_OP(",", ",%d,%d,%d,%d,%d,%d,%d,%d",
                notify2app_gstr.data.timestamp,
                notify2app_gstr.data.Accelerator_X,
                notify2app_gstr.data.Accelerator_Y,
                notify2app_gstr.data.Accelerator_Z,
                notify2app_gstr.data.Gyro_X,
                notify2app_gstr.data.Gyro_Y,
                notify2app_gstr.data.Gyro_Z,
                notify2app_gstr.data.Temperature);

    MahonyAHRSupdateIMU(&qtn, gReal.x, gReal.y, gReal.z, aReal.x, aReal.y, aReal.z);

    motion_tracking_flag_gstr.MPU_notify2app = true;

    if ((tracking_variables_gtr.state_machine == CONFIG_STATE) || (EdgeSw1.EdgeFlag != 0))
    {
      EdgeSw1.EdgeFlag = 0;
      SetupCalib();
      //            system_t.Reset = 1;
      if (tracking_variables_gtr.state_machine == CONFIG_STATE)
        tracking_variables_gtr.state_machine = WORKING_STATE;
    }

    if (system_t.Reset != 0)
    {
      system_t.Reset = 0;
      esp_restart();
    }

    //        if((M0.Output == 0)&&(M1.Output == 0)) {
    //            system_t.ModeOutput = 'z';
    //        }
    //        else if((M0.Output == 0)&&(M1.Output == 1)) {
    //            system_t.ModeOutput = 'p';
    //        }
    //        else if((M0.Output == 1)&&(M1.Output == 0)) {
    //            system_t.ModeOutput = 'r';
    //        }
    //        else if((M0.Output == 1)&&(M1.Output == 1)) {
    //            system_t.ModeOutput = 't';//true real
    //        }

    if (M0.Output == 0)
    {
      system_t.ModeOutput = 'z';
    }
    else if (M0.Output == 1)
    {
      system_t.ModeOutput = 'r';
    }
    system_t.Tick_Main1++;
    if (err_system_t.FlagError != 0)
    {
      TIMEOUT_UserCallback();
    }
    else
    {
      if (system_t.Tick_Main1 >= system_t.SetCountLed)
      {
        system_t.Tick_Main1 = 0;
        LED_taskMPU6050 = !LED_taskMPU6050;
        //LedDebugToggle();
      }
    }
  }
}
static void system_initialize()
{
  init_timer(TIMER_GROUP_0, TIMER_DIRECTION_1, TIMER_INTERVAL);
  ESP_LOGD_OP(TAG, "Timer initialize successful.");
  init_adc(ADC_12_BIT, ADC_BATTERY, ADC_ATTEN_11);
  ESP_LOGD_OP(TAG, "ADC Battery initialize successful.");
  init_adc(ADC_12_BIT, ADC_ADAPTOR, ADC_ATTEN_11);
  ESP_LOGD_OP(TAG, "ADC Adaptor initialize successful.");
  init_led_control(LED_BATTERY, PWM_LEDBAT_CHANNEL_0, PWM_LEDBAT_FREQ);
  ESP_LOGD_OP(TAG, "ADC Adaptor initialize successful.");
  init_GPIO_output(GPIO_CHARGE_CONTROL);
  ESP_LOGD_OP(TAG, "GPIO Charge Control Init done.");
  init_GPIO_output(LED_ADAPTOR);
  ESP_LOGD_OP(TAG, "Led Adaptor Init done.");
  init_GPIO_output(LED_RED_BLE_ON);
  ESP_LOGD_OP(TAG, "Led Red BLE ON Init done.");
  init_GPIO_output(LED_GREEN_CALIB);
  ESP_LOGD_OP(TAG, "Led Green Calib Init done.");
  init_GPIO_output(LED_BLUE_BLE_NOTIFY);
  ESP_LOGD_OP(TAG, "Led Blue Ble Notify Init done.");
  init_user_button(BUTTON_USER0);
  ESP_LOGD_OP(TAG, "Button User 0 Init done.");
  init_user_button(BUTTON_USER1);
  ESP_LOGD_OP(TAG, "Button User 1 Init done.");
  init_user_button(MODE_0_GPIO);
  ESP_LOGD_OP(TAG, "Button Mode 0 Init done.");
#ifdef USE_INT_PIN
  init_int_pin();
  ESP_LOGD_OP(TAG, "Interrupt Init Done.");
#endif
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_LOGD_OP(TAG, "NVS Flash Init done.");
}
/*================================================================================================*/
/**
* @function name  app_main
* @brief          Main program
* @parameter      none
* @return value   none
* @note           none
*/
#ifdef MOTION_TRACKING
void app_main()
{
  uint32_t heapsize_u32;
  uint8_t counter_mainloop_u8;

  TaskHandle_t handle_task_uart0_esplog = NULL;
  TaskHandle_t handle_task_gpio_control = NULL;
  TaskHandle_t handle_task_ble_gatt_beacon_advertising = NULL;
  system_initialize();
  tracking_variables_gtr.state_machine = FREEZE_STATE;
  notify2app_gstr.data.start = 0x41;
  notify2app_gstr.data.reserved = 0xAA;
  notify2app_gstr.data.Yaw = 0.05f;
  notify2app_gstr.data.Pitch = 0.7f;
  notify2app_gstr.data.Roll = 0.09f;
  notify2app_gstr.data.Accelerator_X = -125;
  notify2app_gstr.data.Accelerator_Y = 365;
  notify2app_gstr.data.Accelerator_Z = 128;
  notify2app_gstr.data.Temperature = 29;
  notify2app_gstr.data.Gyro_X = 0x5353;
  notify2app_gstr.data.Gyro_Y = 0x5454;
  notify2app_gstr.data.Gyro_Z = 0x5555;
  notify2app_gstr.data.end = 0x7F;
  motion_tracking_flag_gstr.bat_charging = 1;
  motion_tracking_flag_gstr.adaptor_plugin = 1;
  motion_tracking_flag_gstr.request_adc_adaptor = 1;
  tracking_variables_gtr.counter_userbutton0_delay = 0;
  motion_tracking_flag_gstr.received_data_from_app = false;

  ReadMemoryU16("device_address", &tracking_variables_gtr.device_address);
  ESP_LOGI_OP(MAIN_TAG, "Device address: %x", tracking_variables_gtr.device_address);
  vTaskDelay(pdMS_TO_TICKS(5000));
  counter_mainloop_u8 = 0;

  if (handle_task_uart0_esplog == NULL)
  {
    /*Core_1*/
    task_create(uart0_esplog_task, "uart0_esplog_task", 2048, NULL, TASK_PRIORYTI_4, &handle_task_uart0_esplog, APP_CPU_NUM);
  }

  if (handle_task_gpio_control == NULL)
  {
    /*Core_1*/
    task_create(adc_gpio_control_task, "adc_gpio_control_task", 2048, NULL, TASK_PRIORYTI_1, &handle_task_gpio_control, APP_CPU_NUM);
  }

  while (1)
  {
    /*--------------Check state_machine for BLE task----------------*/
    switch (tracking_variables_gtr.state_machine)
    {
    case FREEZE_STATE:
      /* do nothing */
      if (handle_task_ble_gatt_beacon_advertising != NULL)
      {
        gattclient_deinit();
        myTaskDelete(handle_task_ble_gatt_beacon_advertising);
        handle_task_ble_gatt_beacon_advertising = NULL;
      }
      taskMPU6050_Suspend();
      break;
    case BEACON_STATE:
      if (handle_task_ble_gatt_beacon_advertising == NULL)
      {
        ble_gatt_init();
        task_create(ble_gatt_beacon_advertising_task, "ble_gatt_beacon_advertising_task", 4096, NULL, TASK_PRIORYTI_6,
                    &handle_task_ble_gatt_beacon_advertising, APP_CPU_NUM);
      }
      taskMPU6050_Suspend();
      break;
    case WORKING_STATE:
      if (handle_taskMPU6050 == NULL)
      {
        //delay_ms(200);//delay for enable BLE , after enable MPU6050,. for reduce power
        /* MPU6050 init and read task */
        task_create(taskMPU6050, "taskMPU6050", 4096, NULL, TASK_PRIORYTI_5, &handle_taskMPU6050, APP_CPU_NUM);
      }
      else
      {
        taskMPU6050_Resume();
      }
      break;
    default:
      break;
    }

    /*Display HEAP_SIZE for Debug*/
    counter_mainloop_u8++;
    if (!(counter_mainloop_u8 % 50))
    {
      heapsize_u32 = esp_get_free_heap_size();
      ESP_LOGD_OP(MAIN_TAG, "Current heap: %d", heapsize_u32);
    }
    /*---------------------------------------------------------------*/
    /*---Check Data received from app (SmartPhone*/
    process_data_from_app();

    vTaskDelay(100 / portTICK_RATE_MS);
  }
}
#endif
#ifdef READ_DATA_ONLY
void app_main()
{
  TickType_t xLastWakeTime;
  uint8_t time = 20;
  uint8_t err;
  system_initialize();
  ESP_ERROR_CHECK(i2c_master_init());
  ESP_LOGI_OP(TAG, "i2c_master_init() successful.");
  MPU6050_Setup();
  ESP_LOGI_OP(TAG, "MPU6050_Setup() successful.");
  calib(&err_acc, &err_gyr);
  ESP_LOGI_OP(TAG, "calib(&err_acc, &err_gyr) successful.");
  xLastWakeTime = xTaskGetTickCount();
  while (true)
  {
    if (time == 0)
    {
      time = 20;
      MPU6050_dmpReadLoop();
    }
    err = MPU6050_getMotion6_Mod(&aReal, &gReal, &tReal);
    aReal.x -= err_acc.x;
    aReal.y -= err_acc.y;
    aReal.z -= err_acc.z;
    gReal.x -= err_gyr.x;
    gReal.y -= err_gyr.y;
    gReal.z -= err_gyr.z;
    ESP_LOGI_OP(TAG, "%d|%d|%d|%d|%d|%d|%d", aReal.x, aReal.y, aReal.z, gReal.x, gReal.y, gReal.z, tReal);
    time--;
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
  }
}
#endif

/*================================================================================================*/
/**
* @function name  process_data_from_app
* @brief          processing data form app
* @parameter      none
* @return value   none
* @note           none
*/
static void process_data_from_app(void)
{
  uint8_t counter_general_u8;
  uint8_t checksum_u8;
  if (!motion_tracking_flag_gstr.received_data_from_app)
    return;

  motion_tracking_flag_gstr.received_data_from_app = false;
  if (receive_from_app_gtr.data.start != RECEIVE_FROM_APP_START)
  {
    ESP_LOGD_OP(MAIN_TAG, "DATA FROM APP - START ERROR");
    return;
  }
  if (receive_from_app_gtr.data.end != RECEIVE_FROM_APP_END)
  {
    ESP_LOGD_OP(MAIN_TAG, "DATA FROM APP - END ERROR");
    return;
  }
  checksum_u8 = 0;

  for (counter_general_u8 = 0; counter_general_u8 < (RECEIVE_FROM_APP_DATA_LEN - 2); counter_general_u8++)
  {
    checksum_u8 ^= receive_from_app_gtr.array[counter_general_u8];
  }
  if (checksum_u8 != receive_from_app_gtr.array[counter_general_u8])
  {
    /*Data Error*/
    ESP_LOGD_OP(MAIN_TAG, "DATA FROM APP - CHECKSUM ERROR");
    return;
  }
  /*----------*/
  switch (receive_from_app_gtr.data.command)
  {
  case RECEIVE_FROM_APP_CALIB:
    /*nothing*/
    tracking_variables_gtr.state_machine = CONFIG_STATE;
    break;
  case RECEIVE_FROM_APP_SETTING:
    tracking_variables_gtr.counter_milisecond = receive_from_app_gtr.data.timestamp;
    tracking_variables_gtr.device_address = receive_from_app_gtr.data.addr;
    SaveMemoryU16("device_address", tracking_variables_gtr.device_address);
    break;
  default:
    ESP_LOGD_OP(MAIN_TAG, "DATA FROM APP - COMMAND IS NOT SUPPORT");
    break;
  }
}

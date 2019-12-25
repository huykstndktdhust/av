/**
*   @file name     esplog_task_uart0.c
*   Project:       Motion_Tracking
*   @Purpose       uart0 source file
*   Author:        TaiVV2@fsoft.com.vn
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "esplog_task_uart0.h"
#include "ExternVariablesFunctions.h"
/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
static void MPU6050_logdata(void);

/*================================================================================================*/
/**
* @function name  uart0_esplog_task
* @brief          Task UART_0 LOG
* @parameter      *arg
* @return value   none
* @note           none
*/
_tracking_variables tracking_variables_gtr;
_motion_tracking_flag motion_tracking_flag_gstr;
void uart0_esplog_task(void* arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    while (1)
    {
        /*MPU6050 INFOR*/

        MPU6050_logdata();

        /*Notify others*/
        if (motion_tracking_flag_gstr.uart_log_other)
        {
            motion_tracking_flag_gstr.uart_log_other = false;

            /*Notify State of machine*/
            switch (tracking_variables_gtr.state_machine)
            {
                case FREEZE_STATE:
                    ESP_LOGD_OP("<STATE_MACHINE>", "<FREEZE>");
                    break;
                case BEACON_STATE:
                    ESP_LOGD_OP("<STATE_MACHINE>", "<BEACON>");
                    break;
                case CONFIG_STATE:
                    ESP_LOGD_OP("<STATE_MACHINE>", "<CONGIF - CALIB MPU6050>");
                    break;
                case WORKING_STATE:
                    ESP_LOGD_OP("<STATE_MACHINE>", "<WORKING>");
                    break;
                default:
                    /*LOG ERROR*/
                    ESP_LOGE_OP("<STATE_MACHINE>", "<ERROR>");
                    break;
            }

            /*Notify status of Adaptor*/
            if (motion_tracking_flag_gstr.adaptor_plugin)
            {
                ESP_LOGD_OP("<ADAPTOR>", "<PLUGIN>");
            }
            else
            {
                ESP_LOGE_OP("<ADAPTOR>", "<UN_PLUGIN>");
            }

            /*Notify status of Charge/Not_Charge*/
            if (motion_tracking_flag_gstr.bat_charging)
            {
                /*Log WHITE*/
                ESP_LOGD_OP("<BATTERY>", "<CHARGING>");
            }
            else
            {
                ESP_LOGE_OP("<BATTERY>", "<DIS_CHARGE>");
            }

        }

//        motion_tracking_flag_gstr._1_seconds = 0;
//        motion_tracking_flag_gstr._2_seconds = 0;
//        motion_tracking_flag_gstr._5_seconds = 0;

//        tracking_variables_gtr.state_machine++;
//        if (tracking_variables_gtr.state_machine >9)
//            tracking_variables_gtr.state_machine = 0;
//        vTaskDelay(2000 / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/*================================================================================================*/
/**
* @function name  MPU6050_logdata
* @brief          log of MPU6050's data
* @parameter      none
* @return value   none
* @note           none
*/
static void MPU6050_logdata(void)
{
    if (!motion_tracking_flag_gstr.MPU_notify2uart)
        return;
    motion_tracking_flag_gstr.MPU_notify2uart = 0;

    /*While color*/
//    ESP_LOGW_OP("<MPU6050_DATA>", "<----------START DATA---------->");
//
//    ESP_LOGI_OP("--", "Start: \t\t%X\t|| Addr: %X", notify2app_gstr.data.start, notify2app_gstr.data.addr);
//    ESP_LOGI_OP("--", "Timestamp: \t%d ", notify2app_gstr.data.timestamp);
//    ESP_LOGI_OP("--", "Quaternion: \t%0.2f \t%0.2f \t%0.2f \t%0.2f", qtn.w, qtn.x, qtn.y, qtn.z);
//    ESP_LOGI_OP("--", "Yaw,Pitch,Roll: \t%0.2f \t%0.2f \t%0.2f", notify2app_gstr.data.Yaw, notify2app_gstr.data.Pitch, notify2app_gstr.data.Roll);
//    ESP_LOGI_OP("--", "Accelerator_X_Y_Z:\t%d \t%d  \t%d ", notify2app_gstr.data.Accelerator_X, notify2app_gstr.data.Accelerator_Y, notify2app_gstr.data.Accelerator_Z);
//    ESP_LOGI_OP("--", "Gyro_X_Y_Z: \t\t%d \t%d  \t%d  ", notify2app_gstr.data.Gyro_X, notify2app_gstr.data.Gyro_Y, notify2app_gstr.data.Gyro_Z);
//    ESP_LOGI_OP("--", "Temperature: \t%d", notify2app_gstr.data.Temperature);
//    ESP_LOGI_OP("--", "Checksum: \t%X", notify2app_gstr.data.checksum);
//    ESP_LOGI_OP("--", "End frame: \t%X ", notify2app_gstr.data.end);
//
//    ESP_LOGW_OP("<MPU6050_DATA>", "<----------END DATA---------->");

//    ESP_LOGI_OP("<MPU6050_DATA>", "Start: \t\t%X\t|| Addr: %X", notify2app_gstr.data.start, notify2app_gstr.data.addr);
//    ESP_LOGI_OP("<MPU6050_DATA>", "Timestamp: \t%d ", notify2app_gstr.data.timestamp);
//    ESP_LOGI_OP("<MPU6050_DATA>", "Yaw,Pitch,Roll: \t%0.2f \t%0.2f \t%0.2f", notify2app_gstr.data.Yaw, notify2app_gstr.data.Pitch, notify2app_gstr.data.Roll);
//    ESP_LOGI_OP("<MPU6050_DATA>", "Accelerator_X_Y_Z:\t%d \t%d  \t%d ", notify2app_gstr.data.Accelerator_X, notify2app_gstr.data.Accelerator_Y, notify2app_gstr.data.Accelerator_Z);
//    ESP_LOGI_OP("<MPU6050_DATA>", "Gyro_X_Y_Z: \t\t%d \t%d  \t%d  ", notify2app_gstr.data.Gyro_X, notify2app_gstr.data.Gyro_Y, notify2app_gstr.data.Gyro_Z);
//    ESP_LOGI_OP("<MPU6050_DATA>", "Temperature: \t%d", notify2app_gstr.data.Temperature);
//    ESP_LOGI_OP("<MPU6050_DATA>", "Checksum: \t%X", notify2app_gstr.data.checksum);
//    ESP_LOGI_OP("<MPU6050_DATA>", "End frame: \t%X ", notify2app_gstr.data.end);
}
/*================================================================================================*/

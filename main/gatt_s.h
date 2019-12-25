/**
 *   @file name     gatt_s.h
 *   Project:       Motion_Tracking
 *   @Purpose       GATT_S header file
 *   Author:        HuyNNQ@fsoft.com.vn
 *   Final editors: SangTN@fsoft.com.vn
 *   @date:         07-30-2019
 */

#ifndef MAIN_GATT_S_H_
#define MAIN_GATT_S_H_
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "tracking_main.h"

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#define DEVICE_NAME                     "Motion_Tracking"
#define GATTS_TABLE_TAG                 "EVEN_GATTS"
#define HRPS_HT_MEAS_MAX_LEN            (13)

#define HRPS_MANDATORY_MASK             (0x0F)
#define HRPS_BODY_SENSOR_LOC_MASK       (0x30)
#define HRPS_HR_CTNL_PT_MASK            (0xC0)

#define MOTION_TRACKING_SERVICE_UUID     0x4200
#define MOTION_TRACKING_CHAR_RW_UUID     0x4201 //R-W
#define MOTION_TRACKING_CHAR_N_UUID      0x4202 //N
#define MOTION_TRACKING_APP_ID           0x1200

#define MOTION_TRACKING_PROFILE_NUM      1
#define MOTION_TRACKING_APP_IDX          0
#define MOTION_TRACKING_SVC_INST_ID      0

#define GATTS_CHAR_VAL_LEN_MAX           0x40
#define TEST_MANUFACTURER_DATA_LEN       0x19
#define RECEIVE_FROM_APP_DATA_LEN        10

#define RECEIVE_FROM_APP_START           0x43
#define RECEIVE_FROM_APP_END             0x7F
#define RECEIVE_FROM_APP_CALIB           0x01
#define RECEIVE_FROM_APP_SETTING         0x02

#define ADV_CONFIG_FLAG                  (1 << 0)
#define SCAN_RSP_CONFIG_FLAG             (1 << 1)

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
void ble_gatt_init(void);
void gattclient_deinit(void);
void ble_gatt_beacon_advertising_task(void* arg);

#endif /* MAIN_GATT_S_H_ */

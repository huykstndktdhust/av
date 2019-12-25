/**
*   @file name     logger.h
*   Project:       Motion_Tracking
*   @Purpose       logger header file
*   Author:        TaiVV2@fsoft.com.vn
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/

#ifndef MAIN_LOGGER_H_
#define MAIN_LOGGER_H_
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "tracking_main.h"

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#define LOG_FORMAT(letter, format)  LOG_COLOR_ ## letter #letter " (%d) %s: " format LOG_RESET_COLOR "\n"
/*Error - RED Color*/
#define ESP_LOGE_OP(tag, format, ...) esp_log_write(ESP_LOG_ERROR, tag, LOG_FORMAT(E, format), esp_log_timestamp(), tag, ##__VA_ARGS__)
/*Infor - GREEN Color*/
#define ESP_LOGI_OP(tag, format, ...) esp_log_write(ESP_LOG_INFO, tag, LOG_FORMAT(I, format), esp_log_timestamp(), tag, ##__VA_ARGS__)
/*Warning - YELLOW Color*/
#define ESP_LOGW_OP(tag, format, ...) esp_log_write(ESP_LOG_WARN, tag, LOG_FORMAT(W, format), esp_log_timestamp(), tag, ##__VA_ARGS__)
/*Debug - WHITE  Color*/
#define ESP_LOGD_OP(tag, format, ...) esp_log_write(ESP_LOG_DEBUG, tag, LOG_FORMAT(D, format), esp_log_timestamp(), tag, ##__VA_ARGS__)

#endif /* MAIN_LOGGER_H_ */

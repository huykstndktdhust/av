/**
*   @file name     esplog_task_uart0.h
*   Project:       Motion_Tracking
*   @Purpose       uart0 header file
*   Author:        TaiVV2@fsoft.com.vn
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/

#ifndef MAIN_ESPLOG_TASK_UART0_H_
#define MAIN_ESPLOG_TASK_UART0_H_
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "tracking_main.h"

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#define RX_BUF_SIZE                      1024

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
void init_uart(void);
void uart0_esplog_task(void* arg);

#endif /* MAIN_ESPLOG_TASK_UART0_H_ */

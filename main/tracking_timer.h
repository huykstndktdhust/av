/**
*   @file name     tracking_timer.h
*   Project:       Motion_Tracking
*   @Purpose       timer header file
*   Author:        TanTD82@fsoft.com.vn
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/

#ifndef MAIN_TRACKING_TIMER_H_
#define MAIN_TRACKING_TIMER_H_
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "tracking_main.h"

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#define TIMER_DIVIDER                                         16  /*!< Hardware timer clock divider */
#define TIMER_SCALE                                           (TIMER_BASE_CLK / TIMER_DIVIDER)  /*!< used to calculate counter value */
#define TIMER_INTERVAL                                        5000 /*!< test interval for timer 1ms = (TIMER_BASE_CLK / TIMER_DIVIDER) * 0.001 */
#define TIMER_DIRECTION_1                                     TIMER_1
#define TIMER_VALUE_UPDATE                                    1
#define TIMER_CLEAR_FLAG                                      1

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
void init_timer(uint8_t timer_group, uint8_t timer_idx, uint16_t timer_interval);

#endif /* MAIN_TRACKING_TIMER_H_ */

/**
*   @file name     tracking_timer.c
*   Project:       Motion_Tracking
*   @Purpose       timer source file
*   Author:        TanTD8@fsoft.com.vn
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "tracking_timer.h"
#include "ExternVariablesFunctions.h"
/*================================================================================================*/
_tracking_variables tracking_variables_gtr;
_motion_tracking_flag motion_tracking_flag_gstr;
/**
* @function name  timer_group0_isr
* @brief          ISR Timer 1ms, timestamp 1ms
* @parameter      *para
* @return value   none
* @note           IRAM_ATTR: Forces code into IRAM instead of flash.
*/
void IRAM_ATTR timer_group0_isr(void *para)
{
    /*  ADC_power.timestamp ++;*/
//    notify2app_gstr.data.timestamp ++;
    /*--------------------*/
    tracking_variables_gtr.counter_milisecond++;

    /*100ms*/
    if (!(tracking_variables_gtr.counter_milisecond % 100))
    {
        motion_tracking_flag_gstr.test ^= 1;

        motion_tracking_flag_gstr.led_user1_blue ^= 1;

        tracking_variables_gtr.counter_ctrl_pwm_ledbat++;
        if (tracking_variables_gtr.counter_ctrl_pwm_ledbat >= PWM_LEDBAT_CYCLES_2S)
            tracking_variables_gtr.counter_ctrl_pwm_ledbat = 0;

        if ((motion_tracking_flag_gstr.button_user0_down) && (tracking_variables_gtr.counter_userbutton0_delay < BUTTON_USER0_DELAY))
        {
            tracking_variables_gtr.counter_userbutton0_delay++;
        }
    }
    /*5 seconds*/
    if(!(tracking_variables_gtr.counter_milisecond % 5000))
    {
        motion_tracking_flag_gstr.request_adc_adaptor = true;
        motion_tracking_flag_gstr.request_adc_bat = true;
        motion_tracking_flag_gstr.uart_log_other = true;

//        motion_tracking_flag_gstr.MPU_notify2uart = true;
    }

    /*time-base counter value update */
    TIMERG0.hw_timer[1].update = TIMER_VALUE_UPDATE;
    /*clear_interrupt_flag*/
    TIMERG0.int_clr_timers.t1 = TIMER_CLEAR_FLAG;
    TIMERG0.hw_timer[TIMER_DIRECTION_1].config.alarm_en = TIMER_ALARM_EN;
}

/*================================================================================================*/
/**
* @function name  init_timer
* @brief          configure Timer
* @parameter      timer_group: timer group
*                 timer_idx: Select a hardware timer from timer groups
*                 timer_interval: interval of timer
* @return value   none
* @note           none
*/
void init_timer(uint8_t timer_group, uint8_t timer_idx, uint16_t timer_interval)
{
    timer_config_t config;
    config.alarm_en = 1;
    config.auto_reload = 1;
    config.counter_dir = TIMER_COUNT_UP;
    config.divider = TIMER_DIVIDER;
    config.intr_type = TIMER_INTR_LEVEL;
    config.counter_en = TIMER_PAUSE;
    /*Configure timer*/
    timer_init(timer_group, timer_idx, &config);
    /*Load counter value */
    timer_set_counter_value(timer_group, timer_idx, 0x00000000ULL);
    /*Set alarm value*/
    timer_set_alarm_value(timer_group, timer_idx, TIMER_INTERVAL);
    /*Enable timer interrupt*/
    timer_enable_intr(timer_group, timer_idx);
    timer_isr_register(timer_group, timer_idx, timer_group0_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(timer_group, timer_idx);

    /*Flag_timestamp*/
    notify2app_gstr.data.timestamp = 0;
}
/*================================================================================================*/

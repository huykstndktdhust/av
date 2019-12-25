/*
 * motion_tracking.h
 *
 *  Created on: Jul 19, 2019
 *      Author: TaiJohnSon
 */

#ifndef MAIN_MOTION_TRACKING_H_
#define MAIN_MOTION_TRACKING_H_

#include <stdint.h>

#define     MOTION_TRACKING_DEBUG          0

#define     TXD_PIN                         (GPIO_NUM_17)
#define     RXD_PIN                         (GPIO_NUM_16)

#define     LED_ON                          0
#define     LED_OFF                         1

#define     BATTERY_CHARGE_ENABLE           1
#define     BATTERY_CHARGE_DISABLE          0
#define     ADAPTOR_PLUGIN                  1
#define     ADAPTOR_UNPLUGIN                0
#define     HIGH_BATTERY                    1
#define     LOW_BATTERY                     0

#define     FREEZE_STATE                    0
#define     BEACON_STATE                    1
#define     CONFIG_STATE                    2
#define     WORKING_STATE                   3

typedef struct
{
	float volt_battery;
	float volt_adaptor;
	int16_t bat_adc_raw;
	int16_t adaptor_adc_raw;
	uint8_t state_machine;
	uint32_t counter_milisecond;
	uint16_t device_adderss;
	uint8_t counter_ctrl_pwm_ledbat;
	int16_t counter_userbutton0_delay;
} _tracking_variables;

/*frame notify to smart phone (application) - 36 bytes*/
typedef union
{
	struct
	{
	  uint8_t start;
    uint8_t reserved;
    uint16_t addr;
    uint32_t timestamp;
    float Yaw;
    float Pitch;
    float Roll;
    int16_t Accelerator_X;
    int16_t Accelerator_Y;
    int16_t Accelerator_Z;
    int16_t Temperature;
    int16_t Gyro_X;
    int16_t Gyro_Y;
    int16_t Gyro_Z;
    uint8_t checksum;
    uint8_t end;
	} data;
	uint8_t array[36];
}_notify2app;

typedef union
{
    struct
    {
    uint8_t start;
    uint8_t command;
    uint16_t addr;
    uint32_t timestamp;
    uint8_t checksum;
    uint8_t end;
    } data;
    uint8_t array[10];
}_receive_from_app;

typedef struct
{
    unsigned bat_charging :1;
    unsigned bat_low :1;
    unsigned bat_under_voltcharge :1;
    unsigned bat_upper_voltcharge :1;
    unsigned adaptor_plugin :1;
    unsigned request_adc_adaptor :1;
    unsigned adc_adaptor_error :1;
    unsigned request_adc_bat :1;
    unsigned adc_bat_error :1;
    unsigned MPU_notify2app :1;
    unsigned MPU_notify2uart :1;
    unsigned button_user0_down :1;
    unsigned button_user1_down :1;
    unsigned led_user1_blue :1;
    unsigned led_user2_green :1;
    unsigned received_data_from_app :1;
    unsigned uart_log_other :1;

    unsigned test :1;
} _motion_tracking_flag;


#endif /* MAIN_MOTION_TRACKING_H_ */

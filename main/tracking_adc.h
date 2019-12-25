/**
*   @file name     tracking_adc.h
*   Project:       Motion_Tracking
*   @Purpose       tracking_adc header file
*   Author:        TanTD8@fsoft.com.vn
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/

#ifndef MAIN_TRACKING_ADC_H_
#define MAIN_TRACKING_ADC_H_

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "tracking_main.h"

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/
#define ADC_DEBUG                                             1

#define ADC_ERROR                                             1
#define ADC_OK                                                0

#define ADAPTOR_ADC_CHANNEL                                   ADC1_CHANNEL_7
#define BATTERY_ADC_CHANNEL                                   ADC1_CHANNEL_6

#define ADC_12_BIT                                            3
#define ADC_BATTERY                                           6
#define ADC_ADAPTOR                                           7
#define ADC_ATTEN_11                                          ADC_ATTEN_DB_11

#define ADC_CONVERT_VOLT_BATTERY                              3.30 / 4096 * (30000 + 22000) / 30000
#define ADC_CONVERT_VOLT_DAPTOR                               3.30 / 4096 * (20000 + 22000)  / 20000

#define BUTTON_USER0_DELAY                                    30 /*x100 miliseconds*/
#define BUTTON_USER1_DELAY                                    30 /*x100 miliseconds*/

//#define GPIO_INPUT_PIN_SEL                                    ((1ULL<<BUTTON_USER0) | (1ULL<<BUTTON_USER1))

#define VOLT_ADAPTOR_DETECT                                   4.0/*Vin of Adaptor = 4.8*/
//#define VOLT_DEFAULT_ADAPTOR                                  4.8
//#define VOLT_CHARGE_BATTERY                                   3.6/*Vin of high battery = 3.6*/
//#define VOLT_LOW_BATTERY                                      3.3/*Vin of Low battery = 3.3*/
//#define VOLT_HIGH_BATTERY                                     4.0/*Vin of High battery = 4.0*/

#define VOLT_BATTERY_LOW                                      3.3/*Vin of High battery = 4.0*/
#define VOLT_BATTERY_CHARGE                                   3.9/*Vin of High battery = 3.9*/
#define VOLT_BATTERY_OFFSET                                   0.2
#define PWM_LEDBAT_CHANNEL_0                                  0
//#define PWM_LEDBAT_CHARGED_DUTY                               25
//#define PWM_LEDBAT_LOWPOWER_DUTY                              75
//#define PWM_LEDBAT_FULLCHARGE_DUTY                            0
//#define PWM_FULL                                              100

/*PWM Define*/
#define LED_BATTERY                                           GPIO_NUM_32
#define DUTY_RESOLUTION                                       81.92
#define PWM_LEDBAT_FREQ                                       500    /*Hz*/
#define PWM_LEDBAT_DUTY_50                                    50
#define PWM_LEDBAT_CYCLES_2S                                  20    /*CYCLE = PWM_LEDBAT_CYCLES_2S * 100ms*/

//#define COUNTER_CTRL_PWM_LEDBAT                               10
//#define COUNTER_CTRL_PWM_LEDBAT_UPPER                         40


#define DURATION_USER                                         1000

//#define VOLT_ADAPTOR_DETECT_RAW                               (VOLT_ADAPTOR_DETECT*20000/(20000+22000)-0.06)*4096/3.30
/*==================================================================================================
*                                      VARIABLES
==================================================================================================*/
//typedef struct
//{
//    unsigned flag_bat_charge_enable :1;
//    unsigned flag_bat_level :1;
//    unsigned flag_adaptor_plugin :1;
//    unsigned flag_timestamp :1;
//    uint32_t timestamp;
//    float volt_battery;
//} _ADC_power;

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
void adc_gpio_control_task(void* arg);
void init_adc(uint8_t resolution, uint8_t channel, uint8_t atten);
void init_led_control(uint8_t GPIO_NUM, uint8_t Channel, uint32_t freq);
void init_GPIO_output(uint8_t GPIO);
void init_int_pin(void);
void init_user_button(uint64_t gpio_pin);
//void setup_dutycycle(uint16_t DutyCycle, uint8_t Channel);
//void Battery_Charge(void);
//void Check_Battery(void);
//void Check_Adaptor();

#endif /* MAIN_TRACKING_ADC_H_ */

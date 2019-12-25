/**
*   @file name     tracking_adc.c
*   Project:       Motion_Tracking
*   @Purpose       ADC source file
*   Author:        TaiVV2@fsoft.com.vn
*   Final editors: SangTN@fsoft.com.vn
*   @date:         07-30-2019
*/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "tracking_adc.h"
#include "ExternVariablesFunctions.h"
/*==================================================================================================
*                                      VARIABLES
==================================================================================================*/
static const char *TAG_ADC_ERROR = "ADC_CONVERT_ERROR";
static const char *TAG_ADC_OK = "ADC_CONVERT_OK";

static void adaptor_check(void);
static void battery_check(void);
static void capture_user_buttons(void);
static void control_gpio(void);
static void led_control_pwm(bool  on_off, uint8_t channel);
_tracking_variables tracking_variables_gtr;
_motion_tracking_flag motion_tracking_flag_gstr;
/*================================================================================================*/
uint16_t volt_adaptor_detect_raw_gu16 = (VOLT_ADAPTOR_DETECT * 20000 / (20000 + 22000)) * 4096 / 3.30;

uint16_t volt_battery_undercharge_raw_gu16 = ((VOLT_BATTERY_CHARGE - VOLT_BATTERY_OFFSET) * 30000 / (30000 + 22000)) * 4096 / 3.30;
uint16_t volt_battery_uppercharge_raw_gu16 = ((VOLT_BATTERY_CHARGE + VOLT_BATTERY_OFFSET) * 30000 / (30000 + 22000)) * 4096 / 3.30;
uint16_t volt_battery_low_raw_gu16 = ((VOLT_BATTERY_LOW) * 30000 / (30000 + 22000)) * 4096 / 3.30;
/* Description: Read ADC from Battery and Adaptor, Check Adaptor pluggin and Level of Battery, display LED(GPIO/PWM)
 * Parameter: None
 * Return: None
 */
void adc_gpio_control_task(void* arg)
{
    while (1)
    {
        adaptor_check();
        battery_check();
        capture_user_buttons();
        control_gpio();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
/*================================================================================================*/
/**
* @function name  init_int_pin
* @brief          configure GPIO is Input
* @parameter      gpio_pin: number of GPIO config
* @return value   none
* @note           none
*/
void init_int_pin(void)
{
    //change gpio intrrupt type for one pin
    gpio_set_intr_type(INT_MPU_GPIO, GPIO_INTR_POSEDGE);
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);//ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(INT_MPU_GPIO, gpio_isr_handler, NULL);

    gpio_intr_enable(INT_MPU_GPIO);
}
/*
 * Description: configure GPIO is Input
 * Parameter:
 *   gpio_pin: number of GPIO config
 * Return: none
 */
void init_user_button(uint64_t gpio_pin)
{
    gpio_config_t io_conf;
    /*Disable interrupt*/
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    /*bit mask of the pins, use GPIO31/30 here*/
    io_conf.pin_bit_mask = ((uint64_t)1) << gpio_pin;
    /*set as input mode*/
    io_conf.mode = GPIO_MODE_INPUT;
    /*disable pull-up mode*/
    io_conf.pull_up_en = 0;
    /*disable pull-down mode*/
    io_conf.pull_down_en = 0;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

/*================================================================================================*/
/**
* @function name  control_gpio
* @brief          Check Status machine
* @parameter      none
* @return value   none
* @note           none
*/
void control_gpio(void)
{
    /*----------Control USER_LEDs----------*/
    switch (tracking_variables_gtr.state_machine)
    {
        case FREEZE_STATE:
            /*off 3 user leds*/
        ESP_ERROR_CHECK(gpio_set_level(LED_RED_BLE_ON, LED_OFF));
        //gpio_set_level(LED_GREEN_USER, LED_OFF);
        gpio_set_level(LED_BLUE_BLE_NOTIFY, LED_OFF);
        //gpio_set_level(LED_GREEN_CALIB, LED_OFF);
            break;
        case BEACON_STATE:
        /*ON YELLOW led*/
        ESP_ERROR_CHECK(gpio_set_level(LED_RED_BLE_ON, LED_ON));
        /*OFF other leds*/
        gpio_set_level(LED_BLUE_BLE_NOTIFY, LED_OFF);
        //gpio_set_level(LED_GREEN_CALIB, LED_OFF);
            break;
        case CONFIG_STATE:
        /*ON RED LED*/
        ESP_ERROR_CHECK(gpio_set_level(LED_RED_BLE_ON, LED_ON));

        gpio_set_level(LED_BLUE_BLE_NOTIFY, LED_OFF);
        //gpio_set_level(LED_GREEN_CALIB, motion_tracking_flag_gstr.led_user2_green);
            break;
        case WORKING_STATE:
        /*ON GREEN Led*/
        //ESP_ERROR_CHECK(gpio_set_level(LED_GREEN_CALIB, LED_OFF));
        /*Off other leds*/
        ESP_ERROR_CHECK(gpio_set_level(LED_RED_BLE_ON, LED_ON));
        gpio_set_level(LED_BLUE_BLE_NOTIFY, motion_tracking_flag_gstr.led_user1_blue);
            break;
        default:
        /*off 3 user leds*/
        //ESP_ERROR_CHECK(gpio_set_level(LED_GREEN_CALIB, LED_OFF));
        gpio_set_level(LED_RED_BLE_ON, LED_OFF);
        gpio_set_level(LED_BLUE_BLE_NOTIFY, LED_OFF);
            break;
    }
    gpio_set_level(LED_GREEN_CALIB, LED_taskMPU6050);

    /*----------Control BATT CHARGE----------*/
    if (motion_tracking_flag_gstr.adaptor_plugin == ADAPTOR_PLUGIN)
    {
        /*ON led indicate: ADAPTOR was Plugin*/
        ESP_ERROR_CHECK(gpio_set_level(LED_ADAPTOR, LED_ON));

        /*Control charge battery*/
        if (motion_tracking_flag_gstr.bat_under_voltcharge)
        {
            /*Charge for Battery*/
            ESP_ERROR_CHECK(gpio_set_level(GPIO_CHARGE_CONTROL, BATTERY_CHARGE_ENABLE));
            motion_tracking_flag_gstr.bat_charging = BATTERY_CHARGE_ENABLE;
        }
        else if (motion_tracking_flag_gstr.bat_upper_voltcharge)
        {
            /*Disable Charge*/
            ESP_ERROR_CHECK(gpio_set_level(GPIO_CHARGE_CONTROL, BATTERY_CHARGE_DISABLE));
            motion_tracking_flag_gstr.bat_charging = BATTERY_CHARGE_DISABLE;
        }
    }
    else
    {
        /*OFF led indicate: Adaptor was not plugin*/
        ESP_ERROR_CHECK(gpio_set_level(LED_ADAPTOR, LED_OFF));
        /*Dis_Charge*/
        gpio_set_level(GPIO_CHARGE_CONTROL, BATTERY_CHARGE_DISABLE);
        motion_tracking_flag_gstr.bat_charging = BATTERY_CHARGE_DISABLE;
    }

    /*----------Control Indicator_LEDs----------*/
    /*counter_ctrl_pwm_ledbat --> from 0 to 9, each step is 100 milisecond*/
    if (motion_tracking_flag_gstr.bat_charging)
    {
        /*LED BATTERY ON During 1s*/
        if (tracking_variables_gtr.counter_ctrl_pwm_ledbat < (PWM_LEDBAT_CYCLES_2S/2))
        {
            led_control_pwm(LED_ON,PWM_LEDBAT_CHANNEL_0);
        }
        else
        {
            /*LED BATTERY OFF During 1s*/
            led_control_pwm(LED_OFF,PWM_LEDBAT_CHANNEL_0);
        }
    }
    else if (motion_tracking_flag_gstr.bat_low)
    {
        /*LED BATTERY ON During 100ms*/
        if (tracking_variables_gtr.counter_ctrl_pwm_ledbat == 5)
        {
            led_control_pwm(LED_ON,PWM_LEDBAT_CHANNEL_0);
        }
        else
        {
            /*LED BATTERY OFF During 1900ms*/
            led_control_pwm(LED_OFF,PWM_LEDBAT_CHANNEL_0);
        }
    }
    else if (motion_tracking_flag_gstr.bat_upper_voltcharge)
    {
        /*LED BATTERY ON 100%*/
        led_control_pwm(LED_ON,PWM_LEDBAT_CHANNEL_0);
    }
    /*Indicate: NOT ADAPTOR but BATTERY is Normal*/
    else
    {
        if ((tracking_variables_gtr.counter_ctrl_pwm_ledbat == 0) || (tracking_variables_gtr.counter_ctrl_pwm_ledbat == 2))
        {
            led_control_pwm(LED_ON, PWM_LEDBAT_CHANNEL_0);
        }
        else
        {
            /*LED BATTERY OFF During others*/
            led_control_pwm(LED_OFF, PWM_LEDBAT_CHANNEL_0);
        }
    }
}

/*================================================================================================*/
/**
* @function name  capture_user_buttons
* @brief          Check status level of button and Enable Flag button user
* @parameter      none
* @return value   none
* @note           none
*/
static void capture_user_buttons(void)
{
    Sw0.Input = gpio_get_level(BUTTON_USER0);
    DIGITAL_INTEGRAL_FILTER_V3(Sw0);
    //EdgeSw0.StateNew = Sw0.Output;
    //FALLING_EDGE_TRIGGER_DETECTION(EdgeSw0);

    Sw1.Input = gpio_get_level(BUTTON_USER1);
    DIGITAL_INTEGRAL_FILTER_V3(Sw1);
    EdgeSw1.StateNew = Sw1.Output;
    FALLING_EDGE_TRIGGER_DETECTION(EdgeSw1);

    M0.Input = gpio_get_level(MODE_0_GPIO);
    DIGITAL_INTEGRAL_FILTER_V3(M0);
//    M1.Input = gpio_get_level(MODE_1_GPIO);
//    DIGITAL_INTEGRAL_FILTER_V3(M1);

    /*Check Button User0*/
    if (Sw0.Output == 0)
    {
        /*pressed*/
        if ((tracking_variables_gtr.counter_userbutton0_delay >= BUTTON_USER0_DELAY) && (motion_tracking_flag_gstr.button_user0_down))
        {
            /*Button was pressed during more than 3 seconds*/
            if (tracking_variables_gtr.state_machine == FREEZE_STATE)
            {
                /*Switch to BECON State*/
                tracking_variables_gtr.state_machine = BEACON_STATE;
                motion_tracking_flag_gstr.button_user0_down = false;
                tracking_variables_gtr.counter_userbutton0_delay = 0;
            }
            else if ((tracking_variables_gtr.state_machine == BEACON_STATE) || (tracking_variables_gtr.state_machine == WORKING_STATE))
            {
                /*Switch to FREEZE State*/
                tracking_variables_gtr.state_machine = FREEZE_STATE;
                motion_tracking_flag_gstr.button_user0_down = false;
                tracking_variables_gtr.counter_userbutton0_delay = 0;
            }
            else
            {
                motion_tracking_flag_gstr.button_user0_down = false;
            }
        }
        else
            motion_tracking_flag_gstr.button_user0_down = true;
    }
    /*button UnPress*/
    else
    {
        tracking_variables_gtr.counter_userbutton0_delay = 0;
        motion_tracking_flag_gstr.button_user0_down = false;
    }
}

/*================================================================================================*/
/**
* @function name  init_GPIO_output
* @brief          configure GPIO Output Mode
* @parameter      GPIO: GPIO NUMBER
* @return value   none
* @note           none
*/
void init_GPIO_output(uint8_t GPIO)
{
    gpio_pad_select_gpio(GPIO);
    /* Set the GPIO as a push/pull output */
    ESP_ERROR_CHECK(gpio_set_direction(GPIO, GPIO_MODE_OUTPUT));
}

/*================================================================================================*/
/**
* @function name  init_led_control
* @brief          Configure Led Control (PWM)
* @parameter      GPIO_NUM: Number of GPIO
*                 Channel : Channel of PWM (0->7)
*                 freq    : frequency
* @return value   none
* @note           none
*/
void init_led_control(uint8_t GPIO_NUM, uint8_t Channel, uint32_t freq)
{
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer =
    {
        /*resolution of PWM duty*/
        .duty_resolution = LEDC_TIMER_13_BIT,
        /*frequency of PWM signal*/
        .freq_hz = freq,
        /*timer mode*/
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        /*timer index*/
        .timer_num = LEDC_TIMER_0
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
            .channel = Channel,
            .duty = 0,
            .gpio_num = GPIO_NUM,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .timer_sel = LEDC_TIMER_0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

/*================================================================================================*/
/**
* @function name  led_control_pwm
* @brief
* @parameter      on_off:
*                 channel:
* @return value   none
* @note           none
*/
static void led_control_pwm(bool  on_off, uint8_t channel)
{
    uint32_t duty_u32;

    if (!on_off)
        duty_u32 = PWM_LEDBAT_DUTY_50 * DUTY_RESOLUTION;
    else
        duty_u32 = 100 * DUTY_RESOLUTION;
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, channel, duty_u32);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, channel);
}

/*================================================================================================*/
/**
* @function name  init_adc
* @brief          Configure ADC
* @parameter      resolution: adc_bits_width_t
*                 channel: Channel to get the gpio number
*                 atten: attenuation of ADC value
* @return value   none
* @note           none
*/
void init_adc(uint8_t resolution, uint8_t channel, uint8_t atten)
{
    /*initialize ADC*/
    adc1_config_width(resolution);
    adc1_config_channel_atten(channel, atten);
}

/*================================================================================================*/
/**
* @function name  adaptor_check
* @brief          Charge/Not charge for battery & Led control
* @parameter      none
* @return value   none
* @note           none
*/
static void adaptor_check(void)
{
    /*get adc_raw_value of adaptor*/
    if (!motion_tracking_flag_gstr.request_adc_adaptor)
        return;
    /*-------------------------------------------------------------------------------------------------*/
    motion_tracking_flag_gstr.request_adc_adaptor = 0;

    tracking_variables_gtr.adaptor_adc_raw = adc1_get_raw(ADAPTOR_ADC_CHANNEL);

    if (tracking_variables_gtr.adaptor_adc_raw == (-1))
    {
        motion_tracking_flag_gstr.adc_adaptor_error = ADC_ERROR;
        ESP_LOGE_OP("<ADAPTOR> ", "%s", TAG_ADC_ERROR);
    }
    else
    {
        motion_tracking_flag_gstr.adc_adaptor_error = ADC_OK;
        ESP_LOGI_OP("<ADAPTOR> ", "%s", TAG_ADC_OK);
        /* Volt Adaptor*/
        tracking_variables_gtr.volt_adaptor = roundf(tracking_variables_gtr.adaptor_adc_raw * ADC_CONVERT_VOLT_DAPTOR * 100) / 100;

        ESP_LOGI_OP("<ADAPTOR> <VOLT>", "%f", tracking_variables_gtr.volt_adaptor);

        if (tracking_variables_gtr.adaptor_adc_raw >= volt_adaptor_detect_raw_gu16)
        {
            motion_tracking_flag_gstr.adaptor_plugin = ADAPTOR_PLUGIN;
        }
        else
        {
            motion_tracking_flag_gstr.adaptor_plugin = ADAPTOR_UNPLUGIN;
        }
    }
}
/*================================================================================================*/
/**
* @function name  battery_check
* @brief          battery check
* @parameter      none
* @return value   none
* @note           none
*/
static void battery_check(void)
{
    /*get adc_raw_value of battery*/
    if (!motion_tracking_flag_gstr.request_adc_bat)
        return;
    /*-------------------------------------------------------------------------------------------------*/
    motion_tracking_flag_gstr.request_adc_bat = 0;

    tracking_variables_gtr.bat_adc_raw = adc1_get_raw(BATTERY_ADC_CHANNEL);

    if (tracking_variables_gtr.bat_adc_raw == (-1))
    {
        motion_tracking_flag_gstr.adc_bat_error = ADC_ERROR;
        ESP_LOGE_OP("<BATTERY> ", "%s", TAG_ADC_ERROR);
    }
    else
    {
        motion_tracking_flag_gstr.adc_bat_error = ADC_OK;
        ESP_LOGI_OP("<BATTERY> ", "%s", TAG_ADC_OK);
        /* Volt Adaptor*/
        tracking_variables_gtr.volt_battery = roundf(tracking_variables_gtr.bat_adc_raw * ADC_CONVERT_VOLT_BATTERY * 100) / 100;
        ESP_LOGI_OP("<BATTERY> <VOLT>", "%f", tracking_variables_gtr.volt_battery);
        if (tracking_variables_gtr.bat_adc_raw >= volt_battery_uppercharge_raw_gu16)
        {
            /*Battery is full*/
            motion_tracking_flag_gstr.bat_upper_voltcharge = true;
            motion_tracking_flag_gstr.bat_under_voltcharge = false;
            motion_tracking_flag_gstr.bat_low = false;
            ESP_LOGI_OP("<BATTERY> <VOLT>", "<FULL>");
        }
        else if (tracking_variables_gtr.bat_adc_raw <= volt_battery_low_raw_gu16)
        {
            /*battery is LOW*/
            motion_tracking_flag_gstr.bat_upper_voltcharge = false;
            motion_tracking_flag_gstr.bat_under_voltcharge = true;
            motion_tracking_flag_gstr.bat_low = true;
            /*Warning*/
            ESP_LOGW_OP("<BATTERY> <VOLT>", "<LOW>");
        }
        else if (tracking_variables_gtr.bat_adc_raw <= volt_battery_undercharge_raw_gu16)
        {
            /*battery Need charge*/
            motion_tracking_flag_gstr.bat_upper_voltcharge = false;
            motion_tracking_flag_gstr.bat_under_voltcharge = true;
            motion_tracking_flag_gstr.bat_low = false;
            ESP_LOGI_OP("<BATTERY> <VOLT>", "<NEED CHARGE>");
        }
        else
        {
            motion_tracking_flag_gstr.bat_upper_voltcharge = false;
            motion_tracking_flag_gstr.bat_under_voltcharge = false;
            motion_tracking_flag_gstr.bat_low = false;
        }
    }
}
/*================================================================================================*/


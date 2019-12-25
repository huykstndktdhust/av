/**
  ******************************************************************************
  *   @file Delay.c
  *   @author SangTN@fsoft.com.vn - FPT Company
  *   @version V1.4
  *   @date 19-11-2014
  *   Final editors: SangTN@fsoft.com.vn
  *   @date:         07-30-2019
  ******************************************************************************
  *   @source
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "Delay.h"
#include "ExternVariablesFunctions.h"

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
void IRAM_ATTR timer_group1_isr(void *para)
{
}
/*
 * Initialize selected timer of the timer group 0
 *
 * timer_idx - the timer number to initialize
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void tg1_timer_init(int timer_idx,
                           bool auto_reload) //, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = 80;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_DIS;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = auto_reload;
    timer_init(TIMER_GROUP_1, timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_1, timer_idx, 0x00000000ULL);
    timer_disable_intr(TIMER_GROUP_1, timer_idx);
    timer_start(TIMER_GROUP_1, timer_idx);
}

void TIM_Delay_Config(void)
{
    tg1_timer_init(TIMER_0, TIMER_AUTORELOAD_DIS); //, 10);//10 second
}
uint64_t CountTimerG1_0 = 0;
void delay_us(uint16_t us)
{
    uint64_t count = us;
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);
    CountTimerG1_0 = 0;
    while (1)
    {
        timer_get_counter_value(TIMER_GROUP_1, TIMER_0, &CountTimerG1_0);
        if (count <= CountTimerG1_0)
            break;
    }
}

inline void delay_ms(uint16_t ms)
{
    //while(ms--)delay_us(1000);
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = ms; //Perform an action every 1 ticks //1000Hz
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
}

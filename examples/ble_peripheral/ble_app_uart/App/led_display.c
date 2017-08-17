#include "led_display.h"
#include "nrf_gpio.h"
#include "app_timer.h"


#define LED_CHANGE_TIMER_MS                         1000
#define APP_TIMER_PRESCALER                         0
#define TIMER_LED_CHANGE_INTERVAL                   APP_TIMER_TICKS(LED_CHANGE_TIMER_MS, APP_TIMER_PRESCALER) 

#define LED_CLOSE_LED_TIMER_MS                      100
#define TIMER_CLOSE_LED_INTERVAL                    APP_TIMER_TICKS(LED_CLOSE_LED_TIMER_MS, APP_TIMER_PRESCALER) 

APP_TIMER_DEF(m_led_frame_change_id);
APP_TIMER_DEF(m_close_led_id);

/* --------------------------------------------------------------------------
* static funtion
*/

/**
* @brief : timer for changing led on and off, change one frame in severial ms
*
* @parain: null
* @paraout: null
*
* @return:null
*/
static void Timer_change_frame_process(void * p_contex)
{
    uint32_t err_code  = 0;
    
    printf("1. enter Timer_change_frame_process fun\r\n");
//    err_code = app_timer_stop(m_close_led_id);
//    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_close_led_id, TIMER_LED_CHANGE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void Timer_close_led_process(void * p_contex)
{
    printf("2. enter Timer_close_led_process fun\r\n");
    
}


void led_init(void)
{
    uint8_t i;
    for(i=0;i<32;i++)
    {
        nrf_gpio_cfg_output(i);
        nrf_gpio_pin_set(i);
    }

}

void led_timer_init(void)
{
    uint32_t err_code  = 0;
    err_code = app_timer_create(&m_led_frame_change_id, APP_TIMER_MODE_REPEATED, Timer_change_frame_process);
    APP_ERROR_CHECK(err_code);    
    err_code = app_timer_create(&m_close_led_id, APP_TIMER_MODE_SINGLE_SHOT, Timer_close_led_process);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_led_frame_change_id, TIMER_LED_CHANGE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}



#include "led_display.h"
#include "nrf_gpio.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "pca10040.h"


#define LED_CHANGE_TIMER_MS                         10
#define APP_TIMER_PRESCALER                         0
#define TIMER_LED_CHANGE_INTERVAL                   APP_TIMER_TICKS(LED_CHANGE_TIMER_MS, APP_TIMER_PRESCALER) 

#define LED_CLOSE_LED_TIMER_MS                      5
#define TIMER_CLOSE_LED_INTERVAL                    APP_TIMER_TICKS(LED_CLOSE_LED_TIMER_MS, APP_TIMER_PRESCALER) 

APP_TIMER_DEF(m_led_frame_change_id);
APP_TIMER_DEF(m_close_led_id);

uint16_t led_chang_time_cnt         = 100;
uint16_t led_close_time_cnt         = 100;
static uint16_t led_change_num      = 0;
static uint16_t led_close_num       = 0;

typedef struct
{
    const uint8_t * pic_frame;
    uint16_t offset;
} bubble_wall_pic_frame;

bubble_wall_pic_frame m_bubble_wall_pic_frame;

typedef struct
{
    uint8_t latch_pin;              // 锁存脚
    uint8_t latch_data_pin;         // 数据脚
} latch_chip_pin;

latch_chip_pin m_latch_chip_pin[32] = 
{
    {.latch_pin = 4,.latch_data_pin = 15}, {GPIO_A7,GPIO_P12},{GPIO_A7,GPIO_P13}, {GPIO_A7,GPIO_P14}, {GPIO_A7,GPIO_P15},{GPIO_A7,GPIO_P16},
    {GPIO_A5,GPIO_P9}, {GPIO_A5,GPIO_P10},{GPIO_A5,GPIO_P11}, {GPIO_A5,GPIO_P12},{GPIO_A5,GPIO_P13}, {GPIO_A5,GPIO_P14},{GPIO_A5,GPIO_P15}, {GPIO_A5,GPIO_P16},
    {GPIO_A6,GPIO_P9}, {GPIO_A6,GPIO_P10},{GPIO_A6,GPIO_P11}, {GPIO_A6,GPIO_P12},{GPIO_A6,GPIO_P13}, {GPIO_A6,GPIO_P14},{GPIO_A6,GPIO_P15}, {GPIO_A6,GPIO_P16},
    {GPIO_A4,GPIO_P9}, {GPIO_A4,GPIO_P10},{GPIO_A4,GPIO_P11}, {GPIO_A4,GPIO_P12},{GPIO_A4,GPIO_P13}, {GPIO_A4,GPIO_P14},{GPIO_A4,GPIO_P15}, {GPIO_A4,GPIO_P16},
};

const uint8_t bubble_wall_pic[BUBBLE_WALL_PIC_LENGTH] =
{
    0x80,0x00,0x00,0x04,0x40,0x00,0x00,0x08,0x20,0x00,0x00,0x10,0x10,0x00,0x00,0x20,
    0x08,0x00,0x00,0x40,0x04,0x00,0x00,0x80,0x02,0x00,0x01,0x00,0x01,0x00,0x02,0x00,
    0x00,0x80,0x04,0x00,0x80,0x40,0x08,0x04,0x40,0x20,0x10,0x08,0x20,0x10,0x20,0x10,
    0x10,0x08,0x40,0x20,0x08,0x04,0x80,0x40,0x04,0x03,0x00,0x80,0x02,0x00,0x01,0x00,
    0x01,0x00,0x02,0x00,0x00,0x80,0x04,0x00,0x80,0x40,0x08,0x04,0x40,0x20,0x10,0x08,
    0x20,0x10,0x20,0x10,0x10,0x08,0x40,0x20,0x08,0x04,0x80,0x40,0x04,0x03,0x00,0x80,
    0x02,0x00,0x01,0x00,0x01,0x00,0x02,0x00,0x00,0x80,0x04,0x00,0x00,0x40,0x08,0x00,
    0x00,0x20,0x10,0x00,0x00,0x10,0x20,0x00,0x00,0x08,0x40,0x00,0x00,0x04,0x80,0x00,
    0x00,0x03,0x00,0x00,0x02,0x00,0x01,0x00,0x04,0x00,0x00,0x80,0x08,0x00,0x00,0x40,
    0x10,0x00,0x00,0x20,0x20,0x00,0x00,0x10,0x40,0x00,0x00,0x08,0x80,0x00,0x00,0x04,
    0x02,0x00,0x01,0x00,0x04,0x00,0x00,0x80,0x08,0x00,0x00,0x40,0x10,0x00,0x00,0x20,
    0x20,0x03,0x00,0x10,0x40,0x04,0x80,0x08,0x80,0x08,0x40,0x04,0x00,0x10,0x20,0x00,
    0x00,0x20,0x10,0x00,0x00,0x40,0x08,0x00,0x00,0x80,0x04,0x00,0x01,0x00,0x02,0x00,
    0x02,0x00,0x01,0x00,0x04,0x03,0x00,0x80,0x08,0x04,0x80,0x40,0x10,0x08,0x40,0x20,
    0x20,0x10,0x20,0x10,0x40,0x20,0x10,0x08,0x80,0x40,0x08,0x04,0x00,0x80,0x04,0x00,
    0x01,0x00,0x02,0x00,0x02,0x00,0x01,0x00,0x04,0x03,0x00,0x80,0x08,0x04,0x80,0x40,
    0x10,0x08,0x40,0x20,0x20,0x10,0x20,0x10,0x40,0x20,0x10,0x08,0x80,0x40,0x08,0x04,
    0x00,0x80,0x04,0x00,0x01,0x00,0x02,0x00,0x02,0x00,0x01,0x00,0x04,0x00,0x00,0x80,
    0x08,0x00,0x00,0x40,0x10,0x00,0x00,0x20,0x20,0x00,0x00,0x10,0x40,0x00,0x00,0x08,
    0x80,0x00,0x00,0x04,0x01,0x00,0x02,0x00,0x02,0x80,0x05,0x00,0x04,0x40,0x08,0x80,
    0x08,0x20,0x10,0x40,0x10,0x10,0x20,0x20,0x20,0x08,0x40,0x10,0x40,0x04,0x80,0x08,
    0x80,0x03,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x03,0x00,0x00,0x00,0x04,0x80,0x00,0x00,0x08,0x40,0x00,0x00,0x10,0x20,0x00,
    0x00,0x20,0x10,0x00,0x00,0x40,0x08,0x00,0x00,0x80,0x04,0x00,0x01,0x00,0x02,0x00,
    0x02,0x00,0x01,0x00,0x04,0x00,0x00,0x80,0x08,0x00,0x00,0x40,0x10,0x00,0x00,0x20,
    0x20,0x00,0x00,0x10,0x40,0x00,0x00,0x08,0x80,0x00,0x00,0x04,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x0F,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x03,0xF0,0x3F,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xFC,0x00,0x00,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0xC0,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3E,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x80,0x01,0xF0,0x00,0x40,0x00,0x00,0x00,0x20,0x00,0x00,0x00,
    0x10,0x00,0x0F,0x80,0x08,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x02,0x00,0x00,0x7C,
    0x01,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x40,0x00,0x00,0x80,0x20,0x00,0x00,
    0x40,0x10,0x00,0x00,0x20,0x08,0x00,0x00,0x10,0x04,0x00,0x00,0x08,0x02,0x00,0x00,
    0x04,0x01,0x00,0x00,0x02,0x00,0x80,0x00,0x01,0x00,0x40,0x00,0x00,0x80,0x20,0x00,
    0x00,0x40,0x10,0x00,0x00,0x20,0x08,0x00,0x00,0x10,0x04,0x00,0x00,0x08,0x02,0x00,
    0x00,0x04,0x01,0x00,0x00,0x02,0x00,0x80,0x00,0x01,0x00,0x40,0x00,0x00,0x80,0x20,
    0x00,0x00,0x40,0x10,0x00,0x00,0x20,0x08,0x00,0x00,0x10,0x04,0x00,0x00,0x08,0x00,
    0x00,0x00,0x04,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x80,
    0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x20,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x08,
    0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00
};

uint8_t bubble_wall_pic_one_frame[32] = {0};


uint8_t latch_gpio_pin[8] = 
{
    GPIO_P9,GPIO_P10,GPIO_P11,GPIO_P12,GPIO_P13,GPIO_P14,GPIO_P15,GPIO_P16
};

uint8_t latch_pin[32] = 
{
    GPIO_A7,GPIO_A7,GPIO_A7,GPIO_A7,GPIO_A7,GPIO_A7,
    GPIO_A5,GPIO_A5,GPIO_A5,GPIO_A5,
    GPIO_A5,GPIO_A5,GPIO_A5,GPIO_A5,
    GPIO_A6,GPIO_A6,GPIO_A6,GPIO_A6,GPIO_A6,GPIO_A6,
    GPIO_A6,GPIO_A6,
    GPIO_A4,GPIO_A4,GPIO_A4,GPIO_A4,GPIO_A4,GPIO_A4,GPIO_A4,GPIO_A4,
    32,32
};


/* --------------------------------------------------------------------------
* static funtion
*/


/**
* @brief : turn_on_led
*
* @parain: i,j-> select led num to turn on
* @paraout: null
*
* @return:null
*/
static void turn_on_led(uint8_t dis_i,uint8_t dis_j)
{
    uint8_t display_num;
    
    display_num = (dis_i+1)*(dis_j+1);
    printf("display_num is:%d \r\n",display_num);
    
    
    // 关闭锁存端口
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
    // 开启指定脚led
    nrf_gpio_pin_set(latch_gpio_pin[dis_j]);
//    nrf_delay_ms(10);
        
    // 开启指定脚的锁存端口
    nrf_gpio_pin_set(latch_pin[display_num-1]);
    if((display_num>=24) && (display_num<=30) )
    {
        printf("display_num is %d\r\n",display_num);
        printf("latch_pin[display_num-1] is %d\r\n",latch_pin[display_num-1]);
        printf("display_num is %d\r\n",display_num);
    }

    return;
}

/**
* @brief : turn_off_led
*
* @parain: null
* @paraout: null
*
* @return:null
*/
static void turn_off_led(void)
{
    uint8_t i=0;

    // 关闭锁存端口
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);

    
    // 显示第一个锁存器控制的几个LED
    for(i=2;i<8;i++)
    {
        nrf_gpio_pin_clear(latch_gpio_pin[i]);
    }
    nrf_gpio_pin_set(GPIO_A7);    
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
    // 显示第二个锁存器控制的几个LED
    for(i=0;i<8;i++)
    {
        nrf_gpio_pin_clear(latch_gpio_pin[i]);
    }
    nrf_gpio_pin_set(GPIO_A5);    
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
    // 显示第三个锁存器控制的几个LED
    for(i=0;i<8;i++)
    {
        nrf_gpio_pin_clear(latch_gpio_pin[i]);
    }
    nrf_gpio_pin_set(GPIO_A6);    
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
    // 显示第四个锁存器控制的几个LED
    for(i=0;i<8;i++)
    {
        nrf_gpio_pin_clear(latch_gpio_pin[i]);
    }
    nrf_gpio_pin_set(GPIO_A4);    
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);

    return;
}


/**
* @brief : display one frame
*
* @parain: one frame data to display
* @paraout: null
*
* @return:null
*/
static void display_one_frame(const uint8_t * data)
{
    uint8_t i=0;
    uint32_t data_frame;
    // 关闭锁存端口
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
    data_frame = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3] << 0);
    
    for(i=0;i<32;i++)
    {
        if(data_frame & (1<<i))
        {
            bubble_wall_pic_one_frame[31-i] = 1;
        }
        else
        {
            bubble_wall_pic_one_frame[31-i] = 0;
        }
    }
    
    // 显示第一个锁存器控制的几个LED
    for(i=2;i<8;i++)
    {
        if(1 == bubble_wall_pic_one_frame[i-2])
        {
            nrf_gpio_pin_set(latch_gpio_pin[i]);
        } 
        else
        {
            nrf_gpio_pin_clear(latch_gpio_pin[i]);
        }
    }
    nrf_gpio_pin_set(GPIO_A7);    
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
    // 显示第二个锁存器控制的几个LED
    for(i=0;i<8;i++)
    {
        if(1 == bubble_wall_pic_one_frame[i+6])
        {
            nrf_gpio_pin_set(latch_gpio_pin[i]);
        } 
        else
        {
            nrf_gpio_pin_clear(latch_gpio_pin[i]);
        }
    }
    nrf_gpio_pin_set(GPIO_A5);    
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
    // 显示第三个锁存器控制的几个LED
    for(i=0;i<8;i++)
    {
        if(1 == bubble_wall_pic_one_frame[i+14])
        {
            nrf_gpio_pin_set(latch_gpio_pin[i]);
        } 
        else
        {
            nrf_gpio_pin_clear(latch_gpio_pin[i]);
        }
    }
    nrf_gpio_pin_set(GPIO_A6);    
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
    // 显示第四个锁存器控制的几个LED
    for(i=0;i<8;i++)
    {
        if(1 == bubble_wall_pic_one_frame[i+22])
        {
            nrf_gpio_pin_set(latch_gpio_pin[i]);
        } 
        else
        {
            nrf_gpio_pin_clear(latch_gpio_pin[i]);
        }
    }
    nrf_gpio_pin_set(GPIO_A4);    
    nrf_gpio_pin_clear(GPIO_A4);
    nrf_gpio_pin_clear(GPIO_A5);
    nrf_gpio_pin_clear(GPIO_A6);
    nrf_gpio_pin_clear(GPIO_A7);
    
}

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
    
    if(led_change_num < led_chang_time_cnt)
    {
        led_change_num ++;
        return;
    }
    else
    {
        led_change_num = 0;
    }
    
    printf("Timer_change_frame_process ----> display one frame\r\n");
    
    m_bubble_wall_pic_frame.pic_frame   = &bubble_wall_pic[m_bubble_wall_pic_frame.offset];
    display_one_frame(m_bubble_wall_pic_frame.pic_frame);
    
    if(m_bubble_wall_pic_frame.offset < BUBBLE_WALL_PIC_LENGTH - 8)
    {
        m_bubble_wall_pic_frame.offset      = m_bubble_wall_pic_frame.offset + 4;
    } else
    {
        m_bubble_wall_pic_frame.offset      = 0;
    }
    

    err_code = app_timer_start(m_close_led_id, TIMER_CLOSE_LED_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
}

static void Timer_close_led_process(void * p_contex)
{
    uint8_t i;
    int8_t j;
    uint32_t err_code;
    if(led_close_num < led_close_time_cnt)
    {
        led_close_num ++;
        return;
    }
    else
    {
        led_close_num = 0;
    }
    
    printf("Timer_close_led_process ----> turn off all led\r\n");
    
    turn_off_led();
    
    err_code = app_timer_stop(m_close_led_id);
    APP_ERROR_CHECK(err_code);

}

static void led_timer_init(void)
{
    uint32_t err_code  = 0;
    err_code = app_timer_create(&m_led_frame_change_id, APP_TIMER_MODE_REPEATED, Timer_change_frame_process);
    APP_ERROR_CHECK(err_code);    
    err_code = app_timer_create(&m_close_led_id, APP_TIMER_MODE_REPEATED, Timer_close_led_process);
    APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_start(m_led_frame_change_id, TIMER_LED_CHANGE_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
    
    m_bubble_wall_pic_frame.offset = 0;
}


void led_init(void)
{
    uint8_t i;
    
//    for(i=0;i<32;i++)
//    {
//        nrf_gpio_cfg_output(i);
//        nrf_gpio_pin_clear(i);
//    }
    
    led_timer_init();
//    for(i=0;i<32;i++)
//    {
//        if((TX_PIN_NUMBER != i) && (RX_PIN_NUMBER != i) && (CTS_PIN_NUMBER != i) && (RTS_PIN_NUMBER != i))
//        {
//            nrf_gpio_cfg_output(i);
//            nrf_gpio_pin_set(i);
//        }
//    }
    // 使用的IO口设置为输出
    nrf_gpio_cfg_output(GPIO_A1);
    nrf_gpio_cfg_output(GPIO_A2);
    nrf_gpio_cfg_output(GPIO_A3);
    nrf_gpio_cfg_output(GPIO_A4);
    nrf_gpio_cfg_output(GPIO_A5);
    nrf_gpio_cfg_output(GPIO_A6);
    nrf_gpio_cfg_output(GPIO_A7);
    
    nrf_gpio_cfg_output(GPIO_P9);
    nrf_gpio_cfg_output(GPIO_P10);
    nrf_gpio_cfg_output(GPIO_P11);
    nrf_gpio_cfg_output(GPIO_P12);
    nrf_gpio_cfg_output(GPIO_P13);
    nrf_gpio_cfg_output(GPIO_P14);
    nrf_gpio_cfg_output(GPIO_P15);
    nrf_gpio_cfg_output(GPIO_P16);
    
    nrf_gpio_pin_set(GPIO_A1);
    nrf_gpio_pin_set(GPIO_A2);
    nrf_gpio_pin_set(GPIO_A3);
    nrf_gpio_pin_set(GPIO_A4);
    nrf_gpio_pin_set(GPIO_A5);
    nrf_gpio_pin_set(GPIO_A6);
    nrf_gpio_pin_set(GPIO_A7);
    
    nrf_gpio_pin_set(GPIO_P9);
    nrf_gpio_pin_set(GPIO_P10);
    nrf_gpio_pin_set(GPIO_P11);
    nrf_gpio_pin_set(GPIO_P12);
    nrf_gpio_pin_set(GPIO_P13);
    nrf_gpio_pin_set(GPIO_P14);
    nrf_gpio_pin_set(GPIO_P15);
    nrf_gpio_pin_set(GPIO_P16);
    

}

// set led_chang_time_cnt used in debug mode
void set_led_chang_time_cnt(uint16_t data)
{
    led_chang_time_cnt = data;
    return;
}

// set led_close_time_cnt used in debug mode
void set_led_close_time_cnt(uint16_t data)
{
    led_close_time_cnt = data;
    return;
}







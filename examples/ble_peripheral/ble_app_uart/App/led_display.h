#ifndef __LED_DISPLAY_H
#define __LED_DISPLAY_H

#include <stdint.h>

#define BUBBLE_WALL_PIC_LENGTH      680
#define GPIO_A1                          28
#define GPIO_A2                          30
#define GPIO_A3                          29
#define GPIO_A4                          31
#define GPIO_A5                          3
#define GPIO_A6                          2
#define GPIO_A7                          4

#define GPIO_P9                          13
#define GPIO_P10                         14
#define GPIO_P11                         15
#define GPIO_P12                         16
#define GPIO_P13                         17
#define GPIO_P14                         18
#define GPIO_P15                         19
#define GPIO_P16                         20



void led_init(void);

#endif      // #ifndef __LED_DISPLAY_H

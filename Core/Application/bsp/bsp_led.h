/**
  ******************************************************************************
  * @file    bsp_led.h
  * @brief   三色 LED 控制头文件
  ******************************************************************************
  */

#ifndef BSP_LED_H
#define BSP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* LED 颜色定义 */
typedef enum {
    LED_RED = 0,
    LED_GREEN,
    LED_BLUE
} led_color_t;

/* LED 状态定义 */
typedef enum {
    LED_STATE_INIT = 0,     // 初始化状态（蓝色闪烁）
    LED_STATE_IDLE,         // 空闲状态（绿色常亮）
    LED_STATE_REMOTE,       // 遥控模式（绿色闪烁）
    LED_STATE_ERROR         // 错误状态（红色闪烁）
} led_state_t;

/* 初始化 LED */
void bsp_led_init(void);

/* 设置单个 LED */
void bsp_led_set(led_color_t color, bool state);

/* 翻转 LED */
void bsp_led_toggle(led_color_t color);

/* 设置 LED 状态（根据系统状态） */
void bsp_led_set_state(led_state_t state);

/* LED 更新任务（在定时器中调用，处理闪烁） */
void bsp_led_update(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LED_H */

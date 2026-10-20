/**
  ******************************************************************************
  * @file    bsp_led.c
  * @brief   三色 LED 控制实现
  ******************************************************************************
  */

#include "bsp_led.h"
#include "gpio.h"

/* LED GPIO 定义（与 CubeMX 配置一致） */
#define LED_RED_PORT        GPIOH
#define LED_RED_PIN         GPIO_PIN_12
#define LED_GREEN_PORT      GPIOH
#define LED_GREEN_PIN       GPIO_PIN_11
#define LED_BLUE_PORT       GPIOH
#define LED_BLUE_PIN        GPIO_PIN_10

/* 闪烁控制 */
static led_state_t current_state = LED_STATE_INIT;
static uint32_t blink_counter = 0;

/**
 * @brief  初始化 LED（GPIO 已由 CubeMX 初始化）
 */
void bsp_led_init(void)
{
    // GPIO 已由 MX_GPIO_Init() 初始化
    // 关闭所有 LED
    bsp_led_set(LED_RED, false);
    bsp_led_set(LED_GREEN, false);
    bsp_led_set(LED_BLUE, false);
}

/**
 * @brief  设置单个 LED
 * @param  color: LED 颜色
 * @param  state: true=点亮, false=熄灭
 */
void bsp_led_set(led_color_t color, bool state)
{
    GPIO_PinState pin_state = state ? GPIO_PIN_SET : GPIO_PIN_RESET;

    switch (color) {
        case LED_RED:
            HAL_GPIO_WritePin(LED_RED_PORT, LED_RED_PIN, pin_state);
            break;
        case LED_GREEN:
            HAL_GPIO_WritePin(LED_GREEN_PORT, LED_GREEN_PIN, pin_state);
            break;
        case LED_BLUE:
            HAL_GPIO_WritePin(LED_BLUE_PORT, LED_BLUE_PIN, pin_state);
            break;
    }
}

/**
 * @brief  翻转 LED
 */
void bsp_led_toggle(led_color_t color)
{
    switch (color) {
        case LED_RED:
            HAL_GPIO_TogglePin(LED_RED_PORT, LED_RED_PIN);
            break;
        case LED_GREEN:
            HAL_GPIO_TogglePin(LED_GREEN_PORT, LED_GREEN_PIN);
            break;
        case LED_BLUE:
            HAL_GPIO_TogglePin(LED_BLUE_PORT, LED_BLUE_PIN);
            break;
    }
}

/**
 * @brief  设置 LED 状态
 */
void bsp_led_set_state(led_state_t state)
{
    current_state = state;
    blink_counter = 0;

    // 关闭所有 LED
    bsp_led_set(LED_RED, false);
    bsp_led_set(LED_GREEN, false);
    bsp_led_set(LED_BLUE, false);
}

/**
 * @brief  LED 更新任务（1kHz 调用）
 */
void bsp_led_update(void)
{
    blink_counter++;

    switch (current_state) {
        case LED_STATE_INIT:
            // 蓝色快速闪烁（5Hz）
            if (blink_counter >= 100) {
                blink_counter = 0;
                bsp_led_toggle(LED_BLUE);
            }
            break;

        case LED_STATE_IDLE:
            // 绿色常亮
            bsp_led_set(LED_GREEN, true);
            break;

        case LED_STATE_REMOTE:
            // 绿色慢速闪烁（2Hz）
            if (blink_counter >= 250) {
                blink_counter = 0;
                bsp_led_toggle(LED_GREEN);
            }
            break;

        case LED_STATE_ERROR:
            // 红色快速闪烁（5Hz）
            if (blink_counter >= 100) {
                blink_counter = 0;
                bsp_led_toggle(LED_RED);
            }
            break;
    }
}

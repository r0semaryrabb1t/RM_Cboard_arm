/**
  ******************************************************************************
  * @file    remote_control.c
  * @brief   遥控器数据解析实现
  ******************************************************************************
  */

#include "remote_control.h"
#include "bsp_remote.h"
#include "robot_config.h"
#include <string.h>

#define RC_OFFLINE_TIMEOUT_MS   100         // 遥控器离线超时 (ms)

static remote_data_t rc_data;

/**
 * @brief  将原始通道值归一化到 [-1, 1]
 */
static float normalize_channel(int16_t raw_value)
{
    float normalized = (float)(raw_value - RC_CH_VALUE_OFFSET) /
                       (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_OFFSET);

    // 限幅
    if (normalized > 1.0f) normalized = 1.0f;
    if (normalized < -1.0f) normalized = -1.0f;

    // 应用死区
    if (fabsf(normalized) < RC_DEADZONE) {
        normalized = 0.0f;
    }

    return normalized;
}

/**
 * @brief  解析 SBUS 帧
 */
static bool parse_sbus_frame(const uint8_t *buf, remote_data_t *data)
{
    // SBUS 帧格式：起始字节 0x0F, 16 通道各 11 bit, 标志位, 结束字节 0x00
    if (buf[0] != 0x0F) {
        return false;  // 起始字节错误
    }

    // 提取通道数据（11 bit per channel）
    int16_t ch[4];
    ch[0] = ((buf[1] | buf[2] << 8) & 0x07FF);
    ch[1] = ((buf[2] >> 3 | buf[3] << 5) & 0x07FF);
    ch[2] = ((buf[3] >> 6 | buf[4] << 2 | buf[5] << 10) & 0x07FF);
    ch[3] = ((buf[5] >> 1 | buf[6] << 7) & 0x07FF);

    // 归一化
    data->ch0 = normalize_channel(ch[0]);
    data->ch1 = normalize_channel(ch[1]);
    data->ch2 = normalize_channel(ch[2]);
    data->ch3 = normalize_channel(ch[3]);

    // 拨杆（通道 5 和 6）
    uint16_t ch4 = ((buf[6] >> 4 | buf[7] << 4) & 0x07FF);
    uint16_t ch5 = ((buf[7] >> 7 | buf[8] << 1 | buf[9] << 9) & 0x07FF);

    // 映射拨杆位置
    if (ch4 < 500) data->sw_left = RC_SW_DOWN;
    else if (ch4 < 1500) data->sw_left = RC_SW_MID;
    else data->sw_left = RC_SW_UP;

    if (ch5 < 500) data->sw_right = RC_SW_DOWN;
    else if (ch5 < 1500) data->sw_right = RC_SW_MID;
    else data->sw_right = RC_SW_UP;

    return true;
}

/**
 * @brief  初始化遥控器
 */
void remote_control_init(void)
{
    memset(&rc_data, 0, sizeof(rc_data));
    rc_data.online = false;
}

/**
 * @brief  更新遥控器数据（主循环调用）
 */
void remote_control_update(void)
{
    uint8_t frame_buf[RC_FRAME_LENGTH];

    // 检查是否有新帧
    if (bsp_remote_get_frame(frame_buf)) {
        if (parse_sbus_frame(frame_buf, &rc_data)) {
            rc_data.online = true;
            rc_data.last_update = HAL_GetTick();
        }
    }

    // 超时检测
    uint32_t now = HAL_GetTick();
    if (rc_data.online && (now - rc_data.last_update) > RC_OFFLINE_TIMEOUT_MS) {
        rc_data.online = false;

        // 清零所有通道
        rc_data.ch0 = 0.0f;
        rc_data.ch1 = 0.0f;
        rc_data.ch2 = 0.0f;
        rc_data.ch3 = 0.0f;
    }
}

/**
 * @brief  获取遥控器数据指针
 */
const remote_data_t* remote_control_get_data(void)
{
    return &rc_data;
}

/**
 * @brief  检查遥控器是否在线
 */
bool remote_control_is_online(void)
{
    return rc_data.online;
}

/**
  ******************************************************************************
  * @file    remote_control.h
  * @brief   遥控器数据解析头文件（NDJ6 SBUS 协议）
  ******************************************************************************
  */

#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* 遥控器数据结构 */
typedef struct {
    // 摇杆通道（归一化到 [-1, 1]）
    float ch0;              // 右横（横滚）
    float ch1;              // 右纵（俯仰）
    float ch2;              // 左纵（油门）
    float ch3;              // 左横（偏航）

    // 拨杆
    uint8_t sw_left;        // 左拨杆（1=上, 2=下, 3=中）
    uint8_t sw_right;       // 右拨杆（1=上, 2=下, 3=中）

    // 状态
    bool online;            // 遥控器在线状态
    uint32_t last_update;   // 上次更新时间戳
} remote_data_t;

/* 初始化遥控器 */
void remote_control_init(void);

/* 更新遥控器数据（在主循环调用） */
void remote_control_update(void);

/* 获取遥控器数据 */
const remote_data_t* remote_control_get_data(void);

/* 检查遥控器是否在线 */
bool remote_control_is_online(void);

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_CONTROL_H */

/**
  ******************************************************************************
  * @file    dm_motor.h
  * @brief   达妙 4310p 电机驱动（MIT 模式）
  * @note    封装 MIT 模式命令、电机反馈解析、状态管理
  ******************************************************************************
  */

#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* 电机数量 */
#define DM_MOTOR_COUNT          2

/* 电机 MIT 模式限制（达妙 4310p） */
#define DM_P_MIN                -12.5f      // 位置最小值 (rad)
#define DM_P_MAX                12.5f       // 位置最大值 (rad)
#define DM_V_MIN                -30.0f      // 速度最小值 (rad/s)
#define DM_V_MAX                30.0f       // 速度最大值 (rad/s)
#define DM_T_MIN                -10.0f      // 扭矩最小值 (N·m)
#define DM_T_MAX                10.0f       // 扭矩最大值 (N·m)
#define DM_KP_MIN               0.0f        // Kp 最小值
#define DM_KP_MAX               500.0f      // Kp 最大值
#define DM_KD_MIN               0.0f        // Kd 最小值
#define DM_KD_MAX               5.0f        // Kd 最大值

/* 电机控制命令 */
typedef struct {
    float position;         // 目标位置 (rad)
    float velocity;         // 目标速度 (rad/s)
    float kp;               // 位置增益
    float kd;               // 速度增益
    float torque;           // 前馈扭矩 (N·m)
} dm_motor_cmd_t;

/* 电机反馈数据 */
typedef struct {
    uint8_t  id;            // 电机 ID
    float    position;      // 当前位置 (rad)
    float    velocity;      // 当前速度 (rad/s)
    float    torque;        // 当前扭矩 (N·m)
    uint8_t  temperature;   // 温度 (℃)
    uint8_t  error;         // 错误代码
    bool     online;        // 是否在线
    uint32_t last_update;   // 上次更新时间戳 (ms)
} dm_motor_fb_t;

/* 初始化电机驱动 */
void dm_motor_init(void);

/* 使能电机 */
bool dm_motor_enable(uint8_t motor_id);

/* 失能电机 */
bool dm_motor_disable(uint8_t motor_id);

/* 发送控制命令（MIT 模式） */
bool dm_motor_send_command(uint8_t motor_id, const dm_motor_cmd_t *cmd);

/* 获取电机反馈数据 */
const dm_motor_fb_t* dm_motor_get_feedback(uint8_t motor_id);

/* 处理接收到的 CAN 消息 */
void dm_motor_process_feedback(const can_msg_t *msg);

/* 检查电机是否在线（超时检测） */
bool dm_motor_is_online(uint8_t motor_id, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* DM_MOTOR_H */

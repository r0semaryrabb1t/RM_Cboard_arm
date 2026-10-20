/**
  ******************************************************************************
  * @file    robot_config.h
  * @brief   机械臂全局配置参数
  * @note    包含电机ID、CAN参数、控制参数等全局常量
  ******************************************************************************
  */

#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ==================== 硬件配置 ==================== */

// CAN 总线配置
#define MOTOR_CAN_INSTANCE      hcan1       // 电机使用 CAN1
#define MOTOR_CAN_BAUDRATE      1000000     // 1Mbps

// 达妙电机 CAN ID
#define MOTOR_JOINT1_ID         0x01        // 关节1（基座）
#define MOTOR_JOINT2_ID         0x02        // 关节2（大臂）

/* ==================== 遥控器配置 ==================== */

// 遥控器数据接收
#define RC_UART_INSTANCE        huart3      // NDJ6 使用 UART3
#define RC_FRAME_LENGTH         18          // SBUS 帧长度
#define RC_RX_BUFFER_SIZE       36          // 双缓冲区大小

// 遥控器通道范围
#define RC_CH_VALUE_MIN         364         // 最小值
#define RC_CH_VALUE_OFFSET      1024        // 中位值
#define RC_CH_VALUE_MAX         1684        // 最大值

// 拨杆位置定义
#define RC_SW_UP                1
#define RC_SW_MID               3
#define RC_SW_DOWN              2

/* ==================== 控制参数 ==================== */

// 摇杆死区（归一化后）
#define RC_DEADZONE             0.05f       // 5% 死区

// 电机控制参数（MIT 模式）
#define MOTOR_KP_DEFAULT        10.0f       // 位置增益
#define MOTOR_KD_DEFAULT        1.0f        // 速度增益
#define MOTOR_MAX_TORQUE        10.0f       // 最大扭矩 (N·m)

// 机械臂运动限制
#define JOINT1_MIN_ANGLE        -3.14f      // 关节1 最小角度 (rad)
#define JOINT1_MAX_ANGLE        3.14f       // 关节1 最大角度 (rad)
#define JOINT2_MIN_ANGLE        -1.57f      // 关节2 最小角度 (rad)
#define JOINT2_MAX_ANGLE        1.57f       // 关节2 最大角度 (rad)

#define JOINT_MAX_VELOCITY      5.0f        // 最大角速度 (rad/s)
#define JOINT_MAX_ACCEL         10.0f       // 最大角加速度 (rad/s²)

// 控制周期
#define CONTROL_FREQ_HZ         1000        // 1kHz 控制频率
#define CONTROL_PERIOD_MS       1           // 1ms 控制周期

/* ==================== 状态机 ==================== */

// 系统状态
typedef enum {
    SYS_STATE_INIT = 0,     // 初始化
    SYS_STATE_IDLE,         // 空闲（电机失能）
    SYS_STATE_REMOTE,       // 遥控模式
    SYS_STATE_ERROR         // 错误状态
} system_state_t;

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_CONFIG_H */

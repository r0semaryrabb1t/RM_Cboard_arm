/**
  ******************************************************************************
  * @file    arm_control.h
  * @brief   机械臂运动控制头文件
  ******************************************************************************
  */

#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "robot_config.h"
#include "remote_control.h"
#include <stdbool.h>

/* 机械臂关节状态 */
typedef struct {
    float position[2];          // 关节角度 [joint1, joint2] (rad)
    float velocity[2];          // 关节速度 (rad/s)
    float target_position[2];   // 目标角度 (rad)
} arm_state_t;

/* 初始化机械臂控制 */
void arm_control_init(void);

/* 更新机械臂控制（1kHz 调用） */
void arm_control_update(const remote_data_t *rc_data);

/* 设置系统状态 */
void arm_control_set_state(system_state_t new_state);

/* 获取系统状态 */
system_state_t arm_control_get_state(void);

/* 获取机械臂状态 */
const arm_state_t* arm_control_get_arm_state(void);

#ifdef __cplusplus
}
#endif

#endif /* ARM_CONTROL_H */

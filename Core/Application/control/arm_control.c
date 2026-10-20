/**
  ******************************************************************************
  * @file    arm_control.c
  * @brief   机械臂运动控制实现
  ******************************************************************************
  */

#include "arm_control.h"
#include "dm_motor.h"
#include <math.h>
#include <string.h>

/* 控制增益映射 */
#define RC_TO_VELOCITY_GAIN     2.0f        // 摇杆到速度的映射增益 (rad/s)

/* 系统状态 */
static system_state_t system_state = SYS_STATE_INIT;
static arm_state_t arm_state;

/* 上一拍遥控器左拨杆状态（用于边沿检测） */
static uint8_t last_sw_left = RC_SW_MID;

/**
 * @brief  限幅关节角度
 */
static float clamp_joint_angle(float angle, uint8_t joint_id)
{
    float min_angle = (joint_id == 0) ? JOINT1_MIN_ANGLE : JOINT2_MIN_ANGLE;
    float max_angle = (joint_id == 0) ? JOINT1_MAX_ANGLE : JOINT2_MAX_ANGLE;

    if (angle < min_angle) return min_angle;
    if (angle > max_angle) return max_angle;
    return angle;
}

/**
 * @brief  初始化机械臂控制
 */
void arm_control_init(void)
{
    memset(&arm_state, 0, sizeof(arm_state));
    system_state = SYS_STATE_IDLE;

    // 初始化目标位置为零点
    arm_state.target_position[0] = 0.0f;
    arm_state.target_position[1] = 0.0f;
}

/**
 * @brief  设置系统状态
 */
void arm_control_set_state(system_state_t new_state)
{
    system_state = new_state;

    // 状态切换时的动作
    switch (new_state) {
        case SYS_STATE_IDLE:
            // 失能电机
            dm_motor_disable(MOTOR_JOINT1_ID);
            dm_motor_disable(MOTOR_JOINT2_ID);
            break;

        case SYS_STATE_REMOTE:
            // 使能电机
            dm_motor_enable(MOTOR_JOINT1_ID);
            dm_motor_enable(MOTOR_JOINT2_ID);
            break;

        default:
            break;
    }
}

/**
 * @brief  获取系统状态
 */
system_state_t arm_control_get_state(void)
{
    return system_state;
}

/**
 * @brief  获取机械臂状态
 */
const arm_state_t* arm_control_get_arm_state(void)
{
    return &arm_state;
}

/**
 * @brief  处理遥控器输入（状态机）
 */
static void process_remote_input(const remote_data_t *rc)
{
    // 右拨杆：系统状态控制
    // 上档 = 遥控模式，中档 = 空闲，下档 = 失能
    switch (rc->sw_right) {
        case RC_SW_UP:
            if (system_state != SYS_STATE_REMOTE) {
                arm_control_set_state(SYS_STATE_REMOTE);
            }
            break;

        case RC_SW_MID:
            if (system_state != SYS_STATE_IDLE) {
                arm_control_set_state(SYS_STATE_IDLE);
            }
            break;

        case RC_SW_DOWN:
            if (system_state != SYS_STATE_IDLE) {
                arm_control_set_state(SYS_STATE_IDLE);
            }
            break;
    }

    }
    last_sw_left = rc->sw_left;
}

/**
 * @brief  遥控模式控制
 */
static void remote_mode_control(const remote_data_t *rc)
{
    // 摇杆映射：
    // ch3 (左横) -> 关节1 (基座旋转)
    // ch1 (右纵) -> 关节2 (大臂俯仰)

    float dt = 0.001f;  // 1ms 控制周期

    // 速度控制：目标角度 += 摇杆输入 * 增益 * dt
    arm_state.target_position[0] += rc->ch3 * RC_TO_VELOCITY_GAIN * dt;
    arm_state.target_position[1] += rc->ch1 * RC_TO_VELOCITY_GAIN * dt;

    // 限幅
    arm_state.target_position[0] = clamp_joint_angle(arm_state.target_position[0], 0);
    arm_state.target_position[1] = clamp_joint_angle(arm_state.target_position[1], 1);

    // 发送电机命令
    dm_motor_cmd_t cmd;

    // 关节1
    cmd.position = arm_state.target_position[0];
    cmd.velocity = 0.0f;
    cmd.kp = MOTOR_KP_DEFAULT;
    cmd.kd = MOTOR_KD_DEFAULT;
    cmd.torque = 0.0f;
    dm_motor_send_command(MOTOR_JOINT1_ID, &cmd);

    // 关节2
    cmd.position = arm_state.target_position[1];
    cmd.velocity = 0.0f;
    cmd.kp = MOTOR_KP_DEFAULT;
    cmd.kd = MOTOR_KD_DEFAULT;
    cmd.torque = 0.0f;
    dm_motor_send_command(MOTOR_JOINT2_ID, &cmd);
}

/**
 * @brief  更新电机反馈到机械臂状态
 */
static void update_arm_feedback(void)
{
    const dm_motor_fb_t *fb1 = dm_motor_get_feedback(MOTOR_JOINT1_ID);
    const dm_motor_fb_t *fb2 = dm_motor_get_feedback(MOTOR_JOINT2_ID);

    if (fb1 != NULL) {
        arm_state.position[0] = fb1->position;
        arm_state.velocity[0] = fb1->velocity;
    }

    if (fb2 != NULL) {
        arm_state.position[1] = fb2->position;
        arm_state.velocity[1] = fb2->velocity;
    }
}

/**
 * @brief  机械臂控制主循环（1kHz）
 */
void arm_control_update(const remote_data_t *rc_data)
{
    // 处理遥控器输入（状态机）
    if (rc_data != NULL && rc_data->online) {
        process_remote_input(rc_data);
    } else {
        // 遥控器离线：切换到空闲状态
        if (system_state != SYS_STATE_IDLE) {
            arm_control_set_state(SYS_STATE_IDLE);
        }
    }

    // 根据系统状态执行控制
    switch (system_state) {
        case SYS_STATE_REMOTE:
            if (rc_data != NULL && rc_data->online) {
                remote_mode_control(rc_data);
            }
            break;

        case SYS_STATE_IDLE:
            // 保持失能状态
            break;

        default:
            break;
    }

    // 更新机械臂状态
    update_arm_feedback();
}

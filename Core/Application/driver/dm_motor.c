/**
  ******************************************************************************
  * @file    dm_motor.c
  * @brief   达妙 4310p 电机驱动实现
  ******************************************************************************
  */

#include "dm_motor.h"
#include "bsp_can.h"
#include "robot_config.h"
#include <string.h>
#include <math.h>

/* CAN 命令码 */
#define DM_CMD_MOTOR_MODE       0x01        // 进入电机模式
#define DM_CMD_RESET_MODE       0x02        // 复位模式
#define DM_CMD_ZERO_POSITION    0x03        // 设置零点
#define DM_CMD_MIT_CTRL         0xFF        // MIT 控制模式

/* 电机状态 */
static dm_motor_fb_t motor_fb[DM_MOTOR_COUNT];
static const uint8_t motor_ids[DM_MOTOR_COUNT] = {
    MOTOR_JOINT1_ID,
    MOTOR_JOINT2_ID
};

/* 浮点数限幅 */
static inline float clamp(float value, float min, float max)
{
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/* 浮点数转换为 uint（MIT 模式编码） */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    float x_clamped = clamp(x, x_min, x_max);
    return (uint16_t)((x_clamped - offset) * ((float)((1 << bits) - 1)) / span);
}

/* uint 转换为浮点数（MIT 模式解码） */
static float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  初始化电机驱动
 */
void dm_motor_init(void)
{
    memset(motor_fb, 0, sizeof(motor_fb));

    // 初始化电机 ID
    for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++) {
        motor_fb[i].id = motor_ids[i];
        motor_fb[i].online = false;
    }
}

/**
 * @brief  使能电机（进入电机模式）
 */
bool dm_motor_enable(uint8_t motor_id)
{
    can_msg_t msg = {
        .id = motor_id,
        .dlc = 8,
        .is_extended = false
    };

    msg.data[0] = 0xFF;
    msg.data[1] = 0xFF;
    msg.data[2] = 0xFF;
    msg.data[3] = 0xFF;
    msg.data[4] = 0xFF;
    msg.data[5] = 0xFF;
    msg.data[6] = 0xFF;
    msg.data[7] = 0xFC;  // 进入电机模式

    return bsp_can_send(&MOTOR_CAN_INSTANCE, &msg, 10);
}

/**
 * @brief  失能电机（退出电机模式）
 */
bool dm_motor_disable(uint8_t motor_id)
{
    can_msg_t msg = {
        .id = motor_id,
        .dlc = 8,
        .is_extended = false
    };

    msg.data[0] = 0xFF;
    msg.data[1] = 0xFF;
    msg.data[2] = 0xFF;
    msg.data[3] = 0xFF;
    msg.data[4] = 0xFF;
    msg.data[5] = 0xFF;
    msg.data[6] = 0xFF;
    msg.data[7] = 0xFD;  // 退出电机模式

    return bsp_can_send(&MOTOR_CAN_INSTANCE, &msg, 10);
}

/**
 * @brief  发送 MIT 控制命令
 */
bool dm_motor_send_command(uint8_t motor_id, const dm_motor_cmd_t *cmd)
{
    // MIT 模式编码
    uint16_t p_int = float_to_uint(cmd->position, DM_P_MIN, DM_P_MAX, 16);
    uint16_t v_int = float_to_uint(cmd->velocity, DM_V_MIN, DM_V_MAX, 12);
    uint16_t kp_int = float_to_uint(cmd->kp, DM_KP_MIN, DM_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(cmd->kd, DM_KD_MIN, DM_KD_MAX, 12);
    uint16_t t_int = float_to_uint(cmd->torque, DM_T_MIN, DM_T_MAX, 12);

    can_msg_t msg = {
        .id = motor_id,
        .dlc = 8,
        .is_extended = false
    };

    msg.data[0] = (p_int >> 8) & 0xFF;
    msg.data[1] = p_int & 0xFF;
    msg.data[2] = (v_int >> 4) & 0xFF;
    msg.data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);
    msg.data[4] = kp_int & 0xFF;
    msg.data[5] = (kd_int >> 4) & 0xFF;
    msg.data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);
    msg.data[7] = t_int & 0xFF;

    return bsp_can_send(&MOTOR_CAN_INSTANCE, &msg, 10);
}

/**
 * @brief  处理电机反馈（CAN 接收回调）
 */
void dm_motor_process_feedback(const can_msg_t *msg)
{
    // 查找对应电机
    uint8_t idx = 0xFF;
    for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++) {
        if (motor_ids[i] == msg->id) {
            idx = i;
            break;
        }
    }

    if (idx == 0xFF || msg->dlc != 8) {
        return;  // 未知电机或数据长度错误
    }

    // 解码反馈数据
    uint16_t p_int = (msg->data[0] << 8) | msg->data[1];
    uint16_t v_int = (msg->data[2] << 4) | (msg->data[3] >> 4);
    uint16_t t_int = ((msg->data[3] & 0x0F) << 8) | msg->data[4];

    motor_fb[idx].position = uint_to_float(p_int, DM_P_MIN, DM_P_MAX, 16);
    motor_fb[idx].velocity = uint_to_float(v_int, DM_V_MIN, DM_V_MAX, 12);
    motor_fb[idx].torque = uint_to_float(t_int, DM_T_MIN, DM_T_MAX, 12);
    motor_fb[idx].temperature = msg->data[5];
    motor_fb[idx].error = msg->data[6];
    motor_fb[idx].online = true;
    motor_fb[idx].last_update = HAL_GetTick();
}

/**
 * @brief  获取电机反馈
 */
const dm_motor_fb_t* dm_motor_get_feedback(uint8_t motor_id)
{
    for (uint8_t i = 0; i < DM_MOTOR_COUNT; i++) {
        if (motor_ids[i] == motor_id) {
            return &motor_fb[i];
        }
    }
    return NULL;
}

/**
 * @brief  检查电机是否在线
 */
bool dm_motor_is_online(uint8_t motor_id, uint32_t timeout_ms)
{
    const dm_motor_fb_t *fb = dm_motor_get_feedback(motor_id);
    if (fb == NULL) {
        return false;
    }

    uint32_t now = HAL_GetTick();
    return (fb->online && (now - fb->last_update) < timeout_ms);
}

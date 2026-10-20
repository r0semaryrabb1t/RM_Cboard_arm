/**
  ******************************************************************************
  * @file    system_task.c
  * @brief   系统任务调度实现
  ******************************************************************************
  */

#include "system_task.h"
#include "tim.h"
#include "bsp_can.h"
#include "dm_motor.h"
#include "remote_control.h"
#include "arm_control.h"
#include "robot_config.h"

/**
 * @brief  启动系统任务（配置 1kHz 定时器）
 */
void system_task_start(void)
{
    // 启动 TIM6（假设 TIM6 配置为 1kHz）
    // 注：需要在 CubeMX 中配置 TIM6 为 1ms 周期
    HAL_TIM_Base_Start_IT(&htim6);
}

/**
 * @brief  1kHz 控制周期（由定时器中断调用）
 */
void system_task_1ms(void)
{
    // 1. 更新遥控器数据
    remote_control_update();

    // 2. 获取遥控器数据
    const remote_data_t *rc_data = remote_control_get_data();

    // 3. 执行机械臂控制
    arm_control_update(rc_data);
}

/**
 * @brief  处理 CAN 接收队列（主循环轮询）
 */
void system_task_process_can(void)
{
    can_msg_t msg;

    // 批量处理接收队列
    while (bsp_can_receive(&MOTOR_CAN_INSTANCE, &msg)) {
        dm_motor_process_feedback(&msg);
    }
}

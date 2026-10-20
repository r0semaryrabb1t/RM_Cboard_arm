/**
  ******************************************************************************
  * @file    bsp_can.h
  * @brief   CAN 总线板级支持包
  * @note    封装 CAN 收发、滤波器配置、中断接收队列
  ******************************************************************************
  */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* CAN 接收队列配置 */
#define CAN_RX_QUEUE_SIZE       64          // 接收队列深度

/* CAN 消息结构 */
typedef struct {
    uint32_t id;                            // CAN ID
    uint8_t  data[8];                       // 数据负载
    uint8_t  dlc;                           // 数据长度
    bool     is_extended;                   // 是否扩展帧
} can_msg_t;

/* 初始化 CAN 总线（滤波器 + 启动接收） */
void bsp_can_init(void);

/* 发送 CAN 消息（阻塞） */
bool bsp_can_send(CAN_HandleTypeDef *hcan, const can_msg_t *msg, uint32_t timeout_ms);

/* 从接收队列获取消息（非阻塞） */
bool bsp_can_receive(CAN_HandleTypeDef *hcan, can_msg_t *msg);

/* 获取接收队列中待处理消息数量 */
uint32_t bsp_can_get_rx_count(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_H */

/**
  ******************************************************************************
  * @file    bsp_can.c
  * @brief   CAN 总线板级支持包实现
  ******************************************************************************
  */

#include "bsp_can.h"
#include "can.h"
#include <string.h>

/* CAN1 接收队列 */
static can_msg_t can1_rx_queue[CAN_RX_QUEUE_SIZE];
static volatile uint32_t can1_rx_head = 0;  // 写指针
static volatile uint32_t can1_rx_tail = 0;  // 读指针

/**
 * @brief  配置 CAN 滤波器（接收所有消息）
 */
static void can_filter_config(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter_config;

    filter_config.FilterBank = 0;  // CAN1 使用 FilterBank 0
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_config.FilterIdHigh = 0x0000;
    filter_config.FilterIdLow = 0x0000;
    filter_config.FilterMaskIdHigh = 0x0000;
    filter_config.FilterMaskIdLow = 0x0000;
    filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter_config.FilterActivation = ENABLE;
    filter_config.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(hcan, &filter_config);
}

/**
 * @brief  初始化 CAN 总线
 */
void bsp_can_init(void)
{
    // 配置 CAN1 滤波器
    can_filter_config(&hcan1);

    // 启动 CAN1
    HAL_CAN_Start(&hcan1);

    // 使能接收中断
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
 * @brief  发送 CAN 消息
 */
bool bsp_can_send(CAN_HandleTypeDef *hcan, const can_msg_t *msg, uint32_t timeout_ms)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if (msg->is_extended) {
        tx_header.IDE = CAN_ID_EXT;
        tx_header.ExtId = msg->id;
    } else {
        tx_header.IDE = CAN_ID_STD;
        tx_header.StdId = msg->id;
    }

    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = msg->dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, &tx_header,
                                                     (uint8_t *)msg->data,
                                                     &tx_mailbox);

    return (status == HAL_OK);
}

/**
 * @brief  从队列获取接收消息
 */
bool bsp_can_receive(CAN_HandleTypeDef *hcan, can_msg_t *msg)
{
    if (hcan->Instance != CAN1) {
        return false;  // 当前仅支持 CAN1
    }

    // 检查队列是否为空
    if (can1_rx_tail == can1_rx_head) {
        return false;
    }

    // 读取消息
    memcpy(msg, (void *)&can1_rx_queue[can1_rx_tail], sizeof(can_msg_t));

    // 更新读指针
    can1_rx_tail = (can1_rx_tail + 1) % CAN_RX_QUEUE_SIZE;

    return true;
}

/**
 * @brief  获取队列中待处理消息数量
 */
uint32_t bsp_can_get_rx_count(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) {
        return 0;
    }

    if (can1_rx_head >= can1_rx_tail) {
        return can1_rx_head - can1_rx_tail;
    } else {
        return CAN_RX_QUEUE_SIZE - can1_rx_tail + can1_rx_head;
    }
}

/**
 * @brief  CAN 接收中断回调（HAL 库调用）
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) {
        return;
    }

    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    // 检查队列是否满
    uint32_t next_head = (can1_rx_head + 1) % CAN_RX_QUEUE_SIZE;
    if (next_head == can1_rx_tail) {
        return;  // 队列满，丢弃消息
    }

    // 写入队列
    can_msg_t *msg = (can_msg_t *)&can1_rx_queue[can1_rx_head];
    msg->is_extended = (rx_header.IDE == CAN_ID_EXT);
    msg->id = msg->is_extended ? rx_header.ExtId : rx_header.StdId;
    msg->dlc = rx_header.DLC;
    memcpy(msg->data, rx_data, 8);

    can1_rx_head = next_head;
}

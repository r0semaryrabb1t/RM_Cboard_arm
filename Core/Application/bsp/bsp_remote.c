/**
  ******************************************************************************
  * @file    bsp_remote.c
  * @brief   遥控器 UART+DMA 接收实现（NDJ6 SBUS 协议）
  ******************************************************************************
  */

#include "bsp_remote.h"
#include "usart.h"
#include <string.h>

/* 双缓冲区 DMA 接收 */
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_SIZE];
static uint8_t sbus_rx_frame[RC_FRAME_LENGTH];
static volatile bool frame_ready = false;

/**
 * @brief  初始化遥控器 UART 接收
 */
void bsp_remote_init(void)
{
    // 使能 UART 空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    // 启动 DMA 循环接收
    HAL_UART_Receive_DMA(&huart3, sbus_rx_buf[0], SBUS_RX_BUF_SIZE);
}

/**
 * @brief  检查是否有新帧就绪
 */
bool bsp_remote_frame_ready(void)
{
    return frame_ready;
}

/**
 * @brief  获取最新帧数据
 * @param  buf: 输出缓冲区（至少 RC_FRAME_LENGTH 字节）
 * @return 是否成功获取
 */
bool bsp_remote_get_frame(uint8_t *buf)
{
    if (!frame_ready) {
        return false;
    }

    memcpy(buf, (void *)sbus_rx_frame, RC_FRAME_LENGTH);
    frame_ready = false;

    return true;
}

/**
 * @brief  UART 空闲中断处理（在 stm32f4xx_it.c 中调用）
 */
void bsp_remote_uart_idle_handler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) {
        // 清除空闲中断标志
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);

        // 停止 DMA
        HAL_UART_DMAStop(&huart3);

        // 计算接收长度
        uint32_t recv_len = SBUS_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

        // 验证帧长度
        if (recv_len == RC_FRAME_LENGTH) {
            memcpy((void *)sbus_rx_frame, sbus_rx_buf[0], RC_FRAME_LENGTH);
            frame_ready = true;
        }

        // 重启 DMA 接收
        HAL_UART_Receive_DMA(&huart3, sbus_rx_buf[0], SBUS_RX_BUF_SIZE);
    }
}

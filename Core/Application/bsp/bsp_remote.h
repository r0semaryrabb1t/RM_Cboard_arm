/**
  ******************************************************************************
  * @file    bsp_remote.h
  * @brief   遥控器 UART+DMA 接收头文件
  ******************************************************************************
  */

#ifndef BSP_REMOTE_H
#define BSP_REMOTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define SBUS_RX_BUF_SIZE        36          // 接收缓冲区大小
#define RC_FRAME_LENGTH         18          // SBUS 帧长度

/* 初始化遥控器接收 */
void bsp_remote_init(void);

/* 检查是否有新帧 */
bool bsp_remote_frame_ready(void);

/* 获取最新帧数据 */
bool bsp_remote_get_frame(uint8_t *buf);

/* UART 空闲中断处理函数 */
void bsp_remote_uart_idle_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_REMOTE_H */

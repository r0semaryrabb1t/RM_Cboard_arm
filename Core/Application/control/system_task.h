/**
  ******************************************************************************
  * @file    system_task.h
  * @brief   系统任务调度头文件
  ******************************************************************************
  */

#ifndef SYSTEM_TASK_H
#define SYSTEM_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 启动系统任务（1kHz 定时器） */
void system_task_start(void);

/* 1ms 控制周期回调（由定时器中断触发） */
void system_task_1ms(void);

/* 处理 CAN 接收队列（在主循环调用） */
void system_task_process_can(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_TASK_H */

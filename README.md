# RM C板机械臂控制项目

基于大疆 RoboMaster C 型开发板的两自由度机械臂控制系统。

## 硬件配置

- **主控**：大疆 RoboMaster C 型开发板（STM32F407IGH6）
- **电机**：2 × 达妙 4310p 电机（CAN1 总线）
  - 关节 1（基座）：CAN ID = 0x01
  - 关节 2（大臂）：CAN ID = 0x02
- **遥控器**：大疆 NDJ6 遥控器（UART3 SBUS 协议）
- **LED 指示**：三色 LED（PH10/11/12）

## 引脚配置

| 外设 | 引脚 | 功能 |
|------|------|------|
| CAN1 | PD0 (RX), PD1 (TX) | 电机通信（1Mbps） |
| CAN2 | PB5 (RX), PB6 (TX) | 备用 |
| USART3 | PC11 (RX), PC10 (TX) | 遥控器接收（100000bps, 8E2） |
| LED_RED | PH12 | 错误指示 |
| LED_GREEN | PH11 | 正常运行 |
| LED_BLUE | PH10 | 初始化 |
| TIM6 | - | 1kHz 控制周期 |

## 代码架构

```
Core/Application/
├── bsp/                  # 板级支持包
│   ├── bsp_can          # CAN 收发封装
│   ├── bsp_remote       # 遥控器 UART+DMA
│   └── bsp_led          # LED 状态指示
│
├── driver/              # 设备驱动层
│   ├── dm_motor         # 达妙电机 MIT 协议
│   └── remote_control   # 遥控器数据解析
│
└── control/             # 控制逻辑层
    ├── robot_config.h   # 全局配置参数
    ├── arm_control      # 机械臂运动控制
    └── system_task      # 任务调度
```

**分层设计**：BSP → Driver → Control，职责清晰，易于维护。

## 快速开始

### 1. 用 STM32CubeMX 打开项目

```bash
# 打开 robotic_arm.ioc
# Project Manager → Toolchain/IDE: STM32CubeIDE
# 点击 Generate Code
```

### 2. 集成应用层代码

在 CubeMX 生成代码后，`Core/Application/` 已经在位，需要在 `main.c` 中集成：

```c
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "bsp_remote.h"
#include "bsp_led.h"
#include "dm_motor.h"
#include "remote_control.h"
#include "arm_control.h"
#include "system_task.h"
/* USER CODE END Includes */

/* USER CODE BEGIN 2 */
bsp_can_init();
bsp_remote_init();
bsp_led_init();
dm_motor_init();
remote_control_init();
arm_control_init();
system_task_start();
/* USER CODE END 2 */

/* USER CODE BEGIN WHILE */
while (1)
{
    system_task_process_can();
    /* USER CODE END WHILE */
}
```

在 `stm32f4xx_it.c` 中添加：

```c
/* USER CODE BEGIN Includes */
#include "bsp_remote.h"
#include "system_task.h"
#include "bsp_led.h"
/* USER CODE END Includes */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        system_task_1ms();
        bsp_led_update();  // LED 闪烁更新
    }
}

void USART3_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)) {
        bsp_remote_uart_idle_handler();
    }
    HAL_UART_IRQHandler(&huart3);
}
```

### 3. 编译烧录

使用 STM32CubeIDE 或 probe-rs：

```bash
# 使用 probe-rs
probe-rs run --chip STM32F407IGHx build/robotic_arm.elf
```

## 遥控器映射

### 摇杆控制

- **左横（CH3）**：关节 1（基座旋转）
- **右纵（CH1）**：关节 2（大臂俯仰）

### 拨杆控制

- **右拨杆**：
  - 上档：遥控模式（电机使能，绿色LED闪烁）
  - 中档：空闲模式（电机失能，绿色LED常亮）
  - 下档：空闲模式
  
- **左拨杆**：预留

## LED 状态指示

| LED 状态 | 含义 |
|---------|------|
| 蓝色快闪 | 系统初始化中 |
| 绿色常亮 | 空闲模式（电机失能） |
| 绿色慢闪 | 遥控模式（电机使能） |
| 红色快闪 | 错误状态 |

## 硬件连接

### 电机接线（CAN1）

| 电机 | 线色 | 连接到 |
|------|------|--------|
| 关节1 | 红 | CAN1_H (PD0) |
|      | 黑 | CAN1_L (PD1) |
|      | 黄 | 24V+ |
|      | 白 | GND |
| 关节2 | 同上 | 并联到 CAN1 |

**注意**：设置电机 ID（关节1 = 0x01, 关节2 = 0x02）

### 遥控器接线（UART3）

| 遥控器接收器 | C 板 |
|-------------|------|
| VCC | 5V |
| GND | GND |
| SBUS | UART3_RX (PC11) |

## 参数调整

编辑 `Core/Application/control/robot_config.h`：

```c
// 电机控制参数
#define MOTOR_KP_DEFAULT        10.0f       // 位置增益
#define MOTOR_KD_DEFAULT        1.0f        // 速度增益

// 关节角度限位
#define JOINT1_MIN_ANGLE        -3.14f      // -180°
#define JOINT1_MAX_ANGLE        3.14f       // +180°
#define JOINT2_MIN_ANGLE        -1.57f      // -90°
#define JOINT2_MAX_ANGLE        1.57f       // +90°

// 遥控器死区
#define RC_DEADZONE             0.05f       // 5%
```

编辑 `arm_control.c` 中的速度映射增益：

```c
#define RC_TO_VELOCITY_GAIN     2.0f        // rad/s
```

## 常见问题

### 遥控器无响应
1. 检查 UART3 配置：100000 bps, 8E2
2. 验证接收器接线（VCC, GND, SBUS）
3. 确认遥控器已对码

### 电机不转
1. 检查 CAN1 配置：1 Mbps
2. 确认电机 ID 正确（0x01, 0x02）
3. 验证右拨杆在上档（遥控模式）
4. 检查 24V 电源

### LED 不亮
1. 确认 GPIO 初始化（PH10/11/12）
2. 检查 `bsp_led_init()` 是否调用
3. 验证 `bsp_led_update()` 在定时器中断中调用

## 控制流程

```
1kHz 定时器中断
    ↓
system_task_1ms()
    ↓
remote_control_update() → 解析 SBUS 帧
    ↓
arm_control_update() → 状态机 + 运动控制
    ↓
dm_motor_send_command() → MIT 命令下发
    ↓
CAN1 发送
```

## 开发参考

- 基于 [RM-C-board-volleyball](../RM-C-board-volleyball) 项目改进
- MIT 模式电机控制：位置/速度/扭矩三环
- SBUS 协议：11-bit 通道数据，100000 bps
- 控制周期：1kHz（1ms）

## License

本项目代码仅供学习参考使用。

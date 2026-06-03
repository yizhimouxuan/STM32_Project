# STM32 智能小车项目

基于 STM32F446RET6 的智能循迹/避障小车工程，集成 RPLidar C1 激光雷达、MPU6050 姿态传感器、蓝牙无线调试与 OLED 显示，用于迷宫导航与自主避障算法的验证与开发。

---

## 📋 目录

- [项目简介](#项目简介)
- [硬件平台](#硬件平台)
- [功能特性](#功能特性)
- [项目结构](#项目结构)
- [软件架构](#软件架构)
- [构建与烧录](#构建与烧录)
- [调试接口](#调试接口)
- [关键文件说明](#关键文件说明)

---

## 项目简介

本项目以 STM32F4 微控制器为核心，驱动一台配备差速轮系的智能小车。系统通过激光雷达实时感知周围环境距离信息，结合 MPU6050 的 DMP 姿态解算，实现迷宫路径跟踪、自主转弯与障碍物规避。所有调试信息可通过蓝牙串口实时回传至 PC 或手机端，也可通过车载 OLED 屏本地显示。

---

## 硬件平台

| 组件 | 型号/规格 | 说明 |
|------|-----------|------|
| **主控 MCU** | STM32F446RET6 | ARM Cortex-M4，180 MHz |
| **激光雷达** | RPLidar C1 | UART 通信，波特率 460800 |
| **姿态传感器** | MPU6050 | I2C 接口，内置 DMP |
| **电机驱动** | TB6612FNG / 类似 H 桥 | PWM 调速 + 方向控制 |
| **编码器** | 霍尔增量式编码器 | 测速与里程计反馈 |
| **蓝牙模块** | HC-05 / 类似串口蓝牙 | USART1，无线调试 |
| **显示屏** | SSD1306 OLED | 128×64，I2C 接口 |
| **供电** | 7.4V / 12V 锂电池 | 降压至 5V/3.3V |

### 引脚/外设分配

| 外设 | 功能 | 备注 |
|------|------|------|
| USART3 + DMA | RPLidar C1 数据接收 | 循环接收，IDLE 中断 |
| USART1 | 蓝牙串口调试 | 打印日志、远程指令 |
| I2C1 | MPU6050 + OLED | 共享总线 |
| TIM1/TIM8 | 电机 PWM 输出 | 4 路 PWM |
| TIM2/TIM3 | 编码器输入 | 正交解码 |
| GPIO | 电机方向、LED、蜂鸣器等 | |

---

## 功能特性

- **激光雷达测距**
  - 通过 USART3 + DMA 持续接收 RPLidar C1 扫描数据
  - 流式字节解析，支持跨帧残留拼接
  - 提取前、后、左、右及左前、右前、左后、右后 8 个扇区距离

- **迷宫循迹与避障**
  - 基于距离传感器的有限状态机（FSM）控制逻辑
  - 直行、转弯、掉头、沿墙走等基础行为
  - 距离滤波与障碍物阈值判断

- **姿态感知**
  - MPU6050 原始数据读取
  - DMP 数字运动处理器解算 yaw/pitch/roll
  - 用于转弯角度闭环控制

- **电机控制**
  - PID 速度闭环 / 开环 PWM 控制
  - 编码器测速反馈
  - 差速转向逻辑

- **调试与显示**
  - 蓝牙串口实时输出：距离、姿态、状态机、PID 参数等
  - OLED 本地显示关键运行信息
  - 支持运行时通过蓝牙指令调整参数

---

## 项目结构

```text
STM32_new/
├── Core/                       # HAL 库自动生成 + 用户代码
│   ├── Inc/                    # 核心头文件 (main.h, stm32f4xx_it.h 等)
│   └── Src/                    # 核心源文件
│       ├── main.c              # 主循环、状态机、控制逻辑
│       ├── usart.c             # USART1/3 初始化，DMA 接收配置
│       ├── i2c.c               # I2C1 初始化
│       ├── tim.c               # 定时器初始化 (PWM + 编码器)
│       ├── gpio.c              # GPIO 初始化
│       ├── stm32f4xx_it.c      # 中断服务函数
│       └── ...
├── BSP/                        # 板级支持包 (Board Support Package)
│   ├── Inc/                    # 驱动头文件
│   │   ├── lidar.h             # 激光雷达接口
│   │   ├── mpu6050.h           # MPU6050 接口
│   │   ├── inv_mpu.h           # DMP 驱动头
│   │   ├── motor.h             # 电机控制接口
│   │   ├── encoder.h           # 编码器接口
│   │   ├── pid.h               # PID 控制器接口
│   │   ├── bluetooth.h         # 蓝牙串口调试接口
│   │   └── ssd1306.h           # OLED 驱动接口
│   └── Src/                    # 驱动实现
│       ├── lidar.c             # RPLidar C1 数据解析与扇区提取
│       ├── mpu6050.c           # MPU6050 底层读写
│       ├── inv_mpu.c           # InvenSense DMP 驱动
│       ├── inv_mpu_dmp_motion_driver.c
│       ├── motor.c             # 电机 PWM 与方向控制
│       ├── encoder.c           # 编码器测速
│       ├── pid.c               # PID 算法实现
│       ├── bluetooth.c         # 蓝牙调试协议与格式化输出
│       └── ssd1306.c           # OLED 显示驱动
├── Drivers/                    # CMSIS + STM32F4xx_HAL_Driver
├── MDK-ARM/                    # Keil µVision 工程文件
├── build/                      # 编译输出目录
├── STM32_new.ioc               # STM32CubeMX 配置文件
├── STM32F446RETX_FLASH.ld      # 链接脚本
└── README.md                   # 本文件
```

---

## 软件架构

### 数据流概览

```text
RPLidar C1 扫描数据
       |
  USART3 + DMA (循环接收)
       |
 HAL_UARTEx_RxEventCallback()   <-- IDLE / 半传输中断
       |
   Lidar_ProcessFrame()         <-- 扇区距离提取
       |
  main.c 控制循环               <-- 距离滤波 + 状态机决策
       |
   +------------------+------------------+
   |                  |                  |
 电机控制           蓝牙输出           OLED 显示
 (PID + PWM)      (USART1)         (I2C)
```

### 主要任务/模块

1. **LiDAR 数据链路** (`usart.c` → `lidar.c`)
   - DMA 循环缓冲区接收，避免每字节中断
   - 流式解析支持帧头同步与跨缓冲区节点拼接
   - 输出 360° 环向距离直方图，供控制层使用

2. **运动控制** (`main.c` → `motor.c` / `pid.c` / `encoder.c`)
   - 主循环中以固定周期执行状态机
   - 根据前方/侧方障碍物距离切换行为状态
   - 转弯时可用 MPU6050 yaw 角做闭环对准

3. **调试与可视化** (`bluetooth.c` / `ssd1306.c`)
   - 蓝牙串口输出兼容常见串口助手 / 自制上位机
   - OLED 分页显示：距离雷达图、姿态角、当前状态、电池电压等

---

## 构建与烧录

### 开发环境

- **STM32CubeMX** 6.x：初始化代码与外设配置生成
- **Keil MDK-ARM** 5.38+：打开 `MDK-ARM/STM32_new.uvprojx`
- **VS Code + EIDE**（可选）：打开 `STM32_new.code-workspace`
- **ST-Link Utility / STM32CubeProgrammer**：固件烧录

### 编译步骤（Keil）

1. 用 Keil 打开 `MDK-ARM/STM32_new.uvprojx`
2. 确认目标设备为 **STM32F446RETx**
3. 点击 **Rebuild** 编译
4. 连接 ST-Link，点击 **Download** 烧录

### 编译步骤（VS Code + EIDE）

1. 用 VS Code 打开工程根目录
2. 在 EIDE 插件中选择对应工具链（ARMCC / GCC）
3. 点击 **Build** → **Upload**

---

## 调试接口

### 蓝牙串口 (USART1)

- **默认波特率**：115200
- **数据位**：8，**停止位**：1，**校验**：无
- **输出内容**：
  - 各扇区距离（单位：米）
  - MPU6050 DMP 姿态角（Yaw / Pitch / Roll）
  - 当前迷宫状态机状态
  - 电机 PWM 与编码器速度
  - 运行时间与帧率统计

### OLED 显示

- **分辨率**：128 × 64 像素
- **接口**：I2C1
- **显示页面**：可通过代码切换，默认显示前方距离与当前状态

### 现场调试建议

1. 上电后先观察 OLED 是否正常初始化（显示画面）
2. 打开手机或 PC 蓝牙串口助手，确认有周期性数据输出
3. 检查 LiDAR 是否正常旋转并返回有效距离值
4. 将小车置于迷宫入口，观察状态机是否正常切换

---

## 关键文件说明

| 文件 | 职责 |
|------|------|
| `Core/Src/main.c` | 系统初始化、主循环、迷宫状态机、控制调度 |
| `Core/Src/usart.c` | USART1/3 与 DMA 初始化；接收回调入口 |
| `Core/Src/stm32f4xx_it.c` | 系统中断服务函数（DMA、UART、TIM） |
| `BSP/Src/lidar.c` | RPLidar C1 协议解析、扇区距离统计 |
| `BSP/Src/motor.c` | 电机方向与 PWM 控制 |
| `BSP/Src/pid.c` | 增量式/位置式 PID 算法 |
| `BSP/Src/encoder.c` | 编码器脉冲计数与速度计算 |
| `BSP/Src/mpu6050.c` | MPU6050 寄存器读写与校准 |
| `BSP/Src/inv_mpu*.c` | InvenSense 官方 DMP 驱动与运动解算 |
| `BSP/Src/bluetooth.c` | 蓝牙调试帧封装与格式化打印 |
| `BSP/Src/ssd1306.c` | OLED 基础绘图与字符显示 |

---

## 参考资料

- [STM32F446RE 参考手册 (RM0390)](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [RPLidar C1 通信协议](https://www.slamtec.ai/wp-content/uploads/2023/11/RPLIDARC1_Interface_Manual.pdf)
- [MPU6050 DMP 驱动 (InvenSense)](https://www.invensense.com/)
- STM32CubeF4 HAL/LL 驱动说明

---

> **提示**：本项目目前基于 HAL 库中断/轮询架构运行。若需扩展为多任务实时系统，可将 LiDAR 解析与蓝牙输出拆分为 FreeRTOS 任务，通过队列进行块描述符与结果数据的传递，详见 `CODINGDAY3.md` 中的架构设计讨论。

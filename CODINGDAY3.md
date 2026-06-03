# Week4.5 Coding Day README

## 1. 项目概述

本项目基于现有的 STM32F446RE 小车工程展开，当前仓库已经完成了以下基础能力：

- `USART3 + DMA` 接入 RPLidar C1，波特率为 `460800`；
- 在 [Core/Src/usart.c](Core/Src/usart.c) 中通过 `HAL_UARTEx_ReceiveToIdle_DMA()` 持续接收 LiDAR 数据；
- 在 `HAL_UARTEx_RxEventCallback()` 中将收到的数据交给 [BSP/Src/lidar.c](BSP/Src/lidar.c) 的 `Lidar_ProcessFrame()` 处理；
- 在 `lidar.c` 中完成 RPLidar C1 标准 `SCAN` 数据流解析，并提取前方、右侧、右前、左侧距离信息；
- 在 [Core/Src/main.c](Core/Src/main.c) 中对 LiDAR 距离做滤波，并用于迷宫循迹/避障控制；
- 通过 `USART1` 蓝牙串口和 OLED 输出调试信息。

因此，本工程已经具备 Coding Day 所需的“LiDAR 连续字节流采集 + 流式解析 + 调试输出”基础。  
根据《Week4.5-CodingDay.pptx》的要求，本次 Coding Day 的核心工作不是从零开始接传感器，而是把当前“中断/回调里直接解析”的结构，重构成一个标准的 FreeRTOS 软件管道。

## 2. 当前工程与 Coding Day 目标的对应关系

### 当前已经完成的基础

- LiDAR 串口初始化和 DMA 接收：`Core/Src/usart.c`
- LiDAR 数据解析与扇区距离提取：`BSP/Src/lidar.c`
- 距离滤波、迷宫控制、调试输出：`Core/Src/main.c`
- 外设中断入口：`Core/Src/stm32f4xx_it.c`

### 当前结构的特点

- 当前接收链路是：`UART3 DMA -> IDLE 回调 -> Lidar_ProcessFrame() -> main.c 使用距离结果`
- 当前解析逻辑已经支持“连续字节流 + 残留字节拼接”，因为 `lidar.c` 中已经有：
  - `lidar_node_carry[4]`
  - `lidar_parse_work[2052]`
  - `lidar_parse_scan_nodes()`
- 当前输出链路已经具备基础调试能力，因为 `main.c` 中有：
  - `Send_Data_EverySecond()`
  - `Lidar_ForwardTask()`
  - 蓝牙串口输出和 OLED 展示

### 与 Coding Day 要求的差异

- 当前代码还没有 FreeRTOS
- 当前还没有两个任务和两个队列
- 当前 `HAL_UARTEx_RxEventCallback()` 会直接调用 `Lidar_ProcessFrame()`，不符合“ISR 保持简短，只发小描述符”的要求
- 当前 DMA 接收使用的是 `ReceiveToIdle` 思路，还没有明确展示“半缓冲/满缓冲”的乒乓式软件管道

### 两种传输方法的优劣对比

本项目目前已经实现并稳定运行的方法是：

- 方法 A：`USART3 + DMA + ReceiveToIdle + 回调中直接解析`

Coding Day 文档更鼓励展示的方法是：

- 方法 B：`DMA 半缓冲/满缓冲 + ISR 只发描述符 + Queue + Parser Task + Output Task`

#### 方法 A 的优点

- 已经和当前工程深度集成，联调成本低
- 当前 `lidar.c` 已经支持连续字节流解析和残留字节拼接，能满足实际测距需求
- 与 [Core/Src/main.c](Core/Src/main.c) 中的距离滤波、迷宫控制、蓝牙调试输出已经形成稳定链路
- 不需要额外引入 FreeRTOS 任务切换和队列管理，现阶段更容易保持小车行为稳定
- 对当前项目目标来说，开发效率更高，修改范围更小

#### 方法 A 的不足

- 中断回调到解析逻辑之间的分层不够彻底
- 不容易完整展示 FreeRTOS 的 `BLOCKED / READY / RUNNING` 调度行为
- 不能直观体现“DMA 半缓冲/满缓冲 + 队列通知”的标准 RTOS 架构
- 从课程展示角度看，可解释性不如任务化方案强

#### 方法 B 的优点

- 架构分层更清晰，完全符合 Coding Day 对“ISR、解析任务、输出任务三层分离”的要求
- 更适合展示 RTOS 中的任务阻塞、唤醒、优先级和队列通信
- ISR 更短，更符合实时系统设计原则
- 解析和输出解耦后，后续扩展性更好

#### 方法 B 的不足

- 需要对 `Core/Src/usart.c`、`Core/Src/stm32f4xx_it.c`、`BSP/Src/lidar.c`、`Core/Src/main.c` 做较大范围重构
- 需要新增 FreeRTOS 任务、队列、任务栈、优先级和更多联调工作
- 会改变当前 LiDAR 数据进入控制层的时序，可能影响已经调好的小车运动表现
- 在当前阶段切换方案，风险主要不在“能不能写出来”，而在“会不会破坏现有稳定功能”

### 本组当前取舍

经过评估，我们选择以方法 A 作为当前项目的主实现方案，并在 README 中补充方法 B 的标准架构说明与原理论证。

这样选择的原因是：

- 当前项目已经进入小车逻辑控制层面的开发，不再只是单独的 LiDAR 驱动实验
- [Core/Src/main.c](Core/Src/main.c) 中已经开始并持续完善：
  - `RefreshLidarDistances()`
  - `MazeControlTask()`
  - `Control()`
  - `Send_Data_EverySecond()`
- 这些逻辑已经直接依赖 LiDAR 的实时距离结果，并和电机控制、姿态信息、蓝牙调试输出形成联动
- 如果此时把底层数据通路整体改成 FreeRTOS 双任务双队列方案，会连带修改接收时序、解析入口、调试输出路径和控制层取数方式
- 从工程进度和系统稳定性考虑，当前阶段不适合再做大幅重构

因此，本项目在文档中完整说明了 Coding Day 推荐架构，也明确给出了两种方案的优劣与取舍依据；但在实现层面，优先保留了当前已经联调稳定、且能够继续支撑小车控制功能开发的方案。

## 3. Coding Day 目标架构

本次演示需要把工程重构为如下软件管道：

```text
RPLidar C1
   |
USART3 + DMA 循环接收
   |
DMA Half/Full Interrupt
   |
ISR 仅发送 block descriptor 到 Queue1
   |
LiDAR Parser Task（高优先级）
   |
解析结果 result struct 发送到 Queue2
   |
Output / Debug Task（低优先级）
   |
UART / ITM / 蓝牙终端输出
```

这个结构要清楚展示三层分离：

- 中断层：只做“通知”，不做重解析
- 解析层：从 DMA 原始缓冲区中直接读取就绪块并解析
- 输出层：只负责打印、展示和调试

## 4. 推荐的软件设计

### 4.1 DMA 缓冲区策略

建议沿用当前工程中的 `2048` 字节接收缓冲区，但改为“双半区”策略：

- 总缓冲区：`2048 bytes`
- 半缓冲区大小：`1024 bytes`
- DMA 工作方式：循环模式
- 触发点：半完成中断 + 完成中断

选择 `1024 bytes` 作为单块大小的理由：

- 当前工程已经稳定使用 `2048` 字节缓冲区，迁移风险小
- `460800 bps` 在 `8N1` 条件下约为 `46080 bytes/s`
- `1024 bytes` 大约对应 `22 ms` 数据量，既不会过于频繁打断系统，也能保证解析延迟可控
- 对于连续 LiDAR 流，`1024 bytes` 足够容纳大量扫描节点，便于演示“块级处理”

### 4.2 队列 1：ISR 到解析任务

队列 1 不传原始大块数据，只传小描述符：

```c
typedef struct
{
    uint16_t offset;
    uint16_t length;
    uint16_t block_id;
    uint32_t tick_ms;
} lidar_dma_block_t;
```

这样设计的原因：

- ISR 时间短，满足实时性要求
- 队列消息小，复制开销低
- 原始数据仍保留在 DMA 缓冲区中，解析任务可以按偏移直接读取

### 4.3 解析任务

解析任务阻塞在 Queue1 上，收到 `lidar_dma_block_t` 后：

- 根据 `offset + length` 从 DMA 原始缓冲区读取本次就绪块
- 复用当前 `lidar.c` 中已有的流式解析逻辑
- 继续保留“残留字节拼接”机制，解决扫描节点跨块的问题
- 输出每块的统计结果，例如：
  - 有效测量点数量
  - 提取到的节点数
  - 前/右/右前/左扇区距离
  - 当前块的最小距离
  - 简单障碍物摘要

建议结果结构体如下：

```c
typedef struct
{
    uint16_t block_id;
    uint16_t bytes;
    uint16_t valid_nodes;
    float front_m;
    float right_m;
    float right_front_m;
    float left_m;
    float min_m;
    uint32_t tick_ms;
} lidar_parse_result_t;
```

### 4.4 输出/调试任务

输出任务阻塞在 Queue2 上，收到 `lidar_parse_result_t` 后：

- 格式化输出当前块的解析结果
- 通过 `USART1` 蓝牙串口、ITM/SWV 或串口终端打印
- 与解析任务解耦，避免打印阻塞影响 LiDAR 实时解析

### 4.5 任务优先级建议

- `LiDAR Parser Task`：高优先级
- `Output / Debug Task`：低优先级

原因：

- LiDAR 数据流是连续到来的，解析任务如果不及时处理，容易丢块
- 输出任务本质上是“展示层”，允许稍后执行
- 这也可以清楚演示 FreeRTOS 中 `BLOCKED / READY / RUNNING` 的切换过程

## 5. 需要向助教解释清楚的点

### 5.1 为什么 ISR 必须短

- ISR 中不能做复杂解析和大量串口打印
- ISR 只负责确认哪半块缓冲区已就绪，并把描述符发到 Queue1
- 真正耗时的工作放到任务上下文中完成

### 5.2 为什么队列 1 只传描述符

- 不复制大块原始数据，降低 ISR 开销
- 队列消息更小、更稳定
- DMA 缓冲区和任务解析之间的职责边界更清晰

### 5.3 解析任务如何处理连续字节流

- LiDAR 扫描节点不是按 DMA 块边界对齐的
- 一个 5 字节节点可能跨越两个 DMA 块
- 因此解析任务必须保留上一个块末尾的残留字节
- 当前工程里的 `lidar_node_carry` 思路可以直接沿用

### 5.4 为什么解析任务优先级更高

- 输出变慢只会影响展示
- 解析变慢会导致 DMA 后续块来不及处理，进而影响整个数据链路

### 5.5 FreeRTOS 内存对象有哪些

- Queue1 的队列控制块和存储区
- Queue2 的队列控制块和存储区
- Parser Task 的任务栈和任务控制块
- Output Task 的任务栈和任务控制块

## 6. 建议修改的代码位置

如果后续正式实现 FreeRTOS 版本，建议按下面的代码块拆分：

- `Core/Src/usart.c`
  - 负责 `USART3 + DMA` 的循环接收
  - 负责 DMA 半缓冲/满缓冲触发配置
- `Core/Src/stm32f4xx_it.c`
  - 负责 DMA 中断入口
  - 负责在 ISR 中把描述符送入 Queue1
- `BSP/Src/lidar.c`
  - 负责 LiDAR 连续字节流解析
  - 负责残留字节拼接
  - 负责扇区距离、最小距离、有效节点统计
- `Core/Src/main.c`
  - 负责 FreeRTOS 对象创建
  - 负责任务优先级配置
  - 负责 Queue2 输出结果的终端打印

说明：如果后续使用 CubeMX 生成 FreeRTOS 代码，也可以把任务/队列创建部分单独放到新文件中，例如 `app_freertos.c`，但当前 README 仍以现有工程结构为主。

## 7. 演示流程建议

建议现场按以下顺序展示：

1. 上电后启动 LiDAR，DMA 开始持续接收数据
2. DMA 半缓冲或满缓冲触发中断
3. ISR 向 Queue1 发送一个块描述符
4. Parser Task 从 `BLOCKED` 变为 `READY/RUNNING`，读取 DMA 就绪块
5. Parser Task 输出本块统计结果到 Queue2
6. Output Task 被唤醒，打印调试信息
7. 再次说明为什么解析任务优先级高于输出任务

## 8. 六人分工与答辩安排

以下分工按“从底层到上层”的讲解顺序安排。  
成员姓名可以直接替换为你们组员的真实名字。

### 成员 1：项目总负责人 / 架构总述

- 负责讲解内容：
  - 说明本项目的背景是“在现有 STM32 小车 + RPLidar C1 工程基础上，重构为 FreeRTOS 软件管道”
  - 介绍完整数据流：`DMA -> ISR -> Queue1 -> Parser Task -> Queue2 -> Output Task`
  - 解释为什么要做任务分层，为什么解析任务优先级更高
  - 说明本次 Coding Day 的考察点：中断驱动、任务通信、调度、流式解析
- 负责代码块：
  - `Core/Src/main.c` 中的系统初始化
  - FreeRTOS 任务与队列创建部分
  - 任务优先级与整体调度说明

### 成员 2：DMA 与中断负责人

- 负责讲解内容：
  - 解释 `USART3 + DMA` 如何接收 LiDAR 连续字节流
  - 说明什么是半缓冲/满缓冲中断，什么是乒乓式缓冲区
  - 解释为什么 ISR 只发描述符，不在 ISR 里直接做解析
  - 介绍 Queue1 的消息结构设计
- 负责代码块：
  - `Core/Src/usart.c`
  - `Core/Src/stm32f4xx_it.c`
  - DMA 缓冲区、块描述符、ISR 入队逻辑

### 成员 3：LiDAR 流式解析负责人

- 负责讲解内容：
  - 解释 LiDAR 数据流为什么不能按块硬切分
  - 说明“残留字节拼接”的必要性
  - 讲解如何从连续字节流中提取标准扫描节点
  - 说明当前工程中已有的 `lidar_node_carry` 和 `lidar_parse_scan_nodes()` 如何迁移到任务级解析
- 负责代码块：
  - `BSP/Src/lidar.c`
  - 解析任务核心逻辑
  - 节点提取、合法性校验、跨块拼接逻辑

### 成员 4：结果汇总与算法负责人

- 负责讲解内容：
  - 说明如何把原始节点变成“有意义的信息”
  - 介绍有效测量点计数、最小距离、前/右/右前/左扇区距离的生成方法
  - 解释为什么采用扇区摘要而不是直接输出全部原始点
  - 说明 Queue2 结果结构体设计
- 负责代码块：
  - `BSP/Src/lidar.c`
  - 扇区统计、最小距离、结果结构体组装
  - `lidar_parse_result_t` 相关逻辑

### 成员 5：输出与调试负责人

- 负责讲解内容：
  - 说明为什么要把“解析”和“输出”分成两个任务
  - 展示 Output Task 如何阻塞在 Queue2 上，收到结果后再打印
  - 介绍调试输出通道：蓝牙串口、串口终端、ITM/SWV
  - 讲解如何通过打印结果观察任务唤醒和系统行为
- 负责代码块：
  - `Core/Src/main.c`
  - `Send_Data_EverySecond()` 风格的调试输出部分
  - `USART1` / 蓝牙终端输出逻辑

### 成员 6：测试验证与文档负责人

- 负责讲解内容：
  - 说明为什么选择 `2048` 总缓冲区、`1024` 半缓冲区
  - 说明队列长度、任务优先级、任务栈大小的设计依据
  - 总结 FreeRTOS 的内存对象有哪些
  - 负责现场 demo 串词，回答“如果输出任务变慢会怎样”“如果 ISR 里直接解析会怎样”这类问题
- 负责代码块：
  - 调试计数器、统计字段、参数宏定义
  - `README.md`
  - 演示脚本、测试记录、参数论证材料

## 9. 我们组在答辩时的推荐讲解顺序

- 成员 1：项目目标与总体架构
- 成员 2：DMA 双半缓冲与 ISR 入队
- 成员 3：连续字节流解析与残留字节拼接
- 成员 4：结果结构体与障碍物摘要
- 成员 5：输出任务与调试展示
- 成员 6：参数论证、任务状态、内存对象、总结

## 10. 结论

本项目的优势在于：基础工程已经完成 LiDAR 接收、节点解析、距离提取和小车控制，因此本次 Coding Day 的重点可以集中在“正确的 RTOS 软件架构”上，而不是停留在驱动层连通性。

同时，本组也做出了明确的工程取舍：由于项目已经进入小车逻辑控制层面的联调阶段，当前实现方案已经和控制逻辑紧密耦合。若在此时大幅重构为完整的 FreeRTOS 双队列双任务流水线，虽然更符合课堂展示范式，但会显著增加联调风险，并可能影响现有稳定功能。

最终要展示的不是“我们能读到 LiDAR 数据”，而是：

- 我们能把连续 LiDAR 数据安全地从中断上下文交接到任务上下文
- 我们能用两个任务和两个队列完成职责分离
- 我们理解为什么解析任务优先级更高
- 我们理解 DMA 半缓冲机制、任务阻塞与唤醒、以及 FreeRTOS 内存对象的本质

这正是《Week4.5-CodingDay.pptx》要求展示的核心能力。

如果答辩时需要说明实现取舍，可以总结为：

- 我们理解并能够论证标准 FreeRTOS 架构
- 我们比较了两种传输方案的优劣
- 我们当前保留了更适合现阶段小车联调的实现
- 我们没有回避课程要求，而是基于工程稳定性做出了合理选择

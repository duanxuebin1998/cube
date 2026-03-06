# CUBE 仓库导览与新人上手指南

> 最后核对时间：2026-03-05  
> 目标：帮助新同学在 1~2 天内建立“能编译、能跟流程、知道改哪里”的整体认知。

---

## 1. 仓库定位

该仓库是 **STM32 双板固件项目**，主线是两个工程：

- `LTD_MAIN_CPU2/`：CPU2 主控测量板固件（测量流程、参数管理、设备控制、Modbus 从站）
- `LTD_DISPLAY_CPU3/`：CPU3 显示通讯板固件（显示、外部协议适配、轮询 CPU2、参数下发）

顶层通过 CMake 管理目标选择，`CUBE_TARGET` 可选：

- `LTD_MAIN_CPU2`
- `LTD_DISPLAY_CPU3`

---

## 2. 当前仓库目录（以实际内容为准）

根目录当前主要包含：

- `LTD_MAIN_CPU2/`：CPU2 主工程
- `LTD_DISPLAY_CPU3/`：CPU3 主工程
- `docs/`：文档
- `cmake/`：通用工具链配置
- `tools/`：一键构建脚本
- `old/`：历史归档
- `WirelessHost_V4.1_init/`：历史/独立验证工程
- `build/`：本地构建输出（可删除重建）

说明：

- 历史版本文档里出现过的目录（如 `LTD_MAIN_CPU2_TEXT/`、`MOTOR_TMC5130/`、`measure_water/` 等）在当前仓库中已不存在或不在主线。
- 新人开发入口请固定为 `LTD_MAIN_CPU2/` 与 `LTD_DISPLAY_CPU3/`。

---

## 3. 系统架构与通信拓扑

### 3.1 双板职责

- **CPU2（主控）**
  - 执行测量命令与状态机
  - 管理设备参数（FRAM 持久化）
  - 控制传感器、电机、4-20mA、HART 等外设
  - 对外暴露 Modbus 保持/输入寄存器

- **CPU3（显示通讯）**
  - 处理屏幕与菜单
  - 管理 COM1/COM2/COM3 的协议适配（DSM/Wartsila/LTD）
  - 通过 UART5/RS485 轮询 CPU2（Modbus）
  - 同步参数并展示测量结果

### 3.2 通信链路（简化）

- 外部设备/上位机 ↔ CPU3（COM1/2/3，RS485/UART）
- CPU3 ↔ CPU2（UART5 + RS485，Modbus RTU）
- CPU2 ↔ 板载外设（SPI/UART/GPIO 等）

### 3.3 Modbus 功能码

CPU2 从站主路径支持：

- `0x03` 读保持寄存器（参数）
- `0x04` 读输入寄存器（测量结果/状态）
- `0x10` 写多个保持寄存器（参数下发）

---

## 4. 构建体系（必须先打通）

### 4.1 工具链

- `cmake >= 3.20`
- `ninja`
- `arm-none-eabi-*` 工具链（GCC）

统一工具链文件：

- `cmake/toolchain-arm-none-eabi.cmake`

关键编译选项：

- `-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard`

### 4.2 推荐构建方式（Preset）

```bash
cmake --preset cpu2-debug
cmake --build --preset cpu2-debug

cmake --preset cpu3-debug
cmake --build --preset cpu3-debug
```

也可使用：

- `tools/build_cpu2.sh`
- `tools/build_cpu3.sh`

---

## 5. CPU2 代码主线（主控测量板）

### 5.1 启动主流程

1. `main()`：时钟与外设初始化
2. `App_Init()`：业务初始化（编码器、电机、参数、通信等）
3. `App_MainLoop()`：命令处理与测量调度

### 5.2 核心模块分层

- `Core/`：CubeMX 生成的硬件初始化与中断
- `Application/`：应用调度（`app_main.c`、`measure.c`）
- `Services/`
  - `Modbus/`：协议处理与寄存器映射
  - `ParamStorage/`：参数结构、FRAM 持久化、CRC
  - `MotorControl/`、`Sensor/`、`Weight/` 等：业务服务
- `Drivers/Peripherals/`：具体芯片驱动（TMC5130、AD5421、FRAM 等）

### 5.3 串口收包进入业务路径

- `UART5 IDLE + DMA` 收到一帧后进入 `HostCommuProcess()`（Modbus）
- `USART1 IDLE + DMA` 触发文本命令接收，置位 `new_command_ready`
- `App_MainLoop()` 中处理 `new_command_ready` 与 `g_deviceParams.command`

---

## 6. CPU3 代码主线（显示通讯板）

### 6.1 启动主流程

1. `main()`：时钟与外设初始化
2. `App_Init()`：显示初始化、CPU3 本机参数加载、串口重配
3. `App_MainLoop()`：COM1/2/3 分发处理 + 轮询 CPU2

### 6.2 关键实现点

- `Application/app_main.c`
  - 协议分发入口（按每个 COM 口配置）
  - 串口 busy/pending 发送控制
  - 空闲时执行 `PollingInputData()` 轮询 CPU2
- `Communication/internal/main_board_modbus/cpu2_communicate.c`
  - 组包发送、等待响应、解析 CPU2 返回
- `Application/system_param/cpu3_comm_display_params.c`
  - CPU3 本地参数加载/保存 FRAM
  - COM1/2/3 运行时重配（波特率、校验、协议）

### 6.3 中断模型

CPU3 使用统一的 `IDLE + DMA` 接收框架：

- COM1/2/3：置 `comX_rx_ready`，主循环处理
- UART5（到 CPU2）：根据 `wait_response` 机制接收应答

---

## 7. 跨板“数据契约”（最重要）

### 7.1 两个核心结构

- `DeviceParameters`：设备参数全集（控制、算法、输出、校验）
- `MeasurementResult`：运行态结果全集（状态、调试、油水位、密度等）

CPU2 全局对象：

- `g_deviceParams`
- `g_measurement`

### 7.2 寄存器映射

映射由以下文件共同定义：

- `stateformodbus.h`：寄存器地址与布局
- `dataanalysis_modbus.c`：
  - 结构体 -> 寄存器
  - 寄存器 -> 结构体

修改任何参数/测量字段时，必须联动检查：

1. 结构体定义
2. 寄存器枚举/地址
3. 读写映射代码
4. CPU3 参数元数据（菜单、读写权限、显示）

---

## 8. 建议阅读顺序（新人 Day0）

1. `LTD_MAIN_CPU2/Core/Src/main.c`
2. `LTD_DISPLAY_CPU3/Core/Src/main.c`
3. `LTD_MAIN_CPU2/Application/Src/app_main.c`
4. `LTD_DISPLAY_CPU3/Application/app_main.c`
5. `LTD_MAIN_CPU2/Application/Src/measure.c`
6. `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h`
7. `LTD_MAIN_CPU2/Services/Modbus/stateformodbus.h`
8. `LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c`
9. `LTD_MAIN_CPU2/Services/Modbus/hostcommu.c`
10. `LTD_DISPLAY_CPU3/Communication/internal/main_board_modbus/cpu2_communicate.c`

---

## 9. 新人第一周建议（可执行）

### Day 1：编译打通

- 成功构建 `cpu2-debug` 和 `cpu3-debug`
- 确认输出 `elf/hex/bin/map`

### Day 2：跑通一条命令链路

- 从 CPU3 下发一个简单写参数或命令
- 跟到 CPU2 `HostCommuProcess()` 与 `ProcessMeasureCmd()`

### Day 3：跑通一条参数链路

- UI 改参数 -> CPU3 本地参数 -> 下发 CPU2 -> CPU2 持久化 FRAM

### Day 4：理解中断与调度

- 重点看 `IDLE + DMA + ready flag` 如何把数据从 ISR 交给主循环

### Day 5：做一次“小闭环改动”

- 新增或修改一个参数项，完整打通：
  - 结构体
  - 寄存器映射
  - CPU3 参数元数据/显示
  - 通信读写

---

## 10. 常见坑（提前规避）

1. **只改结构体不改映射**：会导致寄存器错位、参数串位。  
2. **忽略 CPU3 参数表**：UI 看起来改了，实际未同步/未下发。  
3. **RS485 方向切换时机不对**：会造成偶发丢包。  
4. **把历史目录当主线**：请以当前仓库实际目录和本文为准。  
5. **把 `command` 当持久化参数长期保存**：CPU2 已有专门处理逻辑，注意不要破坏。

---

## 11. 相关文档

- `docs/build_cpu2.md`
- `docs/build_cpu3.md`

建议新同学先看本导览，再看两份构建文档。


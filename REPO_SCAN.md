# 仓库扫描报告（新人导览）

## 1. 目录职责

> 该仓库是多工程集合，主要围绕 STM32 固件，核心业务在 `LTD_MAIN_CPU2` 与 `LTD_DISPLAY_CPU3`。

- `LTD_MAIN_CPU2/`
  - **CPU2 主控板工程**：执行测量流程、设备参数管理、传感器/电机控制、主站 Modbus 对外通信。
  - 典型分层：
    - `Core/`：CubeMX 生成的启动与外设初始化。
    - `Application/`：应用编排（`App_Init`、`App_MainLoop`、测量任务调度）。
    - `Services/`：业务服务模块（Modbus、MotorControl、Sensor、ParamStorage 等）。
    - `Drivers/Peripherals/`：具体器件驱动（TMC5130、AD5421、FRAM 等）。

- `LTD_DISPLAY_CPU3/`
  - **CPU3 显示/通讯板工程**：负责显示、串口协议适配、与 CPU2 的 Modbus 通信轮询、参数下发与回读。
  - 结构上含：
    - `Application/display/`：界面显示逻辑。
    - `Communication/internal/main_board_modbus/`：与 CPU2 的内部 Modbus 通道。
    - `Communication/external/*`：外部协议（DSM、Wartsila）适配。
    - `Application/system_param/`：CPU3 本地参数 + FRAM 持久化 + 动态串口重配。

- `LTD_MAIN_CPU2_TEXT/`
  - 与 CPU2 主工程结构相近，通常用于文本/分支验证版本（与主线并行）。

- `MOTOR_TMC5130/`, `STM32_TMC5130/`
  - 电机/驱动芯片相关专项工程（TMC5130 验证与驱动实现）。

- `measure_water/`, `WirelessHost_V4.1_init/`, `text/`
  - 独立功能验证或历史阶段工程。

- `old/`
  - 历史版本归档，不建议作为当前开发入口。

---

## 2. 各 CPU 板通信关系（UART / RS485 / Modbus）

## 2.1 CPU2（主控）侧

- CPU2 使用 `UART5` 作为 Modbus/RS485 通道：
  - 串口初始化：`MX_UART5_Init()`，115200，DMA 收发。
  - 发送完成回调中执行 RS485 方向切换（发完切回接收）。
- Modbus 从站处理入口为 `HostCommuProcess()`：
  - 校验地址、CRC。
  - 支持功能码：03（读保持）、04（读输入）、10（写多寄存器）。
  - 响应帧追加 CRC 后由 `huart5` DMA 发出。

## 2.2 CPU3（显示板）侧

- CPU3 同样使用 `UART5` 作为**与主板 CPU2 的内部链路**。
- `cpu2_communicate.c` 中 `sendToCPU2()`：
  - 置 RS485 发送方向。
  - 经 `HAL_UART_Transmit_DMA(&huart5, ...)` 下发 Modbus 请求。
  - 等待响应后调用 `HostCommuProcess(UART5_RX_BUF, UART5_RX_LEN)` 解析。
- CPU3 与外部设备侧使用 3 路口：
  - COM1 = USART6
  - COM2 = USART2
  - COM3 = USART3
  - 由 `cpu3_comm_display_params` 中每口协议配置进行分发（DSM/Wartsila/LTD）。

## 2.3 逻辑拓扑（简化）

- 外部上位机/设备 ↔（COM1/2/3, RS485/UART）↔ **CPU3**
- **CPU3** ↔（UART5 + RS485, Modbus RTU）↔ **CPU2**
- **CPU2** ↔（SPI/UART/GPIO）↔ 传感器、电机驱动、FRAM、4-20mA 等外设

---

## 3. 关键数据结构

## 3.1 `MeasurementResult`（测量结果聚合）

定义于 `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h`，是 CPU2 运行态测量结果总对象（全局 `g_measurement`）：

- `device_status`：当前状态机状态、错误码、当前命令。
- `debug_data`：编码器、位置、线缆、频率、温度、重量、姿态、电机状态等诊断数据。
- `oil_measurement` / `water_measurement`：油位/水位结果。
- `height_measurement`：实高相关值。
- `single_point_measurement` / `single_point_monitoring`：单点测量/监测结果。
- `density_distribution`：分布测量结果及多测点数组。

## 3.2 `DeviceParameters`（设备参数全集）

同样定义于 `system_parameter.h`（全局 `g_deviceParams`，`#pragma pack(1)`），包含：

- 控制入口：`command`、`powerOnDefaultCommand`。
- 设备/传感器信息：`sensorType`、版本号等。
- 机械与电机参数：轮周、速度、卷尺厚度等。
- 测量算法参数：油位、水位、零点、分布测量、Wartsila 参数。
- 输出参数：DO 阈值、4-20mA 量程与故障电流。
- 校准/命令参数：校油位、校水位、单点位置、电机命令距离。
- 元信息：`param_version`、`struct_size`、`magic`、`crc`（用于 FRAM 持久化校验）。

> 对外协议本质是：通过寄存器读写把 `DeviceParameters` 与 `MeasurementResult` 在 CPU2/CPU3/上位机之间同步。

---

## 4. 启动流程（Reset → init → 任务调度）

## 4.1 CPU2 启动主线

1. **Reset/时钟与 HAL 初始化**（`main`）：
   - `HAL_Init()` → `SystemClock_Config()`
2. **外设初始化**：GPIO、DMA、ADC、DAC、多路 UART、SPI、TIM、CRC、IWDG。
3. **应用初始化**：`App_Init()`
   - 编码器、电机、HART、参数加载、称重、Modbus、故障信息、传感器类型识别。
4. **任务调度**：`while(1) -> App_MainLoop()`
   - 检查新命令（串口/协议侧触发）。
   - 将命令映射为测量任务并调用 `ProcessMeasureCmd()`。
   - 根据命令切换 `device_state` 并执行各测量子流程。

## 4.2 CPU3 启动主线

1. `HAL_Init()` → `SystemClock_Config()`。
2. 初始化 GPIO/DMA/UART/SPI/TIM/CRC/IWDG。
3. `App_Init()`：
   - 显示初始化与 LOGO。
   - 从 FRAM 加载 CPU3 通讯显示参数。
   - 按参数重配 COM1/2/3。
   - 初始化外部协议栈（如 DSM）。
4. `while(1) -> App_MainLoop()`：
   - 处理 COM1/2/3 收包。
   - 按端口协议分发并回包。
   - 与 CPU2 进行轮询式 Modbus 数据同步。
   - 在安全点应用 UART 动态重配。

---

## 5. 建议的阅读顺序（快速上手）

1. `LTD_MAIN_CPU2/Core/Src/main.c`、`LTD_DISPLAY_CPU3/Core/Src/main.c`（总入口）
2. 两侧 `Application/Src/app_main.c`（应用编排）
3. `LTD_MAIN_CPU2/Application/Src/measure.c`（CPU2 任务分发核心）
4. `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h`（核心数据模型）
5. `LTD_MAIN_CPU2/Services/Modbus/hostcommu.c` 与 `LTD_DISPLAY_CPU3/Communication/internal/main_board_modbus/cpu2_communicate.c`（双板通信主链路）


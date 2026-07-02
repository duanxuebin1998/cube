# DCS Modbus 接口规范中文整理

来源文件：

- `../00_原始资料/SI7000官方资料包_含Modbus规范.pdf` 第 10-24 页
- `../00_原始资料/SI7000_modbus_官方.pdf`

原始标题：`SI-7000 Tank Gauging System DCS Modbus Interface Specification`

原始编号：`020-701 Rev B`

整理日期：2026-07-02

## 0. 整理说明

本文是官方 DCS Modbus 接口规范的中文整理稿，用于理解 SI-7000 LTD 设备对外 Modbus 接口。本文不直接代表当前 CUBE 项目的实现状态。当前项目的 SI 协议映射和兼容实现应以 `../02_协议映射/SI协议兼容映射表.md`、源码和联调记录为准。

术语保留口径：

- `LTD`：官方资料中指 SI-7000 液位、温度、密度罐计量系统。
- `DCS`：Distributed Control System，分布式控制系统。
- `Profile`：沿罐高采集温度、密度剖面数据的过程和结果。
- `Auto`、`Cal`、`Manual`、`Profile`：官方运行模式名，正文保留英文并给出中文解释。
- `Probe`：探头或探针组件。本文根据上下文译为“探头”。
- `Tank ID`：罐号，同时作为 Modbus Device ID。

## 0.1 原始页面索引

| PDF 页码 | 原始内容 | 页面图片 |
| --- | --- | --- |
| 10 | DCS Modbus Interface Specification 封面，`020-701 Rev B` | ![DCS Modbus 接口规范封面](assets/p010_modbus_interface.png) |
| 11 | 目的、适用读者和 SI-7000 概述 | ![DCS Modbus 接口规范第 2 页](assets/p011_modbus_interface.png) |
| 12 | Modbus 通信参数、支持的功能码 | ![DCS Modbus 接口规范第 3 页](assets/p012_modbus_interface.png) |
| 13 | Function 1/5 模式和速度地址表 | ![DCS Modbus 接口规范第 4 页](assets/p013_modbus_interface.png) |
| 14 | 模式位和速度位行为说明 | ![DCS Modbus 接口规范第 5 页](assets/p014_modbus_interface.png) |
| 15 | Function 2 状态位、报警位地址表 | ![DCS Modbus 接口规范第 6 页](assets/p015_modbus_interface.png) |
| 16 | 状态位和报警位含义说明 | ![DCS Modbus 接口规范第 7 页](assets/p016_modbus_interface.png) |
| 17 | Function 3/6 Profile 和报警控制寄存器 | ![DCS Modbus 接口规范第 8 页](assets/p017_modbus_interface.png) |
| 18 | Profile 参数、自动 Profile 和报警设定说明 | ![DCS Modbus 接口规范第 9 页](assets/p018_modbus_interface.png) |
| 19 | 缩放规则和缩放表示例 | ![DCS Modbus 接口规范第 10 页](assets/p019_modbus_interface.png) |
| 20 | Function 4 传感器数据和 Profile 数据地址表 | ![DCS Modbus 接口规范第 11 页](assets/p020_modbus_interface.png) |
| 21 | Profile 数据地址表续、当前值和点数说明 | ![DCS Modbus 接口规范第 12 页](assets/p021_modbus_interface.png) |
| 22 | Profile 点数、时间和硬件通信连接 | ![DCS Modbus 接口规范第 13 页](assets/p022_modbus_interface.png) |
| 23 | 编程建议、查找液位、Profile 信息 | ![DCS Modbus 接口规范第 14 页](assets/p023_modbus_interface.png) |
| 24 | Profile 绘图建议 | ![DCS Modbus 接口规范第 15 页](assets/p024_modbus_interface.png) |

## 1. 总体说明

本规范定义了 Scientific Instruments SI-7000 LTD 对 Modbus 协议的有限实现。它面向需要在上位机上开发和测试用户界面的程序人员，说明主机计算机如何读取或控制 SI-7000 的运行模式、探头运动、系统状态、报警、Profile 参数、传感器数据和 Profile 数据。

SI-7000 LTD 是用于大型低温液体储罐的罐计量系统，可测量液位、温度和密度，例如 LNG 储罐。系统通过驱动探头在罐内上下移动，按操作员设定采集温度和密度数据。采集到的数据存储在设备内存中，主机计算机可按需读取。

官方资料特别说明：程序人员应阅读并熟悉 SI-7000 操作手册，因为本 Modbus 规范不会重复介绍设备的基本运行逻辑。

## 2. Modbus 通信参数

通用 Modbus 规范中，帧格式和帧顺序等特性是固定的；接口类型、波特率、校验、停止位数量和传输模式等特性则可由具体实现决定。SI-7000 LTD 的 Modbus 实现采用以下参数：

| 项目 | 官方规定 |
| --- | --- |
| 接口类型 | RS485 |
| RS232 接入 | 如主机侧使用 RS232，需要外接转换器 |
| 波特率 | 9600 baud，当前不可修改 |
| 校验 | Odd parity，奇校验 |
| 数据位 | 8 data bits |
| 停止位 | 1 stop bit |
| 传输模式 | RTU |
| Device ID | 使用每套系统的 Tank ID |
| 主机请求最小间隔 | Host 到 Device 的 Command/Request 最小时间应为 1 秒 |

当一台主机连接多个储罐时，必须逐个地址访问和扫描每一套 SI-7000。每台设备的 Device ID 使用该系统的 Tank ID。Tank ID 可在 SI-7000 触摸屏用户界面中设置。

## 3. 支持的 Modbus 功能码总览

SI-7000 LTD Modbus 接口使用以下功能码：

| 功能码 | 官方用途 | 数据类型 | 读写方向 |
| --- | --- | --- | --- |
| Function 1 | 读取系统模式、电机驱动方向和速度 | 1 bit 数字量 | 读 |
| Function 5 | 设置系统模式、电机驱动方向和速度 | 1 bit 数字量 | 写 |
| Function 2 | 读取单独的状态位和报警位 | 1 bit 数字量 | 读 |
| Function 3 | 读取 Profile 和报警配置寄存器 | 16 bit 寄存器 | 读 |
| Function 6 | 设置 Profile 和报警配置寄存器 | 16 bit 寄存器 | 写 |
| Function 4 | 读取传感器数据、当前罐况和 Profile 数据 | 16 bit 寄存器 | 读 |

官方说明中提到，液位、温度和密度输出在未来固件版本中将支持可配置缩放。当前 Rev B 文档给出的缩放参数为固定值。

## 4. 模式和速度位

### 4.1 地址范围

模式和速度使用 1 bit 数字量。Function 1 用于读取，Function 5 用于设置。

| 项目 | 地址 |
| --- | --- |
| Base Address | 0001 |
| Maximum Legal Address | 0016 |

### 4.2 位定义

| 官方参数名 | 地址 | 中文含义 |
| --- | --- | --- |
| Manual | 00001 | 手动模式 |
| Calibrate | 00002 | 校准模式 |
| Auto | 00003 | 自动模式 |
| Profile | 00004 | Profile 模式 |
| Reserved | 00005 | 保留 |
| Reserved | 00006 | 保留 |
| Reserved | 00007 | 保留 |
| Reserved | 00008 | 保留 |
| Stop | 00009 | 停止探头运动 |
| Up Slow | 00010 | 向上慢速 |
| Up Medium | 00011 | 向上中速 |
| Up Fast | 00012 | 向上快速 |
| Down Slow | 00013 | 向下慢速 |
| Down Medium | 00014 | 向下中速 |
| Down Fast | 00015 | 向下快速 |
| Reserved | 00016 | 保留 |

### 4.3 模式位逻辑

地址 00001-00004 表示当前 LTD 运行模式。各模式互斥：如果某一个模式位置位，其他模式位会清零。

使用 Function 5 改变系统模式时，只需要对新模式对应的位写入置位命令。不需要也不能手动清除旧模式对应的位，设备会自动处理互斥关系。

### 4.4 速度和方向位逻辑

地址 00009-00015 表示探头运动方向和速度。各运动速度同样互斥：如果主机通过 Function 5 设置其中一个运动位，其他运动位会自动清零。

这些位可以随时读取，用于判断当前系统状态，读取动作不会影响设备运行模式。

需要重点注意：如果主机手动设置任何与探头方向或速度相关的位，会停止正在执行的 Profile 活动或液位跟踪，并把系统切换到 Manual drive mode。系统会无限期保持在该手动驱动状态，直到重新设置为 `Auto`、`Cal` 或 `Profile`。在 Manual mode 下，系统不能执行自动功能。

官方建议在上位机界面中突出显示 Manual mode，例如用红色提醒操作员“这不是正常自动运行状态”；而 `Auto` 模式可用绿色显示，因为这是持续跟踪液位的正常运行模式。

## 5. 状态位和报警位

状态和报警使用 1 bit 数字量，通过 Function 2 读取。

| 项目 | 地址 |
| --- | --- |
| Status Base Address | 10001 |
| Alarms Base Address | 10017 |
| Maximum Legal Address | 10032 |

### 5.1 状态位地址表

状态位提供系统运行的重要信息，特别用于验证 LTD 上报的数据是否当前有效。

| 官方状态名 | 地址 | 中文含义 |
| --- | --- | --- |
| Bottom Reference | 10001 | 底部参考开关触发，探头位于罐底 |
| Lower Level Sensor | 10002 | 下液位传感器处于液体中 |
| Upper Level Sensor | 10003 | 上液位传感器处于液体中 |
| Interlock | 10004 | 探头到达允许的最高位置，禁止继续向上 |
| Profile Complete | 10005 | Profile 已完成 |
| Metric | 10006 | 使用公制单位 |
| Reserved | 10007 | 保留 |
| Reel Alarm Disable | 10008 | 卷盘报警禁用 |
| Reel Alarm | 10009 | 卷盘或物理驱动报警 |
| Probe Un-calibrated | 10010 | 探头未校准或当前位置可能不正确 |
| Reserved | 10011 | 保留 |
| Interval Timer | 10012 | 间隔定时 Profile 已启用 |
| Probe At Liquid Level | 10013 | 探头处于液气界面，液位读数为当前值 |
| Reserved | 10014 | 保留 |
| Reserved | 10015 | 保留 |
| Reserved | 10016 | 保留 |

### 5.2 建议上位机必须显示的状态

官方认为下列状态对操作员非常重要，应在自定义用户界面中明确报告：

- `Bottom Reference`
- `Lower Level Sensor`
- `Upper Level Sensor`
- `Interlock`
- `Reel Alarm`
- `Probe Un-calibrated`
- `Interval Timer`
- `Probe At Liquid Level`

其他状态可按需要显示。下面是各状态的含义整理。

### 5.3 状态含义

`Bottom Reference` 在底部参考开关闭合时置位，表示探头位于罐底。

`Lower Level Sensor` 和 `Upper Level Sensor` 在对应传感器处于液体中时置位。这两个状态对观察探头周围的实际物理状态、确认探头在罐内位置很重要。

`Interlock` 在探头到达允许的最高位置时置位；到达该位置后，探头不会继续向上运动。如果正常运行过程中探头触发 Interlock，通常表示某种异常，因为探头正常应处于液位或液位以下。当然，如果操作员手动向上驱动探头，也可能触发该状态。

`Reel Alarm` 置位表示发生物理驱动问题，需要维护人员检查。官方强调这是监控设备正常运行的关键状态。

`Probe Un-calibrated` 在当前探头位置可能不正确时置位。该状态也非常重要，因为它意味着上报的液位值也可能不正确。

`Interval Timer` 置位表示系统将按设定间隔自动执行 Profile；未置位时，Profile 可手动运行。

`Probe At Liquid Level` 在系统处于 Auto 且探头位于液气界面时置位。该状态表示当前上报的液位是最新值。

`Profile Complete` 对程序人员可能比对操作员更重要。该状态清零时，表示 LTD 正在执行 Profile；置位时，表示 Profile 已完成，数据可被采集和分析。

`Reel Alarm Disable` 正常情况下不应置位，因为它会覆盖设备中的安全功能。该安全功能可能在设备故障时防止损坏。官方建议可把该状态绑定到报警，防止它在操作员不知情时被设置。

`Metric` 置位表示数值以公制单位上报；清零表示以英制单位上报。该项一旦设定，通常不需要变更。

### 5.4 报警位地址表

报警位与罐内液体状态相关，对工厂运行很重要。

| 官方报警名 | 地址 | 中文含义 |
| --- | --- | --- |
| Low Density | 10017 | 密度低报警 |
| High Density | 10018 | 密度高报警 |
| Low Temperature | 10019 | 温度低报警 |
| High Temperature | 10020 | 温度高报警 |
| Low, Low Level | 10021 | 低低液位报警 |
| High, High Level | 10022 | 高高液位报警 |
| Low Level | 10023 | 低液位报警 |
| High Level | 10024 | 高液位报警 |
| Profile Temperature Deviation | 10025 | Profile 温度偏差报警 |
| Profile Density Deviation | 10026 | Profile 密度偏差报警 |
| Reserved | 10027 | 保留 |
| Reserved | 10028 | 保留 |
| Profile Low Temperature | 10029 | Profile 低温报警 |
| Profile High Temperature | 10030 | Profile 高温报警 |
| Profile Low Density | 10031 | Profile 低密度报警 |
| Profile High Density | 10032 | Profile 高密度报警 |

## 6. Profile 和报警控制寄存器

Profile 和报警控制使用模拟输出或输入寄存器。Function 3 用于读取，Function 6 用于写入。通过这些寄存器，DCS 操作员可控制 Profile 参数并改变产品报警设定值。

### 6.1 地址范围

| 项目 | 地址 |
| --- | --- |
| Base Address | 40001 |
| Maximum Address | 40023 |

### 6.2 地址表

| 官方参数名 | 地址 | 中文含义 |
| --- | --- | --- |
| Profile First Point | 40001 | Profile 第一个可编程测点位置 |
| Profile Increment | 40002 | Profile 测点间距增量 |
| Profile Dwell Time | 40003 | Profile 每个测点停留时间，单位为秒 |
| Reserved | 40004 | 保留 |
| Reserved | 40005 | 保留 |
| Reserved | 40006 | 保留 |
| Reserved | 40007 | 保留 |
| Reserved | 40008 | 保留 |
| Reserved | 40009 | 保留 |
| Automatic Profile Interval | 40010 | 自动 Profile 间隔，单位为分钟 |
| Automatic Profile Enable | 40011 | 自动 Profile 使能 |
| Automatic Profile Hour | 40012 | 自动 Profile 首次启动小时 |
| Automatic Profile Minute | 40013 | 自动 Profile 首次启动分钟 |
| Low Density Alarm Set Point | 40014 | 低密度报警设定值 |
| High Density Alarm Set Point | 40015 | 高密度报警设定值 |
| Low Temperature Alarm Set Point | 40016 | 低温报警设定值 |
| High Temperature Alarm Set Point | 40017 | 高温报警设定值 |
| Low Low Level Alarm Set Point | 40018 | 低低液位报警设定值 |
| High High Level Alarm Set Point | 40019 | 高高液位报警设定值 |
| Low Level Alarm Set Point | 40020 | 低液位报警设定值 |
| High Level Alarm Set Point | 40021 | 高液位报警设定值 |
| Temperature Deviation Alarm Set Point | 40022 | 温度偏差报警设定值 |
| Density Deviation Alarm Set Point | 40023 | 密度偏差报警设定值 |

### 6.3 参数定义

当 Profile 启动时，探头先移动到罐底，并从罐底开始 Profile。`Profile First Point` 是探头上升过程中第一个可编程停靠测点，用于采集温度和密度。严格来说，它是 Profile 中的第二个点，因为第一个点总是在罐底。

`Profile Increment` 是用于计算后续测点位置的距离增量。设备从 `Profile First Point` 开始，每次加上该增量，决定返回液面过程中在哪些位置停下并采集读数。

`Profile Dwell Time` 是探头在每个测点暂停的时间，单位为秒，用于等待读数稳定后再记录。

Profile 可通过切换到 `Profile Mode` 手动启动，也可按每天固定时间或按重复间隔自动运行。

`Automatic Profile Interval` 表示连续两次 Profile 之间的分钟数。第一次 Profile 按 `Automatic Profile Hour` 和 `Automatic Profile Minute` 启动，之后按该间隔重复。

`Automatic Profile Enable` 表示是否启用自动 Profile：

| 值 | 含义 |
| --- | --- |
| 0 | 禁用自动 Profile |
| 1 | 按设定时间和间隔自动启动 Profile |

如果每天只需要一次 Profile，官方建议间隔设为 1440 分钟，这样第二天同一时间再次启动。启用该功能后的第一次 Profile 会在 `Automatic Profile Hour` 和 `Automatic Profile Minute` 定义的时间启动。

其余寄存器用于设置密度、温度、液位以及温度和密度偏差的报警限值。`Temperature Deviation Limit` 指 Profile 中相邻测点之间温度变化达到多少会触发报警；`Density Deviation Limit` 指相邻测点之间密度变化达到多少会触发报警。

## 7. 缩放规则

### 7.1 总体口径

SI-7000 LTD 对 Function 3、Function 4 和 Function 6 的 Modbus 实现使用 16 bit 有符号或无符号整数与 DCS 通信。因此，每个整数计数代表的物理量需要通过缩放规则定义。

每个物理量都有低限、高限、低缩放值和高缩放值。Rev B 文档中这些限制当前为固定值。官方说明未来固件版本可能允许修改这些值，并在现场调试时为每个站点确定合适参数。

液位、温度和密度分别有独立缩放系数。

### 7.2 缩放表

| 物理量 | 低限 | 高限 | 低缩放值 | 高缩放值 | LTD 上报示例值 | 发送到 DCS 的值 | 整理说明 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| Level，m | 0 m | 65.000 m | 0 | 65000 | 15.000 m | 15000 | 公制液位，1 count = 1 mm |
| Temperature，deg C | -327.68 deg C | 327.67 deg C | -32768 | 32767 | -160.00 deg C | -16000 | 有符号整数，1 count = 0.01 deg C |
| Density，kg/m3 | 0 kg/m3 | 655.35 kg/m3 | 0 | 65535 | 450.00 kg/m3 | 45000 | 1 count = 0.01 kg/m3 |
| Level，ft | 0 ft | 200.00 ft | 0 | 60000 | 100.00 ft | 30000 | 英制液位 |
| Temperature，deg F | -327.68 deg F | 327.67 deg F | -32768 | 32767 | -256.00 deg F | -25600 | 有符号整数，1 count = 0.01 deg F |
| Density，lb/ft3 | 0 lb/ft3 | 65.535 lb/ft3 | 0 | 65535 | 27.500 lb/ft3 | 27500 | 1 count = 0.001 lb/ft3 |

原文中的摄氏度和华氏度使用温度符号。为避免编码和工具兼容问题，本文表格中写为 `deg C` 和 `deg F`。

### 7.3 使用规则

公制液位的 Modbus 16 bit 整数值可直接按毫米读取，为无符号整数。也就是说，每 1 个 Modbus 计数表示 1 mm。

温度无论英制还是公制，每 1 个计数表示对应温标的 0.01 度，并作为有符号整数上报。

读取或写入液位相关量时，例如 `Profile First Point`、`Profile Increment` 和 `High Level Alarm Set Point`，应使用液位缩放系数。该规则同时适用于从主机接收的量和向主机发送的量。主机写入的量会按当前生效缩放值解释。

`Dwell Time` 使用秒，不按液位、温度或密度缩放。

## 8. 传感器数据和 Profile 数据

传感器数据和 Profile 数据使用模拟输入寄存器，通过 Function 4 读取。它们表示探头当前位置的罐内当前状态，以及最近一次 Profile 期间采集的罐况信息。

### 8.1 地址范围

| 项目 | 地址 |
| --- | --- |
| Sensor Data Base Address | 30001 |
| Maximum Legal Address | 30620 |

### 8.2 地址表

| 官方参数名 | 地址 | 中文含义 |
| --- | --- | --- |
| Current Probe Position | 30001 | 当前探头位置 |
| Current Temperature | 30002 | 当前温度 |
| Current Density | 30003 | 当前密度 |
| Liquid Level | 30004 | 最近一次液位读数 |
| Reserved | 30005 | 保留 |
| Number of Points | 30006 | 最近一次 Profile 已采集点数 |
| Profile Month | 30007 | Profile 月 |
| Profile Day | 30008 | Profile 日 |
| Profile Hour | 30009 | Profile 小时 |
| Profile Minute | 30010 | Profile 分钟 |
| Current Hour | 30011 | 当前系统小时 |
| Current Minute | 30012 | 当前系统分钟 |
| Current Second | 30013 | 当前系统秒 |
| Reserved | 30014 | 保留 |
| Reserved | 30015 | 保留 |
| Reserved | 30016 | 保留 |
| Reserved | 30017 | 保留 |
| Reserved | 30018 | 保留 |
| Reserved | 30019 | 保留 |
| Reserved | 30020 | 保留 |
| Profile Probe Position Point 0 | 30021 | Profile 第 0 点探头位置 |
| Profile Temperature Point 0 | 30022 | Profile 第 0 点温度 |
| Profile Density Point 0 | 30023 | Profile 第 0 点密度 |
| Profile Probe Position Point 1 | 30024 | Profile 第 1 点探头位置 |
| Profile Temperature Point 1 | 30025 | Profile 第 1 点温度 |
| Profile Density Point 1 | 30026 | Profile 第 1 点密度 |
| Profile Probe Position Point n | 30021 + 3 * n | Profile 第 n 点探头位置 |
| Profile Temperature Point n | 30022 + 3 * n | Profile 第 n 点温度 |
| Profile Density Point n | 30023 + 3 * n | Profile 第 n 点密度 |
| Profile Probe Position Point 199 | 30618 | Profile 第 199 点探头位置 |
| Profile Temperature Point 199 | 30619 | Profile 第 199 点温度 |
| Profile Density Point 199 | 30620 | Profile 第 199 点密度 |

官方备注：这里使用 `Profile Probe Position`，而不是 `Profile Level`，因为本文中的 `Level` 通常表示实际液位。常规 Profile 中只有最后一个点实际位于液位。

整理备注：本地址表按 30021-30620 只覆盖 Point 0 到 Point 199，共 200 点，每点 3 个寄存器。同一节后文又写“最大点数为 500，0 到 499”。两处口径需要结合实际固件、现场协议和当前项目映射表复核。

### 8.3 当前值定义

`Current Position`、`Current Temperature` 和 `Current Density` 是探头当前所在位置的值。无论探头在罐底还是液位处，这些当前值都表示探头当前位置。

`Liquid Level` 保存最近一次液位读数。如果探头处于液面以下，例如正在执行 Profile，液位读数不会更新，直到探头返回液面。前文的 `Probe At Liquid Level` 状态位用于指示当前上报液位是否为最新值。

### 8.4 Profile 点数和时间

`Number of Points` 表示最近一次 Profile 采集的点数。点数由以下因素决定：

- 可编程起始点位置；
- Profile 增量；
- 当前液位；
- 最大允许点数。

Profile 的第一个点始终是罐底。下一个点是可编程点，通常设为 1 m 这样的整数。之后每次加上可编程增量，增量通常也是 1 m。这样 Profile 点会落在 1 m、2 m、3 m 等整数高度。探头向上移动并采集数据，直到达到液位，或者达到最大点数而导致 Profile 提前结束。最后一个点通常位于液位。

官方文字说明最大点数为 500，即 0-499。如果想把点数限制到更低值，应设置 Profile increment，使最大液位下的点数不超过期望限制。

示例：如果最大液位为 30 m，Profile increment 为 1 m，则最大点数为 31 点，包括底部参考点。位置、温度和密度总共需要 93 个寄存器，即 31 * 3。

`Profile Month`、`Profile Day`、`Profile Hour` 和 `Profile Minute` 表示第一个 Profile 点被采集时的日期和时间。

`Current Hour`、`Current Minute` 和 `Current Second` 表示当前系统时间。官方建议可持续监控系统时间是否变化，用于验证与 LTD 的通信是否正常；如果时间停止变化，应产生报警。

每个 Profile 点的探头位置、温度和密度都可在地址表所列寄存器中读取。超过实际采集点数的寄存器数据为 0。

### 8.5 Function 4 缩放

Function 4 的缩放规则与 Function 3 和 Function 6 使用同一组参数。详见本文第 7 节。

## 9. 硬件和通信连接

### 9.1 总体说明

主机计算机可通过 RS485 现场接线直接连接到 LTD。每个站点通常会提供现场接线图。为了建立通信，需要正确设置多个数据参数。

### 9.2 通信链路

在一种可能配置中，LTD 连接到 `Tank Gauge Interface Module`，即 TGIM。TGIM 作为罐计量仪表的电源开关、电源指示和现场接线连接点，通常安装在控制室或现场设备间。

通常有 2 条通信链路，可用于与主机计算机通信。它们称为 `Host 1` 和 `Host 2`。

LTD 通信链路使用 RS485。如果主机计算机使用 RS232，需要使用转换器。

默认波特率为 9600 baud，当前不可修改。其他参数为奇校验、8 个数据位、1 个停止位，传输模式为 RTU。

Modbus 协议要求使用地址。LTD 中的 Tank ID 可通过触摸屏界面设置，同时作为 Modbus 地址。如果同一数据总线上连接多个 LTD，必须为每台设备设置唯一地址。

## 10. 编程建议

### 10.1 总体建议

要完整理解 LTD 的运行，程序人员应阅读操作手册。本节只给出对编程任务有帮助的相关细节。

LTD 的主要用途有两个：

1. 获取准确液位信息。
2. 获取温度和密度 Profile 信息，从而检测不安全的分层条件。

LTD 检测到的分层通过 `Temperature Deviation` 和 `Density Deviation` 报警指示。

各状态位的重要程度已在状态地址章节说明。开发用户界面时，应参考该章节决定哪些状态必须给操作员显示。

### 10.2 查找液位

LTD 上电后，只有在探头触碰罐底并返回液气界面后，准确液位信息才可用。

系统断电时，最后已知探头位置和液位会存入非易失存储器；系统重新上电时会恢复这些值。但是系统会显示 `Un-cal` 指示，因为这些值可能已经不再正确。

要清除 `Un-cal` 指示，必须让探头触碰罐底。触底会自动把探头位置设置为罐底应显示的值。

LTD 可配置为上电后保持 Manual Mode，也可直接进入 Cal Mode。

如果上电后保持 Manual，应手动启动校准。校准完成后，系统会保持 Auto Mode 并测量液位。对于已经在有液体储罐中完成投运的系统，这通常就是所需的全部操作。LTD 会自动执行查找液位的步骤，切换到 Auto mode，并监测液位。

### 10.3 Profile 信息

将控制器置于 Profile mode 可启动 Profile。启动方式包括：

- 触摸屏用户界面；
- 自动间隔定时器；
- 主机计算机发送的命令。

Point 0 始终是底部点，液位是最后一个点。采集点数由 `Number of Points` 变量给出。

探头在每个点会停留指定等待时间，即 `dwell time`，以便稳定。之后测量温度和密度，再移动到下一个点。Profile 完成后，设备返回 AUTO mode。

Profile 采集过程中，每增加一个新点，`Number of Points` 变量会递增。使用该变量可只导入有效数据点。

新的 Profile 开始时，探头首次触碰罐底后，内存中先前所有数据点和 `Profile Complete` 状态指示都会被清除。Profile 完成并且所有数据可用后，`Profile Complete` 状态指示会置位。

### 10.4 Profile 绘图

查看液体储罐中温度和密度分层的最简单方法是绘图。为了让图有意义，图形必须具备较高分辨率，因为预期数据变化很小，图形必须能显示这些小变化。

官方建议谨慎选择温度和密度的图形范围。一种缩放方式是：把罐底第一个点读取到的值作为图中心，然后随着液位增加绘制相对该值的偏差。

对于多数 LNG 储罐，温度或密度偏差图可采用以下量级作为中心偏差范围：

| 物理量 | 推荐偏差显示范围 |
| --- | --- |
| 密度，公制 | 中心值上下约 5 kg/m3 |
| 密度，英制 | 中心值上下约 0.3 lb/ft3 |

## 11. 对 CUBE 适配工作的提示

以下内容是中文整理时面向本项目的阅读提示，不属于官方原文。

- 官方 Modbus 地址与当前 CUBE 项目的 SI 协议兼容映射可能不同，不能直接把本文地址表视为当前实现。
- 模式和速度位的写操作会打断 Profile 或液位跟踪，并进入 Manual drive mode；做兼容适配或联调脚本时应避免无意写入速度位。
- `Probe At Liquid Level`、`Probe Un-calibrated` 和 `Profile Complete` 是判断液位或 Profile 数据是否有效的关键状态。
- 缩放规则包含有符号温度和无符号液位/密度，PLC 或上位机解释时必须明确 signedness。
- Profile 点数存在“地址表 200 点”和“文字最大 500 点”的官方资料内部差异，应以现场设备、当前固件和联调验证为准。

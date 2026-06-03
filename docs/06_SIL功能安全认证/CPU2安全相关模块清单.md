# CPU2 安全相关模块清单

日期：2026-06-02

用途：为 CPU2 编码规范、静态分析、偏离记录和后续 SIL3 准备工作划定模块范围。

## 1. 分级原则

CPU2 模块按安全影响分为四级：

| 等级 | 名称 | 含义 | 处理策略 |
| --- | --- | --- | --- |
| S0 | 安全核心 | 直接决定安全状态、停机、故障传播、运动边界 | 强制执行编码规范，优先整改 |
| S1 | 安全相关 | 间接影响安全功能或为安全判断提供输入 | 强制检查，允许部分偏离 |
| S2 | 支撑模块 | 影响诊断、通信、参数、显示或维护 | 受控检查，按风险整改 |
| S3 | 非安全/调试 | 不得作为安全功能实现依据 | 安全发布版隔离 |

## 2. S0 安全核心模块

### 2.1 主循环和启动安全态

| 文件 | 作用 | 主要风险 | 首批检查重点 |
| --- | --- | --- | --- |
| `LTD_MAIN_CPU2/Core/Src/main.c` | 系统启动、外设初始化、进入主循环 | 上电后输出未处于安全态，初始化失败后死循环未受控 | 启动安全态、错误处理、看门狗初始化顺序 |
| `LTD_MAIN_CPU2/Application/Src/app_main.c` | CPU2 业务主循环、命令调度、后台检查 | 命令优先级错误、故障态被覆盖、恢复流程抢占人工命令 | 状态机默认分支、错误兜底、阻塞调用 |

### 2.2 故障处理和故障恢复

| 文件 | 作用 | 主要风险 | 首批检查重点 |
| --- | --- | --- | --- |
| `LTD_MAIN_CPU2/Application/Src/fault_manager.c` | 统一故障入口、错误状态和停机 | 停机失败未升级、错误码丢失、状态不一致 | 返回值处理、错误传播、安全态保持 |
| `LTD_MAIN_CPU2/Application/Src/fault_recovery.c` | 测量失败后的自动恢复和重试 | 自动恢复导致重复运动、重试次数错误、旧错误被清除 | 重试边界、状态保持、人工命令抢占 |

### 2.3 电机控制

| 文件 | 作用 | 主要风险 | 首批检查重点 |
| --- | --- | --- | --- |
| `LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl.c` | 电机控制入口和状态 | 电机状态不一致、初始化失败未处理 | 返回值、全局状态、默认安全态 |
| `LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_motion_api.c` | 运动命令、停止、等待 | 超时、命令切换、停止失败、位置越界 | 阻塞等待、停机确认、限位检查 |
| `LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_driver_param.c` | TMC5130 参数、速度、电流、健康检查 | 参数越界、驱动异常未识别 | 范围钳位、返回值、寄存器读写 |
| `LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_lost_step_detect.c` | 丢步检测 | 漏检、误检、未停机 | 阈值类型、错误码传播 |
| `LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_position_model.c` | 电机/编码器位置模型 | 浮点误差、溢出、位置源切换错误 | 定点边界、浮点偏离、持久化恢复 |

### 2.4 看门狗和中断安全

| 文件 | 作用 | 主要风险 | 首批检查重点 |
| --- | --- | --- | --- |
| `LTD_MAIN_CPU2/Core/Src/iwdg.c` | 独立看门狗配置 | 超时时间不匹配、初始化错误 | 看门狗参数、错误处理 |
| `LTD_MAIN_CPU2/Core/Src/stm32f4xx_it.c` | 中断处理、DMA 接收、定时器 | 中断阻塞、自动喂狗掩盖主循环卡死 | 喂狗位置、ISR 长度、共享变量 |

## 3. S1 安全相关模块

### 3.1 测量流程

| 文件 | 作用 | 主要风险 | 首批检查重点 |
| --- | --- | --- | --- |
| `LTD_MAIN_CPU2/Application/Src/measure.c` | 命令解析和测量入口 | 调试命令混入正式路径、命令切换错误 | 命令白名单、默认分支、测试入口 |
| `LTD_MAIN_CPU2/Application/Src/measure_zero.c` | 零点测量 | 找零失败继续运行、碰撞风险 | 停机确认、边界和重试 |
| `LTD_MAIN_CPU2/Application/Src/measure_tank_height.c` | 罐高/罐底测量 | 下行越界、撞底、速度过高 | 下行限位、速度限制、传感异常 |
| `LTD_MAIN_CPU2/Application/Src/measure_oilLevel.c` | 油位测量 | 未找到液位继续运动 | 状态翻转、超时、停机 |
| `LTD_MAIN_CPU2/Application/Src/measure_waterLevel.c` | 水位测量和水位跟随 | 水位判断错误、跟随导致重复运动 | 定点阈值、稳定窗口、自动跟随条件 |
| `LTD_MAIN_CPU2/Application/Src/measure_density.c` | 密度测量 | 浮点算法异常、点位越界 | 测点边界、数组长度、浮点偏离 |
| `LTD_MAIN_CPU2/Application/Src/wartsila_density_measurement.c` | 瓦锡兰密度测量 | 分布点越界、液面判断错误 | 点数限制、目标位置范围 |

### 3.2 传感器和编码器

| 文件 | 作用 | 主要风险 | 首批检查重点 |
| --- | --- | --- | --- |
| `LTD_MAIN_CPU2/Services/Encoder/encoder.c` | 编码器读数、持久化、CRC | 读数跳变、CRC 失败处理不当 | CRC、范围、错误传播 |
| `LTD_MAIN_CPU2/Services/Sensor/sensor.c` | 传感器统一读取和模式切换 | 传感器卡死、错误被清除、模式未稳定 | 超时、重试、状态保持 |
| `LTD_MAIN_CPU2/Services/Sensor/dsm_sensor_communication.c` | DSM 传感器通信 | DMA 长度、字符串解析、BCC | 缓冲区边界、字符串终止 |
| `LTD_MAIN_CPU2/Services/Sensor/ltd_sensor_communication.c` | LTD 传感器通信 | 协议帧错误、超时 | 帧长检查、错误码 |
| `LTD_MAIN_CPU2/Services/Sensor/wireless_communication.c` | 无线滑环通信 | 外部参数不可信、重试逻辑 | 参数范围、超时、返回值 |
| `LTD_MAIN_CPU2/Services/Sensor/wireless_pairing.c` | 无线匹配 | 匹配过程影响测量状态 | 状态隔离、人工命令抢占 |

### 3.3 参数存储和错误日志

| 文件 | 作用 | 主要风险 | 首批检查重点 |
| --- | --- | --- | --- |
| `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.c` | 设备参数、FRAM、CRC、默认值 | 参数损坏后仍执行运动 | 双分区、CRC、范围钳位 |
| `LTD_MAIN_CPU2/Services/Utilities/error_log.c` | 错误日志文本和最近错误去重 | 日志影响实时路径、错误去重掩盖问题 | 安全路径禁打印、错误码保留 |
| `LTD_MAIN_CPU2/Services/Utilities/my_crc.c` | CRC16/CRC32 | CRC 参数错误、缓冲区长度错误 | 空指针、长度、硬件 CRC 使用 |

## 4. S2 支撑模块

| 文件/目录 | 作用 | 主要风险 | 处理策略 |
| --- | --- | --- | --- |
| `LTD_MAIN_CPU2/Services/Modbus/**` | CPU2 对外/内部寄存器协议 | 外部命令或参数影响安全动作 | 寄存器写入必须范围检查 |
| `LTD_MAIN_CPU2/Services/Hart/**` | AD5421/HART 相关 | 输出电流错误影响故障指示 | 检查故障电流策略 |
| `LTD_MAIN_CPU2/Services/Weight/**` | 称重和碰撞辅助判断 | 误触发或漏触发碰撞判断 | 阈值范围、方向判断 |
| `LTD_MAIN_CPU2/Drivers/Peripherals/src/TMC5130.c` | 电机驱动芯片底层访问 | 寄存器写错、读数异常 | 保留偏离，强化返回值 |
| `LTD_MAIN_CPU2/Drivers/Peripherals/src/AS5145.c` | 编码器底层访问 | 数据异常或 DMA 处理错误 | 保留偏离，强化错误传播 |
| `LTD_MAIN_CPU2/Drivers/Peripherals/src/mb85rs2m.c` | FRAM 访问 | 参数持久化失败 | 地址边界和返回值 |
| `LTD_MAIN_CPU2/Drivers/Peripherals/src/ad5421.c` | 电流输出 | 故障电流错误 | 输出范围和状态确认 |

## 5. S3 非安全/调试模块

| 文件 | 原因 | 安全发布策略 |
| --- | --- | --- |
| `LTD_MAIN_CPU2/Application/Src/test.c` | 大量调试命令、直接硬件操作、打印和测试路径 | 安全发布构建排除 |
| `LTD_MAIN_CPU2/Application/Inc/test.h` | 调试接口声明 | 安全发布构建排除或禁用 |
| 临时串口测试命令 | 可绕过正式流程 | 编译开关隔离 |
| 现场人工观察日志 | 不可作为安全动作依据 | 仅用于诊断 |

## 6. 首批整改优先级

### P0 立即处理

- 构建清单显式化，排除 `test.c`。
- 中断中自动喂狗改为健康监督喂狗。
- 故障路径中的停机返回值升级处理。
- 电机运动入口增加统一安全前置检查。
- 参数写入和命令写入路径增加范围检查。

### P1 一个月内处理

- 安全核心模块强制类型转换分类。
- 函数式宏替换或偏离记录。
- 通信帧和 DMA 缓冲区长度检查。
- 浮点安全阈值改定点或偏离记录。
- 直接 `printf` 从安全路径移除。

### P2 后续持续处理

- 普通测量算法 MISRA 逐项整改。
- HAL/CMSIS 偏离记录归档。
- 工具链静态分析报告归档。
- 单元测试和故障注入测试补齐。

## 7. 模块进入安全基线的条件

模块进入 CPU2 安全基线前必须满足：

- 明确安全等级 S0/S1/S2/S3。
- 完成编码规范检查。
- 强制规则违反项已修复或已有偏离记录。
- 影响安全功能的返回值已处理。
- 外部输入边界已验证。
- 测试或评审证据已归档。

## 8. 后续维护

每次 CPU2 版本升级时，应复审本清单：

- 是否新增安全相关模块。
- 是否有调试模块进入构建。
- 是否有普通模块变成安全相关模块。
- 是否有偏离记录需要关闭或重新批准。

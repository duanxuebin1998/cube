# SI协议兼容映射表

资料依据：`docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000_modbus_官方.pdf`，即 SI-7000 DCS Modbus Interface Specification `020-701 Rev B`。本文区分“官方手册口径”和“当前 CUBE 兼容实现”，避免把兼容偏差误写成 SI协议原生行为。

## 0. 资料来源与落地口径

本地 `00_原始资料` 已能确认一部分协议硬约束，但没有找到完整的 `40001~40023` 出厂默认值和非法值总表。后续文档和实现统一按“来源分级”处理：

| 来源分级 | 含义 | 落地规则 |
| --- | --- | --- |
| 官方明确 | 原始 Modbus/DCS 资料直接给出地址、缩放、保留范围或功能语义 | 作为 CPU3 SI 协议层硬约束，脚本和联调必须覆盖 |
| 官方建议 | 手册给出推荐配置，例如每天一次 profile 建议 `40010=1440` | 写入说明和联调建议，不作为唯一合法值 |
| CUBE 工程默认 | 本地资料未给默认值，当前代码为安全运行设定的默认值 | 文档必须标注为 CUBE 默认，不写成 SI 出厂默认 |
| 待现场确认 | 手册缺失或与实现目标有冲突的项目 | 进入待确认清单，不提前扩展协议或改变倍率 |

可直接固化的官方硬约束如下：

| 类别 | 官方资料结论 | 当前落地 |
| --- | --- | --- |
| 串口 | `9600` baud、Odd parity、8 data bits、1 stop bit、RTU | 选择 `COM_PROTO_SI` 后 CPU3 外部串口强制为 `9600 8O1` |
| 地址窗口 | 线圈 `00001~00016`、离散输入 `10001~10032`、保持寄存器 `40001~40023`、输入寄存器 `30001~30620` | CPU3 按窗口做边界保护，越界返回 Modbus 异常 |
| 保留地址 | `00005~00008`、`00016`；`10007`、`10011`、`10014~10016`、`10027~10028`；`40004~40009`；`30005`、`30014~30020` | 保留线圈/保持寄存器拒写；保留离散输入/输入寄存器读 0 |
| 缩放 | Level 1 count = `1 mm`；Temperature 1 count = `0.01°C`；Density 1 count = `0.01 kg/m3` | CUBE 内部单位在 CPU3 SI 层转换，密度超 16 位时钳位 |
| 自动 profile | `40011=0` 禁用，`40011=1` 启用；每天一次建议 `40010=1440` | 关闭使用 `40011=0`；`40010=0` 拒写；`1440` 作为日周期建议值 |
| Profile 点阵 | DCS 地址表只覆盖 `30021~30620`，即 Point 0~199 共 200 点；有效点之外数据为 0 | 当前 SI协议对外最多 200 点，完成前和有效点外保持 0 |
| 内部点数说明 | 操作手册另提设备内部 profile 最大 500 点 | 不作为当前 DCS Modbus 可读点数；超过 200 点需扩展地址或分页方案 |
| 错误显示值 | 操作手册提到温度/密度超量程显示值 | 外部 Modbus 采用 CUBE 兼容哨兵：温度 `-200.00°C`，密度 `0` |

## 1. 当前基线

| 项目 | 当前结论 |
| --- | --- |
| 协议契约 | CPU2/CPU3 共享协议基线为 `DEVICE_PROTOCOL_VERSION = 14` |
| 外部协议入口 | CPU3 对外模拟 SI Modbus RTU 从站，CPU2 不承载 SI 地址、功能码或异常码 |
| Profile 命令 | `00004 Profile` 写 ON 后锁存开始时间并下发 `CMD_SI_PROFILE` |
| 屏幕 Profile 入口 | CPU3 屏幕“密度分布测量 -> SI Profile”确认后走同一启动入口，锁存开始时间并下发 `CMD_SI_PROFILE` |
| Profile 参数 | `40001~40003` 独立存入 CPU2 SI profile 参数，不再复用普通分布测量参数 |
| 自动 profile 与报警阈值 | `40010~40023` 独立存入 CPU3 SI 本机参数，用于 RTC 自动调度和报警合成 |
| Profile 时间戳 | `30007~30010` 为 CPU3 收到 SI `00004 Profile`、屏幕 SI profile 或自动 profile 触发并下发指令时锁存的时间 |
| 液位传感器显示 | `10002` 固定表示下传感器在液体中；`10003` 液位跟随稳定时为空气，其它状态为液体 |
| 探针可信 | `10010 Probe Un-calibrated` 固定输出 `0`，表示探针可信 |
| Point0 和点阵 | Point0 固定为底部点；`40001` 表示底部之后的首个停点；profile 前不单独找液位，运行中遇到液面以上点停止，且不写入该点 |
| 探底失败 fallback | 需要探底但失败时，如有旧底部位置则沿用旧底部；没有旧底部位置时使用当前位置完成 Point0，但不刷新底部参考 |
| 无效值 | 温度无效对外输出 `-200.00°C`，密度无效输出 `0`，液位/位置无效和有效点数外仍输出 `0` |

## 2. 方案定位

- 兼容层位置：`CPU3` 外部串口侧
- 当前实现文件：
  - `LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.c`
  - `LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.h`
- 当前接入方式：
  - `app_main` 主分发路径和 `com_manager` 兼容分发路径均接入 `COM_PROTO_SI`
  - 由 `CPU3` 对外模拟 `SI Modbus RTU` 从站
  - 再复用 `CPU3 <-> CPU2` 内部同步数据作为真实数据源

### 2.1 职责边界

| 层级 | 职责 | 不承担的内容 |
| --- | --- | --- |
| `CPU2` | 真实测量、电机动作、液位/密度/profile 数据、通用辅助状态和必要通用命令 | 不理解 SI 地址、线圈、功能码、缩放和异常响应 |
| `CPU3` | 串口协议选择、SI Modbus RTU 从站、地址表、单位缩放、CPU3 SI 参数、报警合成、自动 profile 调度、命令桥接和时间戳 | 不把 SI Modbus 地址、功能码或异常码下沉到 CPU2 |
| `CPU2 <-> CPU3` 共享协议 | 发布通用业务数据和 SI 状态字段 | 不承载 SI 寄存器编号或 PLC 专用语义 |

## 3. 当前实现状态

- 已完成：
  - `FC01/FC02/FC03/FC04/FC05/FC06` 基础协议框架
  - 16 个线圈、32 个离散输入、23 个保持寄存器、620 个输入寄存器边界保护
  - 已按官方 `020-701 Rev B` 复核功能码、地址窗口、串口参数、缩放单位和 profile 点阵窗口
  - CPU3 外部串口协议分发、菜单协议选项和 9600 8O1 串口配置支持
  - `FC05` 关键线圈到 `CPU2` 命令的桥接，模式线圈和动作线圈在快照区内互斥
  - `FC06` 的 `40001~40003` 到 `CPU2` SI profile 执行参数的真实桥接
  - `00004 Profile` 写 ON 后下发 `CMD_SI_PROFILE`，不再复用普通分布测量命令
  - CPU3 屏幕密度分布测量菜单新增 `SI Profile` 入口，确认后调用同一 SI profile 启动入口
  - CPU2/CPU3 共享 SI 状态字段，供 CPU3 协议转换层生成 SI 状态位
  - CPU3 RTC 时间接口，`30011~30013` 当前时间和 `30007~30010` profile 时间戳已接入
  - 位置/液位按内部 `0.1mm` 到协议 `mm` 转换，温度按内部原始值转换为有符号 `0.01°C`
  - 选择 `COM_PROTO_SI` 后，CPU3 保存和加载本机串口参数时都会强制该端口为 `9600 8O1`
  - 密度按协议示例输出为 `kg/m3 x100`；协议 13 后 CPU2/CPU3 内部密度 raw 已统一为 `kg/m3 x100`，SI 层不再额外乘 10，超出 16 位时钳位到 `65535`
  - `40010~40023` 已作为 CPU3 SI 本机参数持久化，并用于自动 profile 调度和报警合成
  - `10012` 定时器状态和 `10017~10026`、`10029~10032` 报警位已基于 CPU3 SI协议设定点合成
  - `10002/10003` 按液位跟随稳定状态合成，`10010` 固定输出可信
  - Profile 完成锁存后才输出 `30006 Number Of Points` 和点阵数据，范围外保持 0，避免 PLC 读到旧点残留
  - 已新增参考帧脚本和共享协议契约脚本，分别保护 SI协议地址表/异常响应、协议版本、命令号和 CPU2/CPU3 SI协议状态字段顺序
- 保留限制：
  - `FC05` 慢/中/快三档当前统一桥接到强制上行或强制下行，CPU2 侧尚无独立速度档位参数
  - `40010` 禁止写 0，`40011`、`40012`、`40013` 写入时已做基础值域检查，分别限制为使能 `0/1`、小时 `0~23`、分钟 `0~59`
  - SI协议官方手册默认值仍需现场逐项复核；当前代码使用保守默认值
  - 协议密度为 `0.01 kg/m3` 时，常规高密度值可能超过 16 位容量；当前按协议放大并做钳位，该限制已确认可接受

## 4. 串口和协议要求

- 官方接口：`RS485`；若主机侧使用 `RS232`，官方手册要求外接转换器。
- 协议：`Modbus RTU`
- 波特率：`9600`
- 格式：`8O1`
- 站号：官方口径为 `Device ID = Tank ID`，同一总线上每台唯一；当前实现复用现有 `SlaveAddress`，有效范围 `1~247`，无效时使用 SI 层默认地址。
- 轮询周期：建议 `>= 1s`

官方功能码和地址窗口如下：

| 功能码 | 官方用途 | 地址窗口 | 当前支持 |
| --- | --- | --- | --- |
| `FC01` | 读模式、方向和速度线圈 | `00001~00016` | 支持 |
| `FC05` | 写模式、方向和速度线圈 | `00001~00016` | 支持，保留位拒写 |
| `FC02` | 读状态和报警离散输入 | `10001~10032` | 支持 |
| `FC03` | 读 profile 和报警配置保持寄存器 | `40001~40023` | 支持 |
| `FC06` | 写 profile 和报警配置保持寄存器 | `40001~40023` | 支持，保留位和值域非法时返回异常 |
| `FC04` | 读实时测量值、时间和 profile 点阵 | `30001~30620` | 支持 |

官方缩放口径：

| 类别 | 官方量程/缩放 | 当前实现 |
| --- | --- | --- |
| 液位/位置 | 公制下 1 count = `1 mm`，无符号 16 位，`0~65000` 表示 `0~65.000 m` | 内部 `0.1mm` 转 `mm`，负值按 0 输出 |
| 温度 | 有符号 16 位，1 count = `0.01°C` | 内部原始温度按 `raw - 20000` 转有符号 `0.01°C` |
| 密度 | 无符号 16 位，1 count = `0.01 kg/m3`，最高 `655.35 kg/m3` | 内部 `kg/m3 x100` 直接输出，超过 16 位钳位到 `65535` |
| 停留时间 | `40003` dwell time 单位为秒 | 按秒写入 CPU2 SI profile 参数 |

SI协议对外无效值口径：

| 数据 | 内部无效来源 | SI协议对外值 | 说明 |
| --- | --- | --- | --- |
| 温度 | CPU2 温度为 `0`、`9999`、无线温度无效值等 | `-200.00°C`，寄存器值 `-20000`，补码 `0xB1E0` | PLC 按有符号 `0.01°C` 解释 |
| 密度 | `UNVALID_DENSITY = 0` | `0` | 保持现有密度无效值，不改为 `65535` |
| 液位 | `UNVALID_LEVEL` | `0` | 避免 `65535` 被误解为超高液位 |
| 位置 | 负位置或无效位置 | `0` | 对外 0 可解释为底部/未知 |
| profile 有效点外 | 点数之外 | `0` | 保持官方 DCS 点阵口径 |

无效温度和无效密度不参与报警判断，避免 `-200.00°C` 触发低温报警或密度 `0` 触发低密报警。

## 5. 总体归类

当前按四类维护：

### 5.1 可直接复用现有变量和指令

这些内容直接保留，后续只做协议映射，不改内核基准：

- 位置和液位：
  - `g_measurement.debug_data.sensor_position`
  - `g_measurement.debug_data.cable_length`
  - `g_measurement.height_measurement.current_real_height`
  - `g_measurement.oil_measurement.oil_level`
- 温度和密度：
  - `g_measurement.debug_data.temperature`
  - `g_measurement.single_point_monitoring.density`
  - `g_measurement.density_distribution.measurement_points`
  - `g_measurement.density_distribution.single_density_data[]`
- 现有命令的可复用部分：
  - `CMD_FIND_OIL`
  - `CMD_CALIBRATE_ZERO`
  - `CMD_FORCE_MOVE_UP`
  - `CMD_FORCE_MOVE_DOWN`
  - `CMD_SI_PROFILE`
- 现有普通分布和 Wärtsilä 参数仍保留原用途：
  - `spreadTopLimit`
  - `spreadMeasurementDistance`
  - `spreadPointHoverTime`
  - `wartsila_lower_density_limit`
  - `wartsila_upper_density_limit`
  - 这些字段不再作为 SI `40001~40003/40014~40023` 的参数源。

### 5.2 本次在 CPU2 新增/同步的

本次新增的是“中性变量和通用能力”，不是 SI 模式本身：

- 状态变量：
  - `bottom_reference_valid`
  - `probe_at_liquid_level`
  - `liquid_stable`
  - `profile_complete_latched`
  - `profile_complete_counter`
  - `profile_blocked_by_process`
  - `loading_unloading_active`
  - `manual_alarm_inhibit`
  - `manual_level_update_inhibit`
  - `profile_temp_deviation_alarm`
  - `profile_density_deviation_alarm`
- 参数：
  - 已复用原 CPU2 预留参数槽新增 `si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time`、`si_profile_bottom_detect_interval`。
  - `40001~40003` 通过 CPU3 外部 SI Modbus 和 CPU3 菜单桥接到上述 CPU2 参数。
  - 普通分布测量参数 `spreadTopLimit`、`spreadMeasurementDistance`、`spreadPointHoverTime` 不再作为 SI profile 参数源。

### 5.3 本次在 CPU3 落地的

这部分更适合放在协议转换层：

- SI 外部 Modbus RTU 从站
- 9600 8O1 串口配置支持
- `Profile Timestamp` 锁存
- `Current Time` 读取
- CPU3 SI 本机参数：`40010~40023`
- 固定值/保留位填充
- 内部 `0.1mm` 与协议 `mm` 的工程量转换
- 在收到或触发 profile 命令时锁存协议时间戳
- 按 CPU3 RTC 执行自动 profile 调度

### 5.4 直接不支持或先返回固定值的

在没有独立硬件来源或协议已确认固定口径时，当前按以下方式返回：

- `10002 Lower Level Sensor` -> 固定 `1`，表示下传感器在液体中
- `10003 Upper Level Sensor` -> 液位跟随稳定时 `0`，其它状态 `1`
- `10010 Probe Un-calibrated` -> 固定 `0`，表示探针可信
- `10012 Interval Timer` -> `40011 Automatic Profile Enable` 有效时置位
- `10006 Metric/English` -> 固定 `1`
- `10017~10026`、`10029~10032` 阈值报警位 -> 按 CPU3 SI协议阈值实时或 profile 完成后合成，阈值 `0` 是有效阈值
- 保留位 -> 固定 `0`

## 6. 线圈区 FC01 / FC05

官方手册把 `00001~00004` 定义为模式位，`00005~00008` 和 `00016` 定义为保留位，`00009~00015` 定义为停止和手动上下行速度位。官方正文有“Bits 1-5 indicate the current mode”的表述，但同页地址表明确 `00005` 为 Reserved；当前实现按地址表处理，只让 `00001~00004` 互斥，`00005~00008` 拒写。

| SI 地址 | 含义 | 当前映射 | 当前实现状态 | 后续工作 |
| --- | --- | --- | --- | --- |
| `00001` | Manual | 设备处于手动/非自动/非标定/非分布测量状态 | 已接第一版 | 当前 `FC05=ON` 桥接到 `CMD_MAINTENANCE_MODE` |
| `00002` | Calibrate | 标定流程 | 已接第一版 | 当前先保守桥接到 `CMD_CALIBRATE_ZERO` |
| `00003` | Auto | 液位自动跟随 | 已接第一版 | 当前 `FC05=ON` 桥接到 `CMD_FIND_OIL` |
| `00004` | Profile | SI 独立 profile 测量 | 已接 | `FC05=ON` 锁存开始时间并下发 `CMD_SI_PROFILE` |
| `00005~00008` | 预留/文档未重点使用 | 暂无 | 拒绝写入 | `FC05` 写入返回 Illegal Data Address |
| `00009` | Stop | 停止 | 已接第一版 | 当前先桥接到 `CMD_MAINTENANCE_MODE`，承担“打断当前动作并进入手动态” |
| `00010` | Up Slow | 手动慢速上行 | 已接第一版 | 当前先统一桥接到 `CMD_FORCE_MOVE_UP` |
| `00011` | Up Medium | 手动中速上行 | 已接第一版 | 当前先统一桥接到 `CMD_FORCE_MOVE_UP` |
| `00012` | Up Fast | 手动快速上行 | 已接第一版 | 当前先统一桥接到 `CMD_FORCE_MOVE_UP` |
| `00013` | Down Slow | 手动慢速下行 | 已接第一版 | 当前先统一桥接到 `CMD_FORCE_MOVE_DOWN` |
| `00014` | Down Medium | 手动中速下行 | 已接第一版 | 当前先统一桥接到 `CMD_FORCE_MOVE_DOWN` |
| `00015` | Down Fast | 手动快速下行 | 已接第一版 | 当前先统一桥接到 `CMD_FORCE_MOVE_DOWN` |
| `00016` | 预留 | 暂无 | 拒绝写入 | `FC05` 写入返回 Illegal Data Address |

`00001~00004` 模式线圈在快照区内互斥；`00009~00015` 停止和手动方向/速度线圈在快照区内互斥。下一次读回时仍以 CPU2 实际设备状态为准。

官方手册特别说明：主机用 `FC05` 设置任一方向/速度位时，会停止正在进行的 profile 或液位跟随，并让设备进入 Manual drive mode；需要重新设置为 Auto、Cal 或 Profile 后，设备才恢复自动功能。当前实现把 `00010~00015` 统一桥接到强制上/下行命令，符合“进入手动动作”的大方向，但不区分 Slow/Medium/Fast 三档真实速度。

### 6.1 当前线圈只读推导来源

- `Profile`：
  - `STATE_SPREADPOINTING`
  - `STATE_GB_SPREADPOINTING`
  - `STATE_METER_DENSITY`
  - `STATE_INTERVAL_DENSITY`
  - `STATE_WARTSILA_DENSITY_MEASURING`
  - 仅当 `current_command == CMD_SI_PROFILE` 时读回 `00004=1`，避免普通分布和 Wärtsilä 分布误触发 SI Profile 运行态
- `Calibrate`：
  - `STATE_CALIBRATIONOILING`
  - `STATE_CALIBRATE_WATERING`
  - `STATE_CALIBRATE_TANKHEIGHTING`
- `Auto`：
  - `STATE_FLOWOIL`
- 电机方向：
  - `g_measurement.debug_data.motor_state`
  - `1 = 上行`
  - `2 = 下行`
  - 其它 = `Stop`

## 7. 离散输入区 FC02

| SI 地址 | 含义 | 当前来源 | 当前状态 | 差距 |
| --- | --- | --- | --- | --- |
| `10001` | Bottom Reference | `既有测量子结构状态字段.bottom_reference_valid` | 已接 | 探底成功或允许 fallback 成立时置位 |
| `10002` | Lower Level Sensor | CPU3 按液位跟随稳定状态合成 | 已接 | 液位跟随稳定和其它状态均显示下传感器在液体中，即 `1` |
| `10003` | Upper Level Sensor | CPU3 按液位跟随稳定状态合成 | 已接 | 液位跟随稳定时为 `0`，表示上传感器在空气中；其它状态为 `1`，表示在液体中 |
| `10004` | Interlock | `error_code != NO_ERROR` 或 `profile_blocked_by_process` | 已接 | 当前按故障/流程阻挡合成，未细分外部联锁硬件 |
| `10005` | Profile Complete | `density_distribution.profile_complete_latched` + `profile_source == PROFILE_SOURCE_SI` | 已接 | 只在 SI profile 完成时置位，普通分布和 Wärtsilä 分布不再触发 SI 完成态 |
| `10006` | Metric/English | 单位制 | 已接 | 当前固定公制，写死 `1` |
| `10007` | 预留 | 暂无 | 固定 0 | 保留 |
| `10008` | Reel Alarm Disable | 暂无 | 固定 0 | 官方说明该位不应正常置位；当前不提供禁用卷尺报警能力 |
| `10009` | Reel Alarm | 卷尺/电机告警 | 已接简化版 | 当前直接用 `error_code != NO_ERROR`，后续建议细化 |
| `10010` | Probe Un-calibrated | CPU3 固定输出 | 已接 | 固定 `0`，SI协议对外始终显示探针可信；不再等同 CPU2 零点状态 |
| `10011` | 预留 | 暂无 | 固定 0 | 保留 |
| `10012` | Interval Timer | CPU3 SI协议自动 profile 参数 | 已接 | `40011 != 0` 时置 1；`40010` 禁止写入 0 |
| `10013` | Probe At Liquid Level | `oil_measurement.probe_at_liquid_level` | 已接 | 液位流程锁定液面后置位，离开或重启测量时清零 |
| `10014~10016` | 预留 | 暂无 | 固定 0 | 保留 |
| `10017` | Low Density Alarm | 当前密度与 `40014` 比较 | 已接 | 当前密度有效且低于阈值时置 1；阈值 `0` 是有效值 |
| `10018` | High Density Alarm | 当前密度与 `40015` 比较 | 已接 | 当前密度有效且高于阈值时置 1；阈值 `0` 是有效值 |
| `10019` | Low Temp Alarm | 当前温度与 `40016` 比较 | 已接 | 当前温度有效且低于阈值时置 1；`40016` 按有符号 `0.01°C` |
| `10020` | High Temp Alarm | 当前温度与 `40017` 比较 | 已接 | 当前温度有效且高于阈值时置 1；`40017` 按有符号 `0.01°C` |
| `10021` | LL Level Alarm | 当前液位与 `40018` 比较 | 已接 | 当前液位有效且低于阈值时置 1；阈值 `0` 是有效值 |
| `10022` | HH Level Alarm | 当前液位与 `40019` 比较 | 已接 | 当前液位有效且高于阈值时置 1；阈值 `0` 是有效值 |
| `10023` | Low Level Alarm | 当前液位与 `40020` 比较 | 已接 | 当前液位有效且低于阈值时置 1；阈值 `0` 是有效值 |
| `10024` | High Level Alarm | 当前液位与 `40021` 比较 | 已接 | 当前液位有效且高于阈值时置 1；阈值 `0` 是有效值 |
| `10025` | Profile Temp Deviation Alarm | CPU3 扫描相邻 profile 点温差 | 已接 | profile 完成后按相邻点有符号温度差与 `40022` 比较，阈值 `0` 是有效偏差阈值 |
| `10026` | Profile Density Deviation Alarm | CPU3 扫描相邻 profile 点密差 | 已接 | profile 完成后按相邻点差值与 `40023` 比较，阈值 `0` 是有效偏差阈值 |
| `10027` | 预留 | 暂无 | 固定 0 | 保留 |
| `10028` | 预留 | 暂无 | 固定 0 | 保留 |
| `10029` | Profile Low Temp Alarm | profile 点温度与 `40016` 比较 | 已接 | 完成锁存后任一点低于阈值时置 1 |
| `10030` | Profile High Temp Alarm | profile 点温度与 `40017` 比较 | 已接 | 完成锁存后任一点高于阈值时置 1 |
| `10031` | Profile Low Density Alarm | profile 点密度与 `40014` 比较 | 已接 | 完成锁存后任一点低于阈值时置 1 |
| `10032` | Profile High Density Alarm | profile 点密度与 `40015` 比较 | 已接 | 完成锁存后任一点高于阈值时置 1 |

官方语义补充：

- `10002/10003` 在官方手册中表示上下液位传感器是否在液体中；当前是 CPU3 合成值，不是真实双传感器输入。
- `10004 Interlock` 官方定义为探头达到允许最大位置后禁止继续上行；当前按设备错误或 profile 流程阻塞近似合成。
- `10005 Profile Complete` 官方建议作为 profile 数据是否可采集的门控：清零表示 profile 正在采集，置位表示 profile 完成且数据可读。
- `10013 Probe At Liquid Level` 官方定义为 Auto 模式且探头位于液/气界面；该位可用于判断 `30004 Liquid Level` 是否为当前有效液位。

### 7.1 当前已接入的完成态

当前 `10005 Profile Complete` 只由 `density_distribution.profile_complete_latched` 且 `profile_source == PROFILE_SOURCE_SI` 合成。CPU2 普通分布、国标、每米、间隔和 Wärtsilä 分布仍可写入通用分布结果，但不会让 SI协议侧 `10005/30006/30021~30620` 被误判为 SI profile 完成。

## 8. 输入寄存器区 FC04

官方手册说明 `30001~30003` 是探头当前位置处的实时位置、温度和密度；`30004 Liquid Level` 是最近一次液位读数，profile 期间探头低于液面时不会更新，需结合 `10013 Probe At Liquid Level` 判断是否为当前液位。

### 8.1 单值寄存器

| SI 地址 | 含义 | 当前变量来源 | 当前状态 | 说明 |
| --- | --- | --- | --- | --- |
| `30001` | Current Probe Position | `g_measurement.debug_data.sensor_position` | 已接 | 内部 `0.1mm` 转 `mm`，负位置按 0 输出 |
| `30002` | Current Temperature | `g_measurement.debug_data.temperature` | 已接 | 内部原始温度按 `raw - 20000` 转有符号 `0.01°C` |
| `30003` | Current Density | `g_measurement.single_point_monitoring.density` | 已接 | 内部与协议均按 `kg/m3 x100`，超过 16 位时钳位 |
| `30004` | Liquid Level | `g_measurement.oil_measurement.oil_level` | 已接 | 内部 `0.1mm` 转 `mm`；官方要求结合 `10013` 判断是否为当前液位 |
| `30005` | 文档未重点定义/保留 | 无 | 固定 0 | 当前保留为 0 |
| `30006` | Number Of Points | `g_measurement.density_distribution.measurement_points` | 已接 | 官方采集过程中可递增；当前仅在 SI profile 完成锁存有效后输出有效点数 |
| `30007` | Profile Timestamp Month | CPU3 RTC 锁存时间 | 已接 | 官方语义为第一个 profile 点采集时间；当前在 CPU3 收到 SI `00004 Profile`、屏幕 SI profile 或自动 profile 触发时锁存 |
| `30008` | Profile Timestamp Day | CPU3 RTC 锁存时间 | 已接 | 同上 |
| `30009` | Profile Timestamp Hour | CPU3 RTC 锁存时间 | 已接 | 同上 |
| `30010` | Profile Timestamp Minute | CPU3 RTC 锁存时间 | 已接 | 同上 |
| `30011` | Current Time Hour | CPU3 RTC 当前时间 | 已接 | `Cpu3Clock_GetDateTime()` |
| `30012` | Current Time Minute | CPU3 RTC 当前时间 | 已接 | 同上 |
| `30013` | Current Time Second | CPU3 RTC 当前时间 | 已接 | 同上 |
| `30014~30020` | 保留 | 无 | 固定 0 | 当前保留为 0 |

### 8.2 点阵寄存器

SI 官方 DCS 地址表要求从 `30021` 开始按 3 个寄存器一组输出 profile 点，表内只暴露 Point 0 到 Point 199，即 200 个点。官方定义章节另提到设备内部最大 profile 点数可到 500；这不等于 DCS 表可直接读取 500 点。当前实现按官方 DCS 地址窗口 `30021~30620` 输出 200 点，超过 200 点必须另定扩展地址或分页机制。

- `30021 + 3n`：位置
- `30022 + 3n`：温度
- `30023 + 3n`：密度

当前实现已接入：

| SI 地址模式 | 当前变量来源 | 当前状态 | 说明 |
| --- | --- | --- | --- |
| `30021 + 3n` | `g_measurement.density_distribution.single_density_data[n].temperature_position` | 已接 | 内部 `0.1mm` 转 `mm` |
| `30022 + 3n` | `g_measurement.density_distribution.single_density_data[n].temperature` | 已接 | 内部原始温度按 `raw - 20000` 转有符号 `0.01°C` |
| `30023 + 3n` | `g_measurement.density_distribution.single_density_data[n].density` | 已接 | 内部与协议均按 `kg/m3 x100`，超过 16 位时钳位 |

当前上限来源：

- `MAX_MEASUREMENT_POINTS`
- 当前代码按 `SI_INPUT_REG_COUNT = 620` 做边界保护
- 只在 SI profile 完成锁存有效后复制 `30006 Number Of Points` 指定的有效点数，超出有效点数的点阵寄存器保持 0

## 9. 保持寄存器区 FC03 / FC06

当前 `40001~40003` 已正式绑定到 CPU2 SI profile 执行参数；`40010~40023` 已作为 CPU3 SI 本机参数持久化，支持 FC06 写入后回读，并参与自动 profile 调度和报警合成。

官方语义补充：profile 启动后先到罐底采集 Point 0；`40001 Profile First Point` 是罐底之后第一个编程停点，`40002 Profile Increment` 是向液面方向逐点增加的步距，`40003 Profile Dwell Time` 是每点停稳后的等待秒数。CPU2 不在 profile 前单独执行 `SearchOilLevel()`，候选点按底部、首点、步距、罐高和 200 点上限生成；逐点采样时若当前候选点被判定为液面以上点，则停止本轮 profile 且不把该点写入 profile 点阵。自动 profile 首次在 `40012/40013` 指定时刻运行，之后按 `40010` 分钟周期重复；如果只需要每天一次，官方建议 `40010=1440`。关闭自动 profile 按官方口径使用 `40011=0`，当前实现也按此处理，并额外拒绝写入 `40010=0`。

| SI 地址 | 文档含义 | 当前映射字段 | 当前状态 | 备注 |
| --- | --- | --- | --- | --- |
| `40001` | Profile First Point | `g_deviceParams.si_profile_first_point` | 已接 | 协议单位 mm，CPU2 存储单位 `0.1mm`；Point0 固定为底部点，40001 是 Point1 |
| `40002` | Profile Increment | `g_deviceParams.si_profile_increment` | 已接 | 协议单位 mm，CPU2 存储单位 `0.1mm`；从底部向上递增 |
| `40003` | Profile Dwell Time | `g_deviceParams.si_profile_dwell_time` | 已接 | 单位秒，表示停稳后的等待时间；写 0 返回非法数据值 |
| `40004~40009` | 文档保留/扩展 | 暂无 | 拒绝写入 | `FC06` 写入返回 Illegal Data Address，读取仍为 0 |
| `40010` | Automatic Profile Interval | `g_cpu3_comm_display_params.si_auto_profile_interval` | 已接 | CPU3 持久化，单位分钟；默认 60，禁止写 0；每天一次写 1440 |
| `40011` | Automatic Profile Enable | `g_cpu3_comm_display_params.si_auto_profile_enable` | 已接 | CPU3 持久化；官方 `0` 禁用、`1` 启用；当前仅接受 `0/1` |
| `40012` | Automatic Profile Hour | `g_cpu3_comm_display_params.si_auto_profile_hour` | 已接 | CPU3 持久化；仅接受 `0~23` |
| `40013` | Automatic Profile Minute | `g_cpu3_comm_display_params.si_auto_profile_minute` | 已接 | CPU3 持久化；仅接受 `0~59` |
| `40014` | Low Density Setpoint | `g_cpu3_comm_display_params.si_low_density_setpoint` | 已接 | CPU3 独立保存；`0` 是有效阈值 |
| `40015` | High Density Setpoint | `g_cpu3_comm_display_params.si_high_density_setpoint` | 已接 | 同上 |
| `40016` | Low Temperature Setpoint | `g_cpu3_comm_display_params.si_low_temperature_setpoint` | 已接 | CPU3 独立保存；按 SI 有符号温度口径比较，允许负值 |
| `40017` | High Temperature Setpoint | `g_cpu3_comm_display_params.si_high_temperature_setpoint` | 已接 | 同上 |
| `40018` | LL Level Setpoint | `g_cpu3_comm_display_params.si_ll_level_setpoint` | 已接 | CPU3 独立保存，不复用继电器或 AO 报警阈值 |
| `40019` | HH Level Setpoint | `g_cpu3_comm_display_params.si_hh_level_setpoint` | 已接 | 同上 |
| `40020` | Low Level Setpoint | `g_cpu3_comm_display_params.si_low_level_setpoint` | 已接 | CPU3 独立保存 |
| `40021` | High Level Setpoint | `g_cpu3_comm_display_params.si_high_level_setpoint` | 已接 | CPU3 独立保存 |
| `40022` | Temp Deviation Setpoint | `g_cpu3_comm_display_params.si_temp_deviation_setpoint` | 已接 | profile 相邻点温差报警阈值，温度差按 int16 温度值计算 |
| `40023` | Density Deviation Setpoint | `g_cpu3_comm_display_params.si_density_deviation_setpoint` | 已接 | profile 相邻点密差报警阈值 |

### 9.1 默认值、非法值和来源

原始资料没有提供完整的保持寄存器出厂默认值表，因此当前默认值按 CUBE 工程默认记录。联调和提交说明中不得把这些默认值写成 SI 官方出厂默认。

| 地址 | 当前 CUBE 默认 | 默认值来源 | 当前非法值/边界 | 非法值来源 |
| --- | --- | --- | --- | --- |
| `40001` | `100` mm | CUBE 工程默认 | FC06 写 `0` 返回非法数据值；CPU2 异常存储值运行期归一化 | CUBE 安全边界 |
| `40002` | `1000` mm | CUBE 工程默认 | FC06 写 `0` 返回非法数据值；CPU2 异常存储值运行期归一化 | CUBE 安全边界 |
| `40003` | `10` s | CUBE 工程默认 | `0` 拒写；CPU2 运行期限制 `1~3600` | CUBE 安全边界；Modbus 单位按官方 DCS 秒 |
| `40004~40009` | `0` | 官方保留 | 写入返回 `Illegal Data Address` | 官方保留地址 |
| `40010` | `60` min | CUBE 工程默认 | `0` 拒写；每天一次建议写 `1440` | 官方给出 enable 语义和 1440 建议；拒写 0 为 CUBE 防误配置 |
| `40011` | `0` | 官方语义 + CUBE 默认 | 仅接受 `0/1` | 官方 `0` 禁用、`1` 启用 |
| `40012` | `0` | CUBE 工程默认 | 仅接受 `0~23` | 时间字段自然边界 |
| `40013` | `0` | CUBE 工程默认 | 仅接受 `0~59` | 时间字段自然边界 |
| `40014` | `0` | CUBE 工程默认 | `0` 是有效阈值，不能作为禁用值 | 最新确认口径 |
| `40015` | `0` | CUBE 工程默认 | `0` 是有效阈值，不能作为禁用值 | 最新确认口径 |
| `40016` | `0` | CUBE 工程默认 | `0` 是有效阈值；允许负温度阈值 | 最新确认口径 |
| `40017` | `0` | CUBE 工程默认 | `0` 是有效阈值；允许负温度阈值 | 最新确认口径 |
| `40018~40021` | `0` | CUBE 工程默认 | `0` 是有效阈值，不能作为禁用值 | 最新确认口径 |
| `40022` | `0` | CUBE 工程默认 | `0` 是有效偏差阈值，不能作为禁用值 | 最新确认口径 |
| `40023` | `0` | CUBE 工程默认 | `0` 是有效偏差阈值，不能作为禁用值 | 最新确认口径 |

注意：

- `40003` 在操作手册的 Profile Setup 页面曾出现毫秒口径，但 DCS Modbus 资料明确 `40003` 为秒；外部 Modbus 兼容按秒。
- `40016/40017` 的寄存器语义按有符号 `0.01°C` 比较；CPU3 菜单和 Modbus 写入均需允许负温度阈值。
- 报警阈值 `0` 已确认为有效值。后续若需要禁用报警，应新增独立 enable 位或定义“全部阈值始终启用”，不能再用阈值 0 作为禁用条件。

## 10. 参数复用与独立边界

SI协议适配不再把所有写入都压到既有普通测量参数上，当前按“通用测量基础继续复用、SI 专用 profile/报警参数独立保存”的边界维护。

| 类型 | 字段或参数 | 归属 | 当前结论 |
| --- | --- | --- | --- |
| 通用测量基础 | `oilLevelFrequency`、`tankHeight`、`blindZone`、`oilLevelThreshold`、`oilLevelHysteresisThreshold`、`liquidLevelMeasurementMethod`、`oilLevelHysteresisTime` | CPU2 | 继续作为液位跟随、探底和测量闭环基础，不暴露 SI 寄存器语义 |
| SI profile 执行参数 | `si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time`、`si_profile_bottom_detect_interval` | CPU2 | 独立持久化；`40001~40003` 直接写入这些参数，探底频次由 CPU2 执行 profile 时消费，按进入 CPU2 SI profile 入口的触发次数计数 |
| SI 自动调度参数 | `si_auto_profile_interval`、`si_auto_profile_enable`、`si_auto_profile_hour`、`si_auto_profile_minute` | CPU3 | 独立持久化；CPU3 RTC 到点后触发 `CMD_SI_PROFILE` |
| SI 报警阈值 | `si_low_density_setpoint` 到 `si_density_deviation_setpoint` | CPU3 | 独立持久化；不复用继电器、AO、Wartsila 或普通密度测量阈值 |
| 不再作为 SI 参数源 | `spreadTopLimit`、`spreadMeasurementDistance`、`spreadPointHoverTime` | CPU2 | 仅保留普通分布测量用途，SI profile 写入不再影响这些参数 |

## 11. 当前最适合复用的命令入口

`FC05` 写线圈当前已优先桥接到这些现有命令；未列入的协议线圈保持保留：

- `CMD_BACK_ZERO`
- `CMD_FIND_OIL`
- `CMD_SI_PROFILE`
- `CMD_WARTSILA_DENSITY_RANGE`
- `CMD_CALIBRATE_ZERO`
- `CMD_CALIBRATE_OIL`
- `CMD_MOVE_UP`
- `CMD_MOVE_DOWN`
- `CMD_FORCE_MOVE_UP`
- `CMD_FORCE_MOVE_DOWN`

## 12. 本次落地结果

1. 已在 `CPU2` 补充 SI 状态字段中性状态，不改变既有测量核心基准。
2. 已在液位和分布测量流程里维护：
   - `probe_at_liquid_level`
   - `liquid_stable`
   - `profile_complete_latched`
   - `profile_complete_counter`
   - `bottom_reference_valid`
3. 已把 `40001~40003` 绑定到 CPU2 SI profile 执行参数，普通分布测量参数不再受 SI profile 写入影响。
4. 已在 `CPU3` 完成：
   - SI 外部 Modbus RTU 从站
   - 协议时间戳
   - 当前时间
   - 固定值/保留值输出
   - 位置、液位和温度工程量转换
   - CPU3 SI 本机参数持久化
   - 自动 profile 调度
   - 当前值、profile 上下限和 profile 相邻点偏差报警合成
5. 已细化 `FC05` 写线圈的 CPU2 命令桥接和快照区互斥，`FC02` 已接入当前能可靠合成的状态位。
6. 已优化 `FC06` 基础值域检查和 profile 开始时间戳锁存，避免非法自动 profile 参数进入持久配置。
7. 已把“探底失败且无旧底部位置”收敛为一次性 fallback：使用当前位置采集 Point0 并继续本轮 profile，但不刷新 SI 内部底部位置、不置底部位置有效、不清首次探底状态；该约束由 `tools/check_si_protocol_contract.py` 静态检查覆盖。
8. 已把 SI profile 从“预先找液位后裁剪点阵”改为“候选点逐点运行并实时判定液面以上点”：`CMD_SiProfile()` 不再调用 `SearchOilLevel()`，该约束由 `tools/check_si_protocol_contract.py` 静态检查覆盖。

## 13. 当前保留约束

以下约束和优化项已单独记录到 `docs/01_协议与寄存器/SI协议适配/01_计划与需求/SI协议待确认与后续清单.md`，当前实现只覆盖已确认范围：

- SI协议描述 `Device ID = Tank ID`；当前已确认复用现有 `SlaveAddress`，不新增独立 SI Tank ID 参数。若现场后续要求单独配置 Tank ID，再作为扩展方案处理。
- 慢/中/快三个手动速度档位当前统一桥接到上行或下行命令，CPU2 暂无三档速度参数。
- 官方资料明确上下液位传感器是液/气界面判断核心状态；当前按液位跟随稳定状态合成，未接入真实双传感器硬件。
- 密度已按文档示例输出为 `0.01 kg/m3`，但 16 位寄存器最高只能表达 `655.35 kg/m3`；超过范围时会钳位到 `65535`，该限制已确认可接受。
- CPU3 RTC 当前使用 LSI 时钟源，能满足时间显示和时间戳锁存，但长期精度依赖硬件时钟源确认。
- 官方 profile 时间戳为第一个 profile 点采集时间；当前确认保持 CPU3 收到或自动触发 profile 指令时锁存时间，不新增 CPU2 首点采集事件。
- `10004 Interlock` 和 `10009 Reel Alarm` 维持当前简化合成，不再细分卷尺、电机、校准等报警来源。
- 官方 DCS 地址表暴露 200 个 profile 点；官方定义章节另提到设备内部最大 profile 点数 500。直接扩到 500 会影响 CPU2/CPU3 共享结构、同步寄存器、内存、循环耗时、脚本和文档；若现场 PLC 要求超过 200 点，应新增扩展地址或分页机制，而不是直接扩大原 DCS 表。
- 官方编程建议可通过监控 `30011~30013` 当前时分秒持续变化来判断通信是否正常；联调时需确认 CPU3 RTC 正常运行。
- SI协议官方手册逐项默认值仍需现场复核；当前代码采用 CPU2 首点 100mm、步距 1000mm、停留 10s、探底频次 1，CPU3 自动周期 60min、默认关闭。报警阈值 `0` 是有效值，不能再作为禁用值。
- 后续建议补充实物主机侧协议帧测试、SI协议诊断计数，并评估 `si_modbus_slave.c` 拆分和旧 `com_manager` 兼容路径取舍。

## 14. 联调和验证重点

| 类别 | 操作 | 预期结果 |
| --- | --- | --- |
| 串口基础 | 使用 9600 8O1、当前站号轮询 `FC01~FC06` | 合法请求正常响应；保留地址、非法功能码、非法地址和非法值返回标准异常 |
| 模式生命周期 | 依次写 Manual、Auto、Cal、Profile、Stop、Up/Down 三档 | 模式线圈互斥；手动动作可打断自动/profile；Profile 读回只认 SI profile 命令状态，普通分布/Wärtsilä 分布不置 `00004` |
| Profile 参数 | 写入 `40001~40003` 后触发 `00004 Profile` | CPU2 使用 SI 独立 profile 参数执行；Point0 为底部点，Point1 从 `40001` 开始 |
| 屏幕触发 | CPU3 屏幕进入“密度分布测量 -> SI Profile”并确认下发 | 与 `00004 Profile` 一致锁存 `30007~30010` 并下发 `CMD_SI_PROFILE`；返回密度分布测量菜单 |
| 自动 profile | 写入 `40010~40013` 后观察起始时刻和 interval 周期 | 禁用时不触发；启用后从起始时间按 interval 周期运行，可跨天；自动触发下发 `CMD_SI_PROFILE` |
| Profile 数据 | 读取 `10005`、`30006`、`30007~30010` 和 `30021~30620` | 未完成时点数和点阵为 0；完成后点数与有效点一致，点数不超过 200；时间戳为测量开始时间 |
| 探底 fallback | 构造探底失败、有旧底部和无旧底部两类场景 | 有旧底部时沿用旧底部；无旧底部时使用当前位置采 Point0，但不刷新底部位置有效状态 |
| 液位传感器显示 | 观察液位跟随稳定和非跟随状态下 `10002/10003` | 液位跟随稳定时下液体、上空气；其它状态均显示在液体中 |
| 报警阈值 | 写入 `40014~40023` 并构造当前值和 profile 点越限场景 | 当前值报警、profile 上下限报警和相邻点偏差报警按阈值置位；`0` 是有效阈值，不作为禁用值；温度偏差按有符号温差判断 |
| 掉电恢复 | 写入 `40010~40023` 后重启 CPU3 | CPU3 SI 本机参数按持久化设计恢复；默认值、CRC 异常回退和禁用状态明确 |
| 当前时间 | 连续读取 `30011~30013` | 秒值持续变化；PLC 可用该字段辅助判断通信和 RTC 活性 |

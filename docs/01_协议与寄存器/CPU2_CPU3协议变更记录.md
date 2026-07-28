# CPU2/CPU3 协议变更记录

本文记录 CPU2 与 CPU3 之间共享寄存器、共享结构体和能力字段的协议版本。固件版本只表示各 CPU 自身软件版本，不能单独作为跨 CPU 兼容依据。

## 协议版本字段

- 字段位置：`HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION`
- 当前语义：CPU2/CPU3 共享协议版本
- 旧程序语义：保留字段，默认值为 `0`
- 当前正式固件组合：协议版本 `30`，CPU2 `V1.35.0.0` / CPU3 `V1.35.0.0`
- 当前开发语义：协议版本 `30`；新增23类整机供电与电源监控共享故障码，CPU2/CPU3仍按协议版本严格相等判断

该字段由原 `reserved1` 预留位正式替换而来，寄存器地址不移动，不新增存储字段。

## 版本记录

| 协议版本 | 首个 CPU2 版本 | 首个 CPU3 版本 | 密度分布最大点数 | 说明 |
| --- | --- | --- | --- | --- |
| 0 | 旧程序，或未写入协议版本字段 | 旧程序 | 100 | 原 `reserved1` 未定义协议语义；当前CPU3读到0时提示协议不兼容。 |
| 1 | V1.5.0.0 | V1.2.0.0 | 200 | 原 `reserved1` 正式替换为协议版本，密度分布测量、内部输入寄存器和 Wärtsilä 外部密度点扩展到 200 点。 |
| 2 | V1.6.0.0 | V1.4.0.0 | 200 | 新增 `CMD_CANCEL_MEASUREMENT = 16`，用于 CPU3 状态显示界面长按返回键取消当前测量并让 CPU2 进入待机。 |
| 3 | V1.7.0.0 | V1.5.0.0 | 200 | 原 `reserved23` 正式替换为探底修正罐高，用于罐底测量后编码器修正；瓦锡兰分布测后探底前先回固定点监测位置。 |
| 4 | V1.8.0.0 | V1.6.0.0 | 200 | 将 SI 所需补充状态融合进既有测量结构，并通过共享输入寄存器发布给 CPU3 外部协议转换层。 |
| 5 | V1.9.0.0 | V1.7.0.0 | 200 | 新增 `CMD_PAIR_NEAREST_WIRELESS_SLIPRING = 117`，用于 CPU3 菜单或共享命令通道触发 CPU2 执行无线滑环 RSSI 最近匹配；新增无线滑环匹配中/完成设备状态；输入寄存器末尾追加无线滑环匹配结果和从机 MAC 状态。 |
| 6 | V1.10.0.0 | V1.9.0.0 | 200 | 新增 `STATE_DEBUG_MODE = 0x0033`，用于 CPU2 串口调试指令执行期间通过 CPU3 显示“调试模式中”；原 `reserved2` 参数槽复用为故障自动恢复重跑上限；`empty_weight` 空载扭力按 `int32_t` 有符号 32 位解释，寄存器地址和后续字段不移动。 |
| 7 | V1.12.0.0 | V1.10.0.0 | 200 | 新增四路继电器报警输出配置和运行态共享区；CPU3 可显示、写入四路继电器报警输出配置，CPU2 执行 HH/H/L/LL、滞回、锁存清除和无效值策略。 |
| 8 | V1.14.0.0 | V1.13.0.0 | 200 | `liquidLevelMeasurementMethod` 液位测量方式语义扩展：保留 0/1 原相对频率和定频步进方案，补实 2=密度连续找液位，新增 4=连续相对频率、5=连续定频；CPU3 菜单和参数范围同步允许 0..5。 |
| 9 | V1.15.0.0 | V1.14.0.0 | 200 | 读取部件参数增加当前蓝牙连接 RSSI 快照；扩展 `WirelessPairingStatus` 并在输入寄存器尾部追加连接有效、RSSI 有效、RSSI 值、查询错误码和 RSSI 更新计数。 |
| 10 | V1.16.0.0 | V1.15.0.0 | 200 | 新增 AO 模拟电流输出服务和 AD5421 诊断；在 RSSI 运行态后追加 `AoOutputRuntime`；原 `reserved26` 正式替换为 `AoOutputEnable`，地址保持 `0x00C2-0x00C3` 不后移，CPU2/CPU3 同步支持 AO 输出启停，默认关闭。 |
| 11 | V1.18.1.0 | V1.16.1.0 | 200 | 复用 AO 相关保留字段为 AO 正常输出液位量程端点和独立报警液位阈值；同步 CPU2/CPU3 参数菜单、保持寄存器和运行期归一化语义。 |
| 12 | V1.19.0.0 | V1.17.0.0 | 200 | 删除共享命令 115 的强制提零点执行语义；命令码 115、状态码 `0x002F/0x802F` 改为保留，不再由 CPU3 菜单下发或由 CPU2 执行。 |
| 13 | V1.20.0.0 | V1.18.0.0 | 200 | 内部密度 raw 从 `kg/m3 x10` 升级为 `kg/m3 x100`；CPU3 状态页、密度参数菜单和 LTD 自有协议支持两位小数；DSM/Wartsila/SI协议在边界保持原对外口径。 |
| 14 | V1.21.0.0 | V1.19.0.0 | 200 | 新增 `CMD_SI_PROFILE = 20`、CPU2 SI profile 执行参数和 `density_distribution.profile_source` 共享状态；`40001~40003` 从普通分布参数解耦；CPU3 新增 SI 菜单、`40010~40023` 本机参数、自动 profile 调度、开始时间戳、状态/报警合成；SI profile 完成态和点阵只认 `PROFILE_SOURCE_SI`，并修正温度无效值、报警 0 阈值、负温度阈值、自动调度真实日历换算和 SI profile 不预先找液位的流程语义。 |
| 15 | V1.23.0.0 | V1.22.0.0 | 200 | 共享故障码按9个责任域重新分类和编号，新增TMC配置丢失、串口传输异常、无线响应格式异常和AD5421主动报警4项故障；CPU3本机主控通信故障调整为14-12。故障寄存器地址和宽度不变，但数值解释与协议14不兼容。 |
| 16 | 未单独发布（CPU2 V1.24.0.0开发过渡） | 未单独发布（CPU3 V1.22.0.0开发过渡） | 200 | 互换故障类别13和14：通信链路改为13类，传感器与密度改为14类；两类内部子码和故障语义不变。CPU3本机主控通信故障由14-12调整为13-12。故障寄存器地址和宽度不变，但数值解释与协议15及更早版本不兼容。 |
| 17 | V1.25.0.0 | V1.23.0.0 | 200 | 按二代计量仪责任域重新分类、删减并重编号故障码：CPU2保留63项共享故障，CPU3另有1项本机故障；正式表61项使用中、3项保留。CPU3本机主控通信故障由13-12调整为20-7。故障寄存器地址和宽度不变，但数值解释与协议16及更早版本不兼容。 |
| 18 | V1.26.0.0 | V1.24.0.0 | 200 | 在既有输入寄存器尾部、`REG_ENG` 前追加 `si_profile_runtime.phase`、`cycle_counter`、`progress_points` 三个32位运行态字段；用于 Point0 建立周期、有效点进度和回液位后最终快照门禁。SI 外部兼容同时增加 `30014~30016` 镜像、`40004~40009` CPU3 本地持久化槽和 CPU3 FRAM V7。 |
| 19 | 未单独发布；能力随V1.27.0.0发布 | 未单独发布；能力随V1.25.0.0发布 | 200 | 在协议17责任域基础上，把当前程序能够稳定区分且现场处理不同的多义故障全部拆分。CPU2共有129项共享故障，CPU3另有1项本机故障；正式表127项使用中、3项保留。故障寄存器地址和宽度不变，但故障码数值解释与协议18及更早版本不兼容。 |
| 20 | 未单独发布；能力随V1.27.0.0发布 | 未单独发布；能力随V1.25.0.0发布 | 200 | 在协议19故障码基础上重构AO配置和运行态：原13个连续AO持久化槽原位改为工作模式、电流模式、输出源、量程、阻尼、故障、上电和仿真参数；CRC后追加非持久化仿真开关；SI Profile运行态之后追加6个AO扩展运行态字段。参数存储版本和结构总尺寸保持不变，协议19及更早AO布局由CPU2迁移。 |
| 21 | V1.27.0.0 | V1.25.0.0 | 200 | 在协议20输入寄存器尾部追加单点测量完成代际和固定点监测样本代际两个32位字段。CPU2六字段同代发布后递增代际；CPU3执行代际前读、24个结果寄存器私有读取和代际后读，前后一致才原子提交公开缓存。 |
| 22 | 未单独发布；能力随V1.28.0.0发布 | 未单独发布；能力随V1.26.0.0发布 | 200 | 不改变寄存器地址、长度和字段含义；只按一代同义子码优先、二代新增项使用最低可用子码的规则重排32项共享故障码。CPU2 V1.27.0.0协议21编号完整归档，现场按CPU2程序版本选择故障表。 |
| 23 | V1.28.0.0 | V1.26.0.0 | 200 | 保持AO 13个参数槽和运行态地址不变，收敛故障动作、隐藏原错误等级槽位，并将原上电电流及运行态来源0统一解释为非跟随电流/非跟随状态；CPU2迁移协议20~22旧故障模式。 |
| 24 | V1.29.0.0 | V1.27.0.0 | 200 | AO输出源1改为传感器位置，非跟随电流改为初始电流，运行态增加保持状态；成功过程目标建立RAM缓存，命令切换、重新搜索和暂时无有效样本时保持上次成功过程目标。地址、结构尺寸和13个AO槽位不变。 |
| 25 | V1.30.0.0 | V1.29.0.0 | 200 | 原 `reserved3` 32位槽位正式定义为水位滞后时间预留参数，单位s、范围0~3600、默认0；当前只支持菜单、持久化和双端同步，不参与水位算法。地址、结构尺寸和 `DEVICE_PARAM_VERSION=3` 不变。 |
| 26 | 未单独发布；能力随V1.31.0.0发布 | 未单独发布；能力随V1.30.0.0发布 | 200 | AO第4个32位槽由SIL/WHG隐藏预留改为有符号电流修正值，单位0.01mA、范围-1.00~+1.00mA、默认0；启用态所有基础目标统一修正一次，禁用3.40mA不修正。地址、结构尺寸和`DEVICE_PARAM_VERSION=3`不变。 |
| 27 | V1.31.0.0 | V1.30.0.0 | 200 | 旧紧凑地址表由固定功能块地址直接替换，PDU地址作为CPU2/CPU3寄存器数组索引；CPU2 Holding为`0x0000~0x0FFF`，Input为`0x0000~0x29FF`，CPU3本机Holding为`0x7000~0x706B`。实现非阻塞维护、继电器最终逻辑报警动作/实际屏蔽发布、AO设备故障电流屏蔽及命令118/119，能力掩码为`0x0000001F`；未来物理触点反馈仍预留。`DEVICE_PARAM_VERSION=3`和原FRAM结构不变。 |
| 28 | V1.33.0.0 | V1.33.0.0 | 200 | 固定七个带参命令前置参数的`pending/active`运行快照；运行态允许写这些参数，命令执行和自动恢复使用已绑定快照。CPU3以合法FC10 ACK确认本次写值、更新局部镜像并设置逐字段位图，只放行实际需要这些字段的命令；普通运行态、完整参数和固定点快照分别门禁，普通命令不确定结果只消费对应确认资格，恢复出厂保持全量刷新。继电器禁用通道允许逐项配置，启用时执行完整校验，阻尼预留字段固定为0。地址、字段宽度、`DeviceParameters`、`DEVICE_PARAM_VERSION=3`和CPU3本机参数版本`0x0007`均不变。 |
| 29 | V1.34.0.0 | V1.34.0.0 | 200 | 取消不可达的12-11；18类改为扭力检测，21类改为模拟输出与自检；13-2统一为“震动管频率异常”；13-26密度有效上限提高到3000.0 kg/m³，CPU3手输原始上限同步为300000。不提供旧故障码别名或翻译；地址、字段宽度、参数结构和FRAM布局不变。 |
| 30 | V1.35.0.0 | V1.35.0.0 | 200 | 新增23类整机供电与电源监控故障：24 V欠压、ADC溢出、DMA停止、监控启动失败、连续3次恢复失败和掉电位置保存失败。故障寄存器地址和宽度不变，参数结构与FRAM布局不变；协议29及更早程序不能解释新编号。 |

## 兼容判断规则

1. CPU2 启动时把本机 `DeviceParameters.protocolVersion` 修正为当前协议版本，并发布到保持寄存器。
2. CPU3 上电或通信恢复后，先单独读取稳定地址 `0x000E~0x000F` 的 CPU2 协议版本字段；协议尚未确认前不得访问当前协议新增的扩展寄存器。
3. 只有 CPU2 协议版本与 CPU3 当前 `DEVICE_PROTOCOL_VERSION` 完全相同时，才判定协议兼容并继续建立状态、参数和运行态快照。
4. 已确认协议不匹配时，CPU3 只保留协议版本探测作为链路心跳，禁止命令和 CPU2 派生字段访问；设备状态页显示“协议版本不匹配”，不得误报为通信超时。
5. 固件版本只用于显示和追踪发布，不参与 CPU2/CPU3 协议兼容判断。
6. 协议不匹配属于整机状态问题，由设备状态页统一提示；CPU2固件版本参数页只显示版本号。只有连续请求未获得合法响应达到阈值时，才进入 CPU3 本机 `CPU2_COMM_TIMEOUT` 故障。

## 变更记录

### 协议版本 1

关联改动：
- 密度分布测量最大点数从 100 扩展到 200。
- CPU2 `MAX_MEASUREMENT_POINTS` 改为 200。
- CPU3 `MAX_MEASUREMENT_POINTS` 改为 200。
- CPU3 “分布测点数”参数上限改为 200。
- CPU3 Wärtsilä 外部 Modbus `REG_DENSITY_POINT_COUNT` 改为 200。

兼容影响：
- CPU3 V1.2.0.0 与 CPU2 V1.4.1.0 组合时，CPU2 协议版本为 0，不能认为完整兼容。
- CPU3 V1.2.0.0 与 CPU2 V1.5.0.0 组合时，CPU2 协议版本为 1，满足 200 点密度分布协议。

### 协议版本 2

关联改动：
- 新增共享命令 `CMD_CANCEL_MEASUREMENT = 16`，CPU2 与 CPU3 的 `CommandType` 枚举保持一致。
- CPU3 状态显示界面长按返回键时，不进入菜单，改为向 CPU2 写入取消测量命令。
- CPU2 收到取消测量命令后，取消故障自动恢复，停止电机，清除当前命令和错误码，并切换到待机状态。

兼容影响：
- CPU3 V1.4.0.0 与 CPU2 V1.6.0.0 组合时，双方协议版本为 2，支持状态界面长按返回取消测量。
- CPU3 V1.4.0.0 与旧 CPU2 组合时，旧 CPU2 不识别命令 16；协议版本不匹配时应按设备状态页提示处理，不建议混用。

### 协议版本 3

关联改动：
- 原 `reserved23` 正式替换为“探底修正罐高”，保持寄存器地址不移动，单位沿用 0.1mm。
- CPU2 罐底测量后的编码器修正优先使用“探底修正罐高”；该参数为 0 时兼容旧逻辑，回退使用液位罐高。
- 瓦锡兰分布测量完成并达到测后探底频率后，先移动到固定点监测位置，仅移动不读取密度，再执行罐底测量。
- 瓦锡兰测后固定点移动或后续探底失败时，退出后续流程并切回单点监测，不通过 `SET_ERROR` 置错误状态。
- 罐底测量实高与编码器修正目标罐高的偏差大于实高最大偏差时，跳过编码器修正。

兼容影响：
- CPU2/CPU3 必须同为协议版本 3 才能正确同步“探底修正罐高”字段。
- 旧协议存储升级到协议版本 3 时，CPU2 将新增“探底修正罐高”参数恢复为 0，保持旧逻辑兼容。

### 协议版本 4

关联改动：
<- 将 SI 所需补充状态按业务含义融合到 CPU2/CPU3 既有测量结构：`DeviceStatus`、`OilMeasurement`、`ActualHeightMeasurement` 和 `DensityDistribution`。
- CPU2 发布探底参考有效、探头位于液位、液位稳定、profile 完成锁存、profile 完成计数、profile 被工况阻止、装卸液状态、手动报警抑制、手动液位更新抑制、profile 温度偏差报警、profile 密度偏差报警。
- CPU3 读取该状态区后供外部协议转换层使用，SI 地址、线圈、缩放和异常响应只存在于 CPU3 外部协议模块，避免外部协议直接反推或污染 CPU2 内部流程状态。
- 没有新增独立 `ProtocolAssistStatus` 结构，避免在 `MeasurementResult` 里再维护一套外部协议专用镜像；所有字段都落在原业务结构尾部，并由 CPU2/CPU3 两侧同名结构同步。

新增共享字段明细：

| 所属结构 | 新增字段 | 共享寄存器 | 语义 |
| --- | --- | --- | --- |
| `DeviceStatus` | `loading_unloading_active` | `REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE` | 当前是否处于装卸液过程，供外部协议判断工况。 |
| `DeviceStatus` | `manual_alarm_inhibit` | `REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT` | 手动/强制动作期间抑制自动报警语义，避免 CPU3 将手动过程误判为自动测量结果。 |
| `OilMeasurement` | `probe_at_liquid_level` | `REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL` | 找液位成功后置位，SI 可映射为 Probe At Liquid Level。 |
| `OilMeasurement` | `liquid_stable` | `REG_OIL_MEASUREMENT_LIQUID_STABLE` | 找液位成功后置位，表示当前液位结果可认为稳定。 |
| `OilMeasurement` | `manual_level_update_inhibit` | `REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT` | 手动/强制动作期间抑制液位自动更新语义。 |
| `ActualHeightMeasurement` | `bottom_reference_valid` | `REG_HEIGHT_MEASUREMENT_BOTTOM_REFERENCE_VALID` | 探底成功或使用有效回退罐高后置位，SI 可映射为 Bottom Reference。 |
| `DensityDistribution` | `profile_complete_latched` | `REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED` | 分布测量成功后锁存完成状态，失败或命令切换不置位。 |
| `DensityDistribution` | `profile_complete_counter` | `REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER` | 每次 profile 成功完成后递增，CPU3 用计数变化锁存 SI profile 时间戳。 |
| `DensityDistribution` | `profile_blocked_by_process` | `REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS` | profile 被当前工况阻止时置位，供 CPU3 转换为外部协议互锁/阻止状态。 |
| `DensityDistribution` | `profile_temp_deviation_alarm` | `REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM` | 分布温度偏差报警状态。 |
| `DensityDistribution` | `profile_density_deviation_alarm` | `REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM` | 分布密度偏差报警状态。 |

寄存器布局影响：
- `DeviceStatus` 尾部新增 2 个 `uint32_t`，`REG_DEBUG_BASE` 随之顺延。
- `OilMeasurement` 尾部新增 3 个 `uint32_t`，`REG_WATER_MEASUREMENT_WATER_LEVEL` 随之顺延。
- `ActualHeightMeasurement` 尾部新增 1 个 `uint32_t`，`REG_SINGLE_POINT_MEAS_TEMP` 随之顺延。
- `DensityDistribution` 在 `REG_DENSITY_DIST_OIL_LEVEL` 后新增 5 个 `uint32_t`，`REG_DENSITY_DIST_POINT_BASE` 随之顺延。
- CPU2 与 CPU3 的结构字段顺序、寄存器宏表达式和读写打包顺序必须完全一致，否则后续寄存器会错位。

兼容影响：
- CPU2/CPU3 必须同为协议版本 4 才能正确同步融合后的 SI 状态字段。
- 旧 CPU3 不识别新增状态寄存器，不能完整支持 SI 的离散输入状态映射。
- 旧 CPU2 不发布这些新增状态字段，CPU3 V1.6.0.0 不能依赖旧协议数据生成完整 SI 状态。
- 由于多个分组尾部寄存器顺延，协议版本 4 与协议版本 3 不能混用；必须依赖 `DEVICE_PROTOCOL_VERSION` 严格相等检查拦截。

验证结果：
- `py tools\check_si_protocol_contract.py`：确认 CPU2/CPU3 的结构字段、寄存器宏和读写打包顺序一致。
- `py tools\check_si_modbus_frames.py`：确认 SI 外部地址常量和 golden frame 一致。
- `py tools\check_version_bumped.py`：确认 CPU2 V1.8.0.0、CPU3 V1.6.0.0 已匹配本次协议升级。
- `cmake --build build\LTD_MAIN_CPU2`、`cmake --build build\LTD_DISPLAY_CPU3`：两端构建通过。

### 协议版本 5

关联改动：
- 新增共享命令 `CMD_PAIR_NEAREST_WIRELESS_SLIPRING = 117`，CPU2 与 CPU3 的 `CommandType` 枚举保持一致。
- CPU3 维护/调试菜单新增“匹配无线滑环”无参入口，通过现有 `command` 保持寄存器写入 CPU2。
- CPU2 收到该命令后不执行电机 `MeasureStart()` 初始化，直接复用 `WirelessPairing_RunByRssi()` 执行 CH9141K 扫描、RSSI 最近候选选择、连接、默认连接保存、模块复位和主/从节点透传探测。
- 保留 CPU2 调试串口 `SPR`，正式命令与调试命令复用同一 RSSI 匹配流程。
- `DeviceState` 新增 `STATE_WIRELESS_PAIRING = 0x0032` 和 `STATE_WIRELESS_PAIRING_OVER = 0x8032`，用于 CPU3 设备状态页显示无线滑环匹配中和匹配完成。
- 在输入寄存器末尾追加 `WirelessPairingStatus`，不移动协议版本 4 既有输入寄存器地址。字段包括：
  - `result`：`0` 未执行或无结果，`1` 正在匹配，`2` 匹配成功，`3` 匹配失败。
  - `mac_valid`：成功 MAC 是否有效。
  - `mac_high` / `mac_mid` / `mac_low`：分别保存 `AA:BB`、`CC:DD`、`EE:FF` 三段 MAC。
  - `error_code`：匹配失败时的 CPU2 错误码。
  - `update_counter`：CPU2 每次匹配状态变化时递增，CPU3 用于识别新结果。
- CPU3 收到完成设备状态和成功结果后，在设备状态页显示两行从机 MAC；匹配失败时 CPU2 将 `device_status.device_state` 置为 `STATE_ERROR` 并写入 `device_status.error_code`，CPU3 按现有故障页显示错误码。该 MAC 状态为运行态共享数据，不写入 FRAM 参数。

兼容影响：
- CPU2/CPU3 必须同为协议版本 5 才能通过 CPU3 菜单触发无线滑环最近匹配并正确读取 MAC 状态。
- 协议版本 4 仅包含 SI 辅助状态，不包含命令 117 和 `WirelessPairingStatus`；CPU3 V1.7.0.0 不应与协议版本 4 的 CPU2 混用。
- 旧 CPU3 不提供该菜单入口，但不影响 CPU2 新版本通过调试串口 `SPR` 执行匹配。
- 旧协议 CPU3 不知道追加的 `WirelessPairingStatus` 字段；协议版本不匹配时不应继续解释匹配结果。

### 协议版本 6

关联改动：
- `DeviceState` 新增 `STATE_DEBUG_MODE = 0x0033`，CPU2 与 CPU3 枚举值保持一致。
- CPU2 串口调试命令中，除调用正式测量流程的 `F/H/J`、正式命令映射、无线滑环 `SP*` 和演示 `X` 之外，执行期间临时发布 `STATE_DEBUG_MODE`。
- CPU2 串口 `B/BE` 诊断进入时切换到 `STATE_DEBUG_MODE`，退出时恢复进入前的业务状态和错误码快照；诊断期间清除临时错误时仍保持调试模式显示。
- 原 `reserved2` 正式替换为 `fault_auto_recovery_retry_limit`，用于控制故障自动恢复确认成功后最多自动重跑原命令次数：`0` 关闭，`1~10` 为上限，默认 `3`。
- `empty_weight` 空载扭力按 `int32_t` 有符号 32 位参数解释，寄存器仍占 2 个 word，负值按二进制补码传输。
- CPU3 设备状态页新增 `STATE_DEBUG_MODE` 显示文案“调试模式中”，英文文案为 `Debug Mode`。
- CPU3 参数页将原 `COM_NUM_DEVICEPARAM_RESERVED2` 显示为“故障自动恢复重跑次数”，仍写入 `HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION + REG_STRIDE` 对应地址。
- CPU3 外部 DSM 状态转换层将内部 `STATE_DEBUG_MODE` 对外映射为既有维护模式状态，避免 DSM 主站收到未知 `0x0033`。

寄存器布局影响：
- 不新增保持寄存器或输入寄存器地址，不移动后续参数地址。
- `DeviceParameters` 原 `reserved2` 字段语义替换为 `fault_auto_recovery_retry_limit`；CPU2/CPU3 Modbus 打包顺序不变。
- `HOLDREGISTER_DEVICEPARAM_EMPTY_WEIGHT` 地址和长度不变，仅将 32 位寄存器内容解释为有符号值。
- CPU2 旧协议存储升级到协议版本 6 时，将该字段补为默认 `3`，避免旧预留值 `0` 被误解释为关闭自动恢复。
- 本次提升协议版本的原因是共享 `DeviceState` 语义新增和共享参数语义新增，旧 CPU3 无法正确显示新状态，也无法展示/写入新参数语义。

兼容影响：
- CPU2/CPU3 必须同为协议版本 6 才能正确显示串口调试模式状态，并正确同步故障自动恢复重跑上限。
- CPU2 协议版本 6 与旧 CPU3 混用时，旧 CPU3 不识别 `STATE_DEBUG_MODE`；应由协议版本不匹配检查拦截。
- CPU3 协议版本 6 与旧 CPU2 混用时，旧 CPU2 不会发布 `STATE_DEBUG_MODE`，原 `reserved2` 也没有故障恢复次数语义；仍应由协议版本不匹配检查拦截。

### 协议版本 7

关联改动：
- 在 CPU2/CPU3 共享 `DeviceParameters` 的 `reserved33` 后、元信息字段前追加 `RelayAlarmConfig relayAlarm[4]`。
- 每路继电器配置占 13 个 32 位字段，字段顺序为：`operating_mode`、`digital_source`、`contact_type`、`alarm_mode`、`error_value`、`alarm_source`、`HH_alarm_value`、`H_alarm_value`、`L_alarm_value`、`LL_alarm_value`、`alarm_hysteresis`、`damping_factor`、`clear_alarm`。
- `HH/H/L/LL_alarm_value` 与 `alarm_hysteresis` 按 IEEE754 float 原始位通过两个保持寄存器传输；CPU3 菜单按 1 位小数显示/输入。
- 新增保持寄存器基址 `HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_BASE`，四路配置结束地址为 `HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_END`；`param_version`、`struct_size`、`magic`、`crc` 顺延到新配置之后。
- CPU3 新增四路继电器报警输出配置菜单和枚举文字表；原 `AlarmHighDO`、`AlarmLowDO`、`ThirdStateThreshold` 旧 DO 报警接口直接删除，后续 AO、指令参数、尺带补偿、继电器报警输出配置和元信息寄存器整体前移。
- 在 CPU2/CPU3 共享 `MeasurementResult` 的末尾追加 `RelayAlarmRuntimeState relay_alarm_runtime[4]`，用于发布每路当前报警值、HH/H/L/LL、组合报警、任意报警和清锁存运行态。
- `RelayAlarmRuntimeState` 字段顺序与参考程序只读区/不存储区一致：`alarm_value`、`HH_alarm`、`H_alarm`、`HH_H_alarm`、`L_alarm`、`LL_alarm`、`LL_L_alarm`、`any_error`、`clear_alarm`。
- CPU2 参数结构版本同步提升到 `DEVICE_PARAM_VERSION=3`，旧 FRAM 参数不会按新结构误读。

寄存器布局影响：
- 保持寄存器在 `HOLDREGISTER_DEVICEPARAM_RESERVED33 + REG_STRIDE` 后新增 104 个寄存器（四路 * 13 字段 * 2 寄存器）。
- 同时删除旧 DO 报警 3 个 32 位字段，AO 及其后续保持寄存器前移 6 个寄存器；协议版本 7 相对协议版本 6 的保持寄存器净增量为 98 个寄存器。
- 元信息与 CRC 寄存器整体后移，`HOLEREGISTER_STOP` 随之增大。
- 输入寄存器在无线滑环匹配状态后追加四路继电器运行态：`REG_RELAY_ALARM_RUNTIME_BASE = REG_WIRELESS_PAIRING_UPDATE_COUNTER + REG_SIZE_U32`。
- 每路继电器报警输出运行态占 10 个输入寄存器：`alarm_value` 按 IEEE754 float 占 2 个寄存器，其余 8 个状态字段各占 1 个寄存器；四路共追加 40 个输入寄存器，`REG_ENG` 随之后移。
- CPU3 上电和运行期轮询组 3 从 `REG_WIRELESS_PAIRING_RESULT` 读到新的 `REG_ENG`，同时同步无线滑环匹配状态和继电器运行态。

兼容影响：
- CPU2/CPU3 必须同为协议版本 7 才能正确显示、写入和执行四路继电器报警输出配置，并正确读取四路运行态。
- 协议版本 6 的 CPU3 不知道新增继电器配置寄存器、输入运行态寄存器和菜单，不能配置或解释继电器。
- 协议版本 6 的 CPU2 不执行新增 `relayAlarm[4]` 配置，也不会发布四路继电器运行态和顺延后的元信息地址；协议版本不匹配应由 CPU3 严格相等检查拦截。

验证结果：
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过。
- 未做实物联调；需要现场验证 RELAY1~RELAY4 输出极性、锁存清除、无效值故障策略和 CPU3 运行态显示/读取。

### 协议版本 8

关联改动：
- `DeviceParameters.liquidLevelMeasurementMethod` 仍使用原保持寄存器和原 32 位字段，不新增或移动寄存器。
- 液位测量方式语义调整为：`0=相对频率`、`1=定频`、`2=密度连续找液位`、`3=超声预留`、`4=连续相对频率`、`5=连续定频`。
- CPU2 在 `SearchOilLevel()` / `FollowOilLevel()` 中保留 0/1 原步进式频率方案；方法 2 进入密度阈值速度闭环；方法 4/5 进入频率偏差速度闭环。
- CPU3 “液位测量方式”菜单新增“连相对频率”和“连定频”，并将参数元数据有效范围同步为 `0..5`。

寄存器布局影响：
- 不新增保持寄存器、输入寄存器、命令码或共享结构体字段。
- `DeviceParameters` 结构大小、`DEVICE_PARAM_VERSION`、`param_version`、`struct_size`、`magic` 和 `crc` 字段位置不变。
- 本次提升协议版本的原因是已有字段 `liquidLevelMeasurementMethod` 的跨 CPU 参数语义发生变化；旧协议 7 的 CPU3 或外部工具可能不知道 2/4/5 的新含义。

兼容影响：
- CPU2/CPU3 必须同为协议版本 8，才能正确显示、写入并执行新的连续找液位方式。
- 协议版本 7 的 CPU3 只知道旧液位测量方式语义，不应继续向协议版本 8 的 CPU2 写入或解释该字段。
- 协议版本 7 的 CPU2 不支持新的 2/4/5 连续找液位执行路径，协议版本不匹配应由 CPU3 严格相等检查拦截。

验证结果：
- `py -3 tools\check_density_level_control_contract.py`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- 未做实物联调；需要现场验证密度连续、连续相对频率、连续定频三种新方案的电机速度闭环方向、速度响应、稳定判定和命令切换退出。

### 协议版本 9

关联改动：
- `CMD_READ_PART_PARAMS` / `STATE_READPARAMETEROVER` 读取部件参数流程增加当前 CH9141K 蓝牙连接 RSSI 只读查询。
- CPU2 新增 `WirelessPairing_ReadConnectionStatus()` 和 `WirelessPairing_UpdateConnectionStatusSnapshot()`，复用 `BLEMODE`、`BLESTA`、`CCADD` 和 `AT+RSSI=ON/OFF` 查询链路；不扫描、不重新配对、不保存默认连接。
- `WirelessPairingStatus` 在原匹配结果字段后新增 `connection_valid`、`rssi_valid`、`rssi`、`connection_error_code`、`rssi_update_counter`。
- CPU3 读取参数完成态新增 RSSI 显示项：有效时显示 `RSSI:<value>dB`，无效时显示 `RSSI:N/A`。

寄存器布局影响：
- 保留协议版本 8 既有无线滑环匹配状态和四路继电器运行态地址。
- RSSI 运行态字段追加在继电器运行态之后：
  - `REG_WIRELESS_PAIRING_CONNECTION_VALID`
  - `REG_WIRELESS_PAIRING_RSSI_VALID`
  - `REG_WIRELESS_PAIRING_RSSI`
  - `REG_WIRELESS_PAIRING_CONNECTION_ERROR_CODE`
  - `REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER`
- 协议版本 9 中 `REG_ENG` 顺延到 `REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER + REG_SIZE_U32`。

兼容性影响：
- CPU2/CPU3 必须同为协议版本 9，才能正确读取和显示当前蓝牙连接 RSSI。
- 协议版本 8 的 CPU3 不知道 RSSI 尾部输入寄存器，不应解释新增字段。
- 协议版本 8 的 CPU2 不发布 RSSI 快照，协议版本不匹配应由 CPU3 严格相等检查拦截。

验证结果：
- `py -3 tools\check_si_protocol_contract.py`：通过。
- `py -3 tools\check_density_level_control_contract.py`：通过。
- `py -3 tools\check_read_part_params_refresh_contract.py`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- 未做实物联调；需要现场验证已连接、未连接、RSSI 异步超时、命令切换打断和 UART6 透传恢复。

### 协议版本 10

关联改动：
- CPU2 新增 AO 模拟电流输出服务，统一负责液位电流计算、报警电流覆盖、故障/调试电流选择、AD5421 写入和运行态维护。
- AD5421 驱动新增有限 SPI 超时、控制寄存器回读、FAULT 管脚/READFAULT 诊断和专用错误码：`AD5421_INIT_ERROR`、`AD5421_WRITE_CURRENT_ERROR`、`AD5421_FAULT_PIN_ERROR`、`AD5421_READFAULT_ERROR`、`AD5421_READBACK_ERROR`。
- HART Command 2/3 的电流值改为读取 AO 运行态，百分比按标准 4-20mA 量程 `(mA - 4.0) / 16.0` 计算并钳位。
- CPU2/CPU3 共享输入寄存器在 RSSI 运行态后追加 `AoOutputRuntime`，用于发布 AO 目标电流、最近成功写入电流、来源、AD5421 故障标志、READFAULT 原始值、最近错误码和更新时间。
- 原 `reserved26` 正式替换为 `AoOutputEnable`，语义为 `0=关闭`、`1=启用`。
- CPU2 恢复出厂默认值将 `AoOutputEnable` 置为 `0`；旧协议存储升级到协议版本 10 时也强制置为 `0`，避免旧预留值误启用电流输出。
- CPU2 AO 服务在关闭时不初始化、不诊断、不写入 AD5421，并将运行态来源置为 `AO_OUTPUT_SOURCE_DISABLED`，用于未接电流环场景避免 AD5421 环路故障上报。
- CPU3 同步解析新增 AO 运行态字段，补充 AD5421 相关错误码的屏幕故障文案，并通过参数表、保持寄存器读写、参数同步指针和 AO 菜单支持 `AoOutputEnable`。
- 同步收紧 `bottom_detect_mode` 的显示与写入范围为 `0..1`，与 CPU2 实际枚举 `0=扭力`、`1=陀螺仪角度` 一致；异常值运行期归零，避免 CPU3 菜单显示“非法配置”或探底准备流程误判。

寄存器布局影响：
- 输入寄存器在协议版本 9 的 `REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER` 后追加 AO 运行态：
  - `REG_AO_OUTPUT_RUNTIME_TARGET_MA_X100`
  - `REG_AO_OUTPUT_RUNTIME_LAST_SENT_MA_X100`
  - `REG_AO_OUTPUT_RUNTIME_SOURCE`
  - `REG_AO_OUTPUT_RUNTIME_DRIVER_FAULT_FLAGS`
  - `REG_AO_OUTPUT_RUNTIME_DRIVER_FAULT_REGISTER`
  - `REG_AO_OUTPUT_RUNTIME_LAST_ERROR_CODE`
  - `REG_AO_OUTPUT_RUNTIME_UPDATE_COUNTER`
  - `REG_AO_OUTPUT_RUNTIME_LAST_UPDATE_TICK`
  - `REG_AO_OUTPUT_RUNTIME_LAST_SENT_TICK`
- `REG_ENG` 和 `INPUTREGISTER_AMOUNT` 随新增 AO 运行态顺延；旧输入寄存器地址不移动。

- 保持寄存器不新增地址，`HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE = HOLDREGISTER_DEVICEPARAM_DEBUG_CURRENT_mA + REG_STRIDE`。
- `HOLDREGISTER_DEVICEPARAM_RESERVED26` 作为兼容别名等于 `HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE`。
- `HOLDREGISTER_DEVICEPARAM_RESERVED27` 仍为 `HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE + REG_STRIDE`，后续保持寄存器地址不移动。
- `DeviceParameters` 结构体大小和 `DEVICE_PARAM_VERSION` 不变；本次只改变原预留字段语义并追加输入运行态。

兼容性影响：
- CPU2/CPU3 必须同为协议版本 10，才能正确读取 AO 运行态、识别 AD5421 专用错误码，并正确显示、写入和执行 AO 输出使能。
- 协议版本 9 的 CPU3 不知道新增 AO 输入寄存器尾部、AD5421 错误码文案和 `AoOutputEnable` 参数语义，不应与协议版本 10 的 CPU2 混用。
- 协议版本 9 的 CPU2 不发布 AO 运行态，也不按 `AoOutputEnable` 控制 AO 服务，协议版本 10 的 CPU3 应由严格协议版本相等检查拦截。

验证结果：
- `py -3 tools\check_ao_output_enable_contract.py`：通过。
- `py -3 tools\check_si_protocol_contract.py`：通过。
- `py -3 tools\check_density_level_control_contract.py`：通过。
- `py -3 tools\check_wireless_rssi_contract.py`：通过。
- `py -3 tools\check_read_part_params_refresh_contract.py`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- 未做实物联调；需要现场验证 `AoOutputEnable=0` 且电流环未接时不上报 AO 故障，`AoOutputEnable=1` 时 4-20mA 输出、HART 电流值和 AD5421 故障诊断恢复正常。

### 协议版本 11

关联改动：
- 保持寄存器地址不新增、不移动，`AOStartLevel_01mm/AOEndLevel_01mm` 复用原 `reserved24/reserved25` 参数位置作为 AO 正常输出液位量程端点。
- `AOStartLevel_01mm` 复用原 `reserved24` 参数位置，语义为 AO 起点液位，单位 `0.1mm`，对应 `CurrentRangeStart_mA` 正常起点电流。
- `AOEndLevel_01mm` 复用原 `reserved25` 参数位置，语义为 AO 终点液位，单位 `0.1mm`，对应 `CurrentRangeEnd_mA` 正常终点电流。
- `CurrentRangeStart_mA/CurrentRangeEnd_mA` 保留为正常输出起点/终点电流，单位 `0.01mA`；允许起点电流大于终点电流以支持反向 20mA->4mA 输出，二者相等时 CPU2 归一化为默认 4.00mA/20.00mA。
- `AlarmHighAO/AlarmLowAO` 语义改为 AO 独立高/低报警液位阈值，单位 `0.1mm`。
- `AOHighCurrent_mA/AOLowCurrent_mA` 继续表示 AO 高/低报警触发后的输出电流，单位 `0.01mA`；`InitialCurrent_mA/AOHighCurrent_mA/AOLowCurrent_mA/FaultCurrent_mA/DebugCurrent_mA` 统一按 AD5421 硬件输出范围限制为 `3.20..24.00mA`。
- CPU2 AO 输出不再读取继电器 `HH/H` 或 `L/LL` 运行态覆盖电流，改为按 `AlarmHighAO/AlarmLowAO` 独立判断 AO 报警；继电器报警继续使用各路 `relayAlarm[]` 阈值。
- AO 高低报警阈值中任一项为 `0` 表示关闭对应报警；两者均非 `0` 时应保持低报警液位低于高报警液位。如果高报警超出罐高，CPU2 写参后钳位到罐高；如果低报警超出罐高或高低报警重叠，CPU2 写参后恢复为出厂默认高报 `tankHeight`、低报 `0`。

寄存器布局影响：
- 保持寄存器地址和数量不变。
- `HOLDREGISTER_DEVICEPARAM_AO_START_LEVEL` 和 `HOLDREGISTER_DEVICEPARAM_AO_END_LEVEL` 从保留字段变为 AO 液位量程端点。
- `HOLDREGISTER_DEVICEPARAM_AO_NORMAL_CURRENT_START_mA`、`HOLDREGISTER_DEVICEPARAM_AO_NORMAL_CURRENT_END_mA` 地址不变，仅明确为电流端点。
- `HOLDREGISTER_DEVICEPARAM_AO_HIGH_ALARM_LEVEL`、`HOLDREGISTER_DEVICEPARAM_AO_LOW_ALARM_LEVEL` 地址不变，字段单位从旧电流口径修正为液位阈值口径。
- CPU3 操作号同步改为 `COM_NUM_DEVICEPARAM_AO_START_LEVEL`、`COM_NUM_DEVICEPARAM_AO_END_LEVEL`、`COM_NUM_DEVICEPARAM_AO_NORMAL_CURRENT_START_mA`、`COM_NUM_DEVICEPARAM_AO_NORMAL_CURRENT_END_mA`、`COM_NUM_DEVICEPARAM_AO_HIGH_ALARM_LEVEL`、`COM_NUM_DEVICEPARAM_AO_LOW_ALARM_LEVEL`。
- `DeviceParameters` 结构体大小和 `DEVICE_PARAM_VERSION` 不变。

兼容性影响：
- CPU2/CPU3 必须同为协议版本 11，才能正确显示和写入 AO 液位量程端点、AO 独立报警液位阈值和状态电流值。
- 协议版本 10 的 CPU3 会把 `reserved24/reserved25` 显示为保留字段，并把 `AlarmHighAO/AlarmLowAO` 显示为报警电流，不能与协议版本 11 的 CPU2 混用。
- CPU2 从旧协议存储升级到协议版本 11 时，会把 `AOStartLevel_01mm=0`、`AOEndLevel_01mm=tankHeight`、`AlarmHighAO=tankHeight`、`AlarmLowAO=0` 写入运行参数，避免旧保留值或旧电流口径被误解释成液位报警阈值；运行期写参后也会把 AO 液位端点归一化到 `0..tankHeight` 且保持起点小于终点，并把特殊 AO 电流参数归一化到 `320..2400` 后回读。
- `DEVICE_PARAM_VERSION` 保持不变，不会因本次升级触发恢复出厂参数；但协议语义变化后需要现场复核 AO起点/终点液位点、正常起点/终点电流方向和 AO 报警液位阈值。

验证结果：
- `git diff --check`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。
- 未做实物联调；需要现场验证 AO起点/终点液位量程、正向和反向正常电流映射、AO 独立高/低报警电流覆盖，以及继电器报警阈值不再影响 AO 报警输出。

### 协议版本 12

关联改动：
- 共享命令 `115` 从原强制提零点执行命令改为 `CMD_RESERVED_CMD7`。
- CPU2 删除 `CMD_ForceLiftZero()` 执行入口；收到命令 115 时按保留/未知指令处理，只打印暂不支持，不启动电机。
- CPU3 删除“强制提零点”菜单项、无参命令映射、电机运行监控归类和状态页文案。
- 设备状态 `0x002F`、`0x802F` 改为保留状态，不再作为强制提零点运行/完成状态发布或显示。

兼容性影响：
- CPU2/CPU3 必须同为协议版本 12，才能确保命令 115 不再被解释为可执行电机动作。
- 协议版本 11 的 CPU3 仍可能下发命令 115；协议版本 12 的 CPU2 会拒绝执行，但应通过协议不匹配提示避免混用。
- 协议版本 11 的 CPU2 仍可能执行命令 115；协议版本 12 的 CPU3 不再提供该菜单入口，但仍必须依赖严格协议版本相等检查避免旧 CPU2/新 CPU3 混用。
- 本次不新增、不移动寄存器字段，不改变 `DeviceParameters` 存储结构；协议升级原因是跨 CPU 共享命令语义发生变化。

验证结果：
- `py -3 tools\check_reserved_cmd7_contract.py`：通过。
- `py -3 tools\check_ao_output_enable_contract.py`：通过。
- `py -3 tools\check_wireless_rssi_contract.py`：通过。
- `py -3 tools\check_version_bumped.py`：通过。
- `git diff --cached --check`：通过。
- `cmake --build build\LTD_MAIN_CPU2`：通过。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过。

### 协议版本 13

关联改动：
- CPU2 `DENSITY_TO_RAW()` / `RAW_TO_DENSITY()` 统一改为 `kg/m3 x100`。
- CPU2 `DEVICE_PARAM_VERSION` 保持 `3`，通过旧 FRAM 中 `protocolVersion < 13` 识别一次性迁移 `oilLevelDensity`、`oilLevelThreshold`、`oilLevelHysteresisThreshold` 和 `densityCorrection`。
- `densityCorrection` 零点从 `10000` 迁移到 `100000`，修正量从 `(raw - 10000) / 10` 改为 `(raw - 100000) / 100`。
- CPU3 状态页密度显示小数位改为 2，尾零裁剪逻辑保留。
- CPU3 参数菜单中 `液位跟随密度`、`磁通量D`、`密度手输值` 改为两位密度口径；`液位找液阈值`、`液位滞后阈值` 继续面向频率液位显示为 `Hz`，并通过 `point=1` 保持默认 `150/200` 对应 `15.0/20.0 Hz`。
- CPU3 本机 FRAM 参数版本升级到 `0x0005`，读取 `0x0003` 或 `0x0004` 时迁移本机手输密度 `screen_input_d`。
- DSM 外部协议输出密度和密度修正时保持原 `x10` 口径；外部写入密度修正时转换回内部 `x100`。
- Wartsila 外部协议密度继续保持 `scale = 10` / `x10`。
- SI协议继续保持既有 `0.01 kg/m3` 口径，内部升级后取消原先从 `x10` 到 `x100` 的额外乘 10。

兼容影响：
- 协议版本 13 改变内部密度倍率语义，但不移动共享寄存器地址和字段长度。
- CPU2/CPU3 必须同为协议版本 13 才能正确解释内部密度字段。
- 旧 FRAM 参数由 CPU2 按旧协议版本标记迁移后写回 `protocolVersion = 13`，CPU3 本机 FRAM 写回 `0x0005`，避免按新倍率误读旧参数。
- 除 LTD 自有协议外，DSM、Wartsila、SI 主站不需要修改密度倍率解析。

验证结果：
- `py tools\check_density_precision_contract.py`：通过。
- `py tools\check_synthetic_density_mode_contract.py`：通过。
- `py tools\check_si_modbus_frames.py`：通过。
- `py tools\check_si_protocol_contract.py`：通过。

### 协议版本 14

关联改动：
- 新增共享命令 `CMD_SI_PROFILE = 20`，CPU3 的 SI `00004 Profile` 写 ON 后锁存 profile 开始时间，并通过 CPU2 命令保持寄存器下发该命令。
- CPU2 新增 SI profile 执行参数：`si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time`、`si_profile_bottom_detect_interval`。
- CPU2 复用原 `reserved30~reserved33` 预留参数槽，`DEVICE_PARAM_VERSION` 保持 `3`，但 `protocolVersion < 14` 或运行期归一化时会补默认值：首点 `1000`、步距 `10000`、停留 `10`、探底频次 `1`。
- CPU2 新增 `CMD_SiProfile()` 独立测量入口，执行时直接读取本机 SI profile 参数，支持探底频次、旧底部位置回退、Point0、最多 200 点、候选点运行中判定液面以上后停止、完成锁存和完成计数。
- 探底失败且无旧底部位置时，CPU2 使用当前位置采集 Point0 并继续本轮 profile，但不刷新 SI 内部底部位置、不置底部位置有效、不清首次探底状态。
- CPU3 参数同步链路新增 SI profile 参数读写，`40001~40003` 不再桥接 `spreadTopLimit`、`spreadMeasurementDistance`、`spreadPointHoverTime`。
- CPU3 本机参数新增 `40010~40023` 对应的 SI 自动 profile 和报警限值参数，CPU3 本机 FRAM 参数版本从 `0x0005` 升级到 `0x0006`，并提供 V3/V4/V5 迁移。
- CPU3 菜单新增 `SI参数` 入口，并拆分 `Profile参数`、`自动Profile`、`报警限值` 三个子页。
- CPU3 SI 层新增自动 profile 调度任务 `si_modbus_periodic_task()`，按 `40010~40013` 和 CPU3 RTC 从起始时间起周期触发，可跨天，同一分钟去重。
- CPU3 SI协议状态口径调整：`10010 Probe Un-calibrated` 固定 `0`，`10002/10003` 按液位跟随稳定状态合成，`10001 Bottom Reference` 保持既有 `bottom_reference_valid` 映射。
- CPU3 SI协议报警口径调整：`40014~40023` 独立保存，CPU3 合成当前值报警、profile 上下限报警和相邻点温度/密度偏差报警。
- CPU2/CPU3 `DensityDistribution` 共享结构新增 `profile_source` 字段，位置在 `profile_complete_counter` 之后、`profile_blocked_by_process` 之前。
- CPU2 在普通分布、国标、每米、间隔、Wärtsilä 和 SI profile 完成后分别写入 `PROFILE_SOURCE_STANDARD/GB/METER/INTERVAL/WARTSILA/SI`。
- CPU3 SI 协议层读取 `10005 Profile Complete`、`30006 Number of Points` 和 `30021~30620` 点阵时，只认 `profile_source == PROFILE_SOURCE_SI`。
- CPU3 SI 协议层读回 `00004 Profile` 运行态时，同时要求 CPU2 当前命令为 `CMD_SI_PROFILE`，普通分布和 Wärtsilä 分布运行中不再误置 SI Profile 模式线圈。
- SI 温度无效值对外统一为 `-200.00°C`，即寄存器补码 `0xB1E0`；密度无效值保持 `0`。
- SI 报警阈值 `0` 改为有效阈值，不再作为禁用条件；无效温度和无效密度仍不参与报警判断。
- `40016/40017` 按有符号 `0.01°C` 保存和比较，Modbus 写入按 int16 补码解释，CPU3 菜单允许负温度限值。
- `10025 Profile Temp Deviation Alarm` 的相邻点温差按 int16 温度值计算，避免负温度跨零时按无符号补码差值误报警。
- 自动 profile 调度从近似日期索引改为真实日历分钟数，避免跨月或长 interval 漂移。
- `30007~30010 Profile Timestamp` 改为 CPU3 收到 SI `00004 Profile`、屏幕 SI profile 或自动 profile 触发时锁存的测量开始时间。

寄存器布局影响：
- 共享保持寄存器数量不减少；原 `reserved30~reserved33` 对应地址变为 SI profile 参数地址，继电器等后续参数基址保持顺延关系不变。
- 共享输入寄存器在密度分布结果区新增 `REG_DENSITY_DIST_PROFILE_SOURCE`，后续 `profile_blocked_by_process`、profile 偏差报警和点阵基址顺延。
- SI 外部 Modbus 地址窗口不变，仍为 `00001~00016`、`10001~10032`、`40001~40023`、`30001~30620`。
- CPU2 `DeviceParameters` 结构体大小和 `DEVICE_PARAM_VERSION` 不变，但字段语义从预留改为 SI profile 执行参数；旧协议存储通过 `protocolVersion < 14` 补默认值并写回当前协议版本。
- CPU3 本机参数结构新增 SI 自动 profile 和报警限值字段；该变化只影响 CPU3 本机 FRAM 参数版本，不改变 CPU2/CPU3 共享输入寄存器点数上限；`40016/40017` 的 CPU3 存储和菜单解释改为有符号温度阈值。

兼容影响：
- CPU2/CPU3 必须同为协议版本 14，才能正确识别 `CMD_SI_PROFILE`、SI profile 参数字段、`profile_source` 共享状态和 `40001~40003` 的新语义。
- 协议版本 13 的 CPU3 会把 `00004 Profile` 桥接到普通分布测量，并把 `40001~40003` 写入普通分布参数，不能与协议版本 14 的 CPU2 混用。
- 协议版本 13 的 CPU2 不识别 `CMD_SI_PROFILE`，也不会按 SI Point0/探底频次执行 profile；协议版本不匹配应由 CPU3 严格相等检查拦截。
- CPU2 参数存储不会因本次升级恢复出厂参数；但原预留槽会被赋予 SI profile 参数语义，现场升级后需要复核 SI 首点、步距、停留和探底频次。
- CPU3 本机参数升级到 `0x0006` 后会新增自动 profile 和报警限值默认值；旧 V3/V4/V5 本机参数迁移后应复核 SI 自动调度和报警阈值。
- 协议版本 13 的 CPU3 会把普通分布或 Wärtsilä 分布结果误认为 SI profile 结果；协议版本 14 修正为只认 SI profile 来源。
- 协议版本 13 的 CPU3 菜单和 Modbus 侧不能正确保存负温度限值，并且报警阈值 0 会被视为未启用；协议版本 14 修正为 signed 温度和 0 有效阈值。

验证结果：
- `py tools\check_si_modbus_frames.py --dump`：通过。
- `py tools\check_si_protocol_contract.py`：通过。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，未发现缺字。
- `cmake --build build\LTD_MAIN_CPU2`：通过，Ninja 无需重建。
- `cmake --build build\LTD_DISPLAY_CPU3`：通过，Ninja 无需重建。

### 协议版本 15

关联改动：
- CPU2/CPU3共享故障码按责任域重新分类，类别代码固定为：11电机驱动、12编码器、13传感器与密度、14通信链路、15测量流程、16模拟量输出、17参数存储、18扭力检测、19系统与软件。
- CPU2共享 `ErrorCode` 由73项调整为77项，其中包含2个非故障状态和75个共享故障码。
- 新增 `MOTOR_TMC_CONFIG_LOST = 0x000B000B`，用于TMC关键配置为空或未保持，屏幕显示 `11-11`。
- 新增 `COMM_UART_TRANSFER_ERROR = 0x000E0004`，用于UART或DMA启动、发送、接收硬件错误，屏幕显示 `14-4`。
- 新增 `WIRELESS_RESP_FORMAT_ERROR = 0x000E000B`，用于无线模块响应字段缺失或解析失败，屏幕显示 `14-11`。
- 新增 `AD5421_FAULT_STATUS_ERROR = 0x00100005`，用于AD5421故障寄存器报告非零状态，屏幕显示 `16-5`。
- CPU3本机 `CPU2_COMM_TIMEOUT` 调整为 `0x000E000C`，屏幕显示 `14-12`；该码仍只由CPU3产生，不加入CPU2共享枚举。
- 测量流程码消除历史跳号，连续使用 `15-1` 至 `15-15`。
- AD5421相关故障由19类调整到16类；传感器物理量与通信链路分开；参数地址和内部调用条件调整到19类。
- CPU2错误日志名称、原因、责任域和类别兜底已同步新版编号；CPU3枚举、类别显示和具体原因已同步新版编号。

寄存器布局影响：
- 不新增、不删除、不移动故障状态寄存器，错误码仍按32位值传递。
- 本次改变故障码数值和类别解释，因此属于共享协议语义变更。
- `DeviceParameters` 结构大小、CPU2 `DEVICE_PARAM_VERSION` 和CPU3本机参数存储布局不因本次故障码调整而变化。

兼容性影响：
- 协议版本15与14及更早版本的故障码数值不兼容，CPU2和CPU3必须成对升级。
- 协议版本14的CPU3会按旧值显示错误类别和原因，不能与协议版本15的CPU2混用；协议版本15的CPU3也不能按新版表解释协议版本14的CPU2错误值。
- 上位机、日志分析工具或售后资料如果按数值解析故障码，必须同步新版编号表。
- 旧日志继续按产生时的CPU2程序版本和旧编号表解释，不对历史记录做数值转换；CPU3本机故障按CPU3程序版本单独确认。
- 首个使用协议版本15的CPU2固件为 `V1.23.0.0`，CPU3固件为 `V1.22.0.0`；协议14及更早版本不得与协议15交叉烧写或混用。

验证结果：
- `py tools\check_fault_code_catalog_contract.py`：通过，共享码77个、CPU3本机码1个、正式表使用中61项、保留15项。
- `py tools\check_sensor_fault_contract.py`、`check_tmc5130_diagnostic_contract.py`、`check_ad5421_diagnostic_contract.py`、`check_parameter_measurement_fault_contract.py`：通过。
- `py tools\check_cpu3_fault_reason_visibility.py`、`check_cpu3_cpu2_comm_timeout_fault_contract.py`：通过。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，缺字0个。
- `py tools\check_docs.py`、`py tools\check_markdown_links.py`、`git diff --check`：通过。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`：协议 15 代码范围构建通过；升版前产物为 `LTD_MAIN_CPU2_V1.22.0.0.hex`，版本修正后应为 `LTD_MAIN_CPU2_V1.23.0.0.hex`，本记录不把升版前文件名写成最终产物证据。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`：协议 15 代码范围构建通过；升版前产物为 `LTD_DISPLAY_CPU3_V1.21.0.0.hex`，版本修正后应为 `LTD_DISPLAY_CPU3_V1.22.0.0.hex`，本记录不把升版前文件名写成最终产物证据。
- 实物故障注入和协议14/15交叉烧写拦截仍需台架验证。

### 协议版本 16

关联改动：
- 共享故障码13类和14类互换：13类改为通信链路，14类改为传感器与密度。
- 通信链路完整码由 `0x000E0001`～`0x000E000B` 调整为 `0x000D0001`～`0x000D000B`；各枚举的低16位子码、名称、原因和处理语义不变。
- 传感器与密度完整码由 `0x000D0001`～`0x000D0006` 调整为 `0x000E0001`～`0x000E0006`；各枚举的低16位子码、名称、原因和处理语义不变。
- CPU3本机 `CPU2_COMM_TIMEOUT` 由 `0x000E000C`（屏幕 `14-12`）调整为 `0x000D000C`（屏幕 `13-12`），产生条件和恢复口径不变。
- CPU2错误日志的类别模块和兜底原因映射随高16位类别号同步；CPU3类别显示同步为 `0x000D` 通信故障、`0x000E` 传感器故障。

寄存器布局影响：
- 不新增、不删除、不移动故障状态寄存器，错误码仍按32位值传递。
- 不改变类别内部子码、故障语义、触发条件和恢复策略。
- `DeviceParameters` 结构大小、CPU2 `DEVICE_PARAM_VERSION` 和CPU3本机参数存储布局不变。

兼容性影响：
- 协议版本16与15及更早版本的故障码数值解释不兼容，CPU2和CPU3必须成对升级。
- 上位机、日志分析工具和资料如果按数值解析故障码，必须同步协议16编号表。
- 旧日志继续按产生时的CPU2程序版本选择对应代码表；旧版 `13-x`、`14-x` 不做原地改写。CPU3本机故障按CPU3程序版本单独确认。
- `LTD故障代码统一表.xlsx` 的“历史故障代码_CPU2V1.24.0.0”工作表归档CPU2 V1.24.0.0开发基线编号。该工作表按CPU2程序版本归档；`DEVICE_PROTOCOL_VERSION` 仍用于CPU2/CPU3通信兼容判断，但不作为正式故障码表的现场选择依据。后续删除、合并或重编号不得回写该历史表，后续CPU2版本可以复用相同编号，旧日志按产生时的CPU2程序版本查询；CPU3本机13-12按CPU3程序版本单独确认。
- 协议16仅作为故障类别互换的开发过渡基线保留，未形成独立发布固件；正式发布直接使用协议17。

验证结果：
- `py tools\check_fault_code_catalog_contract.py`：通过，共享码77个、CPU3本机码1个，正式表使用中61项、保留15项。
- `py tools\check_cpu3_cpu2_comm_timeout_fault_contract.py`、`check_cpu3_fault_reason_visibility.py`：通过，CPU3本机故障为 `13-12 / 0x000D000C`，13类和14类显示原因完整。
- `py tools\check_sensor_fault_contract.py`、`check_parameter_measurement_fault_contract.py`、`check_tmc5130_diagnostic_contract.py`、`check_ad5421_diagnostic_contract.py`：通过。
- `py tools\check_sensor_safe_protocol_contract.py`、`check_ltd_modbus_contract.py`、`check_dsm_compat_contract.py`、`check_si_protocol_contract.py`：通过，共享协议版本和相关外部映射一致。
- `py LTD_DISPLAY_CPU3\font_check.py`：通过，缺字0个。
- CPU2、CPU3 clean-first构建曾在协议16开发基线通过，分别生成 `LTD_MAIN_CPU2_V1.24.0.0.hex`、`LTD_DISPLAY_CPU3_V1.22.0.0.hex`；该结果只作为过渡阶段证据，不作为协议17发布产物。
- `py tools\check_docs.py`、`py tools\check_markdown_links.py`、`git diff --check`：通过。
- 实物故障注入和协议15/16交叉烧写拦截仍需台架验证。

### 协议版本 17

关联改动：
- 二代计量仪故障码按一代类别延续关系和二代新增责任域重新归档；当前CPU2共享故障为63项，CPU3同步共享定义并额外保留1项本机通信故障。
- 正式表共64个故障项目，其中61项标记“使用中”，仅保留 `12-6 编码器上电值变化`、`15-16 实高偏差过大`、`21-5 扭力传感器饱和` 3项未启用预留码。
- 从当前枚举移出12项低价值或被具体故障替代的项目：电机设置失败、电机复位失败、滑环校验错误、滑环丢包、传感器温度异常、传感器电压异常、密度不稳定、密度范围无效、AD5421故障管脚报警、未知错误、地址读取错误和电源波动。
- 这12项仅保留在 `LTD故障代码统一表.xlsx` 的“历史故障代码_CPU2V1.24.0.0”工作表中，不再进入当前CPU2/CPU3枚举、日志映射和CPU3显示映射。
- CPU3本机 `CPU2_COMM_TIMEOUT` 由 `0x000D000C`（屏幕 `13-12`）调整为 `0x00140007`（屏幕 `20-7`）；产生条件、连续失败门槛和恢复口径不变。

寄存器与参数存储影响：
- 不新增、不删除、不移动故障状态寄存器，错误码仍按32位值传递，高16位为类别、低16位为类别内编号。
- CPU2 `DeviceParameters` 结构大小、`DEVICE_PARAM_VERSION = 3`、元信息和CRC覆盖范围均不变；本次升级不会因为参数结构变化而恢复出厂参数。
- CPU3本机参数存储布局和版本不变。

兼容性影响：
- 协议版本17与16及更早版本的故障码数值解释不兼容，CPU2和CPU3必须使用相同 `DEVICE_PROTOCOL_VERSION`；外部主站、日志分析工具和资料应同步当前编号表。
- 现场选择共享故障码表时仍按CPU2程序版本，不按CPU2/CPU3共享协议版本选表；共享协议版本只用于CPU2与CPU3严格配套判断。
- CPU3本机故障不属于CPU2共享故障码表，应按CPU3程序版本单独确认。
- 首个正式使用协议17的固件为CPU2 `V1.25.0.0`、CPU3 `V1.23.0.0`。本次参数存储结构兼容且不会清参，因此按功能级变更提升次版本，不提升软件主版本。

验证状态：
- `check_fault_code_catalog_contract.py`、`check_cpu3_cpu2_comm_timeout_fault_contract.py`、`check_cpu3_fault_reason_visibility.py`、`check_sensor_fault_contract.py`、`check_parameter_measurement_fault_contract.py`、`check_tmc5130_diagnostic_contract.py`、`check_ad5421_diagnostic_contract.py` 全部通过。
- `check_sensor_safe_protocol_contract.py`、`check_ltd_modbus_contract.py`、`check_dsm_compat_contract.py`、`check_si_protocol_contract.py` 全部通过，故障码调整未改变相关外部协议地址和数据格式。
- `py LTD_DISPLAY_CPU3\font_check.py` 通过，显示缺字为0。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`、`cmake --build build\LTD_DISPLAY_CPU3 --clean-first` 通过；发布产物分别为CPU2 `V1.25.0.0`、CPU3 `V1.23.0.0`。
- `py tools\check_docs.py`、`py tools\check_markdown_links.py`、`git diff --check` 通过。
- 实物故障注入、协议16/17交叉烧写拦截和现场恢复路径仍需台架验证。

### 协议版本 18

关联改动：

- 新增独立 `SiProfileRuntime` 运行态，字段为 `phase`、`cycle_counter` 和 `progress_points`。阶段值固定为 `0=IDLE`、`1=PREPARING`、`2=MEASURING`、`3=RETURNING_LEVEL`、`4=COMPLETE`、`5=ABORTED`、`6=FAILED`。
- 三个字段追加在 AO 运行态之后、`REG_ENG` 之前；现有 Profile 点阵、无线、继电器、AO 及其它前序寄存器地址均不移动。
- CPU2 在 Point0 有效样本提交后最后递增 `cycle_counter`，并从 `progress_points=1` 开始按有效液体点递增；空气点、失败样本和重试不计数，不预设最终点数。
- CPU2 在 Point0 前保留上一轮已发布结果；Point0 后活动期以及取消/失败出口不发布半成品点阵。回液位成功、液位稳定且电机停止后，CPU2提交候选结果并最后递增原有完成计数。
- CPU3根据新运行态维护本地SI投影：完整分块读取候选点阵，复核周期、完成计数、点数、来源和阶段，再一次开放最终 `Complete/N/点阵/Profile报警` 组合；CPU2候选Complete不再直接穿透SI侧。
- CPU3首次启动或重连时不能把当前RTC冒充Point0时间；无法重建时，当前周期 `30007~30010` 返回0。若CPU2已处于同代SI COMPLETE，CPU3仍允许恢复最终N、点阵和报警，但时间保持0。
- CPU3已发布SI快照独立保存在本地投影中；后续普通分布、国标、每米、间隔或Wärtsilä分布不会清除或改写旧SI的Complete、N、时间、点阵和Profile报警。旧SI COMPLETE/ABORTED/FAILED只有在SI收尾上下文中才覆盖FC01终态线圈，不得遮住当前非SI动作。

SI外部兼容和存储影响：

- `30014=(FC01 byte1<<8)|FC01 byte0`；`30015=(FC02 byte1<<8)|FC02 byte0`；`30016=(FC02 byte3<<8)|FC02 byte2`。`30017~30020`继续固定为0。
- `40004~40009`作为CPU3本地原始 `uint16_t` 兼容槽，默认值依次为 `0、50、1000、0、5、1`；支持FC03连续读取和FC06单寄存器写入。这六个槽不参与测量、报警或控制。
- `40004~40023`统一使用CPU3本机参数事务写入口：只有FRAM写入及写后读回校验成功才向FC06正常回显；失败时恢复整份写前运行态、尽力恢复旧FRAM镜像，并返回设备故障 `0x06`，避免局部参数或运行态/持久化不一致。
- CPU3本机参数存储版本从 `V6 / 0x0006` 提升到 `V7 / 0x0007`。V6迁移使用显式旧结构和CRC校验，完整保留V6的SI自动调度、报警阈值和三路串口参数，只为六个兼容槽补默认值。
- CPU2 `DeviceParameters` 结构、`DEVICE_PARAM_VERSION = 3`、元信息和CRC范围不因这三个运行态字段变化；本次共享运行态扩展不会触发CPU2参数恢复出厂。

SI Profile外部生命周期：

1. `PREPARING`：Profile进入但Point0尚未建立，保留上一轮Complete、最终点数、Point0时间、点阵和Profile报警。
2. Point0有效样本：建立新周期，Complete清0，`30006=1`，锁存Point0时间；Point0位置为本轮实际采用的底部参考。
3. 活动采集：`30006`只按有效液体点单调递增，`30021~30620`全部为0。
4. 回液位：保持Profile=1、Complete=0，最终点数冻结但不提前发布点阵。
5. 最终发布：CPU3完成分块读取和代际复核，且满足AtLevel、Stable、电机停止和Interlock=0后，一次发布Profile=0、Auto=1、Stop=1、Complete=1、AtLevel=1、最终N、点阵和Profile报警。
6. Point0前显式取消保留上一轮结果；Point0后取消或真实失败均清Complete、N和点阵且不恢复旧Complete。显式取消不置Interlock，真实失败沿用既有错误/Interlock链路。

兼容性和发布门禁：

- 协议18与17及更早版本的输入寄存器总长度和SI生命周期契约不同，CPU2/CPU3必须成对使用协议18；严格相等检查会拦截混搭。
- 首个正式使用协议18的固件为CPU2 `V1.26.0.0`、CPU3 `V1.24.0.0`。CPU2参数存储版本保持3且结构大小不变；CPU3本机参数存储由V6升级为V7。协议18与17及更早版本不得交叉烧写或混用。
- 提交前审查确认3项P1、2项P2和2项既有问题，本次按用户决定只记录、不修改。详细失效机制见`SI协议适配/01_计划与需求/SI协议待确认与后续清单.md`第7节；其中V6→V7迁移写失败、CPU2快速重启和CPU3非整日周期重启属于优先修复项。

验证状态：

- 已实现并通过静态契约检查：`py tools\check_si_protocol_contract.py`、`py tools\check_si_modbus_frames.py --dump`、`py tools\check_wireless_rssi_contract.py`。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`通过，生成`LTD_MAIN_CPU2_V1.26.0.0.hex`，`text=297396 data=2336 bss=33864`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`通过，生成`LTD_DISPLAY_CPU3_V1.24.0.0.hex`，`text=167144 data=37396 bss=61764`。
- 尚未完成真实RS485台架、协议17/18交叉烧写拦截、V6真实FRAM镜像迁移及写失败注入、`40004~40023`掉电/故障注入、CPU2快速重启、CPU3跨日重启、200点PLC响应时延、完成边沿高频轮询、取消/失败、通信中断、非SI分布与已发布SI快照隔离以及1点、30点、200点边界验证。

### 协议版本 19

关联改动：

- 保持协议17确定的10个二代责任域和“一代类别加10、同义子码优先不变”的编号原则；责任域、现场现象或处理方法不同的项目使用独立编号，不为对照关系合并多种原因。
- CPU2枚举共131项，其中2项为非故障状态、129项为共享故障；CPU3同步这131项，并保留本机 `CPU2_COMM_TIMEOUT = 20-7`，共132项定义。
- 正式故障项目共130项，其中127项使用中、3项保留。保留项仍为 `12-6 编码器上电变化`、`15-16 实高偏差过大`、`21-5 扭力传感器饱和`，当前程序不会主动产生，编号继续冻结。
- 将电机报警、传感器设备返回异常、位置失败、测量超时、模拟输出写入/诊断/故障回读、参数地址/内容错误和外围配置错误等通用出口，拆成能够由当前调用点直接判断的具体原因。
- 无线链路进一步区分 `20-8 无线扫描无设备`、`20-9 无线名称重复`、`20-10 无线名称参数无效`、`20-11 无线模块未处于主机模式`、`20-12 无线名称未找到`；`20-6`只表示无线响应内容或格式异常。
- 电机超时进一步区分 `11-18 电机整段运行超时`、`11-25 电机停止等待超时`和`11-26 电机等待到位超时`，避免不同检查方法共用一个编号。
- 传感器安全通信结果细分为 `13-31`～`13-56`，分别覆盖身份、协议版本、模式就绪、配置代次、上报状态、地址、会话、序号、重新握手、重复报文、能力、命令、参数、事务、忙状态、参数校验、采样计数和周期上报结果。
- 罐高与水位标定区分未设置标定值、测量结果无效和计算结果越界；参数存储区分结构大小不匹配、存储版本不匹配和写入后校验失败。
- CPU2统一错误日志名称和原因映射、CPU3中文故障显示、LTD/DSM等输出完整32位故障码的接口均同步现行编号；外部协议地址、功能码和字段宽度不变。
- CPU3对 `20-9`、`20-10`、`20-12` 的屏幕文案分别为“无线名称重复”“无线名称无效”“无线名称未找到”，与正式故障名称保持一致；字库新增“名”的14×14点阵，不再用近义词替代缺字。

历史兼容和版本归档：

- CPU2 V1.25.0.0～V1.26.0.0移出当前枚举的10个通用码只保存在 `LTD故障代码统一表.xlsx` 的“历史故障代码_CPU2V1.25.0.0-V1.26.0.0”工作表，不与更早停用项合并统计；后续CPU2版本可以复用相同编号，旧日志仍按产生时的CPU2程序版本解释。
- CPU2 V1.24.0.0阶段已经停用的12项继续只保存在对应历史工作表；一代计量仪故障表保持独立。
- 现场解释共享故障码时按设备代际和CPU2程序版本选择代码表；CPU3本机故障按CPU3程序版本确认。`DEVICE_PROTOCOL_VERSION`只用于CPU2/CPU3严格配套判断，不代替历史故障表选择。

寄存器、参数和外部协议影响：

- 不新增、不删除、不移动故障寄存器，仍以高16位类别、低16位子码组成32位值传递。
- CPU2 `DeviceParameters`结构、大小、CRC范围和 `DEVICE_PARAM_VERSION = 3` 均不变；本次升级不会因参数结构变化恢复出厂参数。
- 协议18建立的SI Profile运行态、CPU3 FRAM V7、SI地址和生命周期保持不变；协议19只改变共享故障码目录和实际错误出口。
- 协议19与18及更早版本的故障码数值解释不兼容，CPU2和CPU3必须成对使用协议19；解释故障码的屏幕、上位机、日志工具和资料必须同步。

发布状态：

- 协议19没有形成独立正式固件组合，其故障码能力由协议21首个正式组合 CPU2 `V1.27.0.0` / CPU3 `V1.25.0.0` 一并发布。
- 现场解释这批共享故障码时按CPU2 `V1.27.0.0`及后续版本选择现行表；CPU3本机 `20-7`仍按CPU3程序版本确认。

验证状态：

- `docs` 正式工作簿通过故障码目录契约：CPU2枚举131项，其中129项共享故障；CPU3本机故障1项；正式表127项使用中、3项保留。正式源与已验证输出的SHA256一致。
- CPU3原因可见性、CPU2通信超时、传感器、参数/测量/位置、TMC5130、AD5421、AO输出使能和无线RSSI专项检查全部通过；参数写后校验失败只在当前无其它故障时置码，不覆盖既有测量、电机或传感器故障。
- 传感器安全协议、LTD共享Modbus、DSM兼容、SI共享协议和SI黄金帧检查全部通过；协议18建立的相邻共享与外部协议行为未被协议19故障码拆分改变。
- `py LTD_DISPLAY_CPU3/font_check.py`通过：为故障原因新增18个14×14汉字点阵，StockMap共472个字符，点阵顺序一致，OLED显示缺字0个。
- `cmake --build build\LTD_MAIN_CPU2 --clean-first`通过，生成`LTD_MAIN_CPU2_V1.26.0.0.hex`，`text=304708 data=2304 bss=33864`。
- `cmake --build build\LTD_DISPLAY_CPU3 --clean-first`通过，生成`LTD_DISPLAY_CPU3_V1.24.0.0.hex`，`text=170544 data=37452 bss=61764`。
- `py tools\check_docs.py`、`py tools\check_markdown_links.py`和`git diff --check`通过。
- 协议18/19交叉烧写拦截、协议19新增拆分项的真实故障注入、屏幕/日志/停机/恢复闭环仍需台架验证。

### 协议版本 20

关联改动：

- AO当前菜单固定为5组、16个实际显示项：13项可操作，其中12项为持久化配置、`AO输出仿真`为非持久化运行期开关；`AO输入值`、`AO输入百分比`和`AO输出电流`为3项只读运行态。`AO输出电流`复用协议10既有`last_sent_mA_x100`，不新增共享地址或字段；它表示最近成功下发命令，不是DAC回读或物理环路实测值。`AO SIL/WHG`和`DAC回读电流`仅保留内部设计位置，当前不显示、不启用，不计入16项菜单。
- 12项持久化AO配置不受当前工作模式、电流模式、故障模式或仿真开关状态限制，可在禁用状态先行配置并写后读回；模式条件只决定配置何时参与实际输出。非持久化`AO输出仿真`仍仅在输出已启用时可操作；切换到禁用模式时清除仿真，禁用状态写开启返回`0x03 Illegal Data Value`。
- AO工作模式固定为`0=禁用`、`1=4-20mA输出`、`2=HART从站+输出`；电流模式固定为`0=NE`、`1=US`、`2=普通`、`3=固定电流`；输出源固定为`0=储罐液位`、`1=空高`、`2=水位`，单位均为mm。
- 原连续13个32位AO参数槽按菜单逻辑顺序原位解释为`work_mode`、`current_mode`、`output_source`、`sil_whg_reserved`、`fixed_current_mA_x100`、`range_0_01mm`、`range_100_01mm`、`damping_x10_s`、`fault_mode`、`fault_current_mA_x100`、`error_level`、`power_on_current_mA_x100`、`simulation_current_mA_x100`。
- AO电流模式采用NMS81边界：普通过程范围4.00~20.50mA、故障方向3.50/22.60mA；NE过程范围3.80~20.50mA、故障方向3.50/22.60mA；US过程范围3.90~20.80mA、故障方向3.50/22.00mA；固定电流范围4.00~22.50mA。
- 0%和100%对应值使用有符号32位补码传输，工程倍率为0.1mm；当前产品有效工程范围为0到所选源上限，两值禁止相等但允许反向。切换输出源时，CPU3菜单确认后只写输出源，CPU2在该事务内装载新源默认量程，CPU3随后补读完整AO配置；单独修改量程时仍要求0%和100%对应值成对校验。
- 正常过程输入先做一阶阻尼再换算电流；故障、固定电流、仿真和上电电流不经过阻尼。固定电流不依赖过程源有效性；过程源失效只对过程输出进入故障处理。
- 故障模式固定为最小值、最大值、最近有效值、实际值和设定值；“实际值”仅在过程源仍有效时继续输出，源无效时回退最大值。错误级别只控制AO是否因整机错误切入故障输出，不创建、抑制或改变整机统一故障事件。
- 禁用模式固定输出3.40mA；上电电流范围3.40~22.60mA、默认4.00mA，由初始化写入并在首次后续刷新时退出；仿真电流范围3.40~23.00mA、默认12.00mA，仿真开关上电固定关闭。
- HART只在`HART从站+输出`模式响应；PV跟随当前AO输出源，电流使用最后成功下发给AD5421的值，百分比按实际百分数返回，例如50%返回`50.00`而不是`0.5`。

保持寄存器影响：

- 原AO连续地址`0x00AC~0x00C5`不移动，13个32位字段依次对应上述`AoOutputConfig`逻辑顺序；后续标定参数和元信息地址不移动。
- 在既有CRC字段`0x0156~0x0157`之后追加`HOLDREGISTER_AO_SIMULATION_ENABLE=0x0158~0x0159`。该字段是运行期仿真开关，不进入`DeviceParameters`、CRC或FRAM，重新上电固定为0。
- CPU2保持寄存器有效窗口扩展为`0x0000~0x0159`；`0x015A~0x01FF`仍为空洞。FC10写入必须经过枚举、范围、量程和成对关系校验，非法配置返回非法数据值并保持原有效参数，不得保存半成品。

输入寄存器影响：

- 协议10建立的9个AO运行态继续位于`0x0A20~0x0A31`，协议18的SI Profile三项运行态继续位于`0x0A32~0x0A37`，两段地址均不移动。
- 在SI Profile运行态之后追加`0x0A38~0x0A43`共6个32位字段：过程值、过程百分比`x100`、过程值有效标志、仿真开启标志、DAC回读电流预留和DAC回读有效标志。
- 当前没有物理环路电流采样，也尚未实现AD5421 RDDAC设定码读取；DAC回读值保持0且有效标志保持0，CPU3不得显示该预留项，也不得把它作为输出精度或断环正常的证据。
- 输入寄存器有效窗口因此为`0x0000~0x0A43`，`REG_ENG/INPUTREGISTER_AMOUNT=0x0A44`。

参数存储迁移：

- `DEVICE_PARAM_VERSION`继续保持3，`sizeof(DeviceParameters)`、`struct_size`、元信息位置和CRC覆盖范围均不改变，本次升级不会因为结构版本不匹配清除其它现场参数。
- CPU2发现旧`protocolVersion<20`时，先按协议19及更早的原始13槽布局读取AO旧值，再生成协议20配置。协议10起旧使能非0迁移为普通4-20mA输出；协议11起合法且不相等的旧液位端点继续保留；合法初始、故障和调试电流分别迁移为上电、故障和仿真电流；不再使用的正常电流端点、高低报警及其电流不继续赋予旧语义。
- 新增工作模式、电流模式、输出源、SIL/WHG预留、固定电流、阻尼、故障模式和错误级别按协议20默认值补齐；迁移在AO归一化之前完成，避免旧槽位被新语义提前解释。

兼容性和发布边界：

- 协议20改变AO保持寄存器语义、扩展保持/输入窗口并增加迁移规则，与协议19及更早版本不兼容；CPU2和CPU3必须严格使用相同`DEVICE_PROTOCOL_VERSION`，混搭应由现有相等门禁拦截。
- 协议19的故障码拆分继续由协议20继承，协议18建立的SI Profile地址和生命周期不变；上位机既要采用协议19建立的现行故障码，也要采用协议20的AO字段解释。
- 协议20没有形成独立正式固件组合，其AO能力由协议21首个正式组合 CPU2 `V1.27.0.0` / CPU3 `V1.25.0.0` 一并发布；协议18正式组合及更早固件不得按协议20解释AO字段。

验证状态：

- 软件实现已进入协议20开发工作区；最终双端 clean-first 构建、OLED字库和静态差异检查均已通过。CPU2 ELF为text 306124、data 2328、bss 33976；CPU3 ELF为text 174856、data 37940、bss 61940。
- 尚未完成真实电流表精度、禁用3.40mA、NMS81边界、断环恢复、HART叠加、协议19/20混搭拦截和DAC设定码回读台架验证；DAC回读属于后续功能，不作为当前软件验收阻塞项。

### 协议版本 21

关联改动：

- `single_point_measurement`和`single_point_monitoring`各包含温度、密度、温度位置、标准密度、VCF20和计重密度6个32位字段。旧程序由测量流程直接逐字段写共享运行态，CPU3分时轮询期间可能把前一轮和后一轮字段拼成不存在的组合。
- CPU2新增`measurement_complete_counter`和`monitoring_sample_counter`。真实稳定候选形成后，发布器在短临界区写完对应6字段、执行内存屏障，再递增对应代际；命令切换期间拒绝发布，样机数据也必须经过同一入口。
- CPU3新增固定点私有读取缓冲和三阶段握手：先读取两个代际，再一次读取两组结果共24个16位寄存器，最后复读代际。任一请求失败或前后代际变化时丢弃候选并从第一阶段重来，不污染`InputRegisterArray`或公开`g_measurement`。
- 前后代际一致时，CPU3在短临界区一次提交24个结果寄存器、两个代际和两组公开结果，并置固定点快照有效；首次内部通信失败会同时撤销固定点、状态、参数和协议快照，恢复后必须重新建立。
- 在协议20尾部追加`REG_SINGLE_POINT_MEAS_COMPLETE_COUNTER=0x0A44`和`REG_SINGLE_POINT_MON_SAMPLE_COUNTER=0x0A46`，两个字段均为32位、各占两个16位寄存器；`REG_ENG/INPUTREGISTER_AMOUNT`由`0x0A44`扩展为`0x0A48`，有效地址截止`0x0A47`，既有地址均不移动。

兼容性和发布边界：

- 协议21改变输入寄存器总长度和CPU3固定点快照消费方式，与协议20及更早版本不兼容；CPU2和CPU3必须严格使用相同`DEVICE_PROTOCOL_VERSION=21`，混搭由现有相等门禁拦截。
- CPU3协议21在上电和通信恢复后先通过FC03读取`0x0010~0x0011`协议版本。协议18、19、20 CPU2返回旧版本后，CPU3在访问`0x0A38~0x0A47`等新扩展区前进入已确认不匹配状态，状态页显示“协议版本不匹配”，普通命令和CPU2派生字段继续保持Busy门禁；公共区成功与扩展区非法地址不再形成永久“通讯尝试中”循环。
- 已确认协议不匹配表示链路在线，不累计为`CPU2_COMM_TIMEOUT`；后续版本探测连续无合法响应时，仍按既有连续10次规则报告CPU3本机`20-7`。该调整只改变CPU3轮询和显示门禁，不新增共享字段、不改变协议21地址或语义。
- 协议19故障码和协议20 AO语义在协议21中继续有效；协议19、20未形成独立发布固件，首个正式包含全部三阶段能力的组合为CPU2 `V1.27.0.0` / CPU3 `V1.25.0.0`。
- CPU2 `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构大小、`struct_size`、元信息、FRAM A/B地址和CRC范围不变；两个代际属于运行态，不进入持久化结构。协议20的13个AO槽仍按旧协议布局迁移，升级不会因本次变更清除其它现场参数。
- CPU3本机参数存储保持`0x0007`，协议21不改变本机FRAM布局。

验证状态：

- 代码静态检查确认CPU2所有单点测量、固定点监测和样机发布入口均经过同代发布器；CPU3中间响应只写私有缓冲，只有前后代际相同才提交公开缓存。
- `check_cpu3_cpu2_comm_timeout_fault_contract.py`已覆盖协议优先探测、私有协议快照、协议不匹配与快照未知状态分离，以及扩展上电组不得先于协议探测执行的静态契约。
- `git diff --check`、协议/故障码/外部协议契约、CPU3字库检查和双端clean-first构建结果以`2026-07-16_CPU2_V1.27.0.0_CPU3_V1.25.0.0_协议21故障码AO与通信一致性_改动与测试方案.md`记录为准。
- 尚未完成高频固定点样本切换、代际变化重试、32位计数回绕、协议20/21交叉烧写、真实RS485中断/坏帧以及掉线恢复后的固定点首帧台架验证。

### 协议版本 22

关联改动：

- 共享故障项目数量保持129项，CPU3本机`20-7`保持不变；重排电机驱动8项、零点与位置5项、测量过程8项、模拟输出8项和系统与软件3项编号。
- 一代存在同义故障时优先使用“类别加10、子码相同”，例如电机驱动配置丢失由`11-19`调整为`11-3`，电机堵转由`11-20`调整为`11-11`。
- 一代没有直接同义项时，从不占用一代既有含义的最低可用子码安排；二代旧版本不再永久占用当前编号。
- 正式工作簿新增`历史故障代码_CPU2V1.27.0.0`，完整保留CPU2 V1.27.0.0、CPU3 V1.25.0.0、协议21的130个故障项目。历史页内容不反向改写，现场共享故障按CPU2程序版本查询。

兼容性和发布边界：

- 协议22与协议21及更早版本的故障码数值解释不兼容；CPU2和CPU3必须严格使用相同`DEVICE_PROTOCOL_VERSION=22`，混搭由既有相等门禁拦截。
- 故障寄存器地址、宽度、传递方式、错误产生条件、日志名称、CPU3显示文案、恢复逻辑和CPU3本机故障编号均未改变。
- `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构、大小、默认值、元信息和CRC范围不变；从协议21升级不会因本次故障码重排清除现场参数。
- 协议22未单独形成固件组合，其重编号能力随协议23首个正式组合CPU2 V1.28.0.0 / CPU3 V1.26.0.0发布；CPU2 V1.27.0.0仍按协议21历史页解释。

验证状态：

- 已同步CPU2/CPU3共享枚举、正式工作簿、内部治理表、当前程序清单和本地契约工具；129项共享故障两端数值一致且无重复，正式表130项与CPU2 V1.27.0.0协议21历史页均完成复核。
- 参数/测量、传感器、CPU3故障原因显示、CPU2通信超时、TMC5130、AD5421、SI、AO和固定点契约通过；Markdown链接和故障码目标文件`git diff --check`通过。
- CPU2、CPU3 clean-first构建通过：CPU2 ELF为text 306676、data 2328、bss 33992；CPU3 ELF为text 179760、data 37940、bss 80404。
- 当前工作区并行功能新增CPU3本机`20-13`，尚未纳入本轮正式表和目录契约；全量故障码目录检查因此报告未登记本机码。流程导航同步检查另受并行功能移除或改名`CPU2_CommFetchSiProfilePayload`影响。本记录不把这两项写成本轮通过，也不扩大本轮处理范围。
- 尚未完成协议21/22交叉烧写、代表性故障注入、屏幕编号、日志、停机和恢复路径的台架验证。

### 协议版本 23

关联改动：

- `DeviceParameters.ao_output`仍占原13个连续32位槽位，保持寄存器地址、结构大小、元信息和CRC范围均不变。
- `fault_mode`语义收敛为`0=输出配置的故障电流`、`1=保持上次有效过程电流`；保持动作无历史时回退非跟随电流。旧协议22及更早的最小值、最大值、最近有效值、实际值和设定值由CPU2启动迁移，其中最小/最大值固化到配置故障电流，最近有效值迁移为保持动作。
- `error_level`原槽位改为隐藏预留并固定为0；`power_on_current_mA_x100`保留字段名和地址，但协议语义改为非跟随电流，范围3.40~22.60mA、默认4.00mA。
- `AoOutputRuntime.source=0`由一次性上电状态改为持续的非跟随状态。禁用、过程、故障、仿真、固定和驱动错误来源的其余数值保持不变。
- 液位和空高只在`STATE_FLOWOIL`且当前命令为液位寻找/标定/修正时跟随；水位只在`STATE_FOLLOW_WATERING`且当前命令为水位跟随/标定时跟随。存在待切换命令、过程无效、搜索、待机或其他测量时输出非跟随电流。
- 故障动作仅在整机明确为`STATE_ERROR`且错误码不是`NO_ERROR`或`STATE_SWITCH`时执行；警告、取消、切换和过程无效不再触发故障电流。上次有效过程电流只记录正常过程路径成功写入AD5421的值。
- CPU3 AO菜单固定为基本设置、量程设置、故障设置、运行状态和模拟设置5组16项；故障设置仅显示故障动作、故障电流和非跟随电流，运行页标题显示当前输出状态。

兼容性和存储边界：

- CPU2和CPU3必须严格使用相同`DEVICE_PROTOCOL_VERSION=23`；协议22的CPU3会按旧五种故障模式和错误等级解释同一槽位，不能混用。
- `DEVICE_PARAM_VERSION`保持3，协议20~22旧AO配置由CPU2原位迁移并写回，不恢复出厂、不清除其它现场参数。
- 首个正式协议23组合为CPU2 V1.28.0.0 / CPU3 V1.26.0.0；协议22未单独发布，协议21/22/23混搭均由严格版本门禁阻止运行。

验证状态：

- 已完成源码、正式资料和契约同步；静态检查与双端构建结果以`2026-07-16_CPU2_V1.28.0.0_CPU3_V1.26.0.0_协议23故障码AO分布同步与现场兼容_改动与测试方案.md`为准。
- 仍需台架验证液位/空高/水位匹配跟随、非跟随回退、两种故障动作、缓存清除、屏幕状态标题和AD5421实际电流。

### 协议版本 24

关联改动：

- `DeviceParameters.ao_output`仍占原13个连续32位槽位；寄存器地址、结构大小、元信息、CRC范围和`DEVICE_PARAM_VERSION=3`均不变。
- `output_source`数值保持0~2，但语义改为`0=储罐液位`、`1=传感器位置`、`2=水位`。传感器位置直接读取`g_measurement.debug_data.sensor_position`；编码轮模式由`Encoder_IsReady()`判定有效，电机记步模式由`MotorCtrl_IsDriverInitValid()`判定有效，0.0mm允许作为有效位置。
- `power_on_current_mA_x100`保留字段名、地址、范围3.40~22.60mA和默认4.00mA，协议语义由非跟随电流改为初始电流，只在首次有效过程目标成功写入AD5421前或故障保持无历史时使用。
- `AoOutputRuntime.source=0`改为初始状态，新增`source=7`保持状态；1过程、2故障、3仿真、4固定、5禁用、6驱动错误保持原数值。保持状态及采用保持动作的故障状态通过既有过程值、比例和有效标志发布缓存快照，不新增运行态字段。
- 目标优先级固定为禁用、真实设备故障、仿真、固定电流、普通过程输出。液位和水位仅在匹配跟随状态更新；传感器位置在位置源就绪后持续跟随且不受命令门禁限制。
- 正常过程目标只有成功写入AD5421后才更新RAM缓存，缓存内容为实际成功下发电流及同轮阻尼后输入值和比例。命令切换、重新搜索、自动恢复等待新样本或过程样本暂时失效时保持缓存电流；禁用、输出源、电流模式、量程变化清缓存，传感器位置源的罐高或记步方式变化也清缓存。
- 保持期间冻结一阶阻尼时间，恢复有效过程输入后从保持值继续；AD5421写失败不更新缓存。缓存不写入FRAM，掉电后重新从初始状态建立。

兼容性和存储边界：

- CPU2和CPU3必须严格使用相同`DEVICE_PROTOCOL_VERSION=24`。协议23及更早CPU3会把输出源1显示为空高且无法识别保持状态，不能与协议24 CPU2混用。
- 参数存储不迁移、不清除其它现场参数；但旧配置中的`output_source=1`升级后会直接解释为传感器位置，升级前后必须人工复核AO输出源和0%/100%量程。
- 协议24首个正式固件组合为CPU2 V1.29.0.0 / CPU3 V1.27.0.0；双端必须成对升级，协议23及更早组合不得混搭。

验证状态：

- AO协议24契约、AD5421诊断契约、协议事实清单、CPU3参数短名宽度、OLED字库、Markdown链接和`git diff --check`通过。
- 2026-07-17双端clean-first构建通过：CPU2生成`LTD_MAIN_CPU2_V1.29.0.0.hex`，ELF为text 307604、data 2328、bss 34064；CPU3生成`LTD_DISPLAY_CPU3_V1.27.0.0.hex`，ELF为text 182344、data 37932、bss 80508。固定名与当前版本名HEX的SHA-256分别一致。
- 全库文档结构检查仍报告既有目录`docs/05_测试记录/协议台架验收`缺少`README.md`；该目录不属于本次AO范围，本轮未扩展修改。
- 仍需台架验证三源实际电流、命令切换保持、自动恢复、两种故障动作、AD5421写失败、阻尼连续性、0.0mm传感器位置、CPU3四行运行页和真实环路精度。

### 协议版本 25

关联改动：

- 原 `DeviceParameters.reserved3` 32位槽位正式命名为 `water_level_hysteresis_time_s`，保持寄存器地址和后续字段均不移动。
- 参数正式名为“水位滞后时间”，OLED短名为“滞后时间”；单位为s，范围0~3600，默认0。
- CPU2负责默认值、范围归一化、持久化和完整参数快照发布；CPU3负责参数同步、菜单编辑和写回。
- 当前参数不接入水位寻找、稳定判断或跟随算法，仅作为后续功能的协议和菜单预留。

兼容性和存储边界：

- CPU2和CPU3必须严格使用相同 `DEVICE_PROTOCOL_VERSION=25`；协议24及更早程序不会识别该槽位的新语义，不得与协议25混用。
- 协议24及更早存储升级到协议25时，CPU2将该槽位强制归零，避免把历史保留值解释为有效时间。
- `DeviceParameters`结构总尺寸、字段偏移、保持寄存器总布局、CRC范围和 `DEVICE_PARAM_VERSION=3` 均保持不变。
- 协议25首个正式组合为CPU2 V1.30.0.0 / CPU3 V1.29.0.0；双端必须成对升级，协议24及更早组合不得混搭。

验证状态：

- 已完成双端源码、屏幕菜单Word文档和本协议记录同步。
- 静态契约、菜单宽度、字库和双端构建结果以本轮实际验证记录为准。
- 仍需在后续实现运行逻辑时补充水位算法语义和台架验证；本轮不应把参数非零值解释为已生效。

### 协议版本 26

关联改动：

- `DeviceParameters.ao_output`第4个32位槽原位由`sil_whg_reserved`改为有符号`current_correction_mA_x100`；对应保持寄存器地址、后续字段地址和13槽总尺寸均不移动。
- 修正值单位为0.01mA，默认0，范围-100~100（-1.00~+1.00mA）；CPU2/CPU3按`int32_t`补码同步，CPU3在“AO输出 -> 量程设置 -> 电流修正”中编辑。
- 过程基础电流仍按`4.00mA + 百分比 × 16.00mA`计算并执行既有电流模式限幅；初始、固定、仿真和故障配置值也先作为基础目标。
- AO启用时，过程、初始、固定、仿真、故障电流、故障保持和普通保持统一执行`修正后目标 = 基础目标 + 电流修正值`，随后限制到AD5421物理范围3.20~24.00mA；AO禁用时3.40mA明确旁路修正。
- 上次有效缓存保存成功过程目标的基础电流、输入值和比例；保持时从缓存基础电流重新形成一次修正后目标，不缓存已经修正的目标，避免每轮保持重复叠加修正值。

兼容性和存储边界：

- CPU2和CPU3必须严格使用相同`DEVICE_PROTOCOL_VERSION=26`；协议25及更早程序仍把该AO槽位视为隐藏预留，不能与协议26混用。
- 协议25及更早存储升级到协议26时，CPU2把该槽位归零，避免历史预留值被解释为有效修正；其余AO现场参数保持不变。
- `DEVICE_PARAM_VERSION`继续保持3，`sizeof(DeviceParameters)`、`struct_size`、字段偏移、FRAM A/B区和CRC范围均不改变，不因本次升级恢复出厂。
- 协议26未单独形成固件组合；其AO电流修正能力随协议27首个正式组合CPU2 V1.31.0.0 / CPU3 V1.30.0.0交付。

验证状态：

- AO协议静态契约、双端寄存器布局、CPU3菜单短名/字库检查和CPU2/CPU3 clean-first构建结果见协议27版本方案；真实电流环仍需台架验证。

### 协议版本 27

主题：用原有地址宏、枚举和寄存器数组形式建立长期固定的标准Modbus地址表。

#### 变更原因

- 协议26及更早版本使用连续紧凑地址，向中间增加字段会推动后续地址，不利于长期现场维护和多端同步。
- 现场需要直接按标准Modbus PDU地址、3x/4x参考号、类型、字序、缩放、偏移和单位查表，不应再维护另一套编号。

#### 契约变化

- `DEVICE_PROTOCOL_VERSION`从26提升到27；CPU2/CPU3继续严格等值匹配，协议27不得与旧地址程序混用。
- 仍只使用标准Modbus `FC03`、`FC04`、`FC10`，不增加私有功能码。
- 删除内部旧紧凑地址和地址换算；`stateformodbus.h`中的PDU地址直接作为`HoldingRegisterArray[]`或`InputRegisterArray[]`索引。
- CPU2共享Holding范围为`0x0000~0x0FFF`，共享Input范围为`0x0000~0x29FF`；CPU3本机Holding范围为`0x7000~0x706B`。
- 共享协议兼容号位于Holding `0x000E~0x000F`，命令入口位于Holding `0x0F00~0x0F01`，能力位位于Holding `0x0F10~0x0F11`。
- 共享标量通常占两个寄存器并采用高字在前；继电器既有16位运行标志继续保持单寄存器，不为追求形式统一改变原字段宽度。
- 所有CPU2写参数和命令必须等待CPU2合法ACK；读操作只发布已确认快照。
- 200点密度表固定为Input `0x2000~0x295F`，每点12个寄存器；`REG_ENG/INPUTREGISTER_AMOUNT=0x2A00`。

#### 维护、继电器与能力状态

- Input `0x0010~0x0011`发布维护模式状态，Bool、非持久化、上电默认关闭；命令109开启、命令118退出，均不占用设备状态机，维护中仍可执行指令和测量。
- Input `0x0012~0x0013`发布继电器实际屏蔽状态；维护模式或人工运动抑制任一有效即为1。
- Input `0x0014~0x001B`依次发布K1～K4最终逻辑报警动作，统一为`0=未报警动作、1=报警动作`；它是屏蔽后的软件逻辑动作，不是HH/H/L/LL条件，也不是NC/NO触点反馈。
- Input `0x0034~0x003B`预留K1～K4未来物理触点反馈；当前标记为`reserved/not_supported`并固定返回0，不能用于宣称物理动作成功。
- 取消测量继续使用`CMD_CANCEL_MEASUREMENT=16`，不新增独立快速停止命令；SI Stop同步映射到命令16。
- `CMD_MAINTENANCE_EXIT=118`和`CMD_CLEAR_ALL_RELAY_LATCHED_ALARMS=119`已实现；清锁存为一次性运行请求，不写FRAM，报警条件仍成立时保持或重新进入报警。
- 维护/人工屏蔽期间继电器阈值、滞回和锁存仍继续计算，只把最终逻辑动作压为0，再按NC/NO配置驱动物理输出。
- 维护模式下AO只跳过设备故障电流分支；固定、仿真、过程、保持最近值、初始值和单一电流修正链仍执行，AD5421自身诊断不屏蔽。
- 能力位0～4依次为`MAINTENANCE_MODE`、`RELAY_ALARM_ACTION_STATUS`、`RELAY_ALARM_INHIBIT`、`AO_FAULT_CURRENT_INHIBIT`、`CLEAR_ALL_RELAY_LATCHED_ALARMS`；当前支持掩码为`0x0000001F`。
- 上述状态全部进入CPU2同一输入寄存器快照；任何实际共享命令仍必须等待CPU2合法ACK后才能向外返回成功。
- 现有K1 HH、K2 H、K3 L、K4 LL等阈值参数、单一AO电流修正值和既有HH/H/L/LL报警条件状态不重复建立。
- `parameter_update_flag`只在CPU2确认FRAM A/B原值已与请求一致，或完成双分区写入并回读一致后递增；写后校验失败不得以该标志通知CPU3参数持久化完成，同值写入也能得到一次完成通知。

#### 参数存储兼容性

- `DEVICE_PARAM_VERSION`保持3；`DeviceParameters`大小、字段偏移、magic、CRC范围、A/B地址和单槽`0x0800`容量均不因地址重排改变。
- CPU2仍按原结构格式3读取和保存FRAM，没有新增第二种参数容器；共享Modbus地址改变不会单独触发清参。
- 协议25及更早镜像升级时，AO原隐藏保留槽仍按协议26既有迁移规则归零后解释为单一电流修正值；其它合法现场参数继续保留。
- 只有原有magic、结构大小、参数版本、CRC或A/B槽有效性校验失败时，才按既有逻辑回退另一槽或恢复出厂默认值。

#### 版本与验证边界

- 协议27首个正式固件组合为CPU2 `V1.31.0.0`、CPU3 `V1.30.0.0`。
- 静态检查必须覆盖CPU2/CPU3同名地址、标准单帧数量限制、CPU3分组轮询、命令ACK、维护/继电器真实状态、未来触点反馈固定值、能力掩码和参数结构不变。
- CPU2、CPU3需按最终源码完成clean-first构建，并记录ELF的text/data/bss占用，确认扩大后的直接地址数组仍满足RAM余量。
- 真实UART5与外部COM抓包、FRAM掉电和A/B损坏恢复、维护中测量/指令、继电器NO/NC物理输出及AO真实环路仍需台架验证。
- 仍需台架验证正负修正、过程0%/50%/100%、初始、固定、仿真、故障电流、普通保持、故障保持、禁用3.40mA和物理上下限饱和；静态检查与构建不能证明真实环路精度。

### 协议版本 28

主题：固定带参命令的参数生命周期，并消除参数帧与命令帧之间的本地Busy误判。

#### 变更原因

- 七个命令前置参数仍是持久参数，但旧实现直接从全局`g_deviceParams`取值；上位机、屏幕或外部协议在命令执行期间再次写入时，当前流程可能读到下一条命令的参数。
- CPU3收到参数FC10 ACK后，CPU2的延迟FRAM保存会稍后递增`parameter_update_flag`。若该变化恰好出现在参数帧与命令帧之间，CPU3会使完整参数快照失效，并把本应合法的命令本地拦截为`0x06`。
- LH若等待FRAM完成代次，会在同步阻塞事务中等待只能由后续主循环推进的状态，形成假失败；DSM旧参数同步逐项写入时也可能在运行态先提交部分字段再失败。

#### CPU2命令参数快照

- 七个字段保持协议27地址不变：`0x0518~0x0525`，依次为标定液位、标定水位、标定罐高、单点测量位置、单点监测位置、电机运行位置和电机运行距离。
- 每个字段成功提交后递增本机RAM写入代次；命令FC10成功时，把七个字段的值和代次一起绑定为`pending`快照。
- 主循环原子完成“取走命令、只消费匹配的pending、提升为active、发布current_command”。执行中的业务只读active，后来写入只影响下一条命令。
- 自动恢复没有新正式命令时复用原active；恢复决定重试后若又到达正式命令，正式命令在同一短临界区优先，旧恢复不覆盖它。
- 液位/水位标定结束后的条件清零只在全局字段代次仍等于active代次时执行，避免清除执行期间刚写入的下一条命令参数。
- 自中断命令白名单、不同命令打断、非白名单重复命令忽略、取消测量和设备状态切换规则均保持协议27业务语义，不因快照实现改变。

#### CPU3写入确认与外部反馈

- 七个命令前置参数不再使用普通持久参数状态白名单，可在有效CPU2运行态快照、协议匹配且通信故障未锁存时写入；CPU2仍执行字段范围和帧合法性最终校验。
- CPU3以CPU2合法FC10 ACK作为本次写入的成功依据，不再追加同步FC03。ACK后用本次线序寄存器更新CPU3对应保持寄存器、菜单解析缓存和`g_deviceParams`局部镜像，同时使完整参数快照失效并请求后台全量刷新；因此不会出现“CPU2已写成功、追加FC03失败、对外却返回失败”的双事务假失败。
- 七字段获得FC10 ACK后，CPU3只给本次写入覆盖的字段设置易失确认位。完整参数刷新期间，仅当待下发命令的实际消费字段全部具有确认位，才可凭有效运行态快照越过完整参数快照门禁；无参数普通命令和需要其它字段的命令不能借用该资格。
- CPU3把普通运行态、完整参数和固定点结果拆成独立读取门禁：设备状态、错误码、液位及其它普通FC04数据只要求状态、当前连接协议兼容和无通信故障；FC03继续要求完整参数快照；固定点代次`0x0030~0x0033`及单点测量/监测结果`0x1100~0x113F`额外要求固定点三阶段握手完成。分布汇总和SI Profile运行态从`0x1140`起，不受固定点握手阻塞。
- 普通FC03/FC04单次超时、CRC、UART或帧异常只累计通信失败，不清除字段确认位；连续第三次失败触发完整会话重同步时统一清除。已发出的参数写未取得合法响应时结果不确定，清除全部确认位并请求参数刷新；普通命令结果不确定时只消费该命令对应的确认位，不使完整参数快照失效；恢复出厂命令例外，仍清除全部确认位并强制完整参数刷新。TX DMA未启动属于确定未发送，不额外清除。标准Modbus异常`0x01/0x02/0x03/0x04/0x06`属于合法确定失败，不累计物理通信失败，也不消费确认位。
- 命令获得CPU2合法ACK后消费对应字段确认位。恢复出厂命令ACK后立即清除全部确认位并使完整参数快照失效，但不立即补读；CPU3继续轮询状态，待CPU2默认参数保存完成使`parameter_update_flag`变化后才启动完整刷新。对外参数读取在刷新完成前持续返回Busy，不再暴露恢复前旧参数。
- 字段到命令的当前消费映射为：标定液位→102/103，标定水位→116，标定罐高→4/110，单点测量位置→5，单点监测位置→6/14，电机运行位置→9，电机距离→104/105/113/114。
- 恢复出厂命令从CPU2接收ACK、主循环取走命令到默认参数保存完成期间，七字段写入统一返回`0x06`；启动FRAM连续读取失败触发的默认恢复也使用同一保护。该窄窗口用于防止“先ACK后被整套默认值覆盖”，不恢复普通测量状态白名单。
- 取消命令16继续只依赖状态快照、协议兼容和无通信故障；普通持久参数仍必须在原21个允许状态写入，参数刷新中继续返回Busy。
- LH命令参数以CPU2合法FC10 ACK为最终成功条件，不等待延迟FRAM代次，也不追加同步FC03；普通LH持久参数仍等待保存完成计数和目标字段回读。
- DSM批量同步在发送首帧前预检完整差异集；存在普通参数差异且状态不允许时整体拒绝。只有命令前置参数存在差异时允许运行态写入，并走与LTD/LH相同的ACK确认入口。
- 屏幕把协议不匹配、参数正在刷新和真实CPU2通信失败分开提示，不能把本地快照门禁统一显示为“连接失败”。

#### 继电器配置校验与旧参数迁移

- 继电器通道处于禁用状态时，阈值写入只校验本次触及字段是否为有限值并落在当前报警源物理范围内，不要求尚未配置完成的四级阈值立即满足`HH >= H >= L >= LL`；因此可以按HH、H、L、LL逐项写入负温度阈值。
- 把通道切换为启用时必须一次性通过枚举、四级阈值顺序、物理范围、滞回、阻尼和清锁存字段完整校验；已启用通道修改任一配置字段后仍必须保持完整合法，失败返回`0x03`并整体回滚本次FC10候选。
- 液位罐高变化只复核已启用且报警源为储罐液位的通道；水位罐高变化只复核已启用且报警源为水位的通道。禁用通道或无关报警源不阻断罐高写入。
- 旧FRAM加载时，NaN、无穷或物理越界阈值只把对应单字段恢复为0，保留其余合法阈值；纯顺序冲突无法可靠判断错误字段，四个阈值全部保留并禁用通道等待人工确认。
- 四路继电器`damping_factor`仍是预留字段，CPU2只接受0，CPU3菜单输入范围固定为`0..0`。本次不增加字段、不改变参数结构大小、字段偏移、CRC覆盖范围、FRAM地址或参数存储版本。

#### 多控制端语义

- 协议28不增加来源ID、端口所有权、事务号或参数租约。参数帧与命令帧之间若由上位机和屏幕交错写同一字段，CPU2按命令到达前“最后一次已确认写入值”绑定快照。
- 因此本次修复消除的是串读、误清和本地Busy假失败，不承诺把某一来源的参数永久绑定到该来源后续命令。需要来源级隔离时必须另行扩展共享协议。

#### 兼容性和存储边界

- 协议28改变带参命令参数的跨CPU生命周期和写入门禁，与协议27及更早程序不兼容；CPU2和CPU3必须严格等值匹配。
- 不新增、不删除、不移动共享寄存器，不改变字段类型、宽度、字序、缩放、命令码、设备状态或错误码。
- `DEVICE_PARAM_VERSION`保持3；`DeviceParameters`大小、字段偏移、`struct_size`、magic、CRC范围、FRAM A/B地址和单槽容量不变，不因协议28清除现场参数。
- CPU3本机参数版本保持`0x0007`，逐字段ACK确认位图和CPU2的pending/active/字段代次均只存在RAM，不进入FRAM。
- 协议28首个正式CPU2/CPU3固件组合为CPU2 `V1.33.0.0` / CPU3 `V1.33.0.0`；两端必须配套升级，协议27及更早程序不得与协议28混搭。

#### 验证边界

- 静态契约应覆盖七字段地址、CPU2原子take、恢复仲裁、逐字段代次清零、恢复出厂写保护、CPU3 ACK后局部镜像、失败分类、字段与命令匹配、ACK后资格消费、LH成功条件和DSM批量预检。
- 继电器主机边界测试应覆盖禁用通道逐项负阈值、启用完整校验、已启用单字段破坏顺序、相关/无关罐高写、NaN/Inf/越界逐字段迁移、纯顺序冲突保值禁用和四路阻尼固定0。
- CPU2、CPU3需按最终源码完成clean-first构建，并执行通信超时、AO、保留命令7及协议卷相关契约。
- 上位机与屏幕交错写、真实UART5压力、自动恢复期间插入新命令、运动安全、FRAM掉电和所有外部协议实机响应仍需台架验证；静态检查和构建不能替代设备证据。

### 协议版本 29

主题：取消无实际产生路径的12-11，交换扭力与模拟输出故障类别，并放宽密度有效上限。

#### 故障码调整

- 删除 `ENCODER_INVALID_DATA = 0x000C000B`。AS5145状态映射在校验、OCF、COF和LIN均正常时返回`NO_ERROR`，CPU2错误日志和CPU3中文显示不再保留12-11分支。
- `SONIC_FREQ_ABNORMAL = 0x000D0002` 的名称统一为“震动管频率异常”；13-21“传感器无谐振”保持独立含义。
- 扭力检测由21类直接改为18类：`18-1`～`18-6`；其中 `18-5 WEIGHT_SENSOR_SATURATION` 继续作为预留项。
- 模拟输出与自检由18类直接改为21类：`21-2`、`21-6`、`21-9`～`21-16`。
- `12-6 ENCODER_POWERON_CHANGE`和`15-16 MEASUREMENT_HEIGHT_DEVIATION`继续保留；CPU3本机`20-7`、`20-13`不变。
- CPU2/CPU3共享故障由129项减为128项；CPU3仍额外保留2项本机故障。

#### 密度边界

- CPU2密度判定改为 `density <= 0.0f || density > 3000.0f` 时产生13-26，因此`3000.0 kg/m³`有效，只有大于该值才超限。
- CPU3密度手输值仍按`kg/m³ x100`保存和显示，原始上限由200000改为300000，即菜单允许`0.00～3000.00 kg/m³`。

#### 兼容性和存储边界

- 协议29不提供协议28旧故障码别名、数值翻译或运行时规避；CPU2与CPU3必须严格使用相同`DEVICE_PROTOCOL_VERSION=29`。
- 不新增、不删除、不移动共享寄存器，不改变故障寄存器的32位宽度或外部协议透传方式。
- `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构大小、字段偏移、CRC范围、FRAM A/B地址不变；CPU3本机参数版本和FRAM布局也不变。
- 协议29首个正式CPU2/CPU3固件组合为CPU2 `V1.34.0.0` / CPU3 `V1.34.0.0`；两端必须配套升级，协议28及更早程序不得与协议29混搭。

#### 验证边界

- 静态检查覆盖双端枚举一致性、无重复编号、故障名称和屏幕原因、密度3000边界、协议文档及工作簿同步。
- CPU2/CPU3需按最终源码完成clean-first构建；12-11取消、18/21新编号显示、13-26边界和外部32位故障码仍需故障注入或台架实测，构建与静态检查不能替代设备证据。

### 协议版本 30

主题：为24 V真实低压、ADC/DMA监控链路和掉电位置保存结果增加独立共享故障码，避免复用电机电荷泵欠压码。

#### 故障码调整

- 新增 `23-1 POWER_SUPPLY_24V_UNDERVOLTAGE = 0x00170001`：ADC链路正常但24 V低于20 V安全阈值。
- 新增 `23-2 POWER_MONITOR_ADC_OVERRUN = 0x00170002`：ADC发生OVR，采样时效无法确认。
- 新增 `23-3 POWER_MONITOR_DMA_STOPPED = 0x00170003`：连续采样DMA意外停止，内存采样可能陈旧。
- 新增 `23-4 POWER_MONITOR_INIT_FAILED = 0x00170004`：启动ADC/DMA或5 ms内取得首个样本失败。
- 新增 `23-5 POWER_MONITOR_RECOVERY_FAILED = 0x00170005`：主循环连续3次局部恢复ADC/DMA失败。
- 新增 `23-6 POWER_LOSS_POSITION_SAVE_FAILED = 0x00170006`：真实低压紧急保存未确认，或BOR/POR上电时发现真实掉电回执与A/B记录不一致、提交标记或CRC不完整。
- 同一事件只按 `23-6 > 23-5 > 23-1 > 23-2／23-3 > 23-4`升级当前对外故障；首次原因和ADC/DMA恢复原因独立保留。

#### 行为和恢复边界

- 24 V低于20 V时立即禁止运动、禁止普通FRAM写并请求编码器紧急保存；恢复到22 V以上并连续稳定100 ms后恢复普通FRAM写和监控布防，但不自动清故障码、不恢复电机和旧测量。
- ADC溢出、DMA停止或启动失败后由主循环最多执行3次局部恢复；恢复成功保留故障码，失败后保持安全禁止，不自动重启CPU2。
- 新的顶层正式测量命令先清历史编码器锁存，再检查电源监控；只有ADC/DMA健康、无OVR、24 V不低于22 V且低压状态已解除时，才清电源锁存并重新开放运动。
- 参数Modbus成功响应仍表示数据已接受到RAM；延后FRAM持久化失败通过故障状态和读回发现，不把ACK解释为已经落盘。
- PC3／ADC1_IN13仍未进入周期采样，本协议版本不包含12 V监控能力。

#### 兼容性和存储边界

- 协议30不新增、不删除、不移动共享寄存器，不改变故障寄存器32位宽度、字序或外部透传方式；变化仅是新增共享故障码数值和解释。
- CPU2和CPU3必须严格使用相同`DEVICE_PROTOCOL_VERSION=30`。协议29及更早CPU3会把23类显示为未知故障，不能与协议30 CPU2混用。
- `DEVICE_PARAM_VERSION`保持3，`DeviceParameters`结构大小、字段偏移、CRC范围和参数FRAM A/B地址不变；编码器独立持久化记录升级到V2并兼容读取V1，不会清除设备参数。
- 协议30首个正式固件组合为CPU2 `V1.35.0.0` / CPU3 `V1.35.0.0`，两端必须严格配套。

#### 验证边界

- 2026-07-28 CPU2和CPU3均按最终源码完成clean-first构建，分别生成`LTD_MAIN_CPU2_V1.35.0.0.hex`和`LTD_DISPLAY_CPU3_V1.35.0.0.hex`。
- 静态检查覆盖双端协议30、6个数值一致、CPU2名称／原因、CPU3显示分支、现有字库和无新增双斜杠注释。
- 真实20 V／22 V阈值、ADC OVR、DMA停止、三次恢复、随机掉电、FRAM阶段失败、CPU2／CPU3交叉版本拦截和OLED显示仍需台架验证；构建与静态检查不能替代设备证据。

## 后续维护要求

- 2026-06-10 CPU2 `V1.12.1.3` / CPU3 `V1.11.1.3` 同步 CPU2 系统参数出厂默认值和 CPU3 状态页显示参数矩阵，复用现有共享测量数据、设备状态和参数寄存器，不新增共享寄存器、命令码或参数语义，因此 `DEVICE_PROTOCOL_VERSION` 保持 `7`。
- 每次修改 CPU2/CPU3 共享寄存器地址、共享结构体容量、字段含义或跨 CPU 命令语义时，都要先判断是否需要提升协议版本。
- 只修改单 CPU 内部实现、不改变共享数据含义时，不提升协议版本。
- 新协议必须在本文追加记录，并说明旧协议默认行为。

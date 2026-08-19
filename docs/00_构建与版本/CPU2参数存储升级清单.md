# CPU2 参数存储升级清单

更新日期：2026-08-17

本文用于发布前和现场升级前判断 CPU2 固件升级是否会恢复出厂参数。CPU2 参数保存在 FRAM A/B 两个分区，加载时会同时校验 `magic`、`struct_size`、`param_version` 和 `crc`。两份分区都连续读取失败时，CPU2 会执行 `RestoreFactoryParamsConfig()`，恢复出厂默认参数并重新写入 FRAM。

## 快速结论

| 升级目标 | 旧参数保留情况 | 现场动作 |
| --- | --- | --- |
| 升级到 CPU2 `V1.12.0.0` 及之后的 `DEVICE_PARAM_VERSION=3` 固件，且原机为 `V1.11.0.1` 或更早 `DEVICE_PARAM_VERSION=2` 固件 | 会恢复出厂参数 | 升级前备份或记录现场参数，升级后按出厂默认值重新核对和下发 |
| 从更早的 `DEVICE_PARAM_VERSION=1` 固件升级到 `DEVICE_PARAM_VERSION=2` 固件 | 会恢复出厂参数 | 升级前备份或记录现场参数，升级后重新核对和下发 |
| 从未带当前 `LTDM` 元信息、结构体大小或 CRC 校验不匹配的旧固件升级 | 会恢复出厂参数 | 按首次写入新参数区处理 |
| CPU2 `V1.12.0.0` 到 `V1.40.0.0` 协议34之间升级 | 通常保留 | 确认 FRAM 未损坏即可；继续保持 `DEVICE_PARAM_VERSION=3` 和结构尺寸。协议25及更早升级时原SIL/WHG槽归零；协议26～32的0.01mA修正原始值乘10迁移为协议33/34的0.001mA原始值；协议34加载编码轮周长旧值0时归一化为`95000`。若降级到不识别当前传感器类型或AO倍率的旧固件，先备份并复核相关参数 |
| CPU2 `V1.1.0.0` 到 `V1.11.0.1` 之间升级 | 通常保留 | 仅关注具体版本的参数语义变化 |

“通常保留”表示 `param_version` 和 `struct_size` 不变时不会因为存储版本清空；如果 FRAM 本身 CRC 错误、A/B 分区都损坏，仍会按异常处理恢复出厂参数。

## 存储版本变化表

| CPU2 固件/提交范围 | `DEVICE_PARAM_VERSION` | 是否清参数 | 说明 |
| --- | ---: | --- | --- |
| 2025-12-16 起引入当前参数元信息的早期固件 | 1 | 旧格式升级到该格式会清 | 引入 `param_version`、`struct_size`、`magic`、`crc` 校验；不满足当前元信息的旧 FRAM 会加载失败 |
| 2026-03-05 系统参数增加 | 2 | 从版本 1 升级会清 | `DEVICE_PARAM_VERSION` 从 1 提升到 2，旧 FRAM 参数版本不匹配 |
| CPU2 `V1.1.0.0` 到 `V1.11.0.1` | 2 | 版本 2 内通常不清 | 多数新增字段复用保留位或运行期补默认值，不提升存储版本 |
| CPU2 `V1.12.0.0` 四路继电器报警输出 | 3 | 从版本 2 升级会清 | `DeviceParameters` 增加四路 `RelayAlarmConfig`，元信息寄存器顺延，`DEVICE_PARAM_VERSION` 从 2 提升到 3 |
| CPU2 `V1.12.0.1` 到当前版本头 `V1.40.0.0` | 3 | 版本 3 内通常不清 | 当前未改变参数存储结构总尺寸；期间新增字段优先复用原槽位或追加运行态，`DEVICE_PARAM_VERSION`、`DeviceParameters`大小、元信息和CRC范围保持不变 |

## 不清参数但需要关注语义的版本

| 版本 | 存储影响 | 注意事项 |
| --- | --- | --- |
| CPU2 `V1.7.0.0` | 不清参数 | 原 `reserved23` 改为“探底修正罐高”，旧存储升级时默认按 0 处理，继续沿用液位罐高 |
| CPU2 `V1.10.0.0` | 不清参数 | 原 `reserved2` 改为故障自动恢复重跑上限，旧存储升级到协议版本 6 时补默认值 `3`；`empty_weight` 按 `int32_t` 有符号解释 |
| CPU2 `V1.12.0.0` | 清参数 | 新增四路继电器报警输出配置，默认全部禁用；升级后必须重新核对现场参数 |
| CPU2 `V1.12.1.1` | 不清参数 | 仅调整参数注释、串口打印和文档中的扭力/探底中文口径，不改变默认值、字段顺序、结构大小或保存格式 |
| CPU2 `V1.12.1.2` | 不清参数 | 同步现场确认后的 11 项恢复出厂默认值，`DEVICE_PARAM_VERSION` 和 `struct_size` 不变；旧 FRAM 参数保留，新默认值只在恢复出厂或 FRAM 无效时生效 |
| CPU2 `V1.12.1.3` | 不清参数 | 继续按《LNG计量仪屏幕菜单.docx》同步 9 项恢复出厂默认值，`DEVICE_PARAM_VERSION` 和 `struct_size` 不变；旧 FRAM 参数保留，新默认值只在恢复出厂、FRAM A/B 均无效或首次初始化时生效 |
| CPU2 `V1.12.2.0` | 不清参数 | 读取部件参数完成态改为持续刷新，水位调试字段更名为电容快照；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.13.0.0` | 不清参数 | 新增点动长距离运动接口和 BJ/BJP 本地串口测试命令；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.13.0.1` | 不清参数 | 串口测试命令实现从 `measure.c` 拆到 `test.c`，构建流程和文档资料整理；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.13.1.0` | 不清参数 | 优化瓦锡兰分布测量点间移动、空气点液位识别、有效点裁剪和失败不写回；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.13.2.0` | 不清参数 | 修复电机绝对目标越界保护和参数错误自动恢复过滤；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.14.0.0` | 不清参数 | 新增密度连续、连续相对频率和连续定频找液位方式；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.14.0.1` | 不清参数 | 优化电机点动与位置模式运动流程；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.15.0.0` | 不清参数 | 读取部件参数增加蓝牙 RSSI 快照，协议版本 9 仅追加运行态输入寄存器；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.16.0.0` | 不清参数 | 新增 AO 运行态和 AO 输出使能；原 `reserved26` 语义改为 `AoOutputEnable`，旧协议存储升级到协议版本 10 时默认关闭 AO 输出，`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.16.1.0` | 不清参数 | 修复继电器报警限值写入相关打印和核对能力；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.16.2.0` | 不清参数 | 固定点、密度和瓦锡兰移动改用 Jog 两段减速定位；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.17.0.0` | 不清参数 | 旧无线主机/从机自身通信探测改为 CH9141K 蓝牙主从机状态查询，RSSI 刷新和 AT 恢复增强；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.18.0.0` | 不清参数 | 修复零点标定脱离零点扭力误报，AO 改为 TIM4 请求/PendSV 延后刷新并补正式回读测试命令；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.18.1.0` | 不清参数 | 修复继电器液位报警数据源有效标志恢复和温度初始值无效判定；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.19.0.0` | 不清参数 | 命令 115 改为保留、修复液位/水位标定固定偏差、读取部件参数增加连接 MAC 快照并统一液位/扭力口径；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.20.0.0` | 不清参数 | 内部密度 raw 从 `kg/m3 x10` 升级到 `kg/m3 x100`，协议版本升至 13；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 密度相关字段按旧协议版本标记运行期迁移，旧 FRAM 参数保留 |
| CPU2 `V1.20.1.0` | 不清参数 | 参数打印、Modbus 写参差异、分布密度悬停单位和初始化日志整理；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.20.2.0` | 不清参数 | 编码器通信诊断、传感器上电 AT 干扰过滤、全局错误归因和中断优先级调整；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.20.3.0` | 不清参数 | AS5145 SSI 错误重试、持续故障一次性上报和有效帧门控调整；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.21.0.0` | 不清参数 | 独立 SI Profile 命令、参数和输入寄存器点阵；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留并通过协议迁移或运行期归一化补 SI Profile 默认值 |
| CPU2 `V1.21.1.0` | 不清参数 | 修复 SI Profile 首点绝对位置、点位输出、空气点判定和无检测运行通信策略；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.21.2.0` | 不清参数 | SI Profile 成功完成后自动排队找液位，恢复液位跟随；`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.21.3.0` | 不清参数 | 优化液位跟随滞后确认和固定频率速度闭环边界；`oilLevelHysteresisTime` 仅用于方法 0/1 跟随变化确认，`DEVICE_PARAM_VERSION` 和 `struct_size` 不变，旧 FRAM 参数保留 |
| CPU2 `V1.21.4.0` | 不清参数 | 修复编码轮模式速度补偿参考长度；`DEVICE_PARAM_VERSION`、`DeviceParameters` 大小、元信息和 CRC 范围不变，旧 FRAM 参数保留 |
| CPU2 `V1.21.5.0` | 不清参数 | 修复 AD5421 断环重接恢复与 AO 运行态错误处理；`DEVICE_PARAM_VERSION`、`DeviceParameters` 大小、元信息和 CRC 范围不变，旧 FRAM 参数保留 |
| CPU2 `V1.21.6.0` | 不清参数 | 修复 DSM `CN` 编号响应解析并增加 `uint32_t` 溢出保护，同时清理未用代码；`DEVICE_PARAM_VERSION`、`DeviceParameters` 大小、元信息和 CRC 范围不变，旧 FRAM 参数保留 |
| CPU2 `V1.22.0.0` | 不清参数 | 新增 USART1 串口严格收帧、命令解析、查询和停止入口；`DEVICE_PARAM_VERSION=3`、`DeviceParameters` 大小、元信息和 CRC 范围不变，旧 FRAM 参数保留 |
| CPU2 `V1.23.0.0` | 不清参数 | 共享故障码升至协议 15 并完善底层诊断；`DEVICE_PARAM_VERSION=3`、`DeviceParameters` 大小、元信息和 CRC 范围不变，旧 FRAM 参数保留 |
| CPU2 `V1.24.0.0` | 不清参数 | 新增 `SAFE_SENSOR=14` 和设备侧安全传感器协议栈，复用既有 `sensorType` 存储位置；结构大小、元信息和 CRC 范围不变，旧 FRAM 参数保留；降级前需恢复旧固件支持的传感器类型 |
| CPU2 `V1.25.0.0` | 不清参数 | 协议17重排二代故障码，仅改变共享故障数值解释；参数结构、默认值、元信息和CRC范围不变 |
| CPU2 `V1.26.0.0` | 不清参数 | 协议18只在输入寄存器追加SI Profile阶段、周期和进度；不进入`DeviceParameters`，CPU2参数存储版本仍为3 |
| 协议19开发过渡 | 不清参数 | 仅拆分共享故障码；没有形成独立正式CPU2版本组合，参数结构和默认值不变 |
| CPU2 `V1.27.0.0` / 协议21 | 不清参数 | 正式发布协议19故障码、协议20 AO和协议21固定点代际。原连续13个AO槽位原位解释为`AoOutputConfig`并在归一化前按旧协议布局迁移合法旧值；固定点代际只属于运行态。结构总尺寸、元信息和CRC范围不变 |
| CPU2 `V1.28.0.0` / 协议23 | 不清参数 | AO故障动作收敛为故障电流/保持上次有效过程电流，原上电电流改为非跟随电流；13个槽位原位迁移，结构总尺寸、元信息和CRC范围不变 |
| CPU2 `V1.29.0.0` / 协议24 | 不清参数 | 输出源1由空高改为传感器位置，原非跟随电流改为初始电流，并增加RAM过程目标缓存；持久化结构不变。旧`output_source=1`会直接按新语义运行，升级前后必须人工复核输出源、0%/100%量程、初始电流和故障动作 |
| CPU2 `V1.30.0.0` / 协议25 | 不清参数 | 原`reserved3`槽位定义为`water_level_hysteresis_time_s`，范围0～3600s、默认0，当前不参与算法；`DEVICE_PARAM_VERSION=3`、结构大小、元信息和CRC范围不变。协议24及更早升级时强制把该槽归零，避免历史保留值被解释为有效秒数 |
| CPU2 `V1.31.0.0`～`V1.32.0.0` / 协议27 | 不清参数 | 原`AO SIL/WHG`预留槽原位改为有符号`current_correction_mA_x100`，范围-100～100、单位0.01mA、默认0；协议25及更早升级时强制归零，避免历史预留值成为有效修正。`DEVICE_PARAM_VERSION=3`、`DeviceParameters`大小、`struct_size`、FRAM A/B地址、元信息和CRC范围均不变。维护模式、继电器运行态、能力掩码和参数完成代次只驻留RAM/共享输入寄存器，不进入持久化结构 |
| CPU2 `V1.33.0.0` / 协议28 | 不清参数 | 七个命令前置参数增加pending/active快照和字段代次，CPU3增加逐字段ACK确认位；这些状态均只驻留RAM。共享字段地址、`DeviceParameters`结构、`DEVICE_PARAM_VERSION=3`、元信息、CRC范围和FRAM A/B布局不变 |
| CPU2 `V1.34.0.0` / 协议29 | 不清参数 | 仅调整共享故障码数值解释和密度有效上限；不新增、删除或移动参数字段，`DeviceParameters`结构、`DEVICE_PARAM_VERSION=3`、元信息、CRC范围和FRAM A/B布局不变 |
| CPU2 `V1.35.0.0` / 协议30 | 不清参数 | 新增23-1～23-6共享故障码；编码器位置独立记录升级为V2并兼容读取V1。`DeviceParameters`结构、`DEVICE_PARAM_VERSION=3`、元信息、CRC范围和参数FRAM A/B布局不变 |
| CPU2 `V1.36.0.0` / 协议31 | 不清参数 | 复用共享调试输入寄存器的未使用扭力参数槽发布Newhall扭力模块温度，不进入`DeviceParameters`；参数结构、版本、CRC范围和参数FRAM A/B布局不变 |
| CPU2 `V1.36.1.0` | 不清参数 | 只修复CH9141K RSSI半ACK清理与透传恢复；共享协议31、参数结构、`DEVICE_PARAM_VERSION=3`和参数FRAM布局均不变 |
| CPU2 `V1.36.2.0` | 不清参数 | 液位标定写入新罐高后复用既有AO归一化函数收敛动态端点；不新增字段，不改变共享协议31、`DeviceParameters`结构、`DEVICE_PARAM_VERSION=3`、CRC范围或参数FRAM布局 |
| CPU2 `V1.36.3.0`～`V1.36.4.0` | 不清参数 | 电机动态等待预算和DSM定长事务修复只改变运行逻辑；参数结构、版本、CRC范围和FRAM布局不变 |
| CPU2 `V1.37.0.0` / 协议32 | 不清参数 | 在既有`sensorType`字段增加`MULTIPARAM_V4=15`，共享调试Input追加四项运行结果；不改变`DeviceParameters`结构、版本、CRC范围或FRAM布局。降级前需恢复旧固件支持的传感器类型 |
| CPU2 `V1.38.0.0` / 协议32 | 不清参数 | DSM `Cv/CV` RAM版本上下文、CK和CM结果均不进入持久参数；结构、版本、CRC范围和FRAM布局不变 |
| CPU2 V1.39.0.0 / 协议33 | 不清参数 | AO第4槽原位改为`current_correction_mA_x1000`，协议26～32旧值乘10迁移；协议25及更早或未知版本归零。13个AO槽、`DeviceParameters`大小、`struct_size`、`DEVICE_PARAM_VERSION=3`、CRC范围和FRAM A/B布局不变 |
| CPU2 `V1.39.1.0`～`V1.39.2.0` / 协议33 | 不清参数 | 传感器识别修正和固定频率液位闭环复用既有参数字段；不新增或移动字段，不改变存储版本、CRC范围或FRAM布局 |
| CPU2 `V1.40.0.0` / 协议34 | 不清参数 | 编码器连续性故障属于共享运行态；`DeviceParameters`结构和版本不变。加载旧参数时编码轮周长0归一化为既有出厂值`95000`，主站写0返回FC10非法数据值，不触发整套参数恢复出厂 |
| CPU2 待发布 / 协议35 | 不清参数 | 同一物理DM4传感器统一为`DM4_SENSOR=14`。加载协议32～34或其它合法旧镜像发现`sensorType=15`时，定向迁移为14并通过既有FRAM A/B保存路径写回；不改变传感器编号或其它现场参数。`DEVICE_PARAM_VERSION=3`、`DeviceParameters`大小、`struct_size`、字段偏移、CRC范围和FRAM A/B布局均不变。降级到协议34及更早程序前必须重新确认类型14/15语义 |

CPU2 `V1.27.0.0`沿用的协议20 AO迁移规则：

1. `protocolVersion<20`时先按协议19及更早的原始13槽布局读取，禁止用新结构直接解释旧FRAM字节。
2. 协议10起旧`AoOutputEnable!=0`迁移为4-20mA输出，否则保持禁用；不会自动启用HART。
3. 协议11起合法、不相等且位于罐高范围内的旧AO起点/终点液位迁移为储罐液位0%/100%对应值。
4. 合法旧初始电流、故障电流和调试电流分别迁移为上电电流、故障电流和仿真电流；超出协议20范围时使用新默认值。
5. 旧正常电流端点、AO高低报警液位及其电流不再保留旧业务语义；电流模式、输出源、固定电流、阻尼、故障模式和错误级别按协议20默认值建立。
6. 迁移后再执行AO归一化并写回协议版本20；不得清除AO之外的现场参数。真实旧FRAM A/B镜像、掉电中断和写回失败仍需在发布前验证。

协议26～35 AO电流修正规则：

1. 协议26～32存储中的`current_correction_mA_x100`按有符号0.01mA解释，升级协议33时乘10迁移为`current_correction_mA_x1000`；协议25及更早或未知未来版本强制归零。
2. 基础AO配置继续使用x100，修正仅在AO启用态的最终输出阶段按`base_x100 * 10 + correction_x1000`叠加一次；保持缓存仍保存基础目标，禁用态固定3.400mA且不修正。
3. 协议33复用原有连续AO槽位，不增加`AoOutputConfig`或`DeviceParameters`尺寸，不移动后续字段、CRC范围和FRAM A/B地址；协议34～35保持该存储语义。

## 后续版本文档填写要求

每次新增版本改动与测试方案时，必须在“协议版本/兼容性”或单独“参数存储兼容性”小节写明：

- CPU2 `DEVICE_PARAM_VERSION`：旧值 -> 新值。
- `sizeof(DeviceParameters)` / `struct_size`：是否变化。
- 是否会触发恢复出厂参数：是/否。
- 如果会清参数：写明从哪些旧版本升级会清、现场升级前是否需要备份参数、升级后需要复核哪些参数。
- 如果不清参数但复用保留字段或改变 signedness/单位/默认值：写明旧存储如何补默认值或归一化。

推荐格式：

```text
参数存储兼容性：
- CPU2 DEVICE_PARAM_VERSION: 2 -> 3。
- DeviceParameters 结构大小：变化，新增 relayAlarm[4]。
- 升级是否清参数：是；从 CPU2 V1.11.0.1 及更早版本升级到 V1.12.0.0 时，旧 FRAM 参数版本/结构不匹配，会恢复出厂默认参数。
- 现场动作：升级前记录现场参数，升级后重新下发并核对继电器报警输出、液位、扭力、分布测量和通信参数。
```

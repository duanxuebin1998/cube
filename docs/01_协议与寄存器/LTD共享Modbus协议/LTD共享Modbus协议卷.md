# LTD 共享 Modbus 协议卷

更新日期：2026-07-12

## 1. 协议定位

LTD 协议是 CPU2 与 CPU3 共用的一套 Modbus RTU 寄存器和命令协议，不再把“CPU3 菜单中的 LTD”与“CPU2/CPU3 板间协议”分成两套协议。

当前共享协议版本为 `DEVICE_PROTOCOL_VERSION = 15`。协议 15 没有改变本协议卷中的寄存器地址、字段宽度和功能码，但重新分类并编号了共享故障码，因此 CPU2、CPU3 和解释故障码的上位机资料必须同步使用协议 15。

CPU3 采用简单的读写分工：

- `FC03`、`FC04`：CPU3 使用最近一次从 CPU2 取得的缓存快照独立响应，避免每次外部读取都占用 UART5。
- `FC10`：CPU3 把完整 32 位字段写入转发给 CPU2；CPU2 返回合法 ACK 后，CPU3 才返回写成功。
- CPU2 快照未就绪、协议版本不匹配、CPU2 通信故障或写入未获得合法 ACK 时，返回 `0x06 Device Busy`。
- 参数写成功后，CPU3 使参数快照失效并主动重读；重读完成前 `FC03/FC04` 均返回 `0x06`。只写命令字段时，ACK 后直接更新命令缓存。

该实现不是逐帧透明透传；对外寄存器、命令和线序与 CPU2 相同，读响应时刻以 CPU3 最近缓存为准。

## 2. RTU 链路格式

| 项目 | 当前约定 |
| --- | --- |
| 物理协议 | Modbus RTU / RS485 |
| 从站地址 | CPU3 使用系统 `SlaveAddress`，合法范围 `1~247`，非法值回退为 `1` |
| 广播地址 `0` | 当前 CPU3 LTD 实现静默忽略，不执行写入、不回包 |
| CRC | CRC16/MODBUS，多项式 `0xA001`，初值 `0xFFFF`，低字节先发送 |
| 寄存器字节序 | 单个 16 位寄存器高字节先发送 |
| 32 位字序 | 起始地址保存高 16 位，下一地址保存低 16 位 |
| 有符号数 | `int32_t` 使用 32 位二进制补码 |
| 浮点数 | IEEE 754 单精度原始 32 位位型，仍按高 16 位寄存器在前 |

所有共享保持寄存器字段均占两个连续 16 位寄存器。`FC10` 只接受偶数起始地址和偶数寄存器数量，防止只改写半个 32 位字段。

## 3. 功能码

| 功能码 | 名称 | 数量限制 | CPU3 LTD 行为 |
| --- | --- | --- | --- |
| `0x03` | 读保持寄存器 | `1~125` | 从 CPU2 参数缓存快照返回 |
| `0x04` | 读输入寄存器 | `1~125` | 从 CPU2 状态/测量缓存快照返回 |
| `0x10` | 写多个保持寄存器 | `2~122` 且为偶数 | 转发 CPU2，等待合法 ACK 后回显起始地址和数量 |

其他功能码返回 `0x01 Illegal Function`。

## 4. 保持寄存器快照（FC03/FC10）

保持寄存器有效窗口为 `0x0000~0x0157`，共 `344` 个 16 位寄存器、`172` 个 32 位字段；`0x0158` 是结束地址，不可访问。CPU3 本机参数段 `0x0200` 不属于 LTD 共享协议。

| 地址范围 | 32 位字段数 | 内容 | 源码边界 |
| --- | ---: | --- | --- |
| `0x0000~0x0015` | 11 | 命令、传感器/软件版本、上电命令、错误策略、协议版本、自动恢复次数、预留 | `COMMAND` 至 `RESERVED3` |
| `0x0016~0x0025` | 8 | 位置源、电机电流、编码轮/尺带参数、记步方式 | `POSITION_SOURCE_AUTO_SWITCH` 至 `MOTOR_COUNT_FIRST_LOOP_CIRC` |
| `0x0026~0x0039` | 10 | 空载/满载扭力、上下限、变化比例和预留 | `EMPTY_WEIGHT` 至 `RESERVED9` |
| `0x003A~0x0045` | 6 | 零点扭力阈值、忽略区、最大偏差、找零后下行和预留 | `ZERO_WEIGHT_THRESHOLD_RATIO` 至 `RESERVED11` |
| `0x0046~0x0057` | 9 | 液位罐高、盲区、阈值、方式、频率、密度、滞后时间 | `TANKHEIGHT` 至 `OILLEVEL_HYSTERESIS_TIME` |
| `0x0058~0x0069` | 9 | 水位罐高、方式、盲区、电容阈值、最大下行、稳定阈值、修正 | `WATER_TANK_HEIGHT` 至 `WATER_LEVEL_CORRECTION` |
| `0x006A~0x007B` | 9 | 罐底模式/阈值、罐高刷新、初始/当前罐高、编码器修正 | `BOTTOM_DETECT_MODE` 至 `WATER_LAG_CAP_THRESHOLD` |
| `0x007C~0x0083` | 4 | 密度/温度修正和预留 | `DENSITYCORRECTION` 至 `RESERVED19` |
| `0x0084~0x009F` | 14 | 分布/区间测量开关、顺序、点数、间距、上下限、悬停和预留 | `REQUIREBOTTOMMEASUREMENT` 至 `RESERVED21` |
| `0x00A0~0x00AB` | 6 | Wärtsilä 密度区间和探底参数 | `WARTSILA_UPPER_DENSITY_LIMIT` 至 `BOTTOM_ENCODER_CORRECTION_TANK_HEIGHT` |
| `0x00AC~0x00AF` | 2 | AO 液位量程起点/终点 | `AO_START_LEVEL` 至 `AO_END_LEVEL` |
| `0x00B0~0x00C5` | 11 | AO 正常电流、报警液位、各状态电流、输出使能和预留 | `AO_NORMAL_CURRENT_START_mA` 至 `RESERVED27` |
| `0x00C6~0x00D7` | 9 | 液位/水位/罐高标定值、单点位置、分布液位、电机距离和预留 | `CALIBRATE_OIL_LEVEL` 至 `RESERVED29` |
| `0x00D8~0x00DF` | 4 | 上次液位修正、气相温度、尺带膨胀系数和标定温度 | `LAST_OIL_CORRECTION_LEVEL` 至 `TAPE_CALIBRATION_TEMPERATURE` |
| `0x00E0~0x00E7` | 4 | SI Profile 首点、间距、悬停、探底间隔 | `SI_PROFILE_FIRST_POINT` 至 `SI_PROFILE_BOTTOM_DETECT_INTERVAL` |
| `0x00E8~0x014F` | 52 | 4 路继电器报警配置，每路 13 个 32 位字段 | `RELAY_ALARM_BASE + ch * 0x1A` |
| `0x0150~0x0157` | 4 | 参数版本、结构大小、magic、CRC | `PARAM_VERSION` 至 `CRC` |

每路继电器配置依次为：工作模式、数字源、触点类型、报警模式、错误值、报警源、HH、H、L、LL、回差、阻尼、清除报警；通道 `ch=0~3`，每路占 `0x1A` 个寄存器。

参数含义、单位、范围和默认值以 [系统参数出厂默认值.md](../系统参数出厂默认值.md) 为配套说明；字段地址的 CPU2/CPU3 双端权威定义分别是：

- `LTD_MAIN_CPU2/Services/Modbus/stateformodbus.h`
- `LTD_DISPLAY_CPU3/Application/system_param/stateformodbus.h`

## 5. 输入寄存器快照（FC04）

输入寄存器有效窗口为 `0x0000~0x0A31`，共 `2610` 个 16 位寄存器；`0x0A32` 是结束地址，不可访问。

| 地址范围 | 内容 | 结构/字段 |
| --- | --- | --- |
| `0x0000~0x000F` | 设备状态 | 工作模式、状态、错误码、当前命令、零点状态、参数更新标志、装卸液、手动报警抑制 |
| `0x0010~0x002F` | 调试状态 | 编码器、位置、尺带、电机步数/距离、频率、温度、幅值、水位电容、扭力、角度、电机速度/状态 |
| `0x0030~0x003F` | 液位测量 | 液位、空气/油中/跟随/当前频率、探头到位、稳定、手动更新抑制 |
| `0x0040~0x0047` | 水位测量 | 水位、零点/油中/当前电容 |
| `0x0048~0x004D` | 实高测量 | 标定液位、当前实高、罐底参考有效 |
| `0x004E~0x0059` | 单点测量 | 温度、密度、位置、标准密度、VCF20、重量密度 |
| `0x005A~0x0065` | 单点监测 | 温度、密度、位置、标准密度、VCF20、重量密度 |
| `0x0066~0x007F` | 分布汇总/状态 | 平均值、点数、液位、完成锁存/计数、来源、工况阻止、温度/密度偏差报警 |
| `0x0080~0x09DF` | 分布点阵 | 最多 200 点，每点 12 个寄存器：温度、密度、位置、标准密度、VCF20、重量密度 |
| `0x09E0~0x09ED` | 无线滑环匹配 | 结果、MAC 有效/三段、错误码、更新计数 |
| `0x09EE~0x0A15` | 继电器运行态 | 4 路，每路 10 个寄存器：报警值和 8 个状态量 |
| `0x0A16~0x0A1F` | 蓝牙连接/RSSI | 连接有效、RSSI 有效、RSSI、连接错误码、更新计数 |
| `0x0A20~0x0A31` | AO 运行态 | 目标/最近电流、来源、驱动故障、故障寄存器、错误码、更新计数、更新时间 |

分布点 `i=0~199` 的起始地址为 `0x0080 + i * 12`；六个 32 位字段依次位于偏移 `0、2、4、6、8、10`。

设备状态区的故障码、无线滑环匹配区的错误码、蓝牙连接区的连接错误码和 AO 运行态的最近错误码，均直接输出 CPU2/CPU3 当前 32 位内部故障码。协议 15 应按 `../../00_构建与版本/故障码/LTD故障代码统一表.xlsx` 解释为十进制“故障类别-故障码”；协议 14 及以前的旧日志、旧上位机映射和旧编号表不能直接套用。寄存器仍保持高 16 位在前、低 16 位在后的两个寄存器格式。

## 6. 命令快照

命令通过保持寄存器 `0x0000~0x0001` 写入 32 位值。

| 值 | 符号 | 含义 |
| ---: | --- | --- |
| 0 | `CMD_NONE` | 无命令 |
| 1 | `CMD_BACK_ZERO` | 回零点 |
| 2 | `CMD_FIND_OIL` | 寻找液位 |
| 3 | `CMD_FIND_WATER` | 寻找水位 |
| 4 | `CMD_FIND_BOTTOM` | 寻找罐底 |
| 5 | `CMD_MEASURE_SINGLE` | 单点测量 |
| 6 | `CMD_MONITOR_SINGLE` | 单点监测 |
| 7 | `CMD_SYNTHETIC` | 综合测量 |
| 8 | `CMD_FOLLOW_WATER` | 水位跟随 |
| 9 | `CMD_RUN_TO_POSITION` | 运行到指定位置 |
| 10 | `CMD_MEASURE_DISTRIBUTED` | 普通分布测量 |
| 11 | `CMD_GB_MEASURE_DISTRIBUTED` | 国标分布测量 |
| 12 | `CMD_MEASURE_DENSITY_METER` | 密度每米测量 |
| 13 | `CMD_MEASURE_DENSITY_RANGE` | 区间密度测量 |
| 14 | `CMD_WARTSILA_DENSITY_RANGE` | Wärtsilä 密度区间测量 |
| 15 | `CMD_READ_PART_PARAMS` | 读取部件参数 |
| 16 | `CMD_CANCEL_MEASUREMENT` | 取消测量并进入待机 |
| 20 | `CMD_SI_PROFILE` | SI Profile |
| 21、22 | `CMD_RESERVED_CMD2/3` | 保留，不作为现场命令使用 |
| 100 | `CMD_DEBUG_MODE` | 调试模式 |
| 101 | `CMD_CALIBRATE_ZERO` | 标定零点 |
| 102 | `CMD_CALIBRATE_OIL` | 标定液位 |
| 103 | `CMD_CORRECT_OIL` | 修正液位 |
| 104 | `CMD_MOVE_UP` | 上行 |
| 105 | `CMD_MOVE_DOWN` | 下行 |
| 106 | `CMD_SET_EMPTY_WEIGHT` | 设置空载扭力 |
| 107 | `CMD_SET_FULL_WEIGHT` | 设置满载扭力 |
| 108 | `CMD_RESTORE_FACTORY` | 恢复出厂设置 |
| 109 | `CMD_MAINTENANCE_MODE` | 维护模式 |
| 110 | `CMD_CALIBRATE_TANKHEIGHT` | 标定罐高 |
| 111、112 | `CMD_RESERVED_CMD5/6` | 保留，不作为现场命令使用 |
| 113 | `CMD_FORCE_MOVE_UP` | 强制上行 |
| 114 | `CMD_FORCE_MOVE_DOWN` | 强制下行 |
| 115 | `CMD_RESERVED_CMD7` | 保留，原命令已删除 |
| 116 | `CMD_CALIBRATE_WATER` | 标定水位 |
| 117 | `CMD_PAIR_NEAREST_WIRELESS_SLIPRING` | 匹配最近无线滑环 |
| 255 | `CMD_UNKNOWN` | 未知命令标记，不作为下发命令 |

## 7. 异常响应

| 异常码 | 含义 | 典型触发条件 |
| --- | --- | --- |
| `0x01` | Illegal Function | 功能码不是 `03/04/10` |
| `0x02` | Illegal Data Address | 起始地址或连续范围超出对应窗口 |
| `0x03` | Illegal Data Value | 数量为 0/超限、FC10 未按 32 位对齐、byte count 或帧长不一致 |
| `0x06` | Slave Device Busy | CPU2 快照不可用、协议不匹配、通信故障或 FC10 未获得 CPU2 合法 ACK |

地址不匹配、CRC 错误和过短且无法形成合法 RTU 请求的帧不回包。

## 8. 参考帧

以下帧从站地址均为 `1`，CRC 已包含在末尾。

| 场景 | 帧 |
| --- | --- |
| FC03 读命令请求 | `01 03 00 00 00 02 C4 0B` |
| FC03 命令值为 0 的响应 | `01 03 04 00 00 00 00 FA 33` |
| FC04 读前两个状态寄存器 | `01 04 00 00 00 02 71 CB` |
| FC04 两个寄存器均为 0 的响应 | `01 04 04 00 00 00 00 FB 84` |
| FC10 写 `CMD_MONITOR_SINGLE=6` | `01 10 00 00 00 02 04 00 00 00 06 73 AD` |
| FC10 写成功响应 | `01 10 00 00 00 02 41 C8` |
| FC03 设备忙 | `01 83 06 C1 32` |
| FC04 非法地址 | `01 84 02 C2 C1` |
| FC10 非法值 | `01 90 03 0C 01` |
| FC10 设备忙 | `01 90 06 CC 02` |
| FC06 非法功能 | `01 86 01 83 A0` |

参考帧和关键代码闭环由 `py tools/check_ltd_modbus_contract.py` 校验。

## 9. 版本与验证边界

- 当前 `DEVICE_PROTOCOL_VERSION = 15`。LTD 寄存器地址、字段宽度、命令和读写行为未变；版本升级来自共享故障码责任域与编号重排。
- 协议 14 与协议 15 的故障码解释不兼容，CPU2/CPU3 必须成对升级，外部主站也应同步新版故障码表。
- CPU3 从“LTD 占位不回包”变为实际协议响应，属于 CPU3 新增对外功能；本功能首个版本为 CPU3 `V1.21.0.0`，配套 CPU2 版本为 `V1.22.0.0`。
- 本协议卷描述源码实现，仍需在实机 RS485 和 PLC 上验证波特率/校验位、长帧、CPU2 掉线、协议不匹配、参数写后补读和命令执行结果。

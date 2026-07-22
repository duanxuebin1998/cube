# CPU3状态页不同状态显示信息确认表

日期：2026-07-22

适用版本：本次正式固件组合为共享协议 `27`、CPU2 `V1.31.0.0` / CPU3 `V1.30.0.0`。读取部件参数页RSSI显示从协议9起支持，密度两位小数显示从协议13起支持，SI Profile来源隔离和点阵输入寄存器从协议14起支持，SI生命周期字段从协议18起支持。协议19拆分多义故障，协议20新增AO配置和运行态，协议21增加固定点结果代际和已确认快照，协议22只重排32项共享故障编号，协议23收敛AO故障动作，协议24定义传感器位置源以及初始/过程/保持输出语义，协议25增加水位滞后时间预留参数，协议26定义单一AO电流修正，协议27发布固定功能块地址、非阻塞维护叠加态、继电器屏蔽后逻辑动作/锁存状态和参数完成代次。协议21及更早固件必须按对应CPU2历史页解释故障编号；协议23及更早版本不能按协议24解释AO输出源1、初始电流和保持状态，协议20及更早版本缺少协议21固定点快照契约。

源码依据：`LTD_DISPLAY_CPU3/Application/display/display.c`、`LTD_DISPLAY_CPU3/Communication/internal/main_board_modbus/cpu2_communicate.c`、`LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h`、`LTD_MAIN_CPU2/Services/Sensor/sensor.c`

故障状态下，屏幕使用十进制“类别代码-故障码”格式，例如 `13-3`。当前有故障码的类别为11电机驱动、12编码器、13传感器与密度、14零点与位置检测、15测量过程、17参数与存储、18模拟输出与自检、20设备通信链路、21扭力检测、22系统与软件；完整说明以 `../00_构建与版本/故障码/LTD故障代码统一表.xlsx` 为准。

## 1. 显示规则总览

CPU3 状态页第一行固定显示设备主状态文字。协议27的维护模式是易失、非阻塞叠加态，不覆盖设备主状态，因此进入/退出维护模式不会把状态页主状态改成`STATE_MAINTENANCEMODE`；该枚举仅保留给遗留状态显示分支。通常在右侧显示电机运行图标；CPU2/CPU3协议兼容且AO输出仿真开启时，右侧改为反显“模拟”/`SIM`，关闭仿真后清除提示并恢复电机图标。除无线滑环匹配特殊页外，后续结果行由 `DisplayResultContext` 决定：

| 显示项 | 显示条件 | 数据源 | 显示口径 |
| --- | --- | --- | --- |
| 液位 | 当前状态允许显示液位，且液位不是 `UNVALID_LEVEL` | 液位状态取 `oil_measurement.oil_level`；分布完成取 `density_distribution.Density_oil_level`；综合完成取 `oil_measurement.oil_level` | `0.1 mm`；值为 `OILLEVELDOWNLIMIT` / `LEVEL_DOWNLIMIT` 时显示“低于盲区” |
| 水位 | 当前状态允许显示水位，且 `water_measurement.water_level != LEVEL_DOWNLIMITWATER` | `water_measurement.water_level` | `0.1 mm`；当前实现水位为 `0` 时隐藏，不显示“低于盲区” |
| 密度 | 当前上下文有密度源，且密度不是 `UNVALID_DENSITY` | 单点测量、单点监测或分布平均密度；LTD 密度分布测量中显示 `density_distribution.average_density` | `0.01 kg/m3`，内部 raw 为 `kg/m3 x100` |
| 温度 | 当前上下文有温度源，且温度 `> 0` 且 `< 40000` | 单点测量、单点监测或分布平均温度；LTD 密度分布测量中显示 `density_distribution.average_temperature`；读取参数完成显示 `debug_data.temperature` | 显示值为 `temperature - 20000`，小数 2 位，单位 `℃` |
| 位置 | 正常状态页路径下固定显示 | `debug_data.sensor_position` | `0.1 mm`；值为 `0` 时也显示 |
| 扭力 | 正常状态页路径下固定显示 | `debug_data.current_weight` | 整数显示，无明确单位；值为 `0` 时也显示 |
| 频率 | 液位过程/液位跟随状态，或读取参数完成，且当前频率有效 | 液位过程取 `oil_measurement.current_frequency` 或 `debug_data.frequency`；读取参数完成取 `debug_data.frequency` | `Hz` |
| 电容 | 水位过程/水位跟随状态，或读取参数完成，且当前电容有效 | 水位过程取 `water_measurement.current_capacitance`；读取参数完成取 `debug_data.water_capacitance_x10` | 显示为 0.1pF 口径，小数 1 位 |
| X/Y角 | 读取参数完成时只要求角度不为 `0`；罐高上下文仍要求 `bottom_detect_mode != 0` 且角度不为 `0` | `debug_data.angle_x` / `debug_data.angle_y` | 原始值为角度 `x100`，小数 2 位，单位 `°` |
| 蓝牙 RSSI | 读取参数完成；`rssi_valid != 0` 时显示数值，否则显示无有效值 | `wireless_pairing_status.rssi_valid` / `wireless_pairing_status.rssi` | 单位 `dB`，无效时显示 `RSSI:N/A` |
| 罐高 | 罐底完成或罐高标定完成，且 `current_real_height != 0` | `height_measurement.current_real_height` | `0.1 mm` |
| 错误码 | `STATE_ERROR` | `device_status.error_code` | 状态行追加错误类型和位置，格式为 `type-pos` |
| 故障详情 | `STATE_ERROR` 且 `error_code != NO_ERROR` | `Display_GetErrorReasonByCode(error_code)` | 结果区显示 `故障:` 原因，过长时拆成两行 |
| AO仿真提示 | CPU2/CPU3协议兼容且 `ao_output_runtime.simulation_enabled != 0` | `g_measurement.ao_output_runtime.simulation_enabled` | 状态行右上角反显“模拟”/`SIM`并临时替代电机图标；关闭仿真、协议不兼容或提示行切换时先清除旧区域，再恢复电机图标，避免残留维护态提示 |
| CPU2通信超时 | `STATE_ERROR` 且 `error_code == CPU2_COMM_TIMEOUT` | `CPU2_CommRecordFailure()`、`CPU2_CommShouldShowStartup()`、`CPU2_CommIsAvailable()`、`CPU2_CommCanSendCommand()` | 连续请求失败计数从0开始；1000 ms超时、非法响应、UART错误和TX DMA启动失败均按1次累计。已建立快照后，第1～2次失败只计数并重试，不清快照、不退回“通讯尝试中...”；连续第3次失败才清空公开快照并完整重同步；第10次置 `STATE_ERROR` 和 `0x00140007`，屏幕显示 `20-7`。冷启动因尚无快照，在重试期间仍显示“通讯尝试中...”。只有完整包含设备状态和错误码的0x04响应才能解除故障锁存。任何已发起的非命令参数写失败仍立即关闭参数快照并强制补读，覆盖CPU2已写入但ACK丢失场景。参数刷新期间普通写关闭，但状态/协议快照有效、协议匹配且无故障时，屏幕 `CMD_CANCEL_MEASUREMENT` 仍可达；SI `00009 Stop`现在也发送`CMD_CANCEL_MEASUREMENT`，不再进入维护模式，但仍按外部协议普通命令门禁处理，不使用屏幕专用绕过 |
| CPU2/CPU3协议不匹配 | 当前连接协议快照有效且CPU2协议不等于CPU3协议 | `CPU2_CommIsProtocolCompatible()`、`CPU2_CommIsProtocolMismatch()` | 上电或恢复先读取`0x0010~0x0011`协议版本；确认不匹配后退出通讯尝试页并显示“协议版本不匹配”，不显示伪`20-7`，不读取新协议扩展区，不开放普通写或CPU2派生字段。协议心跳后续连续10次无合法响应时才转入`20-7`通信超时故障 |

## 2. 状态显示确认表

除无线滑环匹配专用页外，所有状态默认显示“状态、位置、扭力”。下表中的“状态页显示信息”列保留关键业务项，未逐项重复说明固定显示的位置和扭力。

| 状态 | 显示上下文 | 状态页显示信息 | 数据来源 | 备注 |
| --- | --- | --- | --- | --- |
| `STATE_FINDOIL` | 液位过程 | 状态、位置、扭力、频率 | `debug_data.sensor_position`、`debug_data.current_weight`、`oil_measurement.current_frequency` / `debug_data.frequency` | 不显示旧液位、旧水位、旧密度 |
| `STATE_CALIBRATIONOILING` | 液位过程 | 状态、位置、扭力、频率 | `debug_data.sensor_position`、`debug_data.current_weight`、`oil_measurement.current_frequency` / `debug_data.frequency` | 标定液位过程中显示过程量 |
| `STATE_FLOWOIL` | 液位结果/跟随 | 状态、液位、频率 | `oil_measurement.oil_level`、`oil_measurement.current_frequency` / `debug_data.frequency` | 液位无效时隐藏；低于盲区显示“低于盲区” |
| `STATE_FINDOILOVER` | 液位结果 | 状态、液位 | `oil_measurement.oil_level` | 不显示频率；不混入旧密度/水位 |
| `STATE_FINDWATER` | 水位过程 | 状态、位置、电容 | `debug_data.sensor_position`、`water_measurement.current_capacitance` | 不显示旧液位频率 |
| `STATE_FOLLOW_WATER_POINT_SEARCHING` | 水位过程 | 状态、位置、电容 | `debug_data.sensor_position`、`water_measurement.current_capacitance` | 寻找水位跟随点过程量 |
| `STATE_CALIBRATE_WATERING` | 水位过程 | 状态、位置、电容 | `debug_data.sensor_position`、`water_measurement.current_capacitance` | 水位标定过程量 |
| `STATE_FOLLOW_WATERING` | 水位结果/跟随 | 状态、水位、电容 | `water_measurement.water_level`、`water_measurement.current_capacitance` | 水位为 `LEVEL_DOWNLIMITWATER` 时隐藏 |
| `STATE_FINDWATER_OVER` | 水位结果 | 状态、水位 | `water_measurement.water_level` | 不显示旧液位/旧密度 |
| `STATE_CALIBRATE_WATER_OVER` | 水位结果 | 状态、水位 | `water_measurement.water_level` | 水位标定完成结果 |
| `STATE_SINGLEPOINTING` | 单点测量 | 状态、密度、温度、位置、扭力 | `single_point_measurement`、`debug_data.sensor_position`、`debug_data.current_weight` | 密度/温度有效时会显示当前单点测量值 |
| `STATE_RUNTOPOINTING` | 单点测量过程 | 状态、密度、温度、位置、扭力 | `single_point_measurement`、`debug_data.sensor_position`、`debug_data.current_weight` | 运行到测量点过程中不使用单点监测值 |
| `STATE_SINGLEPOINTOVER` | 单点测量完成 | 状态、位置、扭力、密度、温度 | `debug_data.sensor_position`、`debug_data.current_weight`、`single_point_measurement.density`、`single_point_measurement.temperature` | 不使用 `single_point_monitoring` |
| `STATE_SPTESTING` | 单点监测 | 状态、位置、扭力、密度、温度 | `debug_data.sensor_position`、`debug_data.current_weight`、`single_point_monitoring.density`、`single_point_monitoring.temperature` | 单点监测也固定显示当前位置和扭力 |
| `STATE_GB_SPREADPOINTING` | 分布测量过程 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 不显示旧平均密度/温度 |
| `STATE_SPREADPOINTING` | 分布测量过程 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 不显示旧平均密度/温度 |
| `STATE_METER_DENSITY` | 分布测量过程 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 密度每米测量中 |
| `STATE_INTERVAL_DENSITY` | 分布测量过程 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 区间密度测量中 |
| `STATE_WARTSILA_DENSITY_START` | 分布测量过程 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | LTD/Wartsila 密度分布开始 |
| `STATE_WARTSILA_DENSITY_MEASURING` | 分布测量过程 | 状态、位置、扭力、平均密度、平均温度 | `debug_data.sensor_position`、`debug_data.current_weight`、`density_distribution.average_density`、`density_distribution.average_temperature` | 按矩阵显示测量中的密度/温度，不显示分布液位 |
| `STATE_GB_SPREADPOINTOVER` | 分布测量完成 | 状态、液位、平均密度、平均温度 | `density_distribution.Density_oil_level`、`average_density`、`average_temperature` | 当前实现不显示测点数 |
| `STATE_SPREADPOINTOVER` | 分布测量完成 | 状态、液位、平均密度、平均温度 | `density_distribution.Density_oil_level`、`average_density`、`average_temperature` | 当前实现不显示测点数 |
| `STATE_COM_METER_DENSITY_OVER` | 分布测量完成 | 状态、液位、平均密度、平均温度 | `density_distribution.Density_oil_level`、`average_density`、`average_temperature` | 当前实现如分布液位有效也会显示液位 |
| `STATE_INTERVAL_DENSITY_OVER` | 分布测量完成 | 状态、液位、平均密度、平均温度 | `density_distribution.Density_oil_level`、`average_density`、`average_temperature` | 当前实现不显示测点数 |
| `STATE_WARTSILA_DENSITY_OVER` | 分布测量完成 | 状态、液位、平均密度、平均温度 | `density_distribution.Density_oil_level`、`average_density`、`average_temperature` | 当前实现不显示测点数 |
| `STATE_SYNTHETICING` | 综合过程/运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 保守显示过程量，不显示旧业务结果 |
| `STATE_SYNTHETICING_OVER` | 综合完成 | 状态、液位、水位、平均密度、平均温度 | `oil_measurement.oil_level`、`water_measurement.water_level`、`density_distribution.average_density`、`average_temperature` | OLED 分页显示；水位为 `0` 时隐藏 |
| `STATE_READPARAMETEROVER` | 读取参数完成/持续刷新 | 状态、位置、扭力、温度、频率、电容、X角、Y角、RSSI | `debug_data.sensor_position`、`debug_data.current_weight`、`debug_data.temperature`、`debug_data.frequency`、`debug_data.water_capacitance_x10`、`debug_data.angle_x/y`、`wireless_pairing_status.rssi_valid/rssi` | CPU2 `CMD_ReadPartParams()` 在该状态内每 1s 刷新部件参数，RSSI 快照按 5s 节流刷新；读取参数态的 X/Y 角不再受 `bottom_detect_mode` 限制；显示侧展示最新部件参数快照，不混用液位、水位、罐高等历史业务结果 |
| `STATE_FINDBOTTOM` | 罐高过程 | 状态、位置、扭力、X角、Y角 | `debug_data.sensor_position`、`debug_data.current_weight`、`debug_data.angle_x/y` | 角度需 `bottom_detect_mode != 0` |
| `STATE_CALIBRATE_TANKHEIGHTING` | 罐高过程 | 状态、位置、扭力、X角、Y角 | `debug_data.sensor_position`、`debug_data.current_weight`、`debug_data.angle_x/y` | 角度需 `bottom_detect_mode != 0` |
| `STATE_FINDBOTTOM_OVER` | 罐高完成 | 状态、位置、扭力、X角、Y角、罐高 | `debug_data`、`height_measurement.current_real_height` | 罐高不为 `0` 时显示 |
| `STATE_CALIBRATE_TANKHEIGHT_OVER` | 罐高完成 | 状态、位置、扭力、X角、Y角、罐高 | `debug_data`、`height_measurement.current_real_height` | 罐高标定完成结果 |
| `STATE_RUNUPING` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 不显示旧业务结果 |
| `STATE_RUNDOWNING` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 不显示旧业务结果 |
| `STATE_RUNUPOVER` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 当前实现扭力有效时仍显示 |
| `STATE_RUNDOWNOVER` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 当前实现扭力有效时仍显示 |
| `STATE_RUN_TO_POSITIONING` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 运行到指定位置中 |
| `STATE_RUN_TO_POSITION_OVER` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 运行到指定位置完成 |
| `STATE_FORCE_RUNUPING` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 强制上行中 |
| `STATE_FORCE_RUNDOWNING` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 强制下行中 |
| `STATE_FORCE_RUNUP_OVER` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 强制上行完成 |
| `STATE_FORCE_RUNDOWN_OVER` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 强制下行完成 |
| `STATE_FORCEZERO` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 设置电机零点中 |
| `STATE_FORCEZERO_OVER` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 设置电机零点完成 |
| `STATE_MAINTENANCEMODE` | 遗留显示分支 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 协议27命令109/118使用独立`maintenance_mode_active`叠加态，不再把主设备状态切到该枚举；仅在旧路径显式发布该状态时沿用本行显示规则 |
| `STATE_DEBUG_MODE` | 运动调试 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 调试模式不显示旧业务结果 |
| `STATE_GET_FULLWEIGHT` | 扭力 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 满载扭力中 |
| `STATE_GET_EMPTYWEIGHT` | 扭力 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 空载扭力中 |
| `STATE_GET_FULLWEIGHT_OVER` | 扭力 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 满载扭力完成 |
| `STATE_GET_EMPTYWEIGHT_OVER` | 扭力 | 状态、位置、扭力 | `debug_data.sensor_position`、`debug_data.current_weight` | 空载扭力完成 |
| `STATE_ERROR` | 故障 | 状态、错误码、位置、扭力、故障详情 | `device_status.error_code`、`Display_GetErrorReasonByCode()`、`debug_data.sensor_position`、`debug_data.current_weight` | 状态页不自动混入旧业务测量值；`CPU2_COMM_TIMEOUT` 属于 CPU3 本机通信故障，不能把旧 CPU2 缓存数据表现为正在刷新 |
| `STATE_WIRELESS_PAIRING` | 特殊页 | 状态 | 无线滑环匹配状态 | `oled_equipment()` 显示状态后直接返回 |
| `STATE_WIRELESS_PAIRING_OVER` | 特殊页 | 状态、MAC 或 `MAC N/A` | `wireless_pairing_status` | 成功且 MAC 有效时分两行显示 MAC |

## 3. 仅显示固定项的状态

以下状态当前未归入测量结果上下文，状态页显示状态、位置、扭力和电机图标，不显示历史液位、水位、密度、温度等业务测量值：

| 状态 | 备注 |
| --- | --- |
| `STATE_STANDBY` | 待机态显示位置、扭力；未实现最近结果缓存 |
| `STATE_INIT` | 初始化状态 |
| `STATE_BACKZEROING` | 回零点中 |
| `STATE_FINDZEROING` | 标定零点中 |
| `STATE_READPARAMETERING` | 读取参数中 |
| `STATE_SETZEROCIRCLING` | 设置零点编码值中 |
| `STATE_SETZEROANGLING` | 设置零点编码值中 |
| `STATE_EFACTORYSETTING_RESTORING` | 恢复出厂设置中 |
| `STATE_BACKUPING` | 备份配置文件中 |
| `STATE_RESTORYING` | 恢复配置文件中 |
| `STATE_ONTANKOPRATIONING` | 罐上仪表操作中 |
| `STATE_FINDZEROOVER` | 标定零点完成 |
| `STATE_SETZEROCIRCLOVER` | 设置零点编码值完成 |
| `STATE_SETZEROANGLOVER` | 设置零点编码值完成 |
| `STATE_EFACTORYSETTING_RESTOROVER` | 恢复出厂设置完成 |
| `STATE_BACKUPOVER` | 备份配置文件完成 |
| `STATE_RESTORYOVER` | 恢复配置文件完成 |
| `STATE_ONTANKOPRATIONCOMPLATE` | 罐上仪表操作完成 |

## 4. 现场确认点

| 待确认项 | 当前实现 | 建议确认 |
| --- | --- | --- |
| 水位低于盲区 | `water_level == LEVEL_DOWNLIMITWATER` 时隐藏水位 | 是否需要显示“低于盲区”或“无有效水位” |
| 单点监测位置来源 | `STATE_SPTESTING` 当前固定显示 `debug_data.sensor_position` | 是否需要改为显示单点监测结果自带位置 |
| 分布测点数 | 分布完成态当前不显示 `measurement_points` | OLED 行数不足时是否需要显示点数，以及优先级 |
| 待机最近结果 | `STATE_STANDBY` 当前不显示最近测量结果 | 是否新增“最近”来源标签和本地缓存 |
| CPU2通信超时故障 | 状态快照、完整参数快照、当前连接协议快照、连续请求失败和故障恢复锁存已拆分；已建立快照后，第1～2次失败保留快照并重试，第3次失败才清空公开快照并完整重同步，连续第10次未获得合法响应进入 `STATE_ERROR`，错误码 `0x00140007`，屏幕显示 `20-7`。上电或恢复先独立读取`0x0010~0x0011`协议字段，协议兼容后才读取状态、完整参数和扩展运行态；协议不匹配时只保留版本心跳并显示“协议版本不匹配”。任何已发起的非命令FC16失败，或FC16已返回合法ACK但紧随的定向FC03补读失败，都会立即关闭参数快照并强制补读四组。主循环可调度时按100 ms门限检查轮询，外部流量不能永久饿死CPU2 | 先建立完整快照，再分别复测第1、2、3、9、10次连续失败；确认前两次不退回通讯尝试页，第3次开始完整重同步，第10次显示20-7；覆盖超时、非法帧、UART/TX失败；交叉烧写协议18/19/20 CPU2，确认只读取稳定协议地址、不访问新扩展区、不累计通信超时并显示协议不匹配；对菜单/DSM/SI/Wartsila参数写注入ACK丢失，确认参数快照立即失效且强制补读不依赖更新标志；Wartsila补读前四参数读返回 `0x06`，补读后返回CPU2实际值；持续外部流量下记录含1000 ms同步阻塞在内的最坏轮询间隔 |
| 读取参数页分页 | 读取参数完成态当前包含位置、扭力、温度、频率、电容、X/Y角、RSSI，超过一屏时按状态页分页显示 | 现场确认翻页操作是否足够直观 |
| RSSI 无效值 | `rssi_valid == 0` 时显示 `RSSI:N/A`；RSSI 查询只读当前连接，不扫描、不断开、不保存默认连接 | 现场确认是否需要在无效时额外显示错误码或连接状态 |
| 设置菜单空闲退出 | CPU3 设置菜单 120 秒无按键后自动返回状态页，屏幕亮度/息屏等显示设置生效后仍按该超时规则处理 | 现场确认配置页长时间无人操作时退出状态页是否符合调试习惯 |

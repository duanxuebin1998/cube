# SI协议需求与实施总览

日期：2026-07-20

本文是 SI 计划与需求目录的主维护文档，合并原“官方资料整理”“说明书需求确认”“下一步工作计划”“CPU2 建议新增项”“协议适配执行计划”和“进一步兼容需求与实施方案”的有效内容。旧拆分文档不再单独维护；待确认事项集中维护在 `SI协议待确认与后续清单.md`。

本文同时维护需求、设计、实现状态、影响范围和验证口径。2026-07-01 将SI profile从普通分布测量解耦；2026-07-02按官方中文全文译稿补充受控偏差；2026-07-14按站2抓包和用户确认完成共享协议17→18、Point0/递增点数/最终同代发布、私有镜像、CPU3本地兼容槽和FRAM V6→V7实现。

## 1. 当前定位

当前 SI协议适配已经完成第一版基础协议兼容：

- CPU3 作为 SI Modbus RTU 从站，支持 `FC01/FC02/FC03/FC04/FC05/FC06`。
- SI 串口参数在选择协议后锁定为 `9600 8O1`。
- CPU3 负责 SI协议地址表、功能码、异常响应、单位缩放、保持寄存器快照、CPU3 SI 参数、报警合成、时间戳和命令桥接。
- CPU2 负责真实测量、电机动作、液位/密度/profile 数据、通用辅助状态和必要通用命令。
- CPU2 不理解 SI 地址号、线圈、功能码、缩放和 Modbus 异常码。
- `tools/check_si_modbus_frames.py` 和 `tools/check_si_protocol_contract.py` 已用于保护 SI协议地址表、参考帧和 CPU2/CPU3 共享字段契约。

SI功能已经推进到协议18生命周期兼容正式基线，首个匹配固件为CPU2 `V1.26.0.0`、CPU3 `V1.24.0.0`；源码、协议契约、参考帧和双端clean-first构建已经完成。最新正式组合为协议27、CPU2 `V1.31.0.0`、CPU3 `V1.30.0.0`。协议27重排CPU2/CPU3共享地址但不改变SI对外地址、生命周期和参数；SI Stop改为下发取消测量命令16，不再隐式进入维护模式。真实RS485、V6实镜像迁移与写失败注入、掉电保存、CPU2/CPU3重启、PLC响应时延、完成边沿一致性、取消/失败/通信中断及1/30/200点边界仍需台架验证，协议18提交前审查遗留见 `SI协议待确认与后续清单.md` 第7节。

## 2. 官方资料要点

资料依据：

| 类型 | 本地路径 | 用途 |
| --- | --- | --- |
| 官方操作手册 | `docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000官方操作手册.pdf` | Auto、Cal、Profile、Manual、Power-up、报警和传感器状态语义 |
| 官方资料包/含 DCS Modbus 规范 | `docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000官方资料包_含Modbus规范.pdf` | 功能码、线圈、离散输入、输入寄存器、保持寄存器、缩放和串口参数 |
| 原始协议资料 | `docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000协议.docx` | 早期 SI 地址和软件需求依据 |

关键协议要求：

- 物理层：RS-485。
- 协议：Modbus RTU。
- 串口格式：`9600 8O1`。
- 功能码：`FC01`、`FC02`、`FC03`、`FC04`、`FC05`、`FC06`。
- 线圈区：`00001~00016`，用于 Manual、Cal、Auto、Profile、Stop 和手动方向/速度。
- 离散输入区：`10001~10032`，用于状态、限位、报警和 profile 完成状态。
- 保持寄存器区：`40001~40023`，用于 profile 参数、自动 profile 参数和报警设定点。
- 输入寄存器区：`30001~30620`，用于实时测量值、当前时间和 profile 点阵。
- DCS 地址表暴露 `30021~30620`，即 200 个 profile 点；若 PLC 需要 250+ 点，需要另定扩展方案。

测量行为要点：

- Auto 模式跟随液/气界面。SI 原机依赖上下两个液位传感器，下传感器在液体中、上传感器在气相中时认为探头在液位界面。
- Cal 模式驱动探头到罐底，建立 bottom reference，然后返回液位界面并回 Auto。
- Profile 模式按 SI 语义应先到罐底采集第 0 点；协议18最终点阵中的 Point0 位置输出本轮实际采用的底部参考，`40001 Profile First Point` 是 Point1 的绝对位置。
- Manual 模式用于维护。下发 Up/Down/Stop 会停止 profile 或液位跟踪并进入 Manual；官方明确 Manual 期间不报告报警、不更新液位，系统保持 Manual 直到返回 Auto/Cal/Profile。
- Profile 时间戳更接近 profile 数据采集开始或首点采集时刻，不应长期使用完成时刻。
- 官方操作手册还明确：Profile run 和 Cal run 完成后返回 Auto；自动 Profile 到点时若系统处于 Manual 不会开始。
- `Probe At Liquid Level`、`Probe Un-calibrated` 和 `Profile Complete` 是判断液位或 Profile 数据是否可信的关键状态，不能只看 `30004` 或点阵本身。

## 3. 当前实现基线

| 项目 | 第一版实现 | 2026-07-01 后当前实现 |
| --- | --- | --- |
| SI Profile 线圈 | 第一版 `00004` 写入后下发 `CMD_MEASURE_DISTRIBUTED` | 下发 `CMD_SI_PROFILE = 20`；命令成功进入PREPARING，Point0周期事件才锁存时间 |
| profile 测量流程 | 复用普通分布测量流程，点阵由现有分布测量结果生成 | CPU2独立流程生成候选，Point0输出本轮实际采用底部参考；CPU3回液位后完成点阵分块和代际复核，再发布最终结果 |
| 探底频次 | 已有 Wärtsilä 探底间隔参数和流程可参考 | CPU2 新增 `si_profile_bottom_detect_interval`，不复用 Wärtsilä 参数 |
| `10001` | 当前按 `bottom_reference_valid` 映射 | 保持现有逻辑，不改为固定值 |
| `10010` | 当前按 `zero_point_status` 映射 | CPU3 SI 层固定 `0`，对外显示位置可信 |
| `10002/10003` | 当前固定 `0` | CPU3 按液位跟随稳定状态合成：稳定跟随时下液体上空气，其它状态均液体 |
| `40001~40003` | 第一版桥接 `spreadTopLimit`、`spreadMeasurementDistance`、`spreadPointHoverTime` | 桥接 CPU2 `si_profile_first_point/increment/dwell_time`，普通分布参数不受影响 |
| `40010~40013` | 第一版为 CPU3 影子回读，不真实调度 | CPU3 本机参数，掉电保存并驱动自动 profile 调度 |
| `40014~40023` | 第一版为 CPU3 影子回读，并参与部分报警合成 | CPU3 SI协议独立掉电保存参数，由 CPU3 判断并合成报警状态 |
| profile 时间戳 | 第一版按完成计数变化锁存完成时刻 | CPU3观察到协议18 Point0新周期时锁存；首次上线/重连无法重建时四项为0 |

### 3.1 官方全文复核后的主要偏差

| 项目 | 当前状态 | 影响 |
| --- | --- | --- |
| Manual 报警抑制 | CPU2 本机继电器报警支持 `manual_alarm_inhibit`，但 CPU3 SI 报警位仍按阈值合成 | Manual 期间 PLC 可能看到 SI 报警，和官方语义不一致 |
| Manual 下自动 Profile | CPU3 自动调度到点后直接触发 SI profile，没有 Manual 跳过条件 | 维护或手动操作可能被自动 profile 打断 |
| Profile/Cal 后回 Auto | SI profile 成功后会在 CPU2 侧排队 `CMD_FIND_OIL`，下一轮主循环进入液位跟随；`00002 Calibrate` 当前仍桥接零点标定，不是完整 Level Calibration | Profile 后可观察 `00003 Auto` 和 `10013` 恢复；Cal 后仍需按待确认项处理 |
| Probe Un-calibrated | `10010` 固定输出 `0` | 外部主机无法通过该位发现底部参考或位置可信性风险 |
| 双液位传感器 | `10002/10003` 是合成值，不是真实上下 level sensor | 不能用于真实硬件联锁 |
| Interlock/Reel Alarm | `10004/10009` 由错误码或阻塞状态概括合成 | 不能区分高限互锁、卷盘故障和普通错误 |
| Profile 时间戳 | `30007~30010` 已改为Point0新周期事件时间；无法重建时为0 | Point0边界已兼容；RTC失败和重启恢复仍需台架验证 |
| 探底失败 fallback | 无可信旧底且探底失败时可用当前位置继续，Point0输出实际采用位置 | 数据可用性优先；当前位置降级不等于可靠物理罐底，需保留诊断 |

这些偏差的处理决策维护在 `SI协议待确认与后续清单.md` 的 R-01 到 R-09，PLC 侧验收步骤维护在 `../04_联调测试/SI协议_PLC联调检查表.md`。

### 3.2 2026-07-01阶段实现结果

> 本节是2026-07-01协议14/FRAM V6阶段的历史快照，其中“命令触发时锁存时间”和`0x0006`不是协议18建立并由协议19沿用的口径。当前实现以3.4节及后续正文为准：Point0建立新周期时锁存时间，FRAM版本为V7/`0x0007`。

| 类别 | 已落地内容 | 关键文件 |
| --- | --- | --- |
| 共享协议 | `DEVICE_PROTOCOL_VERSION` 升至 `14`；新增 `CMD_SI_PROFILE = 20`，并增加 `profile_source` 隔离 SI profile 结果 | `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h`、`LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h` |
| CPU2 profile 参数 | 复用原预留槽新增 `si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time`、`si_profile_bottom_detect_interval`；`DEVICE_PARAM_VERSION` 保持 `3`，通过协议版本迁移和运行期归一化补默认值 | `system_parameter.h/c`、`stateformodbus.h`、`dataanalysis_modbus.c` |
| CPU2 profile 流程 | 新增 `CMD_SiProfile()`，支持探底频次、旧底部位置回退、Point0、最多 200 点、候选点运行中判定液面以上后停止、完成锁存和计数 | `LTD_MAIN_CPU2/Application/Src/measure_density.c`、`measure.c`、`fault_recovery.c` |
| CPU3 本机参数 | 2026-07-01阶段新增 `40010~40023` 并把FRAM升至V6/`0x0006`；协议18已继续升至V7/`0x0007`并增加六个兼容槽，协议19沿用 | `cpu3_comm_display_params.h/c` |
| CPU3 菜单 | `参数配置 -> 测量参数 -> SI参数` 下新增 `Profile参数`、`自动Profile`、`报警限值` 三个子页 | `display_tankopera.h/c`、`system_parameter.c` |
| SI Modbus | 历史阶段由`00004 Profile`命令时锁存timestamp；协议18起只下发`CMD_SI_PROFILE`，改为观察Point0新周期事件时锁存，协议19沿用；`si_modbus_periodic_task()`负责自动调度 | `si_modbus_slave.c/h`、`app_main.c` |
| 状态和报警 | `10010=0` 固定可信；`10002/10003` 按液位跟随稳定状态合成；当前值报警、profile 上下限报警和相邻点偏差报警由 CPU3 合成 | `si_modbus_slave.c` |
| 文档和脚本 | golden frame 更新为 CPU3 自动 profile 默认值；协议契约脚本覆盖协议版本、命令号、参数字段和状态口径 | `tools/check_si_modbus_frames.py`、`tools/check_si_protocol_contract.py` |

### 3.3 当前默认值

本地资料未提取到可直接落地的逐项 SI协议手册默认值表，因此本轮代码采用保守默认值，并把“手册逐项默认值复核”保留为现场/资料复核项：

| 参数 | 当前代码默认值 |
| --- | --- |
| CPU2 `si_profile_first_point` | `1000`，单位 `0.1mm`，即 100mm |
| CPU2 `si_profile_increment` | `10000`，单位 `0.1mm`，即 1000mm |
| CPU2 `si_profile_dwell_time` | `10s` |
| CPU2 `si_profile_bottom_detect_interval` | `1`，表示每次 profile 前探底 |
| CPU3 `40010 Automatic Profile Interval` | `60min` |
| CPU3 `40011 Automatic Profile Enable` | `0`，默认关闭 |
| CPU3 `40012/40013` | `00:00` |
| CPU3 `40014~40023` | `0`，作为有效报警阈值参与比较；无效温度和无效密度不参与报警判断 |
| CPU3 `40004~40009` | `0、50、1000、0、5、1`，仅作为站2原始兼容槽，不参与控制 |

### 3.4 2026-07-14协议18实施结果

| 类别 | 当前实现 | 验证状态 |
| --- | --- | --- |
| 共享协议 | `DEVICE_PROTOCOL_VERSION 17 -> 18`；追加 `phase/cycle_counter/progress_points` 三个32位运行态字段，不移动既有点阵和前序地址 | 静态契约通过；混搭拦截待台架 |
| Profile生命周期 | Point0前保留旧Complete/N/时间/点阵/报警；Point0后Complete=0、N从1按有效液体点递增、活动点阵全0 | 源码和双端clean-first构建通过；1/30/200点待台架 |
| 最终发布 | CPU2回到稳定液位且电机停止后提交候选；CPU3分块读取并复核cycle、完成计数、N、来源、阶段，再一次发布Complete/N/点阵/Profile报警 | 静态契约通过；RS485时序待台架 |
| 异常出口 | Point0前取消保留旧结果；Point0后取消/失败清Complete/N/点阵且不恢复旧Complete；取消不置Interlock，真实失败沿用错误链 | 静态契约通过；故障注入待台架 |
| 私有镜像 | `30014~30016`分别镜像FC01、FC02前16位和后16位；`30017~30020=0` | golden frame通过；实机逐阶段对照待台架 |
| 私有保持寄存器 | `40004~40009`默认 `0、50、1000、0、5、1`，支持FC03/FC06、掉电保存且不参与控制；`40004~40023`均采用FRAM写后校验和整份运行态回滚，失败返回FC06异常 `0x06` | 静态契约通过；掉电和故障注入待台架 |
| CPU3存储 | FRAM `V6 / 0x0006 -> V7 / 0x0007`；显式V6结构迁移保留旧SI参数和合法三路串口配置，V6/V7路径不再按值猜测并改写Wartsila `4800 8N1/8N2` | 编译期布局断言和静态契约通过；真实V6镜像迁移待台架 |

## 4. 已确认的进一步兼容需求

根据 2026-07-01 用户确认，本轮 SI协议兼容目标从“地址表和基础读写兼容”提升为“按 Scientific Instruments SI profile 行为做独立测量流程兼容”。核心需求如下：

1. SI `Profile` 必须新增独立 CPU2 命令和独立测量流程，不再复用普通分布测量 `CMD_MEASURE_DISTRIBUTED`。
2. `40001`、`40002`、`40003`、`40010`、`40011`、`40012`、`40013` 都作为 SI 独立保持寄存器，不再直接复用普通分布测量参数；这些参数需要掉电保存。
3. 参数存放方向确认：`40001~40003` 和探底频次放入 CPU2 参数存储，`40010~40023` 放入 CPU3 SI 参数存储；CPU3 负责外部保持寄存器和菜单映射。
4. `40001 Profile First Point` 表示 Point1 的绝对位置；Point0在底部采样，最终对外位置输出本轮实际采用的底部参考。
5. `40002 Profile Increment` 从 `40001` 绝对位置开始逐点递增；`40003 Profile Dwell Time` 是每个点停稳后的等待时间。
6. SI profile 最大 200 点；profile 前不单独找液位，候选点逐点运行，到点采样判定为液面以上时停止，不采集该液面以上点；达到上限或点数上限也停止。
7. SI profile 测量过程按科学仪器方案适配：profile 开始前按探底次数频次决定是否真实探底；不探底时也必须运行到底部位置采集第 0 点密度。
8. 上电后首次 profile 必须尝试探底；不探底时使用上次探底保存的底部位置；如果没有有效底部位置，应尝试探底，探底失败且仍无旧底时使用当前位置继续但不刷新底部参考。
9. 探底失败时优先使用旧底部位置继续；如果没有旧底部位置，则直接继续后续 profile 流程。
10. `10001 Bottom Reference` 按现有逻辑不改，不再作为本轮固定输出项。
11. `10010 Probe Un-calibrated` 固定显示“可信”，即对外始终返回 `0`。
12. `10002 Lower Level Sensor` / `10003 Upper Level Sensor` 按液位跟随状态合成：探针跟随液面稳定后下传感器为液体、上传感器为空气；其它状态均显示在液体，包括故障、维护、手动移动、profile 测量和初始化。
13. 自动 profile 调度需要落地，`40010~40013` 作为 SI 独立保持寄存器参与调度；从起始时间开始按 `40010` 分钟周期运行，可跨天。
14. `40010 = 0` 表示禁用周期，同时禁止将 `40010` 设置为 `0`；关闭自动 profile 应使用 `40011 = 0`。
15. 自动 profile 到点后打断当前测量，直接执行 SI profile；自动 profile 和手动 profile 使用同一套 SI profile 流程。
16. `40014~40023` 全部作为 CPU3 SI协议独立报警限值参数，不复用现有限值；需要掉电保存，CPU3 能合成的报警均由 CPU3 合成。
17. `30007~30010 Profile Timestamp` 以Point0有效样本建立新周期的时刻为准；CPU3首次上线/重连无法重建时本周期四项为0，不能用重连时刻冒充Point0时间。
18. SI 探底频次默认值为 `1`；`1` 表示每次 profile 前探底，更大值表示每 N 次触发探底一次，计数按进入 CPU2 SI profile 入口的触发次数统计，不按成功完成次数统计。
19. 探底失败且没有可信旧底部位置时，使用当前位置作为Point0参考继续；不按失败处理、不置Interlock，Point0输出实际采用的当前位置，但不得宣称该位置是可靠物理罐底。
20. `40001/40002/40003/40010~40023` 默认值按 SI协议手册默认值。
21. 自动 profile 打断当前测量时直接覆盖命令。
22. SI profile 采点形成有效候选后，若当前没有其它待执行命令，CPU2 会进入 `RETURNING_LEVEL` 并排队 `CMD_FIND_OIL`；回到稳定液位且电机停止后才提交最终Complete组合。采点失败或命令切换不发布最终结果。
23. `10025/10026` profile 偏差报警按相邻点差值判断。
24. Tank ID 复用当前 `SlaveAddress`。
25. 接受当前密度缩放和 16 位上限，继续使用 `0.01 kg/m3` 和超限钳位口径。
26. Manual Slow/Medium/Fast 不需要真实三档速度。
27. RTC 使用 CPU3 RTC 当前时间能力，不新增 PLC 校时或额外高精度时钟要求。
28. 不再新增 CPU2/CPU3 profile 命令上下文；`40001~40003` 和 SI 探底频次改为 CPU2 持久化 profile 执行参数，CPU3 通过现有参数读写链路映射。
29. `30014~30016`按站2私有行为镜像FC01/FC02，`30017~30020`继续固定为0。
30. `40004~40009`作为CPU3本地原始兼容槽，默认 `0、50、1000、0、5、1`，支持FC03连续读取、FC06单写和掉电保存，不赋予业务含义；与 `40010~40023` 一样，FRAM保存或写后读回失败时恢复整份写前运行态，并向FC06返回 `0x06`。
31. Point0前保留上一轮已发布Complete、N、时间、点阵和Profile报警；Point0有效样本建立新周期并把N置1。
32. 活动期N只按有效液体点递增，空气点、失败样本和重试不计数；本轮最终点数事先未知。
33. Point0后到最终发布前，`30021~30620`全部为0；回液位并通过CPU3候选分块和代际复核后整体开放新点阵。
34. Point0后显式取消或真实失败均清Complete、N和点阵，不恢复旧Complete；显式取消不置Interlock，真实失败沿用既有错误/Interlock。
35. CPU3 FRAM从V6升至V7，V6迁移必须保留旧SI自动调度、报警阈值和合法三路串口参数，只为六个兼容槽补默认值；不得把合法Wartsila `4800 8N1/8N2`猜成历史默认值并改写。

## 5. 设计原则

- CPU3 继续负责 SI Modbus RTU 地址、功能码、异常响应、缩放、对外保持寄存器表达和自动调度。
- CPU2 新增真实测量所需的通用命令、流程、必要状态和 SI profile 执行参数，但不在 CPU2 中引入 SI 地址号、功能码、异常码或 Modbus 帧细节。
- SI 专用 profile 与普通分布测量、Wärtsilä 密度测量互相隔离，避免改 SI 行为时影响 DSM、Wärtsilä、菜单分布测量或综合测量。
- `40001~40003` 作为 SI 独立保持寄存器处理，但底层存储在 CPU2 profile 执行参数中，避免继续把 SI profile 参数误解释为普通分布测量参数。
- `40004~40009` 原始兼容槽、`40010~40013` 自动 profile 和 `40014~40023` 报警限值均由 CPU3 持久化；FC06只有在整份FRAM镜像写后校验通过后才成功回显，否则回滚整份运行态并返回 `0x06`。
- `40014~40023` 报警限值不复用现有参数，统一作为 CPU3 SI协议专用掉电保存参数；CPU3 能合成的报警均在 CPU3 合成。
- 对外协议状态以已确认口径优先：`Bottom Reference` 保持现有映射逻辑，`Probe Un-calibrated` 固定输出可信。

### 5.1 总体归属

| 类型 | 建议归属 | 说明 |
| --- | --- | --- |
| SI Modbus 地址、功能码、缩放、异常码 | CPU3 | 这是协议外观，不进入 CPU2 |
| `40001 Profile First Point`、`40002 Profile Increment`、`40003 Profile Dwell Time` | CPU2 持久化，CPU3 负责 SI 读写映射和菜单入口 | 不再复用 `spreadTopLimit`、`spreadMeasurementDistance`、`spreadPointHoverTime` |
| `40010~40013` 自动 profile 调度参数 | CPU3 | CPU3 有 RTC、Modbus 和调度入口 |
| `40014~40023` 报警限值类 SI 参数 | CPU3 | 已确认不复用现有限值，作为 SI 独立掉电保存参数 |
| 探底频次和 SI profile 执行策略参数 | CPU2 持久化，CPU3 负责菜单和协议写入入口 | 这是测量执行策略，CPU2 直接读取最稳 |
| Profile 实际运动、探底、Point0、逐点测量、停留 | CPU2 | 控制和测量闭环必须在 CPU2 |
| Point0时间戳、当前时间、SI 状态合成 | CPU3 | `30007~30010` 在CPU3观察到Point0新周期事件时锁存；无法重建时当前周期全0 |
| Lower/Upper Level Sensor 显示规则 | CPU3 | 按 SI协议显示语义由 CPU3 合成 |
| `Bottom Reference`、`Probe Un-calibrated` | CPU3 映射层输出 | `10001` 保持现有逻辑；`10010` 固定可信 |
| Profile complete、counter、blocked、真实测量失败状态 | CPU2 产生，CPU3 映射 | CPU2 产生真实生命周期状态，CPU3 负责 SI 寄存器表达 |

### 5.2 旧新增字段处理

之前新增到 CPU2 的大部分内容是“协议辅助状态”，不是可配置参数。处理原则是：真实测量或安全流程需要的状态留在 CPU2；纯 SI 展示语义尽量由 CPU3 合成。

| 字段或能力 | 建议处理 |
| --- | --- |
| `probe_at_liquid_level`、`liquid_stable` | 保留 CPU2 产生，作为液位跟随稳定状态；CPU3 用于 `10002/10003` 和 `10013` 合成 |
| `profile_complete_latched`、`profile_complete_counter` | 保留 CPU2 产生，作为 profile 完成和点阵有效性的真实来源 |
| `manual_alarm_inhibit`、`manual_level_update_inhibit` | 保留 CPU2，和维护/手动控制过程有关 |
| `bottom_reference_valid` | 保留 CPU2 内部和现有 `10001` 映射逻辑；本轮不固定覆盖 `10001` |
| `profile_blocked_by_process`、`loading_unloading_active` | 若只服务 SI 显示，可由 CPU3 合成；若未来参与 CPU2 联锁，再由 CPU2 产生 |
| `profile_temp_deviation_alarm`、`profile_density_deviation_alarm` | 本轮优先 CPU3 按 SI 点阵和 `40022/40023` 合成；如算法必须依赖 CPU2 原始过程数据，再新增最小共享结果 |

### 5.3 参数组、执行参数和状态回传

建议形成四段边界，避免 CPU2 被 SI Modbus 细节污染，同时让 profile 执行参数由执行侧持久化。

| 边界 | 内容 | 所在 CPU | 说明 |
| --- | --- | --- | --- |
| CPU2 SI profile 执行参数 | `si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time`、`si_profile_bottom_detect_interval` | CPU2 | CPU2 掉电保存并在 `CMD_SI_PROFILE` 执行时直接读取；CPU3 菜单和 SI `40001~40003` 只作为读写入口 |
| CPU3 SI协议参数 | `si_compat_holding[0..5]`、`auto_profile_interval`、`auto_profile_enable`、`auto_profile_hour`、`auto_profile_minute`、`alarm_limit_*`、后续兼容开关和诊断项 | CPU3 | 对应 `40004~40023`；六项兼容槽只保存/回读，其余参数用于报警和调度，统一采用事务式FRAM写 |
| Profile 触发命令 | `CMD_SI_PROFILE` | CPU3 下发，CPU2 执行 | `00004 Profile`、CPU3 屏幕 `SI Profile` 和自动调度共用启动入口；只表示“启动 SI profile”，不携带 profile 参数上下文 |
| 状态回传 | profile lifecycle、Point0 captured、complete latch/counter、点阵、测量失败/阻挡状态、液位跟随稳定状态 | CPU2 产生，CPU3 读取并映射 | CPU3 再转换成 `100xx`、`300xx` 和报警位 |

该结构的目标是把 SI 专用配置从普通分布参数中拆出来：执行参数归 CPU2，调度和报警协议参数归 CPU3。CPU2 可以知道“SI profile 执行参数”的名字，但不处理 `40001` 这类外部地址、功能码或异常码。

## 6. SI Profile 独立测量流程

### 6.1 新增命令

已新增 CPU2/CPU3 共享命令：

| 命令 | 编号 | 用途 |
| --- | --- | --- |
| `CMD_SI_PROFILE` | `20` | 执行 SI 专用 profile 测量 |

CPU3 `si_apply_coil_write()` 中 `SI_COIL_PROFILE` 不再发送 `CMD_MEASURE_DISTRIBUTED`，改为发送 `CMD_SI_PROFILE`。CPU3 屏幕“密度分布测量 -> SI Profile”也必须调用同一 SI profile 启动入口，保证 `30007~30010` 时间戳和外部 `00004`、自动调度一致。

该变更会改变 CPU2/CPU3 共享命令契约，需要升级 `DEVICE_PROTOCOL_VERSION`，并同步更新 CPU2/CPU3 协议变更记录、SI 映射表和 golden frame/契约检查脚本。

### 6.2 测量步骤

CPU2 已新增独立入口 `CMD_SiProfile()`，流程如下：

1. 收到 `CMD_SI_PROFILE`。
2. 从 CPU2 参数区读取 `si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time` 和 `si_profile_bottom_detect_interval`。
3. 进入 `PREPARING`，保留上一轮已发布Complete、N、时间、点阵和Profile报警，不在命令入口提前清旧结果。
4. 根据SI探底频次判断是否真实探底；上电首次必须尝试探底。新探底成功优先使用新底，失败时沿用可信旧底；两者都没有时使用当前位置作为本轮Point0参考继续，不按失败处理、不置Interlock。
5. 在本轮采用的底部参考位置等待稳定并采集Point0温度、密度；Point0位置写入实际采用的底部参考。
6. Point0有效样本提交时清本轮候选Complete和旧发布载荷，写 `progress_points=1`、`phase=MEASURING`，使用内存屏障后最后递增 `cycle_counter`。CPU3只把该周期变化视为Point0事件，并在连续在线时锁存RTC。
7. 按CPU2保存的profile参数生成后续停点：Point1=`first_point`，Point2=`first_point+increment`，PointN=`first_point+(N-1)*increment`。
8. 逐点运行、停稳后等待、采集温度和密度；只有有效液体点提交后才递增 `progress_points`。失败样本、重试和空气点不计数；空气点停止本轮且不写入点阵。
9. 达到液面、罐高或200点上限后冻结最终N，进入 `RETURNING_LEVEL`。此时对SI仍是Profile=1、Complete=0，活动点阵全0。
10. 找液位成功、液位稳定且电机停止后，CPU2把候选点阵、最终N、液位和SI来源复制到共享结果，最后递增 `profile_complete_counter` 并进入 `COMPLETE`。
11. CPU3读取紧凑头和生命周期字段，按内部帧长分块拉取完整点阵，再复读头部；cycle、完成计数、N、来源或阶段任一变化都丢弃候选并从头重试。
12. CPU3计算六项Profile报警；只有AtLevel、Stable、电机停止、Interlock=0和候选代际复核全部通过，才一次发布Profile=0、Auto=1、Stop=1、Complete=1、最终N、完整点阵和Profile报警。
13. Point0前显式取消不建立新周期并保留上一轮结果；Point0后取消或失败清Complete、N和点阵且不恢复旧Complete。显式取消不置Interlock，真实失败沿用既有错误/Interlock出口。
14. CPU3首次上线或重连不能把当前时刻冒充Point0时间；无法重建时当前周期 `30007~30010`保持0。

### 6.3 探底频次

已新增 SI 专用 CPU2 执行参数：

| 参数 | 含义 | 当前归一化范围 | 默认值 |
| --- | --- | --- | --- |
| `si_profile_first_point` | 第一个可编程测点绝对位置，CPU2 内部单位 `0.1mm` | `0` 会归一化为默认值 | `1000` |
| `si_profile_increment` | 每点步距，CPU2 内部单位 `0.1mm` | `0` 会归一化为默认值 | `10000` |
| `si_profile_dwell_time` | 每点停稳后的等待时间，单位秒 | `1~3600`，超出归一化为默认值 | `10` |
| `si_profile_bottom_detect_interval` | SI profile 探底次数频次；`1` 表示每次 profile 前探底，`N` 表示每 N 次 SI profile 触发前探底一次；进入 CPU2 SI profile 入口后即计数，不等待 profile 成功完成 | `1~1000`，超出归一化为默认值 | `1` |

不建议复用 `wartsila_bottom_detect_interval`，原因是该参数已经是 Wärtsilä 专用语义；SI profile 的点阵和第 0 点含义不同，后续现场参数也可能不同。

### 6.4 底部位置和第 0 点

本轮明确要求：不探底时也必须运行到底部位置读取第 0 点密度。

建议把“真实探底”和“运行到底部测量位置”分开：

| 动作 | 含义 | 是否刷新底部参考 |
| --- | --- | --- |
| 真实探底 | 用扭力或角度等底部检测方式确认罐底 | 是，内部可刷新底部参考 |
| 运行到底部测量位置 | 按已有底部参考或配置位置运行到第 0 点采样位置 | 否，不应改变内部探底可信状态 |

底部参考优先级为“本轮新探底结果→可信旧底→当前探头位置降级”。没有有效底部时仍先尝试探底；若探底失败且没有可信旧底，则使用当前位置继续，不按失败处理、不置Interlock，但保留内部诊断并且不得把该位置解释为可靠物理罐底。最终SI点阵的Point0位置输出本轮实际采用的参考，不使用普通分布测量 `spreadBottomLimit`。活动期Point0点阵本身不提前开放，只有最终代际复核通过后才随整套点阵发布。

## 7. 状态位输出方案

### 7.1 Lower/Upper Level Sensor

| 状态 | `10002 Lower Level Sensor` | `10003 Upper Level Sensor` | 对外含义 |
| --- | --- | --- | --- |
| 液位跟随状态 | `1` | `0` | 下传感器在液体，上传感器在空气 |
| 其它状态 | `1` | `1` | 两个传感器均显示在液体 |

第一版建议在 CPU3 SI协议映射层根据 CPU2 设备状态合成：

- `STATE_FLOWOIL` 或后续明确的液位跟随状态：按“下液体、上空气”输出。
- 其它状态：按“两者均在液体”输出。

### 7.2 特殊状态输出

| 地址 | 名称 | 输出口径 | 内部状态处理 |
| --- | --- | --- | --- |
| `10001` | Bottom Reference | 保持现有逻辑，不在本轮固定覆盖 | 内部真实探底状态仍可继续维护，用于测量流程、底部位置和故障判断 |
| `10010` | Probe Un-calibrated | 固定返回 `0`，一直显示可信 | 内部 `zero_point_status`、编码器可信状态、探底失败等仍用于本机保护和测量错误处理 |

文档和联调表必须注明：SI协议对外 `10010` 不再等同 CPU2 内部 `zero_point_status`；`10001` 保持现有映射口径。

## 8. 独立保持寄存器方案

以下地址改为 SI 独立保持寄存器，不再直接复用普通分布测量参数：

| 地址 | 名称 | 独立参数含义 | 单位/范围建议 |
| --- | --- | --- | --- |
| `40001` | Profile First Point | Point1 绝对位置，不能覆盖 Point0 | mm，`1~65535`；写0按当前CUBE安全边界拒绝 |
| `40002` | Profile Increment | 从 `40001` 开始逐点递增的 profile 每点步距 | mm，`1~65535` |
| `40003` | Profile Dwell Time | 每点停稳后的等待时间 | 秒，`1~3600` |
| `40004~40009` | 站2私有兼容槽 | 不解释业务含义的六个原始 `uint16_t` 值 | 默认 `0、50、1000、0、5、1`；完整16位可读写 |
| `40010` | Automatic Profile Interval | 自动 profile 周期 | 分钟，禁止写入 `0`；`0` 仅作为禁用/未配置内部值 |
| `40011` | Automatic Profile Enable | 自动 profile 使能 | `0/1` |
| `40012` | Automatic Profile Hour | 自动 profile 起始小时 | `0~23` |
| `40013` | Automatic Profile Minute | 自动 profile 起始分钟 | `0~59` |

`40003` 的“原协议”在本文中指 SI DCS Modbus 资料口径：单位为秒，含义是探针到达每个 profile 停点并停稳后的等待时间。操作手册 Profile Setup 页面曾出现毫秒口径，但对外 Modbus 兼容按 DCS 秒口径实现和联调。

存储实现：

- `40001~40003` 底层存入 CPU2 `DeviceParameters`，作为 SI profile 执行参数；本轮复用原预留槽，`DEVICE_PARAM_VERSION` 保持 `3`，通过 `protocolVersion < 14` 迁移和运行期归一化补默认值。
- SI 探底频次同样放入 CPU2 参数存储，因为它直接决定 profile 执行流程。
- `40004~40023`由CPU3持久化；CPU3本机参数FRAM版本已经从V6/`0x0006`升至V7/`0x0007`。V6迁移使用显式旧结构和CRC校验，完整保留旧SI参数与合法三路串口配置，只为 `40004~40009`补默认值；V6迁移和V7重载都不再按合法物理参数猜测Wartsila历史默认值；运行期FC06写只有在写后读回校验通过时成功，失败整份回滚并返回 `0x06`。
- CPU3 对外仍暴露 `40001~40023` 保持寄存器；其中 `40001~40003` 的读写必须桥接 CPU2 参数同步链路，不能再只写 `s_holding_regs` 影子值。
- 这些保持寄存器当前按保守默认值落地；SI协议官方手册逐项默认值仍需现场或资料复核。

## 9. 自动 Profile 调度

自动 profile 调度已放在 CPU3：

- CPU3 持有 SI协议、`40010~40013` 和 RTC。
- 到点后由 CPU3 向 CPU2 下发 `CMD_SI_PROFILE`。
- CPU2 只负责收到命令后执行真实测量流程。

实现规则：

1. `40011 = 0` 时不自动触发 profile。
2. `40011 = 1` 时启用自动 profile。
3. 第一次触发时间为 `40012:40013`。
4. 后续按 `40010` 分钟周期触发，可跨天。
5. `40010 = 0` 表示周期禁用或未配置，但 FC06 写入 `40010 = 0` 应返回非法数据值；关闭自动 profile 使用 `40011 = 0`。
6. 自动 profile 到点时打断当前测量，直接覆盖当前命令并下发 SI profile；自动 profile 和手动 profile 使用同一套 CPU2 SI profile 流程。
7. 当前只按分钟去重，避免同一分钟重复触发；若触发时 CPU2 已在其它测量或 profile 中，仍按用户确认的“直接覆盖命令”口径下发 `CMD_SI_PROFILE`。
8. `10012 Interval Timer` 当前按自动 profile 使能状态输出；`40010` 被值域检查保证为非 0。

当前已维护上次触发分钟，避免同一分钟重复触发；下次触发时间、触发源、跳过原因和自动触发计数可作为后续诊断页增强。

## 10. 报警限值和状态兼容

需要兼容的报警限值和状态包括：

| 限值地址 | 状态地址 | 含义 |
| --- | --- | --- |
| `40014` | `10017`、`10031` | Low Density / Profile Low Density |
| `40015` | `10018`、`10032` | High Density / Profile High Density |
| `40016` | `10019`、`10029` | Low Temperature / Profile Low Temperature |
| `40017` | `10020`、`10030` | High Temperature / Profile High Temperature |
| `40018` | `10021` | LL Level Alarm |
| `40019` | `10022` | HH Level Alarm |
| `40020` | `10023` | Low Level Alarm |
| `40021` | `10024` | High Level Alarm |
| `40022` | `10025` | Profile Temp Deviation Alarm |
| `40023` | `10026` | Profile Density Deviation Alarm |

配置口径：

- `40014~40023` 均为 CPU3 SI协议独立掉电保存参数，不复用现有报警限值参数。
- 可复用现有报警比较函数、状态读取和工程量换算能力，但不能让 PLC 写 SI 限值时意外改变 AO、继电器、Wärtsilä、DSM 或普通分布测量参数。
- CPU3 能合成的报警均由 CPU3 合成；只有后续发现必须依赖 CPU2 原始过程数据时，才新增最小 CPU2 共享结果。

状态生成建议分两层：

1. 当前值报警：CPU3 读取当前密度、温度、液位，按 `40014~40021` 计算 `10017~10024`；阈值 `0` 是有效值，不再作为禁用条件。
2. Profile 报警：SI profile 完成后由 CPU3 扫描有效点阵，生成 `10029~10032`；`10025/10026` 按相邻点温度差和密度差分别与 `40022/40023` 比较生成。

## 11. Profile Timestamp

用户要求：`Profile Timestamp` 改成测量开始时间。

实施口径：

- `30007`：profile 测量开始月份。
- `30008`：profile 测量开始日期。
- `30009`：profile 测量开始小时。
- `30010`：profile 测量开始分钟。

协议18由CPU3在连续在线观察到 `cycle_counter`变化时锁存本机RTC，该变化只在Point0有效样本提交后发生。自动、手动和屏幕profile使用同一Point0事件口径。首次上线或断线重连时即使CPU2已经处于MEASURING/RETURNING_LEVEL/COMPLETE，也不能用当前时间补造Point0时间；无法重建的当前周期四个寄存器始终为0，下一周期Point0再尝试锁存。

RTC 使用 CPU3 RTC 当前时间能力，不新增 PLC 校时或额外高精度时钟要求。若现场后续要求更高时间精度，再单独评估 LSE/校时入口。

## 12. 其它协议口径

| 项目 | 已确认口径 |
| --- | --- |
| Tank ID | 复用当前 `SlaveAddress`，不新增独立 SI Tank ID 参数 |
| 密度缩放 | 接受当前 `0.01 kg/m3` 缩放和 16 位上限；超过范围继续按现有钳位口径 |
| Manual Slow/Medium/Fast | 不需要真实三档速度，继续按方向命令处理 |
| Profile 完成后状态 | 采点完成后先进入 `RETURNING_LEVEL` 并排队 `CMD_FIND_OIL`；回到稳定液位、电机停止且候选代际复核通过后，才发布最终Complete并进入液位跟随 |

## 13. CPU2/CPU3 能力边界

CPU2 建议新增的是“中性变量、通用参数、通用判定能力”，不要新增一整套 `SI_MODE_xxx` 内核。

CPU2 优先新增或细化的能力：

| 能力 | 用途 |
| --- | --- |
| `CMD_SI_PROFILE` 独立执行入口 | 承接 CPU3 下发的 SI profile 启动命令 |
| SI profile 执行参数 | CPU2 持久保存 `first_point`、`increment`、`dwell_time` 和探底频次，执行时直接读取 |
| `bottom_reference_valid` 或等价底部参考状态 | 内部测量和通用位置基准判断；`10001` 继续按现有逻辑映射 |
| 底部位置有效性和探底计数 | 支持上电首次探底、按触发次数探底、不探底时到底部位置采 Point0 |
| `probe_at_liquid_level`、`liquid_stable` | 液位跟随、报警抑制、Auto 状态判断 |
| `profile_complete_latched`、`profile_complete_counter` | 回液位后的最终完成锁存和候选代际校验；Profile时间戳改由Point0 `cycle_counter`事件触发 |
| `profile_blocked_by_process` | 表达工况禁止 profile |
| `manual_alarm_inhibit`、`manual_level_update_inhibit` | Manual 期间报警和液位更新口径 |
| profile 生命周期事件 | start、bottom reached、Point0 captured、complete、clear |
| profile 点阵和失败状态 | 提供 CPU3 输出 `30021~30620`、`10005` 和必要的异常状态 |

CPU3 应继续负责：

- 罐底基准坐标换算。
- `Profile Timestamp` 和 `Current Time`。
- CPU3 SI协议自动调度参数、报警限值、保持寄存器快照和掉电保存。
- 固定值/保留值填充。
- SI 地址语义到 CPU2 参数读写、启动命令和状态映射的转换。
- `40014~40023` 报警限值判断和 `10017~10026/10029~10032` 报警状态合成。
- 自动 profile 调度、直接覆盖当前命令策略和 `CMD_SI_PROFILE` 触发。

## 14. 实施完成情况

本轮已按两个主阶段落地：

1. CPU2 profile 执行参数和独立测量阶段：
   - 新增 `CMD_SI_PROFILE`。
   - 新增 CPU2 SI profile 执行参数：`first_point`、`increment`、`dwell_time`、探底频次。
   - 将 `40001~40003` 从普通分布测量参数中解耦，不再写 `spreadTopLimit`、`spreadMeasurementDistance`、`spreadPointHoverTime`。
   - 复用 CPU2 预留参数槽，`DEVICE_PARAM_VERSION` 保持 `3`，通过协议版本迁移和运行期归一化补默认值。
   - 新增 SI profile 独立流程，落地 Point 0、探底频次、profile 点阵和必要失败状态。
2. CPU3 映射、菜单和调度阶段：
   - 将 `40001~40003` 的 SI Modbus 读写和 CPU3 菜单项桥接到 CPU2 profile 执行参数。
   - 将 `40010~40013` 变成 CPU3 独立保持寄存器并驱动自动 profile 调度。
   - 将 `40014~40023` 作为 CPU3 SI协议独立报警限值持久化。
   - 修正 `10010/10002/10003`，并保持 `10001` 现有逻辑。
   - 修正 profile 开始时间戳，补齐当前值报警和基础 profile 报警。

本轮具体完成顺序和代码归属：

1. 先扩展验证：SI 功能码、异常帧、缩放边界、报警合成、共享协议一致性。
2. CPU2 新增 SI profile 执行参数和参数版本迁移。
3. CPU3 菜单和 SI Modbus 将 `40001~40003` 读写桥接到 CPU2 参数，不再桥接普通分布参数。
4. 新增 `CMD_SI_PROFILE`，升级 `DEVICE_PROTOCOL_VERSION`。
5. CPU2 新增 SI profile 独立测量入口，执行时直接读取 CPU2 保存的 SI profile 参数。
6. CPU3 把 `00004 Profile` 改为只下发 `CMD_SI_PROFILE`，并落地自动调度、状态合成、报警合成，以及Point0周期事件到达时的timestamp锁存。
7. 更新映射表、PLC 联调表、协议变更记录、版本资料和测试记录。

### 14.1 详细任务拆解

| 阶段 | 目标 | CPU3 改动 | CPU2 改动 | 验证重点 |
| --- | --- | --- | --- | --- |
| 1. CPU2 参数源收拢 | SI profile 执行参数存入 CPU2 | 菜单和 Modbus 写入路径不再写普通分布参数 | 新增 `si_profile_first_point`、`si_profile_increment`、`si_profile_dwell_time`、`si_profile_bottom_detect_interval`；`DEVICE_PARAM_VERSION` 保持 `3` | 上电、恢复默认、断电重启后参数一致 |
| 2. CPU3 协议参数 | 兼容槽、自动调度和报警限值存入 CPU3 | CPU3 SI参数组覆盖 `40004~40023`；本机参数FRAM由V6/`0x0006`升至V7/`0x0007`，V6迁移保留既有SI参数和合法三路串口配置 | 无 | 全部SI本机参数掉电保存；V6实镜像迁移后复核串口原值 |
| 3. 菜单入口 | 现场可直接配置 SI profile、自动调度和报警限值 | 在参数配置菜单新增 `SI参数` 入口；profile 页桥接 CPU2 参数，自动和报警页走 CPU3 本机参数 | 接收 CPU3 参数写入并保存 `40001~40003` 对应执行参数 | 菜单显示、输入范围、写入后回读、两 CPU 同步 |
| 4. 保持寄存器解耦 | `40001~40023` 不再复用普通分布测量参数 | `si_modbus_slave.c` 按地址分别读写 CPU2 profile 参数和 CPU3 SI 参数；非法值返回异常 | 保存 profile 参数并供执行流程读取 | PLC 写入和菜单写入互相可见，不污染普通分布参数 |
| 5. 启动命令 | Profile 只下发启动命令，不带上下文 | 下发 `CMD_SI_PROFILE`，不在命令入口锁存timestamp；观察到Point0 `cycle_counter`边沿时才锁存RTC | 接收 `CMD_SI_PROFILE` 后读取本机 SI profile 参数 | 共享协议版本、命令号、Point0前旧时间保持和Point0边沿锁时 |
| 6. CPU2 独立流程 | Profile 测量独立于普通分布测量 | 触发、状态映射、点阵读取和完成态保持 | 实现探底频次、Point0、逐点上行、停稳后 dwell、最大 200 点；液面以上候选点不写入并停止 | 探底失败、旧底部位置、无旧底部位置、上电首次 profile |
| 7. 自动调度 | CPU3 RTC 驱动自动 profile | 根据 `40011/40010/40012/40013` 计算下一次触发；到点后直接覆盖当前普通测量并下发 SI profile | 接收覆盖后的新命令并清理当前流程残留 | 跨天周期、禁用、重启后下一次触发、冲突覆盖 |
| 8. 状态和报警 | 对外状态符合 SI 语义 | 合成 `10002/10003/10010`，保持 `10001` 现有逻辑；按 `40014~40023` 合成当前值和 profile 报警 | 产生 profile complete、counter、点阵、真实失败状态和必要偏差原始结果 | 状态位、报警置位和恢复、Point0周期事件timestamp |
| 9. 文档和联调 | 让 PLC、固件、测试资料一致 | 更新映射表、PLC 联调清单、golden frame 脚本、协议变更记录和 CPU3 流程图 | 更新 CPU2 参数和 profile 流程图 | 文档地址、倍率、默认值和代码一致 |

### 14.2 CPU3 菜单更新计划

CPU3 菜单更新已作为 SI协议对外配置入口落地，不单独维护一套显示变量。当前入口为 `参数配置 -> 测量参数 -> SI参数`，第一版常显；写入只影响 SI profile、自动调度和报警兼容逻辑。

| 菜单页 | 菜单项建议名 | 对应数据 | 单位/取值 | 实现要求 |
| --- | --- | --- | --- | --- |
| `SI Profile` | `首点` | CPU2 `si_profile_first_point` / SI `40001` | mm，按手册默认值和范围 | Point0 在底部采样并在最终点阵输出本轮实际采用的底部参考；首点是 Point1 的绝对位置；菜单写 CPU2 参数 |
| `SI Profile` | `步距` | CPU2 `si_profile_increment` / SI `40002` | mm，按手册默认值和范围 | 只影响 SI profile，不写普通分布步距 |
| `SI Profile` | `停留` | CPU2 `si_profile_dwell_time` / SI `40003` | s | 表示停稳后的等待时间；菜单写 CPU2 参数 |
| `SI Profile` | `SI探底` | CPU2 `si_profile_bottom_detect_interval` | 次，`1` 表示每次 profile 前探底 | 不映射到 SI协议手册保持寄存器；菜单写 CPU2 参数 |
| `自动 Profile` | `SI自动` | `40011 Automatic Profile Enable` | `0/1` | 关闭自动 profile 只允许通过此项 |
| `自动 Profile` | `SI周期` | `40010 Automatic Profile Interval` | min，禁止 `0` | 菜单和 Modbus 都禁止设置 `0` |
| `自动 Profile` | `SI时` | `40012 Automatic Profile Hour` | `0~23` | 与 CPU3 RTC 一起计算首次触发 |
| `自动 Profile` | `SI分` | `40013 Automatic Profile Minute` | `0~59` | 支持从起始时间按 interval 跨天运行 |
| `SI 报警` | `低密度`、`高密度` | `40014/40015` | 按 SI 映射表倍率 | CPU3 独立保存并合成 `10017/10018` 和 profile 对应报警 |
| `SI 报警` | `低温`、`高温` | `40016/40017` | 按 SI 映射表倍率 | 当前温度报警和 profile 温度上下限报警共用此限值 |
| `SI 报警` | `LL液位`、`HH液位` | `40018/40019` | mm | 对当前液位合成 LL/HH 报警 |
| `SI 报警` | `低液位`、`高液位` | `40020/40021` | mm | 对当前液位合成 low/high 报警 |
| `SI 报警` | `温差`、`密差` | `40022/40023` | 按 SI 映射表倍率 | Profile 相邻点差值报警 |
| `SI 状态` | `下次时间`、`触发源`、`完成计数` | CPU3 调度和 CPU2 回传状态 | 只读 | 可作为第二阶段诊断页，非协议必需项 |

具体代码落点建议：

1. `display_tankopera.h`：新增 CPU2 参数操作号 `COM_NUM_DEVICEPARAM_SI_PROFILE_*`，放在 CPU2 参数区并保持 CPU2/CPU3 枚举顺序一致；新增 CPU3 本机操作号 `COM_NUM_CPU3_SI_*`，放在 `COM_NUM_PARA_LOCAL_START` 到 `COM_NUM_PARA_LOCAL_STOP` 之间用于自动调度和报警限值。
2. `display_tankopera.c`：在参数配置菜单新增 `SI参数` 入口，拆分 `SI Profile`、`自动 Profile`、`SI 报警` 子页；profile 页走 CPU2 参数读写，自动和报警页走 CPU3 本机参数读写。
3. `display_tankopera.c`：在“测量命令 -> 密度分布测量”菜单新增 `SI Profile` 无参命令项，确认后调用 SI profile 共享启动入口并下发 `CMD_SI_PROFILE`；不在菜单确认时锁存时间。
4. CPU2 `system_parameter.h/c`、`dataanalysis_modbus.c`：新增 SI profile 执行参数字段、默认值、参数表、读写寄存器和版本迁移；这些字段不得复用普通分布参数。
5. CPU3 `device_param_sync.c`、内部主板 Modbus 参数读写路径：让 CPU3 菜单和 SI Modbus 写 `40001~40003` 时能同步写入 CPU2 参数并刷新本地镜像。
6. `cpu3_comm_display_params.h/c`：在 CPU3 本机参数结构中维护 `40004~40023` 对应 SI 参数组，补齐默认值、范围归一化、`Cpu3Local_ReadValue`、`Cpu3Local_WriteValueChecked`、整份运行态回滚和 FRAM 镜像迁移；自动调度相关参数写入成功后重新计算下一次触发时间。
7. `si_modbus_slave.c`：保持寄存器读写按地址分流，`40001~40003` 访问 CPU2 profile 参数，`40004~40023` 访问 CPU3 SI 参数；非法值返回 SI 异常码，本地FRAM保存失败返回设备故障 `0x06`，成功值应与菜单回读一致。
8. CPU3 调度模块：提供 `Si_Profile_Request(reason, overwrite)` 或等价入口，供 Modbus `00004 Profile`、自动调度和菜单触发共用；入口只发送CPU2启动命令，profile timestamp由后续Point0周期事件统一锁存。
9. 显示字库和排版：128x64 OLED 每行空间有限，当前短名为 `SI首点`、`SI步距`、`SI停留`、`SI探底`、`SI周期`、`SI自动`、`SI时`、`SI分` 等；修改后必须运行 `tools/check_cpu3_menu_name_width.py` 和 `LTD_DISPLAY_CPU3/font_check.py`。

### 14.3 CPU3 菜单验收项

| 验收项 | 预期 |
| --- | --- |
| 菜单入口 | 选择 SI协议后可以进入 `SI参数`；未选择 SI 时不影响其它协议 |
| 范围限制 | `40010=0`、小时大于 23、分钟大于 59、使能非 `0/1` 均不能通过菜单保存 |
| 同源读写 | PLC 写入保持寄存器后，菜单显示新值；菜单修改后，PLC 读回新值 |
| 掉电保存 | 修改 `40001~40003` 后 CPU2 重启保持；修改 `40004~40023` 后 CPU3 重启保持；菜单和保持寄存器均保持新值；任一本地参数FRAM写后校验失败时整份运行态回滚 |
| 普通分布隔离 | 修改 `首点/步距/停留` 不改变普通分布测量参数 |
| 调度重算 | 修改自动使能、周期、起始时分后，下一次自动 profile 时间立即按新配置计算 |
| 显示质量 | 中文和英文界面下无重叠、无截断到不可识别、单位和小数点显示正确 |

## 15. 版本和兼容性影响

| 改动项 | 是否影响 CPU2/CPU3 共享协议 | 是否影响参数存储 | 说明 |
| --- | --- | --- | --- |
| 新增 `CMD_SI_PROFILE` | 是 | 否 | 新增共享命令，必须升级 `DEVICE_PROTOCOL_VERSION` |
| CPU3 `00004` 改下发新命令 | 是 | 否 | 命令语义变化，需更新映射表和 golden frame |
| 新增 CPU2 SI profile 执行参数 | 是 | CPU2 参数存储 | 复用原预留字段，结构大小和 `DEVICE_PARAM_VERSION` 保持不变；协议版本迁移补默认值并同步 CPU2/CPU3 参数枚举 |
| `10001/10010` 状态口径调整 | 否 | 否 | `10001` 保持现有逻辑；`10010` 由 CPU3 SI 层固定输出可信 |
| `10002/10003` 状态合成 | 否 | 否 | 第一版只按 CPU3 已有状态合成 |
| `40001~40003` 独立保持寄存器 | 是 | CPU2 参数存储 | CPU3 对外暴露 SI 地址，底层桥接 CPU2 profile 执行参数 |
| `40004~40023` CPU3本地保持寄存器 | 否 | CPU3 参数存储 | `40004~40009`新增原始兼容槽，`40010~40023`保留既有自动调度和报警语义；FRAM `V6 -> V7`兼容迁移 |
| 自动 profile 调度 | 是 | CPU3 参数存储 | CPU3 调度本身在 CPU3；打断当前测量并下发新命令会改变命令/流程契约 |
| 报警限值全部兼容 | 否 | CPU3 参数存储 | CPU3 独立保存限值并合成报警；如后续新增 CPU2 偏差结果再评估共享协议 |
| Point0时间戳 | 是 | 否 | 协议18以共享 `cycle_counter`变化作为Point0事件；首次上线/重连无法重建时为0 |
| SI profile 结果来源隔离 | 是 | 否 | 协议版本 14 新增 `profile_source`，CPU3 SI协议 只认 `PROFILE_SOURCE_SI` |
| SI 温度无效值和报警阈值口径 | 否 | CPU3 参数 signedness | 温度无效值输出 `0xB1E0`，报警阈值 `0` 是有效值，`40016/40017` 允许负温度 |
| SI Profile生命周期和最终发布 | 是 | 否 | 协议18追加阶段、周期和有效点进度；CPU3复核候选并同代发布Complete/N/点阵/Profile报警 |

SI生命周期扩展已在协议18完成，首个正式匹配固件组合为CPU2 `V1.26.0.0` / CPU3 `V1.24.0.0`。协议18属于跨CPU新功能，双端必须成对升级；CHANGELOG、协议变更记录和版本测试资料已经同步。当前整机协议25同样严格配套；协议19至25的后续共享协议变化均不改变本节SI实现。真实硬件验收与协议18提交前审查遗留见 `SI协议待确认与后续清单.md` 第6、7节。

## 16. 验证计划

已执行静态检查：

- `py tools\check_si_modbus_frames.py`：通过。
- `py tools\check_si_protocol_contract.py`：通过。
- CPU2、CPU3 clean-first目标构建：通过；提交前需在最终升版源码上复跑。
- 如果新增命令、共享状态或 CPU2 参数字段，扩展检查脚本覆盖命令号、字段顺序、单位、范围和协议版本。
- CPU2 `DEVICE_PARAM_VERSION`保持3；协议18验证重点是CPU2/CPU3三项生命周期字段和寄存器顺序一致、Point0提交顺序、候选分块/代际复核、V6→V7迁移布局及SI来源隔离。

协议帧验证：

| 项目 | 验证点 |
| --- | --- |
| `10001` | 保持现有映射逻辑，不作为固定值验证 |
| `10010` | 任意状态读取都为 `0` |
| `10002/10003` | 液位跟随时 `10002=1,10003=0`；其它状态 `10002=1,10003=1` |
| `40001~40003` | 写入后可回读，CPU2 参数镜像一致，且不改变普通分布测量参数 |
| `40004~40009` | 默认 `0、50、1000、0、5、1`；FC03完整读取和六项FC06写入正确；重启保持，FRAM失败时整份 `40004~40023` 运行态回滚并返回FC06 `0x06` |
| `40010~40023` FRAM事务 | 任一合法FC06写入只有在FRAM写后读回校验通过时成功；故障注入时返回 `0x06`，当前写值以及同一结构内其它参数均保持写前值 |
| `40010~40013` | 合法值可写可回读；`40010=0`、非法小时/分钟/使能值返回异常 |
| 自动 profile | 到点后下发 SI profile；正在其它测量或已有命令时直接覆盖命令并执行；同一分钟不重复触发 |
| `30007~30010` | Point0周期事件时锁存；RTC失败、首次上线或重连无法重建时四项全0 |
| `30014~30016` | 与同一时刻FC01/FC02前32位逐位一致；`30017~30020`仍为0 |
| 报警限值 | 当前值报警、profile 上下限报警、profile 偏差报警均按限值置位和恢复 |
| 报警阈值 0 | `0` 作为有效阈值参与比较，不能再作为禁用条件 |
| Manual 官方语义 | Manual 下 SI 报警是否清零、自动 profile 是否跳过、`30004` 是否只在 `10013=1` 时作为当前液位使用 |
| Cal 回 Auto | SI Profile 成功后已自动排队找液位；`00002 Calibrate` 仍只是零点标定，需确认是否改成完整 Level Calibration 并自动回液位跟随 |
| `10010` 可信性 | 在底部参考无效、探底失败或维护后状态下，确认 `10010=0` 是否仅作为兼容固定值使用 |
| 负温度阈值 | `40016/40017` 写入 int16 补码后，菜单和回读均保持负值语义 |
| 温度无效值 | 当前温度和 profile 点温度无效时输出 `-200.00°C` / `0xB1E0` |
| SI profile 隔离 | 已发布SI快照不被普通分布、国标、每米、间隔或 Wärtsilä 分布改写；这些流程不置SI Profile运行态，也不清除或替换既有 `10005/30006/时间/点阵/Profile报警`，只有下一轮SI Point0才使旧快照失效 |
| Tank ID | 复用 `SlaveAddress` |
| Manual 速度档位 | Slow/Medium/Fast 不产生真实速度差异，只按方向命令处理 |

测量流程验证：

1. 上电后首次 profile 必须探底，然后采集 Point 0。
2. 探底频次为 `1`：每次 profile 前探底，然后采集 Point 0。
3. 探底频次为 `N`：按进入 CPU2 SI profile 入口的触发次数统计，前 `N-1` 次不探底，第 `N` 次探底；第 `N` 次触发计为新一轮第 1 次，不按 profile 成功完成次数统计。
4. 不探底时使用上次探底保存的底部位置运行到底部测量位置，不能刷新内部底部参考。
5. 没有有效底部位置时必须强制探底；探底失败且仍无旧底部位置时使用当前位置作为底部采样运动目标并继续 profile，但不得刷新底部参考。
6. 最终第0点写入 `30021~30023`，位置为本轮实际采用的底部参考；Point0后活动期三项保持0，直到最终同代发布。
7. `40001` 对应 Point1 的绝对位置，不能覆盖 Point0；Point1 对外位置固定等于 `40001`，Point2 等于 `40001 + 40002`，PointN 等于 `40001 + (N-1) * 40002`。
8. 即使实际底部位置高于 `40001`，仍按 `40001` 作为第一个可编程测点运行和输出，不跳到第一个高于底部的步进点。
9. profile 最多输出 200 点；profile 前不单独找液位，候选点运行中判定液面以上后停止，不采集该液面以上点；达到上限或点数上限也停止。
10. Point0前取消保留上一轮结果；Point0后取消/失败清Complete、N和点阵且不恢复旧Complete，均不递增完成计数。
11. Point0后N从1按有效液体点单调递增，空气点、失败样本和重试不计数；最终分别覆盖1点、约30点和200点。
12. 采点完成进入RETURNING_LEVEL期间，Profile=1、Complete=0、N冻结、点阵全0；回液位稳定且CPU3候选复核通过后，最终模式、Complete、N、点阵和Profile报警同代发布。
13. CPU3分别在Point0前、Point0后、点阵分块读取中重启；不得误置Complete或混合点阵，无法重建Point0时间时四项为0。
14. CPU3冷启动或重连时若CPU2已处于同代SI COMPLETE，可恢复旧SI最终N、点阵和报警；因未观察到Point0事件，`30007~30010`必须为0，不得使用启动时刻。
15. SI最终快照发布后依次执行普通分布、国标、每米、间隔和Wärtsilä测量；旧SI快照保持不变，旧SI终态也不得把当前非SI动作强制投影为Stop。

尚未执行的真实硬件验证：RS485站2主机完整两轮、协议17/18混搭拦截、V6真实FRAM镜像迁移、`40004~40023`掉电保存与FRAM故障注入、取消/失败/回液位故障注入、CPU3冷启动/重连、普通分布与已发布SI快照隔离、CPU2通信中断以及1/30/200点边界。

## 17. 文档同步清单

本轮实现需要同步更新：

- `docs/01_协议与寄存器/SI协议适配/02_协议映射/SI协议兼容映射表.md`
- `docs/01_协议与寄存器/SI协议适配/04_联调测试/SI协议_PLC联调检查表.md`
- `docs/01_协议与寄存器/SI协议适配/03_改动记录/SI协议适配改动整理.md`
- `docs/01_协议与寄存器/CPU2_CPU3协议变更记录.md`
- 如果 CPU2 profile 流程改变，更新对应 CPU2 程序流程 HTML/SVG。
- 如果 CPU3 自动调度或 SI 映射流程改变，更新对应 CPU3 程序流程 HTML/SVG。
- 本轮未升级 CPU2 `DEVICE_PARAM_VERSION`；若后续不再复用预留槽而改变结构大小，再更新参数存储升级清单、版本改动与测试方案和 `CHANGELOG.md`。

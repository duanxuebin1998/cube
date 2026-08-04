# DSM 传感器协议

更新日期：2026-08-03

本目录维护 CUBE CPU2 与 DSM CPU1 传感器之间的传统 UART 文本协议资料。该链路由 CPU2 UART6 使用，当前串口参数为 9600 bit/s、8 数据位、无校验、1 停止位。

当前实现差异、风险等级和关闭条件见 [DSM 传感器协议适配复查](../../../03_问题分析与整改/未处理/2026-07-18_DSM传感器协议适配复查.md)。该报告已于 2026-07-21 按用户决定收敛 CUBE-only 边界，并于 2026-08-03 按 `MAIN@66d5c062bf672e1d2e49ff78f1c770c6e33f98dd` 重新核对；具体落地设计见 [DSM 版本化定长事务层详细方案](../../../02_需求与计划/未实现/2026-07-21_DSM版本化定长事务层详细方案.md)。两份文档仍是方案和静态证据，不代表固件已经整改或完成真实 DSM 台架验证。

本目录与 `../../DSM协议适配/` 的职责不同：

- 本目录：CPU2 与 DSM 传感器之间的 `C + 单字符命令` 协议。
- `DSM协议适配/`：CPU3 面向外部主机兼容的一代 DSM Modbus V1.228 协议。

## 文件职责

| 文件 | 维护方式 | 用途 |
| --- | --- | --- |
| [DSM_CPU1对外通信协议.md](DSM_CPU1对外通信协议.md) | 自动同步，禁止在 CUBE 中直接修改 | DSM CPU1 传感器端协议完整正文，唯一真源位于 `D:\keil_workspace\DSM_CPU1_SIL\docs\01_协议与接口\DSM_CPU1对外通信协议.md` |
| [protocol/README.md](protocol/README.md) | 自动同步，禁止在 CUBE 中直接修改 | 机器数据入口和维护命令 |
| [protocol/dsm_protocol_matrix.json](protocol/dsm_protocol_matrix.json) | 自动同步，禁止在 CUBE 中直接修改 | 机器可读的物理层、命令、响应 profile、版本标识和 15 个版本/快照完整命令能力 |
| [protocol/golden_frames/README.md](protocol/golden_frames/README.md) | 自动同步，禁止在 CUBE 中直接修改 | golden frame 证据类型和使用边界 |
| [protocol/golden_frames/dsm_protocol_golden_frames.json](protocol/golden_frames/dsm_protocol_golden_frames.json) | 自动同步，禁止在 CUBE 中直接修改 | 20 组确定性正常帧、错误帧、版本帧、BCC 控制字符边界、扫频帧和复位无响应契约向量 |
| [DSM_CPU1对外通信协议.sync.json](DSM_CPU1对外通信协议.sync.json) | 自动生成 | 分别记录五个同步制品的 SHA-256、Git blob、源工作区状态和 CUBE 目标路径 |
| `README.md` | CUBE 人工维护 | 记录 CUBE 当前消费范围、实现差异、同步方法和验证边界 |

协议同步成功只证明 CUBE 中的协议副本与 DSM 源仓库一致，不代表 CUBE CPU2 已自动兼容所有上游命令和行为。

## 唯一真源和同步入口

唯一允许直接修改的协议正文：

```text
D:\keil_workspace\DSM_CPU1_SIL\docs\01_协议与接口\DSM_CPU1对外通信协议.md
```

同步到 CUBE：

```powershell
cd D:\keil_workspace\DSM_CPU1_SIL
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\sync_dsm_sensor_protocol_to_cube.ps1 -CubeRoot D:\CUBE -Sync
```

只检查一致性、不写文件：

```powershell
cd D:\keil_workspace\DSM_CPU1_SIL
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\sync_dsm_sensor_protocol_to_cube.ps1 -CubeRoot D:\CUBE -Check
```

同步脚本先执行 DSM 源仓库协议契约检查，再更新协议 Markdown、机器矩阵、golden frames 和 `.sync.json`；不修改 CUBE 固件、不升版、不暂存、不提交。

## CUBE 当前实现

当前程序基线：CPU2 `V1.36.2.0`／CPU3 `V1.36.0.0`，`DEVICE_PROTOCOL_VERSION=31`。DSM 专用适配从原方案形成后没有版本化定长事务功能落地，下面的“当前状态”仍对应最新程序；期间的注释完善、读取部件参数清理和液位标定AO归一化不构成该整改实现。

实现入口：

- `LTD_MAIN_CPU2/Core/Src/usart.c`：UART6 9600-8-N-1。
- `LTD_MAIN_CPU2/Services/Sensor/dsm_sensor_communication.h`：DSM 命令常量和公开接口。
- `LTD_MAIN_CPU2/Services/Sensor/dsm_sensor_communication.c`：发送、DMA 接收、重试、BCC、错误映射和业务字段解析。
- `LTD_MAIN_CPU2/Services/Sensor/sensor.c`：多传感器识别和 DSM/V2/安全协议分发。

### 已有业务调用

| 命令 | CUBE 用途 | 当前状态 |
| --- | --- | --- |
| `CD` | 切换 DSM 密度模式 | 已实现 |
| `Cd` | 读取频率、密度、温度 | 已实现 |
| `Cl` | 读取水测量电容 | 已实现；检查 11 字节、BCC 和 CRLF；`<30.0 pF` 改写为 `99999.9 pF` 已由产品确认是特殊处理，后续事务层整改必须保持该行为 |
| `CB` | 切换 DSM 液位模式 | 已实现 |
| `Cb` | 读取液位频率 | 已实现 |
| `CN` | 读取传感器/振动管编号 | 已实现 |
| `Ch` | 读取陀螺仪 X/Y 角度 | 已实现 |

### 尚未形成当前业务调用

| 命令 | 上游含义 | CUBE 当前状态 |
| --- | --- | --- |
| `CL` | 执行历史测水探头准备动作 | 接口已实现，当前主流程未找到调用点；上游 PA5/SCL 语义存在已知风险 |
| `CK` | 读取 DSM 供电电压 | 接口已实现，当前主流程未找到调用点；正常小写 `e` 仍会被日志误报为低压 |
| `CF`、`CP`、`CA`、`CH`、`CZ` | 兼容或准备命令 | 未形成当前业务调用 |
| `Cf` | 读取频率、密度、温度 | CUBE 当前使用 `Cd`，未使用 `Cf` |
| `Cz` | 读取零点电压 | 未形成当前业务调用 |
| `CV` | 读取组合版本 | 头文件保留宏，当前未形成业务调用 |
| `Cv` | 读取 DSM CPU1 本机版本 | 未形成当前业务调用 |
| `CM` | 读取密度分析参数 | 头文件保留宏，当前未形成业务调用 |
| `Ca` | 读取液位频率 | CUBE 当前使用 `Cb`，未使用 `Ca` |
| `Cp` | 读取液位 AD 缓存 | 未形成当前业务调用 |
| `CR` | DSM CPU1 软件复位 | 未形成当前业务调用 |
| `CX` | V5.4 PB4 电源域主动掉电 | 未形成当前业务调用；属于 V5.4 可选命令，未识别 `Cv=05.40` 前禁止发送 |
| `CS` | 扫频 | DSM V5.2～V5.4 已停用，CUBE 不应发送 |

## 当前适配边界

- CPU2 后续加入的统一 UART DMA/IDLE 恢复队列只管理 USART1、USART2、UART4、UART5，明确不接管 USART6。DSM UART6 仍由 DSM／LTD／安全传感器／CH9141 等业务协议模块自行管理，因此其它串口恢复治理不代表 DSM 事务层已经实现。
- CUBE 当前通用接收路径遇到缓冲区中的第一个 `LF` 就结束，不是按命令预期长度收帧。无 CRLF 的 6 字节 `CV/Cv` 会超时，BCC 本身为 `0x0A` 的 11 字节边界向量会被提前截断；多数命令也没有统一严格校验长度、固定字段和 CRLF。
- 通信层只对超时、UART、BCC 和已映射错误帧执行有界重试；ACK 或业务字段在上层解析失败时不会自动重发，启动主动 ACK 或错配帧可能结束本次调用。
- CPU2 未读取 `Cv/CV`，也没有通过配置绑定传感器维护线；现有实现按 V5.4 当前规范对照，但不能可靠区分 V4.x、V5.x、V6.x 或 V5.0-Lemis。
- `g_deviceParams.sensorSoftwareVersion` 是既有持久化通用设备参数，当前只有默认赋值和 Modbus 同步，没有由 DSM 探测自动更新；本批版本识别继续规划为 RAM-only 上下文，不直接复用该字段。
- V5.4 的 CUBE host profile 与当前 `Ch` 参数静态一致：500 ms 单次等待、最多 3 次、重试前 300 ms、每次尝试前清理旧帧。冷唤醒首次 500 ms 允许超时；最新协议还明确本次取角失败会返回 19 字节错误帧。CPU2 会把首轮超时或错误帧记入重试日志，形成 `DSM-CUBE-09` 诊断噪声；迟到帧清理和第二次成功时限仍待台架证明。
- `CD`、`CB` 的 `%\r\n` 只代表 DSM CPU1 已处理命令，不证明其内部 CPU0 已完成模式切换；上游还存在正常响应后空指针写风险。
- `Cb` 的上游实现可能掩盖本次 CPU0 通信失败；CUBE 收到语法正确的频率帧时无法仅凭帧内容确认本次内部读取成功。
- 发送前 `UART6_DrainRX_UntilIdle(5)` 只有连续空闲条件，没有固定总时限或命令切换出口；持续噪声或残流可能长期阻塞。
- 上游 V5.2～V5.4 已停用 `CS`，因此 `Cp` 使用的扫频缓存可能为初值或旧值；正温度和液位 AD 帧还存在字段超宽后被固定 BCC 下标覆盖的历史行为。
- 上游首字符大写 `E` 表示供电过低或过高；正常 `CK` 固定以小写 `e` 开头。当前公共日志把 `E/e` 都描述为“电压过低”，不能作为供电方向或数据质量的可靠证据。
- 当前结论来自源码静态对照，尚未用真实 DSM V5.4 传感器完成全命令抓包、PB4 时序、控制字符 BCC、CPU0 故障、持续噪声和边界值验证。

## 上游协议更新后的处理规则

1. 在 DSM 源仓库同步修改固件行为、机器矩阵、golden frames 和协议正文。
2. 运行 `py tools\check_dsm_protocol_contract.py --write-doc` 更新协议正文中的完整版本命令表。
3. 运行 `py tools\check_dsm_protocol_contract.py`，确认归档源码、矩阵、生成表和 golden frames 一致。
4. 运行同步脚本 `-Sync`，再运行 `-Check`，确认五个制品的 SHA-256、Git blob 和 manifest 一致。
5. 对照本 README 的“已有业务调用”检查受影响命令。
6. 如果只改说明文字，CUBE 只同步协议交付物，不提升 CPU2/CPU3 固件版本。
7. 如果修改已用命令的帧长、BCC、字段、单位、异常或 ACK 语义，必须同步检查 CPU2 解析、调用点、错误映射和台架用例。
8. 只有 CPU2 固件行为实际变化时，才按 CUBE 发布流程处理 CPU2 版本、`CHANGELOG.md` 和改动与测试方案。
9. 只有 CPU2/CPU3 共享契约变化时，才提升 `DEVICE_PROTOCOL_VERSION`；单纯 DSM 传感器协议同步不提升共享协议版本。

## 验证建议

- 每次同步至少执行 DSM 协议契约检查和同步脚本 `-Check`。
- golden frames 当前属于静态契约向量；其中 `synthetic` 样例不等同于真实目标板抓包。
- 在 CUBE 中运行 Markdown 链接和结构检查。
- 上游改变已用命令时，增加或更新对应的协议契约/主机测试，并重新构建 CPU2。
- 真实硬件验证应覆盖正常帧、控制字符 BCC、低压 `E`、19 字节错误帧、模式切换、通信超时和字段超宽。

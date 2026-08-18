# DSM 传感器协议

更新日期：2026-08-17

本目录维护 CUBE CPU2 与 DSM CPU1 传感器之间的传统 UART 文本协议资料。该链路由 CPU2 UART6 使用，当前串口参数为 9600 bit/s、8 数据位、无校验、1 停止位。

当前实现差异、风险等级和关闭条件见 [DSM 传感器协议适配复查](../../../03_问题分析与整改/未处理/2026-07-18_DSM传感器协议适配复查.md)。该报告于2026-08-17按`MAIN@056c7913dfda`、CPU2/CPU3 V1.40.0.0和共享协议34复核；具体实现设计与剩余验收入口见 [DSM 版本化定长事务层详细方案](../../../02_需求与计划/未实现/2026-07-21_DSM版本化定长事务层详细方案.md)。CPU2侧核心整改已经发布，但静态检查和构建记录仍不能替代真实DSM台架、故障注入和多维护线验证。

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

当前程序基线：CPU2 `V1.40.0.0`／CPU3 `V1.40.0.0`，`DEVICE_PROTOCOL_VERSION=34`。DSM命令级定长事务和严格帧校验自CPU2 V1.36.4.0起发布，`Cv/CV` RAM版本上下文、CK供电电压和CM密度分析参数自CPU2 V1.38.0.0起进入生产路径；后续协议33/34未改变DSM UART6帧形和业务语义。

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
| `Cv/CV` | 读取DSM CPU1/组合版本 | 已接入生产探测链，建立RAM-only版本上下文并按V3/V4/V5/V6及歧义版本分类 |
| `CN` | 读取传感器/振动管编号 | 已实现 |
| `Ch` | 读取陀螺仪 X/Y 角度 | 已实现 |
| `CK` | 读取 DSM 供电电压 | 已接入读取部件参数，使用独立有效位；失败不阻断其它已成功字段输出 |
| `CM` | 读取密度分析参数 | 已接入读取部件参数，使用独立有效位并支持19字节错误帧 |

### 尚未形成当前业务调用

| 命令 | 上游含义 | CUBE 当前状态 |
| --- | --- | --- |
| `CL` | 执行历史测水探头准备动作 | 接口已实现，当前主流程未找到调用点；上游 PA5/SCL 语义存在已知风险 |
| `CF`、`CP`、`CA`、`CH`、`CZ` | 兼容或准备命令 | 未形成当前业务调用 |
| `Cf` | 读取频率、密度、温度 | CUBE 当前使用 `Cd`，未使用 `Cf` |
| `Cz` | 读取零点电压 | 未形成当前业务调用 |
| `Ca` | 读取液位频率 | CUBE 当前使用 `Cb`，未使用 `Ca` |
| `Cp` | 读取液位 AD 缓存 | 未形成当前业务调用 |
| `CR` | DSM CPU1 软件复位 | 未形成当前业务调用 |
| `CX` | V5.4 PB4 电源域主动掉电 | 未形成当前业务调用；属于 V5.4 可选命令，未识别 `Cv=05.40` 前禁止发送 |
| `CS` | 扫频 | DSM V5.2～V5.4 已停用，CUBE 不应发送 |

## 当前适配边界

- `DsmCommandFrameSpec`按命令定义3、6、11、19和27字节响应形状；接收按精确长度完成，严格校验`%\r\n` ACK、BCC、CRLF、固定字段和数字格式，业务解析失败也进入同一有界重试。BCC为`0x0A`或`0x0D`不再被误当作提前结束符。
- 发送前drain具备100 ms总时限、命令切换和UART错误出口；CPU2统一UART恢复队列仍不接管USART6，DSM／LTD／安全传感器／CH9141等业务模块继续共同维护UART6所有权。
- `Cv`优先探测CPU1版本，失败时回退`CV`；运行期上下文区分V3、V4、V5、V6、V5.0歧义和`CV07-ambiguous`。`g_deviceParams.sensorSoftwareVersion`仍是独立持久参数，DSM探测不写FRAM。
- CK和CM结果使用独立有效位，读取部件参数允许部分成功；正常小写`e`不再误报低压。真实CK／CM响应、错误帧和部分成功组合仍需台架覆盖。
- V5.4 `Ch`冷唤醒首轮500 ms允许超时，取角失败可返回19字节错误帧；迟到帧清理、第二次成功时限和重试日志噪声仍待真实设备验证。
- `CD`、`CB` 的`%\r\n`只证明DSM CPU1返回了正确ACK帧，不证明内部CPU0模式切换已经完成；上游正常响应后的空指针风险也不由CUBE严格校验消除。
- `Cb`上游实现可能在本次CPU0通信失败后返回旧值，CUBE无法仅凭语法正确的频率帧证明数据新鲜。
- 上游V5.2～V5.4已停用`CS`，`Cp`扫频缓存可能为初值或旧值；`CL`尚无主流程调用，`CX`保持未启用。
- `<30.0 pF -> 99999.9 pF`是已确认产品特殊处理，必须保留；它不代表真实低电容台架已经验收。
- 当前结论来自源码、版本记录和静态契约，尚未用真实DSM V5.4传感器完成全命令抓包、PB4时序、控制字符BCC、CPU0故障、持续噪声和边界值验证。

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

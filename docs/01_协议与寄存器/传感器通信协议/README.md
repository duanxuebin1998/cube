# 传感器通信协议资料索引

更新日期：2026-07-18

本目录保存 CPU2 与传感器之间的内部通信协议资料，包括现有 DSM/LTD/V2 通信内容参考、新一代安全通信协议卷、SIL 设计理由和验证证据映射。

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [DSM传感器协议/README.md](DSM传感器协议/README.md) | CPU2 UART6 与 DSM CPU1 传感器之间的传统 `C + 单字符命令` 协议入口，包含上游自动同步副本、来源清单、CUBE 当前命令子集和适配边界 |
| [传感器新一代安全通信协议卷.md](传感器新一代安全通信协议卷.md) | 冻结的 V1 维护正文，覆盖 SRS/SIL 声明边界、HELLO 挑战、固定 44 字节安全快报、配置摘要、安全反应、标准主题映射和验证矩阵 |
| [传感器新一代安全通信协议落地方案.md](传感器新一代安全通信协议落地方案.md) | 面向 CPU2、传感器端、测试工具和安全论证证据的长期执行方案，包含 P0 安全需求门禁、文件拆分、任务顺序、验证计划和风险控制 |
| [传感器新一代安全通信协议验证记录.md](传感器新一代安全通信协议验证记录.md) | 区分已执行 CPU2 主机验证和未执行真实硬件验证，登记命令、覆盖范围、构建结果及证据保留要求 |
| [传感器安全通信残余危险失效率论证.md](传感器安全通信残余危险失效率论证.md) | 定义 CRC32C、非 CRC 诊断、误码模型、PFH 计算输入和定量关闭条件；当前明确保持未关闭 |
| [sensor_safe_protocol_v1.json](golden_frames/sensor_safe_protocol_v1.json) | V1 控制帧、44 字节快报、连续序号和异常注入的确定性 golden vectors |
| [sensor_safe_param_digest_v1.json](golden_frames/sensor_safe_param_digest_v1.json) | `param_crc` 与 `safety_param_crc` 规范化序列化和字段顺序向量 |

### 新一代安全协议检查

`py tools\check_sensor_safe_protocol_contract.py` 校验线上字节契约，`py tools\check_sensor_safe_cpu2_integration.py` 校验 19 个冻结命令、115 项服务校验、完整参数表适配层公开入口、CPU2 探测回退和业务分发契约，`py tools\check_sensor_safe_host_tests.py` 统一编译运行七组主机测试及四项 GCC 路径分析。三者覆盖范围不同，均通过才可声明当前源码与协议卷一致。

### 传统 DSM 协议检查

传统 DSM 协议的契约工具位于唯一真源仓库 `D:\keil_workspace\DSM_CPU1_SIL`。先在该目录运行 `py -X utf8 tools\check_dsm_protocol_contract.py`，再运行 `powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\sync_dsm_sensor_protocol_to_cube.ps1 -CubeRoot D:\CUBE -Check`。两项通过只证明上游协议制品自洽且 CUBE 副本哈希一致，不能证明 CPU2 已兼容全部维护线或真实 UART/传感器行为；当前差异见 [2026-07-18 DSM 传感器协议适配复查](../../03_问题分析与整改/未处理/2026-07-18_DSM传感器协议适配复查.md)。

## 维护口径

- 本目录只维护 CPU2 与传感器之间的内部协议，不维护 CPU2/CPU3 共享寄存器协议。
- DSM CPU1 传统串口协议正文以 `D:\keil_workspace\DSM_CPU1_SIL\docs\01_协议与接口\DSM_CPU1对外通信协议.md` 为唯一真源；CUBE 副本通过源仓库同步脚本更新，不在本目录直接修改。
- 若新协议后续暴露到 CPU3、外部 Modbus、SI7000 或 Wartsila 协议，应同步更新对应协议目录和 `../CPU2_CPU3协议变更记录.md`。
- 当前 Markdown 为维护正文；同名 PDF 如存在，仅作为阅读导出件，不作为权威正文。
- 标准名、协议字段名和命令名可保留英文；项目解释和执行计划优先使用中文。

# 传感器通信协议资料索引

更新日期：2026-08-03

本目录保存 CPU2 与传感器之间的内部通信协议资料，包括现有 DSM/LTD/V2 通信内容参考、新一代安全通信协议卷、SIL 设计理由和验证证据映射。

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [DSM传感器协议/README.md](DSM传感器协议/README.md) | CPU2 UART6 与 DSM CPU1 传感器之间的传统 `C + 单字符命令` 协议入口，包含上游自动同步副本、来源清单、CUBE 当前命令子集和适配边界 |
| [Newhall称重传感器协议/README.md](Newhall称重传感器协议/README.md) | 独立 MSP430FR2476 称重主控的 `19200 8N1` UART 协议归档，包含当前 `0x83` 全量帧、未启用 `0x81/0x82` 组包、下行查询、BCC、字段格式和联调边界；CUBE协议31已实现CPU2被动接收`0x83`重量/温度帧并由CPU3显示，真实串口和整机联调仍未完成 |
| [传感器安全协议SIL/README.md](传感器安全协议SIL/README.md) | 新一代安全通信协议的集中维护入口，包含 V1 协议正文、双功能传感器同总线优化设计、落地方案、验证记录和残余危险失效率论证 |
| [sensor_safe_protocol_v1.json](golden_frames/sensor_safe_protocol_v1.json) | V1 控制帧、44 字节快报、连续序号和异常注入的确定性 golden vectors |
| [sensor_safe_param_digest_v1.json](golden_frames/sensor_safe_param_digest_v1.json) | `param_crc` 与 `safety_param_crc` 规范化序列化和字段顺序向量 |

### 新一代安全协议检查

`py tools\check_sensor_safe_protocol_contract.py` 校验线上字节契约，`py tools\check_sensor_safe_cpu2_integration.py` 校验 19 个冻结命令、115 项服务校验、完整参数表适配层公开入口、CPU2 探测回退和业务分发契约，`py tools\check_sensor_safe_host_tests.py` 统一编译运行七组主机测试及四项 GCC 路径分析。三者覆盖范围不同，均通过才可声明当前源码与协议卷一致。

### 传统 DSM 协议检查

传统 DSM 协议的契约工具位于唯一真源仓库 `D:\keil_workspace\DSM_CPU1_SIL`。先在该目录运行 `py -X utf8 tools\check_dsm_protocol_contract.py`，再运行 `powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\sync_dsm_sensor_protocol_to_cube.ps1 -CubeRoot D:\CUBE -Check`。两项通过只证明上游协议制品自洽且 CUBE 副本哈希一致，不能证明 CPU2 已兼容全部维护线或真实 UART/传感器行为；当前差异见 [2026-07-18 DSM 传感器协议适配复查](../../03_问题分析与整改/未处理/2026-07-18_DSM传感器协议适配复查.md)。

## 维护口径

- 本目录只维护 CPU2 与传感器之间的内部协议，不维护 CPU2/CPU3 共享寄存器协议。
- Newhall 协议当前作为独立称重主控参考资料归档；只有后续完成 CUBE 端口、解析、故障和新鲜度适配后，才能改写为 CUBE 当前实现。
- DSM CPU1 传统串口协议正文以 `D:\keil_workspace\DSM_CPU1_SIL\docs\01_协议与接口\DSM_CPU1对外通信协议.md` 为唯一真源；CUBE 副本通过源仓库同步脚本更新，不在本目录直接修改。
- 若新协议后续暴露到 CPU3、外部 Modbus、SI7000 或 Wartsila 协议，应同步更新对应协议目录和 `../CPU2_CPU3协议变更记录.md`。
- `传感器安全协议SIL/` 是新一代安全通信协议的正式维护入口；当前 Markdown 为维护正文，同名 PDF 是阅读导出件，不作为权威正文。
- 标准名、协议字段名和命令名可保留英文；项目解释和执行计划优先使用中文。

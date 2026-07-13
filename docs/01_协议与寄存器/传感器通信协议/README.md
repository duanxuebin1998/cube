# 传感器通信协议资料索引

更新日期：2026-07-12

本目录保存 CPU2 与传感器之间的内部通信协议资料，包括现有 DSM/LTD/V2 通信内容参考、新一代安全通信协议卷、SIL 设计理由和验证证据映射。

## 文档索引

| 文档 | 内容 |
| --- | --- |
| [传感器新一代安全通信协议卷.md](传感器新一代安全通信协议卷.md) | 冻结的 V1 维护正文，覆盖 SRS/SIL 声明边界、HELLO 挑战、固定 44 字节安全快报、配置摘要、安全反应、标准主题映射和验证矩阵 |
| [传感器新一代安全通信协议落地方案.md](传感器新一代安全通信协议落地方案.md) | 面向 CPU2、传感器端、测试工具和安全论证证据的长期执行方案，包含 P0 安全需求门禁、文件拆分、任务顺序、验证计划和风险控制 |
| [传感器新一代安全通信协议验证记录.md](传感器新一代安全通信协议验证记录.md) | 区分已执行 CPU2 主机验证和未执行真实硬件验证，登记命令、覆盖范围、构建结果及证据保留要求 |
| [传感器安全通信残余危险失效率论证.md](传感器安全通信残余危险失效率论证.md) | 定义 CRC32C、非 CRC 诊断、误码模型、PFH 计算输入和定量关闭条件；当前明确保持未关闭 |
| [sensor_safe_protocol_v1.json](golden_frames/sensor_safe_protocol_v1.json) | V1 控制帧、44 字节快报、连续序号和异常注入的确定性 golden vectors |
| [sensor_safe_param_digest_v1.json](golden_frames/sensor_safe_param_digest_v1.json) | `param_crc` 与 `safety_param_crc` 规范化序列化和字段顺序向量 |

配套检查入口：`py tools\check_sensor_safe_protocol_contract.py` 校验线上字节契约，`py tools\check_sensor_safe_cpu2_integration.py` 校验 19 个冻结命令、115 项服务校验、完整参数表适配层公开入口、CPU2 探测回退和业务分发契约，`py tools\check_sensor_safe_host_tests.py` 统一编译运行七组主机测试及四项 GCC 路径分析。三者覆盖范围不同，均通过才可声明当前源码与协议卷一致。

## 维护口径

- 本目录只维护 CPU2 与传感器之间的内部协议，不维护 CPU2/CPU3 共享寄存器协议。
- 若新协议后续暴露到 CPU3、外部 Modbus、SI7000 或 Wartsila 协议，应同步更新对应协议目录和 `../CPU2_CPU3协议变更记录.md`。
- 当前 Markdown 为维护正文；同名 PDF 如存在，仅作为阅读导出件，不作为权威正文。
- 标准名、协议字段名和命令名可保留英文；项目解释和执行计划优先使用中文。

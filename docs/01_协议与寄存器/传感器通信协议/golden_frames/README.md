# 传感器安全通信 Golden Frames

本目录保存传感器新一代安全通信协议的确定性测试向量，供协议实现、跨 CPU 联调和自动门禁共同使用。JSON 文件只记录可复算的字节序列、CRC 与预期结果，不登记真实设备验证结论。

## 文件

| 文件 | 用途 |
| --- | --- |
| `sensor_safe_protocol_v1.json` | 线缆常量、字段偏移、命令/模式/标志/状态/诊断/能力/错误码，以及控制帧、快报帧、CRC-32C 与非法帧的固定向量 |
| `sensor_safe_param_digest_v1.json` | 安全参数规范化字节流和摘要 CRC 向量 |

## 生成与校验

```powershell
py tools\sensor_safe_protocol_vectors.py --write docs\01_协议与寄存器\传感器通信协议\golden_frames\sensor_safe_protocol_v1.json --write-param-digest docs\01_协议与寄存器\传感器通信协议\golden_frames\sensor_safe_param_digest_v1.json
py tools\check_sensor_safe_protocol_contract.py
```

向量生成必须保持确定性；需要修改协议字段或 CRC 覆盖范围时，应先更新正式协议说明和生成器，再重新生成并通过契约检查，禁止手工只改 JSON 结果。契约检查器还会解析 CPU2 `sensor_safe_types.h`，要求 C 侧帧常量、字段偏移、枚举和位掩码与本 JSON 完全一致。

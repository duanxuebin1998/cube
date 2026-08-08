# 现行安全通信协议 V1

本目录保存当前冻结的传感器安全通信协议 V1。

| 文档 | 状态 | 说明 |
| --- | --- | --- |
| [传感器安全通信协议V1.md](传感器安全通信协议V1.md) | 当前冻结基线 | 线上 `protocol_version=0x01`；CPU2设备侧已有实现证据 |

配套机器契约：

- [sensor_safe_protocol_v1.json](../../golden_frames/sensor_safe_protocol_v1.json)
- [sensor_safe_param_digest_v1.json](../../golden_frames/sensor_safe_param_digest_v1.json)

V1 传感器端实现、真实长线、故障注入、FTTI 和定量认证证据仍未关闭。新设计见 [候选协议 V2](../02_候选协议V2/README.md)。

# LTD 共享 Modbus 协议

本目录维护 CPU2 与 CPU3 共用的 LTD Modbus RTU 对外协议。LTD 菜单协议和 CPU2/CPU3 板间共享寄存器不再作为两套协议分别管理。

## 当前资料

| 文档 | 用途 |
| --- | --- |
| [LTD共享Modbus协议卷.md](LTD共享Modbus协议卷.md) | 当前协议版本、帧格式、功能码、寄存器窗口、命令、CPU3 响应方式、异常码和联调帧 |
| [../CPU2_CPU3协议变更记录.md](../CPU2_CPU3协议变更记录.md) | `DEVICE_PROTOCOL_VERSION` 历史和兼容性 |
| [../系统参数出厂默认值.md](../系统参数出厂默认值.md) | 保持寄存器参数含义、单位和默认值 |

## 当前结论

- 当前共享协议版本为 `DEVICE_PROTOCOL_VERSION = 14`。
- CPU3 的 LTD 外部端口使用与 CPU2 相同的保持寄存器、输入寄存器和命令码。
- CPU3 读请求由最近一次 CPU2 缓存快照独立响应；写请求转发给 CPU2，只有 CPU2 返回合法 ACK 才向外返回成功。
- 本目录只维护 Markdown，不生成或维护 PDF 副本。

# Wartsila 协议适配

更新日期：2026-08-03

本目录保存 WARTSILA/Wärtsilä/瓦锡兰 LTD 设备协议相关资料、现场抓包整理和项目适配结论。

当前正式组合为CPU2 `V1.36.2.0` / CPU3 `V1.36.0.0`、共享协议31；协议30和31均未改变Wartsila外部地址、FC03/FC10格式或字段语义。FC10现场格式收紧仍以CPU3 `V1.29.0.0`为首次正式基线。

## 资料清单

| 资料 | 说明 |
| --- | --- |
| `00_原始资料/2026-01-27_莆田现场Wartsila抓包数据处理.xlsx` | 现场抓包 Excel 原始整理表，保留为协议证据源，不直接改写 |
| `2026-01-27_莆田现场Wartsila抓包协议整理.md` | 基于抓包、公开资料和当前工程实现整理的协议口径、寄存器、命令、状态码和兼容风险 |
| `当前兼容情况与协议栈.md` | 以当前源码为准整理 CUBE 已兼容范围、协议栈链路、寄存器覆盖、缺口和修复优先级 |

## 维护口径

- 原始 Excel 只归档，不在工程内直接覆盖；若后续有新抓包，新增到 `00_原始资料/` 并补 Markdown 结论。
- 当前公开网络资料只能确认 Wartsila/Whessoe LTD 设备使用 Modbus/RS485 或冗余 Modbus，未找到完整公开寄存器表；寄存器细节以现场抓包和项目源码核对为准。
- 修改 `LTD_DISPLAY_CPU3/Communication/external/wartsila_modbus/` 或 `Communication/external/external_read_freshness.h` 中 Wartsila 校验 helper 的行为时，同步更新本目录两份 Markdown、`docs/05_测试记录/协议台架验收/` 的工具 catalog/派生矩阵和 Wartsila 专项契约。
- 已发布版本记录保留当时的真实兼容口径；尚未升版的工作树行为必须标为“未发布”，不得反写进旧版 CHANGELOG 或旧版改动与测试方案。

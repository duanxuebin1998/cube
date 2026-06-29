# CPU2 故障码资料索引

更新日期：2026-06-27

本目录保存 CPU2 侧 LTD 故障代码、设备参数和测量结果辅助表。当前 `LTD故障代码表.xlsx` 是故障码表格维护入口，定义口径以 CPU2 源码为准。

| 文档 | 内容 |
| --- | --- |
| `LTD故障代码表.xlsx` | CPU2 故障代码总表、整理说明、设备参数和测量结果字段；已按当前 `ErrorCode` 补齐 `AD5421_*` 错误码 |

维护时优先对照 `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h` 的 `ErrorCode`、`LTD_MAIN_CPU2/Services/Utilities/error_log.c` 的 `ErrorLog_GetCodeName()` / `ErrorLog_GetReasonByCode()`，以及 `LTD_MAIN_CPU2/Services/Modbus/` 的错误码上报位置。

如需整理 CPU2/CPU3 故障码表，以本目录表格和 CPU2 源码为权威入口；CPU3 表只作为显示、保持寄存器和 `param_meta` 辅助资料。跨目录判定见 `D:/CUBE/docs/00_构建与版本/故障代码表权威来源判定.md`。

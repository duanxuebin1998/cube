# CPU3 故障码资料索引

更新日期：2026-06-27

本目录保存 CPU3 显示侧使用的 LTD 故障代码表、设备参数、测量结果和保持寄存器辅助表。这里的 `LTD故障代码表.xlsx` 不作为故障码定义权威来源。

| 文档 | 内容 |
| --- | --- |
| `LTD故障代码表.xlsx` | CPU3 故障代码、设备参数、测量结果、保持寄存器和 `param_meta` 辅助表；当前已包含四路继电器报警输出参数 |

维护 CPU3 显示文案时优先对照 `LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h` 的兼容枚举、`LTD_DISPLAY_CPU3/Application/display/display.c` 的 `Display_GetErrorReasonByCode()` 和 `D:/CUBE/docs/04_界面与菜单/`。

如需整理故障码定义，先以 `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h` 和 `docs/00_构建与版本/故障码/CPU2/LTD故障代码表.xlsx` 为准；本目录表格只同步显示侧需要的辅助信息。跨目录判定见 `D:/CUBE/docs/00_构建与版本/故障代码表权威来源判定.md`。

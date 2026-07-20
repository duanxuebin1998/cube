# 协议台架验收产物说明

更新日期：2026-07-18

本目录保存 CUBE 固件协议事实、LTDDeviceDebugTool 协议目录及两者的静态比较/命令闭环产物。这里的 `Unverifiable`、`BenchPending` 或非零状态是验收结论，不应为了“全绿”而删除；静态差异为 0 也不等于真实设备、PLC 或业务完成闭环已经通过。

## 文件职责

| 文件 | 类型 | 维护口径 |
| --- | --- | --- |
| `firmware-protocol-facts.json` | 输入快照 | 由 `tools/protocol_fact_inventory.py` 从当前 CUBE 固件源码生成 |
| `tool-protocol-catalog.json` | 输入快照 | 由 `D:\CUBE_TOOLS\LTDDeviceDebugTool` 的 `export-catalog` 从工具运行时目录生成；同步前先确认工具源码/测试已采用当前固件协议口径 |
| `protocol-catalog-policy.json` | 受控派生输入 | 仅允许基于精确键和源码证据关闭可解释差异，并绑定 facts/catalog SHA-256 |
| `protocol-catalog-diff.json` | 派生报告 | 严格比较 facts、catalog 和 policy，并记录三项输入 SHA-256 |
| `command-closure.json` | 派生报告 | 生成命令语义、ACK、完成、Cleanup 和台架资格闭环状态 |
| `多协议命令闭环矩阵.md` / `.csv` | 派生阅读件 | 与 `command-closure.json` 同源，绑定闭环 JSON 和四项输入 SHA-256 |

## 当前 Wartsila 字节计数契约

- FC03 响应按标准 Modbus 使用 `byteCount=2*quantity`。
- FC10 命令和参数请求仅接受莆田现场 `byteCount=quantity`，实际数据区仍为 `2*quantity`，RTU 长度为 `9+2*quantity`，PDU 长度为 `6+2*quantity`。
- 标准 FC10 `byteCount=2*quantity` 返回 `0x03`，且不得写寄存器池或下发 CPU2。

## 重建顺序

在 `D:\CUBE` 中使用本机维护的 `tools/`，按以下顺序生成。输出文件是正式文档快照；运行前后都应核对 `git status --short`，避免覆盖其它任务的暂存或工作树内容。

```powershell
py -X utf8 tools\protocol_fact_inventory.py --root D:\CUBE --output "docs\05_测试记录\协议台架验收\firmware-protocol-facts.json" --check

py -X utf8 tools\protocol_catalog_policy.py --firmware-facts "docs\05_测试记录\协议台架验收\firmware-protocol-facts.json" --tool-catalog "docs\05_测试记录\协议台架验收\tool-protocol-catalog.json" --output "docs\05_测试记录\协议台架验收\protocol-catalog-policy.json"

py -X utf8 tools\protocol_catalog_compare.py --firmware-facts "docs\05_测试记录\协议台架验收\firmware-protocol-facts.json" --tool-catalog "docs\05_测试记录\协议台架验收\tool-protocol-catalog.json" --policy "docs\05_测试记录\协议台架验收\protocol-catalog-policy.json" --output "docs\05_测试记录\协议台架验收\protocol-catalog-diff.json"

py -X utf8 tools\protocol_command_closure.py --firmware-facts "docs\05_测试记录\协议台架验收\firmware-protocol-facts.json" --tool-catalog "docs\05_测试记录\协议台架验收\tool-protocol-catalog.json" --policy "docs\05_测试记录\协议台架验收\protocol-catalog-policy.json" --comparison "docs\05_测试记录\协议台架验收\protocol-catalog-diff.json" --output-json "docs\05_测试记录\协议台架验收\command-closure.json" --output-markdown "docs\05_测试记录\协议台架验收\多协议命令闭环矩阵.md" --output-csv "docs\05_测试记录\协议台架验收\多协议命令闭环矩阵.csv"
```

`protocol_catalog_compare.py` 和 `protocol_command_closure.py` 返回 `3` 表示存在不可静态证明项，是当前受控验收状态；返回 `2` 才表示参数、输入绑定或 schema 无效。生成后必须确认 policy、diff、closure 和 Markdown 输入绑定中的 SHA-256 与当前文件一致。

## 工具 catalog 同步

完整 catalog 只能从工具维护源导出，不在本目录手工增删参数、命令或台架条目：

```powershell
cd D:\CUBE_TOOLS\LTDDeviceDebugTool
dotnet run --project .\src\LTDDeviceDebugTool.Cli -c Release -- export-catalog --output "D:\CUBE\docs\05_测试记录\协议台架验收\tool-protocol-catalog.json"
```

工具维护源若同时包含其它尚未纳入 CUBE 验收范围的 catalog schema 扩展，应先单独评审整份导出差异，不能把无关的大批字段变化与一次固件协议文档同步混在一起。

## 验证边界

- facts/catalog 比较只证明双方静态目录在已建模维度上的一致性。
- policy 只能关闭有精确源码证据的受控差异，不能把未知项批量白名单化。
- `RegisterWriteAckOnly` 不能证明 CPU2 业务接受或命令完成。
- 静态 catalog、golden frame 和主机契约不能替代真实串口电气、设备状态、故障注入和 Cleanup 验证。

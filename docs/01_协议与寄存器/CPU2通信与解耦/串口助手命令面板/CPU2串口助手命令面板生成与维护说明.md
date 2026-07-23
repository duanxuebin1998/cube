# CPU2 串口助手命令面板生成与维护说明

更新日期：2026-07-11

本文说明 CPU2 串口助手命令面板的文件职责、生成方式、同步规则和验证流程。后续不要手工长期维护导出的 JSON；应修改生成脚本，再重新生成配置。

## 1. 文件职责

| 文件 | 职责 |
| --- | --- |
| `tools/generate_cpu2_serial_command_panel.py` | 命令面板唯一维护源；定义分组、按钮说明、payload、生成格式和结构校验 |
| `CPU2串口助手命令面板配置.json` | 仓库内正式生成产物，可直接导入串口助手 |
| `../CPU2串口调试命令协议卷.md` | 串口参数、命令语法、响应、安全边界和验证矩阵的正式协议正文 |
| `LTD_MAIN_CPU2/Application/Src/serial_command_parser.c` | 固件实际接受的命令语法和参数范围 |
| `tools/test_serial_command_parser.c` | 解析器回归用例，并可批量校验面板中的 payload |

维护优先级为：固件实现和协议卷确定命令能力，生成脚本确定面板内容，JSON 只作为脚本生成的可导入产物。桌面或其他位置的副本不作为维护源。

## 2. 当前配置基线

- 共 `10` 个分组、`67` 个按钮。
- 第一组固定为 `00 系统查询与安全控制`。
- 每个按钮使用 `sendMode=text`、`delayMs=100`。
- 每个命令组使用 `type=group`、`repeatCount=0`。
- `groups` 和 `rootCommands` 保持为空数组。
- 串口助手应配置为自动追加 `CRLF`；固件也接受单独 `LF`。

顶层结构如下：

```json
{
  "groups": [],
  "items": [
    {
      "type": "group",
      "name": "00 系统查询与安全控制",
      "repeatCount": 0,
      "commands": [
        {
          "comment": "查看命令帮助",
          "payload": "HELP",
          "sendMode": "text",
          "delayMs": 100
        }
      ]
    }
  ],
  "rootCommands": []
}
```

## 3. 生成和校验

在仓库根目录 `D:\CUBE` 执行。

### 3.1 生成仓库内正式配置

```powershell
py tools\generate_cpu2_serial_command_panel.py
```

默认输出到：

```text
docs\01_协议与寄存器\CPU2通信与解耦\串口助手命令面板\CPU2串口助手命令面板配置.json
```

### 3.2 检查正式配置是否需要重新生成

```powershell
py tools\generate_cpu2_serial_command_panel.py --check
```

该命令只比较文件，不写入。目标不存在或与脚本生成结果不一致时返回非零退出码。

### 3.3 同步到桌面串口助手配置

```powershell
py tools\generate_cpu2_serial_command_panel.py `
  --output "C:\Users\admin\OneDrive\Desktop\ComAssistantCommandPanel.json"
```

同步后可再检查指定文件：

```powershell
py tools\generate_cpu2_serial_command_panel.py `
  --output "C:\Users\admin\OneDrive\Desktop\ComAssistantCommandPanel.json" `
  --check
```

桌面位置因电脑和账户而异，因此脚本不硬编码桌面路径。其他电脑使用时只需替换 `--output` 的绝对路径。

## 4. 修改命令面板的标准流程

### 4.1 新增命令

1. 在 `serial_command_parser.c` 和实际执行入口中实现严格解析及业务行为。
2. 在 `tools/test_serial_command_parser.c` 增加合法、非法、边界和尾随字符用例。
3. 在 `CPU2串口调试命令协议卷.md` 补充语法、参数单位、范围、响应和风险。
4. 在 `build_panel()` 的合适分组中增加 `(按钮说明, payload)`。
5. 重新生成 JSON，执行本文第 6 节验证。

如果只是为已有参数化命令增加一个常用按钮，可以从第 4 步开始，但仍需确认 payload 满足当前解析器和协议卷约束。

### 4.2 修改命令

不要只改 JSON。先确认修改属于哪一类：

- 只改按钮文案或常用参数：修改生成脚本，重新生成并验证。
- 改 payload、命令格式或参数范围：同步修改固件解析器、解析器测试、协议卷和生成脚本。
- 改命令行为或风险：同步检查业务实现、停止路径、应答口径和协议卷。

### 4.3 删除命令

先确认是否仍需兼容现场工具或旧调试流程。删除固件命令时，同步删除解析器用例、协议卷条目和生成脚本按钮；如果固件暂时保留兼容而面板不再推荐，可只从面板移除，并在协议卷标注兼容状态。

## 5. 分组与安全规则

- `STOP/HELP/VER?/STAT?/ERR?` 必须保留在第一组。
- `STOP` 在第一组和循环测试组各放一个按钮是有意设计，便于紧急停止，不视为跨组重复错误。
- 每个组内的 payload 不得重复。
- `Q` 必须放在高风险组，按钮文案固定明确提示“会覆盖参数”，不得与普通测量命令相邻。
- 循环、耐久和长流程命令应与单次正式测量分开，并在同组末尾保留醒目的 `STOP`。
- 手动运动默认值优先使用小距离和正常速度，避免把极限距离、高速或破坏性参数做成一键按钮。
- 参数模板必须在文案中标记“先编辑”，例如 `SPN=DEVICE_NAME`。
- 查询命令应保持只读；如果命令会写参数、切换连接、启动电机或改变输出，文案应体现影响。

生成脚本会校验固定顶层字段、第一组、字段完整性、发送属性、组内重复、五个系统命令及 `10` 组/`67` 按钮基线。新增或删除按钮时，应有意识地同步更新脚本中的规模断言。

## 6. 完整验证流程

### 6.1 生成一致性

```powershell
py tools\generate_cpu2_serial_command_panel.py
py tools\generate_cpu2_serial_command_panel.py --check
```

### 6.2 编译解析器测试

```powershell
$taskId = "cpu2-serial-panel-$(Get-Date -Format 'yyyyMMdd-HHmmss')"
$testBuildDir = [IO.Path]::GetFullPath((Join-Path (Get-Location) "tmp\$taskId\build"))
[void][IO.Directory]::CreateDirectory($testBuildDir)

gcc -std=c11 -Wall -Wextra -Werror `
  -I LTD_MAIN_CPU2/Application/Inc `
  LTD_MAIN_CPU2/Application/Src/serial_command_parser.c `
  tools/test_serial_command_parser.c -lm `
  -o "$testBuildDir\serial_command_parser_tests.exe"
```

### 6.3 用实际解析器校验全部面板 payload

```powershell
$panel = Get-Content -Raw -Encoding UTF8 `
  "docs\01_协议与寄存器\CPU2通信与解耦\串口助手命令面板\CPU2串口助手命令面板配置.json" `
  | ConvertFrom-Json
$payloads = @($panel.items.commands.payload)
& "$testBuildDir\serial_command_parser_tests.exe" $payloads
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
```

测试程序先运行固定回归用例，再逐条把 JSON 中的 payload 交给实际 C 解析器。任何 `INVALID` 或 `UNSUPPORTED` 都会使测试失败。

### 6.4 文档与文本检查

```powershell
py tools\check_flow_docs.py
git diff --check -- `
  tools/generate_cpu2_serial_command_panel.py `
  "docs/01_协议与寄存器/CPU2通信与解耦/README.md" `
  "docs/01_协议与寄存器/CPU2通信与解耦/串口助手命令面板"
```

还应确认脚本、Markdown 和 JSON 均为 UTF-8，无中文乱码；生成 JSON 使用 CRLF 且带末尾换行。

### 6.5 现场导入冒烟验证

1. 导入生成的 JSON，确认显示 10 个分组且按钮文案完整。
2. 串口设置为 `115200 8N1`，文本发送并追加 `CRLF`。
3. 先执行 `VER?`、`STAT?`、`ERR?`，确认均收到 `ACK` 和对应查询内容。
4. 在具备安全条件时用小距离命令验证发送，例如 `A+10`，随后执行 `STOP`。
5. 不把 `Q` 作为普通冒烟测试命令；它会恢复出厂参数。

## 7. 版本与提交边界

仅调整生成脚本、按钮文案或已有命令的常用参数，不改变固件和通信协议行为时，无需单独升级 CPU2 固件版本或共享协议版本。若同时改变解析器、命令行为、参数范围或应答，应按最终固件差异评估并更新 CPU2 版本及版本改动与测试文档。

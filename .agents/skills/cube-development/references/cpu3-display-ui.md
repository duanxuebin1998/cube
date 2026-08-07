# CPU3 显示、菜单和 UI 评审

## 适用范围

- CPU3 OLED 菜单、状态页、软键、确认页、输入页、显示参数和中文文案。
- 用户询问 UI 是否合理、参数是否生效、显示是否立即变化、是否需要重启。
- CPU3 本机显示参数、FRAM 保存参数和运行态 `screen_parameter` 之间的一致性检查。

## 只读分析边界

- 用户说“评价一下 UI”“有没有更好方案”“当前是否有用”“先不改代码”“帮我确认问题并制定方案”时，保持只读。
- 只读阶段先给证据链和方案，不修改源码、不升版本、不补发布文档。
- 用户明确说“帮我修改”“可以改”“提交”后，再进入实现、构建、版本和文档流程。

## CPU3 OLED 基本约束

- CPU3 屏幕按 128x64 OLED 和 4 行交互设计，优先保持 4 行节奏。
- 评审或修改菜单时先查 `LTD_DISPLAY_CPU3/Application/display/display.h` 的行列常量。
- 菜单入口和页面逻辑主要在 `LTD_DISPLAY_CPU3/Application/display/display_tankopera.c`。
- 状态页和运行显示逻辑主要在 `LTD_DISPLAY_CPU3/Application/display/display.c`。
- 文档证据优先查 `docs/04_界面与菜单/` 下的菜单优化、状态页确认表和相关说明。
- 不要把现有菜单树整体推翻为大屏 UI；优先做流程、文案、返回路径、确认页和高频路径优化。

## UI 评审检查

- 先确认当前菜单树、状态页、分页、确认页、输入页和软键行为，再提出方案。
- 检查主菜单和参数菜单的分组是否仍符合业务域。
- 检查 `menuselect()` 或对应选择函数的分页行为，确认新文案不破坏 4 行显示。
- 检查 `oled_fit_text()`、值显示函数、确认页函数和输入函数是否已有可复用能力。
- 检查返回路径，尤其是多级菜单和 4 级子页面，避免返回到祖父级或错误父级。
- 检查状态页是否混入过期测量结果；无效值要按文档显示，例如无效 RSSI 显示为 `RSSI:N/A`。
- 修改中文显示内容时同步读取 `encoding-and-text.md`，检查文件编码、字库覆盖、字符串长度和截断风险。

## 参数是否生效检查

遇到“参数是否有用”“改完是否立即生效”“是否需要重启”时，必须同时追踪：

1. 参数表或菜单入口：确认用户看到的文案、命令号、寄存器号或枚举值。
2. 写入口：确认写入函数是否只保存 FRAM，还是同步了运行态变量。
3. 运行态消费点：确认实际业务逻辑读取的是哪个变量或结构体。
4. 启动加载路径：确认上电、恢复默认或 FRAM 加载时是否同步运行态。

不要只凭以下证据判断立即生效：

- 菜单能读回新值。
- FRAM 保存函数被调用。
- Modbus 或本机参数表存在该项。
- 启动加载路径会同步，但写入口没有同步。

## CPU3 显示参数运行态

- `Cpu3Local_WriteValue()` 是 CPU3 本机参数统一写入口之一；显示类参数变更时要判断是否需要立即同步运行态。
- `Cpu3Local_ApplyDisplayRuntimeParams()` 是把本机显示参数同步到 `screen_parameter` 的核心入口；它会同步语言、小数位、密码、息屏和亮度等显示相关运行态。
- `Cpu3_Params_LoadFromFRAM()` 启动加载和恢复默认时会同步运行态，但这不能替代写入后的立即同步。
- 息屏开关是 OLED 显示开/关逻辑，不是 MCU 低功耗睡眠；不要把它描述成整机休眠。
- `COM_NUM_SCREEN_OFF` 是 0/1 选择项，不是息屏时长字段；现有空闲阈值按显示逻辑中的固定时间判断。
- 修改亮度、语言、小数位、密码、息屏等显示参数时，保持“保存 + 统一运行态应用”的策略，避免单个分支直接设置导致策略不一致。

## 常用搜索入口

- 菜单和页面：`rg -n "menuselect|display_menu_item_with_value|param_protect_confirm|ifsendcmd|inputcmdpara" LTD_DISPLAY_CPU3/Application/display`
- 状态页和息屏：`rg -n "screenoff|ScreenOff|OLED_DisplayOff|OLED_DisplayOn|RefreshScreen|screen_parameter" LTD_DISPLAY_CPU3/Application/display LTD_DISPLAY_CPU3/Application/system_param`
- 本机参数：`rg -n "Cpu3Local_WriteValue|Cpu3Local_ApplyDisplayRuntimeParams|Cpu3_Params_LoadFromFRAM|COM_NUM_SCREEN" LTD_DISPLAY_CPU3/Application/system_param`
- 菜单/状态页文档：`rg -n "菜单|状态页|RSSI|息屏|显示|确认|返回" docs/04_界面与菜单`

在 PowerShell 中不要把 `LTD_DISPLAY_CPU3\Application\display\*.c` 当作单个路径传给 `rg`；优先搜索目录，或使用 `rg -g '*.c'`。

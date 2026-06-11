# CPU3 屏幕驱动与刷新问题分析及整改方案

更新时间：2026-06-11

本文合并 CPU3 OLED 屏幕资料确认、当前驱动问题、静电花屏恢复思路、花屏检测边界，以及本次已落地的代码整改。本文只针对当前固件实际匹配的 `HGS128645-Y-EH-LV / SSD1325 OLED` 屏幕，不纳入其它不匹配屏幕资料。

## 结论摘要

当前 CPU3 固件匹配的屏幕是 `HGS128645-Y-EH-LV`，控制器为 `SSD1325`。该屏在串行模式下只允许写操作，因此 CPU3 不能可靠读回 OLED GDDRAM，也不能仅靠软件百分百判断玻璃上的真实显示是否花屏。

本次整改已完成以下代码落地：

| 项目 | 结果 |
| --- | --- |
| TIM3 定时刷新 | 中断内只调用 `Display_RequestRefresh()`，主循环 `Display_Task()` 执行真正刷新 |
| 按键 EXTI | 中断内只记录按键事件，不再 `printf()`，不再直接 `KeyProcess()` |
| TIM1 长按 | 中断内只投递长按动作，不再直接进菜单或弹确认页 |
| 普通清屏和恢复清屏 | `oled_clear()` 改为 `OLED_Clear()`；带复位恢复集中到 `OLED_RecoverAndClear()` |
| SPI 发送 | `HAL_MAX_DELAY` 改为有限超时 `OLED_SPI_TIMEOUT_MS`，并累计 SPI 错误计数 |
| 清屏 SPI 开销 | 满屏填充改为 64 字节块发送，初始化命令和填充数据都会在首个失败处停止 |
| 息屏 | 到达息屏条件时发送 `OLED_DisplayOff()`，状态页和菜单前台绘制前都会唤醒并按需恢复 |
| 软件一致性校验 | 增加 5120 字节 shadow buffer、CRC32 和刷新序号，只在 SPI 无新增错误的完整帧后更新 |
| 菜单清屏 | 菜单中的清零式 `all_screen(0x00)` 改为 `oled_clear()`，避免菜单切页频繁复位 OLED |
| 菜单态恢复 | `FlagofTankOpera == true` 时也会响应 SPI 错误和 60 秒恢复周期，恢复后重绘当前菜单页 |
| 状态页差量刷新 | 普通状态页改为快照比较，布局不变时不再整屏清零，只在数值变化时局部清除重绘 |
| 数据变化提示 | 当前页数值或文字与上一次不同会短暂反白显示，用于提示用户数据已刷新 |
| 息屏空闲计时 | 息屏开关打开后按最后按键时间计时，避免每轮刷新都清掉亮屏标志导致立即息屏 |

仍需现场验证的是：静电打花屏后的恢复成功率、低频恢复周期是否合适、`OLED_NREST` 复位脉冲是否只在预期场景出现，以及真实屏幕显示是否仍存在局部残影或亮度拖尾。

## 当前屏幕资料

项目内只保留当前匹配屏幕资料：

- `docs/04_界面与菜单/00_屏幕资料/HGS128645-Y-EH-LV_SSD1325_OLED_当前液位计屏幕_中文版.pdf`

该资料对应要点：

- 屏幕型号：`HGS128645-Y-EH-LV`；
- 驱动 IC：`SSD1325`；
- 显示类型：OLED 黄字屏；
- 点阵：128Dots x 64Dots；
- 占空比：1/64；
- 支持接口：8-bit 8080、8-bit 6800、SPI；
- 串行模式：`D1-SI`、`D0-SCL`；
- 串行模式限制：only write operations are allowed。

当前 `LTD_DISPLAY_CPU3/Application/display/hgs.c` 使用的 `0x15`、`0x75`、`0x81`、`0x86`、`0xA0`、`0xA1`、`0xA2`、`0xA8`、`0xB1`、`0xB2`、`0xB3`、`0xB8`、`0xBC`、`0xBE`、`0xBF`、`0xAD`、`0xAF` 等命令与 SSD1325 初始化和显示控制逻辑匹配。

## 原始问题

### 1. 中断里执行重绘和阻塞传输

原 `TIM3_IRQHandler()` 直接调用 `RefreshScreen()`，刷新链路会执行清屏、绘字、满屏 SPI 写入，旧 `oled_clear()` 还会复位 OLED 并 `HAL_Delay()`。这会导致中断执行时间不可控，影响串口、DMA、按键等实时任务。

原按键链路中，`HAL_GPIO_EXTI_Callback()` 可直接调用 `KeyProcess()`，`TIM1_UP_TIM10_IRQHandler()` 可直接进入菜单函数。菜单函数内又存在清屏、延时和绘屏，导致按键中断也可能变成长耗时中断。

### 2. `oled_clear()` 名称和实际行为不一致

旧定义为：

```c
#define oled_clear() all_screen(0x00)
```

而 `all_screen()` 实际做的是复位 OLED、重新发送初始化命令、满屏写数据、开显示。这是恢复动作，不是普通清屏。普通刷新和静电恢复混在一起，会让每次页面刷新、菜单切页甚至息屏都可能复位 OLED。

### 3. SPI 发送无限等待且粒度过小

旧 `WriteCommand()` / `WriteSingleData()` 使用：

```c
HAL_SPI_Transmit(&hspi1, &Data, 1, HAL_MAX_DELAY);
```

如果 SPI 或 OLED 侧异常，可能长时间卡住。满屏清屏原本按 5120 个字节逐字节调用 HAL，也放大了刷新耗时。

### 4. 不能直接识别真实花屏

SSD1325 串行模式只写不读，CPU3 只能知道自己发了什么，不能确认 OLED 控制器内部和玻璃实际显示是否正确。因此软件方案只能做到：

- 检测 SPI 发送错误；
- 检测刷新耗时异常；
- 通过 shadow buffer 校验 CPU3 软件侧待显示数据；
- 周期性或异常时触发 `OLED_RecoverAndClear()` 恢复。

它不能等价于“真实花屏识别”。

## 已实施整改

### 1. 新增显示任务层

新增接口：

```c
void Display_Task(void);
void Display_RequestRefresh(void);
```

调用关系改为：

```text
TIM3_IRQHandler()
  -> Display_RequestRefresh()

App_MainLoop()
  -> Display_Task()
     -> Display_ProcessPendingInput()
     -> RefreshScreen()
```

`Display_Task()` 放在 `cpu3_apply_uart_reinit_if_pending()` 之后、串口业务处理之前执行，避免串口繁忙时显示请求长期得不到调度。

### 2. 按键和长按动作延后处理

新增接口：

```c
void Display_RequestKey(uint8_t keypress);
void Display_RequestLongPressAction(uint8_t long_press_key);
uint8_t Display_TakePendingKey(void);
uint8_t Display_TakePendingLongPressAction(void);
```

`HAL_GPIO_EXTI_Callback()` 现在只做亮屏、按键事件记录、长按计时启动，不直接打印、不直接进菜单、不直接绘屏。

`TIM1_UP_TIM10_IRQHandler()` 达到长按阈值后只调用 `Display_RequestLongPressAction(long_press_key)`，真正的 `useKey()`、菜单入口和取消测量确认页由 `Display_Task()` 在主循环中执行。

按键事件由 8 深度 FIFO 队列保存，不再用 bitmask 合并。因此连续按两次同一个键会按顺序处理，不会被压成一次。队列满时丢弃最新按键并累计内部溢出计数，避免中断覆盖尚未处理的历史按键。

### 3. 拆分普通清屏和恢复清屏

新增接口：

```c
void OLED_Clear(void);
void OLED_RecoverAndClear(void);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
```

现在：

- `oled_clear()` 映射到 `OLED_Clear()`，只设置窗口并写 0；
- `OLED_RecoverAndClear()` 调用带复位和初始化的 `all_screen(0x00)`；
- `OLED_DisplayOff()` 发送 `0xAE`；
- `OLED_DisplayOn()` 发送 `0xAF`。

普通状态页和菜单切页清屏不再默认复位 OLED。静电恢复能力由 `OLED_RecoverAndClear()` 集中承担。

### 4. 息屏和唤醒策略

`RefreshScreen()` 中如果达到息屏条件，只在刚进入息屏时调用一次 `OLED_DisplayOff()`，不再周期性清屏或复位。息屏判断由 `DISPLAY_SCREEN_OFF_IDLE_MS` 控制，当前默认 30 秒；`SetScreenBright()` 在每次按键时刷新最后活动时间，避免普通刷新周期内把亮屏标志清掉后马上息屏。

按键唤醒时 `SetScreenBright()` 会请求刷新；如果上一次处于息屏状态，下一次刷新会先 `OLED_DisplayOn()`，再标记 `display_recover_requested`，使绘制前执行一次 `OLED_RecoverAndClear()`。

如果息屏后用户长按进入菜单，或在菜单态按键翻页，显示层会在真正绘制菜单页前先执行 `OLED_DisplayOn()`，再按 SPI 错误计数和恢复周期决定是否执行 `OLED_RecoverAndClear()`，避免 OLED 仍处于关显示状态时只在软件里重画。

### 5. 状态页差量刷新和变化提示

旧状态页每次 `TIM3` 触发刷新都会先 `Display_ClearBeforeDraw()`，等价于先把整屏写黑，再重新绘制设备状态和数值。OLED 没有双缓冲，用户能看到整屏先灭再亮，因此表现为周期性一闪一闪。

本次增加状态页快照 `DisplayStatusSnapshot`，刷新流程改为：

```text
Display_BuildStatusSnapshot()
  -> Display_StatusLayoutChanged()
     -> true  : Display_DrawStatusFull()
     -> false : Display_DrawStatusDelta()
```

处理原则：

- 首次绘制、状态变化、语言变化、页号变化、显示项数量变化、行位置变化、SPI 错误恢复和 60 秒保底恢复时，仍执行一次整页清空重画；
- 普通状态页布局不变时，不再整屏清空；
- 当前页某个数值和上一次不同，只调用 `OLED_ClearArea()` 清除该数值右侧区域，再重画该值；
- 数值变化后保留 `DISPLAY_VALUE_HIGHLIGHT_FRAMES` 帧反白显示，随后再重画回正常显示；
- 状态页多页轮播由 `DISPLAY_STATUS_PAGE_HOLD_REFRESHES` 控制，当前每页保持 3 个刷新周期，避免每秒翻页造成视觉闪烁。

这个方案的目标是消除“整屏灭一下”的闪烁，同时让用户能看出某个数据确实刷新了。反白只作用在数值或文字值区域，标签仍保持普通显示，避免整行大面积闪烁。

### 6. SPI 健康计数和刷新耗时统计

底层 SPI 写命令和写数据改为有限超时：

```c
#define OLED_SPI_TIMEOUT_MS 20U
```

发送失败或超时时累计 `oled_spi_error_count`，显示层通过 `OLED_GetSpiErrorCount()` 发现计数变化后，在下一次绘制前触发 `OLED_RecoverAndClear()`。

显示任务记录最近一次刷新耗时 `display_last_refresh_ms`，超过 `DISPLAY_REFRESH_TIMEOUT_MS` 时累计 `display_refresh_timeout_count`。

底层 shadow buffer 只在 `HAL_SPI_Transmit()` 返回 `HAL_OK` 后推进；窗口命令链路也只有全部成功后才更新 shadow 窗口。满屏填充 `OLED_WriteFillData()` 在首个 64 字节块发送失败时立即返回，恢复初始化命令序列也在首个失败命令处停止，避免 SPI 异常时一次恢复持续等待大量超时。

### 7. 低频保底恢复

显示层保留 60 秒低频保底恢复：

```c
#define DISPLAY_RECOVER_INTERVAL_MS 60000U
```

策略为：

- 普通状态页刷新：布局不变时只做差量刷新，布局变化或恢复场景才清屏重画；
- 普通菜单刷新：`OLED_Clear()`；
- 上电初始化：`OLED_Init()` 内仍执行恢复式清屏；
- 从息屏唤醒：请求一次恢复式清屏；
- SPI 错误计数变化：请求一次恢复式清屏；
- 超过低频恢复周期：请求一次恢复式清屏。
- 菜单状态：不再跳过恢复检查；触发恢复后重绘当前菜单页。

如果现场静电问题严重，可后续把恢复周期做成宏配置或参数配置。

### 8. Shadow buffer 和软件一致性校验

新增 5120 字节 shadow buffer：

```c
#define OLED_SHADOW_SIZE 5120U
```

它按当前 SSD1325 完整写窗口维护 CPU3 理论上写给 OLED 的数据副本，并提供：

```c
uint32_t OLED_GetShadowCrc(void);
uint32_t OLED_GetRefreshSeq(void);
void OLED_MarkFrameComplete(void);
```

用途：

- 页面生成或绘制数据异常时，CRC 可用于软件侧诊断；
- `OLED_GetRefreshSeq()` 可用于判断显示任务是否仍在推进；
- 后续做局部刷新或 dirty row 时，可复用 shadow buffer。

帧完成口径已收紧：只有本次绘制期间 `OLED_GetSpiErrorCount()` 没有新增错误，才调用 `OLED_MarkFrameComplete()` 更新 CRC 和刷新序号。这样 `OLED_GetRefreshSeq()` 更接近“无 SPI 错误的完整刷新次数”，失败传输不会被误记为成功帧。

边界：shadow buffer 只能证明 CPU3 软件侧“已成功交给 HAL/SPI 的数据”是否一致，不能证明 OLED 控制器内部或玻璃真实显示没有花屏。

### 9. 菜单态恢复与当前页重绘

菜单态原先由 `RefreshScreen()` 直接跳过，因此 SPI 错误恢复、60 秒低频恢复和息屏唤醒恢复都不会在菜单停留期间生效。本次增加：

```c
bool DisplayTankOpera_RedrawCurrentPage(void);
bool DisplayTankOpera_CanProcessKey(uint8_t keypress);
bool Display_CanEnterCancelMeasurementConfirm(void);
```

处理原则：

- 菜单按键真正会触发页面动作前，先确保 OLED 已经 `DisplayOn`；
- 若 SPI 错误计数变化、从息屏唤醒或达到 60 秒恢复周期，先执行 `OLED_RecoverAndClear()`；
- 恢复后重绘当前菜单页，并临时把 `NowKeyPress` 置 0，避免重复执行上一次确认、返回、上下键动作；
- `KeyProcess()` 增加 `func_index` 边界检查，异常索引不会直接访问 `keymenu[]`。

## 关键文件

| 文件 | 本次作用 |
| --- | --- |
| `LTD_DISPLAY_CPU3/Core/Src/stm32f4xx_it.c` | TIM3/TIM1 中断改为只投递显示和长按请求 |
| `LTD_DISPLAY_CPU3/Application/app_main.c` | 主循环调用 `Display_Task()` |
| `LTD_DISPLAY_CPU3/Application/display/display.c` | 显示任务、恢复策略、息屏唤醒、菜单态恢复、状态页快照差量刷新、数据变化反白、刷新耗时统计 |
| `LTD_DISPLAY_CPU3/Application/display/display.h` | 新增显示任务接口声明 |
| `LTD_DISPLAY_CPU3/Application/display/exit.c` | 按键事件队列和 EXTI 轻量化 |
| `LTD_DISPLAY_CPU3/Application/display/exit.h` | 新增按键事件接口声明 |
| `LTD_DISPLAY_CPU3/Application/display/hgs.c` | SSD1325 普通清屏、局部清除、恢复清屏、SPI 超时、shadow buffer、失败早停 |
| `LTD_DISPLAY_CPU3/Application/display/hgs.h` | 新增 OLED 驱动接口声明，包括 `OLED_ClearArea()` |
| `LTD_DISPLAY_CPU3/Application/display/display_tankopera.c` | 菜单切页普通清屏、当前页重绘、按键动作返回绘制结果 |
| `tools/check_cpu3_display_isr_boundaries.py` | 回归检查中断边界、菜单态恢复和 shadow buffer 接口 |

## 花屏检测方案边界

### 可以做的

- SPI 发送错误计数：发现 `HAL_ERROR`、`HAL_TIMEOUT`；
- 刷新耗时统计：发现刷新阻塞或耗时异常；
- 刷新序号：发现显示任务不再推进；
- shadow CRC：发现 CPU3 软件侧待显示数据异常变化或没有变化；
- 周期恢复：即使不能识别真实花屏，也定期拉回 OLED 控制器状态；
- 唤醒恢复：息屏后唤醒时重新初始化，降低静电后显示异常残留概率。
- 菜单态恢复：长时间停在菜单时也能周期性恢复 OLED 控制器状态。
- 状态页差量刷新：减少周期性整屏灭屏造成的视觉闪烁。

### 不能做的

- 不能通过当前 SPI 串行方式读回 SSD1325 GDDRAM；
- 不能确认 OLED 控制器内部扫描状态；
- 不能确认玻璃真实像素是否和 shadow buffer 一致；
- 不能把 shadow CRC 宣称为真实花屏识别。

### 如果必须识别真实花屏

需要额外硬件或工装：

- 摄像头工装拍固定测试图案；
- 屏幕角落放测试块，用光敏二极管/光电传感器检测亮灭；
- 后续硬件改版选用可可靠读回显存或状态的显示接口；
- 加强 ESD 防护，减少花屏发生概率，而不是只靠软件恢复。

## 现场验证清单

### 1. 中断耗时

用示波器在 TIM3 中断入口/出口翻转 GPIO。预期 TIM3 中断只置位请求，持续时间显著短于旧版本，不包含满屏刷新。

### 2. OLED 复位脚

观察 `OLED_NREST`。预期：

- 普通 2 秒刷新不再触发复位；
- 菜单切页不再触发复位；
- 上电、息屏唤醒、SPI 错误恢复、60 秒低频恢复、菜单态恢复时才触发复位。

### 3. SPI 时序和刷新耗时

用逻辑分析仪观察 SCK/MOSI/DC。预期清屏使用 64 字节块发送，刷新耗时较旧单字节调用降低。

### 4. 状态页闪烁和数据变化提示

固定在普通状态页观察 1 分钟。预期：

- 数值不变时不再每秒整屏闪黑；
- 单个数值变化时，只有该数值区域短暂反白，随后恢复正常显示；
- 页面轮播时允许整页切换，但每页保持多个刷新周期，不应每秒跳页；
- 60 秒保底恢复时允许出现一次明显重绘或复位闪烁，这是静电恢复策略带来的可预期动作。

### 5. 按键和通信压测

持续 CPU2/CPU3 通信，同时快速按键进菜单、翻页、返回。预期按键不乱跳、通信无新增超时、菜单显示不残缺。

### 6. 静电恢复

按现场既有静电测试方法打屏幕或接口，记录：

- 是否花屏；
- 是否在低频恢复或唤醒恢复后恢复；
- 恢复耗时；
- `OLED_NREST` 是否按预期动作；
- 是否影响测量和通信。

### 7. Shadow buffer 诊断

在固定页面连续刷新，观察 `OLED_GetShadowCrc()` 和 `OLED_GetRefreshSeq()`。预期：

- 页面内容不变且 SPI 无新增错误时 CRC 稳定；
- 页面内容变化时 CRC 改变；
- 刷新序号随无新增 SPI 错误的完整刷新递增；
- 若强制制造 SPI 超时，错误计数增加，但本次失败帧不应更新刷新序号。

## 硬件侧建议

软件恢复只能把已经异常的 OLED 拉回，不能减少静电进入芯片的概率。建议同步检查：

- `SCK`、`MOSI`、`DC`、`CS/NSS`、`RESET` 是否有合适串阻；
- OLED 接口是否有 TVS 或 ESD 防护；
- 排线地回流路径是否短且可靠；
- `RESET` 是否有稳定上拉或 RC 滤波；
- 外壳、屏蔽层和主板地连接方式是否会把静电耦合到信号线上。

## 验证记录

已在当前主线工作区 `D:\CUBE` 执行：

```text
py tools\check_cpu3_display_isr_boundaries.py
cmake --build build\LTD_DISPLAY_CPU3
git diff --check -- LTD_DISPLAY_CPU3/Application/display/display.c LTD_DISPLAY_CPU3/Application/display/hgs.c LTD_DISPLAY_CPU3/Application/display/hgs.h tools/check_cpu3_display_isr_boundaries.py docs/03_问题分析与整改/CPU3屏幕驱动与刷新问题分析及整改方案.md
```

构建产物生成在 `build\LTD_DISPLAY_CPU3` 下，当前文档不记录或整理构建产物。

## 后续建议

1. 用示波器确认 `OLED_NREST` 在普通刷新和菜单切页期间不再周期性复位。
2. 做一次现场静电干扰测试，确认 `OLED_RecoverAndClear()` 的恢复成功率和 60 秒恢复周期是否合适。
3. 如果现场仍然频繁花屏，把恢复周期做成可配置参数，并评估按键长按触发手动恢复入口。
4. 如果状态页仍有轻微拖影或局部闪烁，再把字形绘制合并为连续块发送，或进一步做 dirty row / DMA 刷屏。
5. 若要真正识别玻璃显示花屏，需要增加外部传感器或摄像头工装，单靠当前 SSD1325 串行写屏无法闭环确认。
6. 提交本次 CPU3 显示行为变化前，应统一升级 CPU3 固件版本并同步 `CHANGELOG.md` 和版本改动与测试方案；`DEVICE_PROTOCOL_VERSION` 不需要变更。

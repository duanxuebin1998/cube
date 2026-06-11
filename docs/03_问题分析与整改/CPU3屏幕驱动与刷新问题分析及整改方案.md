# CPU3 屏幕驱动与刷新问题分析及整改方案

更新时间：2026-06-11

本文整理 CPU3 当前 OLED 屏幕驱动、定时刷新、按键绘屏、静电花屏恢复和显示健康检测方案。本文只记录分析与方案，不表示相关代码已经实施修改。

## 背景

现场反馈 OLED 芯片在受到静电影响后可能出现花屏。当前代码在清屏函数中加入 OLED 复位和重新初始化动作，可以作为静电异常后的软件恢复手段。

这个恢复思路本身是合理的，但当前实现把“普通清屏”和“异常恢复”绑定在同一个 `oled_clear()` 调用里，并且该调用会出现在定时器中断和按键中断相关路径中，导致正常刷新也会执行较重的复位、初始化和满屏写入。

## 当前屏幕资料

当前 CPU3 固件匹配的屏幕是 `HGS128645-Y-EH-LV`，控制器为 `SSD1325`。

项目内资料：

- `docs/04_界面与菜单/00_屏幕资料/HGS128645-Y-EH-LV_SSD1325_OLED_当前液位计屏幕_中文版.pdf`

当前 `hgs.c` 使用的命令与 `SSD1325` 对应：

```text
0x15  Set Column Address
0x75  Set Row Address
0x81  SEG 电流等级
0x86  SEG 电流范围
0xA0  Re-map
0xA1  Display Start Line
0xA2  Display Offset
0xA8  Multiplex Ratio
0xB1  Phase Length
0xB2  Row Period
0xB3  Display Clock
0xB8  Gray Scale Table
0xBC  Pre-charge Voltage
0xBE  VCOMH
0xBF  VSL
0xAD  VCC 来源
0xAF  Display On
```

`HGS128645-Y-EH-LV` 手册中的关键点：

- 显示类型：OLED / 黄字；
- 数据输入：8-bit 8080、8-bit 6800、SPI；
- 占空比：1/64；
- 驱动 IC：`SSD1325`；
- 点阵：128Dots x 64Dots；
- 串行模式：`D1-SI`、`D0-SCL`；
- 串行模式下 only write operations are allowed。

因此，当前 CPU3 固件应继续按 `HGS128645-Y-EH-LV / SSD1325` 评估和整改。不匹配当前驱动的屏幕资料不纳入本方案。

## 当前调用链

### 定时刷新链路

```text
TIM3_IRQHandler()
  -> RefreshScreen()
     -> oled_workingdata() / EquipFirstPower()
        -> oled_clear()
           -> all_screen(0x00)
```

主要文件：

- `LTD_DISPLAY_CPU3/Core/Src/stm32f4xx_it.c`
- `LTD_DISPLAY_CPU3/Application/display/display.c`
- `LTD_DISPLAY_CPU3/Application/display/hgs.h`
- `LTD_DISPLAY_CPU3/Application/display/hgs.c`

`TIM3` 配置为待机界面刷新定时器。根据当前 `Prescaler = 17999`、`Period = 10000` 和系统时钟配置，刷新周期约为 2 秒。

### 按键和菜单链路

```text
HAL_GPIO_EXTI_Callback()
  -> KeyProcess()
     -> 菜单页面函数
        -> all_screen() / oled_clear() / HAL_Delay()

TIM1_UP_TIM10_IRQHandler()
  -> keymenu[...].execute_opera()
     -> 菜单页面函数
```

主要文件：

- `LTD_DISPLAY_CPU3/Application/display/exit.c`
- `LTD_DISPLAY_CPU3/Application/display/display_tankopera.c`
- `LTD_DISPLAY_CPU3/Core/Src/stm32f4xx_it.c`

## 当前主要问题

### 1. 刷新在中断里执行过重

`TIM3_IRQHandler()` 直接调用 `RefreshScreen()`。`RefreshScreen()` 可能触发 OLED 复位、`HAL_Delay()`、大量阻塞式 SPI 发送和页面重绘。

影响：

- 中断执行时间不可控；
- 可能影响串口通信、DMA 回调、按键响应和其他实时任务；
- 一旦 SPI 阻塞或 OLED 状态异常，中断路径会被拉长；
- 后续维护时很难判断卡顿来自显示、通信还是按键。

说明：当前 SysTick 优先级高于 TIM3，`HAL_Delay()` 在 TIM3 中断中不一定直接死锁，但在 ISR 中执行延时和长时间阻塞传输仍属于高风险设计。

### 2. `oled_clear()` 名称和实际行为不一致

当前定义：

```c
#define oled_clear() all_screen(0x00)
```

而 `all_screen()` 实际执行：

1. 拉低 `OLED_NREST`；
2. `HAL_Delay(10)`；
3. 拉高 `OLED_NREST`；
4. 重新发送 OLED 初始化命令；
5. 写满屏数据；
6. 打开显示并配置 VCC 来源。

这不是普通清屏，而是“复位 + 重新初始化 + 清屏 + 开显示”。名称不准确会导致调用者误以为它只是轻量清屏。

### 3. 普通刷新和静电恢复没有分层

由于 `oled_clear()` 同时承担普通清屏和异常恢复，所有页面刷新、菜单切换和息屏清空都可能触发复位恢复动作。

影响：

- 正常刷新也会频繁复位 OLED；
- 可能产生肉眼可见闪烁；
- SPI 传输量和 CPU 占用增加；
- 无法单独调整静电恢复频率；
- 后续如果想优化刷新效率，会被恢复逻辑牵制。

### 4. 息屏逻辑不是真正息屏

当前 `ScreenOff()` 为真时，`RefreshScreen()` 调用的是 `oled_clear()`，没有发送 OLED 控制器的 display off 命令。

影响：

- 息屏状态仍会周期性执行复位、初始化和满屏清空；
- 不能降低 OLED 控制器工作负担；
- 唤醒逻辑和恢复逻辑没有明确边界。

### 5. SPI 发送粒度过小

`WriteCommand()` 和 `WriteSingleData()` 每次只发送 1 字节，并使用 `HAL_SPI_Transmit(..., HAL_MAX_DELAY)`。

满屏清除时当前循环写入：

```text
80 行 * 64 字节 = 5120 字节
```

如果每个字节都单独调用一次 HAL SPI 发送，会产生大量函数调用和等待开销。该问题在中断上下文中会被进一步放大。

### 6. 按键中断路径包含绘屏和延时

`HAL_GPIO_EXTI_Callback()` 中会直接调用 `KeyProcess()`，长按检测的 `TIM1_UP_TIM10_IRQHandler()` 中也会直接进入菜单函数。菜单函数中存在 `all_screen()`、`oled_clear()` 和 `HAL_Delay()`。

影响：

- 按键中断可能变成长耗时中断；
- 连续按键时更容易出现响应抖动；
- 菜单绘制和按键采样耦合过紧；
- 中断里 `printf()` 也会进一步增加不确定性。

### 7. OLED 初始化存在重复动作

`OLED_Init()` 中发送一套初始化命令后又调用 `all_screen(0x00)`，而 `all_screen()` 内部会再次复位和发送初始化命令。之后 `OLED_Init()` 又发送显示开关和 VCC 来源命令。

影响：

- 上电初始化时间增加；
- 初始化流程难以维护；
- 不利于区分“首次初始化”和“异常恢复”。

### 8. 不能直接软件识别真实花屏

`HGS128645-Y-EH-LV` 手册说明 SSD1325 串行模式下只允许写操作。因此 CPU3 通过当前 SPI 串行写屏方式，一般只能确认“数据已发出”，不能确认玻璃上最终显示是否正确。

结论：

- CPU3 不能可靠读回 OLED GDDRAM；
- CPU3 不能直接知道屏幕是否真实花屏；
- SPI 发送成功不代表屏幕显示一定正确；
- 静电导致的显示扫描、控制寄存器或模拟驱动异常，可能无法被软件直接观测。

短期方案应以“异常风险检测 + 自动恢复”为主，而不是声明能百分百识别花屏。

## 整改目标

1. 保留静电花屏后的软件恢复能力；
2. 让普通刷新不再无条件复位 OLED；
3. 中断中只记录事件，不执行绘屏、延时和长时间 SPI；
4. 分清普通清屏、显示开关、异常恢复三种动作；
5. 增加 SPI/刷新异常检测和恢复计数；
6. 通过内部影子缓冲提升软件侧显示链路可诊断性；
7. 降低刷新耗时和闪屏概率；
8. 保持现有 UI 样式和菜单内容不变。

## 推荐整改方案

### 第一阶段：刷新从中断移到主循环

目标：行为变化最小，先消除长耗时中断风险。

建议新增显示刷新请求标志：

```c
volatile bool g_display_refresh_pending = false;
```

`TIM3_IRQHandler()` 中只置位：

```c
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
    g_display_refresh_pending = true;
}
```

主循环或应用层任务中执行：

```c
void Display_Task(void)
{
    if (g_display_refresh_pending) {
        g_display_refresh_pending = false;
        RefreshScreen();
    }
}
```

注意点：

- 该阶段可以暂时保留当前 `oled_clear()` 行为；
- 不改变现有抗静电恢复策略；
- 重点是把 `HAL_Delay()`、阻塞 SPI 和页面重绘移出 ISR。

### 第二阶段：按键处理从中断移到主循环

目标：中断只采样和记录按键事件，菜单逻辑在主循环执行。

建议 EXTI 回调只做：

- 记录按键类型；
- 记录亮屏请求；
- 启动或停止长按计时；
- 不调用 `KeyProcess()`；
- 不绘屏；
- 不 `HAL_Delay()`；
- 不 `printf()`。

示例结构：

```c
typedef enum {
    DISPLAY_KEY_NONE = 0,
    DISPLAY_KEY_BACK,
    DISPLAY_KEY_UP,
    DISPLAY_KEY_DOWN,
    DISPLAY_KEY_SURE
} DisplayKeyEvent;

volatile DisplayKeyEvent g_display_key_event = DISPLAY_KEY_NONE;
```

主循环中：

```c
void Display_KeyTask(void)
{
    DisplayKeyEvent event = g_display_key_event;

    if (event != DISPLAY_KEY_NONE) {
        g_display_key_event = DISPLAY_KEY_NONE;
        KeyProcess(event);
    }
}
```

实际实现时需要结合现有长按进入菜单、长按返回取消测量的逻辑，避免改变用户操作语义。

### 第三阶段：拆分 OLED 清屏和恢复接口

目标：保留抗静电恢复，但让普通刷新可控。

建议拆分接口：

```c
void OLED_Clear(void);
void OLED_DisplayOn(void);
void OLED_DisplayOff(void);
void OLED_RecoverAndClear(void);
```

建议职责：

| 接口 | 职责 | 是否复位 OLED | 典型调用场景 |
| --- | --- | --- | --- |
| `OLED_Clear()` | 只设置窗口并写 0 清屏 | 否 | 普通页面刷新、菜单切页 |
| `OLED_DisplayOff()` | 发送 `0xAE` 关显示 | 否 | 息屏 |
| `OLED_DisplayOn()` | 发送 `0xAF` 开显示 | 否 | 唤醒 |
| `OLED_RecoverAndClear()` | 复位、初始化、清屏 | 是 | 上电、静电恢复、低频保底恢复 |

建议将 `all_screen()` 改为内部恢复函数或更名，避免继续作为普通清屏使用。

### 第四阶段：制定静电恢复策略

目标：不取消恢复能力，但降低正常刷新成本。

推荐策略：

- 上电初始化：调用 `OLED_RecoverAndClear()`；
- 从息屏唤醒：调用 `OLED_RecoverAndClear()`；
- 普通定时刷新：调用 `OLED_Clear()`；
- 低频保底恢复：每 30 秒或 60 秒调用一次 `OLED_RecoverAndClear()`；
- 现场静电问题严重时：恢复周期可配置或通过宏定义控制；
- 检测到 SPI 错误或 OLED 异常时：立即调用 `OLED_RecoverAndClear()`。

建议不要每 2 秒无条件复位 OLED。若现场验证发现必须高频恢复，再通过配置开关保留兼容策略。

### 第五阶段：SPI 发送状态和刷新超时检测

目标：把显示异常从“不可见”变成可统计、可恢复的健康状态。

检测点：

- `HAL_SPI_Transmit()` 返回 `HAL_ERROR` 或 `HAL_TIMEOUT`；
- SPI 长时间 `BUSY`；
- 一次整屏刷新耗时超过阈值；
- 连续多次刷新失败；
- 刷新任务被阻塞或长时间未完成。

建议改造：

1. 让 `WriteCommand()`、`WriteSingleData()` 返回 `HAL_StatusTypeDef`。
2. 不再无条件忽略 SPI 返回值。
3. 使用有限超时替代 `HAL_MAX_DELAY`。
4. 累计 `display_spi_error_count`、`display_refresh_timeout_count`。
5. 连续异常达到阈值后执行 `OLED_RecoverAndClear()`。

该方案只能识别通信和执行异常，不能确认玻璃显示是否已经花屏。

### 第六阶段：优化 SPI 发送

目标：降低清屏和绘字耗时。

短期优化：

- 清屏时准备 64 字节或 128 字节的全 0 buffer；
- 循环按块调用 `HAL_SPI_Transmit()`；
- 避免 5120 次单字节发送。

中期优化：

- 字符绘制按行或按字形批量发送；
- 减少重复设置行列窗口；
- 对状态页做局部刷新。

长期优化：

- 引入小 framebuffer 或 dirty row 标记；
- SPI DMA 刷新；
- 将显示任务做成非阻塞状态机。

### 第七阶段：真正实现息屏和唤醒

目标：息屏状态不再反复复位和清屏。

建议：

```c
void OLED_DisplayOff(void)
{
    WriteCommand(0xAE);
}

void OLED_DisplayOn(void)
{
    WriteCommand(0xAF);
}
```

息屏流程：

```text
到达息屏条件
  -> OLED_DisplayOff()
  -> 标记当前为息屏状态
```

唤醒流程：

```text
按键触发亮屏
  -> OLED_DisplayOn()
  -> OLED_RecoverAndClear()
  -> 请求刷新当前页面
```

如果 OLED 控制器在静电后可能无法响应普通开显示命令，唤醒时使用 `OLED_RecoverAndClear()` 是更稳妥的选择。

### 第八阶段：内部影子缓冲和软件一致性校验

目标：提高 CPU3 软件侧显示链路的可控性和可诊断性，为后续局部刷新做准备。

内部影子缓冲的含义是：CPU3 在自己的 RAM 中维护一份“当前屏幕理论上应该显示什么”的副本。由于 SSD1325 串行模式不能可靠读回 OLED 实际内容，CPU3 不能直接问屏幕当前显示是否正确，但可以校验 CPU3 侧的页面生成、刷新调度和待发送数据是否一致。

建议数据流：

```text
页面状态/测量数据
  -> Display_RenderToShadow()
  -> shadow buffer
  -> CRC/页号/刷新序号校验
  -> OLED_SendBuffer()
  -> SPI 状态和刷新耗时统计
```

SSD1325 的 GDDRAM 为 128 x 80 x 4 bit，每字节包含两个像素。如果按完整 GDDRAM 建缓冲，需要：

```text
128 x 80 / 2 = 5120 字节
```

如果只维护当前 128 x 64 可视区，理论上需要：

```text
128 x 64 / 2 = 4096 字节
```

考虑当前代码清屏按 80 行写入，建议先按 5120 字节完整窗口建缓冲，避免行偏移和控制器 RAM 区域不一致。

示例状态：

```c
uint8_t display_shadow[5120];
uint32_t display_shadow_crc;
uint32_t display_refresh_seq;
uint32_t display_last_refresh_ms;
```

可检测的问题：

- CPU3 页面生成逻辑异常，例如页面状态没变但 CRC 异常变化；
- 字符或数字绘制越界导致 shadow buffer 边界保护触发；
- 刷新任务长时间没有更新 `display_refresh_seq`；
- 待发送数据和页面状态不一致；
- 局部刷新或后续 dirty row 逻辑漏刷。

不可检测的问题：

- SPI 已成功发送，但 OLED 控制器被静电打乱；
- OLED 控制器内部扫描异常；
- OLED 玻璃真实显示花屏；
- 供电、排线或屏体硬件异常导致的显示错误。

实施建议：

- 不建议作为第一阶段整改项；
- 第一阶段先完成中断移出、SPI 错误检测和恢复接口拆分；
- 第二阶段再引入 shadow buffer；
- 对外描述时应写成“显示数据一致性校验”，不要写成“花屏识别”。

## 建议增加的状态变量

| 变量 | 含义 |
| --- | --- |
| `display_spi_error_count` | SPI 发送失败计数 |
| `display_refresh_timeout_count` | 刷新超时计数 |
| `display_recover_count` | OLED 恢复次数 |
| `display_last_refresh_ms` | 最近一次刷新耗时 |
| `display_last_recover_tick` | 最近一次恢复时间 |
| `display_health_flags` | 显示健康状态位 |
| `display_shadow_crc` | 当前 shadow buffer 的 CRC |
| `display_refresh_seq` | 显示刷新序号，用于判断刷新任务是否停滞 |
| `display_page_id` | 当前渲染页面类型或页面编号 |

这些变量可先用于调试日志，后续再决定是否映射到外部诊断寄存器。

## 建议实施顺序

| 顺序 | 工作项 | 影响范围 | 风险 |
| --- | --- | --- | --- |
| 1 | TIM3 中断只置刷新标志，主循环执行 `RefreshScreen()` | `stm32f4xx_it.c`、`display.c`、主循环 | 低 |
| 2 | EXTI/TIM1 按键路径只记录事件，主循环执行 `KeyProcess()` | `exit.c`、`stm32f4xx_it.c`、显示任务 | 中 |
| 3 | 拆分 `OLED_Clear()` 和 `OLED_RecoverAndClear()` | `hgs.c`、`hgs.h`、显示调用点 | 中 |
| 4 | SPI 发送增加返回值、有限超时和错误计数 | `hgs.c`、显示健康状态 | 中 |
| 5 | 普通刷新改用 `OLED_Clear()`，上电/唤醒/低频恢复用 `OLED_RecoverAndClear()` | `display.c`、`display_tankopera.c` | 中 |
| 6 | 清屏 SPI 改为块发送 | `hgs.c` | 低 |
| 7 | 增加真正息屏和唤醒策略 | `display.c`、`hgs.c` | 中 |
| 8 | 引入 shadow buffer、CRC、刷新序号 | 显示渲染层、刷新任务 | 中 |

建议先完成第 1 步。第 1 步不改变现有复位恢复策略，但能显著降低中断风险。

## 验证方案

### 1. 手册和驱动一致性

- 检查 SSD1325 初始化命令是否与 `HGS128645-Y-EH-LV` 手册一致。
- 检查行列地址、`0x0C` 行偏移和 `0x4B` 结束行是否符合当前屏实际显示区域。
- 检查 SPI 串行时序、D/C、CS、RESET 逻辑与手册接口定义一致。

### 2. 复位脚观测

使用示波器或逻辑分析仪观察 `OLED_NREST`。

修改前预期：

- 待机刷新期间约每 2 秒出现一次复位脉冲。

第一阶段修改后预期：

- 复位脉冲仍可能按现有刷新策略出现；
- 但不再发生在 TIM3 ISR 中。

完成清屏和恢复拆分后预期：

- 普通刷新不再触发 `OLED_NREST`；
- 上电、唤醒、低频恢复或异常恢复时才触发复位。

### 3. 中断耗时测量

在 TIM3 中断入口和出口翻转测试 GPIO。

修改前预期：

- TIM3 中断高电平持续时间包含整屏刷新耗时。

修改后预期：

- TIM3 中断只置位标志，持续时间显著缩短。

### 4. 通信和按键压测

测试场景：

- CPU2/CPU3 通信持续运行；
- 快速按键进入菜单、翻页、返回；
- 同时观察串口日志、页面刷新和测量状态。

预期：

- 无明显按键卡顿；
- 通信无新增超时；
- 菜单显示不乱跳；
- 状态页刷新节奏稳定。

### 5. 静电恢复测试

测试场景：

- 对 OLED 相关接口做现场既有静电干扰测试；
- 记录是否花屏；
- 记录健康检查是否触发恢复；
- 记录恢复耗时和恢复成功率；
- 记录花屏后能否在设定恢复周期内恢复。

预期：

- 普通刷新不会频繁复位；
- 静电导致花屏后，上电、唤醒、低频恢复或手动恢复能重新拉回正常显示；
- 恢复动作不在中断中执行；
- 恢复计数和错误计数可追踪。

### 6. 影子缓冲一致性测试

测试场景：

- 固定状态页数据，连续刷新并记录 `display_shadow_crc`；
- 改变液位、温度、状态页类型，确认 CRC 随显示内容变化；
- 人为构造长字符串或边界坐标，确认不会越界写缓冲；
- 暂停刷新任务，确认 `display_refresh_seq` 停滞可以被发现。

预期：

- 页面内容不变时 CRC 稳定；
- 页面内容变化时 CRC 可追踪；
- 刷新序号持续递增；
- 越界保护和刷新停滞可被诊断。

## 硬件侧建议

软件复位只能恢复花屏，不能减少被静电打乱的概率。建议同步检查：

- OLED 的 `SCK`、`MOSI`、`DC`、`CS/NSS`、`RESET` 是否有合适串阻；
- OLED 接口是否有 TVS 或 ESD 保护器件；
- OLED 排线地回流路径是否短且可靠；
- `RESET` 是否有稳定上拉或 RC 滤波；
- 外壳、屏蔽层和主板地的连接方式是否会把静电耦合到信号线上。

如果硬件上暂时无法调整，软件侧应保留 `OLED_RecoverAndClear()` 作为现场恢复兜底。

若要真正识别屏幕实际花屏，需要额外硬件或工装支持，例如：

- 屏幕边角测试区域 + 光敏二极管/光电传感器；
- 摄像头工装拍摄固定测试图案；
- 后续硬件改版时评估是否支持可靠显示读回。

## 风险和注意事项

- 拆分清屏和恢复后，必须确认普通 `OLED_Clear()` 不会降低静电恢复能力，因为恢复能力应由 `OLED_RecoverAndClear()` 承担。
- 按键事件从中断移到主循环后，要确认长按进入菜单和长按返回取消测量的时间语义不变。
- 如果主循环存在长时间阻塞，显示任务可能延后执行，需要同步检查主循环调度。
- SPI 批量发送时要确认 OLED 控制器对 `DC` 和 `NSS/CS` 时序的要求。
- 当前 SPI1 使用硬件 NSS。如果 OLED 对片选连续性敏感，后续可结合示波器确认是否需要改为软件片选。
- 影子缓冲只能校验 CPU3 软件侧显示数据一致性，不能证明 OLED 玻璃上的真实显示正确。

## 结论

当前 CPU3 固件匹配 `HGS128645-Y-EH-LV / SSD1325 OLED`。当前 OLED 复位恢复是为了解决静电花屏，方向可以保留。真正需要整改的是：

1. 不要在中断中执行重绘、延时和阻塞 SPI；
2. 不要把普通清屏和异常恢复混在同一个 `oled_clear()` 中；
3. 不要让息屏状态反复复位和清空 OLED；
4. 增加 SPI 发送错误、刷新超时、恢复次数等显示健康状态；
5. 后续用 shadow buffer 做软件一致性校验，但不要把它等同于真实花屏识别；
6. 逐步降低 SPI 发送开销。

建议先实施“TIM3 只置刷新标志，主循环执行刷新”。这是最小、最稳妥的第一步，能保留现有抗静电恢复逻辑，同时降低系统实时性风险。

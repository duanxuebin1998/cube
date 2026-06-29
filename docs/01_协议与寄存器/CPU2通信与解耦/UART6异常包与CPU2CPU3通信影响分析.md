# UART6 异常包与 CPU2/CPU3 通信影响整理

## 1. 问题背景

现场日志中曾出现较多 UART6 通信失败，例如液位稳定或液位跟随日志后面粘着：

```text
[UART6] 通信失败，重试 x/3
```

这类日志容易被理解成“液位稳定状态直接触发 UART6 错误”。结合当前代码看，原始现象更可能由两类因素叠加：

1. 液位日志没有换行，下一轮 UART6 传感器读取失败日志被粘到同一行。
2. CPU2 在 UART5 中断中处理 CPU3 的 Modbus 请求，若中断占用时间过长，会影响主循环对 UART6 的 9600 波特率前台轮询接收。

本文按当前代码重新整理：哪些风险已经处理，哪些仍需要继续做。

## 2. 当前通信结构

### 2.1 UART6 传感器通信

位置：

- `LTD_MAIN_CPU2/Core/Src/usart.c`
- `LTD_MAIN_CPU2/Services/Sensor/dsm_sensor_communication.c`
- `LTD_MAIN_CPU2/Services/Sensor/ltd_sensor_communication.c`
- `LTD_MAIN_CPU2/Services/Sensor/wireless_pairing.c`

当前 USART6 仍是 `9600 8N1`，传感器通信仍使用前台逐字节轮询：

```c
HAL_UART_Receive(&huart6, &byte, 1, 1)
```

9600 8N1 下，1 字节线上时间约为：

```text
10 bit / 9600 = 1.04 ms
```

因此 UART6 对 1ms 级 CPU 占用比较敏感。若主循环被其它中断或阻塞流程打断，可能出现接收超时、短帧、BCC 错误或 ORE 溢出。

### 2.2 CPU2/CPU3 通信

位置：

- `LTD_MAIN_CPU2/Core/Src/stm32f4xx_it.c`
- `LTD_MAIN_CPU2/Services/Modbus/hostcommu.c`
- `LTD_MAIN_CPU2/Services/Modbus/hostcommu_modbus.c`
- `LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c`

CPU2 和 CPU3 使用 UART5 DMA 收发，但当前 UART5 IDLE 中断中仍直接调用：

```c
HostCommuProcess(UART5_RX_BUF, UART5_RX_LEN);
```

也就是说，DMA 只减少字节搬运成本，并没有把 Modbus 解析、寄存器映射、CRC、组包和发送启动从中断中移出去。

## 3. 已解决或已缓解的问题

### 3.1 已解决：CRC 错误路径不再在中断内大量打印

位置：`LTD_MAIN_CPU2/Services/Modbus/hostcommu.c`

当前 `HostCommuProcess()` 对长度、地址、CRC、功能码、寄存器范围等异常，已改为调用 `HostCommu_RecordDeferredLog()` 记录必要字段。

`HostCommu_ProcessDeferredLogs()` 在主循环中输出统一错误日志，并且同类异常有 1 秒限频和抑制计数。

结论：文档中原先提到的“CRC 错误帧在 UART5 中断里多行阻塞打印”已基本解决。

### 3.2 已解决：04 输入寄存器响应不再实时读取 TMC5130

位置：`LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c`

当前 `write_measurement_result_to_InputRegisters()` 在写电机状态时只读取缓存：

```c
write_u32_to_regs(regs, REG_DEBUG_MOTOR_STATE, g_measurement.debug_data.motor_state);
```

代码注释也明确说明：Modbus 读输入寄存器可能发生在 UART5 中断上下文，不能在这里实时读取 TMC5130；电机状态由主循环 `MotorCtrl_PollRuntimePosition()` 统一刷新。

结论：CPU3 每次读 04 输入寄存器触发 TMC5130 SPI 读取的风险已经处理。

### 3.3 已缓解：UART6 通信重试次数已统一

位置：

- `LTD_MAIN_CPU2/Services/Sensor/sensor.h`
- `LTD_MAIN_CPU2/Services/Sensor/dsm_sensor_communication.c`
- `LTD_MAIN_CPU2/Services/Sensor/ltd_sensor_communication.c`
- `LTD_MAIN_CPU2/Services/Sensor/wireless_pairing.c`

当前统一使用：

```c
#define UART6_COMM_MAX_RETRY 3U
```

DSM 文本协议、LTD/V2 二进制协议、无线主从参数读取都按 3 次重试收敛，避免旧逻辑中较多重试拖慢测量流程。

结论：UART6 错误后长时间阻塞测量流程的风险已缓解，但这不是接收可靠性的根治。

### 3.4 已缓解：UART6 发送前会清残留并清 ORE

当前 DSM/LTD/无线通信发送前都会调用类似 `UART6_DrainRX_UntilIdle()` 的清理逻辑，并执行：

```c
__HAL_UART_CLEAR_OREFLAG(&huart6);
```

结论：历史残留字节和悬挂 ORE 对下一帧的影响已有缓解；但运行中 ORE/FE/NE 的分类统计仍不完整。

### 3.5 已解决：TMC5130 XACTUAL 读取可靠性已有保护

相关问题虽然不是本文主线，但它会影响“CPU3 轮询期间电机通信是否被放大异常”。

当前 TMC5130 `XACTUAL` 已有 CS guard、多次稳定读取和上层跳变过滤，异常 `XACTUAL` 直接污染电机位置的风险已明显降低。

结论：电机侧读数异常问题已收口，但 UART5 中断长时间占用 CPU 仍可能拉长前台电机控制流程。

## 4. 仍未解决的问题

### 4.1 UART5 中断仍直接执行完整 HostCommuProcess

位置：`LTD_MAIN_CPU2/Core/Src/stm32f4xx_it.c`

当前 UART5 IDLE 中断仍在停止 DMA、计算长度后直接执行：

```c
HostCommuProcess(UART5_RX_BUF, UART5_RX_LEN);
```

这是本文最大的剩余结构风险。只要 CPU3 高频轮询、写参数或发送异常帧，CPU2 仍会在 UART5 中断上下文里执行较多业务逻辑，主循环在这段时间内无法继续轮询 UART6。

状态：未解决。

### 4.2 0x10 写保持寄存器路径仍有阻塞 printf

位置：`LTD_MAIN_CPU2/Services/Modbus/hostcommu_modbus.c`

当前仍存在：

```c
printf("command: %lu\r\n", (unsigned long)g_deviceParams.command);
printf("HoldingRegisterArray[%d] = %d\n", i, HoldingRegisterArray[i]);
```

`PresetRegister()` 会对写入范围内每个寄存器打印一行。CPU2 的 `printf` 最终走 USART1 阻塞发送，如果该路径在 UART5 中断内执行，0x10 写多个寄存器时很容易超过 1ms。

状态：未解决，建议优先处理。

### 4.3 UART6 仍是前台 1ms 轮询接收

位置：`LTD_MAIN_CPU2/Services/Sensor/*communication.c`

当前没有看到 USART6 DMA/IDLE 或 RXNE 环形缓冲接收改造，仍是主流程里反复 `HAL_UART_Receive(..., 1, 1)`。

状态：未解决。只要存在 1ms 级以上的中断占用，UART6 仍可能丢字节或形成短帧。

### 4.4 DSM 帧格式校验仍不够严格

位置：`LTD_MAIN_CPU2/Services/Sensor/dsm_sensor_communication.c`

通用 `UART6_SendCommand()` 仍主要检查：

- `recvLen == 0`
- `recvLen < 3`
- BCC

部分业务函数有格式检查，但 `Read_Water_Capacitance()` 中长度、帧头、结尾、BCC、电压异常等检查仍被注释掉，没有真正返回错误。

状态：未解决。短帧只要 BCC 对上，仍可能被后续解析逻辑当作有效数据。

### 4.5 UART6 错误原因统计仍不完整

当前错误码已经能区分超时、短帧格式错误、BCC 错误、设备返回错误等，但还缺少面向现场定位的统计闭环：

- USART6 ORE/FE/NE 分类计数。
- 收到的原始 HEX 帧按错误类型记录。
- 超时、短帧、BCC 错误、格式错误的累计计数。
- 与 CPU3 轮询频率、电机运行状态的关联记录。

状态：部分缓解，仍需完善。

### 4.6 HostCommuProcess 实际耗时仍未量化

文档原先建议用 GPIO 翻转或 DWT 计数测量 `HostCommuProcess()`，当前代码中未看到对应耗时统计。

状态：未解决。没有实测数据时，只能根据代码路径判断风险，无法确认 03/04/0x10/异常帧的最大耗时。

## 5. 当前优先级建议

### P0：先移除 UART5 中断路径中的阻塞打印

优先处理：

1. `Response10Process()` 的 `command` 打印。
2. `PresetRegister()` 中每个保持寄存器一行的打印。
3. 其它可能在 `HostCommuProcess()` 路径内执行的非调试宏保护打印。

建议改为调试宏保护、计数、延后日志，或完全移除普通调试打印。

### P0：补 UART6 错误原因分类和原始帧记录

建议至少区分：

1. 发送失败。
2. 接收超时。
3. 短帧。
4. BCC 错误。
5. 帧格式错误。
6. 设备返回错误。
7. USART6 ORE/FE/NE。

现场排查时再结合 CPU3 轮询状态、电机运行状态一起看。

### P0：加强 DSM 帧格式校验

对液位频率、电容、水位等响应，应按协议固定校验：

- 帧头。
- 固定长度。
- 数字位数和小数点位置。
- BCC 固定位置。
- `\r\n` 结尾。
- 合理数值范围。

校验失败时不能更新液位频率或测量状态。

### P1：测量 HostCommuProcess 最大耗时

建议用 GPIO 或 DWT，不要在中断里 `printf`。

分别统计：

- 03 读保持寄存器。
- 04 读输入寄存器。
- 0x10 写保持寄存器。
- CRC/地址/长度异常帧。

判断标准：

- `<0.5ms`：对 UART6 风险较低。
- `0.5~1ms`：边界风险。
- `>1ms`：对 USART6 9600 前台轮询有明显风险。
- `>2ms`：很可能导致 UART6 丢字节、短帧或超时。

### P1：将 HostCommuProcess 移出 UART5 中断

中期结构建议：

- UART5 IDLE 中断只停止 DMA、记录长度、复制帧或置事件标志。
- 主循环或通信任务中调用 `HostCommuProcess()`。
- 中断内禁止业务解析、参数保存请求、寄存器映射和阻塞打印。

这是从结构上降低 UART5 对 UART6 和前台电机控制影响的关键改造。

### P2：评估 UART6 DMA/IDLE 或 RXNE 环形缓冲

如果现场确认 UART6 仍因主循环被打断而短帧，建议把 USART6 改为：

- DMA + IDLE 接收；或
- RXNE 中断 + 环形缓冲，主循环解析完整帧。

这样即使主循环短时间被打断，也不容易丢 UART6 字节。

## 6. 当前结论

目前已经解决了几项高风险点：CRC 错误路径不再在 UART5 中断里大量打印，04 输入寄存器响应不再实时访问 TMC5130，UART6 重试次数已统一收敛，发送前残留清理和 ORE 清除也已存在。

但根因级结构风险仍在：UART5 中断仍直接执行完整 `HostCommuProcess()`，0x10 写寄存器路径仍有阻塞 `printf`，UART6 仍是 9600 波特率前台 1ms 轮询接收，DSM 帧格式校验也还不够严格。

下一步建议先处理 P0：移除 UART5 中断路径里的阻塞打印、补 UART6 错误分类、加强 DSM 帧格式校验。完成后再测量 `HostCommuProcess()` 实际耗时，并决定是否推进“HostCommuProcess 移出中断”和“UART6 DMA/IDLE 接收”。

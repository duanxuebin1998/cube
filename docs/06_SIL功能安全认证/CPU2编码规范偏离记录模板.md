# CPU2 编码规范偏离记录模板

日期：2026-06-02

用途：记录 CPU2 固件中暂时无法满足 MISRA C:2012、CERT C 或项目强制规则的代码项，并给出安全影响、缓解措施、验证证据和批准状态。

## 1. 偏离记录使用原则

偏离记录不是放弃整改。只有在以下场景中允许提交偏离：

- 芯片厂商 HAL/CMSIS 或启动代码无法修改。
- 硬件寄存器访问需要 `volatile`、指针转换或特定地址映射。
- 协议序列化、CRC、FRAM 持久化需要受控字节级访问。
- 测量算法暂时保留浮点，且已有范围检查和验证证据。
- 历史代码整改风险高于保留风险，且已有后续整改计划。
- 安全机构、项目负责人或功能安全负责人批准的特殊场景。

以下情况不得用偏离记录绕过：

- 新增代码未做空指针检查。
- 未校验外部输入直接控制电机运动。
- 安全状态切换依赖打印或人工观察。
- 已知可能越界的数组访问。
- 未处理硬件访问返回值。
- 没有验证证据的隐式缩窄转换。

## 2. 偏离状态

| 状态 | 含义 |
| --- | --- |
| 开放 | 已发现违反项，尚未完成分析或批准 |
| 待批准 | 已完成分析，等待负责人批准 |
| 已批准 | 允许在当前版本保留，并有缓解措施 |
| 整改中 | 已批准整改计划，正在修改或验证 |
| 已关闭 | 违反项已修复，或代码已移除 |
| 作废 | 记录不再适用，例如规则范围调整或误报 |

## 3. 偏离记录模板

```text
Deviation ID:
标题:
状态:
发现日期:
适用版本:

规则来源:
- MISRA C:2012:
- CERT C:
- CPU2 项目规则:

文件:
行号:
函数/符号:

代码位置说明:

违反描述:

偏离原因:

安全影响分析:

缓解措施:

验证证据:

替代方案:

整改计划:

批准人:
批准日期:
复审日期:
关闭日期:
备注:
```

## 4. 示例记录

```text
Deviation ID: CPU2-DEV-0001
标题: HAL 初始化代码中的寄存器访问保留厂商实现
状态: 待批准
发现日期: 2026-06-02
适用版本: CPU2 V1.9.1.0

规则来源:
- MISRA C:2012: 指针和 volatile 相关规则
- CERT C: EXP / CON 相关规则
- CPU2 项目规则: CUBE-C-011 全局变量必须受控

文件: LTD_MAIN_CPU2/Core/Src/gpio.c
行号: 待静态分析工具确认
函数/符号: MX_GPIO_Init

代码位置说明:
CubeMX 生成的 GPIO 初始化代码，包含 HAL 宏和寄存器配置。

违反描述:
静态分析工具可能报告寄存器访问、宏展开或外部对象访问违反编码规范。

偏离原因:
该代码来自 ST HAL/CubeMX 生成逻辑，直接修改会影响后续 CubeMX 维护和厂商支持。

安全影响分析:
该代码参与硬件初始化。安全影响取决于 GPIO 默认电平和电机驱动使能脚配置。必须通过启动安全态测试确认输出默认安全。

缓解措施:
- 固定 CubeMX 和 HAL 版本。
- 对安全相关 GPIO 单独列出初始化期望。
- 启动阶段执行 MotorCtrl_BootSafeStop。
- 后续安全输出 GPIO 增加上电默认态验证。

验证证据:
- CPU2 构建通过。
- 上电安全态测试记录。
- GPIO 默认态检查表。

替代方案:
将安全输出 GPIO 初始化封装为项目自有安全初始化函数。

整改计划:
在安全发布构建中补充安全输出初始化验证，不直接修改 HAL 生成代码。

批准人:
批准日期:
复审日期:
关闭日期:
备注:
```

## 5. 首批偏离分类建议

### 5.1 厂商代码

适用文件：

- `LTD_MAIN_CPU2/Drivers/STM32F4xx_HAL_Driver/**`
- `LTD_MAIN_CPU2/Drivers/CMSIS/**`
- `LTD_MAIN_CPU2/Core/Startup/startup_stm32f429zgtx.s`

处理策略：

- 不直接整改。
- 固定版本。
- 建立供应商资料清单。
- 对安全相关初始化做外部验证。

### 5.2 硬件寄存器和外设驱动

适用文件：

- `LTD_MAIN_CPU2/Drivers/Peripherals/src/TMC5130.c`
- `LTD_MAIN_CPU2/Drivers/Peripherals/src/AS5145.c`
- `LTD_MAIN_CPU2/Drivers/Peripherals/src/mb85rs2m.c`
- `LTD_MAIN_CPU2/Drivers/Peripherals/src/ad5421.c`

处理策略：

- 指针转换和 volatile 访问允许提交偏离。
- 通信返回值、超时和边界检查必须整改，不建议偏离。

### 5.3 协议和持久化字节访问

适用文件：

- `LTD_MAIN_CPU2/Services/Modbus/**`
- `LTD_MAIN_CPU2/Services/ParamStorage/**`
- `LTD_MAIN_CPU2/Services/Utilities/my_crc.c`

处理策略：

- 字节序转换、`memcpy` 和 CRC 计算可提交受控偏离。
- 寄存器地址、长度、外部输入边界必须整改。

### 5.4 测量算法浮点

适用文件：

- `LTD_MAIN_CPU2/Application/Src/measure_density.c`
- `LTD_MAIN_CPU2/Application/Src/measure_waterLevel.c`
- `LTD_MAIN_CPU2/Application/Src/measure_oilLevel.c`
- `LTD_MAIN_CPU2/Application/Src/wartsila_density_measurement.c`
- `LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_position_model.c`

处理策略：

- 测量计算可暂时保留浮点。
- 安全阈值判断、限位、停机条件优先改定点。
- 保留浮点必须写明单位、范围、误差和测试证据。

### 5.5 调试打印

适用文件：

- `LTD_MAIN_CPU2/Application/Src/test.c`
- 包含大量 `printf` 的业务模块。

处理策略：

- 安全发布版隔离测试文件。
- 安全路径禁止直接打印。
- 普通诊断打印转为延迟日志或编译开关控制。

## 6. 偏离审批流程

1. 开发者记录违反项和初步原因。
2. 模块负责人确认是否可整改。
3. 安全负责人判断是否影响安全功能。
4. 测试负责人确认验证证据。
5. 项目负责人批准保留或要求整改。
6. 每个发布版本复审开放偏离。

## 7. 偏离记录编号规则

编号格式：

```text
CPU2-DEV-YYYY-NNN
```

示例：

```text
CPU2-DEV-2026-001
```

编号不得复用。已关闭或作废的记录保留在历史清单中。

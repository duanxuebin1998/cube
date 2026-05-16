# LTD_MAIN_CPU2 解耦实施详细计划（CubeMX 兼容版，12 周）

## 1. 背景与核心约束

> 重要约束：`Core/`、`Drivers/`、`*.ioc` 及其目录结构属于 CubeMX 自动生成资产。**本计划不调整这些目录层级，不重命名，不搬迁**，仅在 `USER CODE` 区域做最小改动。

### 1.1 目标
- 在不破坏 CubeMX 再生成能力的前提下完成解耦。
- 保持现有协议与测量行为等价（Modbus 03/04/10、关键测量命令）。
- 将“业务逻辑、硬件访问、参数状态管理”逐步分离，降低后续维护成本。

### 1.2 范围
- 仅限 `LTD_MAIN_CPU2/`。
- 不改 `LTD_DISPLAY_CPU3/`。
- 不引入 RTOS。

### 1.3 完成标准
- 中断函数不直接执行业务处理（改为事件投递 + 主循环消费）。
- 新增业务模块不再直接依赖 HAL 头文件。
- `g_deviceParams / g_measurement` 的写入口收敛到统一服务。
- 每阶段可独立回退，且持续可编译。

---

## 2. 当前痛点（基于现状）

1. `Application/Src/app_main.c` 职责过重（初始化 + 指令处理 + 状态推进）。
2. `Application/Src/measure.c` 大型 `switch` 既做路由又做执行。
3. `Core/Src/stm32f4xx_it.c` 存在 ISR 直接调用业务处理。
4. `Services/ParamStorage/system_parameter.h` 全局状态可被多处直接写。
5. `Services/Modbus/hostcommu.h` 通过宏直接绑定 RS485 GPIO 方向控制。

---

## 3. 目标形态（不改 CubeMX 目录结构）

不新增顶层 `Domain/Ports/Adapters` 目录，避免对工程结构产生“看起来像重构目录”的风险；
改为在**现有 `Application/` 与 `Services/` 内分层**：

```text
LTD_MAIN_CPU2/
├─ Core/                        # CubeMX 自动生成，目录结构保持不变
├─ Drivers/                     # CubeMX 自动生成，目录结构保持不变
├─ Application/
│  ├─ Inc/
│  └─ Src/
│     ├─ app_main.c             # 仅保留入口编排
│     ├─ app_runner.c           # 新增：主循环调度
│     └─ app_event_queue.c      # 新增：事件队列
└─ Services/
   ├─ AppLayer/                 # 新增：命令路由/用例编排（纯业务）
   ├─ DomainLogic/              # 新增：状态机、规则、参数校验（纯业务）
   ├─ PortInterfaces/           # 新增：抽象接口（.h 为主）
   ├─ AdapterImpl/              # 新增：对 Motor/Sensor/Comm/Storage 的具体适配
   └─ 现有子模块/              # 逐步作为底层实现被适配器调用
```

### 3.1 依赖方向
- `Core(中断/外设)` -> `Application(app_runner/event)`
- `Application` -> `Services/AppLayer`
- `AppLayer` -> `DomainLogic + PortInterfaces`
- `AdapterImpl` -> 现有 `Services/* + HAL`
- `DomainLogic` 不直接依赖 HAL

---

## 4. 实施策略（12 周，6 阶段）

## 阶段 0（第 1 周）：基线冻结 + CubeMX 安全边界

### 任务
- [ ] 新增 `LTD_MAIN_CPU2/docs/decoupling_baseline.md`：记录命令行为、状态流转、关键帧样例。
- [ ] 新增 `LTD_MAIN_CPU2/docs/cubemx_edit_boundary.md`：定义“可改文件与不可改区域”。
- [ ] 约定改动规则：
  - `Core/*`、`Drivers/*` 仅允许 `/* USER CODE BEGIN/END */` 内修改。
  - 禁止修改 CubeMX 生成函数签名（如 `MX_*_Init`）。
  - 禁止移动 `Core`/`Drivers`/`Startup` 目录。

### 验收
- 评审通过“边界文档”，团队成员对改动红线一致。

---

## 阶段 1（第 2-3 周）：入口瘦身（不改目录）

### 任务
- [ ] 新增 `Application/Src/app_runner.c` + `Application/Inc/app_runner.h`。
- [ ] `app_main.c` 保留初始化逻辑，循环里仅调用 `App_RunOnce()`。
- [ ] 将调试命令文本解析从 `measure.c` 拆到 `Services/AppLayer/debug_command_handler.c`。
- [ ] 建立 `AppContext`（放在 `Services/AppLayer/`），集中保存运行态引用。

### 验收
- `app_main.c` 只负责“启动 + 调度”。
- 功能无回退。

---

## 阶段 2（第 4-5 周）：中断事件化（重点兼容 CubeMX）

### 任务
- [ ] 新增 `Application/Src/app_event_queue.c`（单生产者 ISR / 单消费者主循环环形队列）。
- [ ] 在 `Core/Src/stm32f4xx_it.c` 的 `USER CODE` 区域：
  - UART5 只采样长度、投递 `EVENT_UART5_FRAME_READY`。
  - USART1 只投递 `EVENT_DEBUG_CMD_READY`。
- [ ] 在 `App_RunOnce()` 消费事件，调用 `Services/AppLayer/comm_dispatcher.c`。

### 验收
- ISR 中不再直接调用 `HostCommuProcess(...)`（改为主循环调用）。
- 通信回归通过。

---

## 阶段 3（第 6-7 周）：参数受控写入

### 任务
- [ ] 新增 `Services/DomainLogic/param_service.c/.h`。
- [ ] 定义接口：
  - `ParamService_SetPendingCommand(...)`
  - `ParamService_UpdateDeviceState(...)`
  - `ParamService_UpdateMeasurement(...)`
  - `ParamService_PersistIfNeeded(...)`
- [ ] 先替换 `app_main.c` 与 `measure.c` 高频写入点，再逐模块推进。

### 验收
- 新代码不再直接写 `g_deviceParams/g_measurement`。
- 全局状态变更有统一入口与日志点。

---

## 阶段 4（第 8-9 周）：接口抽象（目录内演进）

### 任务
- [ ] 在 `Services/PortInterfaces/` 定义：
  - `motor_port.h`、`sensor_port.h`、`comm_port.h`、`storage_port.h`。
- [ ] 在 `Services/AdapterImpl/` 实现：
  - `motor_port_adapter.c`（调用现有 `MotorControl`）
  - `sensor_port_adapter.c`（调用现有 `Sensor`）
  - `comm_port_adapter.c`（调用现有 `Modbus`）
  - `storage_port_adapter.c`（调用现有 `ParamStorage/FRAM`）
- [ ] 修改 `measure_*` 用例优先依赖 `PortInterfaces`。

### 验收
- 新增业务代码只 include `PortInterfaces/*.h`。
- 旧模块仍可保留，迁移可分批进行。

---

## 阶段 5（第 10-11 周）：命令路由从 switch 迁移到注册表

### 任务
- [ ] 新增 `Services/AppLayer/command_registry.c/.h`。
- [ ] 拆分 handler：
  - `command_handler_basic.c`
  - `command_handler_level.c`
  - `command_handler_density.c`
- [ ] 增加统一执行包装：前置检查、错误映射、后置状态提交。

### 验收
- 新增命令只需加 handler + registry 映射。
- `measure.c` 体积和复杂度明显下降。

---

## 阶段 6（第 12 周）：收口与防退化

### 任务
- [ ] 更新 `LTD_MAIN_CPU2/CMakeLists.txt`：
  - 维持现有顶层结构不变，仅补充新模块源文件。
  - 不引入会影响 CubeMX 再生成的目录依赖。
- [ ] 新增边界检查脚本（基于 `rg`）：
  - 检查 `Application/` 与 `Services/DomainLogic/` 是否误 include HAL 头。
  - 检查 `Core/*.c` 非 USER CODE 区域是否被修改（通过 diff 规则）。
- [ ] 文档收口：架构图、调试流程、常见回滚步骤。

### 验收
- 连续构建稳定。
- CubeMX 再生成后，手工改动可最小化冲突保留。

---

## 5. 文件级优先改造清单

### 第一批（高优先）
- `Application/Src/app_main.c`（瘦身）
- `Core/Src/stm32f4xx_it.c`（仅 USER CODE 区域事件化）
- `Application/Src/measure.c`（路由与执行拆分）
- `Services/Modbus/hostcommu.c/.h`（RS485 控制抽象）
- `Services/ParamStorage/system_parameter.c/.h`（受控访问入口）

### 第二批（跟进）
- `Services/MotorControl/*`（适配器封装）
- `Services/Sensor/*`（适配器封装）
- `Drivers/Peripherals/*`（仅作为底层依赖，不让上层直接 include）

---

## 6. 例行检查（每周）

- [ ] `cmake -S LTD_MAIN_CPU2 -B build/LTD_MAIN_CPU2 -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug`
- [ ] `cmake --build build/LTD_MAIN_CPU2`
- [ ] `arm-none-eabi-size build/LTD_MAIN_CPU2/LTD_MAIN_CPU2.elf`
- [ ] `git diff -- Core Drivers`（确保仅 USER CODE 允许改动）

---

## 7. 风险与回滚

1. **CubeMX 再生成覆盖手工改动**
   - 应对：仅在 USER CODE 区域修改，所有逻辑尽量外置到 `Application/` 与 `Services/` 新文件。

2. **事件队列导致丢帧**
   - 应对：增加溢出计数与告警，队列容量按峰值 2~4 倍配置。

3. **重构过深导致联调停滞**
   - 应对：每阶段单独合并；保留旧路径开关，支持快速回滚。

---

## 8. 本周可执行的 3 件事

1. 补齐 `decoupling_baseline.md` 与 `cubemx_edit_boundary.md`。
2. 引入 `App_RunOnce()`，先不改业务行为。
3. 在 `stm32f4xx_it.c` 先做 UART5 事件投递骨架（仅 USER CODE）。

> 这版计划的核心是：**解耦可以做，但必须把 CubeMX 目录和自动生成机制当成硬约束来设计。**

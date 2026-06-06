# CPU3 参数菜单与故障自动恢复次数说明

## 适用版本

- 引入版本：CPU2 `V1.10.0.0` / CPU3 `V1.9.0.0` / `DEVICE_PROTOCOL_VERSION = 6`
- 当前主线：CPU2 `V1.12.0.0` / CPU3 `V1.10.0.0` / `DEVICE_PROTOCOL_VERSION = 7`

本文只整理本次新增的“故障自动恢复次数”参数在协议、参数表和 CPU3 菜单中的对应关系。完整协议演进见 `docs/01_协议与寄存器/CPU2_CPU3协议变更记录.md`。

## 菜单入口

| 项目 | 内容 |
| --- | --- |
| 菜单路径 | 主菜单 -> 参数配置 -> 基础信息 -> 自动恢复次数 |
| 中文显示 | 自动恢复次数 |
| 英文显示 | AutoRecover |
| 写权限 | 可写 |
| 显示范围 | `0~10` |
| 默认值 | `3` |

该菜单项来自 CPU3 `param_meta[]`，不新增独立菜单页面。菜单生成逻辑会过滤名称仍为“保留”的参数项；本参数已从“保留2”改为“自动恢复次数”，因此会在基础信息分组中显示。

## 参数语义

| 参数值 | 行为 |
| --- | --- |
| `0` | 关闭故障自动恢复后的原命令自动重跑 |
| `1~10` | 故障恢复确认成功后，最多自动重跑原命令的次数 |
| `>10` | CPU2 运行时归一化为默认值 `3` |

自动恢复确认仍按原策略每 1 秒读取一次部件参数。该参数只限制“恢复确认成功后重新执行原测量命令”的次数，不限制恢复确认阶段的部件参数读取。

## 协议和寄存器映射

| 层级 | 名称 |
| --- | --- |
| CPU2/CPU3 参数字段 | `fault_auto_recovery_retry_limit` |
| 旧字段位置 | 原 `reserved2` |
| CPU3 菜单操作码 | `COM_NUM_DEVICEPARAM_RESERVED2` |
| 保持寄存器宏 | `HOLDREGISTER_DEVICEPARAM_FAULT_AUTO_RECOVERY_RETRY_LIMIT` |
| 兼容别名 | `HOLDREGISTER_DEVICEPARAM_RESERVED2` |
| 地址关系 | `HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION + REG_STRIDE` |
| 寄存器长度 | 2 个保持寄存器，按 `uint32_t` 读写 |

本次不新增保持寄存器地址，也不移动后续参数地址。为了避免已有代码或旧文档中的 `RESERVED2` 名称立即断裂，保持寄存器宏保留 `HOLDREGISTER_DEVICEPARAM_RESERVED2` 作为新宏的别名。

## 同步链路

1. CPU3 菜单项“自动恢复次数”通过 `COM_NUM_DEVICEPARAM_RESERVED2` 定位参数。
2. CPU3 `device_param_sync.c` 将该操作码映射到 `g_deviceParams.fault_auto_recovery_retry_limit`。
3. CPU3 内部 Modbus 打包时使用 `HOLDREGISTER_DEVICEPARAM_FAULT_AUTO_RECOVERY_RETRY_LIMIT`。
4. CPU2 内部 Modbus 解包后写入 `g_deviceParams.fault_auto_recovery_retry_limit`。
5. CPU2 参数保存流程将该字段写入 FRAM。
6. CPU2 故障自动恢复模块读取该字段决定是否进入自动恢复重跑，以及最多重跑次数。

## 升级兼容

- 旧协议存储中该位置是 `reserved2`，常见值可能为 `0`。
- CPU2 从旧协议升级到协议版本 6 时，会把该字段补为默认值 `3`，避免旧预留值 `0` 被误解释为关闭自动恢复。
- 启动归一化时，如果读取到大于 `10` 的异常值，会恢复为默认值 `3`。
- 当前主线仍保留该参数语义；CPU2/CPU3 必须使用相同 `DEVICE_PROTOCOL_VERSION`，协议版本 7 还包含后续四路继电器方式2共享字段。

## 联调检查

| 检查项 | 预期 |
| --- | --- |
| CPU3 菜单显示 | 基础信息中显示“自动恢复次数” |
| 菜单写入 `0` | CPU2 后续测量失败后不进入自动恢复重跑 |
| 菜单写入 `1` | 恢复确认成功后最多自动重跑 1 次 |
| 菜单写入 `3` | 保持默认最多自动重跑 3 次 |
| 异常写入 `>10` | CPU2 启动或加载参数后归一化为 `3` |
| 字库检查 | `py LTD_DISPLAY_CPU3\font_check.py` 无缺字 |

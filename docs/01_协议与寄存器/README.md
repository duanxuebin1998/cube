# 协议与寄存器文档索引

更新日期：2026-06-13

本目录用于保存 CPU2/CPU3 共享协议、外部 Modbus/DSM/SI7000 适配、寄存器表、协议版本和兼容性记录。凡是会影响通信地址、字段语义、命令、状态、缩放、补码解释或协议兼容性的资料，优先归入本目录。

## 分类

| 分类 | 资料 | 用途 |
| --- | --- | --- |
| CPU2/CPU3 共享协议 | `CPU2_CPU3协议变更记录.md` | 记录 `DEVICE_PROTOCOL_VERSION`、共享寄存器、参数语义、兼容影响和验证结果；当前共享协议版本为 `9` |
| 系统参数默认值 | `系统参数出厂默认值.md` | 整理 CPU2 恢复出厂写入的 `DeviceParameters` 默认值、单位和小数位；当前 CPU2 参数存储版本为 `3` |
| DSM 外部协议 | `DSM寄存器说明V1.225.xlsx` | DSM 寄存器表、外部协议字段和现场联调参考 |
| SI7000 协议适配 | `SI7000协议适配/` | SI7000 原始资料、需求、映射、改动记录和 PLC 联调检查 |

## 近期协议/参数口径

| 主题 | 当前结论 | 依据 |
| --- | --- | --- |
| 共享协议版本 | `DEVICE_PROTOCOL_VERSION = 9`，CPU2/CPU3 严格相等才兼容 | `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h`、`LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h`、`CPU2_CPU3协议变更记录.md` |
| CPU2 参数存储 | `DEVICE_PARAM_VERSION = 3`，CPU2 `V1.12.0.0` 到当前 `V1.15.0.0` 之间通常不清参数 | `LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.c`、`../00_构建与版本/CPU2参数存储升级清单.md` |
| CPU3 本地显示/通信参数 | `CPU3_PARAM_VERSION = 0x0004`，V3 升级时补默认屏幕亮度并写回 V4 | `LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.c`、`../00_构建与版本/版本改动与测试/2026-06-11_CPU3_V1.12.0.0_屏幕亮度配置与OLED刷新恢复_改动与测试方案.md` |
| 读取部件参数与 DSM 调试区 | 共享协议为 9；`debug_data.water_capacitance_x10` 为水位电容快照，蓝牙 RSSI 通过 `WirelessPairingStatus` 尾部输入寄存器发布 | `../00_构建与版本/版本改动与测试/2026-06-13_CPU2_V1.15.0.0_CPU3_V1.14.0.0_读取部件参数蓝牙RSSI与CPU3菜单显示优化_改动与测试方案.md` |

## 维护规则

- 修改 CPU2/CPU3 共享协议、共享参数语义、寄存器长度、命令码或跨 CPU 状态含义时，同步更新 `CPU2_CPU3协议变更记录.md`。
- 修改 SI7000 外部可见行为时，同步更新 `SI7000协议适配/02_协议映射/`、`03_改动记录/` 和 `04_联调测试/`。
- 外部供应商原始资料优先放入对应适配目录的 `00_原始资料/`，不要直接覆盖原始文件。
- 协议名、标准名、供应商资料名可保留英文；项目解释和执行计划优先使用中文文件名。

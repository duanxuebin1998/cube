# GD32 功能安全与自检库资料

更新日期：2026-06-30

## 目录用途

本目录用于记录把 CPU2 从 `STM32F429ZGT6` 迁移到 GD32 候选芯片时需要关注的功能安全、自检库、认证证据和官方获取路径。

当前结论只基于公开官方页面和 GD32 下载中心可检索条目；正式选型和认证前仍需向 GigaDevice 或代理商确认芯片型号、库版本、证书附件、授权范围和认证机构接受口径。

## 结论摘要

| 问题 | 当前结论 |
| --- | --- |
| GD32F450/GD32F470 是否有自检库资料 | 有。GD32 下载中心列出 `AN156 Porting Guide of IEC60730 ClassB Library based on GD32F4xx`，适用于 GD32F4xx 系列 IEC60730 ClassB 认证库移植。 |
| GD32F4xx 是否有 UL/IEC 60730 口径 | 有。GigaDevice 功能安全页面说明 GD32F4xx 系列 MCU 已通过 UL 60730 Class B 认证，并提供认证软件测试库和安全文档支持家电控制器等设备满足安全规范。 |
| GD32F450/GD32F470 是否有 IEC 61508 SIL2/SIL3 自检库证书 | 公开页面未看到 GD32F4xx 的 IEC 61508 SC3/SIL2/SIL3 证书。GigaDevice 页面列出 IEC 61508 SC3(SIL2/SIL3) 软件测试库证书的系列包括 GD32F3、GD32H7、GD32F5/GD32G5 等，不包含本次候选的 GD32F450/GD32F470。 |
| 能否替代 ST X-CUBE-STL 证据链 | 不能直接替代。GD32F4xx 的公开安全库口径偏 IEC/UL 60730 Class B；ST X-CUBE-STL 是面向 IEC 61508 的 STL 证据链。CPU2 如果要继续走 SIL2，应把 MCU 迁移视为安全证据体系重建。 |

## 官方入口

| 资料 | 官方链接 | 本地状态 | 用途 |
| --- | --- | --- | --- |
| GigaDevice Functional Safety | [Functional Safety](https://www.gigadevice.com/solution/key-technologies/functional-safety) | 链接保留，未归档页面 | 查看 GD 功能安全路线、UL/IEC60730、IEC61508 和证书系列覆盖范围 |
| GD32F450ZGT6 产品页 | [GD32F450ZGT6](https://www.gigadevice.com/product/mcu/mcus-product-selector/gd32f450zgt6) | 链接保留 | CPU2 候选替换芯片，资源接近 STM32F429ZGT6 |
| GD32F470ZGT6 产品页 | [GD32F470ZGT6](https://www.gigadevice.com/product/mcu/mcus-product-selector/gd32f470zgt6) | 链接保留 | CPU2 候选替换芯片，性能和 SRAM 余量更大 |
| GD32F4xx 下载中心 | [GD32F4 下载页](https://www.gd32mcu.com/en/download/0?kw=GD32F4) | 链接保留 | 查找 datasheet、user manual、errata、firmware library、demo suites、IEC60730 相关应用笔记 |
| IEC60730 检索入口 | [GD32 IEC60730 检索](https://www.gd32mcu.com/en/download/0?kw=IEC60730) | 链接保留 | 查找 AN038、AN039、AN156 等 IEC60730 自检库和 Flash 自检相关文档 |
| AN156 英文 PDF | [AN156 English PDF](https://www.gd32mcu.com/download/down/document_id/492/path_type/1) | 未归档 | GD32F4xx IEC60730 ClassB 库移植指南 |
| AN156 中文 PDF | [AN156 Chinese PDF](https://www.gd32mcu.com/download/down/document_id/492/path_type/2) | 未归档 | GD32F4xx IEC60730 ClassB 库移植指南中文版本 |

## 已确认的公开资料点

| 资料点 | 说明 | 对 CPU2 的影响 |
| --- | --- | --- |
| `AN156` | GD32 下载中心条目名为 `AN156 Porting Guide of IEC60730 ClassB Library based on GD32F4xx`，版本 1.0，发布日期 2024-07-11。页面说明该文档介绍 GD32F4xx IEC60730 ClassB 认证库在不同 IDE 下的移植注意事项，帮助客户实现 IEC60730 标准认证。 | 可作为 GD32F450/GD32F470 自检库调研起点，但需要拿到实际库包、API、例程、库版本和授权说明。 |
| GD32F4xx UL 60730 | GigaDevice 功能安全页面说明 GD32F4xx 系列 MCU 已通过 UL 60730 Class B 认证，并有认证软件测试库和安全文档。 | 适合支撑家电/控制器类 Class B 自检要求；不能直接说明满足 IEC 61508 SIL2。 |
| IEC 61508 SC3(SIL2/SIL3) | GigaDevice 功能安全页面列出 GD32F3、GD32H7、GD32F5/GD32G5 等系列的软件测试库 IEC 61508 SC3(SIL2/SIL3) 证书。 | 若 CPU2 必须以厂商 STL 证书作为 SIL2 输入，应优先向 GD 确认 F450/F470 是否有未公开的 IEC 61508 证书或改选 GD32F5/G5/H7 等证书覆盖系列。 |

## 与 ST X-CUBE-STL 的差异

| 项目 | ST X-CUBE-STL | GD32F4xx 当前公开资料 |
| --- | --- | --- |
| 主要标准口径 | IEC 61508 STL，证书和附件可作为 SIL2/SIL3 相关设计输入 | UL/IEC 60730 Class B 自检库，偏家电控制器安全规范 |
| CPU2 当前匹配性 | 当前 CPU2 是 `STM32F429ZGT6`，已有 STM32F4 Safety Manual、FMEA/FMEDA、X-CUBE-STL 包内资料归档 | GD32F450/GD32F470 与现有资源接近，但安全资料链路需重新确认 |
| 可直接复用程度 | 可继续用于 STM32 路线的安全输入 | 不能直接复用 ST 的 Safety Manual、FMEDA、X-CUBE-STL 证书或诊断覆盖假设 |
| 认证风险 | 已有资料体系较完整，但仍需项目级需求、架构、FMEDA、测试证据 | 需要重新建立芯片安全手册、失效率、诊断覆盖、库集成和测试证据 |

## CPU2 迁移前必须确认

1. 向 GigaDevice 或代理商索取 GD32F450/GD32F470 的 IEC60730 ClassB 库包、安全文档、证书附件和版本清单。
2. 明确 GD32F4xx 是否存在 IEC 61508 SC3(SIL2/SIL3) 软件测试库证书；如果没有，评估是否改选 GD32F5/G5/H7 等公开列入 IEC 61508 证书页面的系列。
3. 获取或确认 GD32F450/GD32F470 的 safety manual、FMEDA/FMEA、failure rate、diagnostic coverage、errata 和工具链限制。
4. 逐 pin 复核 LQFP144 引脚复用，尤其是 CPU2 现用的 `UART4/5/8`、`USART1/2/3/6`、`SPI2/3/4/5`、ADC、DAC、TIM、DMA、IWDG、CRC、SWD、HSE、复位和启动脚。
5. 重新评估 CPU2 安全需求、诊断策略、启动自检、运行时自检、故障上报、看门狗和安全状态机；不要把 IEC60730 ClassB 库当作 SIL2 充分证据。
6. 让认证机构提前确认 GD32F4xx ClassB 资料能在 CUBE SIL2 路线中承担什么证据角色，避免后期发现标准口径不匹配。

## 当前建议

如果目标只是降低成本或供应链替代，`GD32F450ZGT6` 仍是 STM32F429ZGT6 的首选 GD32F4xx 候选；`GD32F470ZGT6` 适合需要更多 SRAM 和性能余量的方案。

如果目标是继续推进 CPU2 SIL2，并希望 MCU 厂商提供类似 X-CUBE-STL 的 IEC 61508 自检库证书，则不要只锁定 GD32F450/GD32F470。应同步评估 GD32F5/G5/H7 等公开列入 GigaDevice IEC 61508 SC3(SIL2/SIL3) 软件测试库证书范围的系列，以及由此带来的硬件改板和软件迁移成本。

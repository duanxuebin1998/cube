# ST 官方 PDF 资料

更新日期：2026-06-06

## 目录用途

本目录归档用户提供的 ST 官方 SIL、STM32F4 Safety Manual、FMEA/FMEDA 和 X-CUBE-STL 应用笔记 PDF。

官方文件名保留英文和文档编号，便于和 ST 官网、认证资料清单、评审记录交叉核对。

## 文件索引

| 文件 | 重点用途 |
| --- | --- |
| `AN5698 Adapting the X-CUBE-STL functional safety package for STM32 IEC 61508 compliant to other safety standards.pdf` | X-CUBE-STL 与 IEC 61508 之外安全标准的适配参考 |
| `Customizing FMEDA for STM32 - AN5321 rev3.pdf` | STM32 FMEDA 定制、系统假设和项目化调整参考 |
| `FMEA for STM32F4 - AN5141.pdf` | STM32F4 FMEA 分析输入 |
| `FMEDA for STM32F4 - AN5140.pdf` | STM32F4 FMEDA、安全指标和诊断覆盖率分析输入 |
| `Safety self-test library Arm® Cortex®-M integration tips AN5659 rev1.pdf` | STL 在 Cortex-M 上的集成注意事项 |
| `Tool analysis for X-CUBE-STL functional safety support package AN5829 rev1.pdf` | X-CUBE-STL 工具分析和工具使用边界 |
| `um1840-stm32f4-series-safety-manual-stmicroelectronics.pdf` | STM32F4 系列 Safety Manual，关注使用假设、限制条件和硬件安全约束 |
| `X-CUBE-STL advanced topics AN5936 rev2.pdf` | X-CUBE-STL 高级主题和集成补充说明 |

## 使用提醒

- 正式认证时需确认这些资料版本是否仍为 ST 当前推荐版本。
- FMEA/FMEDA 中的假设和指标不能直接替代项目级安全分析，必须结合 CPU2 实际 MCU、硬件电路、诊断策略和认证机构意见复核。

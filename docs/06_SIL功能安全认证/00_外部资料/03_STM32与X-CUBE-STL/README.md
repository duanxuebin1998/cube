# STM32 与 X-CUBE-STL 资料

更新日期：2026-06-06

## 本地归档目录

| 目录 | 内容 |
| --- | --- |
| `ST_X-CUBE-STL资料使用说明.html` | ST 官方 SIL/X-CUBE-STL 资料是什么、做 SIL 认证时如何使用、当前 CUBE 工程如何落地 |
| `01_ST官方PDF/` | 用户提供的 ST 官方 SIL、STM32F4 Safety Manual、FMEA/FMEDA 和 X-CUBE-STL 应用笔记 |
| `02_X-CUBE-STL-F4_V2.0.0包内文档/` | X-CUBE-STL-F4 V2.0.0 包内用户手册、安全手册、发布说明和许可证 |
| `99_ST官方原始包/` | X-CUBE-STL-F4 V2.0.0 原始 zip 和包 Readme，本地保留；zip 受 `.gitignore` 的 `*.zip` 规则忽略 |

## 官方入口

| 资料 | 官方链接 | 本地状态 |
| --- | --- | --- |
| X-CUBE-STL 产品页 | [ST X-CUBE-STL](https://www.st.com/en/embedded-software/x-cube-stl.html) | 链接保留 |
| X-CUBE-STL data brief | [DB3595](https://www.st.com/resource/en/data_brief/x-cube-stl.pdf) | 本机直连 ST PDF 超时，未归档 |
| X-CUBE-STL 到其他安全标准映射 | [AN5698](https://www.st.com/resource/en/application_note/an5698-.pdf) | 已归档到 `01_ST官方PDF/` |
| STL 集成到时间关键应用 | [AN6179](https://www.st.com/resource/en/application_note/an6179-how-to-integrate-the-stl-firmware-into-a-time-critical-user-application-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32F4 Safety Manual | [UM1840](https://www.st.com/resource/en/user_manual/um1840-stm32f4-series-safety-manual-stmicroelectronics.pdf) | 已归档到 `01_ST官方PDF/`，X-CUBE-STL 包内 safety manual 已归档到 `02_X-CUBE-STL-F4_V2.0.0包内文档/` |
| STM32F4 参考手册 | [RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| Cortex-M4 编程手册 | [PM0214](https://www.st.com/resource/en/programming_manual/dm00046982-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32 HAL/LL 驱动说明 | [UM1725](https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32-hal-drivers-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32F427/437/429/439 勘误 | [ES0206](https://www.st.com/resource/en/errata_sheet/DM00068628-.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32F427/429 数据手册 | [STM32F427/429 datasheet](https://www.st.com/resource/en/datasheet/stm32f429be.pdf) | 本机直连 ST PDF 超时，未归档 |

## 已归档 ST 官方资料

| 文档 | 本地位置 | 用途 |
| --- | --- | --- |
| AN5698 | `01_ST官方PDF/AN5698 Adapting the X-CUBE-STL functional safety package for STM32 IEC 61508 compliant to other safety standards.pdf` | 参考 X-CUBE-STL 与其他安全标准的适配关系 |
| AN5321 | `01_ST官方PDF/Customizing FMEDA for STM32 - AN5321 rev3.pdf` | 参考 STM32 FMEDA 定制和系统级假设调整方法 |
| AN5141 | `01_ST官方PDF/FMEA for STM32F4 - AN5141.pdf` | 参考 STM32F4 FMEA 分析输入 |
| AN5140 | `01_ST官方PDF/FMEDA for STM32F4 - AN5140.pdf` | 参考 STM32F4 FMEDA 和安全指标分析输入 |
| AN5659 | `01_ST官方PDF/Safety self-test library Arm® Cortex®-M integration tips AN5659 rev1.pdf` | 参考 STL 在 Cortex-M 上的集成注意事项 |
| AN5829 | `01_ST官方PDF/Tool analysis for X-CUBE-STL functional safety support package AN5829 rev1.pdf` | 参考 X-CUBE-STL 工具分析和工具使用边界 |
| UM1840 | `01_ST官方PDF/um1840-stm32f4-series-safety-manual-stmicroelectronics.pdf` | 参考 STM32F4 Safety Manual 的使用假设和硬件安全约束 |
| AN5936 | `01_ST官方PDF/X-CUBE-STL advanced topics AN5936 rev2.pdf` | 参考 X-CUBE-STL 高级主题和集成补充说明 |
| X-CUBE-STL-F4 V2.0.0 包内资料 | `02_X-CUBE-STL-F4_V2.0.0包内文档/` | 参考 STL F4 用户手册、安全手册、勘误、发布说明和许可证 |

## 需向 ST 获取或申请的资料

| 资料 | 获取方式 | 用途 |
| --- | --- | --- |
| X-CUBE-STL Safety Manual 最新认证基线版本 | ST 产品页或 ST 支持渠道；本地已保留 F4 V2.0.0 包内文档 | 说明 STL 使用假设、集成约束、覆盖范围和限制 |
| FMEA/FMEDA 最新版和项目适用包 | ST 支持渠道，通常需要 NDA；本地已保留 STM32F4 参考资料 | 估算硬件失效率、诊断覆盖率、PFH/PFD 等安全指标 |
| Safety Analysis Report | ST 支持渠道，可能需要 NDA | 作为第三方安全库/芯片安全证据输入 |
| 认证证书及附件 | ST 产品页或认证机构页面 | 证明 X-CUBE-STL 的认证范围和限制条件 |

## 软件工程师重点关注

- 确认 CPU2 实际 MCU 型号和封装，与 RM、datasheet、errata、Safety Manual 对应。
- 如果采用 X-CUBE-STL，必须把 STL 自检项、执行周期、运行时间、失败处理、错误上报和启动/运行阶段集成进软件需求和设计。
- ST Safety Manual 和 FMEDA 给出的假设不能自动成立，必须转成项目约束或验证项。
- 勘误表中的外设、时钟、Flash、复位、中断相关限制要进入设计评审和测试计划。
- 本次只归档文档和原始 zip，未把 334MB 的 `STM32CubeExpansion_STL_F4_V2.0.0` 解压源码树纳入仓库；如果后续要集成 STL，需要按工程构建范围单独评审和引入。

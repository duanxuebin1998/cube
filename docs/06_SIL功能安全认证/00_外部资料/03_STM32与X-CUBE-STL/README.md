# STM32 与 X-CUBE-STL 资料

更新日期：2026-06-03

## 官方入口

| 资料 | 官方链接 | 本地状态 |
| --- | --- | --- |
| X-CUBE-STL 产品页 | [ST X-CUBE-STL](https://www.st.com/en/embedded-software/x-cube-stl.html) | 链接保留 |
| X-CUBE-STL data brief | [DB3595](https://www.st.com/resource/en/data_brief/x-cube-stl.pdf) | 本机直连 ST PDF 超时，未归档 |
| X-CUBE-STL 到其他安全标准映射 | [AN5698](https://www.st.com/resource/en/application_note/an5698-.pdf) | 本机直连 ST PDF 超时，未归档 |
| STL 集成到时间关键应用 | [AN6179](https://www.st.com/resource/en/application_note/an6179-how-to-integrate-the-stl-firmware-into-a-time-critical-user-application-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32F4 Safety Manual | [UM1840](https://www.st.com/resource/en/user_manual/um1840-stm32f4-series-safety-manual-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32F4 参考手册 | [RM0090](https://www.st.com/resource/en/reference_manual/rm0090-stm32f4xx-reference-manual-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| Cortex-M4 编程手册 | [PM0214](https://www.st.com/resource/en/programming_manual/dm00046982-stm32-cortexm4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32 HAL/LL 驱动说明 | [UM1725](https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32-hal-drivers-stmicroelectronics.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32F427/437/429/439 勘误 | [ES0206](https://www.st.com/resource/en/errata_sheet/DM00068628-.pdf) | 本机直连 ST PDF 超时，未归档 |
| STM32F427/429 数据手册 | [STM32F427/429 datasheet](https://www.st.com/resource/en/datasheet/stm32f429be.pdf) | 本机直连 ST PDF 超时，未归档 |

## 需向 ST 获取或申请的资料

| 资料 | 获取方式 | 用途 |
| --- | --- | --- |
| X-CUBE-STL Safety Manual | ST 产品页或 ST 支持渠道 | 说明 STL 使用假设、集成约束、覆盖范围和限制 |
| FMEA/FMEDA | ST 支持渠道，通常需要 NDA | 估算硬件失效率、诊断覆盖率、PFH/PFD 等安全指标 |
| Safety Analysis Report | ST 支持渠道，可能需要 NDA | 作为第三方安全库/芯片安全证据输入 |
| 认证证书及附件 | ST 产品页或认证机构页面 | 证明 X-CUBE-STL 的认证范围和限制条件 |

## 软件工程师重点关注

- 确认 CPU2 实际 MCU 型号和封装，与 RM、datasheet、errata、Safety Manual 对应。
- 如果采用 X-CUBE-STL，必须把 STL 自检项、执行周期、运行时间、失败处理、错误上报和启动/运行阶段集成进软件需求和设计。
- ST Safety Manual 和 FMEDA 给出的假设不能自动成立，必须转成项目约束或验证项。
- 勘误表中的外设、时钟、Flash、复位、中断相关限制要进入设计评审和测试计划。

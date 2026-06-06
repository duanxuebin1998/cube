# 外部资料索引

更新日期：2026-06-06

## 目录用途

本目录用于归档 SIL/功能安全准备中可公开下载的官方资料，并记录付费、版权受限或 NDA 资料的官方获取路径。资料只作为工程准备和学习参考，正式认证仍以公司采购的标准、认证机构要求和项目质量体系为准。

## 子目录

| 目录 | 内容 |
| --- | --- |
| `01_标准与流程/` | IEC 61508 概览、HSE 功能安全流程参考和正式标准获取路径 |
| `02_编码规范与静态分析/` | MISRA/CERT C、静态分析、偏离管理和编码检查资料 |
| `03_STM32与X-CUBE-STL/` | STM32、X-CUBE-STL、ST Safety Manual、芯片手册和勘误链接 |
| `04_认证机构与流程/` | TÜV、exida、DNV 等认证机构流程链接和沟通准备项 |
| `05_需购买或NDA资料/` | IEC/MISRA 主标准、ST FMEDA/FMEA、工具资质包等获取清单 |

## 已下载公开资料

| 文件 | 来源 | 用途 |
| --- | --- | --- |
| `01_标准与流程/IEC_61508_Functional_Safety_Overview_2022_IEC.pdf` | [IEC Functional Safety overview](https://assets.iec.ch/public/acos/IEC%2061508%20%26%20Functional%20Safety-2022.pdf) | 快速理解 IEC 61508 框架、生命周期和术语 |
| `01_标准与流程/HSG238_Out_of_Control_HSE.pdf` | [HSE HSG238](https://www.hse.gov.uk/pubns/priced/hsg238.pdf) | 安全相关控制系统实施、管理和验证流程参考 |
| `02_编码规范与静态分析/MISRA_Compliance_2020.pdf` | [MISRA Compliance:2020](https://misra.org.uk/app/uploads/2021/06/MISRA-Compliance-2020.pdf) | deviation、compliance statement、规则偏离闭环依据 |
| `02_编码规范与静态分析/MISRA_C_2012_Amendment_4.pdf` | [MISRA C:2012 Amendment 4](https://misra.org.uk/app/uploads/2023/03/MISRA-C-2012-AMD4.pdf) | 新增规则和修订内容参考，不能替代 MISRA C 主规则 |
| `03_STM32与X-CUBE-STL/01_ST官方PDF/` | 用户提供的 ST 官方 SIL 资料 | STM32F4 Safety Manual、AN5140/AN5141、AN5321、AN5659、AN5698、AN5829、AN5936 等 |
| `03_STM32与X-CUBE-STL/02_X-CUBE-STL-F4_V2.0.0包内文档/` | X-CUBE-STL-F4 V2.0.0 安装包内资料 | STL F4 用户手册、安全手册、包内发布说明和许可证 |

## 官方链接资料

| 资料 | 官方链接 | 本地状态 |
| --- | --- | --- |
| IEC 61508 官方入口 | [IEC Functional Safety](https://www.iec.ch/functionalsafety) | 链接保留 |
| IEC 61508 标准检索 | [IEC Webstore search](https://webstore.iec.ch/searchform&q=IEC%2061508) | 需采购 |
| MISRA C 产品页 | [MISRA C](https://misra.org.uk/product/misra-c2012/) | 主规则需采购 |
| SEI CERT C 在线规则库 | [SEI CERT C Coding Standard](https://wiki.sei.cmu.edu/confluence/display/c/SEI+CERT+C+Coding+Standard) | 链接保留 |
| X-CUBE-STL 产品页 | [ST X-CUBE-STL](https://www.st.com/en/embedded-software/x-cube-stl.html) | 链接保留；已归档用户提供的 F4 V2.0.0 资料 |
| STM32F4 系列资料 | [ST STM32F429/439 product page](https://www.st.com/en/microcontrollers-microprocessors/stm32f429-439.html) | 链接保留，CPU2 当前为 STM32F429ZGTx |

## 资料使用建议

| 工作项 | 优先查看 |
| --- | --- |
| 了解 SIL/功能安全生命周期 | `01_标准与流程/README.md`、IEC 概览、HSG238 |
| 制定编码检查和偏离流程 | `02_编码规范与静态分析/README.md`、MISRA Compliance、CPU2 编码规范 |
| 评估 STM32 诊断和安全库 | `03_STM32与X-CUBE-STL/README.md`、X-CUBE-STL、ST Safety Manual |
| 准备认证机构沟通 | `04_认证机构与流程/README.md`、项目交付物清单 |
| 发起采购或 NDA | `05_需购买或NDA资料/README.md` |

## 维护规则

- 新增外部 PDF 时优先使用官方直链或官方网站下载。
- 不收录非官方分享的 IEC、MISRA、认证报告或供应商 NDA 文件。
- ST、工具厂商和认证机构资料如果需要登录、表单或 NDA，记录官方页面，不绕过访问控制。
- 每个子目录 README 要说明资料用途、获取方式和本地下载状态。

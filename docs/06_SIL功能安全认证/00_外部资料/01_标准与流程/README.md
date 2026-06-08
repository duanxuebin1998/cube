# 标准与流程资料

更新日期：2026-06-03

## 已下载资料

| 文件 | 来源 | 使用场景 |
| --- | --- | --- |
| `IEC_61508_Functional_Safety_Overview_2022_IEC.pdf` | [IEC](https://assets.iec.ch/public/acos/IEC%2061508%20%26%20Functional%20Safety-2022.pdf) | 对齐功能安全术语、生命周期、系统和软件活动边界 |
| `HSG238_Out_of_Control_HSE.pdf` | [UK HSE](https://www.hse.gov.uk/pubns/priced/hsg238.pdf) | 参考安全相关控制系统的管理、实施、验证和维护方法 |

## 需采购或正式受控资料

| 资料 | 获取方式 | 软件工程师用途 |
| --- | --- | --- |
| IEC 61508-1 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508) | 总体要求、管理、生命周期和文档体系 |
| IEC 61508-2 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-2) | E/E/PE 系统硬件要求，供软硬件边界确认 |
| IEC 61508-3 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-3) | 软件安全生命周期、需求、设计、编码、验证和变更控制核心依据 |
| IEC 61784-3 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%20617-84-3) | 面向工业网络安全相关通信（functional safety communication）标准，适用于安全相关传感器/现场总线场景 |
| IEC 61508-4 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-4) | 术语定义，保证需求、设计、报告用词一致 |
| IEC 61508-5 到 61508-7 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508) | SIL 确定、技术措施和应用指南 |

## 软件工程师重点关注

- 先用 IEC 概览和 HSG238 建立流程框架，再由公司采购 IEC 61508 正式标准作为认证依据。
- 软件工作重点是 IEC 61508-3：软件安全需求、架构设计、详细设计、编码标准、验证、确认、配置管理和变更控制。
- 任何提交给认证机构的合规判断，不能只引用概览文档，必须引用正式标准或认证机构确认的项目要求。

## IEC 61784-3 快速引入
- 已新增《IEC_61784-3_传感器通信与功能安全手册.md》，用于判断传感器通信是否属于安全相关通道。
- 当出现安全相关通信争议时，建议先按本手册给出结论，再结合供应商安全认证能力和现场联调记录闭环。

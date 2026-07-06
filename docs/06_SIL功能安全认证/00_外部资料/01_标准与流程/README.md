# 标准与流程资料

更新日期：2026-07-03

## 已下载和已整理资料

| 文件 | 来源 | 使用场景 |
| --- | --- | --- |
| `GB_T_20438_2017/` | 用户提供的 GB/T 20438-2017 七分册 PDF | 功能安全管理、E/E/PE 系统要求、软件安全生命周期、SIL 确定方法、应用指南和技术措施参考 |
| `IEC_61508_Functional_Safety_Overview_2022_IEC.pdf` | [IEC](https://assets.iec.ch/public/acos/IEC%2061508%20%26%20Functional%20Safety-2022.pdf) | 对齐功能安全术语、生命周期、系统和软件活动边界 |
| `HSG238_Out_of_Control_HSE.pdf` | [UK HSE](https://www.hse.gov.uk/pubns/priced/hsg238.pdf) | 参考安全相关控制系统的管理、实施、验证和维护方法 |
| `标准原文归档清单.md` | 工程整理文档，登记已入库原文和官方核查后待补入标准 | 管理标准原文、公开资料、授权状态、建议归档路径和命名规则 |
| `官方预览与公开说明/` | 官方页面核查记录，记录 IEC Webstore、国家标准全文公开系统等公开入口的可下载状态 | 查明官方是否提供完整原文下载，避免把预览页或网页登录页误当标准原文 |
| `待获取标准原文/IEC_61784-3/IEC_61784-3_2021.pdf` | 用户提供的 IEC 61784-3:2021 PDF | 传感器安全通信、functional safety fieldbus 通用规则和 profile 定义参考；CSV 修订整合版仍待授权或用户提供 |
| `待获取标准原文/README.md` | 工程整理目录，记录需通过官方公开下载、授权或用户提供后补入的原文结构 | 放置获取说明和授权登记模板，并归档已取得授权或用户提供的标准原文 |
| `传感器安全通信相关标准适用矩阵.md` | 工程整理文档，链接指向 IEC、ISO、国家标准公开系统和 ISA 官方入口 | 传感器安全通信协议重设计时确认标准适用边界、采购清单、证据包落点和自研安全通信层最低证据 |
| `IEC_61784-3_传感器通信与功能安全手册.md` | 工程整理文档，围绕 IEC 61784-3 的项目适用性说明 | 判断传感器通信是否属于安全相关通道，以及普通通信和安全通信的边界 |

## 待授权或正式受控资料

| 资料 | 获取方式 | 软件工程师用途 |
| --- | --- | --- |
| IEC 61508-1 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508) | 总体要求、管理、生命周期和文档体系 |
| IEC 61508-2 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-2) | E/E/PE 系统硬件要求，供软硬件边界确认 |
| IEC 61508-3 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-3) | 软件安全生命周期、需求、设计、编码、验证和变更控制核心依据 |
| IEC 61784-3:2021 + AMD1:2024 CSV | [IEC Webstore](https://webstore.iec.ch/en/publication/92494) | 面向工业网络安全相关通信（functional safety communication）标准，适用于安全相关传感器/现场总线场景 |
| IEC 62280:2014 | [IEC Webstore](https://webstore.iec.ch/en/publication/6749) | 安全相关设备之间通过不完全可信传输系统通信的要求；自研安全通信层可作为威胁模型和措施参考，正式采用前需认证机构确认 |
| IEC 61508-4 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-4) | 术语定义，保证需求、设计、报告用词一致 |
| IEC 61508-5 到 61508-7 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508) | SIL 确定、技术措施和应用指南 |
| IEC 61511-1 / GB/T 21109.1 | [IEC Webstore](https://webstore.iec.ch/en/publication/24241)、[国家标准全文公开系统](https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=E6FC8CDB266A4EAB6A16F56D48A20E72) | 过程工业安全仪表系统 SIS/SIF 场景的生命周期和应用编程要求 |
| IEC 62061 / ISO 13849-1 | [IEC Webstore](https://webstore.iec.ch/en/publication/112847)、[ISO](https://www.iso.org/standard/73481.html) | 机械安全相关控制系统，按 SIL 或 PL 目标评估安全功能 |
| IEC 62443 系列 | [ISA/IEC 62443](https://www.isa.org/standards-and-publications/isa-standards/isa-iec-62443-series-of-standards) | 传感器链路联网、无线、远程配置或可写安全参数时的工业网络安全要求 |
| IEC 61000-1-2、IEC 61326-3-1、IEC 61000-6-7 | [IEC Webstore](https://webstore.iec.ch/) | 安全相关链路的 EMC 功能安全方法、抗扰度环境和误动作判据 |

## 软件工程师重点关注

- 先用 IEC 概览和 HSG238 建立流程框架；正式认证依据必须来自官方公开完整原文、公司授权文件或用户提供的受控文件。
- 用户提供的 GB/T 20438-2017 可作为 IEC 61508:2010 对应国家标准体系参考；正式认证引用前仍需确认公司受控版本、授权状态和认证机构接受范围。
- 标准原文的入库状态、授权状态和待获取路径统一登记在 `标准原文归档清单.md`；不要把 IEC、ISO、MISRA 或 NDA 资料从非官方来源放入仓库，也不要把网页登录页、预览页或 404 页面当作 PDF 入库。
- 软件工作重点是 IEC 61508-3：软件安全需求、架构设计、详细设计、编码标准、验证、确认、配置管理和变更控制。
- 任何提交给认证机构的合规判断，不能只引用概览文档，必须引用正式标准或认证机构确认的项目要求。
- 本目录 PDF 只作为外部输入资料；CUBE 自身安全生命周期、偏离记录、整改清单和证据包结论写入上层 Markdown。

## IEC 61784-3 快速引入

- 已新增《IEC_61784-3_传感器通信与功能安全手册.md》，用于判断传感器通信是否属于安全相关通道。
- 已新增《传感器安全通信相关标准适用矩阵.md》，用于把 IEC 61508、IEC 61784-3、IEC 62280、IEC 61511、IEC 62061、ISO 13849-1、IEC 62443 和 EMC 标准放到同一张选型矩阵中。
- 当出现安全相关通信争议时，建议先按标准适用矩阵确认认证路线，再结合供应商安全认证能力、SRS、协议设计和现场联调记录闭环。

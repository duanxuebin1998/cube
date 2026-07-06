# 官方获取或受控资料清单

更新日期：2026-07-03

## 处理原则

本目录只记录官方获取路径和用途，不保存非官方副本。对于官方公开可下载资料，可按官方链接入库并记录来源；对于 IEC、ISO、MISRA 正式标准、认证报告、工具资质包和供应商 NDA 资料，如官方入口未公开完整原文，应由公司通过正式渠道授权、采购、申请或签署 NDA 后纳入受控文档库。

## 优先级清单

| 优先级 | 资料 | 获取方式 | 用途 |
| --- | --- | --- | --- |
| P0 | IEC 61508-3 正式标准 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-3) | 软件安全生命周期核心依据 |
| P0 | IEC 61508-1 正式标准 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-1) | 功能安全管理、生命周期和总体要求 |
| P0 | IEC 61508-2 正式标准 | [IEC Webstore](https://webstore.iec.ch/searchform&q=IEC%2061508-2) | E/E/PE 系统、硬件安全指标和安全相关系统要求 |
| P0 | IEC 61784-3:2021+AMD1:2024 CSV | [IEC Webstore](https://webstore.iec.ch/en/publication/92494) | 安全相关传感器通信、fieldbus safety profile 和 black channel 评估依据 |
| P0 | MISRA C 主规则 | [MISRA C publications](https://misra.org.uk/publications/) | 编码规则原文和规则解释 |
| P0 | 静态分析工具资质包 | 工具供应商 | 工具确认、规则覆盖和认证审计证据 |
| P1 | IEC 62280:2014 | [IEC Webstore](https://webstore.iec.ch/en/publication/6749) | 自研安全通信层威胁模型、错误防护措施和测试项参考；正式采用前需认证机构确认 |
| P1 | IEC 61511-1 或 GB/T 21109.1 | [IEC Webstore](https://webstore.iec.ch/en/publication/24241)、[国家标准全文公开系统](https://openstd.samr.gov.cn/bzgk/std/newGbInfo?hcno=E6FC8CDB266A4EAB6A16F56D48A20E72) | 过程工业安全仪表系统 SIS/SIF 场景适用 |
| P1 | IEC 62061 或 ISO 13849-1 | [IEC Webstore](https://webstore.iec.ch/en/publication/112847)、[ISO](https://www.iso.org/standard/73481.html) | 机械安全控制系统按 SIL 或 PL 目标评估时适用 |
| P1 | 单元测试/覆盖率工具资质包 | 工具供应商 | 测试工具适用性和覆盖率证据 |
| P1 | 编译器安全手册和已知问题列表 | 编译器供应商 | 编译器适用性、版本锁定和补充验证 |
| P1 | X-CUBE-STL Safety Manual 最新认证基线版本 | [ST X-CUBE-STL](https://www.st.com/en/embedded-software/x-cube-stl.html) 或 ST 支持渠道；本地已归档 F4 V2.0.0 包内文档作为参考 | STL 集成约束、使用假设、诊断范围 |
| P1 | STM32 FMEA/FMEDA 最新版和项目适用包 | ST 支持渠道，通常需要 NDA；本地已归档 STM32F4 参考资料作为输入 | 失效率、诊断覆盖率、安全指标计算 |
| P2 | IEC 62443 系列 | [ISA/IEC 62443](https://www.isa.org/standards-and-publications/isa-standards/isa-iec-62443-series-of-standards) 或 IEC Webstore | 传感器链路联网、无线、远程配置或上位机可写安全参数时的工业网络安全要求 |
| P2 | IEC 61000-1-2、IEC 61326-3-1、IEC 61000-6-7 | [IEC Webstore](https://webstore.iec.ch/) | 安全相关链路的 EMC 功能安全方法、抗扰度环境和误动作判据 |
| P2 | 认证机构评估计划模板 | 认证机构 | 定义项目评估范围、样例证据和审计节点 |

2026-07-03 核查：IEC Webstore 相关标准页面可见 Preview、价格和 Add to cart 信息，未发现官方公开完整 PDF；国家标准全文公开系统对 GB/T 21109.1-2022 明示因采用 ISO/IEC 等国外组织标准、涉及版权保护，暂不提供在线阅读服务。后续如官方入口开放完整下载或用户提供授权文件，再按 `../01_标准与流程/标准原文归档清单.md` 补入。

## 采购或申请时要确认的问题

| 问题 | 说明 |
| --- | --- |
| 资料版本 | 版本必须和项目认证基线一致，不能混用不同版本规则 |
| 授权范围 | 确认团队成员、项目、地点和归档方式是否被授权 |
| 可引用性 | 确认是否允许在认证证据包中引用页码、章节或附件 |
| 更新机制 | 确认标准修订、工具版本升级、厂商勘误更新后的通知和评估方式 |
| 保密要求 | NDA 资料不得进入公开仓库或未授权共享目录 |

## 项目内部动作

- 采购或申请完成后，在受控文档库登记资料编号、版本、责任人和适用项目。
- 在本目录 README 中只补充“已获取/受控编号/责任人”，不要贴出 NDA 内容。
- 如果资料要求改变编码规范、测试策略或设计约束，要同步更新上级目录的项目文档和整改检查表。

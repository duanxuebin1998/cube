# 认证机构与流程资料

更新日期：2026-06-03

## 官方入口

| 机构 | 官方链接 | 软件团队用途 |
| --- | --- | --- |
| TÜV SÜD | [IEC 61508 Functional Safety](https://www.tuvsud.com/en/services/functional-safety/iec-61508) | 了解评估范围、流程、差距分析和证书申请方式 |
| TÜV Rheinland | [Functional Safety](https://www.tuv.com/world/en/functional-safety.html) | 了解功能安全评估、产品认证和培训入口 |
| exida | [Certification](https://www.exida.com/Certification) | 了解功能安全产品认证、FMEDA 和工具支持 |
| DNV | [Functional Safety Certification](https://www.dnv.com/services/functional-safety-certification-iec-61508-20208/) | 了解 IEC 61508 认证和评估服务 |

## 第一次沟通前准备

| 准备项 | 责任人 | 说明 |
| --- | --- | --- |
| 产品和系统边界 | 系统负责人 | 说明本次申请认证的产品、功能和运行环境 |
| 目标 SIL | 安全负责人 | 明确 SIL3 是否来自风险评估、客户要求或合同要求 |
| 软件范围 | 软件负责人 | 列出 CPU2/CPU3 中安全相关、支撑、非安全模块 |
| 开发流程 | 项目负责人 | 说明需求、设计、编码、评审、测试、发布、变更控制流程 |
| 证据包目录 | 软件负责人 | 整理需求、设计、代码、测试、静态分析、偏离记录、工具确认 |
| 第三方软件清单 | 软件负责人 | HAL、CMSIS、X-CUBE-STL、编译器、静态分析器、测试框架 |

## 软件团队常见问题

- 认证机构会要求说明安全相关软件范围，不能只说“CPU2 整体认证”。
- SIL3 不只看代码质量，还需要需求、设计、测试、工具、配置管理、变更管理和组织流程证据。
- 静态分析报告必须可复现，规则配置和工具版本要受控。
- 所有偏离项需要风险说明和审批，不应只在代码里写抑制注释。

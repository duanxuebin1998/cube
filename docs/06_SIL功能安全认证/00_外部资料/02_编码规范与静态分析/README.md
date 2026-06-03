# 编码规范与静态分析资料

更新日期：2026-06-03

## 已下载资料

| 文件 | 来源 | 使用场景 |
| --- | --- | --- |
| `MISRA_Compliance_2020.pdf` | [MISRA](https://misra.org.uk/app/uploads/2021/06/MISRA-Compliance-2020.pdf) | 建立 compliance statement、deviation record、规则偏离审批流程 |
| `MISRA_C_2012_Amendment_4.pdf` | [MISRA](https://misra.org.uk/app/uploads/2023/03/MISRA-C-2012-AMD4.pdf) | 查看 MISRA C:2012 的公开修订内容 |

## 官方链接

| 资料 | 官方链接 | 状态 |
| --- | --- | --- |
| MISRA C:2012 主规则 | [MISRA C:2012 product page](https://misra.org.uk/product/misra-c2012/) | 需采购，不能用公开 Amendment 替代 |
| MISRA C 后续版本 | [MISRA publications](https://misra.org.uk/publications/) | 按公司选型采购 |
| SEI CERT C Coding Standard | [SEI CERT C wiki](https://wiki.sei.cmu.edu/confluence/display/c/SEI+CERT+C+Coding+Standard) | 在线公开资料，适合补充安全编码规则 |

## 编码检查落地资料

| 资料 | 建议负责人 | 说明 |
| --- | --- | --- |
| CPU2 编码规范 | 软件负责人 | 已在上级目录维护，定义本项目 MISRA/CERT C 子集和禁止项 |
| 静态分析规则映射表 | 软件负责人 + 工具负责人 | 把项目规则映射到工具规则编号，说明不可检测规则的人工评审方式 |
| Deviation record | 模块负责人 | 每个偏离项记录规则、代码位置、理由、风险、补偿措施、审批人 |
| 静态分析报告 | 工具负责人 | 每次受控构建保留工具版本、规则配置、报告和关闭依据 |
| 编译器告警报告 | 构建负责人 | 安全相关模块至少保留编译器版本、告警等级、零告警或已批准告警证据 |

## 建议规则组合

- 强制：未初始化、越界访问、空指针、无界指针运算、整数溢出风险、隐式窄化转换、宏副作用、死代码、不可达代码、未检查返回值。
- 强制：安全相关模块禁止未受控全局状态修改，禁止中断和主循环共享变量缺少 `volatile` 或同步说明。
- 强制：所有 `deviation` 必须先评估再合入，不能把静态分析抑制注释当作审批记录。
- 建议：把 MISRA/CERT 工具输出、编译器告警和人工审查问题统一进入整改检查表。

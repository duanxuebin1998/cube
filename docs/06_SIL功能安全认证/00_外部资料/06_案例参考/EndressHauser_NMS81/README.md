# Endress+Hauser Proservo NMS81 案例参考

更新日期：2026-06-06

## 目录用途

本目录归档用户提供的 Endress+Hauser Proservo NMS81 SIL/功能安全资料，用于 CUBE 后续 SIL 准备时参考外部产品的安全功能定义、安全相关输出、SIL 配置锁定、proof test 和生命周期控制。

## 文件索引

| 文件 | 内容 | 用途 |
| --- | --- | --- |
| `EH_Proservo_NMS81_Functional_Safety_Manual_FY01100G_2023.pdf` | Functional Safety Manual - Proservo NMS81，文件号 FY01100G/00/EN/01.23-00，日期 2023-06-12 | 主来源，参考安全手册结构和使用限制 |
| `SIL认证文档整理_NMS8x_NMS81.html` | NMS8x 证书和 NMS81 安全手册的中文整理 | 快速导读，辅助理解术语、参数、proof test 和项目检查清单 |
| `../../../NMS81案例对CUBE_SIL认证和软件架构的启发.md` | CUBE 视角的整理和架构启发 | 本工程后续 SIL 资料补强入口 |

## 使用边界

- NMS81 资料说明的是 E+H 产品在指定型号、版本、配置和生命周期条件下的安全使用方式。
- CUBE 不能直接继承 NMS81 的 SIL 结论，必须建立自己的 SRS、架构设计、PFDavg 计算、测试、静态分析、偏离记录和变更控制证据。
- 正式引用时优先使用供应商安全手册、证书和认证机构确认，不以本目录中文整理替代主来源。

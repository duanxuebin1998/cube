# X-CUBE-STL-F4 V2.0.0 包内文档

更新日期：2026-06-06

## 目录用途

本目录只归档 `STM32CubeExpansion_STL_F4_V2.0.0` 安装包内的文档、发布说明和许可证，不包含包内 `Drivers`、`Middlewares`、`Projects` 示例源码树。

## 文件索引

| 文件 | 重点用途 |
| --- | --- |
| `Package_license.html` | X-CUBE-STL-F4 V2.0.0 包许可条款 |
| `Release_Notes.html` | X-CUBE-STL-F4 V2.0.0 发布说明和版本变化 |
| `STL_ES_cx757577_2.0.pdf` | STL 包内勘误或补充说明，需结合发布说明核对 |
| `STL_F4_SM_cx616769_7.0.pdf` | X-CUBE-STL-F4 Safety Manual，关注使用假设、集成约束和诊断范围 |
| `STL_F4_UG_cx616705_4.0.pdf` | X-CUBE-STL-F4 User Guide，关注自检库配置、调用方式和示例工程结构 |

## 使用提醒

- 如果后续要把 STL 集成到 CPU2 固件，需要先确定认证构建范围、源码引入路径、库版本锁定、失败处理和测试证据。
- 包内示例源码不在本目录，避免第三方源码和当前固件源码边界混淆。
- `Release_Notes.html` 保留 ST 原始包内相对链接，部分链接指向未归档的 `Drivers`、`Middlewares`、`Projects` 或历史 PDF 文件；项目文档链接检查时按外部原始资料处理。

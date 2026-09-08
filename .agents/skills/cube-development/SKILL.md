---
name: cube-development
description: CUBE 项目的工程约束与任务路由；用于本仓库及明确关联的固件、协议、文档和发布工作。
---

# CUBE 项目开发

## 规则归属

- 通用的分析、编辑、调试、验证和 Git 安全流程由全局 `engineering-workflow` 提供。
- CUBE 的路径、编码、协议、版本、参数、文档和提交规则只在本仓库的 `AGENTS.md` 与本 skill 维护。
- 授权边界、文档目录、写入租约和提交硬规则以根 `AGENTS.md` 为准；本入口不重复操作清单。用户决定定义目标，源码和原始证据说明当前事实，不把目标设计写成已经实现。
- 只读问答从相关文件取证；局部任务不要求完整工程盘点。需要阶段判断、长任务恢复或跨组件分析时，按下方路由读取对应 reference。

## CUBE 硬约束

- 修改已有文件前确认编码和换行。CPU2/CPU3 旧源码可能是 GBK/936；文档、skill 和工具脚本使用 UTF-8。
- C/C++ 新增或修改注释统一使用 `/* ... */`，不得新增 `//`。
- CPU2/CPU3 板间共享契约与 CPU3 对外 LTD 标准 Modbus 的共享区使用同一组地址、字段和命令语义；修改任一侧时同步更新两端实现、LTD 对外暴露、协议版本、正式协议资料和契约测试。
- 修改 Markdown、Word、HTML 或其它源文档时默认不自动生成、刷新或配套新增 PDF；只有用户当前任务明确要求 PDF 交付物时才由 Codex 生成或更新并验证。
- 已由用户人工维护、或任务开始时已有改动的 PDF 是正常项目资料，不得仅因存在同名 Markdown 自动排除、拒绝暂存或拒绝提交；需要检查或交付时按用户范围使用 PDF skill。
- 固件、硬件、台架、现场和 SIL 证据必须分层描述；构建或静态检查不等于真实设备验证。
- 每次真实固件升版都必须同步更新售后版 Excel；改动逐条编号并使用售后易懂的措辞，具体要求见 `references/version-build-release.md`。

## 执行与完成

- 已授权实现持续完成到相关验证及修复结束；“继续”承接已授权的整体目标。用户明确要求分阶段审批时才在该审批点暂停，不自动扩展到未授权的升版、提交或推送。
- 写入前核对 Git 状态和编码；非简单写入用 `scripts/manage_task_lease.py` 声明最小路径。长任务、跨仓库和发布任务用 `scripts/snapshot_cube_context.ps1` 保存 JSON；恢复时用 `scripts/check_context_snapshot.py` 核对，再继续安全分离的范围。
- 文字和注释调整检查编码、diff 及相关链接；固件行为修改完成相关静态检查、测试和受影响固件构建。共享协议、存储及安全路径扩大到相关两端和异常路径；固件提交/发布保留 clean-first 门禁，详见 `references/validation-evidence.md`。
- 必要检查通过即停止验证；只有输入变化、失败或具体未决风险才扩大或重复。发布、正式验收和需要复用的证据用 `scripts/record_validation_evidence.py` 绑定相关输入指纹；输入变化仅使依赖它的证据失效。
- 修改本 Skill 后运行 `scripts/validate_cube_skill.py`；修改校验脚本时运行对应回归测试。结构通过不代表行为正确，还需审查任务授权、路由和完成条件。
- 完成后报告结果、验证及实际残余风险。需要根目录 Snapshot 的任务完成 Compare；非简单写入关闭自己的租约。

## Reference 路由

- 任务阶段、用户用语、范围、并行工作、长任务恢复和完成矩阵：`references/task-lifecycle-and-scope.md`
- 整体工程组件、目录职责、正式事实入口和关联仓库：`references/project-map-and-dependencies.md`
- 原因分析、参数生效、现场日志、时序和回归：`references/diagnostics-and-logs.md`
- 现场证据归档、现场版本与当前源码双基线、抓包边界和原始文件哈希：`references/field-evidence-and-version-baselines.md`
- 编码、中文注释、GBK/UTF-8 和显示字库：`references/encoding-and-text.md`
- CPU3 OLED、菜单、状态页和显示参数：`references/cpu3-display-ui.md`
- 错误日志、错误传递、中断边界和故障恢复：`references/logging-and-recovery.md`
- 故障码、历史编号、工作簿和售后口径：`references/fault-code-governance.md`
- CPU2/CPU3 共享协议、参数存储和运行态：`references/internal-protocol-and-storage.md`
- DSM、Wartsila、SI、LTD 和外部协议：`references/external-protocols.md`
- 传感器安全协议、SIL 证据和故障注入：`references/sensor-safety-protocol.md`
- 构建、版本、CHANGELOG、售后版 Excel 和版本改动与测试方案：`references/version-build-release.md`
- 构建、静态检查、渲染、台架和现场证据的新鲜度与结构化记录：`references/validation-evidence.md`
- 文档、PDF 边界、暂存、提交和门禁：`references/docs-git-validation.md`
- CUBE 正式 Word、参考文档边界、候选提升、目录更新和逐页验收：`references/formal-word-documents.md`
- `docs-site/` 生成边界、受控构建、固定预览和进程收尾：`references/docs-site-preview.md`

只加载当前动作需要的 reference；后续进入新动作时再读取相应资料。正式 Office/PDF 交付使用对应格式 Skill 并保留渲染验收；纯内容查询按需要抽取和检查，不触发制作流程。格式 Skill 的默认目录服从本仓库目录规则。

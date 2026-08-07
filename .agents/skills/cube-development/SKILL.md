---
name: cube-development
description: Repository-scoped workflow for D:\CUBE firmware, protocol, safety, fault-code, parameter-storage, CPU3 UI, documentation, versioning, build, Git commit, and release tasks. Use for every analysis, implementation, review, document, amend, or commit task whose repository root is D:\CUBE.
---

# CUBE 项目开发

## 规则归属

- 通用的分析、编辑、调试、验证和 Git 安全流程由全局 `engineering-workflow` 提供。
- CUBE 的路径、编码、协议、版本、参数、文档和提交规则只在本仓库的 `AGENTS.md` 与本 skill 维护。
- 每次进入提交、amend 或发布阶段，必须在当前 turn 重新读取 `AGENTS.md`、本文件、`references/docs-git-validation.md` 和 `references/version-build-release.md`；不得依赖旧对话摘要声称已经读取。
- 当前用户决定、当前源码和最终 index/worktree diff 是本轮事实；旧对话、记忆和历史提交只用于恢复背景。

## CUBE 硬约束

- 正式业务文档只维护 `docs/`；`docs-site/` 是本机预览层，旧 `site/` 不再维护。
- `docs/00_程序流程导航/` 与 `tools/` 在本机同步但不纳入 Git，不得 `git add -f`。
- 修改已有文件前确认编码和换行。CPU2/CPU3 旧源码可能是 GBK/936；文档、skill 和工具脚本使用 UTF-8。
- C/C++ 新增或修改注释统一使用 `/* ... */`，不得新增 `//`。
- 修改 Markdown、Word、HTML 或其它源文档时默认不生成、刷新或配套新增 PDF；只有用户当前任务明确要求 PDF 交付物时才生成并验证。
- 固件、硬件、台架、现场和 SIL 证据必须分层描述；构建或静态检查不等于真实设备验证。

## 开始和结束门禁

- 非简单任务先运行 `scripts/snapshot_cube_context.ps1`，并按 `AGENTS.md` 创建根目录 Snapshot。
- 记录分支、HEAD、CPU2/CPU3 版本、`DEVICE_PROTOCOL_VERSION`、参数存储版本，以及 staged、unstaged、untracked 三层状态。
- 发现并行改动时只处理能够安全分离的范围；不得 reset、checkout、覆盖或清理用户改动。
- 任务结束前运行根目录 Compare，并报告 Git 提交内容、本机同步内容、已验证项和未验证项。

## 提交硬门禁

- 用户说“全部提交”“提交所有改动”或同义表达时，提交模式必须设为 `all`。这已经授权纳入任务快照中全部 Git 可见且不违反仓库硬规则的改动，包括删除和未跟踪文件。
- `all` 模式不得自行排除任何允许提交的改动。遇到禁止提交、敏感或必须由用户决定的项目时，停止并请求决定；不得排除后继续提交。
- 只有用户点名文件、模块或明确要求保留其它改动时才使用 `scoped` 模式。
- 提交时必须显式设置 `CUBE_COMMIT_SCOPE=all` 或 `CUBE_COMMIT_SCOPE=scoped`，由本地 `pre-commit` 门禁检查；未设置时禁止提交。
- 禁止只执行 `git commit -m "<标题>"`。必须使用消息文件或多段 `-m`，正文严格包含“版本、协议版本/兼容性、本次修改、验证”四个区块。
- 提交前、提交后都读取完整提交正文；只通过标题版本门禁不代表提交信息合格。

## Reference 路由

- 原因分析、参数生效、现场日志、时序和回归：`references/diagnostics-and-logs.md`
- 编码、中文注释、GBK/UTF-8 和显示字库：`references/encoding-and-text.md`
- CPU3 OLED、菜单、状态页和显示参数：`references/cpu3-display-ui.md`
- 错误日志、错误传递、中断边界和故障恢复：`references/logging-and-recovery.md`
- 故障码、历史编号、工作簿和售后口径：`references/fault-code-governance.md`
- CPU2/CPU3 共享协议、参数存储和运行态：`references/internal-protocol-and-storage.md`
- DSM、Wartsila、SI、LTD 和外部协议：`references/external-protocols.md`
- 传感器安全协议、SIL 证据和故障注入：`references/sensor-safety-protocol.md`
- 构建、版本、CHANGELOG 和版本改动与测试方案：`references/version-build-release.md`
- 文档、PDF 边界、暂存、提交和门禁：`references/docs-git-validation.md`

组合任务读取所有命中的 reference。格式类文件同时使用对应的文档、表格、PDF 或浏览器技能；本 skill 负责 CUBE 业务语义与仓库边界。

## 完成定义

- 结论与当前源码、最终 diff 和实际验证证据一致。
- 参数任务覆盖入口、校验、持久化、运行态、消费点和启动加载；协议任务覆盖解析、传递、消费、反馈、异常和兼容性。
- 代码改动完成相关静态检查和构建；无法执行的硬件、台架或现场验证明确列为残余风险。
- 发布任务的版本、协议、参数存储、CHANGELOG、版本方案、构建证据、Git 文件范围和本机流程资料一致。

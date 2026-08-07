---
name: cube-development
description: Repository-scoped workflow for every D:\CUBE engineering task, including read-only analysis, implementation, debugging, system architecture, related repositories, firmware, protocols, parameters, faults, CPU3 UI, documents, long-running work, versioning, builds, Git commits, and releases. Use whenever the task reads, changes, reviews, documents, validates, amends, commits, or releases the CUBE project or its explicitly related source repositories.
---

# CUBE 项目开发

## 规则归属

- 通用的分析、编辑、调试、验证和 Git 安全流程由全局 `engineering-workflow` 提供。
- CUBE 的路径、编码、协议、版本、参数、文档和提交规则只在本仓库的 `AGENTS.md` 与本 skill 维护。
- 每次进入提交、amend 或发布阶段，必须在当前 turn 重新读取 `AGENTS.md`、本文件、`references/docs-git-validation.md` 和 `references/version-build-release.md`；不得依赖旧对话摘要声称已经读取。
- 按“用户当前明确决定、当前实现源码与工作区、已批准且仍适用的正式资料、现场原始证据、历史资料、推测建议”的顺序裁决事实。发生冲突时列出冲突并停止扩大结论，不得用旧正式文档覆盖用户刚确认的事实。
- 先按 `references/task-lifecycle-and-scope.md` 判定只读、方案、实现或发布阶段，再按 `references/project-map-and-dependencies.md` 盘点完整工程对象和关联仓库。

## CUBE 硬约束

- 正式业务文档只维护 `docs/`；`docs-site/` 是本机预览层，旧 `site/` 不再维护。
- `docs/00_程序流程导航/` 与 `tools/` 在本机同步但不纳入 Git，不得 `git add -f`。
- 修改已有文件前确认编码和换行。CPU2/CPU3 旧源码可能是 GBK/936；文档、skill 和工具脚本使用 UTF-8。
- C/C++ 新增或修改注释统一使用 `/* ... */`，不得新增 `//`。
- 修改 Markdown、Word、HTML 或其它源文档时默认不生成、刷新或配套新增 PDF；只有用户当前任务明确要求 PDF 交付物时才生成并验证。
- 固件、硬件、台架、现场和 SIL 证据必须分层描述；构建或静态检查不等于真实设备验证。

## 开始和结束门禁

- 非简单任务先运行 `scripts/snapshot_cube_context.ps1`；长任务、跨仓库任务或可能经历上下文压缩的任务同时写入 JSON 状态文件，并按 `AGENTS.md` 创建根目录 Snapshot。
- 记录分支、HEAD、upstream ahead/behind、工作树身份、CPU2/CPU3 版本、`DEVICE_PROTOCOL_VERSION`、参数存储版本，以及 staged、unstaged、untracked 三层状态。
- 需要写入的非简单任务先用 `scripts/manage_task_lease.py` 声明写入路径；与其它活动任务重叠时停止写入，改用独立 worktree 或等待边界释放。
- 长任务状态同时记录用户已确认决定、假设、开放问题、事实基线、最近批准交付物和下一步；上下文恢复后不得只依赖旧摘要继续写入。
- 上下文恢复时先运行 `scripts/check_context_snapshot.py`；分支、HEAD、Git状态或 watch 文件变化时重新锁定事实和范围，不直接续写。
- 发现并行改动时只处理能够安全分离的范围；不得 reset、checkout、覆盖或清理用户改动。
- “继续”只完成已经确认的当前阶段和交付物，不自动进入下一分册、下一版本、提交或发布；“先输出理解，我来纠正”在用户纠正前保持只读。
- 任务结束前运行根目录 Compare，并报告 Git 提交内容、本机同步内容、已验证项和未验证项。

## 低自由度执行

- 易漏、易错或会改变发布状态的步骤优先使用本 skill 的脚本，不把自然语言清单当作已经执行的门禁。
- 修改本 skill 后运行 `scripts/validate_cube_skill.py`；它使用 Python 标准库检查触发信息、reference 路由、界面元数据、UTF-8 和脚本语法。
- 提交前运行 `scripts/bootstrap_cube_hooks.py --check` 核对本机 hook 模板和 `core.hooksPath`；只有明确需要恢复本机 hooks 时才使用 `--apply`，已有不同内容不得静默覆盖。
- 用 `scripts/record_validation_evidence.py` 创建或复核绑定源文件指纹的验证清单，不手工复用已经过期的构建、渲染或现场结论。
- 构建、静态检查、渲染和现场证据必须绑定其对应源码/文档指纹；最终源变化后旧证据失效。

## 提交硬门禁

- 用户说“全部提交”“提交所有改动”或同义表达时，提交模式必须设为 `all`。这已经授权纳入任务快照中全部 Git 可见且不违反仓库硬规则的改动，包括删除和未跟踪文件。
- `all` 模式不得自行排除任何允许提交的改动。遇到禁止提交、敏感或必须由用户决定的项目时，停止并请求决定；不得排除后继续提交。
- 只有用户点名文件、模块或明确要求保留其它改动时才使用 `scoped` 模式。
- 提交时必须显式设置 `CUBE_COMMIT_SCOPE=all` 或 `CUBE_COMMIT_SCOPE=scoped`，由本地 `pre-commit` 门禁检查；`scoped` 还必须设置 `CUBE_COMMIT_SCOPE_FILE=<任务JSON>`，未设置时禁止提交。
- 禁止只执行 `git commit -m "<标题>"`。必须使用消息文件或多段 `-m`，正文严格包含“版本、协议版本/兼容性、本次修改、验证”四个区块。
- 提交前、提交后都读取完整提交正文；只通过标题版本门禁不代表提交信息合格。

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
- 构建、版本、CHANGELOG 和版本改动与测试方案：`references/version-build-release.md`
- 构建、静态检查、渲染、台架和现场证据的新鲜度与结构化记录：`references/validation-evidence.md`
- 文档、PDF 边界、暂存、提交和门禁：`references/docs-git-validation.md`
- CUBE 正式 Word、参考文档边界、候选提升、目录更新和逐页验收：`references/formal-word-documents.md`
- `docs-site/` 生成边界、受控构建、固定预览和进程收尾：`references/docs-site-preview.md`

组合任务读取所有命中的 reference。格式类文件同时使用对应的文档、表格、PDF 或浏览器技能；本 skill 负责 CUBE 业务语义与仓库边界。

## 完成定义

- 结论与当前源码、最终 diff 和实际验证证据一致。
- 只读、方案、实现、发布阶段没有越权跳转；当前交付物完成后不擅自扩展下一阶段。
- 跨组件或跨仓库任务已覆盖实际参与链路的两端，并记录外部仓库的分支、HEAD 和未提交状态。
- 长任务的最新用户决定、开放问题和交付物状态可从 JSON 恢复；写入任务未与活动租约发生路径冲突。
- 参数任务覆盖入口、校验、持久化、运行态、消费点和启动加载；协议任务覆盖解析、传递、消费、反馈、异常和兼容性。
- 代码改动完成相关静态检查和构建；无法执行的硬件、台架或现场验证明确列为残余风险。
- 发布任务的版本、协议、参数存储、CHANGELOG、版本方案、构建证据、Git 文件范围和本机流程资料一致。

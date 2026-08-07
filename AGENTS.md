# AGENTS.md

- CUBE 项目工作优先使用仓库级 `.agents/skills/cube-development/SKILL.md`；通用工程流程使用全局 `engineering-workflow`。每次进入提交、amend 或发布阶段，当前执行者必须重新读取仓库级 skill 及其命中的 Git/发布 reference，不得用旧对话摘要或“此前已经读过”替代。
- 修改文件时注意编码，避免中文乱码。
- 当前正式文档源只维护 `docs/`；不要新增、恢复或继续维护 `LTD_MAIN_CPU2/docs/`、`LTD_DISPLAY_CPU3/docs/`。
- `docs-site/` 仅作为本机预览工具，内容从 `docs/` 同步生成，不作为业务资料来源，不纳入 Git 跟踪。
- `docs/00_程序流程导航/` 与 `tools/` 保持本机正常更新和同步，但不再纳入 Git 跟踪或提交；不要使用 `git add -f` 强制加入。
- 旧 `site/` 已淘汰；不要恢复为维护入口或发布目标。
- 后续版本提交生成的新文档仍放在 `docs/00_构建与版本/版本改动与测试/`，人工索引可后补，不应阻塞版本提交。
- 创建、修改或改写 Git 提交信息时，标题必须包含受影响固件版本号；整机、文档、工具或跨 CPU 改动写 `（CPU2 Vx.x.x.x / CPU3 Vy.y.y.y）`，单 CPU 改动至少写对应 CPU 版本。正文必须使用仓库级 skill 规定的“版本、协议版本/兼容性、本次修改、验证”四段式格式，禁止只写标题。提交后推送前运行 `py tools\check_commit_subject_versions.py --range origin/MAIN..HEAD`。
- 用户明确要求“全部提交”“提交所有改动”或同义表达时，表示已经授权提交任务开始快照中全部 Git 可见且不违反仓库硬规则的改动，包括已暂存、未暂存、未跟踪文件和删除。执行者不得因归属、质量、风险或个人判断自行排除；如果发现禁止提交、敏感或必须由用户决策的项目，必须在提交前停止并取得新的明确决定，不能排除后继续提交。
- 用户要求按指定范围提交时才允许保留其它工作区改动；提交前必须报告纳入项和保留项。任何提交模式都不得通过 reset、checkout、覆盖或清理损坏用户改动。
- 本工作区提交门禁放在本地 `tools/git-hooks/pre-commit` 和 `tools/git-hooks/commit-msg`；提交命令必须显式设置环境变量 `CUBE_COMMIT_SCOPE=all` 或 `CUBE_COMMIT_SCOPE=scoped`。`pre-commit` 检查范围，`commit-msg` 同时检查版本标题和四段式正文。当前工作区应保持 `git config core.hooksPath tools/git-hooks`；新克隆需先从本地同步源恢复 `tools/`，再配置 hooksPath 后提交。
- CUBE 项目规则只维护在仓库级 `.agents/skills/cube-development/`、本文件及正式项目文档中；不得把 `D:\CUBE` 路径、固件版本、协议、目录或发布规则写入全局 `engineering-workflow`。旧全局 skill 已归档到 `C:\Users\admin\.codex\backups_state\cube-development-global-20260807`，不再参与 skill 发现；不要从备份目录直接维护或恢复同名全局 skill。
- 本仓库的文档结构边界以本文件和 `docs/00_构建与版本/文档库治理清单.md` 为准。

## 仓库根目录与临时产物门禁

- 仓库根目录目录白名单为：`.agents/`、`.git/`、`.github/`、`.metadata/`、`.vscode/`、`.worktrees/`、`build/`、`cmake/`、`docs/`、`docs-site/`、`LTD_DISPLAY_CPU3/`、`LTD_MAIN_CPU2/`、`old/`、`outputs/`、`tmp/`、`tools/`、`WirelessHost_V4.1_init/`。`.agents/` 只存放仓库级 Codex skill。机器可执行白名单以 `tools/check_workspace_root_layout.ps1` 中的 `$AllowedRootDirectories` 为准；新增根目录前必须先取得明确批准并同步更新本节和脚本。
- `build/` 专用于 CPU2/CPU3 固件构建，一级子项只允许 `build/LTD_MAIN_CPU2/` 和 `build/LTD_DISPLAY_CPU3/`；不得在其中存放主机测试、分析器、调试日志、临时可执行文件或其它工具产物。
- 固件以外的临时产物只能写入 `tmp/<任务号>/`；需要长期保留的本机证据写入 `outputs/<任务号>/`。主机测试及分析器的编译输出统一放在 `tmp/<任务号>/build/`，不得在仓库根目录、`build/` 或源码、正式文档目录旁创建临时文件夹。任务号优先使用 Codex 任务 ID；没有任务 ID 时使用唯一、可追溯的任务标签。
- 任务开始前运行 `powershell -ExecutionPolicy Bypass -File tools/check_workspace_root_layout.ps1 -Mode Snapshot -SnapshotPath tmp/<任务号>/root-layout-before.json`；任务结束前运行同一脚本的 `-Mode Compare`。基线后新增的白名单外根目录必须立即失败并先追查来源，不得继续交付。
- 需要检查当前根目录是否已经存在历史违规项时，运行 `powershell -ExecutionPolicy Bypass -File tools/check_workspace_root_layout.ps1 -Mode Check`；不得为消除告警而把异常目录加入 `.gitignore`。
- PowerShell 脚本和命令禁止使用 `md`、`mkdir` 等目录创建别名；目录路径必须先解析并校验为明确绝对路径，再使用 `[IO.Directory]::CreateDirectory(<path>)` 创建（`New-Item` 没有 `-LiteralPath` 参数），读取和删除路径必须使用支持的 `-LiteralPath` 参数。自定义函数必须采用唯一、语义明确的 `Verb-Noun` 名称，例如 `ConvertTo-MarkdownCell`，不得定义 `Md` 等可能与别名、函数、cmdlet 或外部命令冲突的短名称；新增前使用 `Get-Command -Name <名称> -All` 检查冲突。
- 任何清理脚本都必须使用 `[CmdletBinding(SupportsShouldProcess = $true)]`，默认等价于 `-WhatIf`，只有显式 `-Apply` 才允许实际删除；必须对经过解析和边界校验的明确目标逐项调用 `ShouldProcess`，禁止对仓库根目录使用通配符、递归枚举后批量删除或依赖未解析的环境变量。
- 不得把 `传感器类型/`、`设备信息/`、`Enum/` 或以后出现的类似异常名称加入 `.gitignore`；此类目录应由根目录门禁拦截并追查创建进程。

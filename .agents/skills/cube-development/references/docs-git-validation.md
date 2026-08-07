# CUBE 文档、Git 和提交门禁

## 目录

- [文档与资料边界](#文档与资料边界)
- [Office、HTML 和 PDF](#officehtml-和-pdf)
- [当前 turn 必读](#当前-turn-必读)
- [提交范围模式](#提交范围模式)
- [提交前顺序](#提交前顺序)
- [提交信息强制格式](#提交信息强制格式)
- [CUBE 发布一致性](#cube-发布一致性)
- [验证边界](#验证边界)

## 文档与资料边界

- `docs/` 是唯一正式业务文档源；不要新增或恢复 `LTD_MAIN_CPU2/docs/`、`LTD_DISPLAY_CPU3/docs/`。
- `docs/00_程序流程导航/`、`tools/`、`docs-site/`、`outputs/`、临时渲染和中间产物不进入 Git。
- 清理上述本机专用或淘汰路径中的历史跟踪文件时，允许暂存删除；仍禁止新增、修改或用 `git add -f` 恢复这些路径。
- `.agents/skills/cube-development/` 是 CUBE 项目 skill 的正式仓库源，允许跟踪；不要把其中规则反向复制到全局 `engineering-workflow`。
- 文档整理默认不升级 CPU2/CPU3 固件版本；只有固件输出或行为变化才按版本规则处理。
- 移动或重命名资料后同步 README、CHANGELOG、计划、索引和旧路径引用，并运行 Markdown 链接与差异检查。

用户所说的“Git库文档区、项目正式文档区、我的Git库中的文档”默认指 `docs/`；“临时产物”指 `tmp/<任务号>/`，“长期本机证据”指 `outputs/<任务号>/`。如果“我的库、文档区域、资料库”仍无法唯一定位，先确认目标，不得自行移动到 Windows Documents、云端库或其它仓库。

## Office、HTML 和 PDF

- 修改 `.docx`、`.xlsx` 或复杂 HTML 时同时使用对应格式 skill，完成结构与视觉检查。
- 现有 PDF 仍检查来源、关联性、时效、敏感性和提交清单。
- 修改 Markdown、Word、HTML 或其它源文件不自动生成、刷新或新增 PDF；只有用户在当前任务明确要求 PDF 交付物时才生成并执行 PDF 校验。
- 面向外发的资料与内部治理资料分开；提交或推送前检查禁止外传、密钥、现场原始信息和个人数据。

## 当前 turn 必读

提交、amend 或发布前，当前执行者必须重新读取：

1. 仓库根 `AGENTS.md`。
2. `.agents/skills/cube-development/SKILL.md`。
3. 本文件。
4. `version-build-release.md`。

不得使用旧会话摘要、其它代理结论或“此前已读取”替代。

提交前先运行 `py .agents/skills/cube-development/scripts/bootstrap_cube_hooks.py --check --repo-root D:\CUBE`。该检查同时确认本机 hooks 与仓库级模板一致、`core.hooksPath=tools/git-hooks`；失败时不得用 `--no-verify` 绕过。

## 提交范围模式

### 全部提交

用户使用“全部提交”“提交所有改动”“当前所有改动并提交”等表达时：

- 模式固定为 `all`，不得自行改成范围提交。
- 任务快照中全部 Git 可见且不违反仓库硬规则的 staged、unstaged、untracked 和删除都必须纳入。
- 文件质量、归属不明、可能属于其它对话或执行者认为风险较高，都不是静默排除理由。
- 如果存在禁止提交、敏感、用户决定冲突或仓库规则无法同时满足的文件，必须在提交前停止并请求决定。
- 禁止排除部分文件后继续提交并声称完成“全部提交”。
- 暂存后运行 `scripts/check_commit_scope.py --mode all`；存在任何未暂存或未跟踪的可提交文件即失败。

### 指定范围提交

- 只有用户点名文件、模块、提交主题，或明确要求保留其它工作区改动时使用 `scoped`。
- 提交前列出纳入项和保留项；混合状态分别检查 index 与 worktree。
- 不使用 reset、checkout、覆盖、清理或重生成消除范围外改动。
- `scoped` 必须把当前任务 JSON 作为 `CUBE_COMMIT_SCOPE_FILE`；门禁按其中 `task.writePaths` 或 `task.allowedPaths` 逐项核对暂存文件。只设置 `CUBE_COMMIT_SCOPE=scoped` 而没有范围文件时提交失败。

## 提交前顺序

1. 取得两次稳定状态快照，确认没有未知并发写入。
2. 根据用户原话确定 `all` 或 `scoped`，不得自行降级。
3. 按最终 diff 判断版本、协议、参数存储、CHANGELOG、版本方案和流程资料影响。
4. 暂存后检查 `git diff --cached --name-status`、关键 diff、`git diff --cached --check` 和敏感信息。
5. 运行 `scripts/check_commit_scope.py --mode <all|scoped>`。
6. 使用完整消息文件创建提交，并显式设置 `CUBE_COMMIT_SCOPE`；`scoped` 同时设置 `CUBE_COMMIT_SCOPE_FILE=<任务JSON>`。
7. 提交后检查完整正文、文件范围、剩余状态和版本标题门禁。

## 提交信息强制格式

提交信息使用中文。标题必须包含受影响固件版本：文档、工具、整机或跨 CPU 改动写 `（CPU2 Vx.x.x.x / CPU3 Vy.y.y.y）`；单 CPU 改动至少写对应 CPU 版本。

正文必须严格按以下四个区块组织，区块标题、顺序和非空项目均为强制要求：

```text
<类型>: <简短说明>（版本号）

版本：
- CPU2：<旧版本> -> <新版本，或保持不变>
- CPU3：<旧版本> -> <新版本，或保持不变>

协议版本/兼容性：
- <共享协议、参数存储、外部协议和兼容影响；不变化也必须明确写出>

本次修改：
- <改动点 1>
- <改动点 2>

验证：
- <已运行的检查、测试或构建>
- <未运行项、原因和证据边界>
```

禁止只写标题，禁止把四个区块压成单段，禁止以“优化、完善、功能正常”等空泛描述代替具体事实。

提交前运行：

```powershell
py .agents/skills/cube-development/scripts/check_commit_message_format.py --message-file <提交消息文件>
```

## CUBE 发布一致性

- 固件相关提交执行 `check_version_bumped.py`，并将版本头、CHANGELOG 和版本方案纳入同一提交。
- CHANGELOG 和版本方案必须覆盖暂存区全部行为变化、兼容影响、验证和未验证风险。
- 共享协议变化同步协议变更记录，并保证 CPU2/CPU3 版本组合一致。
- 提交后、推送前运行 `py tools/check_commit_subject_versions.py --range origin/MAIN..HEAD`。
- 不因提交自动推送；推送必须由用户明确要求。

## 验证边界

- 文档改动至少运行文档结构、Markdown 链接、编码和 `git diff --check`。
- 源码改动运行相关静态契约与 clean-first 构建；最终代码晚于构建时必须重建。
- 真实目标板、传感器、RS485、FRAM、OLED、电机、故障注入和现场验证未执行时必须明确列出，不得用静态或构建证据替代。

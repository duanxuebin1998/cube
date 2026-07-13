# AGENTS.md

- 优先遵循当前项目可用 skill；修改文件时注意编码，避免中文乱码。
- 当前正式文档源只维护 `docs/`；不要新增、恢复或继续维护 `LTD_MAIN_CPU2/docs/`、`LTD_DISPLAY_CPU3/docs/`。
- `docs-site/` 仅作为本机预览工具，内容从 `docs/` 同步生成，不作为业务资料来源，不纳入 Git 跟踪。
- `docs/00_程序流程导航/` 与 `tools/` 保持本机正常更新和同步，但不再纳入 Git 跟踪或提交；不要使用 `git add -f` 强制加入。
- 旧 `site/` 已淘汰；不要恢复为维护入口或发布目标。
- 后续版本提交生成的新文档仍放在 `docs/00_构建与版本/版本改动与测试/`，人工索引可后补，不应阻塞版本提交。
- 创建、修改或改写 Git 提交标题时，标题必须包含受影响固件版本号；整机、文档、工具或跨 CPU 改动写 `（CPU2 Vx.x.x.x / CPU3 Vy.y.y.y）`，单 CPU 改动至少写对应 CPU 版本；提交后推送前运行 `py tools\check_commit_subject_versions.py --range origin/MAIN..HEAD`。
- 本工作区提交标题门禁仍放在本地 `tools/git-hooks/commit-msg`；当前工作区应保持 `git config core.hooksPath tools/git-hooks`。新克隆需先从本地同步源恢复 `tools/`，再配置 hooksPath 后提交。
- 不修改 `C:\Users\admin\.codex\skills` 下的 skill；本仓库的文档结构边界以本文件和 `docs/00_构建与版本/文档库治理清单.md` 为准。

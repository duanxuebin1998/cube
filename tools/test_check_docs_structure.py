#!/usr/bin/env python3
"""check_docs_structure.py 的结构边界测试。"""

from __future__ import annotations

import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_docs_structure.py")


def load_module():
    spec = importlib.util.spec_from_file_location("check_docs_structure", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_docs_structure.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckDocsStructureTests(unittest.TestCase):
    def test_configure_output_uses_utf8_when_stream_supports_reconfigure(self) -> None:
        module = load_module()
        stdout = io.TextIOWrapper(io.BytesIO(), encoding="cp936")
        stderr = io.TextIOWrapper(io.BytesIO(), encoding="cp936")

        module.configure_output(stdout=stdout, stderr=stderr)

        self.assertEqual("utf-8", stdout.encoding.lower().replace("_", "-"))
        self.assertEqual("utf-8", stderr.encoding.lower().replace("_", "-"))

    def test_run_git_uses_utf8_output_decoding(self) -> None:
        module = load_module()
        calls: list[dict[str, object]] = []

        def fake_run(command, **kwargs):  # noqa: ANN001
            calls.append(kwargs)
            return module.subprocess.CompletedProcess(command, 0, stdout="", stderr="")

        original_run = module.subprocess.run
        try:
            module.subprocess.run = fake_run
            module.run_git(["status", "--short"])
        finally:
            module.subprocess.run = original_run

        self.assertEqual(module.OUTPUT_ENCODING, calls[0].get("encoding"))
        self.assertEqual("replace", calls[0].get("errors"))

    def test_version_output_dir_is_required_but_appendable(self) -> None:
        module = load_module()

        self.assertEqual(
            "docs/00_构建与版本/版本改动与测试",
            module.VERSION_OUTPUT_DIR,
        )
        self.assertIn(
            "docs/00_构建与版本/版本改动与测试/README.md",
            module.REQUIRED_PATHS,
        )
        self.assertIn("AGENTS.md", module.REQUIRED_PATHS)

    def test_agents_boundary_phrases_keep_repo_rules_local(self) -> None:
        module = load_module()

        joined = "\n".join(module.AGENTS_REQUIRED_PHRASES)

        self.assertIn("docs/", joined)
        self.assertIn("LTD_MAIN_CPU2/docs", joined)
        self.assertIn("LTD_DISPLAY_CPU3/docs", joined)
        self.assertIn("docs-site", joined)
        self.assertIn("site", joined)
        self.assertIn("C:\\Users\\admin\\.codex\\skills", joined)

    def test_local_generated_and_legacy_site_outputs_stay_untracked(self) -> None:
        module = load_module()

        self.assertIn("tmp", module.LOCAL_ONLY_DIRS)
        self.assertIn("tmp", module.NOT_TRACKED_PREFIXES)
        self.assertIn("tools/build_static_docs_site.py", module.NOT_TRACKED_PREFIXES)
        self.assertIn("tools/check_static_docs_site_ui.py", module.NOT_TRACKED_PREFIXES)
        self.assertIn("tools/ltd-report-theme.css", module.NOT_TRACKED_PREFIXES)

    def test_readme_warning_limit_matches_governance_rule(self) -> None:
        module = load_module()

        self.assertEqual(60, module.README_WARN_LINE_LIMIT)

    def test_readme_longer_than_limit_warns_without_blocking(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            original_root = module.ROOT
            original_docs = module.DOCS
            root = Path(tmp)
            docs = root / "docs"
            topic = docs / "topic"
            topic.mkdir(parents=True)
            (docs / "README.md").write_text("# docs\n", encoding="utf-8")
            (topic / "README.md").write_text("\n".join(["line"] * 61), encoding="utf-8")

            try:
                module.ROOT = root
                module.DOCS = docs
                errors: list[str] = []
                warnings: list[str] = []
                module.check_readmes(errors, warnings)
            finally:
                module.ROOT = original_root
                module.DOCS = original_docs

        self.assertEqual([], errors)
        self.assertEqual(["README 行数偏长：docs/topic/README.md（61 行）"], warnings)

    def test_legacy_static_site_scripts_fail_when_recreated(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            original_root = module.ROOT
            root = Path(tmp)
            (root / "tools").mkdir()
            (root / "tools" / "build_static_docs_site.py").write_text("# old site\n", encoding="utf-8")

            try:
                module.ROOT = root
                errors: list[str] = []
                module.check_legacy_dirs(errors)
            finally:
                module.ROOT = original_root

        self.assertEqual(
            ["不应恢复旧文档目录、旧站点或旧静态站脚本：tools/build_static_docs_site.py"],
            errors,
        )

    def test_legacy_reference_keywords_only_allowed_in_governance_docs(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            docs_dir = Path(tmp)
            allowed = docs_dir / "00_构建与版本"
            ordinary = docs_dir / "03_问题分析与整改"
            allowed.mkdir(parents=True)
            ordinary.mkdir(parents=True)
            (allowed / "文档库迁移映射表.md").write_text(
                "`LTD_MAIN_CPU2/docs/` 已迁移。\n",
                encoding="utf-8",
            )
            (ordinary / "问题复盘.md").write_text(
                "仍然查看 `LTD_MAIN_CPU2/docs/`。\n",
                encoding="utf-8",
            )

            problems = module.find_disallowed_legacy_reference_lines(docs_dir)

        self.assertEqual(
            [("03_问题分析与整改/问题复盘.md", 1, "LTD_MAIN_CPU2/docs")],
            problems,
        )

    def test_latest_version_output_names_uses_all_docs_on_latest_date(self) -> None:
        module = load_module()

        latest = module.latest_version_output_names(
            [
                Path("README.md"),
                Path("2026-06-01_CPU2_V1.0.0.0_旧方案_改动与测试方案.md"),
                Path("2026-06-27_CPU2_V1.2.0.0_新方案A_改动与测试方案.md"),
                Path("2026-06-27_CPU3_V1.3.0.0_新方案B_改动与测试方案.md"),
            ]
        )

        self.assertEqual(
            [
                "2026-06-27_CPU2_V1.2.0.0_新方案A_改动与测试方案.md",
                "2026-06-27_CPU3_V1.3.0.0_新方案B_改动与测试方案.md",
            ],
            latest,
        )

    def test_version_index_freshness_warning_is_non_blocking(self) -> None:
        module = load_module()
        latest = "2026-06-27_CPU2_V1.2.0.0_新方案_改动与测试方案.md"

        warnings = module.build_version_index_warnings(
            [latest],
            {
                "docs/00_构建与版本/版本总览.md": "旧方案",
                "docs/00_构建与版本/版本改动与测试/README.md": f"已收录 {latest}",
            },
        )

        self.assertEqual(1, len(warnings))
        self.assertIn("版本总览.md", warnings[0])
        self.assertIn("不阻塞版本提交", warnings[0])

    def test_governance_entry_links_require_new_material_and_version_output_paths(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            original_root = module.ROOT
            root = Path(tmp)
            docs = root / "docs"
            build = docs / "00_构建与版本"
            build.mkdir(parents=True)
            (docs / "README.md").write_text(
                "查看 `00_构建与版本/新增资料落位检查表.md`。\n",
                encoding="utf-8",
            )
            (build / "README.md").write_text(
                "入口 `新增资料落位检查表.md`，版本输出 `版本改动与测试/`。\n",
                encoding="utf-8",
            )
            (build / "文档库使用指南.md").write_text(
                "新增资料入口 `新增资料落位检查表.md`，版本输出 `版本改动与测试/`。\n",
                encoding="utf-8",
            )
            (build / "文档库治理清单.md").write_text(
                "新增资料入口 `新增资料落位检查表.md`，版本输出 `版本改动与测试/`。\n",
                encoding="utf-8",
            )

            try:
                module.ROOT = root
                errors: list[str] = []
                module.check_governance_entry_links(errors)
            finally:
                module.ROOT = original_root

        self.assertEqual(
            [
                "治理入口缺少必要引用：docs/README.md -> 00_构建与版本/版本改动与测试/",
            ],
            errors,
        )

    def test_new_material_checklist_requires_core_routing_rules(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            original_root = module.ROOT
            root = Path(tmp)
            checklist = root / "docs" / "00_构建与版本" / "新增资料落位检查表.md"
            checklist.parent.mkdir(parents=True)
            checklist.write_text(
                "\n".join(
                    [
                        "# 新增资料落位检查表",
                        "版本发布仍按既有 skill 规则新增 `docs/00_构建与版本/版本改动与测试/` 文档。",
                        "CPU2/CPU3 程序流程 -> `docs/00_程序流程导航/`",
                        "CPU2/CPU3 共享协议、寄存器、DSM、SI -> `docs/01_协议与寄存器/`",
                        "问题现象、原因分析、整改方案、风险复查 -> `docs/03_问题分析与整改/`",
                        "SIL、MISRA/CERT、功能安全外部资料 -> `docs/06_SIL功能安全认证/`",
                        "不因为新增资料而手工维护 `docs-site/src/content/docs/`。",
                    ]
                ),
                encoding="utf-8",
            )

            try:
                module.ROOT = root
                errors: list[str] = []
                module.check_new_material_checklist(errors)
            finally:
                module.ROOT = original_root

        self.assertEqual(
            [
                "新增资料落位检查表缺少关键规则：硬件原理图、芯片手册、固件硬件对照",
                "新增资料落位检查表缺少关键规则：不把 `tmp/` 里的渲染、截图、抽取文本或 PDF 处理缓存当作正式资料提交",
            ],
            errors,
        )

    def test_staging_governance_docs_require_current_read_only_commands(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            original_root = module.ROOT
            root = Path(tmp)
            build = root / "docs" / "00_构建与版本"
            build.mkdir(parents=True)
            (build / "文档迁移提交前收尾清单.md").write_text(
                "`--stage-commands` 只列出 `--mixed-resolution-plan`。\n"
                "`--mixed-closure-audit` 和 `--version-output-contract` 在别处出现。\n",
                encoding="utf-8",
            )
            (build / "文档库治理清单.md").write_text(
                "`--stage-commands` 只列出 `--version-output-contract`。\n"
                "`--mixed-closure-audit` 在别处出现。\n",
                encoding="utf-8",
            )

            try:
                module.ROOT = root
                errors: list[str] = []
                module.check_staging_governance_commands(errors)
            finally:
                module.ROOT = original_root

        self.assertEqual(
            [
                "文档迁移提交前收尾清单.md 缺少当前收口命令：--write-local-audit",
                "文档迁移提交前收尾清单.md 的 --stage-commands 说明缺少当前收口命令：--mixed-closure-audit",
                "文档迁移提交前收尾清单.md 的 --stage-commands 说明缺少当前收口命令：--version-output-contract",
                "文档迁移提交前收尾清单.md 的 --stage-commands 说明缺少当前收口命令：--write-local-audit",
                "文档迁移提交前收尾清单.md 的 --stage-commands 说明缺少当前收口命令：migration-add",
                "文档库治理清单.md 缺少当前收口命令：--write-local-audit",
                "文档库治理清单.md 的 --stage-commands 说明缺少当前收口命令：--mixed-closure-audit",
                "文档库治理清单.md 的 --stage-commands 说明缺少当前收口命令：--write-local-audit",
                "文档库治理清单.md 的 --stage-commands 说明缺少当前收口命令：migration-add",
            ],
            errors,
        )

    def test_staging_governance_docs_require_local_audit_command(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            original_root = module.ROOT
            root = Path(tmp)
            build = root / "docs" / "00_构建与版本"
            build.mkdir(parents=True)
            shared_text = (
                "`--stage-commands` 包含 `--mixed-closure-audit`、`--version-output-contract` 和 `migration-add`。\n"
                "`--mixed-closure-audit` 可检查混合文件。\n"
                "`--version-output-contract` 可检查版本输出契约。\n"
            )
            (build / "文档迁移提交前收尾清单.md").write_text(shared_text, encoding="utf-8")
            (build / "文档库治理清单.md").write_text(shared_text, encoding="utf-8")

            try:
                module.ROOT = root
                errors: list[str] = []
                module.check_staging_governance_commands(errors)
            finally:
                module.ROOT = original_root

        self.assertEqual(
            [
                "文档迁移提交前收尾清单.md 缺少当前收口命令：--write-local-audit",
                "文档迁移提交前收尾清单.md 的 --stage-commands 说明缺少当前收口命令：--write-local-audit",
                "文档库治理清单.md 缺少当前收口命令：--write-local-audit",
                "文档库治理清单.md 的 --stage-commands 说明缺少当前收口命令：--write-local-audit",
            ],
            errors,
        )

    def test_flow_html_asset_check_reports_missing_local_references(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            original_root = module.ROOT
            original_docs = module.DOCS
            root = Path(tmp)
            docs = root / "docs"
            flow_dir = docs / "00_程序流程导航" / "CPU2"
            assets = flow_dir / "assets"
            assets.mkdir(parents=True)
            (assets / "流程文档样式.css").write_text("body {}\n", encoding="utf-8")
            (flow_dir / "测量流程.html").write_text(
                "\n".join(
                    [
                        "<!doctype html>",
                        '<link rel="stylesheet" href="assets/流程文档样式.css?v=1">',
                        '<script src="assets/流程文档交互.js#main"></script>',
                        '<a href="#top">回到顶部</a>',
                        '<a href="https://example.com/spec.html">外部资料</a>',
                    ]
                ),
                encoding="utf-8",
            )

            try:
                module.ROOT = root
                module.DOCS = docs
                errors: list[str] = []
                module.check_flow_html_local_references(errors)
            finally:
                module.ROOT = original_root
                module.DOCS = original_docs

        self.assertEqual(
            [
                "流程 HTML 引用不存在：docs/00_程序流程导航/CPU2/测量流程.html -> assets/流程文档交互.js#main",
            ],
            errors,
        )

    def test_same_stem_doc_exports_find_markdown_pdf_pairs(self) -> None:
        module = load_module()

        with tempfile.TemporaryDirectory() as tmp:
            docs_dir = Path(tmp)
            (docs_dir / "topic").mkdir()
            (docs_dir / "assets").mkdir()
            (docs_dir / "topic" / "方案.md").write_text("# 方案\n", encoding="utf-8")
            (docs_dir / "topic" / "方案.pdf").write_bytes(b"%PDF-1.4\n")
            (docs_dir / "topic" / "仅Markdown.md").write_text("# 仅 Markdown\n", encoding="utf-8")
            (docs_dir / "assets" / "导出.md").write_text("# 支撑资源\n", encoding="utf-8")
            (docs_dir / "assets" / "导出.pdf").write_bytes(b"%PDF-1.4\n")

            matches = module.find_same_stem_doc_exports(docs_dir)

        self.assertEqual([("topic/方案", [".md", ".pdf"])], matches)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""check_docs_staging.py 的提交前状态测试。"""

from __future__ import annotations

import importlib.util
import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_docs_staging.py")


def load_module():
    spec = importlib.util.spec_from_file_location("check_docs_staging", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_docs_staging.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckDocsStagingTests(unittest.TestCase):
    def test_configure_output_uses_utf8_when_stream_supports_reconfigure(self) -> None:
        module = load_module()
        stdout = io.TextIOWrapper(io.BytesIO(), encoding="cp936")
        stderr = io.TextIOWrapper(io.BytesIO(), encoding="cp936")

        module.configure_output(stdout=stdout, stderr=stderr)

        self.assertEqual("utf-8", stdout.encoding.lower().replace("_", "-"))
        self.assertEqual("utf-8", stderr.encoding.lower().replace("_", "-"))

    def test_parse_porcelain_v1_status_extracts_index_worktree_and_path(self) -> None:
        module = load_module()

        entries = module.parse_porcelain_v1_status(
            [
                "AM docs/00_构建与版本/文档库治理清单.md",
                " M docs/README.md",
                'R  "old path.md" -> "docs/new path.md"',
                "?? tools/check_docs_staging.py",
            ]
        )

        self.assertEqual(("A", "M", "docs/00_构建与版本/文档库治理清单.md"), entries[0])
        self.assertEqual((" ", "M", "docs/README.md"), entries[1])
        self.assertEqual(("R", " ", "docs/new path.md"), entries[2])
        self.assertEqual(("?", "?", "tools/check_docs_staging.py"), entries[3])

    def test_build_staging_report_flags_mixed_states_and_unstaged_cleanup(self) -> None:
        module = load_module()

        report = module.build_staging_report(
            [
                ("A", "M", "docs/00_构建与版本/文档库治理清单.md"),
                ("D", " ", "tmp/pdfs/staged-page.png"),
                (" ", "D", "tmp/pdfs/unstaged-page.png"),
                ("D", " ", "tools/build_static_docs_site.py"),
                (" ", "D", "tools/check_static_docs_site_ui.py"),
                ("D", " ", "LTD_MAIN_CPU2/docs/README.md"),
                (" ", "D", "LTD_DISPLAY_CPU3/docs/README.md"),
                ("?", "?", "tools/check_docs_staging.py"),
                ("M", " ", "LTD_MAIN_CPU2/Application/Src/app_main.c"),
                ("M", " ", "tools/check_motor_motion_target_plan.py"),
                ("M", " ", "docs/01_协议与寄存器/README.md"),
            ]
        )

        self.assertEqual(["docs/00_构建与版本/文档库治理清单.md"], report.mixed_paths)
        self.assertEqual(["tmp/pdfs/unstaged-page.png"], report.local_output_paths)
        self.assertEqual(["tools/check_static_docs_site_ui.py"], report.legacy_site_script_paths)
        self.assertEqual(["LTD_DISPLAY_CPU3/docs/README.md"], report.legacy_doc_paths)
        self.assertEqual(["tmp/pdfs/staged-page.png"], report.staged_local_output_removals)
        self.assertEqual(["tools/build_static_docs_site.py"], report.staged_legacy_site_script_removals)
        self.assertEqual(["LTD_MAIN_CPU2/docs/README.md"], report.staged_legacy_doc_removals)
        self.assertEqual(["tools/check_docs_staging.py"], report.untracked_tool_paths)
        self.assertEqual(
            [
                "docs/00_构建与版本/文档库治理清单.md",
                "docs/01_协议与寄存器/README.md",
                "tools/build_static_docs_site.py",
                "tools/check_static_docs_site_ui.py",
            ],
            report.docs_migration_paths,
        )
        self.assertEqual(["tools/check_docs_staging.py"], report.docs_check_tool_paths)
        self.assertEqual(
            [
                "LTD_MAIN_CPU2/Application/Src/app_main.c",
                "tools/check_motor_motion_target_plan.py",
            ],
            report.firmware_or_build_paths,
        )

    def test_report_is_non_blocking_when_only_untracked_tool_exists(self) -> None:
        module = load_module()

        report = module.build_staging_report([("?", "?", "tools/check_docs_staging.py")])

        self.assertFalse(report.has_blocking_findings())
        self.assertEqual(["tools/check_docs_staging.py"], report.untracked_tool_paths)

    def test_migration_add_excludes_already_staged_docs_deletions(self) -> None:
        module = load_module()

        report = module.build_staging_report(
            [
                ("D", " ", "docs/01_协议与寄存器/SI协议适配/old.pdf"),
                (" ", "D", "docs/01_协议与寄存器/SI协议适配/unstaged.pdf"),
                ("M", " ", "docs/README.md"),
                ("?", "?", "tools/check_docs.py"),
            ]
        )

        self.assertEqual(
            [
                "docs/01_协议与寄存器/SI协议适配/old.pdf",
                "docs/01_协议与寄存器/SI协议适配/unstaged.pdf",
                "docs/README.md",
                "tools/check_docs.py",
            ],
            module.paths_for_list_category(report, "migration-submit"),
        )
        self.assertEqual(
            [
                "docs/01_协议与寄存器/SI协议适配/unstaged.pdf",
                "docs/README.md",
                "tools/check_docs.py",
            ],
            module.paths_for_list_category(report, "migration-add"),
        )

    def test_build_mixed_path_details_combines_staged_and_unstaged_numstat(self) -> None:
        module = load_module()

        details = module.build_mixed_path_details(
            ["docs/README.md", "docs/00_构建与版本/README.md"],
            staged_numstat={
                "docs/README.md": module.Numstat(added=10, deleted=2),
                "docs/00_构建与版本/README.md": module.Numstat(added=None, deleted=None),
            },
            unstaged_numstat={
                "docs/README.md": module.Numstat(added=3, deleted=1),
            },
        )

        self.assertEqual(
            [
                module.MixedPathDetail(
                    path="docs/00_构建与版本/README.md",
                    staged=module.Numstat(added=None, deleted=None),
                    unstaged=module.Numstat(added=0, deleted=0),
                ),
                module.MixedPathDetail(
                    path="docs/README.md",
                    staged=module.Numstat(added=10, deleted=2),
                    unstaged=module.Numstat(added=3, deleted=1),
                ),
            ],
            details,
        )

    def test_print_mixed_path_details_includes_staged_and_unstaged_counts(self) -> None:
        module = load_module()
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_path_details(
                [
                    module.MixedPathDetail(
                        path="docs/README.md",
                        staged=module.Numstat(added=10, deleted=2),
                        unstaged=module.Numstat(added=3, deleted=1),
                    )
                ]
            )

        output = buffer.getvalue()
        self.assertIn("混合文件改动规模", output)
        self.assertIn("docs/README.md", output)
        self.assertIn("已暂存 +10/-2", output)
        self.assertIn("未暂存 +3/-1", output)

    def test_print_mixed_details_report_outputs_focused_diagnostic(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_details_report(
                report,
                staged_numstat={"docs/README.md": module.Numstat(added=10, deleted=2)},
                unstaged_numstat={"docs/README.md": module.Numstat(added=3, deleted=1)},
            )

        output = buffer.getvalue()
        self.assertIn("文档迁移混合文件诊断", output)
        self.assertIn("AM/MM 混合文件：1", output)
        self.assertIn("docs/README.md", output)
        self.assertIn("已暂存 +10/-2", output)
        self.assertIn("未暂存 +3/-1", output)

    def test_print_mixed_details_report_handles_clean_state(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_details_report(report, staged_numstat={}, unstaged_numstat={})

        output = buffer.getvalue()
        self.assertIn("文档迁移混合文件诊断", output)
        self.assertIn("AM/MM 混合文件：0", output)
        self.assertIn("未发现 AM/MM 混合文件。", output)

    def test_print_mixed_matrix_outputs_category_counts_stats_and_actions(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_matrix(
                report,
                staged_numstat={
                    "docs/README.md": module.Numstat(added=10, deleted=2),
                    firmware_doc: module.Numstat(added=1370, deleted=0),
                },
                unstaged_numstat={
                    "docs/README.md": module.Numstat(added=3, deleted=1),
                    firmware_doc: module.Numstat(added=2, deleted=1),
                },
            )

        output = buffer.getvalue()
        self.assertIn("AM/MM 混合文件收口矩阵", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("混合文件总数：2", output)
        self.assertIn("治理文档：1", output)
        self.assertIn("固件相关文档：1", output)
        self.assertIn("docs/README.md", output)
        self.assertIn("分类：治理文档", output)
        self.assertIn("建议：随文档迁移统一核对后暂存", output)
        self.assertIn("已暂存 +10/-2，未暂存 +3/-1", output)
        self.assertIn(firmware_doc, output)
        self.assertIn("分类：固件相关文档", output)
        self.assertIn("建议：跟随对应固件改动单独判断", output)
        self.assertIn("已暂存 +1370/-0，未暂存 +2/-1", output)
        self.assertIn("--mixed-details", output)
        self.assertIn("--mixed-commands", output)

    def test_build_mixed_resolution_steps_outputs_grouped_templates(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        commands = module.build_mixed_resolution_steps(report)

        self.assertEqual(
            [
                "py tools\\check_docs_staging.py --mixed-resolution-check",
                "py tools\\check_docs_staging.py --mixed-boundary-check",
                "py tools\\check_docs_staging.py --submit-boundary-check",
                "py tools\\check_docs_staging.py --mixed-matrix",
                "py tools\\check_docs_staging.py --mixed-commands",
                "py tools\\check_docs_staging.py --list mixed-docs-governance",
                "git diff --cached -- 'docs/README.md'",
                "git diff -- 'docs/README.md'",
                "git add -- 'docs/README.md'",
                "py tools\\check_docs_staging.py --list mixed-firmware-docs",
                f"git diff --cached -- '{firmware_doc}'",
                f"git diff -- '{firmware_doc}'",
                "# 固件相关混合文档不在纯文档迁移模板中自动 git add，需随固件改动单独判断。",
                "py tools\\check_docs_staging.py --summary",
            ],
            commands,
        )

    def test_print_mixed_resolution_plan_outputs_copyable_grouped_templates(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_resolution_plan(report)

        output = buffer.getvalue()
        self.assertIn("AM/MM 混合文件分批收口模板", output)
        self.assertIn("本命令只打印模板，不执行 git add", output)
        self.assertIn("治理文档：1", output)
        self.assertIn("固件相关文档：1", output)
        self.assertIn("治理文档批次", output)
        self.assertIn("固件相关文档批次", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-check", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-boundary-check", output)
        self.assertIn("git add -- 'docs/README.md'", output)
        self.assertIn(f"git diff --cached -- '{firmware_doc}'", output)
        self.assertIn(f"git diff -- '{firmware_doc}'", output)
        self.assertNotIn(f"git add -- '{firmware_doc}'", output)
        self.assertIn("固件相关混合文档不在纯文档迁移模板中自动 git add", output)
        self.assertIn("py tools\\check_docs_staging.py --summary", output)

    def test_build_mixed_resolution_coverage_detects_complete_grouping(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        coverage = module.build_mixed_resolution_coverage(report)

        self.assertTrue(coverage.is_complete())
        self.assertEqual(["docs/README.md"], coverage.governance_paths)
        self.assertEqual([firmware_doc], coverage.firmware_paths)
        self.assertEqual(["docs/README.md", firmware_doc], coverage.grouped_paths)
        self.assertEqual([], coverage.missing_paths)
        self.assertEqual([], coverage.extra_paths)
        self.assertEqual([], coverage.duplicated_paths)

    def test_print_mixed_resolution_check_outputs_complete_coverage(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_resolution_check(report)

        output = buffer.getvalue()
        self.assertIn("AM/MM 混合文件分批模板覆盖率预检", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("混合文件总数：2", output)
        self.assertIn("模板覆盖文件：2", output)
        self.assertIn("治理文档：1", output)
        self.assertIn("固件相关文档：1", output)
        self.assertIn("遗漏：0", output)
        self.assertIn("额外：0", output)
        self.assertIn("重复：0", output)
        self.assertIn("覆盖完整", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-plan", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-boundary-check", output)
        self.assertIn("治理文档可按模板暂存，固件相关文档只核对 diff", output)

    def test_build_mixed_submit_boundary_detects_clean_submit_boundary(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        boundary = module.build_mixed_submit_boundary(report)

        self.assertTrue(boundary.is_clean())
        self.assertEqual(["docs/README.md"], boundary.governance_paths)
        self.assertEqual([firmware_doc], boundary.firmware_paths)
        self.assertEqual([], boundary.governance_missing_from_submit)
        self.assertEqual([], boundary.firmware_present_in_submit)

    def test_print_mixed_boundary_check_outputs_submit_boundary_counts(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_boundary_check(report)

        output = buffer.getvalue()
        self.assertIn("AM/MM 混合文件提交边界检查", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("混合文件总数：2", output)
        self.assertIn("治理文档：1", output)
        self.assertIn("固件相关文档：1", output)
        self.assertIn("治理文档未进入迁移候选：0", output)
        self.assertIn("固件相关文档误入迁移候选：0", output)
        self.assertIn("边界结论：清晰", output)
        self.assertIn("治理文档可随迁移收口，固件相关文档已从纯文档迁移候选排除", output)

    def test_print_mixed_closure_audit_outputs_contract_status_and_next_commands(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        version_doc = "docs/00_构建与版本/版本改动与测试/README.md"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
            version_output_paths=[version_doc],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_closure_audit(report)

        output = buffer.getvalue()
        self.assertIn("AM/MM 混合文件收口审计", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("混合文件总数：2", output)
        self.assertIn("治理文档：1", output)
        self.assertIn("固件相关文档：1", output)
        self.assertIn("覆盖率：完整", output)
        self.assertIn("提交边界：清晰", output)
        self.assertIn("版本输出契约：清晰", output)
        self.assertIn("当前是否可进入混合文件收口完成态：否", output)
        self.assertIn("后续版本文档仍可由 skill 写入", output)
        self.assertIn("py tools\\check_docs_staging.py --version-output-contract", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-plan", output)
        self.assertIn("py tools\\check_docs_staging.py --remaining-checklist", output)

    def test_build_submit_boundary_check_detects_clean_candidate_scope(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=["tmp/generated.html"],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_DISPLAY_CPU3/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=[],
            docs_migration_paths=["AGENTS.md", "docs/README.md", firmware_doc],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=["README.tmp"],
            release_record_paths=["CHANGELOG.md"],
        )

        boundary = module.build_submit_boundary_check(report)

        self.assertTrue(boundary.is_clean())
        self.assertEqual(
            [
                "AGENTS.md",
                "LTD_DISPLAY_CPU3/docs/README.md",
                "docs/README.md",
                "tools/build_static_docs_site.py",
                "tools/check_docs.py",
            ],
            boundary.submit_paths,
        )
        self.assertEqual(["AGENTS.md", "docs/README.md"], boundary.docs_migration_paths)
        self.assertEqual(["tools/check_docs.py"], boundary.docs_check_tool_paths)
        self.assertEqual(
            ["LTD_DISPLAY_CPU3/docs/README.md", "tools/build_static_docs_site.py"],
            boundary.migration_cleanup_paths,
        )
        self.assertEqual([firmware_doc], boundary.excluded_mixed_firmware_paths)
        self.assertEqual([], boundary.release_record_leaks)
        self.assertEqual([], boundary.version_output_leaks)
        self.assertEqual([], boundary.local_output_leaks)

    def test_submit_boundary_check_flags_version_output_leaks(self) -> None:
        module = load_module()
        version_doc = "docs/00_构建与版本/版本改动与测试/README.md"
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[version_doc],
            firmware_or_build_paths=[],
            other_paths=[],
            version_output_paths=[version_doc],
        )

        boundary = module.build_submit_boundary_check(report)

        self.assertFalse(boundary.is_clean())
        self.assertEqual([version_doc], boundary.version_output_leaks)

    def test_print_submit_boundary_check_outputs_candidate_counts(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=["tmp/generated.html"],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_DISPLAY_CPU3/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=[],
            docs_migration_paths=["AGENTS.md", "docs/README.md", firmware_doc],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=["README.tmp"],
            release_record_paths=["CHANGELOG.md"],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_submit_boundary_check(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移候选边界检查", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("迁移候选总数：5", output)
        self.assertIn("文档迁移主体：2", output)
        self.assertIn("文档检查工具：1", output)
        self.assertIn("旧 CPU docs/旧静态站脚本删除记录：2", output)
        self.assertIn("已排除固件相关混合文档：1", output)
        self.assertIn("错误混入固件/构建：0", output)
        self.assertIn("错误混入版本/发布记录：0", output)
        self.assertIn("错误混入后续版本文档输出区：0", output)
        self.assertIn("错误混入本地输出：0", output)
        self.assertIn("边界结论：清晰", output)

    def test_build_remaining_work_items_groups_blockers_and_decisions(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=["LTD_MAIN_CPU2/docs/README.md"],
            local_output_paths=["tmp/generated.html"],
            legacy_site_script_paths=["tools/ltd-report-theme.css"],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=["notes.txt"],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
        )

        items = module.build_remaining_work_items(report)

        self.assertEqual(
            [
                "治理文档混合文件",
                "固件相关混合文档",
                "旧 CPU docs 非纯删除状态",
                "本地输出非纯删除状态",
                "旧静态站脚本非纯删除状态",
                "未跟踪文档检查工具",
                "固件/构建相关路径",
                "版本/发布记录路径",
                "后续版本文档输出区路径",
                "其他未归类路径",
                "本地输出清理记录",
            ],
            [item.title for item in items],
        )
        self.assertEqual(
            [True, True, True, True, True, False, False, False, False, False, False],
            [item.blocking for item in items],
        )
        self.assertIn("--version-output-contract", items[8].action)

    def test_print_remaining_checklist_outputs_actions_and_boundaries(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=[],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_remaining_checklist(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移剩余收口清单", output)
        self.assertIn("候选边界：清晰", output)
        self.assertIn("阻塞收口分组：2", output)
        self.assertIn("仍需人工决策分组：4", output)
        self.assertIn("治理文档混合文件：1 项", output)
        self.assertIn("固件相关混合文档：1 项", output)
        self.assertIn("未跟踪文档检查工具：1 项", output)
        self.assertIn("固件/构建相关路径：1 项", output)
        self.assertIn("版本/发布记录路径：1 项", output)
        self.assertIn("后续版本文档输出区路径：1 项", output)
        self.assertIn("动作：核对 cached/worktree diff 后随文档迁移暂存。", output)
        self.assertIn("动作：按固件改动另行判断版本、构建和流程图影响。", output)
        self.assertIn("动作：运行 `--version-output-contract` 核对边界", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-closure-audit", output)
        self.assertIn("py tools\\check_docs_staging.py --version-output-contract", output)

    def test_blocking_category_count_splits_governance_and_firmware_mixed_docs(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        summary_buffer = io.StringIO()
        final_buffer = io.StringIO()

        with redirect_stdout(summary_buffer):
            module.print_summary(report)
        with redirect_stdout(final_buffer):
            module.print_final_check(report)

        self.assertEqual(2, module.count_blocking_categories(report))
        self.assertIn("阻塞项：2 项", summary_buffer.getvalue())
        self.assertIn("阻塞项：2 项", final_buffer.getvalue())

    def test_build_closure_plan_steps_orders_final_route_and_preserves_version_output_boundary(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        version_doc = "docs/00_构建与版本/版本改动与测试/README.md"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=[version_doc],
            other_paths=["notes.txt"],
        )

        steps = module.build_closure_plan_steps(report)

        self.assertEqual(
            [
                "混合文件分批收口（治理文档 1 个，固件相关文档 1 个）",
                "迁移候选边界确认（候选 4 个）",
                "后续版本文档输出契约确认（1 个）",
                "非迁移路径人工分离（4 个）",
                "本地审计快照生成",
                "迁移暂存模板生成",
                "文档与本地站点验证",
                "最终收口复核",
            ],
            [step.title for step in steps],
        )
        self.assertIn("--mixed-resolution-plan", steps[0].command)
        self.assertIn("--mixed-closure-audit", steps[0].command)
        self.assertIn("--mixed-resolution-check", steps[0].pass_criteria)
        self.assertIn("--mixed-boundary-check", steps[0].pass_criteria)
        self.assertIn("--submit-boundary-check", steps[1].command)
        self.assertIn("后续版本文档输出区", steps[1].pass_criteria)
        self.assertIn("--version-output-contract", steps[2].command)
        self.assertIn("不进入 migration-submit", steps[2].pass_criteria)
        self.assertIn("不修改 C:\\Users\\admin\\.codex\\skills", steps[2].pass_criteria)
        self.assertIn("--submit-plan", steps[3].command)
        self.assertIn("version-outputs", steps[3].pass_criteria)
        self.assertIn("不作为本次纯文档迁移前置条件", steps[3].pass_criteria)
        self.assertIn("--write-local-audit", steps[4].command)
        self.assertIn("tmp/docs-migration-audit.md", steps[4].pass_criteria)
        self.assertIn("--stage-commands", steps[5].command)
        self.assertIn("不执行 git add", steps[5].pass_criteria)
        self.assertIn("py -B tools\\check_docs.py", steps[6].command)
        self.assertIn("npm run check", steps[6].command)
        self.assertIn("--final-check", steps[7].command)

    def test_print_closure_plan_outputs_read_only_route_and_skill_boundary(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=[],
            release_record_paths=[],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_closure_plan(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移收口路线", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("不执行 git add", output)
        self.assertIn("不修改暂存区", output)
        self.assertIn("只服务本次 docs/ 迁移收口", output)
        self.assertIn("后续版本文档输出区", output)
        self.assertIn("不阻塞后续 skill 生成新版本文档", output)
        self.assertIn("迁移提交候选：2", output)
        self.assertIn("必须先收口：1 组", output)
        self.assertIn("必须单独判断：1", output)
        self.assertIn("执行路线：", output)
        self.assertIn("通过口径：", output)
        self.assertIn("py tools\\check_docs_staging.py --stage-commands", output)
        self.assertIn("py tools\\check_docs_staging.py --write-local-audit", output)
        self.assertIn("docs-site", output)
        self.assertNotIn("- docs/README.md", output)

    def test_quote_path_for_powershell_single_quotes_special_paths(self) -> None:
        module = load_module()

        self.assertEqual("'docs/README.md'", module.quote_path_for_powershell("docs/README.md"))
        self.assertEqual("'docs/O''Brien.md'", module.quote_path_for_powershell("docs/O'Brien.md"))

    def test_build_mixed_diff_commands_outputs_cached_and_worktree_checks(self) -> None:
        module = load_module()

        commands = module.build_mixed_diff_commands(["docs/README.md"])

        self.assertEqual(
            [
                "git diff --cached -- 'docs/README.md'",
                "git diff -- 'docs/README.md'",
            ],
            commands,
        )

    def test_print_mixed_commands_outputs_only_templates(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_commands(report)

        output = buffer.getvalue()
        self.assertIn("AM/MM 混合文件 diff 命令模板", output)
        self.assertIn("本命令只打印模板，不执行 git diff", output)
        self.assertIn("混合文件：1", output)
        self.assertIn("git diff --cached -- 'docs/README.md'", output)
        self.assertIn("git diff -- 'docs/README.md'", output)

    def test_print_mixed_commands_handles_clean_state(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_commands(report)

        output = buffer.getvalue()
        self.assertIn("混合文件：0", output)
        self.assertIn("无需生成 mixed diff 命令。", output)

    def test_print_mixed_plan_groups_governance_and_firmware_docs(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_mixed_plan(report)

        output = buffer.getvalue()
        self.assertIn("AM/MM 混合文件收口计划", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("混合文件总数：2", output)
        self.assertIn("治理文档：1", output)
        self.assertIn("固件相关文档：1", output)
        self.assertIn("治理文档可随文档迁移统一核对后暂存", output)
        self.assertIn("固件相关文档需跟随对应固件改动单独判断", output)
        self.assertIn("docs/README.md", output)
        self.assertIn(firmware_doc, output)
        self.assertIn("--mixed-details", output)
        self.assertIn("--list mixed-docs-governance", output)
        self.assertIn("--list mixed-firmware-docs", output)

    def test_build_staging_plan_steps_orders_manual_submit_workflow(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=["CHANGELOG.md"],
        )

        steps = module.build_staging_plan_steps(report)

        self.assertEqual(
            [
                "运行 `py tools\\check_docs_staging.py --mixed-plan`，先区分治理文档和固件相关文档。",
                "运行 `py tools\\check_docs_staging.py --mixed-matrix`，逐项核对分类、已暂存/未暂存规模和建议处理动作。",
                "运行 `py tools\\check_docs_staging.py --mixed-resolution-check`，确认分批模板覆盖全部 AM/MM 混合文件。",
                "运行 `py tools\\check_docs_staging.py --mixed-boundary-check`，确认治理文档进入迁移候选且固件相关文档被排除。",
                "运行 `py tools\\check_docs_staging.py --submit-boundary-check`，确认迁移候选未混入固件、版本记录、后续版本文档输出区、本地输出或其他路径。",
                "运行 `py tools\\check_docs_staging.py --mixed-closure-audit`，汇总混合文件覆盖率、提交边界和版本输出契约状态。",
                "运行 `py tools\\check_docs_staging.py --version-output-contract`，确认后续版本文档输出区不进入本次迁移候选且不影响 skill 继续生成新方案。",
                "运行 `py tools\\check_docs_staging.py --mixed-resolution-plan`，生成治理文档和固件相关文档的分批核对模板。",
                "运行 `py tools\\check_docs_staging.py --mixed-details`，逐个核对 1 个 AM/MM 混合文件。",
                "用 `py tools\\check_docs_staging.py --gate-matrix` 先确认必须收口、可随迁移和必须单独判断的门禁项。",
                "用 `py tools\\check_docs_staging.py --list migration-submit` 导出 4 个文档迁移候选路径。",
                "用 `py tools\\check_docs_staging.py --submit-plan` 复核迁移、固件、版本记录、后续版本文档输出和本地输出拆分。",
                "用 `py tools\\check_docs_staging.py --list firmware-or-build` 单独核对 1 个固件或构建相关路径。",
                "用 `py tools\\check_docs_staging.py --list version-outputs` 单独核对 1 个后续版本文档输出区路径；它不进入 migration-submit，也不阻塞后续 skill 生成新版本文档。",
                "用 `py tools\\check_docs_staging.py --list other` 单独核对 1 个其他路径。",
                "确认 1 个本地输出删除记录是否单独清理，不把 docs-site/site/tmp 当作文档源。",
                "最终暂存已确认的迁移目标范围后运行 `py tools\\check_docs_staging.py --summary` 和完整检查；已知固件相关混合文档不随纯文档迁移模板自动纳入。",
            ],
            steps,
        )

    def test_print_staging_plan_outputs_counts_and_commands(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[
                "docs/README.md",
                "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html",
            ],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=[],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_staging_plan(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移提交手动收口计划", output)
        self.assertIn("迁移提交候选：3", output)
        self.assertIn("需要单独判断：1", output)
        self.assertIn("AM/MM 混合文件：2", output)
        self.assertIn("混合治理文档：1", output)
        self.assertIn("混合固件文档：1", output)
        self.assertIn("--gate-matrix", output)
        self.assertIn("--closure-plan", output)
        self.assertIn("--list migration-submit", output)
        self.assertIn("--list version-outputs", output)
        self.assertIn("--submit-plan", output)
        self.assertIn("--mixed-plan", output)
        self.assertIn("- py tools\\check_docs_staging.py --mixed-matrix", output)
        self.assertIn("- py tools\\check_docs_staging.py --mixed-resolution-check", output)
        self.assertIn("- py tools\\check_docs_staging.py --mixed-boundary-check", output)
        self.assertIn("- py tools\\check_docs_staging.py --mixed-closure-audit", output)
        self.assertIn("- py tools\\check_docs_staging.py --version-output-contract", output)
        self.assertIn("- py tools\\check_docs_staging.py --mixed-resolution-plan", output)
        self.assertIn("--mixed-details", output)
        self.assertIn("--list mixed-docs-governance", output)
        self.assertIn("--list mixed-firmware-docs", output)
        self.assertIn("不执行 git add", output)
        self.assertNotIn("- docs/README.md", output)

    def test_build_stage_command_steps_outputs_copyable_read_only_templates(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=[],
        )

        commands = module.build_stage_command_steps(report)

        self.assertEqual(
            [
                "py tools\\check_docs_staging.py --mixed-plan",
                "py tools\\check_docs_staging.py --mixed-matrix",
                "py tools\\check_docs_staging.py --mixed-resolution-check",
                "py tools\\check_docs_staging.py --mixed-boundary-check",
                "py tools\\check_docs_staging.py --submit-boundary-check",
                "py tools\\check_docs_staging.py --mixed-closure-audit",
                "py tools\\check_docs_staging.py --version-output-contract",
                "py tools\\check_docs_staging.py --mixed-resolution-plan",
                "py tools\\check_docs_staging.py --mixed-details",
                "py tools\\check_docs_staging.py --remaining-checklist",
                "py tools\\check_docs_staging.py --write-local-audit",
                "py tools\\check_docs_staging.py --list firmware-or-build",
                "py tools\\check_docs_staging.py --list release-records",
                "py tools\\check_docs_staging.py --list version-outputs",
                "New-Item -ItemType Directory -Force tmp | Out-Null",
                "py tools\\check_docs_staging.py --list migration-add > tmp\\docs-migration-add.paths",
                "git add --pathspec-from-file=tmp\\docs-migration-add.paths",
                "py tools\\check_docs_staging.py --summary",
            ],
            commands,
        )

    def test_build_stage_command_steps_omits_mixed_diagnostic_when_clean(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            release_record_paths=[],
            version_output_paths=[],
            other_paths=[],
        )

        commands = module.build_stage_command_steps(report)

        self.assertEqual(
            [
                "py tools\\check_docs_staging.py --remaining-checklist",
                "py tools\\check_docs_staging.py --write-local-audit",
                "New-Item -ItemType Directory -Force tmp | Out-Null",
                "py tools\\check_docs_staging.py --list migration-add > tmp\\docs-migration-add.paths",
                "git add --pathspec-from-file=tmp\\docs-migration-add.paths",
                "py tools\\check_docs_staging.py --summary",
            ],
            commands,
        )

    def test_print_stage_commands_outputs_warning_and_templates(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=[],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_stage_commands(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移暂存命令模板", output)
        self.assertIn("本命令只打印模板，不执行 git add", output)
        self.assertIn("迁移提交候选：2", output)
        self.assertIn("py tools\\check_docs_staging.py --list migration-submit", output)
        self.assertIn("py tools\\check_docs_staging.py --list migration-add", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-plan", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-matrix", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-check", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-boundary-check", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-closure-audit", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-plan", output)
        self.assertIn("py tools\\check_docs_staging.py --remaining-checklist", output)
        self.assertIn("py tools\\check_docs_staging.py --write-local-audit", output)
        self.assertIn("py tools\\check_docs_staging.py --list release-records", output)
        self.assertIn("py tools\\check_docs_staging.py --list version-outputs", output)
        self.assertIn("git add --pathspec-from-file=tmp\\docs-migration-add.paths", output)
        self.assertIn("先收口 1 个 AM/MM 混合文件", output)
        self.assertIn("固件相关混合文档需分离判断", output)

    def test_print_decision_checklist_outputs_compact_submit_gates(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[
                "docs/README.md",
                "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html",
            ],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=["CHANGELOG.md"],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_decision_checklist(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移提交决策清单", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("先收口 AM/MM 混合文件：2 个（当前阻塞分组 2 组）", output)
        self.assertIn("治理文档 1 个可随迁移，固件相关文档 1 个需分离判断", output)
        self.assertIn("迁移提交候选：4 个", output)
        self.assertIn("混合治理文档：1 个", output)
        self.assertIn("混合固件文档：1 个", output)
        self.assertIn("需要单独判断：3 个", output)
        self.assertIn("本地输出清理记录：1 个", output)
        self.assertIn("不会阻塞后续 skill 生成版本文档", output)
        self.assertIn("后续版本文档输出区", output)
        self.assertIn("py -B tools\\check_docs_staging.py --final-check", output)
        self.assertIn("py -B tools\\check_docs_staging.py --closure-plan", output)
        self.assertIn("py -B tools\\check_docs_staging.py --gate-matrix", output)
        self.assertIn("py -B tools\\check_docs_staging.py --mixed-resolution-check", output)
        self.assertIn("py -B tools\\check_docs_staging.py --mixed-boundary-check", output)
        self.assertIn("py -B tools\\check_docs_staging.py --mixed-closure-audit", output)
        self.assertIn("py -B tools\\check_docs_staging.py --version-output-contract", output)
        self.assertIn("py -B tools\\check_docs_staging.py --mixed-resolution-plan", output)
        self.assertIn("py -B tools\\check_docs_staging.py --write-local-audit", output)
        self.assertIn("--list mixed-docs-governance", output)
        self.assertIn("--list mixed-firmware-docs", output)
        self.assertIn("py -B tools\\check_docs.py", output)
        self.assertIn("npm run check", output)
        self.assertNotIn("- docs/README.md", output)

    def test_print_gate_matrix_outputs_submit_gates_and_generation_boundary(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        version_doc = "docs/00_构建与版本/版本改动与测试/2026-06-27_CPU2_V1_测试方案.md"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=["LTD_DISPLAY_CPU3/docs/README.md"],
            local_output_paths=["docs-site/public/page.html"],
            legacy_site_script_paths=["tools/ltd-report-theme.css"],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=["notes.txt"],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=[version_doc],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_gate_matrix(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移提交门禁矩阵", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("必须先收口：2 个 AM/MM 混合文件", output)
        self.assertIn("旧 CPU docs 非纯删除状态 1 个", output)
        self.assertIn("可随文档迁移提交：4 个迁移候选", output)
        self.assertIn("本地输出非纯删除状态 1 个", output)
        self.assertIn("必须单独判断：4 个", output)
        self.assertIn("固件/构建 1 个", output)
        self.assertIn("版本/发布记录 1 个", output)
        self.assertIn("后续版本文档输出 1 个", output)
        self.assertIn("其他 1 个", output)
        self.assertIn("不作为正式文档源：2 个", output)
        self.assertIn("后续版本文档输出区：1 个当前改动", output)
        self.assertIn("docs/00_构建与版本/版本改动与测试/", output)
        self.assertIn("人工索引允许后补", output)
        self.assertIn("不需要先运行本迁移门禁", output)
        self.assertIn("不影响版本提交流程生成新文档", output)
        self.assertIn("--mixed-plan", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-matrix", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-check", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-boundary-check", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-plan", output)
        self.assertIn("--list migration-submit", output)
        self.assertNotIn("- docs/README.md", output)

    def test_build_submit_scope_recommendations_groups_paths_by_action(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/site/page.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=["CHANGELOG.md"],
        )

        recommendations = module.build_submit_scope_recommendations(report)

        self.assertEqual(
            [
                module.SubmitScopeRecommendation(
                    title="建议纳入文档迁移提交",
                    paths=["docs/README.md", "tools/check_docs.py"],
                ),
                module.SubmitScopeRecommendation(
                    title="建议随迁移提交保留删除记录",
                    paths=["LTD_MAIN_CPU2/docs/README.md", "tools/build_static_docs_site.py"],
                ),
                module.SubmitScopeRecommendation(
                    title="需要单独判断是否属于本次提交",
                    paths=[
                        "CHANGELOG.md",
                        "LTD_MAIN_CPU2/Application/Src/app_main.c",
                        "docs/00_构建与版本/版本改动与测试/README.md",
                    ],
                ),
                module.SubmitScopeRecommendation(
                    title="本地输出清理记录，提交前确认是否单独处理",
                    paths=["tmp/site/page.html"],
                ),
            ],
            recommendations,
        )

    def test_print_submit_scope_recommendations_outputs_action_titles(self) -> None:
        module = load_module()
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_submit_scope_recommendations(
                [
                    module.SubmitScopeRecommendation(
                        title="建议纳入文档迁移提交",
                        paths=["docs/README.md"],
                    )
                ]
            )

        output = buffer.getvalue()
        self.assertIn("提交范围建议", output)
        self.assertIn("建议纳入文档迁移提交", output)
        self.assertIn("docs/README.md", output)

    def test_print_submit_plan_outputs_split_batches_and_list_commands(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=[],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_submit_plan(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移提交拆分计划", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("建议拆分批次", output)
        self.assertIn("1. 文档迁移提交候选：4 个", output)
        self.assertIn("2. 固件或构建相关：1 个", output)
        self.assertIn("3. 版本或发布记录：1 个", output)
        self.assertIn("4. 后续版本文档输出区：1 个", output)
        self.assertIn("5. 本地输出清理记录：1 个", output)
        self.assertIn("6. 混合文件收口：治理文档 1 个，固件相关文档 1 个", output)
        self.assertIn("--list migration-submit", output)
        self.assertIn("--list firmware-or-build", output)
        self.assertIn("--list release-records", output)
        self.assertIn("--list version-outputs", output)
        self.assertIn("--list local-output-removals", output)
        self.assertIn("--mixed-plan", output)
        self.assertIn("--mixed-matrix", output)
        self.assertIn("--mixed-resolution-check", output)
        self.assertIn("--mixed-boundary-check", output)
        self.assertIn("--mixed-resolution-plan", output)
        self.assertIn("--stage-commands", output)
        self.assertNotIn("- docs/README.md", output)

    def test_build_next_actions_prioritizes_blocking_work_before_scope_review(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[
                "docs/README.md",
                "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html",
            ],
            legacy_doc_paths=["LTD_MAIN_CPU2/docs/README.md"],
            local_output_paths=["tmp/page.html"],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=[],
        )

        actions = module.build_next_actions(report)

        self.assertEqual(
            [
                "先处理 2 个 AM/MM 混合文件（治理文档 1 个，固件相关文档 1 个）：治理文档用 `--mixed-resolution-plan` 核对后随迁移暂存；固件相关文档只核对 diff，随固件改动单独判断。",
                "处理 1 个旧 CPU docs 非纯删除状态：迁移后不应继续保留或修改旧目录。",
                "排除 1 个本地输出非纯删除状态：docs-site/site/tmp 不能进入正式文档提交。",
                "决定 1 个未跟踪工具脚本是否纳入本次文档检查能力提交。",
                "单独判断 1 个固件或构建相关路径是否属于本次提交，避免混入纯文档迁移。",
                "运行 `--version-output-contract` 核对 1 个后续版本文档输出区路径；它不属于本次文档迁移候选，也不是迁移门禁的阻塞项。",
                "确认 1 个本地输出删除记录是否单独清理；它不是后续版本文档生成的前置条件。",
            ],
            actions,
        )

    def test_print_next_actions_outputs_short_checklist(self) -> None:
        module = load_module()
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_next_actions(["先处理混合文件。", "再确认提交范围。"])

        output = buffer.getvalue()
        self.assertIn("建议处理顺序", output)
        self.assertIn("1. 先处理混合文件。", output)
        self.assertIn("2. 再确认提交范围。", output)

    def test_print_summary_outputs_counts_without_long_path_lists(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/page.html"],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=["CHANGELOG.md"],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_summary(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移提交前暂存区摘要", output)
        self.assertIn("阻塞项：1 项", output)
        self.assertIn("AM/MM 混合文件：1", output)
        self.assertIn("混合治理文档：1", output)
        self.assertIn("混合固件文档：0", output)
        self.assertIn("未跟踪工具脚本：1", output)
        self.assertIn("文档迁移相关路径：1", output)
        self.assertIn("迁移提交候选路径：3", output)
        self.assertIn("建议处理顺序", output)
        self.assertIn("治理文档用 `--mixed-resolution-plan` 核对后随迁移暂存", output)
        self.assertIn("固件相关文档只核对 diff", output)
        self.assertIn("运行 `--version-output-contract` 核对 1 个后续版本文档输出区路径", output)
        self.assertNotIn("- docs/README.md", output)

    def test_print_final_check_outputs_blocking_status_and_next_commands(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/page.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=[],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_final_check(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移最终收口状态", output)
        self.assertIn("本命令只读 Git 状态", output)
        self.assertIn("当前结论：尚不能进入最终暂存", output)
        self.assertIn("阻塞项：1 项", output)
        self.assertIn("AM/MM 混合文件：1", output)
        self.assertIn("未跟踪工具脚本：1", output)
        self.assertIn("迁移提交候选：4", output)
        self.assertIn("必须单独判断：3", output)
        self.assertIn("本地输出清理记录：1", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-check", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-boundary-check", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-closure-audit", output)
        self.assertIn("py tools\\check_docs_staging.py --version-output-contract", output)
        self.assertIn("py tools\\check_docs_staging.py --write-local-audit", output)
        self.assertIn("py tools\\check_docs_staging.py --mixed-resolution-plan", output)
        self.assertIn("py tools\\check_docs_staging.py --stage-commands", output)
        self.assertIn("py tools\\check_docs_staging.py --write-local-audit", output)
        self.assertIn("py tools\\check_docs_staging.py --summary", output)
        self.assertIn("不会阻塞后续 skill 生成版本文档", output)
        self.assertNotIn("- docs/README.md", output)

    def test_print_final_check_outputs_ready_status_when_no_blocking_findings(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_final_check(report)

        output = buffer.getvalue()
        self.assertIn("当前结论：可以进入最终暂存前人工确认", output)
        self.assertIn("阻塞项：0 项", output)
        self.assertIn("py tools\\check_docs_staging.py --stage-commands", output)
        self.assertIn("py tools\\check_docs_staging.py --write-local-audit", output)

    def test_build_local_audit_markdown_summarizes_scope_without_touching_skill_outputs(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        version_doc = "docs/00_构建与版本/版本改动与测试/README.md"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=["tools/check_docs_staging.py"],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=["tools/check_docs_staging.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=[],
            release_record_paths=["CHANGELOG.md"],
            version_output_paths=[version_doc],
        )

        markdown = module.build_local_audit_markdown(report)

        self.assertIn("# CUBE 文档迁移本地审计报告", markdown)
        self.assertIn("只写入 `tmp/docs-migration-audit.md`", markdown)
        self.assertIn("不执行 `git add`", markdown)
        self.assertIn("不写入 `docs/00_构建与版本/版本改动与测试/`", markdown)
        self.assertIn("不修改 `C:\\Users\\admin\\.codex\\skills`", markdown)
        self.assertIn("当前结论：尚不能进入最终暂存。", markdown)
        self.assertIn("AM/MM 混合文件：2", markdown)
        self.assertIn("治理文档：1", markdown)
        self.assertIn("固件相关文档：1", markdown)
        self.assertIn("迁移提交候选：3", markdown)
        self.assertIn("后续版本文档输出区：1", markdown)
        self.assertIn("## 阻塞收口项", markdown)
        self.assertIn("### 治理文档混合文件", markdown)
        self.assertIn("### 固件相关混合文档", markdown)
        self.assertIn("## 人工决策项", markdown)
        self.assertIn("### 后续版本文档输出区路径", markdown)
        self.assertIn("契约状态：清晰", markdown)
        self.assertIn("py tools\\check_docs_staging.py --version-output-contract", markdown)

    def test_write_local_audit_report_writes_utf8_snapshot_to_given_path(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "nested" / "audit.md"
            written_path = module.write_local_audit_report(report, output_path=output_path)

            self.assertEqual(output_path, written_path)
            self.assertTrue(output_path.exists())
            content = output_path.read_text(encoding="utf-8")

        self.assertIn("# CUBE 文档迁移本地审计报告", content)
        self.assertIn("当前结论：可以进入最终暂存前人工确认。", content)

    def test_print_local_audit_report_result_outputs_generated_path_and_current_status(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_local_audit_report_result(Path("tmp/docs-migration-audit.md"), report)

        output = buffer.getvalue()
        self.assertIn("本地审计报告已生成", output)
        self.assertIn("tmp/docs-migration-audit.md", output)
        self.assertIn("当前结论：尚不能进入最终暂存", output)
        self.assertIn("不会阻塞后续 skill 生成版本文档", output)

    def test_parse_args_detects_summary_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--summary"]).summary)
        self.assertFalse(module.parse_args([]).summary)

    def test_parse_args_detects_audit_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--audit"]).audit)
        self.assertFalse(module.parse_args([]).audit)

    def test_parse_args_detects_mixed_details_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--mixed-details"]).mixed_details)
        self.assertFalse(module.parse_args([]).mixed_details)

    def test_parse_args_detects_staging_plan_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--staging-plan"]).staging_plan)
        self.assertFalse(module.parse_args([]).staging_plan)

    def test_parse_args_detects_stage_commands_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--stage-commands"]).stage_commands)
        self.assertFalse(module.parse_args([]).stage_commands)

    def test_parse_args_detects_mixed_commands_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--mixed-commands"]).mixed_commands)
        self.assertFalse(module.parse_args([]).mixed_commands)

    def test_parse_args_detects_mixed_plan_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--mixed-plan"]).mixed_plan)
        self.assertFalse(module.parse_args([]).mixed_plan)

    def test_parse_args_detects_mixed_matrix_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--mixed-matrix"]).mixed_matrix)
        self.assertFalse(module.parse_args([]).mixed_matrix)

    def test_parse_args_detects_mixed_resolution_plan_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--mixed-resolution-plan"]).mixed_resolution_plan)
        self.assertFalse(module.parse_args([]).mixed_resolution_plan)

    def test_parse_args_detects_mixed_resolution_check_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--mixed-resolution-check"]).mixed_resolution_check)
        self.assertFalse(module.parse_args([]).mixed_resolution_check)

    def test_parse_args_detects_mixed_boundary_check_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--mixed-boundary-check"]).mixed_boundary_check)
        self.assertFalse(module.parse_args([]).mixed_boundary_check)

    def test_parse_args_detects_mixed_closure_audit_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--mixed-closure-audit"]).mixed_closure_audit)
        self.assertFalse(module.parse_args([]).mixed_closure_audit)

    def test_parse_args_detects_submit_boundary_check_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--submit-boundary-check"]).submit_boundary_check)
        self.assertFalse(module.parse_args([]).submit_boundary_check)

    def test_parse_args_detects_remaining_checklist_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--remaining-checklist"]).remaining_checklist)
        self.assertFalse(module.parse_args([]).remaining_checklist)

    def test_parse_args_detects_decision_checklist_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--decision-checklist"]).decision_checklist)
        self.assertFalse(module.parse_args([]).decision_checklist)

    def test_parse_args_detects_submit_plan_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--submit-plan"]).submit_plan)
        self.assertFalse(module.parse_args([]).submit_plan)

    def test_parse_args_detects_gate_matrix_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--gate-matrix"]).gate_matrix)
        self.assertFalse(module.parse_args([]).gate_matrix)

    def test_parse_args_detects_final_check_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--final-check"]).final_check)
        self.assertFalse(module.parse_args([]).final_check)

    def test_parse_args_detects_version_output_contract_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--version-output-contract"]).version_output_contract)
        self.assertFalse(module.parse_args([]).version_output_contract)

    def test_parse_args_detects_write_local_audit_mode(self) -> None:
        module = load_module()

        self.assertTrue(module.parse_args(["--write-local-audit"]).write_local_audit)
        self.assertFalse(module.parse_args([]).write_local_audit)

    def test_parse_args_detects_list_category(self) -> None:
        module = load_module()

        args = module.parse_args(["--list", "firmware-or-build"])

        self.assertEqual("firmware-or-build", args.list_category)

    def test_parse_args_accepts_mixed_submit_decision_categories(self) -> None:
        module = load_module()

        governance_args = module.parse_args(["--list", "mixed-docs-governance"])
        firmware_args = module.parse_args(["--list", "mixed-firmware-docs"])

        self.assertEqual("mixed-docs-governance", governance_args.list_category)
        self.assertEqual("mixed-firmware-docs", firmware_args.list_category)

    def test_parse_args_accepts_release_record_category(self) -> None:
        module = load_module()

        args = module.parse_args(["--list", "release-records"])

        self.assertEqual("release-records", args.list_category)

    def test_parse_args_accepts_migration_add_category(self) -> None:
        module = load_module()

        args = module.parse_args(["--list", "migration-add"])

        self.assertEqual("migration-add", args.list_category)

    def test_paths_for_list_category_returns_requested_group(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=["LTD_MAIN_CPU2/docs/README.md"],
            local_output_paths=["tmp/page.html"],
            legacy_site_script_paths=["tools/ltd-report-theme.css"],
            staged_legacy_doc_removals=["LTD_DISPLAY_CPU3/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=["notes.txt"],
            release_record_paths=["CHANGELOG.md", "CHANGELOG.pdf"],
        )

        self.assertEqual(["docs/README.md"], module.paths_for_list_category(report, "docs-migration"))
        self.assertEqual(["tools/check_docs.py"], module.paths_for_list_category(report, "docs-check-tools"))
        self.assertEqual(
            ["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            module.paths_for_list_category(report, "firmware-or-build"),
        )
        self.assertEqual(["CHANGELOG.md", "CHANGELOG.pdf"], module.paths_for_list_category(report, "release-records"))
        self.assertEqual(["notes.txt"], module.paths_for_list_category(report, "other"))
        self.assertEqual(
            ["LTD_DISPLAY_CPU3/docs/README.md", "tools/build_static_docs_site.py"],
            module.paths_for_list_category(report, "migration-removals"),
        )
        self.assertEqual(["tmp/old.html", "tmp/page.html"], module.paths_for_list_category(report, "local-output-removals"))
        self.assertEqual(["docs/README.md"], module.paths_for_list_category(report, "mixed"))
        self.assertEqual(
            ["docs/README.md", "tools/check_docs.py"],
            module.paths_for_list_category(report, "migration-add"),
        )

    def test_paths_for_list_category_splits_known_mixed_submit_decision_groups(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[
                "docs/README.md",
                "docs/00_构建与版本/文档库治理清单.md",
                "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html",
            ],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(
            [
                "docs/00_构建与版本/文档库治理清单.md",
                "docs/README.md",
            ],
            module.paths_for_list_category(report, "mixed-docs-governance"),
        )
        self.assertEqual(
            [
                "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html",
            ],
            module.paths_for_list_category(report, "mixed-firmware-docs"),
        )

    def test_migration_submit_list_contains_only_docs_scope_candidates(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=["LTD_MAIN_CPU2/docs/README.md"],
            local_output_paths=["tmp/page.html"],
            legacy_site_script_paths=["tools/ltd-report-theme.css"],
            staged_legacy_doc_removals=["LTD_DISPLAY_CPU3/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["AGENTS.md", "docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=["CHANGELOG.md"],
        )

        self.assertEqual(
            [
                "AGENTS.md",
                "LTD_DISPLAY_CPU3/docs/README.md",
                "docs/README.md",
                "tools/build_static_docs_site.py",
                "tools/check_docs.py",
            ],
            module.paths_for_list_category(report, "migration-submit"),
        )
        self.assertEqual(
            [
                "AGENTS.md",
                "docs/README.md",
                "tools/check_docs.py",
            ],
            module.paths_for_list_category(report, "migration-add"),
        )

    def test_migration_submit_list_excludes_firmware_related_mixed_docs(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(["docs/README.md"], module.paths_for_list_category(report, "migration-submit"))
        self.assertEqual([firmware_doc], module.paths_for_list_category(report, "mixed-firmware-docs"))

    def test_version_output_docs_are_excluded_from_migration_submit(self) -> None:
        module = load_module()
        version_doc = "docs/00_构建与版本/版本改动与测试/2026-06-28_CPU2_V1_测试方案.md"
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
            version_output_paths=[version_doc],
        )

        self.assertEqual(["docs/README.md"], module.paths_for_list_category(report, "migration-submit"))
        self.assertEqual([version_doc], module.paths_for_list_category(report, "version-outputs"))

    def test_version_output_contract_reports_appendable_skill_boundary(self) -> None:
        module = load_module()
        version_doc = "docs/00_构建与版本/版本改动与测试/2026-06-28_CPU2_V1_测试方案.md"
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
            version_output_paths=[version_doc],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_version_output_contract(report)

        output = buffer.getvalue()
        self.assertIn("后续版本文档生成契约检查", output)
        self.assertIn("当前版本输出区改动：1 个", output)
        self.assertIn("错误进入 migration-submit：0 个", output)
        self.assertIn("后续 skill 可继续新增该目录 Markdown", output)
        self.assertIn("不修改 C:\\Users\\admin\\.codex\\skills", output)
        self.assertEqual(0, module.exit_code_for_version_output_contract_mode(report))

    def test_version_output_contract_fails_when_version_output_leaks_into_submit(self) -> None:
        module = load_module()
        version_doc = "docs/00_构建与版本/版本改动与测试/2026-06-28_CPU2_V1_测试方案.md"
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[version_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
            version_output_paths=[version_doc],
        )

        contract = module.build_version_output_contract(report)

        self.assertEqual([version_doc], contract.version_output_leaks)
        self.assertFalse(contract.is_clean())
        self.assertEqual(1, module.exit_code_for_version_output_contract_mode(report))

    def test_print_path_list_outputs_plain_paths_only(self) -> None:
        module = load_module()
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_path_list(["docs/README.md", "tools/check_docs.py"])

        self.assertEqual("docs/README.md\ntools/check_docs.py\n", buffer.getvalue())

    def test_list_mode_returns_success_even_when_report_has_blocking_findings(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_list_mode(report))

    def test_print_scope_audit_outputs_grouped_counts_and_next_actions(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[
                "docs/README.md",
                "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html",
            ],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=["LTD_MAIN_CPU2/docs/README.md"],
            staged_local_output_removals=["tmp/old.html"],
            staged_legacy_site_script_removals=["tools/build_static_docs_site.py"],
            untracked_tool_paths=["tools/check_docs.py"],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            version_output_paths=["docs/00_构建与版本/版本改动与测试/README.md"],
            other_paths=["CHANGELOG.md"],
        )
        buffer = io.StringIO()

        with redirect_stdout(buffer):
            module.print_scope_audit(report)

        output = buffer.getvalue()
        self.assertIn("文档迁移提交范围审计", output)
        self.assertIn("建议纳入迁移提交：4", output)
        self.assertIn("需要单独判断：3", output)
        self.assertIn("不应纳入正式文档提交：1", output)
        self.assertIn("当前阻塞项：2", output)
        self.assertIn("AM/MM 混合文件：2", output)
        self.assertIn("混合治理文档：1", output)
        self.assertIn("混合固件文档：1", output)
        self.assertIn("--list mixed-docs-governance", output)
        self.assertIn("--list mixed-firmware-docs", output)
        self.assertIn("建议处理顺序", output)
        self.assertNotIn("- docs/README.md", output)

    def test_audit_mode_returns_blocking_exit_code(self) -> None:
        module = load_module()
        blocking_report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        clean_report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(1, module.exit_code_for_audit_mode(blocking_report))
        self.assertEqual(0, module.exit_code_for_audit_mode(clean_report))

    def test_mixed_details_mode_returns_blocking_exit_code(self) -> None:
        module = load_module()
        blocking_report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        clean_report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=["LTD_MAIN_CPU2/docs/README.md"],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(1, module.exit_code_for_mixed_details_mode(blocking_report))
        self.assertEqual(0, module.exit_code_for_mixed_details_mode(clean_report))

    def test_staging_plan_mode_always_returns_success_for_read_only_plan(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=["LTD_MAIN_CPU2/docs/README.md"],
            local_output_paths=["tmp/page.html"],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_staging_plan_mode(report))

    def test_stage_commands_mode_always_returns_success_for_read_only_templates(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_stage_commands_mode(report))

    def test_mixed_commands_mode_always_returns_success_for_read_only_templates(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_mixed_commands_mode(report))

    def test_mixed_plan_mode_always_returns_success_for_read_only_guidance(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_mixed_plan_mode(report))

    def test_mixed_matrix_mode_always_returns_success_for_read_only_guidance(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_mixed_matrix_mode(report))

    def test_mixed_resolution_plan_mode_always_returns_success_for_read_only_templates(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_mixed_resolution_plan_mode(report))

    def test_mixed_resolution_check_mode_returns_success_for_complete_coverage(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_mixed_resolution_check_mode(report))

    def test_mixed_resolution_check_mode_returns_failure_for_duplicate_coverage(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md", "docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(1, module.exit_code_for_mixed_resolution_check_mode(report))

    def test_mixed_boundary_check_mode_returns_success_for_clean_boundary(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=["docs/README.md", firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md", firmware_doc],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_mixed_boundary_check_mode(report))

    def test_mixed_boundary_check_mode_returns_failure_for_firmware_submit_leak(self) -> None:
        module = load_module()
        firmware_doc = "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html"
        report = module.StagingReport(
            mixed_paths=[firmware_doc],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[firmware_doc],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(1, module.exit_code_for_mixed_boundary_check_mode(report))

    def test_mixed_closure_audit_mode_returns_failure_until_mixed_and_contract_are_clean(self) -> None:
        module = load_module()
        version_doc = "docs/00_构建与版本/版本改动与测试/README.md"
        mixed_report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
            version_output_paths=[version_doc],
        )
        leaked_report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[version_doc],
            firmware_or_build_paths=[],
            other_paths=[],
            version_output_paths=[version_doc],
        )
        clean_report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
            version_output_paths=[version_doc],
        )

        self.assertEqual(1, module.exit_code_for_mixed_closure_audit_mode(mixed_report))
        self.assertEqual(1, module.exit_code_for_mixed_closure_audit_mode(leaked_report))
        self.assertEqual(0, module.exit_code_for_mixed_closure_audit_mode(clean_report))

    def test_submit_boundary_check_mode_returns_success_for_clean_scope(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["tools/check_docs.py"],
            firmware_or_build_paths=["LTD_MAIN_CPU2/Application/Src/app_main.c"],
            other_paths=[],
            release_record_paths=["CHANGELOG.md"],
        )

        self.assertEqual(0, module.exit_code_for_submit_boundary_check_mode(report))

    def test_submit_boundary_check_mode_returns_failure_for_forbidden_scope_leak(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=["CHANGELOG.md"],
            firmware_or_build_paths=[],
            other_paths=[],
            release_record_paths=["CHANGELOG.md"],
        )

        self.assertEqual(1, module.exit_code_for_submit_boundary_check_mode(report))

    def test_remaining_checklist_mode_returns_failure_until_all_work_is_resolved(self) -> None:
        module = load_module()
        dirty_report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        clean_report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(1, module.exit_code_for_remaining_checklist_mode(dirty_report))
        self.assertEqual(0, module.exit_code_for_remaining_checklist_mode(clean_report))

    def test_closure_plan_mode_always_returns_success_for_read_only_guidance(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_closure_plan_mode(report))

    def test_decision_checklist_mode_always_returns_success_for_read_only_guidance(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_decision_checklist_mode(report))

    def test_submit_plan_mode_always_returns_success_for_read_only_guidance(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_submit_plan_mode(report))

    def test_gate_matrix_mode_always_returns_success_for_read_only_guidance(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=["LTD_MAIN_CPU2/docs/README.md"],
            local_output_paths=["tmp/page.html"],
            legacy_site_script_paths=["tools/ltd-report-theme.css"],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_gate_matrix_mode(report))

    def test_final_check_mode_returns_blocking_exit_code(self) -> None:
        module = load_module()
        blocking_report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )
        clean_report = module.StagingReport(
            mixed_paths=[],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=[],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(1, module.exit_code_for_final_check_mode(blocking_report))
        self.assertEqual(0, module.exit_code_for_final_check_mode(clean_report))

    def test_write_local_audit_mode_always_returns_success_for_local_snapshot(self) -> None:
        module = load_module()
        report = module.StagingReport(
            mixed_paths=["docs/README.md"],
            legacy_doc_paths=[],
            local_output_paths=[],
            legacy_site_script_paths=[],
            staged_legacy_doc_removals=[],
            staged_local_output_removals=[],
            staged_legacy_site_script_removals=[],
            untracked_tool_paths=[],
            docs_migration_paths=["docs/README.md"],
            docs_check_tool_paths=[],
            firmware_or_build_paths=[],
            other_paths=[],
        )

        self.assertEqual(0, module.exit_code_for_write_local_audit_mode(report))

    def test_audit_categories_are_informational(self) -> None:
        module = load_module()

        report = module.build_staging_report(
            [
                ("M", " ", "docs/README.md"),
                ("?", "?", "tools/check_docs.py"),
                ("M", " ", "LTD_DISPLAY_CPU3/Application/app_main.c"),
                ("D", " ", "tools/build_static_docs_site.py"),
                ("M", " ", "CHANGELOG.md"),
            ]
        )

        self.assertFalse(report.has_blocking_findings())
        self.assertEqual(["docs/README.md", "tools/build_static_docs_site.py"], report.docs_migration_paths)
        self.assertEqual(["tools/check_docs.py"], report.docs_check_tool_paths)
        self.assertEqual(["LTD_DISPLAY_CPU3/Application/app_main.c"], report.firmware_or_build_paths)
        self.assertEqual(["CHANGELOG.md"], report.release_record_paths)
        self.assertEqual([], report.other_paths)

    def test_git_status_command_disables_quoted_non_ascii_paths(self) -> None:
        module = load_module()

        self.assertEqual(
            [
                "git",
                "-c",
                "core.quotePath=false",
                "status",
                "--short",
                "--untracked-files=all",
            ],
            module.git_status_command(),
        )

    def test_decode_git_output_uses_utf8_for_chinese_paths(self) -> None:
        module = load_module()

        text = module.decode_git_output("AM docs/00_构建与版本/文档库治理清单.md\n".encode("utf-8"))

        self.assertEqual("AM docs/00_构建与版本/文档库治理清单.md\n", text)

    def test_audit_output_is_sampled(self) -> None:
        module = load_module()
        buffer = io.StringIO()
        paths = [f"docs/topic/{index}.md" for index in range(module.AUDIT_SAMPLE_LIMIT + 2)]

        with redirect_stdout(buffer):
            module.print_audit_paths("提交范围审计：文档迁移相关路径", paths)

        output = buffer.getvalue()
        self.assertIn(f"（{module.AUDIT_SAMPLE_LIMIT + 2} 项）", output)
        self.assertIn("docs/topic/0.md", output)
        self.assertIn(f"docs/topic/{module.AUDIT_SAMPLE_LIMIT - 1}.md", output)
        self.assertNotIn(f"docs/topic/{module.AUDIT_SAMPLE_LIMIT}.md", output)
        self.assertIn("另有 2 项", output)

    def test_informational_cleanup_output_is_sampled(self) -> None:
        module = load_module()
        buffer = io.StringIO()
        paths = [f"tmp/pdfs/page-{index}.png" for index in range(module.AUDIT_SAMPLE_LIMIT + 1)]

        with redirect_stdout(buffer):
            module.print_informational_paths("已暂存的本地输出清理记录：", paths)

        output = buffer.getvalue()
        self.assertIn(f"（{module.AUDIT_SAMPLE_LIMIT + 1} 项）", output)
        self.assertIn("tmp/pdfs/page-0.png", output)
        self.assertIn(f"tmp/pdfs/page-{module.AUDIT_SAMPLE_LIMIT - 1}.png", output)
        self.assertNotIn(f"tmp/pdfs/page-{module.AUDIT_SAMPLE_LIMIT}.png", output)
        self.assertIn("另有 1 项", output)


if __name__ == "__main__":
    unittest.main()

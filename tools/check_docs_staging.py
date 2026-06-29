#!/usr/bin/env python3
"""检查文档迁移提交前的暂存区一致性，不修改 Git 状态。"""

from __future__ import annotations

import subprocess
import sys
from argparse import ArgumentParser, Namespace
from dataclasses import dataclass, field
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ENCODING = "utf-8"

LEGACY_DOC_PREFIXES = (
    "LTD_MAIN_CPU2/docs/",
    "LTD_DISPLAY_CPU3/docs/",
)

LOCAL_OUTPUT_PREFIXES = (
    "docs-site/",
    "site/",
    "tmp/",
)

LEGACY_SITE_SCRIPT_PATHS = {
    "tools/build_static_docs_site.py",
    "tools/check_static_docs_site_ui.py",
    "tools/ltd-report-theme.css",
}

AUDIT_SAMPLE_LIMIT = 20
LOCAL_AUDIT_REPORT_PATH = ROOT / "tmp" / "docs-migration-audit.md"

DOCS_CHECK_TOOL_PATHS = {
    "tools/check_docs.py",
    "tools/check_docs_staging.py",
    "tools/check_docs_structure.py",
    "tools/check_markdown_links.py",
    "tools/test_check_docs.py",
    "tools/test_check_docs_staging.py",
    "tools/test_check_docs_structure.py",
    "tools/test_check_markdown_links.py",
}

FIRMWARE_OR_BUILD_PREFIXES = (
    "LTD_MAIN_CPU2/",
    "LTD_DISPLAY_CPU3/",
    "tools/check_motor_motion_target_plan.py",
    "tools/generate_cpu3_flow_docs.py",
    "tools/update_flow_navigation.py",
)

RELEASE_RECORD_PATHS = {
    "CHANGELOG.md",
    "CHANGELOG.pdf",
}

VERSION_OUTPUT_PREFIX = "docs/00_构建与版本/版本改动与测试/"
VERSION_OUTPUT_LABEL = "后续版本文档输出"

LIST_CATEGORIES = (
    "migration-submit",
    "migration-add",
    "docs-migration",
    "docs-check-tools",
    "firmware-or-build",
    "release-records",
    "version-outputs",
    "other",
    "migration-removals",
    "local-output-removals",
    "mixed",
    "mixed-docs-governance",
    "mixed-firmware-docs",
)

MIXED_FIRMWARE_DOC_PATHS = {
    "docs/03_问题分析与整改/2026-06-27_CPU2系统级通信异常与编码器校验失败整改方案.html",
}


def parse_args(argv: list[str] | None = None) -> Namespace:
    parser = ArgumentParser(description="检查文档迁移提交前的暂存区一致性。")
    parser.add_argument(
        "--summary",
        action="store_true",
        help="只输出摘要、处理顺序和分类计数，不展开路径列表。",
    )
    parser.add_argument(
        "--audit",
        action="store_true",
        help="输出提交范围分组审计和处理顺序，不展开长路径列表。",
    )
    parser.add_argument(
        "--mixed-details",
        action="store_true",
        help="只输出 AM/MM 混合文件的已暂存/未暂存改动规模。",
    )
    parser.add_argument(
        "--mixed-commands",
        action="store_true",
        help="只输出 AM/MM 混合文件的 cached/worktree diff 检查命令模板。",
    )
    parser.add_argument(
        "--mixed-plan",
        action="store_true",
        help="输出 AM/MM 混合文件按治理/固件相关分组的只读收口计划。",
    )
    parser.add_argument(
        "--mixed-matrix",
        action="store_true",
        help="输出 AM/MM 混合文件的分类、改动规模和建议处理动作。",
    )
    parser.add_argument(
        "--mixed-resolution-plan",
        action="store_true",
        help="输出 AM/MM 混合文件按治理/固件相关分批收口的命令模板。",
    )
    parser.add_argument(
        "--mixed-resolution-check",
        action="store_true",
        help="只读检查 AM/MM 混合文件分批模板是否完整覆盖当前混合文件。",
    )
    parser.add_argument(
        "--mixed-boundary-check",
        action="store_true",
        help="只读检查混合治理文档是否进入迁移候选、固件混合文档是否已排除。",
    )
    parser.add_argument(
        "--mixed-closure-audit",
        action="store_true",
        help="只读汇总 AM/MM 混合文件收口、提交边界和后续版本文档输出契约状态。",
    )
    parser.add_argument(
        "--submit-boundary-check",
        action="store_true",
        help="只读检查 migration-submit 候选是否混入固件、版本记录、后续版本文档输出区、本地输出或其他路径。",
    )
    parser.add_argument(
        "--remaining-checklist",
        action="store_true",
        help="只读输出文档迁移最终通过前仍需收口或人工决策的分组清单。",
    )
    parser.add_argument(
        "--closure-plan",
        action="store_true",
        help="只读输出文档迁移从当前状态到最终验证的顺序路线和通过口径。",
    )
    parser.add_argument(
        "--staging-plan",
        action="store_true",
        help="输出文档迁移提交的手动收口顺序和只读清单命令。",
    )
    parser.add_argument(
        "--stage-commands",
        action="store_true",
        help="只输出文档迁移提交候选路径的暂存命令模板，不执行命令。",
    )
    parser.add_argument(
        "--decision-checklist",
        action="store_true",
        help="输出文档迁移提交前的只读决策清单，不展开路径、不修改暂存区。",
    )
    parser.add_argument(
        "--submit-plan",
        action="store_true",
        help="输出文档迁移提交拆分计划，区分迁移、固件、版本记录、后续版本文档输出区和本地输出范围。",
    )
    parser.add_argument(
        "--gate-matrix",
        action="store_true",
        help="输出文档迁移提交门禁矩阵，明确后续版本文档生成边界。",
    )
    parser.add_argument(
        "--final-check",
        action="store_true",
        help="输出文档迁移最终收口状态、阻塞项和下一步命令，不修改暂存区。",
    )
    parser.add_argument(
        "--version-output-contract",
        action="store_true",
        help="只读检查后续版本文档输出区不会进入本次文档迁移候选，也不阻塞 skill 生成新文档。",
    )
    parser.add_argument(
        "--write-local-audit",
        action="store_true",
        help="生成 tmp/docs-migration-audit.md 本地审计快照，不修改暂存区、不写入正式文档源。",
    )
    parser.add_argument(
        "--list",
        choices=LIST_CATEGORIES,
        dest="list_category",
        help="只输出指定分类的路径列表，便于提交前分离范围。",
    )
    return parser.parse_args(argv)


def configure_output(stdout=sys.stdout, stderr=sys.stderr) -> None:  # noqa: ANN001
    for stream in (stdout, stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding=OUTPUT_ENCODING, errors="replace")


@dataclass(frozen=True)
class Numstat:
    added: int | None
    deleted: int | None


@dataclass(frozen=True)
class MixedPathDetail:
    path: str
    staged: Numstat
    unstaged: Numstat


@dataclass(frozen=True)
class SubmitScopeRecommendation:
    title: str
    paths: list[str]


@dataclass(frozen=True)
class RemainingWorkItem:
    title: str
    paths: list[str]
    action: str
    blocking: bool


@dataclass(frozen=True)
class ClosurePlanStep:
    title: str
    command: str
    pass_criteria: str


@dataclass(frozen=True)
class MixedResolutionCoverage:
    mixed_paths: list[str]
    grouped_paths: list[str]
    governance_paths: list[str]
    firmware_paths: list[str]
    missing_paths: list[str]
    extra_paths: list[str]
    duplicated_paths: list[str]

    def is_complete(self) -> bool:
        return not (self.missing_paths or self.extra_paths or self.duplicated_paths)


@dataclass(frozen=True)
class MixedSubmitBoundary:
    governance_paths: list[str]
    firmware_paths: list[str]
    submit_paths: list[str]
    governance_missing_from_submit: list[str]
    firmware_present_in_submit: list[str]

    def is_clean(self) -> bool:
        return not (self.governance_missing_from_submit or self.firmware_present_in_submit)


@dataclass(frozen=True)
class SubmitBoundaryCheck:
    submit_paths: list[str]
    docs_migration_paths: list[str]
    docs_check_tool_paths: list[str]
    migration_cleanup_paths: list[str]
    excluded_mixed_firmware_paths: list[str]
    firmware_or_build_leaks: list[str]
    release_record_leaks: list[str]
    version_output_leaks: list[str]
    local_output_leaks: list[str]
    other_leaks: list[str]
    legacy_non_delete_leaks: list[str]
    excluded_mixed_firmware_leaks: list[str]
    missing_expected_paths: list[str]
    unexpected_paths: list[str]

    def is_clean(self) -> bool:
        return not (
            self.firmware_or_build_leaks
            or self.release_record_leaks
            or self.version_output_leaks
            or self.local_output_leaks
            or self.other_leaks
            or self.legacy_non_delete_leaks
            or self.excluded_mixed_firmware_leaks
            or self.missing_expected_paths
            or self.unexpected_paths
        )


@dataclass(frozen=True)
class VersionOutputContract:
    version_output_paths: list[str]
    submit_paths: list[str]
    version_output_leaks: list[str]

    def is_clean(self) -> bool:
        return not self.version_output_leaks


@dataclass(frozen=True)
class StagingReport:
    mixed_paths: list[str]
    legacy_doc_paths: list[str]
    local_output_paths: list[str]
    legacy_site_script_paths: list[str]
    staged_legacy_doc_removals: list[str]
    staged_local_output_removals: list[str]
    staged_legacy_site_script_removals: list[str]
    untracked_tool_paths: list[str]
    docs_migration_paths: list[str]
    docs_check_tool_paths: list[str]
    firmware_or_build_paths: list[str]
    other_paths: list[str]
    release_record_paths: list[str] = field(default_factory=list)
    version_output_paths: list[str] = field(default_factory=list)
    staged_migration_removals: list[str] = field(default_factory=list)

    def has_blocking_findings(self) -> bool:
        return bool(
            self.mixed_paths
            or self.legacy_doc_paths
            or self.local_output_paths
            or self.legacy_site_script_paths
        )

    def review_paths(self) -> list[str]:
        return merge_sorted_paths(
            self.firmware_or_build_paths,
            self.release_record_paths,
            self.version_output_paths,
            self.other_paths,
        )

    def review_count(self) -> int:
        return len(self.review_paths())


def unquote_status_path(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == '"' and value[-1] == '"':
        return value[1:-1]
    return value


def extract_status_path(raw_path: str) -> str:
    if " -> " in raw_path:
        raw_path = raw_path.rsplit(" -> ", 1)[1]
    return unquote_status_path(raw_path).replace("\\", "/")


def parse_porcelain_v1_status(lines: list[str]) -> list[tuple[str, str, str]]:
    entries: list[tuple[str, str, str]] = []
    for line in lines:
        if len(line) < 3:
            continue
        index_status = line[0]
        worktree_status = line[1]
        path = extract_status_path(line[3:])
        entries.append((index_status, worktree_status, path))
    return entries


def is_docs_migration_path(path: str) -> bool:
    return (
        path in {"AGENTS.md", ".gitignore"}
        or path in LEGACY_SITE_SCRIPT_PATHS
        or (path.startswith("docs/") and not path.startswith(VERSION_OUTPUT_PREFIX))
    )


def is_docs_check_tool_path(path: str) -> bool:
    return path in DOCS_CHECK_TOOL_PATHS


def is_firmware_or_build_path(path: str) -> bool:
    if any(path.startswith(prefix) for prefix in LEGACY_DOC_PREFIXES):
        return False
    return any(path.startswith(prefix) for prefix in FIRMWARE_OR_BUILD_PREFIXES)


def build_staging_report(entries: list[tuple[str, str, str]]) -> StagingReport:
    mixed_paths: list[str] = []
    legacy_doc_paths: list[str] = []
    local_output_paths: list[str] = []
    legacy_site_script_paths: list[str] = []
    staged_migration_removals: list[str] = []
    staged_legacy_doc_removals: list[str] = []
    staged_local_output_removals: list[str] = []
    staged_legacy_site_script_removals: list[str] = []
    untracked_tool_paths: list[str] = []
    docs_migration_paths: list[str] = []
    docs_check_tool_paths: list[str] = []
    firmware_or_build_paths: list[str] = []
    release_record_paths: list[str] = []
    version_output_paths: list[str] = []
    other_paths: list[str] = []

    for index_status, worktree_status, path in entries:
        if path.startswith(VERSION_OUTPUT_PREFIX):
            version_output_paths.append(path)
        elif is_docs_migration_path(path):
            docs_migration_paths.append(path)
            if index_status == "D" and worktree_status == " ":
                staged_migration_removals.append(path)
        elif is_docs_check_tool_path(path):
            docs_check_tool_paths.append(path)
        elif is_firmware_or_build_path(path):
            firmware_or_build_paths.append(path)
        elif path in RELEASE_RECORD_PATHS:
            release_record_paths.append(path)
        elif not any(path.startswith(prefix) for prefix in (*LEGACY_DOC_PREFIXES, *LOCAL_OUTPUT_PREFIXES)):
            other_paths.append(path)

        if index_status not in {" ", "?"} and worktree_status not in {" ", "?"}:
            mixed_paths.append(path)

        if any(path.startswith(prefix) for prefix in LEGACY_DOC_PREFIXES):
            if index_status == "D" and worktree_status == " ":
                staged_legacy_doc_removals.append(path)
            else:
                legacy_doc_paths.append(path)

        if any(path.startswith(prefix) for prefix in LOCAL_OUTPUT_PREFIXES):
            if index_status == "D" and worktree_status == " ":
                staged_local_output_removals.append(path)
            else:
                local_output_paths.append(path)

        if path in LEGACY_SITE_SCRIPT_PATHS:
            if index_status == "D" and worktree_status == " ":
                staged_legacy_site_script_removals.append(path)
            else:
                legacy_site_script_paths.append(path)

        if index_status == "?" and worktree_status == "?" and path.startswith("tools/"):
            untracked_tool_paths.append(path)

    return StagingReport(
        mixed_paths=sorted(set(mixed_paths)),
        legacy_doc_paths=sorted(set(legacy_doc_paths)),
        local_output_paths=sorted(set(local_output_paths)),
        legacy_site_script_paths=sorted(set(legacy_site_script_paths)),
        staged_migration_removals=sorted(set(staged_migration_removals)),
        staged_legacy_doc_removals=sorted(set(staged_legacy_doc_removals)),
        staged_local_output_removals=sorted(set(staged_local_output_removals)),
        staged_legacy_site_script_removals=sorted(set(staged_legacy_site_script_removals)),
        untracked_tool_paths=sorted(set(untracked_tool_paths)),
        docs_migration_paths=sorted(set(docs_migration_paths)),
        docs_check_tool_paths=sorted(set(docs_check_tool_paths)),
        firmware_or_build_paths=sorted(set(firmware_or_build_paths)),
        release_record_paths=sorted(set(release_record_paths)),
        version_output_paths=sorted(set(version_output_paths)),
        other_paths=sorted(set(other_paths)),
    )


def git_status_command() -> list[str]:
    return [
        "git",
        "-c",
        "core.quotePath=false",
        "status",
        "--short",
        "--untracked-files=all",
    ]


def git_numstat_command(*, staged: bool) -> list[str]:
    command = ["git", "-c", "core.quotePath=false", "diff", "--numstat"]
    if staged:
        command.append("--cached")
    return command


def decode_git_output(data: bytes) -> str:
    return data.decode(OUTPUT_ENCODING, errors="replace")


def git_status_lines() -> list[str]:
    result = subprocess.run(
        git_status_command(),
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        message = decode_git_output(result.stderr).strip() or decode_git_output(result.stdout).strip()
        raise RuntimeError(f"git status failed: {message}")
    stdout = decode_git_output(result.stdout)
    return [line.rstrip("\n") for line in stdout.splitlines() if line.strip()]


def parse_numstat_value(value: str) -> int | None:
    if value == "-":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def parse_numstat_lines(lines: list[str]) -> dict[str, Numstat]:
    stats: dict[str, Numstat] = {}
    for line in lines:
        parts = line.split("\t")
        if len(parts) < 3:
            continue
        raw_path = "\t".join(parts[2:])
        path = extract_status_path(raw_path)
        stats[path] = Numstat(
            added=parse_numstat_value(parts[0]),
            deleted=parse_numstat_value(parts[1]),
        )
    return stats


def git_numstat_lines(*, staged: bool) -> list[str]:
    result = subprocess.run(
        git_numstat_command(staged=staged),
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode != 0:
        message = decode_git_output(result.stderr).strip() or decode_git_output(result.stdout).strip()
        raise RuntimeError(f"git diff --numstat failed: {message}")
    stdout = decode_git_output(result.stdout)
    return [line.rstrip("\n") for line in stdout.splitlines() if line.strip()]


def git_numstat(*, staged: bool) -> dict[str, Numstat]:
    return parse_numstat_lines(git_numstat_lines(staged=staged))


def build_mixed_path_details(
    mixed_paths: list[str],
    *,
    staged_numstat: dict[str, Numstat],
    unstaged_numstat: dict[str, Numstat],
) -> list[MixedPathDetail]:
    zero = Numstat(added=0, deleted=0)
    return [
        MixedPathDetail(
            path=path,
            staged=staged_numstat.get(path, zero),
            unstaged=unstaged_numstat.get(path, zero),
        )
        for path in sorted(mixed_paths)
    ]


def format_numstat(value: Numstat) -> str:
    if value.added is None or value.deleted is None:
        return "二进制或不可统计"
    return f"+{value.added}/-{value.deleted}"


def merge_sorted_paths(*path_groups: list[str]) -> list[str]:
    merged: list[str] = []
    for paths in path_groups:
        merged.extend(paths)
    return sorted(set(merged))


def exclude_paths(paths: list[str], excluded: set[str]) -> list[str]:
    return sorted(path for path in paths if path not in excluded)


def migration_cleanup_paths(report: StagingReport) -> list[str]:
    return merge_sorted_paths(
        report.staged_migration_removals,
        report.staged_legacy_doc_removals,
        report.staged_legacy_site_script_removals,
    )


def migration_add_paths(report: StagingReport) -> list[str]:
    cleanup_paths = set(migration_cleanup_paths(report))
    return merge_sorted_paths(
        exclude_paths(exclude_paths(report.docs_migration_paths, MIXED_FIRMWARE_DOC_PATHS), cleanup_paths),
        report.docs_check_tool_paths,
    )


def duplicated_paths(paths: list[str]) -> list[str]:
    seen: set[str] = set()
    duplicates: set[str] = set()
    for path in paths:
        if path in seen:
            duplicates.add(path)
        seen.add(path)
    return sorted(duplicates)


def build_submit_scope_recommendations(report: StagingReport) -> list[SubmitScopeRecommendation]:
    recommendations: list[SubmitScopeRecommendation] = []

    include_paths = merge_sorted_paths(
        exclude_paths(report.docs_migration_paths, MIXED_FIRMWARE_DOC_PATHS),
        report.docs_check_tool_paths,
    )
    if include_paths:
        recommendations.append(
            SubmitScopeRecommendation(
                title="建议纳入文档迁移提交",
                paths=include_paths,
            )
        )

    cleanup_paths = migration_cleanup_paths(report)
    if cleanup_paths:
        recommendations.append(
            SubmitScopeRecommendation(
                title="建议随迁移提交保留删除记录",
                paths=cleanup_paths,
            )
        )

    review_paths = report.review_paths()
    if review_paths:
        recommendations.append(
            SubmitScopeRecommendation(
                title="需要单独判断是否属于本次提交",
                paths=review_paths,
            )
        )

    if report.local_output_paths:
        recommendations.append(
            SubmitScopeRecommendation(
                title="不应纳入正式文档提交的本地输出",
                paths=report.local_output_paths,
            )
        )

    if report.staged_local_output_removals:
        recommendations.append(
            SubmitScopeRecommendation(
                title="本地输出清理记录，提交前确认是否单独处理",
                paths=report.staged_local_output_removals,
            )
        )

    return recommendations


def build_next_actions(report: StagingReport) -> list[str]:
    actions: list[str] = []

    if report.mixed_paths:
        governance_count = mixed_docs_governance_count(report)
        firmware_count = mixed_firmware_docs_count(report)
        actions.append(
            f"先处理 {len(report.mixed_paths)} 个 AM/MM 混合文件"
            f"（治理文档 {governance_count} 个，固件相关文档 {firmware_count} 个）："
            "治理文档用 `--mixed-resolution-plan` 核对后随迁移暂存；"
            "固件相关文档只核对 diff，随固件改动单独判断。"
        )
    if report.legacy_doc_paths:
        actions.append(
            f"处理 {len(report.legacy_doc_paths)} 个旧 CPU docs 非纯删除状态："
            "迁移后不应继续保留或修改旧目录。"
        )
    if report.local_output_paths:
        actions.append(
            f"排除 {len(report.local_output_paths)} 个本地输出非纯删除状态："
            "docs-site/site/tmp 不能进入正式文档提交。"
        )
    if report.legacy_site_script_paths:
        actions.append(
            f"处理 {len(report.legacy_site_script_paths)} 个旧静态站脚本非纯删除状态："
            "旧脚本不应恢复为维护入口。"
        )
    if report.untracked_tool_paths:
        actions.append(
            f"决定 {len(report.untracked_tool_paths)} 个未跟踪工具脚本是否纳入本次文档检查能力提交。"
        )
    if report.firmware_or_build_paths:
        actions.append(
            f"单独判断 {len(report.firmware_or_build_paths)} 个固件或构建相关路径是否属于本次提交，"
            "避免混入纯文档迁移。"
        )
    if report.release_record_paths:
        actions.append(
            f"单独判断 {len(report.release_record_paths)} 个版本或发布记录路径是否属于本次提交，"
            "避免把版本日志/PDF 处理混入纯文档迁移。"
        )
    if report.version_output_paths:
        actions.append(
            f"运行 `--version-output-contract` 核对 {len(report.version_output_paths)} 个后续版本文档输出区路径；"
            "它不属于本次文档迁移候选，也不是迁移门禁的阻塞项。"
        )
    if report.other_paths:
        actions.append(f"单独判断 {len(report.other_paths)} 个其他路径是否属于本次文档迁移提交。")
    if report.staged_local_output_removals:
        actions.append(
            f"确认 {len(report.staged_local_output_removals)} 个本地输出删除记录是否单独清理；"
            "它不是后续版本文档生成的前置条件。"
        )

    return actions


def print_paths(title: str, paths: list[str]) -> None:
    if not paths:
        return
    print(title)
    for path in paths:
        print(f"- {path}")


def print_audit_paths(title: str, paths: list[str]) -> None:
    if not paths:
        return
    print(f"{title}（{len(paths)} 项）")
    for path in paths[:AUDIT_SAMPLE_LIMIT]:
        print(f"- {path}")
    remaining = len(paths) - AUDIT_SAMPLE_LIMIT
    if remaining > 0:
        print(f"- ... 另有 {remaining} 项")


def print_informational_paths(title: str, paths: list[str]) -> None:
    print_audit_paths(title, paths)


def print_mixed_path_details(details: list[MixedPathDetail]) -> None:
    if not details:
        return
    print("混合文件改动规模（用于提交前核对暂存区是否包含最新整理结果）：")
    for detail in details:
        print(
            f"- {detail.path}："
            f"已暂存 {format_numstat(detail.staged)}，"
            f"未暂存 {format_numstat(detail.unstaged)}"
        )


def print_mixed_details_report(
    report: StagingReport,
    *,
    staged_numstat: dict[str, Numstat],
    unstaged_numstat: dict[str, Numstat],
) -> None:
    print("文档迁移混合文件诊断：")
    print(f"- AM/MM 混合文件：{len(report.mixed_paths)}")
    if not report.mixed_paths:
        print("未发现 AM/MM 混合文件。")
        return
    print_mixed_path_details(
        build_mixed_path_details(
            report.mixed_paths,
            staged_numstat=staged_numstat,
            unstaged_numstat=unstaged_numstat,
        )
    )
    print_next_actions(
        [
            "逐个核对已暂存和未暂存 diff，先区分治理文档和固件相关文档。",
            "治理文档确认属于本次迁移后再随迁移暂存；固件相关文档只核对 diff，随固件改动单独判断。",
            "暂存已确认的迁移目标范围后，再运行 --summary 或完整检查确认迁移收口。",
        ]
    )


def mixed_path_category(path: str) -> str:
    if path in MIXED_FIRMWARE_DOC_PATHS:
        return "固件相关文档"
    return "治理文档"


def mixed_path_action(path: str) -> str:
    if path in MIXED_FIRMWARE_DOC_PATHS:
        return "跟随对应固件改动单独判断"
    return "随文档迁移统一核对后暂存"


def print_mixed_matrix(
    report: StagingReport,
    *,
    staged_numstat: dict[str, Numstat],
    unstaged_numstat: dict[str, Numstat],
) -> None:
    details = build_mixed_path_details(
        report.mixed_paths,
        staged_numstat=staged_numstat,
        unstaged_numstat=unstaged_numstat,
    )

    print("AM/MM 混合文件收口矩阵：")
    print("- 本命令只读 Git 状态，不执行 git add、不修改暂存区。")
    print(f"- 混合文件总数：{len(report.mixed_paths)}")
    print(f"- 治理文档：{mixed_docs_governance_count(report)}")
    print(f"- 固件相关文档：{mixed_firmware_docs_count(report)}")
    if not details:
        print("未发现 AM/MM 混合文件。")
        return
    print("逐项矩阵：")
    for detail in details:
        print(
            f"- {detail.path}："
            f"分类：{mixed_path_category(detail.path)}；"
            f"已暂存 {format_numstat(detail.staged)}，"
            f"未暂存 {format_numstat(detail.unstaged)}；"
            f"建议：{mixed_path_action(detail.path)}。"
        )
    print("只读核对命令：")
    print("- py tools\\check_docs_staging.py --mixed-details")
    print("- py tools\\check_docs_staging.py --mixed-commands")
    print("- py tools\\check_docs_staging.py --list mixed-docs-governance")
    print("- py tools\\check_docs_staging.py --list mixed-firmware-docs")


def build_mixed_resolution_steps(report: StagingReport) -> list[str]:
    commands = [
        "py tools\\check_docs_staging.py --mixed-resolution-check",
        "py tools\\check_docs_staging.py --mixed-boundary-check",
        "py tools\\check_docs_staging.py --submit-boundary-check",
        "py tools\\check_docs_staging.py --mixed-matrix",
        "py tools\\check_docs_staging.py --mixed-commands",
    ]

    governance_paths = paths_for_list_category(report, "mixed-docs-governance")
    firmware_paths = paths_for_list_category(report, "mixed-firmware-docs")

    if governance_paths:
        commands.append("py tools\\check_docs_staging.py --list mixed-docs-governance")
        for path in governance_paths:
            quoted_path = quote_path_for_powershell(path)
            commands.append(f"git diff --cached -- {quoted_path}")
            commands.append(f"git diff -- {quoted_path}")
            commands.append(f"git add -- {quoted_path}")

    if firmware_paths:
        commands.append("py tools\\check_docs_staging.py --list mixed-firmware-docs")
        for path in firmware_paths:
            quoted_path = quote_path_for_powershell(path)
            commands.append(f"git diff --cached -- {quoted_path}")
            commands.append(f"git diff -- {quoted_path}")
        commands.append("# 固件相关混合文档不在纯文档迁移模板中自动 git add，需随固件改动单独判断。")

    commands.append("py tools\\check_docs_staging.py --summary")
    return commands


def build_mixed_resolution_coverage(report: StagingReport) -> MixedResolutionCoverage:
    governance_paths = paths_for_list_category(report, "mixed-docs-governance")
    firmware_paths = paths_for_list_category(report, "mixed-firmware-docs")
    grouped_paths = governance_paths + firmware_paths
    mixed_set = set(report.mixed_paths)
    grouped_set = set(grouped_paths)

    return MixedResolutionCoverage(
        mixed_paths=report.mixed_paths,
        grouped_paths=grouped_paths,
        governance_paths=governance_paths,
        firmware_paths=firmware_paths,
        missing_paths=sorted(mixed_set - grouped_set),
        extra_paths=sorted(grouped_set - mixed_set),
        duplicated_paths=duplicated_paths(report.mixed_paths) + duplicated_paths(grouped_paths),
    )


def print_mixed_resolution_check(report: StagingReport) -> None:
    coverage = build_mixed_resolution_coverage(report)

    print("AM/MM 混合文件分批模板覆盖率预检：")
    print("- 本命令只读 Git 状态，不执行 git add、不修改暂存区。")
    print(f"- 混合文件总数：{len(coverage.mixed_paths)}")
    print(f"- 模板覆盖文件：{len(set(coverage.grouped_paths))}")
    print(f"- 治理文档：{len(coverage.governance_paths)}")
    print(f"- 固件相关文档：{len(coverage.firmware_paths)}")
    print(f"- 遗漏：{len(coverage.missing_paths)}")
    print(f"- 额外：{len(coverage.extra_paths)}")
    print(f"- 重复：{len(coverage.duplicated_paths)}")
    if coverage.is_complete():
        print("预检结论：覆盖完整。治理文档批次和固件相关文档批次合计覆盖全部 AM/MM 混合文件。")
        print("建议下一步：")
        print("- py tools\\check_docs_staging.py --mixed-boundary-check")
        print("- py tools\\check_docs_staging.py --submit-boundary-check")
        print("- py tools\\check_docs_staging.py --mixed-resolution-plan")
        print("- 治理文档可按模板暂存，固件相关文档只核对 diff，需随固件改动单独判断。")
        return

    print("预检结论：覆盖不完整。请先修正混合文件分类或模板生成逻辑，再执行分批暂存模板。")
    print_audit_paths("遗漏路径", coverage.missing_paths)
    print_audit_paths("额外路径", coverage.extra_paths)
    print_audit_paths("重复路径", coverage.duplicated_paths)


def build_mixed_submit_boundary(report: StagingReport) -> MixedSubmitBoundary:
    governance_paths = paths_for_list_category(report, "mixed-docs-governance")
    firmware_paths = paths_for_list_category(report, "mixed-firmware-docs")
    submit_paths = paths_for_list_category(report, "migration-submit")
    submit_set = set(submit_paths)

    return MixedSubmitBoundary(
        governance_paths=governance_paths,
        firmware_paths=firmware_paths,
        submit_paths=submit_paths,
        governance_missing_from_submit=sorted(path for path in governance_paths if path not in submit_set),
        firmware_present_in_submit=sorted(path for path in firmware_paths if path in submit_set),
    )


def print_mixed_boundary_check(report: StagingReport) -> None:
    boundary = build_mixed_submit_boundary(report)

    print("AM/MM 混合文件提交边界检查：")
    print("- 本命令只读 Git 状态，不执行 git add、不修改暂存区。")
    print(f"- 混合文件总数：{len(report.mixed_paths)}")
    print(f"- 治理文档：{len(boundary.governance_paths)}")
    print(f"- 固件相关文档：{len(boundary.firmware_paths)}")
    print(f"- 迁移候选总数：{len(boundary.submit_paths)}")
    print(f"- 治理文档未进入迁移候选：{len(boundary.governance_missing_from_submit)}")
    print(f"- 固件相关文档误入迁移候选：{len(boundary.firmware_present_in_submit)}")
    if boundary.is_clean():
        print("边界结论：清晰。治理文档可随迁移收口，固件相关文档已从纯文档迁移候选排除。")
        print("建议下一步：")
        print("- py tools\\check_docs_staging.py --submit-boundary-check")
        print("- py tools\\check_docs_staging.py --mixed-resolution-plan")
        print("- py tools\\check_docs_staging.py --summary")
        return

    print("边界结论：需要修正。请先修正混合文件分类或迁移候选生成逻辑。")
    print_audit_paths("治理文档未进入迁移候选", boundary.governance_missing_from_submit)
    print_audit_paths("固件相关文档误入迁移候选", boundary.firmware_present_in_submit)


def print_mixed_closure_audit(report: StagingReport) -> None:
    coverage = build_mixed_resolution_coverage(report)
    mixed_boundary = build_mixed_submit_boundary(report)
    submit_boundary = build_submit_boundary_check(report)
    version_contract = build_version_output_contract(report)
    mixed_closed = not report.mixed_paths
    all_clean = (
        mixed_closed
        and coverage.is_complete()
        and mixed_boundary.is_clean()
        and submit_boundary.is_clean()
        and version_contract.is_clean()
    )

    print("AM/MM 混合文件收口审计：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print("- 后续版本文档仍可由 skill 写入 docs/00_构建与版本/版本改动与测试/，随对应版本提交另行判断。")
    print(f"- 混合文件总数：{len(report.mixed_paths)}")
    print(f"- 治理文档：{len(coverage.governance_paths)}")
    print(f"- 固件相关文档：{len(coverage.firmware_paths)}")
    print(f"- 覆盖率：{'完整' if coverage.is_complete() else '需要修正'}")
    print(f"- 提交边界：{'清晰' if mixed_boundary.is_clean() and submit_boundary.is_clean() else '需要修正'}")
    print(f"- 版本输出契约：{'清晰' if version_contract.is_clean() else '需要修正'}")
    print(f"- 当前是否可进入混合文件收口完成态：{'是' if all_clean else '否'}")
    if all_clean:
        print("审计结论：混合文件已收口，迁移候选和版本输出契约均清晰。")
    else:
        print("审计结论：仍需继续收口或修正边界后再进入最终暂存确认。")
    print("建议下一步：")
    print("- py tools\\check_docs_staging.py --mixed-resolution-check")
    print("- py tools\\check_docs_staging.py --mixed-boundary-check")
    print("- py tools\\check_docs_staging.py --submit-boundary-check")
    print("- py tools\\check_docs_staging.py --version-output-contract")
    print("- py tools\\check_docs_staging.py --mixed-resolution-plan")
    print("- py tools\\check_docs_staging.py --remaining-checklist")


def build_submit_boundary_check(report: StagingReport) -> SubmitBoundaryCheck:
    submit_paths = paths_for_list_category(report, "migration-submit")
    submit_set = set(submit_paths)
    cleanup_paths = migration_cleanup_paths(report)
    docs_migration_paths = exclude_paths(
        exclude_paths(report.docs_migration_paths, MIXED_FIRMWARE_DOC_PATHS),
        set(cleanup_paths),
    )
    expected_paths = merge_sorted_paths(
        docs_migration_paths,
        report.docs_check_tool_paths,
        cleanup_paths,
    )
    expected_set = set(expected_paths)

    return SubmitBoundaryCheck(
        submit_paths=submit_paths,
        docs_migration_paths=docs_migration_paths,
        docs_check_tool_paths=report.docs_check_tool_paths,
        migration_cleanup_paths=cleanup_paths,
        excluded_mixed_firmware_paths=sorted(path for path in report.docs_migration_paths if path in MIXED_FIRMWARE_DOC_PATHS),
        firmware_or_build_leaks=sorted(path for path in report.firmware_or_build_paths if path in submit_set),
        release_record_leaks=sorted(path for path in report.release_record_paths if path in submit_set),
        version_output_leaks=sorted(path for path in report.version_output_paths if path in submit_set),
        local_output_leaks=sorted(
            path
            for path in merge_sorted_paths(report.local_output_paths, report.staged_local_output_removals)
            if path in submit_set
        ),
        other_leaks=sorted(path for path in report.other_paths if path in submit_set),
        legacy_non_delete_leaks=sorted(path for path in report.legacy_doc_paths if path in submit_set),
        excluded_mixed_firmware_leaks=sorted(path for path in MIXED_FIRMWARE_DOC_PATHS if path in submit_set),
        missing_expected_paths=sorted(path for path in expected_set if path not in submit_set),
        unexpected_paths=sorted(path for path in submit_set if path not in expected_set),
    )


def print_submit_boundary_check(report: StagingReport) -> None:
    boundary = build_submit_boundary_check(report)

    print("文档迁移候选边界检查：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print(f"- 迁移候选总数：{len(boundary.submit_paths)}")
    print(f"- 文档迁移主体：{len(boundary.docs_migration_paths)}")
    print(f"- 文档检查工具：{len(boundary.docs_check_tool_paths)}")
    print(f"- 旧 CPU docs/旧静态站脚本删除记录：{len(boundary.migration_cleanup_paths)}")
    print(f"- 已排除固件相关混合文档：{len(boundary.excluded_mixed_firmware_paths)}")
    print(f"- 错误混入固件/构建：{len(boundary.firmware_or_build_leaks)}")
    print(f"- 错误混入版本/发布记录：{len(boundary.release_record_leaks)}")
    print(f"- 错误混入后续版本文档输出区：{len(boundary.version_output_leaks)}")
    print(f"- 错误混入本地输出：{len(boundary.local_output_leaks)}")
    print(f"- 错误混入其他路径：{len(boundary.other_leaks)}")
    print(f"- 旧 CPU docs 非纯删除误入候选：{len(boundary.legacy_non_delete_leaks)}")
    print(f"- 已排除固件相关混合文档误入候选：{len(boundary.excluded_mixed_firmware_leaks)}")
    print(f"- 候选缺少预期路径：{len(boundary.missing_expected_paths)}")
    print(f"- 候选存在非预期路径：{len(boundary.unexpected_paths)}")
    if boundary.is_clean():
        print("边界结论：清晰。migration-submit 只包含文档迁移主体、文档检查工具和旧目录清理记录。")
        print("建议下一步：")
        print("- py tools\\check_docs_staging.py --list migration-submit")
        print("- py tools\\check_docs_staging.py --stage-commands")
        return

    print("边界结论：需要修正。请先修正 migration-submit 候选生成逻辑或路径分类。")
    print_audit_paths("错误混入固件/构建", boundary.firmware_or_build_leaks)
    print_audit_paths("错误混入版本/发布记录", boundary.release_record_leaks)
    print_audit_paths("错误混入后续版本文档输出区", boundary.version_output_leaks)
    print_audit_paths("错误混入本地输出", boundary.local_output_leaks)
    print_audit_paths("错误混入其他路径", boundary.other_leaks)
    print_audit_paths("旧 CPU docs 非纯删除误入候选", boundary.legacy_non_delete_leaks)
    print_audit_paths("已排除固件相关混合文档误入候选", boundary.excluded_mixed_firmware_leaks)
    print_audit_paths("候选缺少预期路径", boundary.missing_expected_paths)
    print_audit_paths("候选存在非预期路径", boundary.unexpected_paths)


def build_version_output_contract(report: StagingReport) -> VersionOutputContract:
    submit_paths = paths_for_list_category(report, "migration-submit")
    submit_set = set(submit_paths)
    return VersionOutputContract(
        version_output_paths=report.version_output_paths,
        submit_paths=submit_paths,
        version_output_leaks=sorted(path for path in report.version_output_paths if path in submit_set),
    )


def print_version_output_contract(report: StagingReport) -> None:
    contract = build_version_output_contract(report)

    print("后续版本文档生成契约检查：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print("- 正式版本输出目录：docs/00_构建与版本/版本改动与测试/")
    print("- 后续 skill 可继续新增该目录 Markdown，随对应版本提交另行判断。")
    print("- 本检查只验证仓库迁移边界，不修改 C:\\Users\\admin\\.codex\\skills。")
    print(f"- 当前版本输出区改动：{len(contract.version_output_paths)} 个")
    print(f"- migration-submit 候选：{len(contract.submit_paths)} 个")
    print(f"- 错误进入 migration-submit：{len(contract.version_output_leaks)} 个")
    if contract.is_clean():
        print("契约结论：清晰。版本输出区不进入本次纯文档迁移候选，也不是迁移门禁阻塞项。")
        return

    print("契约结论：需要修正。版本输出区不得混入本次 migration-submit。")
    print_audit_paths("错误进入 migration-submit 的版本输出区路径", contract.version_output_leaks)


def build_remaining_work_items(report: StagingReport) -> list[RemainingWorkItem]:
    items: list[RemainingWorkItem] = []

    groups = [
        (
            "治理文档混合文件",
            paths_for_list_category(report, "mixed-docs-governance"),
            "核对 cached/worktree diff 后随文档迁移暂存。",
            True,
        ),
        (
            "固件相关混合文档",
            paths_for_list_category(report, "mixed-firmware-docs"),
            "按固件改动另行判断版本、构建和流程图影响。",
            True,
        ),
        (
            "旧 CPU docs 非纯删除状态",
            report.legacy_doc_paths,
            "迁移后旧 CPU docs 不应继续保留或修改，需先变成纯删除或移出本次范围。",
            True,
        ),
        (
            "本地输出非纯删除状态",
            report.local_output_paths,
            "docs-site/site/tmp 只作本地输出，不能进入正式文档提交。",
            True,
        ),
        (
            "旧静态站脚本非纯删除状态",
            report.legacy_site_script_paths,
            "旧静态站脚本不再作为维护入口，需先保持删除或移出本次范围。",
            True,
        ),
        (
            "未跟踪文档检查工具",
            report.untracked_tool_paths,
            "若属于本次文档检查能力，需在最终提交范围中显式纳入。",
            False,
        ),
        (
            "固件/构建相关路径",
            report.firmware_or_build_paths,
            "按固件改动另行判断版本、构建、CHANGELOG 和流程图影响。",
            False,
        ),
        (
            "版本/发布记录路径",
            report.release_record_paths,
            "随固件发布、升版或明确 PDF 处理另行判断，不默认混入纯文档迁移。",
            False,
        ),
        (
            "后续版本文档输出区路径",
            report.version_output_paths,
            "运行 `--version-output-contract` 核对边界；随对应版本提交另行判断，不默认混入本次文档迁移，也不阻塞 skill 生成新版本文档。",
            False,
        ),
        (
            "其他未归类路径",
            report.other_paths,
            "先判断业务归属，再决定是否加入迁移提交或另开提交。",
            False,
        ),
        (
            "本地输出清理记录",
            report.staged_local_output_removals,
            "只作为本地输出清理单独判断，不是后续版本文档生成前置条件。",
            False,
        ),
    ]

    for title, paths, action, blocking in groups:
        if paths:
            items.append(RemainingWorkItem(title=title, paths=paths, action=action, blocking=blocking))
    return items


def print_remaining_checklist(report: StagingReport) -> None:
    items = build_remaining_work_items(report)
    boundary = build_submit_boundary_check(report)
    blocking_items = [item for item in items if item.blocking]
    decision_items = [item for item in items if not item.blocking]

    print("文档迁移剩余收口清单：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print(f"- 候选边界：{'清晰' if boundary.is_clean() else '需要修正'}")
    print(f"- 阻塞收口分组：{len(blocking_items)}")
    print(f"- 仍需人工决策分组：{len(decision_items)}")
    print(f"- 迁移提交候选：{len(boundary.submit_paths)}")
    if not items:
        print("当前没有剩余收口项；仍需最终人工确认暂存范围。")
        return
    for item in items:
        label = "阻塞" if item.blocking else "决策"
        print(f"{label} - {item.title}：{len(item.paths)} 项")
        print(f"  动作：{item.action}")
        for path in item.paths[:AUDIT_SAMPLE_LIMIT]:
            print(f"  - {path}")
        remaining = len(item.paths) - AUDIT_SAMPLE_LIMIT
        if remaining > 0:
            print(f"  - ... 另有 {remaining} 项")
    print("下一步：")
    print("- py tools\\check_docs_staging.py --mixed-closure-audit")
    if report.version_output_paths:
        print("- py tools\\check_docs_staging.py --version-output-contract")
    print("- py tools\\check_docs_staging.py --mixed-resolution-plan")
    print("- py tools\\check_docs_staging.py --write-local-audit")
    print("- py tools\\check_docs_staging.py --stage-commands")


def build_closure_plan_steps(report: StagingReport) -> list[ClosurePlanStep]:
    submit_count = len(paths_for_list_category(report, "migration-submit"))
    review_count = report.review_count()
    steps: list[ClosurePlanStep] = []

    if report.mixed_paths:
        steps.append(
            ClosurePlanStep(
                title=(
                    f"混合文件分批收口（治理文档 {mixed_docs_governance_count(report)} 个，"
                    f"固件相关文档 {mixed_firmware_docs_count(report)} 个）"
                ),
                command=(
                    "py tools\\check_docs_staging.py --mixed-closure-audit；"
                    "py tools\\check_docs_staging.py --mixed-resolution-plan；"
                    "py tools\\check_docs_staging.py --mixed-details"
                ),
                pass_criteria=(
                    "py tools\\check_docs_staging.py --mixed-resolution-check 返回 0，"
                    "且 py tools\\check_docs_staging.py --mixed-boundary-check 返回 0；"
                    "治理文档可随迁移核对，固件相关混合文档只核对 diff 并跟随固件改动判断。"
                ),
            )
        )

    if report.legacy_doc_paths or report.local_output_paths or report.legacy_site_script_paths:
        steps.append(
            ClosurePlanStep(
                title=(
                    "旧目录和本地输出边界收口"
                    f"（旧 CPU docs {len(report.legacy_doc_paths)} 个，"
                    f"本地输出 {len(report.local_output_paths)} 个，"
                    f"旧静态站脚本 {len(report.legacy_site_script_paths)} 个）"
                ),
                command="py tools\\check_docs_staging.py --remaining-checklist",
                pass_criteria="旧 CPU docs、docs-site/site/tmp 和旧静态站脚本没有非纯删除状态；只读检查不移动、不删除文件。",
            )
        )

    steps.append(
        ClosurePlanStep(
            title=f"迁移候选边界确认（候选 {submit_count} 个）",
            command="py tools\\check_docs_staging.py --submit-boundary-check",
            pass_criteria=(
                "返回 0，且 migration-submit 只包含文档迁移主体、文档检查工具和旧目录清理记录；"
                "不得混入固件/构建、版本/发布记录、后续版本文档输出区、本地输出、其他路径或固件相关混合文档。"
            ),
        )
    )

    if report.version_output_paths:
        steps.append(
            ClosurePlanStep(
                title=f"后续版本文档输出契约确认（{len(report.version_output_paths)} 个）",
                command="py tools\\check_docs_staging.py --version-output-contract",
                pass_criteria=(
                    "返回 0，且后续版本文档输出区不进入 migration-submit；"
                    "该检查只读 Git 状态，不修改 C:\\Users\\admin\\.codex\\skills。"
                ),
            )
        )

    if review_count or report.staged_local_output_removals:
        steps.append(
            ClosurePlanStep(
                title=f"非迁移路径人工分离（{review_count} 个）",
                command=(
                    "py tools\\check_docs_staging.py --submit-plan；"
                    "py tools\\check_docs_staging.py --list firmware-or-build；"
                    "py tools\\check_docs_staging.py --list release-records；"
                    "py tools\\check_docs_staging.py --list version-outputs；"
                    "py tools\\check_docs_staging.py --list other"
                ),
                pass_criteria=(
                    "固件/构建、版本/发布记录、version-outputs、其他路径和本地输出清理记录均已决定归属；"
                    "version-outputs 随对应版本提交，不进入 migration-submit，且不作为本次纯文档迁移前置条件。"
                ),
            )
        )

    steps.append(
        ClosurePlanStep(
            title="本地审计快照生成",
            command="py tools\\check_docs_staging.py --write-local-audit",
            pass_criteria=(
                "生成 tmp/docs-migration-audit.md，报告只作为本地快照，"
                "不进入正式文档源、不执行 git add、不影响后续版本文档输出区。"
            ),
        )
    )
    steps.append(
        ClosurePlanStep(
            title="迁移暂存模板生成",
            command="py tools\\check_docs_staging.py --stage-commands",
            pass_criteria=(
                "输出的模板用 migration-submit 做审计总清单，"
                "用 migration-add 生成可执行 git add 路径文件；"
                "脚本本身只打印模板、不执行 git add、不修改暂存区。"
            ),
        )
    )
    steps.append(
        ClosurePlanStep(
            title="文档与本地站点验证",
            command="py -B tools\\check_docs.py；npm run check（工作目录：docs-site）",
            pass_criteria="文档结构、Markdown 链接、流程 HTML href/src 资源、本地站点构建、内部链接和流程页 HTTP 冒烟检查均通过。",
        )
    )
    steps.append(
        ClosurePlanStep(
            title="最终收口复核",
            command="py tools\\check_docs_staging.py --final-check；py tools\\check_docs_staging.py --summary",
            pass_criteria="阻塞项为 0 后再进入人工暂存确认；如仍有阻塞项，回到本路线对应步骤继续处理。",
        )
    )
    return steps


def print_closure_plan(report: StagingReport) -> None:
    submit_count = len(paths_for_list_category(report, "migration-submit"))
    review_count = report.review_count()
    blocking_count = len([item for item in build_remaining_work_items(report) if item.blocking])

    print("文档迁移收口路线：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print("- 只服务本次 docs/ 迁移收口；后续版本文档输出区不阻塞后续 skill 生成新版本文档。")
    print("当前概览：")
    print(f"- 迁移提交候选：{submit_count}")
    print(f"- 必须先收口：{blocking_count} 组")
    print(f"- 必须单独判断：{review_count + len(report.staged_local_output_removals)}")
    print(f"- 后续版本文档输出区：{len(report.version_output_paths)}")
    print("执行路线：")
    for index, step in enumerate(build_closure_plan_steps(report), start=1):
        print(f"{index}. {step.title}")
        print(f"   命令：{step.command}")
        print(f"   通过口径：{step.pass_criteria}")


def print_mixed_resolution_plan(report: StagingReport) -> None:
    governance_paths = paths_for_list_category(report, "mixed-docs-governance")
    firmware_paths = paths_for_list_category(report, "mixed-firmware-docs")

    print("AM/MM 混合文件分批收口模板：")
    print("- 本命令只打印模板，不执行 git add、不修改暂存区。")
    print(f"- 混合文件总数：{len(report.mixed_paths)}")
    print(f"- 治理文档：{len(governance_paths)}")
    print(f"- 固件相关文档：{len(firmware_paths)}")
    if not report.mixed_paths:
        print("无需生成混合文件收口模板。")
        return
    print("治理文档批次：")
    if governance_paths:
        print("- 先核对 cached/worktree diff，确认未暂存内容属于文档迁移后再暂存。")
    else:
        print("- 当前没有治理文档混合文件。")
    print("固件相关文档批次：")
    if firmware_paths:
        print("- 只输出 diff 核对命令，不输出 git add；需跟随对应固件改动单独判断。")
    else:
        print("- 当前没有固件相关混合文档。")
    print("可复制模板：")
    for command in build_mixed_resolution_steps(report):
        print(command)


def quote_path_for_powershell(path: str) -> str:
    return "'" + path.replace("'", "''") + "'"


def build_mixed_diff_commands(mixed_paths: list[str]) -> list[str]:
    commands: list[str] = []
    for path in sorted(mixed_paths):
        quoted_path = quote_path_for_powershell(path)
        commands.append(f"git diff --cached -- {quoted_path}")
        commands.append(f"git diff -- {quoted_path}")
    return commands


def print_mixed_commands(report: StagingReport) -> None:
    print("AM/MM 混合文件 diff 命令模板：")
    print("- 本命令只打印模板，不执行 git diff、不修改暂存区。")
    print(f"- 混合文件：{len(report.mixed_paths)}")
    if not report.mixed_paths:
        print("无需生成 mixed diff 命令。")
        return
    for command in build_mixed_diff_commands(report.mixed_paths):
        print(command)


def print_mixed_plan(report: StagingReport) -> None:
    governance_paths = paths_for_list_category(report, "mixed-docs-governance")
    firmware_paths = paths_for_list_category(report, "mixed-firmware-docs")

    print("AM/MM 混合文件收口计划：")
    print("- 本命令只读 Git 状态，不执行 git add、不修改暂存区。")
    print(f"- 混合文件总数：{len(report.mixed_paths)}")
    print(f"- 治理文档：{len(governance_paths)}")
    print(f"- 固件相关文档：{len(firmware_paths)}")
    print("处理原则：")
    print("- 治理文档可随文档迁移统一核对后暂存。")
    print("- 固件相关文档需跟随对应固件改动单独判断。")
    print("只读核对命令：")
    print("- py tools\\check_docs_staging.py --mixed-matrix")
    print("- py tools\\check_docs_staging.py --mixed-details")
    print("- py tools\\check_docs_staging.py --list mixed-docs-governance")
    print("- py tools\\check_docs_staging.py --list mixed-firmware-docs")
    print_audit_paths("治理文档混合文件", governance_paths)
    print_audit_paths("固件相关混合文档", firmware_paths)


def build_staging_plan_steps(report: StagingReport) -> list[str]:
    steps: list[str] = []
    submit_count = len(paths_for_list_category(report, "migration-submit"))

    if report.mixed_paths:
        steps.append("运行 `py tools\\check_docs_staging.py --mixed-plan`，先区分治理文档和固件相关文档。")
        steps.append(
            "运行 `py tools\\check_docs_staging.py --mixed-matrix`，"
            "逐项核对分类、已暂存/未暂存规模和建议处理动作。"
        )
        steps.append(
            "运行 `py tools\\check_docs_staging.py --mixed-resolution-check`，"
            "确认分批模板覆盖全部 AM/MM 混合文件。"
        )
        steps.append(
            "运行 `py tools\\check_docs_staging.py --mixed-boundary-check`，"
            "确认治理文档进入迁移候选且固件相关文档被排除。"
        )
        steps.append(
            "运行 `py tools\\check_docs_staging.py --submit-boundary-check`，"
            "确认迁移候选未混入固件、版本记录、后续版本文档输出区、本地输出或其他路径。"
        )
        steps.append(
            "运行 `py tools\\check_docs_staging.py --mixed-closure-audit`，"
            "汇总混合文件覆盖率、提交边界和版本输出契约状态。"
        )
        if report.version_output_paths:
            steps.append(
                "运行 `py tools\\check_docs_staging.py --version-output-contract`，"
                "确认后续版本文档输出区不进入本次迁移候选且不影响 skill 继续生成新方案。"
            )
        steps.append(
            "运行 `py tools\\check_docs_staging.py --mixed-resolution-plan`，"
            "生成治理文档和固件相关文档的分批核对模板。"
        )
        steps.append(
            "运行 `py tools\\check_docs_staging.py --mixed-details`，"
            f"逐个核对 {len(report.mixed_paths)} 个 AM/MM 混合文件。"
        )
    if report.legacy_doc_paths:
        steps.append(
            f"处理 {len(report.legacy_doc_paths)} 个旧 CPU docs 非纯删除状态，"
            "迁移后不应继续保留旧目录改动。"
        )
    if report.local_output_paths or report.legacy_site_script_paths:
        steps.append(
            "先排除本地输出或旧静态站脚本的非纯删除状态，"
            "避免恢复已淘汰维护入口。"
        )

    steps.append(
        "用 `py tools\\check_docs_staging.py --gate-matrix` "
        "先确认必须收口、可随迁移和必须单独判断的门禁项。"
    )
    steps.append(
        "用 `py tools\\check_docs_staging.py --list migration-submit` "
        f"导出 {submit_count} 个文档迁移候选路径。"
    )
    steps.append("用 `py tools\\check_docs_staging.py --submit-plan` 复核迁移、固件、版本记录、后续版本文档输出和本地输出拆分。")
    if report.firmware_or_build_paths:
        steps.append(
            "用 `py tools\\check_docs_staging.py --list firmware-or-build` "
            f"单独核对 {len(report.firmware_or_build_paths)} 个固件或构建相关路径。"
        )
    if report.release_record_paths:
        steps.append(
            "用 `py tools\\check_docs_staging.py --list release-records` "
            f"单独核对 {len(report.release_record_paths)} 个版本或发布记录路径。"
        )
    if report.version_output_paths:
        steps.append(
            "用 `py tools\\check_docs_staging.py --list version-outputs` "
            f"单独核对 {len(report.version_output_paths)} 个后续版本文档输出区路径；"
            "它不进入 migration-submit，也不阻塞后续 skill 生成新版本文档。"
        )
    if report.other_paths:
        steps.append(
            "用 `py tools\\check_docs_staging.py --list other` "
            f"单独核对 {len(report.other_paths)} 个其他路径。"
        )
    if report.staged_local_output_removals:
        steps.append(
            f"确认 {len(report.staged_local_output_removals)} 个本地输出删除记录是否单独清理，"
            "不把 docs-site/site/tmp 当作文档源。"
        )

    steps.append(
        "最终暂存已确认的迁移目标范围后运行 `py tools\\check_docs_staging.py --summary` 和完整检查；"
        "已知固件相关混合文档不随纯文档迁移模板自动纳入。"
    )
    return steps


def print_staging_plan(report: StagingReport) -> None:
    submit_paths = paths_for_list_category(report, "migration-submit")
    review_paths = report.review_paths()
    excluded_paths = merge_sorted_paths(report.local_output_paths, report.staged_local_output_removals)

    print("文档迁移提交手动收口计划：")
    print("- 本命令只读 Git 状态，不执行 git add、不修改暂存区。")
    print(f"- 迁移提交候选：{len(submit_paths)}")
    print(f"- 需要单独判断：{len(review_paths)}")
    print(f"- 不应纳入正式文档提交：{len(excluded_paths)}")
    print(f"- AM/MM 混合文件：{len(report.mixed_paths)}")
    print(f"- 混合治理文档：{mixed_docs_governance_count(report)}")
    print(f"- 混合固件文档：{mixed_firmware_docs_count(report)}")
    print("只读清单命令：")
    print("- py tools\\check_docs_staging.py --closure-plan")
    print("- py tools\\check_docs_staging.py --gate-matrix")
    print("- py tools\\check_docs_staging.py --submit-plan")
    print("- py tools\\check_docs_staging.py --remaining-checklist")
    print("- py tools\\check_docs_staging.py --mixed-plan")
    print("- py tools\\check_docs_staging.py --mixed-matrix")
    print("- py tools\\check_docs_staging.py --mixed-resolution-check")
    print("- py tools\\check_docs_staging.py --mixed-boundary-check")
    print("- py tools\\check_docs_staging.py --submit-boundary-check")
    print("- py tools\\check_docs_staging.py --mixed-closure-audit")
    print("- py tools\\check_docs_staging.py --version-output-contract")
    print("- py tools\\check_docs_staging.py --write-local-audit")
    print("- py tools\\check_docs_staging.py --mixed-resolution-plan")
    print("- py tools\\check_docs_staging.py --mixed-details")
    print("- py tools\\check_docs_staging.py --list mixed-docs-governance")
    print("- py tools\\check_docs_staging.py --list mixed-firmware-docs")
    print("- py tools\\check_docs_staging.py --list migration-submit")
    print("- py tools\\check_docs_staging.py --list firmware-or-build")
    print("- py tools\\check_docs_staging.py --list release-records")
    print("- py tools\\check_docs_staging.py --list version-outputs")
    print("- py tools\\check_docs_staging.py --list other")
    print("- py tools\\check_docs_staging.py --list local-output-removals")
    print_next_actions(build_staging_plan_steps(report))


def build_stage_command_steps(report: StagingReport) -> list[str]:
    commands: list[str] = []
    if report.mixed_paths:
        commands.append("py tools\\check_docs_staging.py --mixed-plan")
        commands.append("py tools\\check_docs_staging.py --mixed-matrix")
        commands.append("py tools\\check_docs_staging.py --mixed-resolution-check")
        commands.append("py tools\\check_docs_staging.py --mixed-boundary-check")
        commands.append("py tools\\check_docs_staging.py --submit-boundary-check")
        commands.append("py tools\\check_docs_staging.py --mixed-closure-audit")
        if report.version_output_paths:
            commands.append("py tools\\check_docs_staging.py --version-output-contract")
        commands.append("py tools\\check_docs_staging.py --mixed-resolution-plan")
        commands.append("py tools\\check_docs_staging.py --mixed-details")
    commands.append("py tools\\check_docs_staging.py --remaining-checklist")
    commands.append("py tools\\check_docs_staging.py --write-local-audit")
    if report.firmware_or_build_paths:
        commands.append("py tools\\check_docs_staging.py --list firmware-or-build")
    if report.release_record_paths:
        commands.append("py tools\\check_docs_staging.py --list release-records")
    if report.version_output_paths:
        commands.append("py tools\\check_docs_staging.py --list version-outputs")
    if report.other_paths:
        commands.append("py tools\\check_docs_staging.py --list other")
    commands.extend(
        [
            "New-Item -ItemType Directory -Force tmp | Out-Null",
            "py tools\\check_docs_staging.py --list migration-add > tmp\\docs-migration-add.paths",
            "git add --pathspec-from-file=tmp\\docs-migration-add.paths",
        ]
    )
    commands.append("py tools\\check_docs_staging.py --summary")
    return commands


def print_stage_commands(report: StagingReport) -> None:
    submit_paths = paths_for_list_category(report, "migration-submit")
    add_paths = paths_for_list_category(report, "migration-add")

    print("文档迁移暂存命令模板：")
    print("- 本命令只打印模板，不执行 git add、不修改暂存区。")
    print(f"- 迁移提交候选：{len(submit_paths)}")
    print(f"- 可执行 git add 清单：{len(add_paths)}")
    print("- 审计总清单：py tools\\check_docs_staging.py --list migration-submit")
    print("- 可暂存清单：py tools\\check_docs_staging.py --list migration-add")
    if report.mixed_paths:
        print(
            f"- 注意：先收口 {len(report.mixed_paths)} 个 AM/MM 混合文件"
            f"（治理文档 {mixed_docs_governance_count(report)} 个，"
            f"固件相关混合文档需分离判断 {mixed_firmware_docs_count(report)} 个），"
            "再执行迁移暂存模板。"
        )
    if report.firmware_or_build_paths or report.release_record_paths or report.version_output_paths or report.other_paths:
        print(
            f"- 注意：另有 "
            f"{len(report.firmware_or_build_paths) + len(report.release_record_paths) + len(report.version_output_paths) + len(report.other_paths)} "
            "个非迁移候选路径需要单独判断，不包含在模板内。"
        )
    print("可复制模板：")
    for command in build_stage_command_steps(report):
        print(command)


def print_decision_checklist(report: StagingReport) -> None:
    submit_paths = paths_for_list_category(report, "migration-submit")
    review_paths = report.review_paths()
    excluded_paths = merge_sorted_paths(report.local_output_paths, report.staged_local_output_removals)

    print("文档迁移提交决策清单：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print("- 适用场景：只服务本次 docs/ 迁移收口，不会阻塞后续 skill 生成版本文档。")
    print("范围概览：")
    print(f"- 迁移提交候选：{len(submit_paths)} 个")
    print(f"- 混合治理文档：{mixed_docs_governance_count(report)} 个")
    print(f"- 混合固件文档：{mixed_firmware_docs_count(report)} 个")
    print(f"- 需要单独判断：{len(review_paths)} 个")
    print(f"- 不应纳入正式文档提交：{len(excluded_paths)} 个")
    print(f"- 本地输出清理记录：{len(report.staged_local_output_removals)} 个")
    print("提交前决策：")
    if report.mixed_paths:
        print(
            f"1. 先收口 AM/MM 混合文件：{len(report.mixed_paths)} 个"
            f"（当前阻塞分组 {count_blocking_categories(report)} 组）。"
        )
        print(
            "2. 用 `--list mixed-docs-governance` 和 `--list mixed-firmware-docs` "
            f"区分治理文档 {mixed_docs_governance_count(report)} 个可随迁移，"
            f"固件相关文档 {mixed_firmware_docs_count(report)} 个需分离判断。"
        )
        print("3. 再决定迁移提交候选是否整体纳入同一次文档迁移提交。")
    else:
        print("1. 当前没有 AM/MM 混合文件。")
        print("2. 决定迁移提交候选是否整体纳入同一次文档迁移提交。")
    print("4. 固件/构建路径、版本/发布记录路径、后续版本文档输出区和其他路径单独判断，不默认混入纯文档迁移。")
    print("5. docs-site/site/tmp 仅作本地输出或预览，不作为正式文档源。")
    print("建议验证：")
    print("- py -B tools\\check_docs.py")
    print("- py -B tools\\check_docs_staging.py --closure-plan")
    print("- py -B tools\\check_docs_staging.py --final-check")
    print("- py -B tools\\check_docs_staging.py --gate-matrix")
    print("- py -B tools\\check_docs_staging.py --submit-plan")
    print("- py -B tools\\check_docs_staging.py --remaining-checklist")
    print("- py -B tools\\check_docs_staging.py --mixed-resolution-check")
    print("- py -B tools\\check_docs_staging.py --mixed-boundary-check")
    print("- py -B tools\\check_docs_staging.py --submit-boundary-check")
    print("- py -B tools\\check_docs_staging.py --mixed-closure-audit")
    print("- py -B tools\\check_docs_staging.py --version-output-contract")
    print("- py -B tools\\check_docs_staging.py --mixed-resolution-plan")
    print("- py -B tools\\check_docs_staging.py --write-local-audit")
    print("- py -B tools\\check_docs_staging.py --summary")
    print("- npm run check（工作目录：docs-site）")


def print_submit_scope_recommendations(recommendations: list[SubmitScopeRecommendation]) -> None:
    if not recommendations:
        return
    print("提交范围建议：")
    for recommendation in recommendations:
        print_audit_paths(recommendation.title, recommendation.paths)


def print_submit_plan(report: StagingReport) -> None:
    submit_count = len(paths_for_list_category(report, "migration-submit"))
    firmware_count = len(report.firmware_or_build_paths)
    release_count = len(report.release_record_paths)
    version_output_count = len(report.version_output_paths)
    local_cleanup_count = len(report.staged_local_output_removals)
    governance_count = mixed_docs_governance_count(report)
    mixed_firmware_count = mixed_firmware_docs_count(report)

    print("文档迁移提交拆分计划：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print("建议拆分批次：")
    print(f"1. 文档迁移提交候选：{submit_count} 个")
    print("   清单：py tools\\check_docs_staging.py --list migration-submit")
    print(f"2. 固件或构建相关：{firmware_count} 个")
    print("   清单：py tools\\check_docs_staging.py --list firmware-or-build")
    print(f"3. 版本或发布记录：{release_count} 个")
    print("   清单：py tools\\check_docs_staging.py --list release-records")
    print(f"4. 后续版本文档输出区：{version_output_count} 个")
    print("   清单：py tools\\check_docs_staging.py --list version-outputs")
    print(f"5. 本地输出清理记录：{local_cleanup_count} 个")
    print("   清单：py tools\\check_docs_staging.py --list local-output-removals")
    print(f"6. 混合文件收口：治理文档 {governance_count} 个，固件相关文档 {mixed_firmware_count} 个")
    print("   分组：py tools\\check_docs_staging.py --mixed-plan")
    print("   矩阵：py tools\\check_docs_staging.py --mixed-matrix")
    print("   预检：py tools\\check_docs_staging.py --mixed-resolution-check")
    print("   边界：py tools\\check_docs_staging.py --mixed-boundary-check")
    print("   候选：py tools\\check_docs_staging.py --submit-boundary-check")
    print("   模板：py tools\\check_docs_staging.py --mixed-resolution-plan")
    print("   细节：py tools\\check_docs_staging.py --mixed-details")
    print("可复制暂存模板：")
    print("- py tools\\check_docs_staging.py --stage-commands")


def print_gate_matrix(report: StagingReport) -> None:
    submit_count = len(paths_for_list_category(report, "migration-submit"))
    review_count = report.review_count()
    excluded_count = len(report.local_output_paths) + len(report.staged_local_output_removals)
    version_output_count = len(report.version_output_paths)

    print("文档迁移提交门禁矩阵：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print("- 适用场景：只服务本次 docs/ 迁移收口，不影响版本提交流程生成新文档。")
    print("门禁项：")
    print(
        f"1. 必须先收口：{len(report.mixed_paths)} 个 AM/MM 混合文件"
        f"（治理文档 {mixed_docs_governance_count(report)} 个，"
        f"固件相关文档 {mixed_firmware_docs_count(report)} 个）；"
        f"旧 CPU docs 非纯删除状态 {len(report.legacy_doc_paths)} 个；"
        f"本地输出非纯删除状态 {len(report.local_output_paths)} 个；"
        f"旧静态站脚本非纯删除状态 {len(report.legacy_site_script_paths)} 个。"
    )
    print("   清单：py tools\\check_docs_staging.py --mixed-plan")
    print("   矩阵：py tools\\check_docs_staging.py --mixed-matrix")
    print("   预检：py tools\\check_docs_staging.py --mixed-resolution-check")
    print("   边界：py tools\\check_docs_staging.py --mixed-boundary-check")
    print("   候选：py tools\\check_docs_staging.py --submit-boundary-check")
    print("   模板：py tools\\check_docs_staging.py --mixed-resolution-plan")
    print("   细节：py tools\\check_docs_staging.py --mixed-details")
    print(f"2. 可随文档迁移提交：{submit_count} 个迁移候选。")
    print("   清单：py tools\\check_docs_staging.py --list migration-submit")
    print(
        f"3. 必须单独判断：{review_count} 个"
        f"（固件/构建 {len(report.firmware_or_build_paths)} 个，"
        f"版本/发布记录 {len(report.release_record_paths)} 个，"
        f"后续版本文档输出 {len(report.version_output_paths)} 个，"
        f"其他 {len(report.other_paths)} 个）。"
    )
    print(
        "   清单：py tools\\check_docs_staging.py --list firmware-or-build / "
        "--list release-records / --list version-outputs / --list other"
    )
    print(f"4. 不作为正式文档源：{excluded_count} 个。")
    print("   清单：py tools\\check_docs_staging.py --list local-output-removals")
    print(
        "5. 后续版本文档输出区："
        f"{version_output_count} 个当前改动；路径 docs/00_构建与版本/版本改动与测试/。"
    )
    print("   规则：skill 后续生成新版本方案可继续写入该目录，人工索引允许后补，不需要先运行本迁移门禁。")
    print("门禁结论：")
    if report.mixed_paths or report.legacy_doc_paths or report.local_output_paths or report.legacy_site_script_paths:
        print("- 迁移提交前仍需收口阻塞项；不要直接把当前候选当成最终提交范围。")
    else:
        print("- 当前未发现迁移提交阻塞项；仍需人工确认单独判断范围。")
    if report.review_paths():
        print("- 固件/构建、版本/发布记录、后续版本文档输出区和其他路径不默认混入纯文档迁移。")
    print("- 本门禁不移动、不生成、不删除文档，不影响版本提交流程生成新文档。")


def print_final_check(report: StagingReport) -> None:
    submit_count = len(paths_for_list_category(report, "migration-submit"))
    review_count = report.review_count()

    print("文档迁移最终收口状态：")
    print("- 本命令只读 Git 状态，不执行 git add、不生成路径文件、不修改暂存区。")
    print("- 适用场景：只服务本次 docs/ 迁移收口，不会阻塞后续 skill 生成版本文档。")
    if report.has_blocking_findings():
        print("当前结论：尚不能进入最终暂存。")
    else:
        print("当前结论：可以进入最终暂存前人工确认。")
    print("状态概览：")
    print(f"- 阻塞项：{count_blocking_categories(report)} 项")
    print(f"- AM/MM 混合文件：{len(report.mixed_paths)}")
    print(f"- 混合治理文档：{mixed_docs_governance_count(report)}")
    print(f"- 混合固件文档：{mixed_firmware_docs_count(report)}")
    print(f"- 未跟踪工具脚本：{len(report.untracked_tool_paths)}")
    print(f"- 迁移提交候选：{submit_count}")
    print(f"- 必须单独判断：{review_count}")
    print(f"- 本地输出清理记录：{len(report.staged_local_output_removals)}")
    print("下一步命令：")
    print("- py tools\\check_docs_staging.py --closure-plan")
    print("- py tools\\check_docs_staging.py --remaining-checklist")
    if report.mixed_paths:
        print("- py tools\\check_docs_staging.py --mixed-resolution-check")
        print("- py tools\\check_docs_staging.py --mixed-boundary-check")
        print("- py tools\\check_docs_staging.py --submit-boundary-check")
        print("- py tools\\check_docs_staging.py --mixed-closure-audit")
        if report.version_output_paths:
            print("- py tools\\check_docs_staging.py --version-output-contract")
        print("- py tools\\check_docs_staging.py --mixed-resolution-plan")
        print("- py tools\\check_docs_staging.py --mixed-details")
    print("- py tools\\check_docs_staging.py --write-local-audit")
    print("- py tools\\check_docs_staging.py --stage-commands")
    print("- py tools\\check_docs_staging.py --summary")
    print("- py -B tools\\check_docs.py")
    print("- npm run check（工作目录：docs-site）")


def append_markdown_path_sample(lines: list[str], paths: list[str]) -> None:
    if not paths:
        lines.append("- 无")
        return
    for path in paths[:AUDIT_SAMPLE_LIMIT]:
        lines.append(f"- `{path}`")
    remaining = len(paths) - AUDIT_SAMPLE_LIMIT
    if remaining > 0:
        lines.append(f"- ... 另有 {remaining} 项")


def build_local_audit_markdown(report: StagingReport) -> str:
    boundary = build_submit_boundary_check(report)
    contract = build_version_output_contract(report)
    items = build_remaining_work_items(report)
    blocking_items = [item for item in items if item.blocking]
    decision_items = [item for item in items if not item.blocking]
    submit_paths = paths_for_list_category(report, "migration-submit")
    review_paths = report.review_paths()
    status = "尚不能进入最终暂存" if report.has_blocking_findings() else "可以进入最终暂存前人工确认"

    lines = [
        "# CUBE 文档迁移本地审计报告",
        "",
        "## 报告边界",
        "",
        "- 本报告只写入 `tmp/docs-migration-audit.md`，属于本地输出，不作为正式文档源。",
        "- 不执行 `git add`，不生成路径文件，不修改暂存区。",
        "- 不写入 `docs/00_构建与版本/版本改动与测试/`，不影响后续 skill 继续生成版本文档。",
        "- 不修改 `C:\\Users\\admin\\.codex\\skills`。",
        "",
        "## 当前状态",
        "",
        f"- 当前结论：{status}。",
        f"- 阻塞收口分组：{len(blocking_items)}",
        f"- 人工决策分组：{len(decision_items)}",
        f"- AM/MM 混合文件：{len(report.mixed_paths)}",
        f"- 治理文档：{mixed_docs_governance_count(report)}",
        f"- 固件相关文档：{mixed_firmware_docs_count(report)}",
        f"- 迁移提交候选：{len(submit_paths)}",
        f"- 必须单独判断：{len(review_paths)}",
        f"- 本地输出清理记录：{len(report.staged_local_output_removals)}",
        f"- 后续版本文档输出区：{len(report.version_output_paths)}",
        f"- 候选边界：{'清晰' if boundary.is_clean() else '需要修正'}",
        f"- 契约状态：{'清晰' if contract.is_clean() else '需要修正'}",
        "",
        "## 阻塞收口项",
        "",
    ]

    if not blocking_items:
        lines.append("- 无")
    for item in blocking_items:
        lines.extend(
            [
                f"### {item.title}",
                "",
                f"- 数量：{len(item.paths)}",
                f"- 动作：{item.action}",
                "",
            ]
        )
        append_markdown_path_sample(lines, item.paths)
        lines.append("")

    lines.extend(
        [
            "## 人工决策项",
            "",
        ]
    )
    if not decision_items:
        lines.append("- 无")
    for item in decision_items:
        lines.extend(
            [
                f"### {item.title}",
                "",
                f"- 数量：{len(item.paths)}",
                f"- 动作：{item.action}",
                "",
            ]
        )
        append_markdown_path_sample(lines, item.paths)
        lines.append("")

    lines.extend(
        [
            "## 下一步顺序",
            "",
            "1. `py tools\\check_docs_staging.py --mixed-closure-audit`",
            "2. `py tools\\check_docs_staging.py --version-output-contract`",
            "3. `py tools\\check_docs_staging.py --remaining-checklist`",
            "4. `py tools\\check_docs_staging.py --stage-commands`",
            "5. `py -B tools\\check_docs.py`",
            "6. `npm run check`（工作目录：`docs-site`）",
            "",
            "## 提交边界提醒",
            "",
            "- `migration-submit` 只作为文档迁移候选集合，固件/构建、版本/发布记录、后续版本文档输出区和其他路径需要单独判断。",
            "- `docs-site/`、`site/`、`tmp/` 只作为本地输出或历史输出，不作为新的正式文档源。",
            "- 后续 skill 生成的新版本文档仍放在 `docs/00_构建与版本/版本改动与测试/`，随对应版本提交另行判断。",
            "",
        ]
    )
    return "\n".join(lines)


def write_local_audit_report(
    report: StagingReport,
    *,
    output_path: Path = LOCAL_AUDIT_REPORT_PATH,
) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(build_local_audit_markdown(report), encoding=OUTPUT_ENCODING)
    return output_path


def display_path(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def print_local_audit_report_result(path: Path, report: StagingReport) -> None:
    status = "尚不能进入最终暂存" if report.has_blocking_findings() else "可以进入最终暂存前人工确认"
    print("本地审计报告已生成：")
    print(f"- {display_path(path)}")
    print(f"- 当前结论：{status}")
    print("- 报告位于 tmp/，不作为正式文档源，不修改暂存区，不会阻塞后续 skill 生成版本文档。")


def print_next_actions(actions: list[str]) -> None:
    if not actions:
        return
    print("建议处理顺序：")
    for index, action in enumerate(actions, start=1):
        print(f"{index}. {action}")


def count_blocking_categories(report: StagingReport) -> int:
    return sum(1 for item in build_remaining_work_items(report) if item.blocking)


def print_summary(report: StagingReport) -> None:
    print("文档迁移提交前暂存区摘要：")
    print(f"- 阻塞项：{count_blocking_categories(report)} 项")
    print(f"- AM/MM 混合文件：{len(report.mixed_paths)}")
    print(f"- 混合治理文档：{mixed_docs_governance_count(report)}")
    print(f"- 混合固件文档：{mixed_firmware_docs_count(report)}")
    print(f"- 旧 CPU docs 非纯删除状态：{len(report.legacy_doc_paths)}")
    print(f"- 本地输出非纯删除状态：{len(report.local_output_paths)}")
    print(f"- 旧静态站脚本非纯删除状态：{len(report.legacy_site_script_paths)}")
    print(f"- 未跟踪工具脚本：{len(report.untracked_tool_paths)}")
    print(f"- 文档迁移相关路径：{len(report.docs_migration_paths)}")
    print(f"- 文档检查工具路径：{len(report.docs_check_tool_paths)}")
    print(f"- 迁移提交候选路径：{len(paths_for_list_category(report, 'migration-submit'))}")
    print(f"- 固件或构建相关路径：{len(report.firmware_or_build_paths)}")
    print(f"- 版本或发布记录路径：{len(report.release_record_paths)}")
    print(f"- 后续版本文档输出区路径：{len(report.version_output_paths)}")
    print(f"- 其他路径：{len(report.other_paths)}")
    print(f"- 已暂存旧 CPU docs 删除记录：{len(report.staged_legacy_doc_removals)}")
    print(f"- 已暂存本地输出清理记录：{len(report.staged_local_output_removals)}")
    print(f"- 已暂存旧静态站脚本删除记录：{len(report.staged_legacy_site_script_removals)}")
    print_next_actions(build_next_actions(report))


def print_scope_audit(report: StagingReport) -> None:
    submit_paths = paths_for_list_category(report, "migration-submit")
    review_paths = report.review_paths()
    excluded_paths = merge_sorted_paths(report.local_output_paths, report.staged_local_output_removals)

    print("文档迁移提交范围审计：")
    print(f"- 建议纳入迁移提交：{len(submit_paths)}")
    print(f"- 需要单独判断：{len(review_paths)}")
    print(f"- 不应纳入正式文档提交：{len(excluded_paths)}")
    print(f"- 当前阻塞项：{count_blocking_categories(report)}")
    print(f"- AM/MM 混合文件：{len(report.mixed_paths)}")
    print(f"- 混合治理文档：{mixed_docs_governance_count(report)}")
    print(f"- 混合固件文档：{mixed_firmware_docs_count(report)}")
    print(f"- 未跟踪工具脚本：{len(report.untracked_tool_paths)}")
    print("路径清单命令：")
    print("- py tools\\check_docs_staging.py --list migration-submit")
    print("- py tools\\check_docs_staging.py --list firmware-or-build")
    print("- py tools\\check_docs_staging.py --list release-records")
    print("- py tools\\check_docs_staging.py --list version-outputs")
    print("- py tools\\check_docs_staging.py --list other")
    print("- py tools\\check_docs_staging.py --list mixed")
    print("- py tools\\check_docs_staging.py --list mixed-docs-governance")
    print("- py tools\\check_docs_staging.py --list mixed-firmware-docs")
    print_next_actions(build_next_actions(report))


def paths_for_list_category(report: StagingReport, category: str) -> list[str]:
    if category == "migration-submit":
        return merge_sorted_paths(
            migration_add_paths(report),
            report.staged_migration_removals,
            report.staged_legacy_doc_removals,
            report.staged_legacy_site_script_removals,
        )
    if category == "migration-add":
        return migration_add_paths(report)
    if category == "docs-migration":
        return report.docs_migration_paths
    if category == "docs-check-tools":
        return report.docs_check_tool_paths
    if category == "firmware-or-build":
        return report.firmware_or_build_paths
    if category == "release-records":
        return report.release_record_paths
    if category == "version-outputs":
        return report.version_output_paths
    if category == "other":
        return report.other_paths
    if category == "migration-removals":
        return merge_sorted_paths(report.staged_legacy_doc_removals, report.staged_legacy_site_script_removals)
    if category == "local-output-removals":
        return merge_sorted_paths(report.local_output_paths, report.staged_local_output_removals)
    if category == "mixed":
        return report.mixed_paths
    if category == "mixed-docs-governance":
        return sorted(path for path in report.mixed_paths if path not in MIXED_FIRMWARE_DOC_PATHS)
    if category == "mixed-firmware-docs":
        return sorted(path for path in report.mixed_paths if path in MIXED_FIRMWARE_DOC_PATHS)
    raise ValueError(f"未知路径分类：{category}")


def mixed_docs_governance_count(report: StagingReport) -> int:
    return len(paths_for_list_category(report, "mixed-docs-governance"))


def mixed_firmware_docs_count(report: StagingReport) -> int:
    return len(paths_for_list_category(report, "mixed-firmware-docs"))


def print_path_list(paths: list[str]) -> None:
    for path in paths:
        print(path)


def exit_code_for_list_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_audit_mode(report: StagingReport) -> int:
    return 1 if report.has_blocking_findings() else 0


def exit_code_for_mixed_details_mode(report: StagingReport) -> int:
    return 1 if report.mixed_paths else 0


def exit_code_for_staging_plan_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_stage_commands_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_mixed_commands_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_mixed_plan_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_mixed_matrix_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_mixed_resolution_plan_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_mixed_resolution_check_mode(report: StagingReport) -> int:
    return 0 if build_mixed_resolution_coverage(report).is_complete() else 1


def exit_code_for_mixed_boundary_check_mode(report: StagingReport) -> int:
    return 0 if build_mixed_submit_boundary(report).is_clean() else 1


def exit_code_for_mixed_closure_audit_mode(report: StagingReport) -> int:
    return 0 if (
        not report.mixed_paths
        and build_mixed_resolution_coverage(report).is_complete()
        and build_mixed_submit_boundary(report).is_clean()
        and build_submit_boundary_check(report).is_clean()
        and build_version_output_contract(report).is_clean()
    ) else 1


def exit_code_for_submit_boundary_check_mode(report: StagingReport) -> int:
    return 0 if build_submit_boundary_check(report).is_clean() else 1


def exit_code_for_remaining_checklist_mode(report: StagingReport) -> int:
    return 1 if build_remaining_work_items(report) else 0


def exit_code_for_closure_plan_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_decision_checklist_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_submit_plan_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_gate_matrix_mode(report: StagingReport) -> int:
    return 0


def exit_code_for_final_check_mode(report: StagingReport) -> int:
    return 1 if report.has_blocking_findings() else 0


def exit_code_for_version_output_contract_mode(report: StagingReport) -> int:
    return 0 if build_version_output_contract(report).is_clean() else 1


def exit_code_for_write_local_audit_mode(report: StagingReport) -> int:
    return 0


def main() -> int:
    configure_output()
    args = parse_args()
    report = build_staging_report(parse_porcelain_v1_status(git_status_lines()))

    if args.list_category:
        print_path_list(paths_for_list_category(report, args.list_category))
        return exit_code_for_list_mode(report)

    if args.audit:
        print_scope_audit(report)
        return exit_code_for_audit_mode(report)

    if args.mixed_details:
        print_mixed_details_report(
            report,
            staged_numstat=git_numstat(staged=True),
            unstaged_numstat=git_numstat(staged=False),
        )
        return exit_code_for_mixed_details_mode(report)

    if args.mixed_commands:
        print_mixed_commands(report)
        return exit_code_for_mixed_commands_mode(report)

    if args.mixed_plan:
        print_mixed_plan(report)
        return exit_code_for_mixed_plan_mode(report)

    if args.mixed_matrix:
        print_mixed_matrix(
            report,
            staged_numstat=git_numstat(staged=True),
            unstaged_numstat=git_numstat(staged=False),
        )
        return exit_code_for_mixed_matrix_mode(report)

    if args.mixed_resolution_plan:
        print_mixed_resolution_plan(report)
        return exit_code_for_mixed_resolution_plan_mode(report)

    if args.mixed_resolution_check:
        print_mixed_resolution_check(report)
        return exit_code_for_mixed_resolution_check_mode(report)

    if args.mixed_boundary_check:
        print_mixed_boundary_check(report)
        return exit_code_for_mixed_boundary_check_mode(report)

    if args.mixed_closure_audit:
        print_mixed_closure_audit(report)
        return exit_code_for_mixed_closure_audit_mode(report)

    if args.submit_boundary_check:
        print_submit_boundary_check(report)
        return exit_code_for_submit_boundary_check_mode(report)

    if args.remaining_checklist:
        print_remaining_checklist(report)
        return exit_code_for_remaining_checklist_mode(report)

    if args.closure_plan:
        print_closure_plan(report)
        return exit_code_for_closure_plan_mode(report)

    if args.staging_plan:
        print_staging_plan(report)
        return exit_code_for_staging_plan_mode(report)

    if args.stage_commands:
        print_stage_commands(report)
        return exit_code_for_stage_commands_mode(report)

    if args.decision_checklist:
        print_decision_checklist(report)
        return exit_code_for_decision_checklist_mode(report)

    if args.submit_plan:
        print_submit_plan(report)
        return exit_code_for_submit_plan_mode(report)

    if args.gate_matrix:
        print_gate_matrix(report)
        return exit_code_for_gate_matrix_mode(report)

    if args.final_check:
        print_final_check(report)
        return exit_code_for_final_check_mode(report)

    if args.version_output_contract:
        print_version_output_contract(report)
        return exit_code_for_version_output_contract_mode(report)

    if args.write_local_audit:
        written_path = write_local_audit_report(report)
        print_local_audit_report_result(written_path, report)
        return exit_code_for_write_local_audit_mode(report)

    if args.summary:
        print_summary(report)
        return 1 if report.has_blocking_findings() else 0

    if report.has_blocking_findings():
        print("文档迁移提交前暂存区一致性检查提示：")
        print_next_actions(build_next_actions(report))
        print_paths("同一文件同时存在已暂存和未暂存改动，提交前需要重新核对：", report.mixed_paths)
        print_paths("旧 CPU docs 仍有未暂存或非删除变更，需要按最终提交范围处理：", report.legacy_doc_paths)
        print_paths("本地输出或临时目录仍有未暂存或非删除变更，不应进入正式提交：", report.local_output_paths)
        print_paths("旧静态站脚本仍有未暂存或非删除变更，且不得恢复脚本：", report.legacy_site_script_paths)
        print_mixed_path_details(
            build_mixed_path_details(
                report.mixed_paths,
                staged_numstat=git_numstat(staged=True),
                unstaged_numstat=git_numstat(staged=False),
            )
        )
        if report.untracked_tool_paths:
            print_paths("未跟踪工具脚本，若属于本次检查能力需要显式纳入提交范围：", report.untracked_tool_paths)
        print_informational_paths("已暂存的旧 CPU docs 删除记录：", report.staged_legacy_doc_removals)
        print_informational_paths("已暂存的本地输出清理记录：", report.staged_local_output_removals)
        print_informational_paths("已暂存的旧静态站脚本删除记录：", report.staged_legacy_site_script_removals)
        print_submit_scope_recommendations(build_submit_scope_recommendations(report))
        print_audit_paths("提交范围审计：文档迁移相关路径", report.docs_migration_paths)
        print_audit_paths("提交范围审计：文档检查工具路径", report.docs_check_tool_paths)
        print_audit_paths("提交范围审计：固件或构建相关路径", report.firmware_or_build_paths)
        print_audit_paths("提交范围审计：其他路径", report.other_paths)
        return 1

    print("文档迁移提交前暂存区一致性检查通过。")
    print_next_actions(build_next_actions(report))
    if report.untracked_tool_paths:
        print_paths("提示：以下未跟踪工具脚本需要按本次提交范围决定是否纳入：", report.untracked_tool_paths)
    print_informational_paths("已暂存的旧 CPU docs 删除记录：", report.staged_legacy_doc_removals)
    print_informational_paths("已暂存的本地输出清理记录：", report.staged_local_output_removals)
    print_informational_paths("已暂存的旧静态站脚本删除记录：", report.staged_legacy_site_script_removals)
    print_submit_scope_recommendations(build_submit_scope_recommendations(report))
    print_audit_paths("提交范围审计：文档迁移相关路径", report.docs_migration_paths)
    print_audit_paths("提交范围审计：文档检查工具路径", report.docs_check_tool_paths)
    print_audit_paths("提交范围审计：固件或构建相关路径", report.firmware_or_build_paths)
    print_audit_paths("提交范围审计：其他路径", report.other_paths)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

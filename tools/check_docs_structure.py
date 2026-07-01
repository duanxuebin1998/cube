#!/usr/bin/env python3
"""检查 CUBE 文档库结构边界，不生成或修改任何文档。"""

from __future__ import annotations

import subprocess
import sys
from collections import Counter
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
OUTPUT_ENCODING = "utf-8"

EXPECTED_TOP_DIRS = {
    "00_程序流程导航",
    "00_构建与版本",
    "01_协议与寄存器",
    "02_需求与计划",
    "03_问题分析与整改",
    "04_界面与菜单",
    "05_测试记录",
    "06_SIL功能安全认证",
    "07_硬件原理图",
}

VERSION_OUTPUT_DIR = "docs/00_构建与版本/版本改动与测试"
VERSION_INDEX_PATHS = (
    "docs/00_构建与版本/版本总览.md",
    f"{VERSION_OUTPUT_DIR}/README.md",
)

REQUIRED_PATHS = (
    "AGENTS.md",
    "docs/README.md",
    "docs/00_程序流程导航/README.md",
    "docs/00_构建与版本/README.md",
    f"{VERSION_OUTPUT_DIR}/README.md",
    "docs/00_构建与版本/文档库使用指南.md",
    "docs/00_构建与版本/新增资料落位检查表.md",
    "docs/00_构建与版本/文档迁移提交前收尾清单.md",
    "docs/00_构建与版本/文档库迁移映射表.md",
    "docs/00_构建与版本/附件与源文件清单.md",
    "docs/01_协议与寄存器/README.md",
    "docs/02_需求与计划/README.md",
    "docs/03_问题分析与整改/README.md",
    "docs/04_界面与菜单/README.md",
    "docs/05_测试记录/README.md",
    "docs/06_SIL功能安全认证/README.md",
    "docs/07_硬件原理图/README.md",
)

LEGACY_DOC_DIRS = (
    "LTD_MAIN_CPU2/docs",
    "LTD_DISPLAY_CPU3/docs",
    "site",
    "tools/build_static_docs_site.py",
    "tools/check_static_docs_site_ui.py",
    "tools/ltd-report-theme.css",
)

LOCAL_ONLY_DIRS = (
    "docs-site",
    "site",
    "tmp",
)

NOT_TRACKED_PREFIXES = (
    "LTD_MAIN_CPU2/docs",
    "LTD_DISPLAY_CPU3/docs",
    "docs-site",
    "site",
    "tmp",
    "tools/build_static_docs_site.py",
    "tools/check_static_docs_site_ui.py",
    "tools/ltd-report-theme.css",
)

README_WARN_LINE_LIMIT = 60

AGENTS_REQUIRED_PHRASES = (
    "正式文档源只维护 `docs/`",
    "不要新增、恢复或继续维护 `LTD_MAIN_CPU2/docs/`、`LTD_DISPLAY_CPU3/docs/`",
    "`docs-site/` 仅作为本机预览工具",
    "旧 `site/` 已淘汰",
    "版本改动与测试/",
    "不修改 `C:\\Users\\admin\\.codex\\skills`",
)

LEGACY_REFERENCE_KEYWORDS = (
    "LTD_MAIN_CPU2/docs",
    "LTD_DISPLAY_CPU3/docs",
    "/html/cpu2",
    "/html/cpu3",
    "8128",
    "build_static_docs_site",
    "check_static_docs_site_ui",
    "ltd-report-theme",
)

LEGACY_REFERENCE_ALLOWED_FILES = {
    "00_构建与版本/CPU2文档索引.md",
    "00_构建与版本/CPU3文档索引.md",
    "00_构建与版本/文档库使用指南.md",
    "00_构建与版本/文档库整理候选清单.md",
    "00_构建与版本/文档库治理清单.md",
    "00_构建与版本/文档库盘点报告.md",
    "00_构建与版本/文档库迁移映射表.md",
    "00_构建与版本/文档迁移提交前收尾清单.md",
    "00_构建与版本/新增资料落位检查表.md",
}

GOVERNANCE_ENTRY_LINKS = {
    "docs/README.md": (
        "00_构建与版本/新增资料落位检查表.md",
        "00_构建与版本/版本改动与测试/",
    ),
    "docs/00_构建与版本/README.md": (
        "新增资料落位检查表.md",
        "版本改动与测试/",
    ),
    "docs/00_构建与版本/文档库使用指南.md": (
        "新增资料落位检查表.md",
        "版本改动与测试/",
    ),
    "docs/00_构建与版本/文档库治理清单.md": (
        "新增资料落位检查表.md",
        "版本改动与测试/",
    ),
}

NEW_MATERIAL_CHECKLIST_PATH = "docs/00_构建与版本/新增资料落位检查表.md"

NEW_MATERIAL_REQUIRED_RULES = (
    (
        "版本发布仍按既有 skill 规则新增 `docs/00_构建与版本/版本改动与测试/` 文档",
        ("docs/00_构建与版本/版本改动与测试/", "skill"),
    ),
    (
        "CPU2/CPU3 程序流程",
        ("CPU2/CPU3 程序流程", "docs/00_程序流程导航/"),
    ),
    (
        "CPU2/CPU3 共享协议、寄存器、DSM、SI",
        ("CPU2/CPU3 共享协议", "docs/01_协议与寄存器/"),
    ),
    (
        "问题现象、原因分析、整改方案、风险复查",
        ("问题现象", "docs/03_问题分析与整改/"),
    ),
    (
        "SIL、MISRA/CERT、功能安全外部资料",
        ("SIL", "docs/06_SIL功能安全认证/"),
    ),
    (
        "硬件原理图、芯片手册、固件硬件对照",
        ("硬件原理图", "芯片手册", "docs/07_硬件原理图/"),
    ),
    (
        "不因为新增资料而手工维护 `docs-site/src/content/docs/`",
        ("docs-site/src/content/docs/",),
    ),
    (
        "不把 `tmp/` 里的渲染、截图、抽取文本或 PDF 处理缓存当作正式资料提交",
        ("tmp/", "PDF 处理缓存", "正式资料"),
    ),
)

STAGING_GOVERNANCE_COMMAND_REQUIREMENTS = {
    "docs/00_构建与版本/文档迁移提交前收尾清单.md": (
        "--mixed-closure-audit",
        "--version-output-contract",
        "--write-local-audit",
    ),
    "docs/00_构建与版本/文档库治理清单.md": (
        "--mixed-closure-audit",
        "--version-output-contract",
        "--write-local-audit",
    ),
}

STAGING_GOVERNANCE_LINE_REQUIREMENTS = {
    "docs/00_构建与版本/文档迁移提交前收尾清单.md": {
        "--stage-commands": (
            "--mixed-closure-audit",
            "--version-output-contract",
            "--write-local-audit",
            "migration-add",
        ),
    },
    "docs/00_构建与版本/文档库治理清单.md": {
        "--stage-commands": (
            "--mixed-closure-audit",
            "--version-output-contract",
            "--write-local-audit",
            "migration-add",
        ),
    },
}

FLOW_HTML_DIR = "docs/00_程序流程导航"
HTML_LOCAL_REFERENCE_ATTRS = {"href", "src"}
IGNORED_HTML_REFERENCE_SCHEMES = {"http", "https", "mailto", "tel", "data", "javascript"}


class LocalHtmlReferenceParser(HTMLParser):
    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.references: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self._collect(attrs)

    def handle_startendtag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self._collect(attrs)

    def _collect(self, attrs: list[tuple[str, str | None]]) -> None:
        for name, value in attrs:
            if name.lower() in HTML_LOCAL_REFERENCE_ATTRS and value:
                self.references.append(value)


def configure_output(stdout=sys.stdout, stderr=sys.stderr) -> None:  # noqa: ANN001
    for stream in (stdout, stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding=OUTPUT_ENCODING, errors="replace")


def rel(path: Path) -> str:
    return path.relative_to(ROOT).as_posix()


def run_git(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["git", *args],
        cwd=ROOT,
        text=True,
        encoding=OUTPUT_ENCODING,
        errors="replace",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def git_lines(args: list[str]) -> list[str]:
    result = run_git(args)
    if result.returncode != 0:
        message = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"git {' '.join(args)} failed: {message}")
    return [line.strip().replace("\\", "/") for line in result.stdout.splitlines() if line.strip()]


def is_asset_like(path: Path, base_dir: Path | None = None) -> bool:
    if base_dir is None:
        base_dir = DOCS
    parts = {part.lower() for part in path.relative_to(base_dir).parts}
    return "assets" in parts or "__pycache__" in parts


def check_required_paths(errors: list[str]) -> None:
    if not DOCS.is_dir():
        errors.append("缺少 docs/ 正式文档目录。")
        return

    for relative_path in REQUIRED_PATHS:
        if not (ROOT / relative_path).exists():
            errors.append(f"缺少必要文档入口：{relative_path}")


def check_agents_boundary(errors: list[str]) -> None:
    agents = ROOT / "AGENTS.md"
    if not agents.exists():
        return
    try:
        text = agents.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        errors.append("AGENTS.md 不是 UTF-8。")
        return

    for phrase in AGENTS_REQUIRED_PHRASES:
        if phrase not in text:
            errors.append(f"AGENTS.md 缺少当前文档边界说明：{phrase}")


def check_top_level_layout(errors: list[str]) -> None:
    if not DOCS.is_dir():
        return

    for child in DOCS.iterdir():
        if child.is_dir() and child.name not in EXPECTED_TOP_DIRS:
            errors.append(f"docs/ 下存在未登记的一级目录：{rel(child)}")
        if child.is_file() and child.name != "README.md":
            errors.append(f"docs/ 根目录存在非 README 文件：{rel(child)}")

    missing_dirs = sorted(name for name in EXPECTED_TOP_DIRS if not (DOCS / name).is_dir())
    for name in missing_dirs:
        errors.append(f"缺少必要一级目录：docs/{name}")


def check_readmes(errors: list[str], warnings: list[str]) -> None:
    if not DOCS.is_dir():
        return

    for directory in DOCS.rglob("*"):
        if not directory.is_dir() or is_asset_like(directory):
            continue
        readme = directory / "README.md"
        if not readme.exists():
            errors.append(f"目录缺少 README.md：{rel(directory)}")

    for readme in DOCS.rglob("README.md"):
        try:
            line_count = len(readme.read_text(encoding="utf-8").splitlines())
        except UnicodeDecodeError:
            errors.append(f"README 不是 UTF-8：{rel(readme)}")
            continue
        if line_count > README_WARN_LINE_LIMIT:
            warnings.append(f"README 行数偏长：{rel(readme)}（{line_count} 行）")


def check_legacy_dirs(errors: list[str]) -> None:
    for relative_path in LEGACY_DOC_DIRS:
        path = ROOT / relative_path
        if path.exists():
            errors.append(f"不应恢复旧文档目录、旧站点或旧静态站脚本：{relative_path}")


def check_git_boundaries(errors: list[str]) -> None:
    for relative_path in LOCAL_ONLY_DIRS:
        result = run_git(["check-ignore", "--quiet", f"{relative_path}/.codex-ignore-check"])
        if result.returncode != 0:
            errors.append(f"{relative_path}/ 未被 .gitignore 忽略。")

    tracked = git_lines(["ls-files", "--", *NOT_TRACKED_PREFIXES])
    if tracked:
        errors.append("以下路径不应被 Git 跟踪：" + ", ".join(tracked))


def collect_file_stats() -> tuple[int, int, Counter[str]]:
    files = [path for path in DOCS.rglob("*") if path.is_file()]
    dirs = [path for path in DOCS.rglob("*") if path.is_dir()]
    suffixes = Counter(path.suffix.lower() or "<none>" for path in files)
    return len(files), len(dirs), suffixes


def count_version_output_docs() -> int:
    return len(collect_version_output_docs())


def collect_version_output_docs() -> list[Path]:
    version_dir = ROOT / VERSION_OUTPUT_DIR
    if not version_dir.is_dir():
        return []
    return sorted(path for path in version_dir.glob("*.md") if path.name != "README.md")


def version_output_date(path: Path) -> str | None:
    name = path.name
    if len(name) < 10:
        return None
    date = name[:10]
    if date[4] == "-" and date[7] == "-" and date.replace("-", "").isdigit():
        return date
    return None


def latest_version_output_names(paths: list[Path]) -> list[str]:
    dated_names = [
        (date, path.name)
        for path in paths
        if path.name != "README.md"
        for date in [version_output_date(path)]
        if date is not None
    ]
    if not dated_names:
        return []
    latest_date = max(date for date, _ in dated_names)
    return sorted(name for date, name in dated_names if date == latest_date)


def build_version_index_warnings(latest_names: list[str], index_texts: dict[str, str]) -> list[str]:
    if not latest_names:
        return []

    labels = {
        "docs/00_构建与版本/版本总览.md": "版本总览.md",
        f"{VERSION_OUTPUT_DIR}/README.md": "版本改动与测试/README.md",
    }
    warnings: list[str] = []
    for relative_path, text in index_texts.items():
        missing_names = [name for name in latest_names if name not in text]
        if missing_names:
            label = labels.get(relative_path, relative_path)
            joined_names = "、".join(missing_names)
            warnings.append(f"{label} 未收录最新日期版本方案 {joined_names}（人工索引，可后补，不阻塞版本提交）")
    return warnings


def check_version_index_freshness(warnings: list[str]) -> None:
    latest_names = latest_version_output_names(collect_version_output_docs())
    if not latest_names:
        return

    index_texts: dict[str, str] = {}
    for relative_path in VERSION_INDEX_PATHS:
        path = ROOT / relative_path
        if not path.exists():
            continue
        try:
            index_texts[relative_path] = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            warnings.append(f"人工索引不是 UTF-8，跳过最新版本方案提示：{relative_path}（不阻塞版本提交）")

    warnings.extend(build_version_index_warnings(latest_names, index_texts))


def find_same_stem_doc_exports(base_dir: Path) -> list[tuple[str, list[str]]]:
    grouped: dict[str, set[str]] = {}
    for path in base_dir.rglob("*"):
        if not path.is_file() or is_asset_like(path, base_dir):
            continue
        suffix = path.suffix.lower()
        if suffix not in {".md", ".pdf"}:
            continue
        key = path.relative_to(base_dir).with_suffix("").as_posix()
        grouped.setdefault(key, set()).add(suffix)

    return [
        (key, sorted(suffixes))
        for key, suffixes in sorted(grouped.items())
        if {".md", ".pdf"}.issubset(suffixes)
    ]


def check_same_stem_exports(warnings: list[str]) -> None:
    if not DOCS.is_dir():
        return

    matches = find_same_stem_doc_exports(DOCS)
    for key, _ in matches:
        warnings.append(f"存在同名 Markdown/PDF：docs/{key}.md 与 docs/{key}.pdf（确认权威 Markdown，PDF 不阻塞提交）")


def find_disallowed_legacy_reference_lines(base_dir: Path) -> list[tuple[str, int, str]]:
    problems: list[tuple[str, int, str]] = []
    for path in sorted(base_dir.rglob("*.md")):
        relative_path = path.relative_to(base_dir).as_posix()
        if relative_path in LEGACY_REFERENCE_ALLOWED_FILES:
            continue

        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except UnicodeDecodeError:
            continue

        for line_no, line in enumerate(lines, start=1):
            for keyword in LEGACY_REFERENCE_KEYWORDS:
                if keyword in line:
                    problems.append((relative_path, line_no, keyword))
    return problems


def check_legacy_reference_keywords(errors: list[str]) -> None:
    if not DOCS.is_dir():
        return

    problems = find_disallowed_legacy_reference_lines(DOCS)
    for relative_path, line_no, keyword in problems:
        errors.append(
            f"旧路径或旧站关键词只允许出现在治理/迁移/索引说明中：docs/{relative_path}:{line_no} ({keyword})"
        )


def check_governance_entry_links(errors: list[str]) -> None:
    for relative_path, required_fragments in GOVERNANCE_ENTRY_LINKS.items():
        path = ROOT / relative_path
        if not path.exists():
            continue

        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            errors.append(f"治理入口不是 UTF-8：{relative_path}")
            continue

        for fragment in required_fragments:
            if fragment not in text:
                errors.append(f"治理入口缺少必要引用：{relative_path} -> {fragment}")


def check_new_material_checklist(errors: list[str]) -> None:
    path = ROOT / NEW_MATERIAL_CHECKLIST_PATH
    if not path.exists():
        return

    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        errors.append(f"新增资料落位检查表不是 UTF-8：{NEW_MATERIAL_CHECKLIST_PATH}")
        return

    for rule_name, fragments in NEW_MATERIAL_REQUIRED_RULES:
        if not all(fragment in text for fragment in fragments):
            errors.append(f"新增资料落位检查表缺少关键规则：{rule_name}")


def check_staging_governance_commands(errors: list[str]) -> None:
    for relative_path, commands in STAGING_GOVERNANCE_COMMAND_REQUIREMENTS.items():
        path = ROOT / relative_path
        if not path.exists():
            continue

        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            errors.append(f"文档迁移治理说明不是 UTF-8：{relative_path}")
            continue

        label = path.name
        for command in commands:
            if command not in text:
                errors.append(f"{label} 缺少当前收口命令：{command}")

        line_requirements = STAGING_GOVERNANCE_LINE_REQUIREMENTS.get(relative_path, {})
        lines = text.splitlines()
        for anchor, required_commands in line_requirements.items():
            anchor_lines = [line for line in lines if anchor in line]
            if not anchor_lines:
                errors.append(f"{label} 缺少当前收口命令说明段：{anchor}")
                continue
            for command in required_commands:
                if not any(command in line for line in anchor_lines):
                    errors.append(f"{label} 的 {anchor} 说明缺少当前收口命令：{command}")


def html_reference_target(raw_reference: str) -> str | None:
    reference = raw_reference.strip()
    if not reference or reference.startswith("#"):
        return None

    parsed = urlsplit(reference)
    if parsed.scheme.lower() in IGNORED_HTML_REFERENCE_SCHEMES or parsed.netloc:
        return None
    if not parsed.path or parsed.path.startswith("/") or parsed.path.startswith("\\"):
        return None

    return unquote(parsed.path)


def find_broken_flow_html_local_references(flow_root: Path) -> list[tuple[Path, str]]:
    if not flow_root.is_dir():
        return []

    broken: list[tuple[Path, str]] = []
    for html_path in sorted(flow_root.rglob("*.html")):
        try:
            text = html_path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            broken.append((html_path, "<HTML 文件不是 UTF-8>"))
            continue

        parser = LocalHtmlReferenceParser()
        parser.feed(text)
        for raw_reference in parser.references:
            target = html_reference_target(raw_reference)
            if target is None:
                continue
            if not (html_path.parent / target).exists():
                broken.append((html_path, raw_reference))

    return broken


def check_flow_html_local_references(errors: list[str]) -> None:
    for html_path, raw_reference in find_broken_flow_html_local_references(ROOT / FLOW_HTML_DIR):
        if raw_reference == "<HTML 文件不是 UTF-8>":
            errors.append(f"流程 HTML 不是 UTF-8：{rel(html_path)}")
            continue
        errors.append(f"流程 HTML 引用不存在：{rel(html_path)} -> {raw_reference}")


def print_summary(warnings: list[str]) -> None:
    file_count, dir_count, suffixes = collect_file_stats()
    markdown_count = suffixes.get(".md", 0)
    html_count = suffixes.get(".html", 0)
    archive_count = file_count - markdown_count - html_count
    version_output_count = count_version_output_docs()

    print("CUBE 文档结构检查通过。")
    print(f"- docs 文件：{file_count} 个，目录：{dir_count} 个")
    print(f"- Markdown：{markdown_count} 个，HTML：{html_count} 个，附件/源文件：{archive_count} 个")
    print("- 正式文档源：docs/")
    print(f"- 版本生成输出区：{VERSION_OUTPUT_DIR}/（允许后续追加，当前方案 {version_output_count} 个）")
    print("- 本地预览目录：docs-site/（不跟踪）")
    print("- 临时渲染目录：tmp/（不跟踪）")
    if warnings:
        print("提示：")
        for warning in warnings:
            print(f"- {warning}")


def main() -> int:
    configure_output()
    errors: list[str] = []
    warnings: list[str] = []

    try:
        check_required_paths(errors)
        check_agents_boundary(errors)
        check_top_level_layout(errors)
        check_readmes(errors, warnings)
        check_version_index_freshness(warnings)
        check_same_stem_exports(warnings)
        check_legacy_reference_keywords(errors)
        check_governance_entry_links(errors)
        check_new_material_checklist(errors)
        check_staging_governance_commands(errors)
        check_flow_html_local_references(errors)
        check_legacy_dirs(errors)
        check_git_boundaries(errors)
    except RuntimeError as exc:
        errors.append(str(exc))

    if errors:
        print("CUBE 文档结构检查失败：")
        for error in errors:
            print(f"- {error}")
        return 1

    print_summary(warnings)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

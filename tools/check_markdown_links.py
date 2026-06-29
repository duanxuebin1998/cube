#!/usr/bin/env python3
"""检查 docs/ 内 Markdown 相对链接是否存在。"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path
from urllib.parse import unquote, urlsplit


ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
OUTPUT_ENCODING = "utf-8"

INLINE_LINK_RE = re.compile(r"(?<!!)\[[^\]\n]+\]\(([^)\n]+)\)")
REFERENCE_LINK_RE = re.compile(r"^\s*\[[^\]\n]+\]:\s*(\S+)")
SKIP_SCHEMES = {
    "http",
    "https",
    "mailto",
    "tel",
    "ftp",
    "file",
    "plugin",
}


@dataclass(frozen=True)
class LinkProblem:
    source: str
    line: int
    target: str
    reason: str


def configure_output(stdout=sys.stdout, stderr=sys.stderr) -> None:  # noqa: ANN001
    for stream in (stdout, stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding=OUTPUT_ENCODING, errors="replace")


def to_posix(path: Path, root: Path) -> str:
    return path.relative_to(root).as_posix()


def strip_wrapping(value: str) -> str:
    value = value.strip()
    if len(value) >= 2 and value[0] == "<" and value[-1] == ">":
        return value[1:-1].strip()
    return value


def iter_markdown_links(text: str) -> list[tuple[int, str]]:
    links: list[tuple[int, str]] = []
    in_fence = False

    for line_no, line in enumerate(text.splitlines(), start=1):
        stripped = line.lstrip()
        if stripped.startswith("```") or stripped.startswith("~~~"):
            in_fence = not in_fence
            continue
        if in_fence:
            continue

        for match in INLINE_LINK_RE.finditer(line):
            links.append((line_no, strip_wrapping(match.group(1))))

        reference_match = REFERENCE_LINK_RE.match(line)
        if reference_match:
            links.append((line_no, strip_wrapping(reference_match.group(1))))

    return links


def should_skip(raw_target: str) -> bool:
    if not raw_target:
        return True
    if raw_target.startswith("#"):
        return True
    if raw_target.startswith("/"):
        return True

    split = urlsplit(raw_target)
    if split.scheme.lower() in SKIP_SCHEMES:
        return True
    if split.netloc:
        return True

    return False


def candidate_paths(source: Path, raw_target: str) -> list[Path]:
    split = urlsplit(raw_target)
    target = unquote(split.path).replace("\\", "/")
    if not target:
        return []

    base = (source.parent / target).resolve()
    candidates = [base]
    if base.suffix == "":
        candidates.extend((base.with_suffix(".md"), base / "README.md", base / "index.md"))
    return candidates


def resolve_problem(source: Path, root: Path, raw_target: str) -> LinkProblem | None:
    if should_skip(raw_target):
        return None

    candidates = candidate_paths(source, raw_target)
    if not candidates:
        return None

    if any(candidate.exists() for candidate in candidates):
        return None

    return LinkProblem(
        source=to_posix(source, root),
        line=0,
        target=raw_target,
        reason="missing target",
    )


def read_markdown(path: Path) -> str:
    data = path.read_bytes()
    return data.decode("utf-8-sig")


def check_markdown_links(docs_dir: Path = DOCS, root: Path = ROOT) -> list[LinkProblem]:
    problems: list[LinkProblem] = []

    for source in sorted(docs_dir.rglob("*.md")):
        text = read_markdown(source)
        for line_no, raw_target in iter_markdown_links(text):
            problem = resolve_problem(source, root, raw_target)
            if problem:
                problems.append(
                    LinkProblem(
                        source=problem.source,
                        line=line_no,
                        target=problem.target,
                        reason=problem.reason,
                    )
                )

    return problems


def main() -> int:
    configure_output()
    problems = check_markdown_links(DOCS, ROOT)
    if problems:
        print("Markdown 链接检查失败：")
        for problem in problems:
            print(f"- {problem.source}:{problem.line}: {problem.target} ({problem.reason})")
        return 1

    checked = sum(1 for _path in DOCS.rglob("*.md"))
    print(f"Markdown 链接检查通过：checked {checked} markdown files; missing links: 0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

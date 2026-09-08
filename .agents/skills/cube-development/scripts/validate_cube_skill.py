#!/usr/bin/env python3
"""Validate the repository-scoped CUBE skill without third-party packages."""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path
from urllib.parse import unquote, urlsplit


SKILL_NAME = "cube-development"


def fail(message: str) -> None:
    raise ValueError(message)


def read_utf8(path: Path) -> str:
    data = path.read_bytes()
    if data.startswith(b"\xef\xbb\xbf"):
        fail(f"UTF-8 BOM is not allowed: {path}")
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError as exc:
        fail(f"File is not valid UTF-8: {path}: {exc}")


def parse_frontmatter(text: str) -> dict[str, str]:
    lines = text.splitlines()
    if not lines or lines[0] != "---":
        fail("SKILL.md must start with YAML frontmatter")
    try:
        end = lines.index("---", 1)
    except ValueError:
        fail("SKILL.md frontmatter is not closed")
    fields: dict[str, str] = {}
    for line in lines[1:end]:
        if not line.strip():
            continue
        match = re.fullmatch(r"([A-Za-z0-9_-]+):\s*(.+)", line)
        if not match:
            fail(f"Unsupported frontmatter line: {line}")
        key, value = match.groups()
        if key in fields:
            fail(f"Duplicate frontmatter field: {key}")
        fields[key] = value.strip().strip('"\'')
    if set(fields) != {"name", "description"}:
        fail("Frontmatter must contain only name and description")
    return fields


def validate_agent_yaml(path: Path) -> None:
    text = read_utf8(path)
    if not text.splitlines() or text.splitlines()[0] != "interface:":
        fail("agents/openai.yaml must start with interface")
    values: dict[str, str] = {}
    for key in ("display_name", "short_description", "default_prompt"):
        match = re.search(rf'^  {key}:\s*"(.*)"\s*$', text, re.MULTILINE)
        if not match:
            fail(f"agents/openai.yaml is missing quoted {key}")
        values[key] = match.group(1)
    length = len(values["short_description"])
    if not 25 <= length <= 64:
        fail(f"short_description length must be 25-64 characters, got {length}")
    if f"${SKILL_NAME}" not in values["default_prompt"]:
        fail(f"default_prompt must mention ${SKILL_NAME}")


def validate_description(description: str) -> None:
    if not description.strip() or len(description) > 1024:
        fail("Skill description must be non-empty and at most 1024 characters")


def validate_reference_routes(skill_root: Path) -> set[Path]:
    """Follow explicit reference routes, allowing references to route to each other."""
    skill_root = skill_root.resolve()
    reference_root = skill_root / "references"
    actual = {path.resolve() for path in reference_root.rglob("*.md")}
    pending = [skill_root / "SKILL.md"]
    visited: set[Path] = set()
    while pending:
        source = pending.pop()
        if source in visited:
            continue
        visited.add(source)
        text = read_utf8(source)
        targets = [
            skill_root / value
            for value in re.findall(r"`(references/[^`]+\.md)`", text)
        ]
        for value in re.findall(r"\]\(([^)]+)\)", text):
            link = urlsplit(value.strip("<>"))
            if not link.scheme and not link.netloc and link.path.endswith(".md"):
                targets.append(source.parent / unquote(link.path))
        for target in targets:
            target = target.resolve()
            if not target.is_relative_to(skill_root):
                fail(f"Reference route leaves skill: {source} -> {target}")
            if not target.is_file():
                fail(f"Missing routed reference: {source} -> {target}")
            if target.is_relative_to(reference_root):
                pending.append(target)
    unrouted = sorted(actual - visited)
    if unrouted:
        fail(f"Reference files not reachable from SKILL.md: {unrouted}")
    return actual


def main() -> int:
    skill_root = Path(__file__).resolve().parents[1]
    skill_path = skill_root / "SKILL.md"
    text = read_utf8(skill_path)
    fields = parse_frontmatter(text)
    if fields["name"] != SKILL_NAME or skill_root.name != SKILL_NAME:
        fail("Skill name, folder name and expected CUBE skill name must match")
    validate_description(fields["description"])
    actual_references = validate_reference_routes(skill_root)

    mentioned_scripts = set(re.findall(r"`scripts/([^`\s]+)`", text))
    missing_scripts = sorted(
        name for name in mentioned_scripts if not (skill_root / "scripts" / name).is_file()
    )
    if missing_scripts:
        fail(f"Missing scripts referenced by SKILL.md: {missing_scripts}")

    validate_agent_yaml(skill_root / "agents" / "openai.yaml")

    required_policy_fragments = {
        skill_root / "references" / "internal-protocol-and-storage.md": (
            "CPU3 对外 LTD 标准 Modbus 的共享区是同一份契约",
        ),
        skill_root / "references" / "external-protocols.md": (
            "不得只改板间或对外一侧",
            "CPU3 本机 `0x7000` 高地址段不属于 CPU2/CPU3 共享区",
        ),
        skill_root / "references" / "docs-git-validation.md": (
            "现有或用户人工维护的 PDF",
            "不自动生成、刷新或新增 PDF",
            "不得仅因 PDF 存在同名 Markdown 就自动排除",
        ),
        skill_root / "references" / "version-build-release.md": (
            "CPU3 对外 LTD 标准 Modbus 的共享区直接使用同一契约",
        ),
    }
    for policy_path, fragments in required_policy_fragments.items():
        policy_text = read_utf8(policy_path)
        for fragment in fragments:
            if fragment not in policy_text:
                fail(f"Required CUBE policy fragment is missing from {policy_path}: {fragment}")

    reference_files = sorted(actual_references)
    text_files = [skill_path, skill_root / "agents" / "openai.yaml"]
    text_files.extend(reference_files)
    text_files.extend(sorted((skill_root / "scripts").glob("*.py")))
    text_files.extend(sorted((skill_root / "scripts").glob("*.ps1")))
    text_files.extend(sorted((skill_root / "assets" / "git-hooks").glob("*")))
    for path in text_files:
        file_text = read_utf8(path)
        for number, line in enumerate(file_text.splitlines(), start=1):
            if line.rstrip(" \t") != line:
                fail(f"Trailing whitespace: {path}:{number}")
        if not file_text.endswith(("\n", "\r\n")):
            fail(f"Text file must end with a newline: {path}")

    for reference in reference_files:
        reference_text = read_utf8(reference)
        if len(reference_text.splitlines()) > 100 and "## 目录" not in reference_text:
            fail(f"Reference longer than 100 lines must contain a table of contents: {reference}")

    for script in sorted((skill_root / "scripts").glob("*.py")):
        ast.parse(read_utf8(script), filename=str(script))

    forbidden_files = [
        path.name
        for path in skill_root.iterdir()
        if path.is_file() and path.name.upper() in {"README.MD", "CHANGELOG.MD", "QUICK_REFERENCE.MD"}
    ]
    if forbidden_files:
        fail(f"Extraneous skill files are not allowed: {forbidden_files}")

    print("CUBE_SKILL_VALIDATION=PASS")
    print(f"references={len(actual_references)}")
    print(f"pythonScripts={len(list((skill_root / 'scripts').glob('*.py')))}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except ValueError as exc:
        print(f"CUBE_SKILL_VALIDATION=FAIL\nreason={exc}", file=sys.stderr)
        raise SystemExit(1)

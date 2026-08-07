#!/usr/bin/env python3
"""Validate the repository-scoped CUBE skill without third-party packages."""

from __future__ import annotations

import ast
import re
import sys
from pathlib import Path


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


def main() -> int:
    skill_root = Path(__file__).resolve().parents[1]
    skill_path = skill_root / "SKILL.md"
    text = read_utf8(skill_path)
    fields = parse_frontmatter(text)
    if fields["name"] != SKILL_NAME or skill_root.name != SKILL_NAME:
        fail("Skill name, folder name and expected CUBE skill name must match")
    if len(fields["description"]) < 80:
        fail("Skill description is too short to provide reliable triggering context")

    mentioned_references = set(re.findall(r"`references/([^`]+\.md)`", text))
    actual_references = {path.name for path in (skill_root / "references").glob("*.md")}
    missing_references = sorted(mentioned_references - actual_references)
    unrouted_references = sorted(actual_references - mentioned_references)
    if missing_references:
        fail(f"Missing routed references: {missing_references}")
    if unrouted_references:
        fail(f"Reference files not routed directly from SKILL.md: {unrouted_references}")

    mentioned_scripts = set(re.findall(r"`scripts/([^`\s]+)`", text))
    missing_scripts = sorted(
        name for name in mentioned_scripts if not (skill_root / "scripts" / name).is_file()
    )
    if missing_scripts:
        fail(f"Missing scripts referenced by SKILL.md: {missing_scripts}")

    validate_agent_yaml(skill_root / "agents" / "openai.yaml")

    reference_files = sorted((skill_root / "references").glob("*.md"))
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

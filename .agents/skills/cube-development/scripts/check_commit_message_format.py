#!/usr/bin/env python3
"""Validate the mandatory CUBE commit-message body structure."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


REQUIRED_HEADINGS = (
    "版本：",
    "协议版本/兼容性：",
    "本次修改：",
    "验证：",
)


def read_commit_message(revision: str) -> str:
    result = subprocess.run(
        ["git", "show", "-s", "--format=%B", revision],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="strict",
    )
    return result.stdout


def normalized_lines(message: str) -> list[str]:
    return [
        line.rstrip()
        for line in message.replace("\r\n", "\n").replace("\r", "\n").split("\n")
        if not line.lstrip().startswith("#")
    ]


def validate_message(message: str) -> list[str]:
    lines = normalized_lines(message)
    errors: list[str] = []

    subject_index = next((index for index, line in enumerate(lines) if line.strip()), None)
    if subject_index is None:
        return ["提交信息为空。"]

    subject = lines[subject_index].strip()
    if subject in REQUIRED_HEADINGS:
        errors.append("提交标题缺失。")

    heading_indexes: list[int] = []
    for heading in REQUIRED_HEADINGS:
        matches = [index for index, line in enumerate(lines) if line.strip() == heading]
        if not matches:
            errors.append(f"缺少强制区块：{heading}")
            continue
        if len(matches) > 1:
            errors.append(f"强制区块重复：{heading}")
        heading_indexes.append(matches[0])

    if len(heading_indexes) != len(REQUIRED_HEADINGS):
        return errors

    if heading_indexes != sorted(heading_indexes):
        errors.append("四个强制区块顺序错误。")

    if heading_indexes[0] <= subject_index:
        errors.append("强制区块必须位于提交标题之后。")

    for position, heading in enumerate(REQUIRED_HEADINGS):
        start = heading_indexes[position] + 1
        end = heading_indexes[position + 1] if position + 1 < len(heading_indexes) else len(lines)
        content = [line.strip() for line in lines[start:end] if line.strip()]
        if not content:
            errors.append(f"区块内容为空：{heading}")
            continue
        if not any(line.startswith("- ") for line in content):
            errors.append(f"区块必须至少包含一个 '- ' 项目：{heading}")

    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--message-file", type=Path)
    source.add_argument("--commit")
    args = parser.parse_args()

    try:
        if args.message_file is not None:
            message = args.message_file.read_text(encoding="utf-8", errors="strict")
        else:
            message = read_commit_message(args.commit)
    except (OSError, UnicodeError, subprocess.CalledProcessError) as exc:
        print(f"CUBE commit message format check failed: {exc}", file=sys.stderr)
        return 1

    errors = validate_message(message)
    if errors:
        print("CUBE commit message format check failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print("CUBE commit message format check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

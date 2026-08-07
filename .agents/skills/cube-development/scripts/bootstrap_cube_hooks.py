#!/usr/bin/env python3
"""Check or restore CUBE local Git hooks from tracked skill templates."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


HOOK_NAMES = ("pre-commit", "commit-msg")


def git_text(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    return result.stdout.strip()


def git_optional_text(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    return result.stdout.strip() if result.returncode == 0 else ""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=r"D:\CUBE")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    repo = Path(args.repo_root).resolve(strict=True)
    skill_root = Path(__file__).resolve().parents[1]
    template_root = skill_root / "assets" / "git-hooks"
    target_root = repo / "tools" / "git-hooks"
    apply_changes = bool(args.apply)
    problems: list[str] = []

    if git_text(repo, "rev-parse", "--is-inside-work-tree") != "true":
        raise ValueError(f"Not a Git worktree: {repo}")

    for name in HOOK_NAMES:
        template = template_root / name
        target = target_root / name
        expected = template.read_bytes()
        if not target.exists():
            if apply_changes:
                target_root.mkdir(parents=True, exist_ok=True)
                target.write_bytes(expected)
            else:
                problems.append(f"missing hook: {target}")
            continue
        if target.read_bytes() != expected:
            problems.append(f"existing hook differs; preserved without overwrite: {target}")

    hooks_path = git_optional_text(repo, "config", "--get", "core.hooksPath")
    if hooks_path != "tools/git-hooks":
        if apply_changes:
            subprocess.run(
                ["git", "-C", str(repo), "config", "core.hooksPath", "tools/git-hooks"],
                check=True,
            )
        else:
            problems.append(f"core.hooksPath is {hooks_path!r}, expected 'tools/git-hooks'")

    if problems:
        print("CUBE_HOOK_BOOTSTRAP=FAIL", file=sys.stderr)
        for problem in problems:
            print(f"- {problem}", file=sys.stderr)
        return 1
    print("CUBE_HOOK_BOOTSTRAP=PASS")
    print(f"mode={'apply' if apply_changes else 'check'}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"CUBE_HOOK_BOOTSTRAP=FAIL\nreason={exc}", file=sys.stderr)
        raise SystemExit(1)

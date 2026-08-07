#!/usr/bin/env python3
"""Validate CUBE staged scope for all or scoped commit mode."""

from __future__ import annotations

import argparse
import subprocess
import sys


def git_lines(*args: str) -> list[str]:
    result = subprocess.run(
        ["git", *args],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return [line for line in result.stdout.splitlines() if line]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=("all", "scoped"))
    args = parser.parse_args()

    try:
        staged = git_lines("diff", "--cached", "--name-only", "--diff-filter=ACMRTUXB")
        staged_deleted = git_lines("diff", "--cached", "--name-only", "--diff-filter=D")
        unstaged = git_lines("diff", "--name-only")
        untracked = git_lines("ls-files", "--others", "--exclude-standard")
    except subprocess.CalledProcessError as exc:
        print(f"CUBE commit scope check failed: git exited with {exc.returncode}.", file=sys.stderr)
        return 1

    staged_count = len(set(staged + staged_deleted))
    if staged_count == 0:
        print("CUBE commit scope check failed: 暂存区为空。", file=sys.stderr)
        return 1

    if args.mode == "all" and (unstaged or untracked):
        print("CUBE commit scope check failed: all 模式仍有未纳入改动。", file=sys.stderr)
        for path in unstaged:
            print(f"- 未暂存：{path}", file=sys.stderr)
        for path in untracked:
            print(f"- 未跟踪：{path}", file=sys.stderr)
        return 1

    print(
        "CUBE commit scope check passed: "
        f"mode={args.mode}, staged={staged_count}, unstaged={len(unstaged)}, untracked={len(untracked)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

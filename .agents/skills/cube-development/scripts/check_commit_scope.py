#!/usr/bin/env python3
"""Validate CUBE staged scope for all or scoped commit mode."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path


PROHIBITED_PREFIXES = (
    "tools/",
    "docs-site/",
    "docs/00_程序流程导航/",
    "tmp/",
    "outputs/",
    "site/",
    "LTD_MAIN_CPU2/docs/",
    "LTD_DISPLAY_CPU3/docs/",
)


def git_lines(repo: Path, *args: str) -> list[str]:
    result = subprocess.run(
        ["git", "-C", str(repo), "-c", "core.quotepath=false", *args],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return [line for line in result.stdout.splitlines() if line]


def normalize_git_path(value: str) -> str:
    normalized = value.replace("\\", "/").strip()
    while normalized.startswith("./"):
        normalized = normalized[2:]
    return normalized


def load_allowed_paths(repo: Path, scope_file: Path) -> list[Path]:
    data = json.loads(scope_file.read_text(encoding="utf-8"))
    if isinstance(data, list):
        values = data
    elif isinstance(data, dict):
        task = data.get("task", {})
        values = task.get("writePaths") or task.get("allowedPaths") or data.get("allowedPaths")
    else:
        values = None
    if not isinstance(values, list) or not values:
        raise ValueError("scope file does not contain non-empty task.writePaths or allowedPaths")

    allowed: list[Path] = []
    for value in values:
        candidate = Path(str(value))
        if not candidate.is_absolute():
            candidate = repo / candidate
        candidate = candidate.resolve(strict=False)
        try:
            candidate.relative_to(repo)
        except ValueError as exc:
            raise ValueError(f"scope path leaves repository: {candidate}") from exc
        allowed.append(candidate)
    return allowed


def path_is_allowed(repo: Path, git_path: str, allowed_paths: list[Path]) -> bool:
    candidate = (repo / normalize_git_path(git_path)).resolve(strict=False)
    for allowed in allowed_paths:
        if candidate == allowed:
            return True
        try:
            candidate.relative_to(allowed)
            return True
        except ValueError:
            continue
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=("all", "scoped"))
    parser.add_argument("--repo-root", default=".")
    parser.add_argument("--scope-file")
    args = parser.parse_args()

    repo = Path(args.repo_root).resolve(strict=True)

    try:
        staged = git_lines(
            repo, "diff", "--cached", "--no-renames", "--name-only", "--diff-filter=ACMRTUXB"
        )
        staged_deleted = git_lines(
            repo, "diff", "--cached", "--no-renames", "--name-only", "--diff-filter=D"
        )
        unstaged = git_lines(repo, "diff", "--no-renames", "--name-only")
        untracked = git_lines(repo, "ls-files", "--others", "--exclude-standard")
    except (OSError, subprocess.CalledProcessError) as exc:
        return_code = getattr(exc, "returncode", "unknown")
        print(f"CUBE commit scope check failed: git exited with {return_code}.", file=sys.stderr)
        return 1

    staged_paths = sorted(set(staged + staged_deleted))
    staged_count = len(staged_paths)
    if staged_count == 0:
        print("CUBE commit scope check failed: 暂存区为空。", file=sys.stderr)
        return 1

    prohibited = [
        path
        for path in staged
        if any(
            normalize_git_path(path).casefold().startswith(prefix.casefold())
            for prefix in PROHIBITED_PREFIXES
        )
    ]
    if prohibited:
        print("CUBE commit scope check failed: 暂存区包含本机专用或淘汰路径。", file=sys.stderr)
        for path in prohibited:
            print(f"- 禁止暂存：{path}", file=sys.stderr)
        return 1

    if args.mode == "all" and (unstaged or untracked):
        print("CUBE commit scope check failed: all 模式仍有未纳入改动。", file=sys.stderr)
        for path in unstaged:
            print(f"- 未暂存：{path}", file=sys.stderr)
        for path in untracked:
            print(f"- 未跟踪：{path}", file=sys.stderr)
        return 1

    if args.mode == "scoped":
        scope_file_value = args.scope_file or os.environ.get("CUBE_COMMIT_SCOPE_FILE", "")
        if not scope_file_value:
            print(
                "CUBE commit scope check failed: scoped 模式必须设置 CUBE_COMMIT_SCOPE_FILE。",
                file=sys.stderr,
            )
            return 1
        scope_file = Path(scope_file_value)
        if not scope_file.is_absolute():
            scope_file = repo / scope_file
        try:
            allowed_paths = load_allowed_paths(repo, scope_file.resolve(strict=True))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            print(f"CUBE commit scope check failed: 无法读取范围文件：{exc}", file=sys.stderr)
            return 1
        out_of_scope = [
            path for path in staged_paths if not path_is_allowed(repo, path, allowed_paths)
        ]
        if out_of_scope:
            print("CUBE commit scope check failed: scoped 暂存区包含批准范围外文件。", file=sys.stderr)
            for path in out_of_scope:
                print(f"- 范围外：{path}", file=sys.stderr)
            return 1

    print(
        "CUBE commit scope check passed: "
        f"mode={args.mode}, staged={staged_count}, unstaged={len(unstaged)}, untracked={len(untracked)}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

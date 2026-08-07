#!/usr/bin/env python3
"""Check whether a saved CUBE task context is still safe to resume."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path


def git_lines(repo: Path, *args: str) -> list[str]:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    return [line for line in result.stdout.splitlines() if line]


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest().upper()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--snapshot", required=True)
    parser.add_argument("--repo-root", default=r"D:\CUBE")
    parser.add_argument("--allow-head-change", action="store_true")
    parser.add_argument("--allow-status-change", action="store_true")
    args = parser.parse_args()

    repo = Path(args.repo_root).resolve(strict=True)
    snapshot_path = Path(args.snapshot)
    if not snapshot_path.is_absolute():
        snapshot_path = repo / snapshot_path
    snapshot = json.loads(snapshot_path.resolve(strict=True).read_text(encoding="utf-8"))
    if int(snapshot.get("schemaVersion", 0)) < 2:
        raise ValueError("Snapshot schema is too old; create a new task context")

    problems: list[str] = []
    repository = snapshot.get("repository", {})
    current_branch = git_lines(repo, "branch", "--show-current")
    branch = current_branch[0] if current_branch else ""
    current_head_lines = git_lines(repo, "rev-parse", "--short=12", "HEAD")
    head = current_head_lines[0] if current_head_lines else ""
    if branch != repository.get("branch"):
        problems.append(f"branch changed: {repository.get('branch')} -> {branch}")
    if not args.allow_head_change and head != repository.get("head"):
        problems.append(f"HEAD changed: {repository.get('head')} -> {head}")

    if not args.allow_status_change:
        current_status = git_lines(repo, "status", "--short", "--untracked-files=all")
        if current_status != repository.get("status", []):
            problems.append("Git status changed")

    task = snapshot.get("task", {})
    for watched in task.get("watchPaths", []):
        path = Path(str(watched.get("path", "")))
        expected_type = watched.get("type")
        if not path.exists():
            actual_type = "missing"
        elif path.is_file():
            actual_type = "file"
        else:
            actual_type = "directory"
        if actual_type != expected_type:
            problems.append(f"watch path type changed: {path}")
            continue
        if actual_type == "file" and sha256_file(path) != watched.get("sha256"):
            problems.append(f"watch file changed: {path}")

    if problems:
        print("CUBE_CONTEXT_SNAPSHOT=DRIFT", file=sys.stderr)
        for problem in problems:
            print(f"- {problem}", file=sys.stderr)
        return 1
    print("CUBE_CONTEXT_SNAPSHOT=STABLE")
    print(f"snapshot={snapshot_path.resolve()}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError, subprocess.CalledProcessError) as exc:
        print(f"CUBE_CONTEXT_SNAPSHOT=FAIL\nreason={exc}", file=sys.stderr)
        raise SystemExit(1)

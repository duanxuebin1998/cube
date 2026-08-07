#!/usr/bin/env python3
"""Coordinate non-destructive write-scope leases for concurrent CUBE tasks."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def git_text(repo: Path, *args: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
        text=True,
        encoding="utf-8",
    )
    return result.stdout.strip()


def normalize_path(repo: Path, value: str) -> Path:
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = repo / candidate
    candidate = candidate.resolve(strict=False)
    try:
        candidate.relative_to(repo)
    except ValueError as exc:
        raise ValueError(f"Write path must stay inside repository: {candidate}") from exc
    if candidate == repo:
        raise ValueError("Repository root is too broad for a task lease")
    return candidate


def overlaps(left: Path, right: Path) -> bool:
    left_text = os.path.normcase(str(left))
    right_text = os.path.normcase(str(right))
    separator = os.sep
    return (
        left_text == right_text
        or left_text.startswith(right_text.rstrip(separator) + separator)
        or right_text.startswith(left_text.rstrip(separator) + separator)
    )


def load_json(path: Path) -> dict[str, object] | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def parse_time(value: object) -> datetime | None:
    if not isinstance(value, str):
        return None
    try:
        return datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None


def active_leases(registry: Path, current_task: str) -> list[dict[str, object]]:
    active: list[dict[str, object]] = []
    now = utc_now()
    for path in sorted(registry.glob("*.json")):
        lease = load_json(path)
        if not lease or lease.get("taskId") == current_task or lease.get("status") != "active":
            continue
        expires = parse_time(lease.get("expiresAt"))
        if expires is not None and expires <= now:
            continue
        active.append(lease)
    return active


def find_conflicts(
    requested: list[Path], leases: list[dict[str, object]]
) -> list[tuple[str, str, str]]:
    conflicts: list[tuple[str, str, str]] = []
    for lease in leases:
        task_id = str(lease.get("taskId", "unknown"))
        for leased_value in lease.get("writePaths", []):
            leased = Path(str(leased_value)).resolve(strict=False)
            for candidate in requested:
                if overlaps(candidate, leased):
                    conflicts.append((task_id, str(candidate), str(leased)))
    return conflicts


def atomic_write(path: Path, data: dict[str, object]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    os.replace(temporary, path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=r"D:\CUBE")
    parser.add_argument("--task-id", required=True)
    parser.add_argument("--action", choices=("acquire", "check", "close"), required=True)
    parser.add_argument("--write-path", action="append", default=[])
    parser.add_argument("--objective", default="")
    parser.add_argument("--ttl-hours", type=float, default=12.0)
    args = parser.parse_args()

    repo = Path(args.repo_root).resolve(strict=True)
    if git_text(repo, "rev-parse", "--is-inside-work-tree") != "true":
        raise ValueError(f"Not a Git worktree: {repo}")
    registry = repo / "tmp" / ".cube-task-leases"
    lease_path = registry / f"{args.task_id}.json"

    if args.action == "close":
        lease = load_json(lease_path)
        if not lease:
            raise ValueError(f"Lease does not exist: {lease_path}")
        lease["status"] = "closed"
        lease["closedAt"] = utc_now().isoformat()
        atomic_write(lease_path, lease)
        print("CUBE_TASK_LEASE=CLOSED")
        print(f"taskId={args.task_id}")
        return 0

    requested = [normalize_path(repo, value) for value in args.write_path]
    if not requested:
        existing = load_json(lease_path)
        if existing:
            requested = [Path(str(value)) for value in existing.get("writePaths", [])]
    if not requested:
        raise ValueError("At least one --write-path is required")

    registry.mkdir(parents=True, exist_ok=True)
    conflicts = find_conflicts(requested, active_leases(registry, args.task_id))
    if conflicts:
        print("CUBE_TASK_LEASE=CONFLICT", file=sys.stderr)
        for task_id, candidate, leased in conflicts:
            print(f"taskId={task_id}\nrequested={candidate}\nleased={leased}", file=sys.stderr)
        return 2

    if args.action == "check":
        print("CUBE_TASK_LEASE=AVAILABLE")
        print(f"writePathCount={len(requested)}")
        return 0

    now = utc_now()
    lease = {
        "schemaVersion": 1,
        "taskId": args.task_id,
        "objective": args.objective,
        "status": "active",
        "acquiredAt": now.isoformat(),
        "expiresAt": (now + timedelta(hours=args.ttl_hours)).isoformat(),
        "repository": str(repo),
        "worktree": git_text(repo, "rev-parse", "--show-toplevel"),
        "branch": git_text(repo, "branch", "--show-current"),
        "head": git_text(repo, "rev-parse", "--short=12", "HEAD"),
        "writePaths": [str(path) for path in sorted(set(requested), key=str)],
    }
    atomic_write(lease_path, lease)
    print("CUBE_TASK_LEASE=ACQUIRED")
    print(f"taskId={args.task_id}")
    print(f"writePathCount={len(requested)}")
    print(f"leasePath={lease_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"CUBE_TASK_LEASE=FAIL\nreason={exc}", file=sys.stderr)
        raise SystemExit(1)

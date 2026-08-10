#!/usr/bin/env python3
"""Create or verify source-bound CUBE validation evidence manifests."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest().upper()


def git_bytes(repo: Path, *args: str) -> bytes:
    result = subprocess.run(
        ["git", "-C", str(repo), *args],
        check=True,
        capture_output=True,
    )
    return result.stdout


def git_text(repo: Path, *args: str) -> str:
    return git_bytes(repo, *args).decode("utf-8", errors="replace").strip()


def resolve_inside(repo: Path, value: str) -> Path:
    candidate = Path(value)
    if not candidate.is_absolute():
        candidate = repo / candidate
    candidate = candidate.resolve(strict=False)
    try:
        candidate.relative_to(repo)
    except ValueError as exc:
        raise ValueError(f"Path must stay inside repository: {candidate}") from exc
    return candidate


def path_state(path: Path) -> dict[str, object]:
    if not path.exists():
        raise ValueError(f"Evidence path does not exist: {path}")
    if path.is_file():
        data = path.read_bytes()
        return {
            "path": str(path),
            "type": "file",
            "sha256": sha256_bytes(data),
            "size": len(data),
            "fileCount": 1,
        }

    digest = hashlib.sha256()
    file_count = 0
    total_size = 0
    for file_path in sorted(path.rglob("*"), key=lambda item: str(item).casefold()):
        if not file_path.is_file() or ".git" in file_path.parts:
            continue
        relative = file_path.relative_to(path).as_posix().encode("utf-8")
        data = file_path.read_bytes()
        digest.update(relative)
        digest.update(b"\0")
        digest.update(hashlib.sha256(data).digest())
        digest.update(b"\0")
        file_count += 1
        total_size += len(data)
    return {
        "path": str(path),
        "type": "directory",
        "sha256": digest.hexdigest().upper(),
        "size": total_size,
        "fileCount": file_count,
    }


def repository_state(repo: Path) -> dict[str, object]:
    return {
        "branch": git_text(repo, "branch", "--show-current"),
        "head": git_text(repo, "rev-parse", "--short=12", "HEAD"),
        "statusSha256": sha256_bytes(git_bytes(repo, "status", "--porcelain=v1", "-z")),
        "stagedPatchSha256": sha256_bytes(
            git_bytes(repo, "diff", "--cached", "--binary", "--no-textconv")
        ),
        "worktreePatchSha256": sha256_bytes(
            git_bytes(repo, "diff", "--binary", "--no-textconv")
        ),
        "untrackedListSha256": sha256_bytes(
            git_bytes(repo, "ls-files", "--others", "--exclude-standard", "-z")
        ),
    }


def first_version(path: Path) -> str:
    if not path.exists():
        return "UNKNOWN"
    import re

    match = re.search(rb"V\d+\.\d+\.\d+\.\d+", path.read_bytes())
    return match.group(0).decode("ascii") if match else "UNKNOWN"


def protocol_versions(repo: Path) -> list[str]:
    import re

    values: list[str] = []
    paths = (
        repo / "LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h",
        repo / "LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h",
    )
    for path in paths:
        if path.exists():
            match = re.search(rb"DEVICE_PROTOCOL_VERSION\s+(\d+)[uU]?", path.read_bytes())
            values.append(match.group(1).decode("ascii") if match else "UNKNOWN")
    return values


def create_manifest(args: argparse.Namespace, repo: Path) -> int:
    output = resolve_inside(repo, args.output)
    allowed_output_roots = (repo / "tmp", repo / "outputs")
    if not any(output == root or root in output.parents for root in allowed_output_roots):
        raise ValueError("Manifest output must be inside tmp/ or outputs/")
    sources = [path_state(resolve_inside(repo, value)) for value in args.source_path]
    artifacts = [path_state(resolve_inside(repo, value)) for value in args.artifact]
    manifest = {
        "schemaVersion": 1,
        "createdAt": datetime.now(timezone.utc).isoformat(),
        "taskId": args.task_id,
        "evidenceType": args.evidence_type,
        "result": args.result,
        "command": args.command,
        "exitCode": args.exit_code,
        "notes": args.note,
        "repository": repository_state(repo),
        "versions": {
            "cpu2": first_version(repo / "LTD_MAIN_CPU2/Application/Inc/app_version.h"),
            "cpu3": first_version(repo / "LTD_DISPLAY_CPU3/Application/app_version.h"),
            "deviceProtocol": protocol_versions(repo),
        },
        "sources": sources,
        "artifacts": artifacts,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    temporary.replace(output)
    print("CUBE_VALIDATION_EVIDENCE=CREATED")
    print(f"output={output}")
    print(f"sourceCount={len(sources)}")
    print(f"artifactCount={len(artifacts)}")
    return 0


def check_manifest(args: argparse.Namespace, repo: Path) -> int:
    manifest_path = resolve_inside(repo, args.manifest).resolve(strict=True)
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    problems: list[str] = []
    for group in ("sources", "artifacts"):
        for expected in manifest.get(group, []):
            try:
                actual = path_state(Path(str(expected["path"])))
            except (OSError, ValueError) as exc:
                problems.append(f"{group}: {exc}")
                continue
            if actual["sha256"] != expected.get("sha256"):
                problems.append(f"{group} changed: {expected['path']}")
    if args.require_repository_state:
        current = repository_state(repo)
        for key, expected_value in manifest.get("repository", {}).items():
            if current.get(key) != expected_value:
                problems.append(f"repository state changed: {key}")
    if problems:
        print("CUBE_VALIDATION_EVIDENCE=STALE", file=sys.stderr)
        for problem in problems:
            print(f"- {problem}", file=sys.stderr)
        return 1
    print("CUBE_VALIDATION_EVIDENCE=FRESH")
    print(f"manifest={manifest_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", default=r"D:\CUBE")
    subparsers = parser.add_subparsers(dest="action", required=True)

    create = subparsers.add_parser("create")
    create.add_argument("--output", required=True)
    create.add_argument("--task-id", required=True)
    create.add_argument(
        "--evidence-type",
        required=True,
        choices=("static", "build", "render", "bench", "field", "certification"),
    )
    create.add_argument("--result", required=True, choices=("pass", "fail", "blocked"))
    create.add_argument("--command", default="")
    create.add_argument("--exit-code", type=int)
    create.add_argument("--source-path", action="append", required=True)
    create.add_argument("--artifact", action="append", default=[])
    create.add_argument("--note", action="append", default=[])

    check = subparsers.add_parser("check")
    check.add_argument("--manifest", required=True)
    check.add_argument("--require-repository-state", action="store_true")

    args = parser.parse_args()
    repo = Path(args.repo_root).resolve(strict=True)
    if git_text(repo, "rev-parse", "--is-inside-work-tree") != "true":
        raise ValueError(f"Not a Git worktree: {repo}")
    return create_manifest(args, repo) if args.action == "create" else check_manifest(args, repo)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, json.JSONDecodeError, subprocess.CalledProcessError) as exc:
        print(f"CUBE_VALIDATION_EVIDENCE=FAIL\nreason={exc}", file=sys.stderr)
        raise SystemExit(1)

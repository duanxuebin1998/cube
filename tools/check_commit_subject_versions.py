#!/usr/bin/env python3
"""Check that CUBE commit subjects contain firmware version tokens."""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


VERSION_TOKEN_RE = re.compile(r"\bCPU[23]\s+V\d+\.\d+\.\d+\.\d+\b")
CPU_VERSION_TOKEN_RE = {
    "CPU2": re.compile(r"\bCPU2\s+V\d+\.\d+\.\d+\.\d+\b"),
    "CPU3": re.compile(r"\bCPU3\s+V\d+\.\d+\.\d+\.\d+\b"),
}
DEFAULT_REMOTE_BASES = ("origin/MAIN", "origin/main")

CPU2_CODE_PREFIXES = (
    "LTD_MAIN_CPU2/",
)
CPU3_CODE_PREFIXES = (
    "LTD_DISPLAY_CPU3/",
)
CROSS_CPU_PREFIXES = (
    "docs/",
    "tools/",
)
CROSS_CPU_FILES = {
    ".gitattributes",
    "AGENTS.md",
    "CHANGELOG.md",
}


@dataclass(frozen=True)
class CommitSubject:
    revision: str
    subject: str


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def decode_git_output(data: bytes) -> str:
    for encoding in ("utf-8", "mbcs", "gbk"):
        try:
            return data.decode(encoding)
        except (LookupError, UnicodeDecodeError):
            continue
    return data.decode("utf-8", errors="replace")


def run_git(args: Sequence[str]) -> str:
    result = subprocess.run(
        ["git", *args],
        cwd=repo_root(),
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return decode_git_output(result.stdout)


def git_output_lines(args: Sequence[str]) -> list[str]:
    return [line.strip().replace("\\", "/") for line in run_git(args).splitlines() if line.strip()]


def git_ref_exists(ref: str) -> bool:
    result = subprocess.run(
        ["git", "rev-parse", "--verify", "--quiet", ref],
        cwd=repo_root(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return result.returncode == 0


def default_range_spec() -> str:
    for base in DEFAULT_REMOTE_BASES:
        if git_ref_exists(base):
            return f"{base}..HEAD"
    if git_ref_exists("HEAD~1"):
        return "HEAD~1..HEAD"
    return "HEAD"


def subject_has_version(subject: str) -> bool:
    return bool(VERSION_TOKEN_RE.search(subject))


def subject_has_cpu_version(subject: str, cpu: str) -> bool:
    return bool(CPU_VERSION_TOKEN_RE[cpu].search(subject))


def subject_from_message(message: str) -> str:
    for line in message.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        return stripped
    return ""


def staged_files() -> list[str]:
    return git_output_lines(["diff", "--cached", "--name-only"])


def commit_files(revision: str) -> list[str]:
    return git_output_lines(["diff-tree", "--root", "--no-commit-id", "--name-only", "-r", revision])


def commit_message_required_cpus() -> set[str]:
    paths = staged_files()
    if paths:
        return required_cpus_for_paths(paths)
    if git_ref_exists("HEAD"):
        return required_cpus_for_paths(commit_files("HEAD"))
    return set()


def required_cpus_for_paths(paths: Sequence[str]) -> set[str]:
    normalized = [path.replace("\\", "/") for path in paths]
    if any(path in CROSS_CPU_FILES or path.startswith(CROSS_CPU_PREFIXES) for path in normalized):
        return {"CPU2", "CPU3"}

    required: set[str] = set()
    if any(path.startswith(CPU2_CODE_PREFIXES) for path in normalized):
        required.add("CPU2")
    if any(path.startswith(CPU3_CODE_PREFIXES) for path in normalized):
        required.add("CPU3")
    return required


def commit_subjects(range_spec: str) -> list[CommitSubject]:
    output = run_git(["log", "--format=%H%x1f%s", range_spec])
    subjects: list[CommitSubject] = []
    for line in output.splitlines():
        if not line:
            continue
        if "\x1f" not in line:
            subjects.append(CommitSubject(revision=line.strip(), subject=""))
            continue
        revision, subject = line.split("\x1f", 1)
        subjects.append(CommitSubject(revision=revision.strip(), subject=subject.strip()))
    return subjects


def missing_version_subjects(subjects: Sequence[CommitSubject]) -> list[CommitSubject]:
    return [item for item in subjects if not subject_has_version(item.subject)]


def missing_required_cpu_versions(subject: str, required_cpus: set[str]) -> set[str]:
    if not required_cpus:
        return set() if subject_has_version(subject) else {"CPU2/CPU3"}
    return {cpu for cpu in sorted(required_cpus) if not subject_has_cpu_version(subject, cpu)}


def print_required_format() -> None:
    print(
        "Required: include CPU2 Vx.x.x.x or CPU3 Vx.x.x.x in the commit subject; "
        "cross-CPU, docs, and tooling commits should use both, for example "
        "（CPU2 V1.20.2.0 / CPU3 V1.18.2.0）."
    )


def check_one_subject(subject: str, required_cpus: set[str] | None = None) -> int:
    missing = missing_required_cpu_versions(subject, required_cpus or set())
    if not missing:
        print("Commit subject version check passed.")
        return 0
    print("Commit subject version check failed:")
    if required_cpus:
        print(f"- {subject or '<empty subject>'} (missing: {', '.join(sorted(missing))})")
    else:
        print(f"- {subject or '<empty subject>'}")
    print_required_format()
    return 1


def check_range(range_spec: str) -> int:
    subjects = commit_subjects(range_spec)
    if not subjects:
        print(f"No commits to check ({range_spec}).")
        return 0
    missing: list[tuple[CommitSubject, set[str]]] = []
    for item in subjects:
        required_cpus = required_cpus_for_paths(commit_files(item.revision))
        missing_cpus = missing_required_cpu_versions(item.subject, required_cpus)
        if missing_cpus:
            missing.append((item, missing_cpus))

    if not missing:
        print(f"Commit subject version check passed ({len(subjects)} commit(s), {range_spec}).")
        return 0

    print(f"Commit subject version check failed ({range_spec}):")
    for item, missing_cpus in missing:
        prefix = item.revision[:12] if item.revision else "<unknown>"
        print(f"- {prefix} {item.subject or '<empty subject>'} (missing: {', '.join(sorted(missing_cpus))})")
    print_required_format()
    return 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Ensure CUBE commit subjects keep CPU2/CPU3 firmware versions visible.",
    )
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--range", dest="range_spec", help="Git revision range to check.")
    group.add_argument("--subject", help="Check one commit subject string.")
    group.add_argument("--commit-msg-file", type=Path, help="Check the subject in a Git commit message file.")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.subject is not None:
        return check_one_subject(args.subject.strip())

    if args.commit_msg_file is not None:
        message = args.commit_msg_file.read_text(encoding="utf-8", errors="replace")
        return check_one_subject(subject_from_message(message), commit_message_required_cpus())

    range_spec = args.range_spec or default_range_spec()
    try:
        return check_range(range_spec)
    except subprocess.CalledProcessError as error:
        sys.stderr.write(decode_git_output(error.stderr))
        return error.returncode


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Bump CUBE firmware version headers.

Usage examples:
  py tools/bump_version.py --cpu2 build
  py tools/bump_version.py --cpu3 patch
  py tools/bump_version.py --both minor
  py tools/bump_version.py --cpu2 set 1.2.0.0
  py tools/bump_version.py --both set V1.2.0.0
  py tools/bump_version.py --both minor --note "新增CPU2/CPU3版本兼容检查"
  py tools/bump_version.py --both build --dry-run
"""

from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from datetime import date
from pathlib import Path


@dataclass(frozen=True)
class VersionTarget:
    name: str
    path: Path
    prefix: str
    string_macro: str


@dataclass(frozen=True)
class Version:
    major: int
    minor: int
    patch: int
    build: int

    def bump(self, kind: str) -> "Version":
        if kind == "major":
            return Version(self.major + 1, 0, 0, 0)
        if kind == "minor":
            return Version(self.major, self.minor + 1, 0, 0)
        if kind == "patch":
            return Version(self.major, self.minor, self.patch + 1, 0)
        if kind == "build":
            return Version(self.major, self.minor, self.patch, self.build + 1)
        raise ValueError(f"Unsupported bump kind: {kind}")

    def validate_byte_fields(self) -> None:
        for name, value in (
            ("major", self.major),
            ("minor", self.minor),
            ("patch", self.patch),
            ("build", self.build),
        ):
            if not 0 <= value <= 255:
                raise ValueError(f"Version {name} must be in range 0..255: {value}")

    def text(self) -> str:
        return f"V{self.major}.{self.minor}.{self.patch}.{self.build}"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def read_text_preserve_newline(path: Path) -> tuple[str, str, str]:
    raw = path.read_bytes()
    newline = "\r\n" if b"\r\n" in raw else "\n"
    for encoding in ("utf-8", "gbk"):
        try:
            return raw.decode(encoding), newline, encoding
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("utf-8", raw, 0, 1, f"unsupported text encoding for {path}")


def write_text_preserve_newline(path: Path, text: str, newline: str, encoding: str) -> None:
    normalized = text.replace("\r\n", "\n").replace("\r", "\n")
    if newline == "\r\n":
        normalized = normalized.replace("\n", "\r\n")
    path.write_bytes(normalized.encode(encoding))


def read_optional_text(path: Path) -> tuple[str, str, str]:
    if not path.exists():
        return "", "\n", "utf-8"
    return read_text_preserve_newline(path)


def macro_pattern(name: str) -> re.Pattern[str]:
    return re.compile(rf"^(#define\s+{re.escape(name)}\s+)(\d+)(u\b.*)$", re.MULTILINE)


def parse_macro(text: str, name: str) -> int:
    match = macro_pattern(name).search(text)
    if not match:
        raise RuntimeError(f"Missing macro: {name}")
    return int(match.group(2))


def parse_version(text: str, prefix: str) -> Version:
    version = Version(
        major=parse_macro(text, f"{prefix}_MAJOR"),
        minor=parse_macro(text, f"{prefix}_MINOR"),
        patch=parse_macro(text, f"{prefix}_PATCH"),
        build=parse_macro(text, f"{prefix}_BUILD"),
    )
    version.validate_byte_fields()
    return version


def parse_version_arg(value: str) -> Version:
    match = re.fullmatch(r"V?(\d+)\.(\d+)\.(\d+)\.(\d+)", value.strip(), re.IGNORECASE)
    if not match:
        raise argparse.ArgumentTypeError("Version must use V<major>.<minor>.<patch>.<build>, for example V1.2.0.0")
    version = Version(*(int(part) for part in match.groups()))
    try:
        version.validate_byte_fields()
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
    return version


def replace_macro(text: str, name: str, value: int) -> str:
    pattern = macro_pattern(name)
    if not pattern.search(text):
        raise RuntimeError(f"Missing macro: {name}")
    return pattern.sub(lambda match: f"{match.group(1)}{value}{match.group(3)}", text)


def replace_string_macro(text: str, name: str, value: str) -> str:
    pattern = re.compile(rf'^(#define\s+{re.escape(name)}\s+)"[^"]*"(.*)$', re.MULTILINE)
    if not pattern.search(text):
        raise RuntimeError(f"Missing macro: {name}")
    return pattern.sub(lambda match: f'{match.group(1)}"{value}"{match.group(2)}', text)


def update_target(target: VersionTarget, kind: str, requested: Version | None, dry_run: bool) -> tuple[Version, Version]:
    text, newline, encoding = read_text_preserve_newline(target.path)
    old = parse_version(text, target.prefix)
    new = requested if kind == "set" else old.bump(kind)
    new.validate_byte_fields()

    updated = text
    for field, value in (
        ("MAJOR", new.major),
        ("MINOR", new.minor),
        ("PATCH", new.patch),
        ("BUILD", new.build),
    ):
        updated = replace_macro(updated, f"{target.prefix}_{field}", value)

    updated = replace_string_macro(updated, target.string_macro, new.text())

    if not dry_run:
        write_text_preserve_newline(target.path, updated, newline, encoding)

    return old, new


def append_changelog(root: Path, changes: list[tuple[VersionTarget, Version, Version, str]], note: str | None, dry_run: bool) -> None:
    changelog_path = root / "CHANGELOG.md"
    note_text = note.strip() if note and note.strip() else "未填写"
    lines = [
        f"## {date.today().isoformat()}",
        "",
    ]

    for target, old, new, kind in changes:
        lines.append(f"- {target.name}: {old.text()} -> {new.text()} ({kind})")

    lines.extend([
        f"- 说明：{note_text}",
        "",
    ])
    entry = "\n".join(lines)

    if dry_run:
        print("CHANGELOG.md would append:")
        print(entry.rstrip())
        return

    current, newline, encoding = read_optional_text(changelog_path)
    if current:
        updated = current.rstrip() + "\n\n" + entry
    else:
        updated = "# 升级日志\n\n记录 CPU2/CPU3 固件版本变更。使用 `tools/bump_version.py` 升级版本时会自动追加记录。\n\n" + entry

    write_text_preserve_newline(changelog_path, updated, newline, encoding)


def build_targets(root: Path) -> dict[str, VersionTarget]:
    return {
        "cpu2": VersionTarget(
            name="CPU2",
            path=root / "LTD_MAIN_CPU2" / "Application" / "Inc" / "app_version.h",
            prefix="CPU2_APP_VERSION",
            string_macro="CPU2_APP_VERSION_STRING",
        ),
        "cpu3": VersionTarget(
            name="CPU3",
            path=root / "LTD_DISPLAY_CPU3" / "Application" / "app_version.h",
            prefix="CPU3_APP_VERSION",
            string_macro="CPU3_APP_VERSION_STRING",
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bump CUBE firmware versions.")
    group = parser.add_mutually_exclusive_group(required=True)
    actions = ("build", "patch", "minor", "major", "set")
    group.add_argument("--cpu2", choices=actions)
    group.add_argument("--cpu3", choices=actions)
    group.add_argument("--both", choices=actions)
    parser.add_argument("version", nargs="?", type=parse_version_arg, help="Required when action is 'set', for example V1.2.0.0.")
    parser.add_argument("--note", help="Change log note appended to CHANGELOG.md when versions are written.")
    parser.add_argument("--no-changelog", action="store_true", help="Do not update CHANGELOG.md.")
    parser.add_argument("--dry-run", action="store_true", help="Print changes without writing files.")
    args = parser.parse_args()

    action = args.cpu2 or args.cpu3 or args.both
    if action == "set" and args.version is None:
        parser.error("the 'set' action requires a version, for example: --cpu2 set 1.2.0.0")
    if action != "set" and args.version is not None:
        parser.error("version can only be provided with the 'set' action")

    return args


def main() -> int:
    args = parse_args()
    root = repo_root()
    targets = build_targets(root)

    selected: list[tuple[VersionTarget, str]] = []
    if args.cpu2:
        selected.append((targets["cpu2"], args.cpu2))
    elif args.cpu3:
        selected.append((targets["cpu3"], args.cpu3))
    else:
        selected.append((targets["cpu2"], args.both))
        selected.append((targets["cpu3"], args.both))

    changes: list[tuple[VersionTarget, Version, Version, str]] = []
    for target, kind in selected:
        old, new = update_target(target, kind, args.version, args.dry_run)
        changes.append((target, old, new, kind))
        if kind == "set":
            action = "would set" if args.dry_run else "set"
            print(f"{target.name}: {old.text()} -> {new.text()} ({action})")
        else:
            action = "would bump" if args.dry_run else "bumped"
            print(f"{target.name}: {old.text()} -> {new.text()} ({action} {kind})")

    if not args.no_changelog:
        append_changelog(root, changes, args.note, args.dry_run)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

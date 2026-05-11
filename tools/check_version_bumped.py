#!/usr/bin/env python3
"""Check that staged firmware changes include the matching version header."""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class FirmwareArea:
    name: str
    version_header: str
    code_prefixes: tuple[str, ...]


CPU2 = FirmwareArea(
    name="CPU2",
    version_header="LTD_MAIN_CPU2/Application/Inc/app_version.h",
    code_prefixes=(
        "LTD_MAIN_CPU2/Application/",
        "LTD_MAIN_CPU2/Core/",
        "LTD_MAIN_CPU2/Drivers/",
        "LTD_MAIN_CPU2/Services/",
        "LTD_MAIN_CPU2/CMakeLists.txt",
        "LTD_MAIN_CPU2/cmake/",
    ),
)

CPU3 = FirmwareArea(
    name="CPU3",
    version_header="LTD_DISPLAY_CPU3/Application/app_version.h",
    code_prefixes=(
        "LTD_DISPLAY_CPU3/Application/",
        "LTD_DISPLAY_CPU3/Communication/",
        "LTD_DISPLAY_CPU3/Core/",
        "LTD_DISPLAY_CPU3/Drivers/",
        "LTD_DISPLAY_CPU3/CMakeLists.txt",
        "LTD_DISPLAY_CPU3/cmake/",
    ),
)

CHANGELOG_PATH = "CHANGELOG.md"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def staged_files() -> list[str]:
    result = subprocess.run(
        ["git", "diff", "--cached", "--name-only"],
        cwd=repo_root(),
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    )
    return [line.strip().replace("\\", "/") for line in result.stdout.splitlines() if line.strip()]


def touches_firmware(area: FirmwareArea, path: str) -> bool:
    if path == area.version_header:
        return False
    return any(path == prefix or path.startswith(prefix) for prefix in area.code_prefixes)


def check_area(area: FirmwareArea, paths: list[str]) -> str | None:
    touched = [path for path in paths if touches_firmware(area, path)]
    if touched and area.version_header not in paths:
        # 提交前只检查已暂存内容，避免把工作区未暂存修改误判为本次提交。
        return (
            f"{area.name} firmware files are staged but {area.version_header} is not staged.\n"
            f"Run tools/bump_version.py for {area.name.lower()} and stage the version header before committing."
        )
    return None


def check_changelog(paths: list[str]) -> str | None:
    version_headers = [area.version_header for area in (CPU2, CPU3)]
    if any(path in paths for path in version_headers) and CHANGELOG_PATH not in paths:
        return f"Version header is staged but {CHANGELOG_PATH} is not staged."
    return None


def main() -> int:
    paths = staged_files()
    if not paths:
        print("No staged changes to check.")
        return 0

    errors = [error for area in (CPU2, CPU3) if (error := check_area(area, paths))]
    changelog_error = check_changelog(paths)
    if changelog_error:
        errors.append(changelog_error)
    if errors:
        print("Version bump check failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    checked = [area.name for area in (CPU2, CPU3) if any(touches_firmware(area, path) for path in paths)]
    suffix = f" ({', '.join(checked)})" if checked else ""
    print(f"Version bump check passed{suffix}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

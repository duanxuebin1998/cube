#!/usr/bin/env python3
"""Check CPU2 water precise-search entry behavior."""

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "measure_waterLevel.c"


def read_source(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("gbk", "utf-8-sig"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise AssertionError(f"cannot decode {path}")


def extract_function(source: str, name: str) -> str:
    match = re.search(
        rf"\b(?:static\s+)?(?:int|uint32_t)\s+{re.escape(name)}\s*\([^)]*\)\s*\{{",
        source,
    )
    if match is None:
        raise AssertionError(f"{name} not found")

    index = match.end()
    depth = 1
    while index < len(source):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[match.end() : index]
        index += 1

    raise AssertionError(f"{name} body is incomplete")


def first_index(text: str, pattern: str, note: str) -> int:
    match = re.search(pattern, text, re.S)
    if match is None:
        raise AssertionError(note)
    return match.start()


def main() -> int:
    source = read_source(SRC)
    precise_body = extract_function(source, "SearchWaterPrecise")

    first_down = first_index(
        precise_body,
        r"MotorCtrl_MoveDown\s*\(",
        "SearchWaterPrecise must still perform the downward precise search",
    )
    first_status = first_index(
        precise_body,
        r"check_water_status\s*\(\s*&water_state\s*\)",
        "SearchWaterPrecise must check water state before precise descent",
    )
    first_up = first_index(
        precise_body,
        r"MotorCtrl_MoveUp\s*\(",
        "SearchWaterPrecise must move up out of WATER before precise descent",
    )
    first_water_guard = first_index(
        precise_body,
        r"water_state\s*==\s*WATER",
        "SearchWaterPrecise must explicitly handle already-WATER entry state",
    )

    if not (first_status < first_down and first_up < first_down and first_water_guard < first_down):
        raise AssertionError(
            "SearchWaterPrecise must escape an already-WATER state before first MotorCtrl_MoveDown"
        )

    print("water precise search contract: ok")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"water precise search contract: FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)

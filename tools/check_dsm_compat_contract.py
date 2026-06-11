#!/usr/bin/env python3
"""检查 DSM 对外兼容契约，防止调试区寄存器口径回退。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DSM_DATA_ANALYSIS = (
    ROOT
    / "LTD_DISPLAY_CPU3"
    / "Communication"
    / "external"
    / "DSM_modbus"
    / "DSM_DataAnalysis_modbus2.c"
)


def extract_function_body(source: str, name: str) -> str:
    """按括号层级截取 C 函数体，避免用整文件字符串误判寄存器写入位置。"""
    match = re.search(rf"\bvoid\s+{re.escape(name)}\s*\([^)]*\)\s*\{{", source)
    if match is None:
        raise AssertionError(f"{name} not found")

    start = match.end()
    depth = 1
    index = start
    while index < len(source):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[start:index]
        index += 1

    raise AssertionError(f"{name} body is incomplete")


def normalize(text: str) -> str:
    """压缩空白字符，使契约检查不受换行和缩进影响。"""
    return re.sub(r"\s+", "", text)


def check_dsm_x_angle_offset(source: str) -> None:
    """确认 DSM 0x0104 仍按一代协议输出 X 角度偏移值。"""
    body = extract_function_body(source, "Input_Write")
    compact_body = normalize(body)

    expected = normalize(
        "WriteOneInputRegister(INPUTREGISTER_CIRCLE, 1, g_measurement.debug_data.angle_x + 0x8000)"
    )
    if expected not in compact_body:
        raise AssertionError(
            "DSM input register 0x0104 must publish X angle as angle_x + 0x8000 for first-generation compatibility"
        )


def main() -> int:
    """读取 DSM 协议实现并执行当前需要固化的一代兼容检查。"""
    source = DSM_DATA_ANALYSIS.read_text(encoding="utf-8")
    try:
        check_dsm_x_angle_offset(source)
    except AssertionError as exc:
        print(f"DSM compatibility contract check failed: {exc}")
        return 1

    print("DSM compatibility contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

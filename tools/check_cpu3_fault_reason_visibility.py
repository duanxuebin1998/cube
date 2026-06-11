#!/usr/bin/env python3
"""检查 CPU3 故障原因只在故障状态页通过受控入口显示。"""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
DISPLAY_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display.c"


def extract_function_body(source: str, name: str) -> str:
    """按括号层级截取 C 函数体，用于做轻量级显示契约检查。"""
    match = re.search(rf"static\s+[A-Za-z_][\w\s\*]*\s+{re.escape(name)}\s*\([^)]*\)\s*\{{", source)
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


def main() -> int:
    """校验故障原因的状态上下文、错误码门控、行注册和渲染入口。"""
    source = DISPLAY_C.read_text(encoding="utf-8")
    calculate_body = extract_function_body(source, "CalculateValidPara")
    show_body = extract_function_body(source, "Display_ShouldShowErrorReason")
    equipment_body = extract_function_body(source, "oled_equipment")

    # 这些约束防止故障原因泄漏到普通状态页，也防止故障态缺少详情入口。
    required_patterns = [
        (
            "state-error context mapping",
            "case STATE_ERROR:" in source
            and "return DISPLAY_RESULT_CONTEXT_ERROR;" in source,
        ),
        (
            "fault reason gated by error context",
            "ctx == DISPLAY_RESULT_CONTEXT_ERROR" in show_body,
        ),
        (
            "fault reason gated by non-zero error code",
            "g_measurement.device_status.error_code != NO_ERROR" in show_body,
        ),
        (
            "fault reason rows registered through guard",
            "if (Display_ShouldShowErrorReason(ctx))" in calculate_body,
        ),
        (
            "primary fault reason row registration",
            "ValidParaDisArr[Para_ErrorReason][PARA_VALID] = true" in calculate_body,
        ),
        (
            "secondary fault reason row registration",
            "Display_IsErrorReasonNeedTwoRows()" in calculate_body
            and "ValidParaDisArr[Para_ErrorReasonMore][PARA_VALID] = true" in calculate_body,
        ),
        (
            "primary fault reason row rendering",
            "Display_ErrorReasonLine(row)" in equipment_body,
        ),
        (
            "secondary fault reason row rendering",
            "Display_ErrorReasonMoreLine(row)" in equipment_body,
        ),
    ]

    failures = [name for name, ok in required_patterns if not ok]
    if failures:
        print("CPU3 fault reason visibility check failed:")
        for name in failures:
            print(f"- Missing or invalid: {name}")
        return 1

    print("CPU3 fault reason visibility check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""检查 CPU2 读取部件参数的命令内持续刷新契约。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APP_MAIN = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "app_main.c"
SENSOR_C = ROOT / "LTD_MAIN_CPU2" / "Services" / "Sensor" / "sensor.c"


def read_text(path: Path) -> str:
    """按源码可能的 UTF-8/GBK 编码读取文件，避免检查脚本误报编码问题。"""
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "gbk"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", data, 0, 1, f"cannot decode {path}")


def extract_function_body(source: str, name: str) -> str:
    """按括号层级截取函数体，用于限定检查只发生在目标命令函数内。"""
    match = re.search(rf"\b(?:static\s+)?(?:void|uint8_t)\s+{re.escape(name)}\s*\([^)]*\)\s*\{{", source)
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
    """压缩空白字符，使契约检查不受格式化差异影响。"""
    return re.sub(r"\s+", "", text)


def strip_c_comments(text: str) -> str:
    """剥离 C 注释，避免说明性注释插在关键语句之间导致契约匹配误报。"""
    return re.sub(r"/\*.*?\*/|//[^\r\n]*", "", text, flags=re.DOTALL)


def main() -> int:
    """校验持续刷新留在 CMD_ReadPartParams 内，并保留命令切换打断路径。"""
    app_main_source = read_text(APP_MAIN)
    sensor_source = read_text(SENSOR_C)
    compact_sensor = normalize(strip_c_comments(sensor_source))

    # 合同点：刷新必须留在命令函数内部，并且等待周期内仍允许新命令打断。
    checks = [
        ("no main-loop polling helper", "App_PollReadPartParamsRefresh" not in app_main_source),
        ("refresh interval", "#defineREAD_PART_PARAMS_REFRESH_INTERVAL_MS1000U" in compact_sensor),
        ("read-params state loop", "while(g_measurement.device_status.device_state==STATE_READPARAMETEROVER)" in compact_sensor),
        ("abortable refresh wait", "AbortableDelay_CommandSwitch(READ_PART_PARAMS_REFRESH_INTERVAL_MS,100U)" in compact_sensor),
        ("part params reader", "Sensor_CheckAllPartParams()" in compact_sensor),
        ("command switch handling", "if(ret==STATE_SWITCH){SET_ERROR(ret);break;}" in compact_sensor),
    ]

    try:
        cmd_body = extract_function_body(sensor_source, "CMD_ReadPartParams")
        cmd_compact = normalize(strip_c_comments(cmd_body))
        checks.append(("initial command read", "Sensor_ReadPartParamsInternal(1U)" in cmd_compact))
        checks.append(("loop uses check entry", "Sensor_CheckAllPartParams()" in cmd_compact))
    except AssertionError as exc:
        print(f"Read-part-params refresh contract check failed: {exc}")
        return 1

    failed = [name for name, passed in checks if not passed]
    if failed:
        print("Read-part-params refresh contract check failed:")
        for name in failed:
            print(f"- missing {name}")
        return 1

    print("Read-part-params refresh contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

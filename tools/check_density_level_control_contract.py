#!/usr/bin/env python3
"""Check CPU2 continuous liquid-level control wiring.

This is a lightweight source contract check for the embedded target where no
host-side C test runner exists in the repository.
"""

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "measure_oilLevel.c"
HDR = ROOT / "LTD_MAIN_CPU2" / "Application" / "Inc" / "measure_oilLevel.h"
CPU3_DISPLAY = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display_tankopera.c"
CPU2_PARAM_HEADER = ROOT / "LTD_MAIN_CPU2" / "Services" / "ParamStorage" / "system_parameter.h"
CPU3_PARAM_HEADER = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.h"
CPU3_PARAM_TABLE = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.c"
PROTOCOL_DOC = ROOT / "docs" / "01_协议与寄存器" / "CPU2_CPU3协议变更记录.md"


def require(text: str, pattern: str, note: str) -> None:
    if not re.search(pattern, text, re.S):
        raise AssertionError(note)


def forbid(text: str, pattern: str, note: str) -> None:
    if re.search(pattern, text, re.S):
        raise AssertionError(note)


def parse_protocol_version(text: str) -> int:
    match = re.search(r"#define\s+DEVICE_PROTOCOL_VERSION\s+(\d+)u?\b", text)
    if match is None:
        raise AssertionError("missing DEVICE_PROTOCOL_VERSION")
    return int(match.group(1))


def main() -> int:
    source = SRC.read_text(encoding="gbk")
    header = HDR.read_text(encoding="gbk")
    cpu3_display = CPU3_DISPLAY.read_text(encoding="utf-8")
    cpu2_param_header = CPU2_PARAM_HEADER.read_text(encoding="gbk")
    cpu3_param_header = CPU3_PARAM_HEADER.read_text(encoding="utf-8")
    cpu3_param_table = CPU3_PARAM_TABLE.read_text(encoding="utf-8")
    protocol_doc = PROTOCOL_DOC.read_text(encoding="utf-8")

    cpu2_protocol_version = parse_protocol_version(cpu2_param_header)
    cpu3_protocol_version = parse_protocol_version(cpu3_param_header)
    if cpu2_protocol_version != cpu3_protocol_version:
        raise AssertionError("CPU2/CPU3 protocol versions must match")
    if cpu2_protocol_version < 8:
        raise AssertionError("shared protocol version must include protocol 8 liquid-level method semantics")

    checks = [
        (
            cpu3_param_table,
            r'"液位测量方式"[\s\S]{0,240}COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD[\s\S]{0,120}true,\s*0,\s*5',
            "CPU3 liquid level measurement method metadata must validate the 0..5 method range",
        ),
        (
            protocol_doc,
            r"###\s+协议版本\s+8[\s\S]+liquidLevelMeasurementMethod[\s\S]+2=密度连续找液位[\s\S]+4=连续相对频率[\s\S]+5=连续定频",
            "protocol change record must document liquidLevelMeasurementMethod protocol version 8 semantics",
        ),
        (
            source,
            r"#define\s+OIL_LEVEL_METHOD_DENSITY\s+2U",
            "density method constant must be defined as 2",
        ),
        (
            source,
            r"#define\s+OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ\s+4U",
            "continuous relative-frequency method must be added as method 4",
        ),
        (
            source,
            r"#define\s+OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ\s+5U",
            "continuous fixed-frequency method must be added as method 5",
        ),
        (
            source,
            r"#define\s+OIL_LEVEL_METHOD_RELATIVE_FREQ\s+0U",
            "relative-frequency method constant must be defined as 0",
        ),
        (
            source,
            r"#define\s+OIL_LEVEL_METHOD_FIXED_FREQ\s+1U",
            "fixed-frequency method constant must be defined as 1",
        ),
        (
            source,
            r"static\s+uint32_t\s+DensityLevel_ComputeSpeedX100\s*\(",
            "density speed pure helper is missing",
        ),
        (
            source,
            r"static\s+uint32_t\s+DensityLevel_RunClosedLoop\s*\(",
            "density closed-loop runner is missing",
        ),
        (
            source,
            r"target_density\s*=\s*RAW_TO_DENSITY\s*\(\s*g_deviceParams\.oilLevelDensity\s*\)",
            "target density must use existing RAW_TO_DENSITY scaling",
        ),
        (
            source,
            r"if\s*\(\s*density_error\s*>\s*deadband",
            "positive density error must be detected",
        ),
        (
            source,
            r"dir\s*=\s*MOTOR_DIRECTION_DOWN",
            "below target density must command down direction",
        ),
        (
            source,
            r"else\s+if\s*\(\s*density_error\s*<\s*-\s*deadband",
            "negative density error must be detected",
        ),
        (
            source,
            r"dir\s*=\s*MOTOR_DIRECTION_UP",
            "above target density must command up direction",
        ),
        (
            source,
            r"MotorCtrl_SlowStop\s*\(\s*\)",
            "closed loop must stop motor before stable confirmation or direction change",
        ),
        (
            source,
            r"DensityLevel_StopAndReturn\s*\(",
            "closed loop errors must stop before return",
        ),
        (
            source,
            r"Read_Density\s*\(",
            "closed loop must read density values",
        ),
        (
            source,
            r"case\s+OIL_LEVEL_METHOD_DENSITY\s*:",
            "SearchOilLevel/FollowOilLevel must branch on method 2",
        ),
        (
            source,
            r"static\s+uint32_t\s+FrequencyLevel_ComputeSpeedX100\s*\(",
            "frequency speed helper is missing",
        ),
        (
            source,
            r"static\s+uint32_t\s+FrequencyLevel_RunClosedLoop\s*\(",
            "frequency closed-loop runner is missing",
        ),
        (
            source,
            r"FREQUENCY_LEVEL_KP_SPEED_X100_PER_HZ",
            "frequency closed loop must use proportional speed gain",
        ),
        (
            source,
            r"frequency_error\s*=\s*\(float\)\s*g_measurement\.oil_measurement\.current_frequency\s*-\s*\(float\)\s*g_measurement\.oil_measurement\.follow_frequency",
            "frequency error must compare current frequency against target frequency",
        ),
        (
            source,
            r"if\s*\(\s*frequency_error\s*>\s*deadband",
            "high current frequency must be detected",
        ),
        (
            source,
            r"else\s+if\s*\(\s*frequency_error\s*<\s*-\s*deadband",
            "low current frequency must be detected",
        ),
        (
            source,
            r"speed_x100\s*=\s*FrequencyLevel_ComputeSpeedX100\s*\([^;]+;\s*if\s*\(\s*speed_x100\s*==\s*0U\s*\)\s*\{\s*speed_x100\s*=\s*FREQUENCY_LEVEL_MIN_SPEED_X100",
            "frequency closed loop must keep a minimum speed when endpoint correction requests motion inside deadband",
        ),
        (
            source,
            r"dir\s*==\s*DENSITY_LEVEL_DIR_NONE[\s\S]{0,900}air_frequency[\s\S]{0,900}dir\s*=\s*MOTOR_DIRECTION_DOWN",
            "relative-frequency closed loop must move down when too close to air endpoint",
        ),
        (
            source,
            r"dir\s*==\s*DENSITY_LEVEL_DIR_NONE[\s\S]{0,900}oil_frequency[\s\S]{0,900}dir\s*=\s*MOTOR_DIRECTION_UP",
            "relative-frequency closed loop must move up when too close to oil endpoint",
        ),
        (
            source,
            r"case\s+OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ\s*:",
            "SearchOilLevel/FollowOilLevel must branch on continuous relative-frequency method",
        ),
        (
            source,
            r"case\s+OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ\s*:",
            "SearchOilLevel/FollowOilLevel must branch on continuous fixed-frequency method",
        ),
        (
            source,
            r"case\s+OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ\s*:\s*case\s+OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ\s*:[\s\S]{0,500}FrequencyLevel_RunClosedLoop\s*\(\s*FREQUENCY_LEVEL_RUN_SEARCH\s*\)",
            "only new continuous frequency search methods must use continuous closed loop",
        ),
        (
            source,
            r"case\s+OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ\s*:\s*case\s+OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ\s*:[\s\S]{0,500}FrequencyLevel_RunClosedLoop\s*\(\s*FREQUENCY_LEVEL_RUN_FOLLOW\s*\)",
            "only new continuous frequency follow methods must use continuous closed loop",
        ),
        (
            source,
            r"case\s+OIL_LEVEL_METHOD_RELATIVE_FREQ\s*:\s*case\s+OIL_LEVEL_METHOD_FIXED_FREQ\s*:[\s\S]{0,500}SearchOilPrecise\s*\(\s*100\s*\)",
            "original relative/fixed frequency methods must keep the legacy precise search path",
        ),
        (
            cpu3_display,
            r"连相对频率",
            "CPU3 menu must expose continuous relative-frequency method",
        ),
        (
            cpu3_display,
            r"连定频",
            "CPU3 menu must expose continuous fixed-frequency method",
        ),
    ]

    for text, pattern, note in checks:
        require(text, pattern, note)

    if "DensityLevel_RunForTest" in source or "DensityLevel_RunForTest" in header:
        raise AssertionError("production code must not expose test-only density entry points")
    if "FrequencyLevel_RunForTest" in source or "FrequencyLevel_RunForTest" in header:
        raise AssertionError("production code must not expose test-only frequency entry points")

    forbid(
        source,
        r"case\s+OIL_LEVEL_METHOD_RELATIVE_FREQ\s*:\s*case\s+OIL_LEVEL_METHOD_FIXED_FREQ\s*:[\s\S]{0,500}FrequencyLevel_RunClosedLoop",
        "original relative/fixed frequency methods must not be replaced by continuous closed loop",
    )

    print("continuous level control contract: ok")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AssertionError as exc:
        print(f"continuous level control contract: FAIL: {exc}", file=sys.stderr)
        raise SystemExit(1)

#!/usr/bin/env python3
"""检查 CPU2/CPU3 的 SI 共享协议契约是否一致。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

CPU2_PARAM = ROOT / "LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h"
CPU3_PARAM = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h"
CPU2_STATE = ROOT / "LTD_MAIN_CPU2/Services/Modbus/stateformodbus.h"
CPU3_STATE = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/stateformodbus.h"
CPU2_MODBUS = ROOT / "LTD_MAIN_CPU2/Services/Modbus/dataanalysis_modbus.c"
CPU3_MODBUS = ROOT / "LTD_DISPLAY_CPU3/Communication/internal/main_board_modbus/dataanalysis_modbus.c"
CPU2_PARAM_C = ROOT / "LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.c"
CPU2_MEASURE = ROOT / "LTD_MAIN_CPU2/Application/Src/measure.c"
CPU2_DENSITY = ROOT / "LTD_MAIN_CPU2/Application/Src/measure_density.c"
CPU3_SI = ROOT / "LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.c"
CPU3_SI_H = ROOT / "LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.h"
CPU3_LOCAL_H = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.h"
CPU3_LOCAL_C = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.c"
CPU3_DISPLAY = ROOT / "LTD_DISPLAY_CPU3/Application/display/display_tankopera.c"
CPU3_DISPLAY_H = ROOT / "LTD_DISPLAY_CPU3/Application/display/display_tankopera.h"

# SI 需要的补充状态融合进已有测量结构，不再维护独立 ProtocolAssistStatus。
EXPECTED_FIELD_GROUPS = {
    "DeviceStatus": [
        "loading_unloading_active",
        "manual_alarm_inhibit",
    ],
    "OilMeasurement": [
        "probe_at_liquid_level",
        "liquid_stable",
        "manual_level_update_inhibit",
    ],
    "ActualHeightMeasurement": [
        "bottom_reference_valid",
    ],
    "DensityDistribution": [
        "profile_complete_latched",
        "profile_complete_counter",
        "profile_source",
        "profile_blocked_by_process",
        "profile_temp_deviation_alarm",
        "profile_density_deviation_alarm",
    ],
}

EXPECTED_PAIRS = [
    ("REG_HEIGHT_MEASUREMENT_BOTTOM_REFERENCE_VALID", "height_measurement.bottom_reference_valid"),
    ("REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL", "oil_measurement.probe_at_liquid_level"),
    ("REG_OIL_MEASUREMENT_LIQUID_STABLE", "oil_measurement.liquid_stable"),
    ("REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED", "density_distribution.profile_complete_latched"),
    ("REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER", "density_distribution.profile_complete_counter"),
    ("REG_DENSITY_DIST_PROFILE_SOURCE", "density_distribution.profile_source"),
    ("REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS", "density_distribution.profile_blocked_by_process"),
    ("REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE", "device_status.loading_unloading_active"),
    ("REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT", "device_status.manual_alarm_inhibit"),
    ("REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT", "oil_measurement.manual_level_update_inhibit"),
    ("REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM", "density_distribution.profile_temp_deviation_alarm"),
    ("REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM", "density_distribution.profile_density_deviation_alarm"),
]

EXPECTED_REGS = [reg for reg, _field in EXPECTED_PAIRS]

EXPECTED_SI_PROFILE_FIELDS = [
    "si_profile_first_point",
    "si_profile_increment",
    "si_profile_dwell_time",
    "si_profile_bottom_detect_interval",
]

EXPECTED_SI_PROFILE_REGS = [
    "HOLDREGISTER_DEVICEPARAM_SI_PROFILE_FIRST_POINT",
    "HOLDREGISTER_DEVICEPARAM_SI_PROFILE_INCREMENT",
    "HOLDREGISTER_DEVICEPARAM_SI_PROFILE_DWELL_TIME",
    "HOLDREGISTER_DEVICEPARAM_SI_PROFILE_BOTTOM_DETECT_INTERVAL",
]

EXPECTED_CPU3_SI_FIELDS = [
    "si_auto_profile_interval",
    "si_auto_profile_enable",
    "si_auto_profile_hour",
    "si_auto_profile_minute",
    "si_low_density_setpoint",
    "si_high_density_setpoint",
    "si_low_temperature_setpoint",
    "si_high_temperature_setpoint",
    "si_ll_level_setpoint",
    "si_hh_level_setpoint",
    "si_low_level_setpoint",
    "si_high_level_setpoint",
    "si_temp_deviation_setpoint",
    "si_density_deviation_setpoint",
]


def read_text(path: Path) -> str:
    """读取 CPU2/CPU3 源文件，兼容 CPU2 历史 GBK 和新增 UTF-8 文件。"""

    data = path.read_bytes()
    # CPU2 旧源码可能是 GBK，CPU3/工具通常是 UTF-8；脚本同时支持两种编码。
    for encoding in ("utf-8-sig", "gbk"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", data, 0, 1, f"cannot decode {path}")


def parse_protocol_version(text: str, path: Path) -> int:
    """提取 DEVICE_PROTOCOL_VERSION，缺失时直接判定协议契约不完整。"""

    match = re.search(r"#define\s+DEVICE_PROTOCOL_VERSION\s+(\d+)u?\b", text)
    if not match:
        raise AssertionError(f"{path}: missing DEVICE_PROTOCOL_VERSION")
    return int(match.group(1))


def parse_struct_fields(text: str, path: Path, struct_name: str) -> list[str]:
    """按声明顺序提取结构体里的 uint32_t 字段名，用于检查 CPU2/CPU3 布局一致。"""

    end_marker = f"}} {struct_name};"
    end = text.find(end_marker)
    if end < 0:
        raise AssertionError(f"{path}: missing {struct_name}")
    start = text.rfind("typedef struct", 0, end)
    if start < 0:
        raise AssertionError(f"{path}: missing {struct_name} typedef")
    body_start = text.find("{", start, end)
    if body_start < 0:
        raise AssertionError(f"{path}: malformed {struct_name}")
    # 只提取字段名，不比较注释内容，避免中文注释调整影响协议契约。
    return re.findall(r"\buint32_t\s+([A-Za-z_][A-Za-z0-9_]*)\s*;", text[body_start + 1:end])


def normalize_expr(expr: str) -> str:
    """去掉宏表达式中的空白，使 CPU2/CPU3 地址链对比不受排版影响。"""

    return re.sub(r"\s+", "", expr)


def strip_c_comments(expr: str) -> str:
    """移除宏同一行中的 C 注释，避免注释风格影响寄存器表达式比较。"""

    expr = re.sub(r"/\*.*?\*/", "", expr)
    return expr.split("//", 1)[0]


def parse_protocol_regs(text: str, path: Path) -> dict[str, str]:
    """提取 SI 共享状态依赖的寄存器宏及其地址表达式。"""

    regs: dict[str, str] = {}
    # 保留表达式文本用于 CPU2/CPU3 对比，确认两边不仅字段名一致，地址链也一致。
    for match in re.finditer(r"#define\s+(REG_(?:DEVICE_STATUS|OIL_MEASUREMENT|HEIGHT_MEASUREMENT|DENSITY_DIST)_[A-Z0-9_]+)\s+(.+)", text):
        regs[match.group(1)] = normalize_expr(strip_c_comments(match.group(2)).strip())
    missing = [name for name in EXPECTED_REGS if name not in regs]
    if missing:
        raise AssertionError(f"{path}: missing register defines: {', '.join(missing)}")
    return regs


def parse_write_pairs(text: str) -> list[tuple[str, str]]:
    """提取共享状态写入顺序，确保 CPU2/CPU3 的打包顺序一致。"""

    # 写寄存器顺序必须与字段表一致，CPU2 和 CPU3 两侧都由该表达式检查。
    pattern = re.compile(
        r"write_u32_to_regs\(\s*regs\s*,\s*"
        r"(REG_(?:DEVICE_STATUS|OIL_MEASUREMENT|HEIGHT_MEASUREMENT|DENSITY_DIST)_[A-Z0-9_]+)\s*,\s*"
        r"g_measurement\.([a-z0-9_]+\.[a-z0-9_]+)\s*\)",
        re.S,
    )
    return [pair for pair in pattern.findall(text) if pair[0] in EXPECTED_REGS]


def parse_read_pairs(text: str) -> list[tuple[str, str]]:
    """提取共享状态读回顺序，防止只改写入不改读取造成调试值错位。"""

    # 读回顺序单独检查，防止只改写入不改读取导致本地调试值错位。
    pattern = re.compile(
        r"g_measurement\.([a-z0-9_]+\.[a-z0-9_]+)\s*=\s*"
        r"read_u32_from_regs\(\s*cregs\s*,\s*(REG_(?:DEVICE_STATUS|OIL_MEASUREMENT|HEIGHT_MEASUREMENT|DENSITY_DIST)_[A-Z0-9_]+)\s*\)",
        re.S,
    )
    return [(reg, field) for field, reg in pattern.findall(text) if reg in EXPECTED_REGS]


def assert_equal(name: str, left: object, right: object) -> None:
    """带上下文的相等断言，失败信息直接指向不一致的契约项。"""

    if left != right:
        raise AssertionError(f"{name} mismatch:\nleft={left!r}\nright={right!r}")


def require_re(text: str, pattern: str, description: str, path: Path) -> None:
    """要求源码满足一个正则契约，失败时直接指出业务口径。"""

    if re.search(pattern, text, re.S) is None:
        raise AssertionError(f"{path}: missing {description}")


def reject_re(text: str, pattern: str, description: str, path: Path) -> None:
    """禁止重新引入已确认废弃的 SI协议兼容路径。"""

    if re.search(pattern, text, re.S) is not None:
        raise AssertionError(f"{path}: forbidden {description}")


def check_param_headers() -> None:
    """检查 CPU2/CPU3 参数头文件中的协议版本和共享结构体字段布局。"""

    cpu2_text = read_text(CPU2_PARAM)
    cpu3_text = read_text(CPU3_PARAM)

    assert_equal("DEVICE_PROTOCOL_VERSION", parse_protocol_version(cpu2_text, CPU2_PARAM), parse_protocol_version(cpu3_text, CPU3_PARAM))
    if parse_protocol_version(cpu2_text, CPU2_PARAM) < 14:
        raise AssertionError("DEVICE_PROTOCOL_VERSION must be bumped for CMD_SI_PROFILE and SI profile params")
    if "ProtocolAssistStatus" in cpu2_text or "ProtocolAssistStatus" in cpu3_text:
        raise AssertionError("ProtocolAssistStatus should be merged into existing measurement structs")
    for struct_name, expected_fields in EXPECTED_FIELD_GROUPS.items():
        cpu2_fields = parse_struct_fields(cpu2_text, CPU2_PARAM, struct_name)
        cpu3_fields = parse_struct_fields(cpu3_text, CPU3_PARAM, struct_name)
        assert_equal(f"{struct_name} required fields", [field for field in cpu2_fields if field in expected_fields], expected_fields)
        assert_equal(f"CPU2/CPU3 {struct_name} fields", cpu2_fields, cpu3_fields)

    cpu2_device_params = parse_struct_fields(cpu2_text, CPU2_PARAM, "DeviceParameters")
    cpu3_device_params = parse_struct_fields(cpu3_text, CPU3_PARAM, "DeviceParameters")
    assert_equal("CPU2 SI profile params", [field for field in cpu2_device_params if field in EXPECTED_SI_PROFILE_FIELDS], EXPECTED_SI_PROFILE_FIELDS)
    assert_equal("CPU3 SI profile params", [field for field in cpu3_device_params if field in EXPECTED_SI_PROFILE_FIELDS], EXPECTED_SI_PROFILE_FIELDS)
    assert_equal("CPU2/CPU3 DeviceParameters fields", cpu2_device_params, cpu3_device_params)
    require_re(cpu2_text, r"CMD_SI_PROFILE\s*=\s*20\b", "CMD_SI_PROFILE = 20", CPU2_PARAM)
    require_re(cpu3_text, r"CMD_SI_PROFILE\s*=\s*20\b", "CMD_SI_PROFILE = 20", CPU3_PARAM)


def check_state_headers() -> None:
    """检查 CPU2/CPU3 共享输入寄存器宏表达式和新增字段跟随关系。"""

    cpu2_regs = parse_protocol_regs(read_text(CPU2_STATE), CPU2_STATE)
    cpu3_regs = parse_protocol_regs(read_text(CPU3_STATE), CPU3_STATE)
    cpu2_text = read_text(CPU2_STATE)
    cpu3_text = read_text(CPU3_STATE)

    for index, name in enumerate(EXPECTED_REGS):
        assert_equal(name, cpu2_regs[name], cpu3_regs[name])
        expr = cpu2_regs[name]
        expected_prev = {
            "REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE": "REG_DEVICE_STATUS_PARAM_UPDATE_FLAG",
            "REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT": "REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE",
            "REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL": "REG_OIL_MEASUREMENT_CURRENT_FREQUENCY",
            "REG_OIL_MEASUREMENT_LIQUID_STABLE": "REG_OIL_MEASUREMENT_PROBE_AT_LIQUID_LEVEL",
            "REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT": "REG_OIL_MEASUREMENT_LIQUID_STABLE",
            "REG_HEIGHT_MEASUREMENT_BOTTOM_REFERENCE_VALID": "REG_HEIGHT_MEASUREMENT_CURRENT_REAL",
            "REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED": "REG_DENSITY_DIST_OIL_LEVEL",
            "REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER": "REG_DENSITY_DIST_PROFILE_COMPLETE_LATCHED",
            "REG_DENSITY_DIST_PROFILE_SOURCE": "REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER",
            "REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS": "REG_DENSITY_DIST_PROFILE_SOURCE",
            "REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM": "REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS",
            "REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM": "REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM",
        }[name]
        if expected_prev not in expr or "REG_SIZE_U32" not in expr:
            raise AssertionError(f"{CPU2_STATE}: {name} should follow {expected_prev}")

    for name in EXPECTED_SI_PROFILE_REGS:
        require_re(cpu2_text, rf"\b{name}\b", f"{name}", CPU2_STATE)
        require_re(cpu3_text, rf"\b{name}\b", f"{name}", CPU3_STATE)


def check_modbus_pack_unpack() -> None:
    """检查 CPU2/CPU3 Modbus 打包和回读代码是否按同一字段顺序处理。"""

    cpu2_text = read_text(CPU2_MODBUS)
    cpu3_text = read_text(CPU3_MODBUS)

    assert_equal(f"{CPU2_MODBUS} write merged SI status order", parse_write_pairs(cpu2_text), EXPECTED_PAIRS)
    assert_equal(f"{CPU3_MODBUS} read merged SI status order", parse_read_pairs(cpu3_text), EXPECTED_PAIRS)

    # CPU2 的输入寄存器反向读取和 CPU3 的测量结果写入是已确认删除的死接口；
    # 如果后续重新引入这些调试方向，再校验它们的字段顺序。
    if "read_measurement_result_from_InputRegisters" in cpu2_text:
        assert_equal(f"{CPU2_MODBUS} read merged SI status order", parse_read_pairs(cpu2_text), EXPECTED_PAIRS)
    if "write_measurement_result_to_InputRegisters" in cpu3_text:
        assert_equal(f"{CPU3_MODBUS} write merged SI status order", parse_write_pairs(cpu3_text), EXPECTED_PAIRS)


def check_si_profile_contract() -> None:
    """检查 SI协议进一步兼容的命令、参数、状态和调度入口。"""

    cpu2_measure = read_text(CPU2_MEASURE)
    cpu2_density = read_text(CPU2_DENSITY)
    cpu2_param_c = read_text(CPU2_PARAM_C)
    cpu3_si = read_text(CPU3_SI)
    cpu3_si_h = read_text(CPU3_SI_H)
    cpu3_local_h = read_text(CPU3_LOCAL_H)
    cpu3_local_c = read_text(CPU3_LOCAL_C)
    cpu3_display = read_text(CPU3_DISPLAY)
    cpu3_display_h = read_text(CPU3_DISPLAY_H)

    require_re(cpu3_si, r"SI_TEMP_INVALID_REGISTER\s+0xB1E0U", "SI invalid temperature register 0xB1E0", CPU3_SI)
    reject_re(cpu3_si, r"return\s+0U\s*;\s*\}\s*return\s+si_clamp_s16", "invalid temperature returning zero", CPU3_SI)
    reject_re(cpu3_si, r"setpoint\s*!=\s*0U", "zero alarm setpoint disables alarm", CPU3_SI)
    reject_re(cpu3_si, r"SI_HR_TEMP_DEVIATION_SETPOINT\]\s*!=\s*0U", "zero temperature deviation setpoint disables alarm", CPU3_SI)
    reject_re(cpu3_si, r"SI_HR_DENSITY_DEVIATION_SETPOINT\]\s*!=\s*0U", "zero density deviation setpoint disables alarm", CPU3_SI)
    require_re(cpu3_local_h, r"int16_t\s+si_low_temperature_setpoint\s*;", "signed SI low temperature setpoint", CPU3_LOCAL_H)
    require_re(cpu3_local_h, r"int16_t\s+si_high_temperature_setpoint\s*;", "signed SI high temperature setpoint", CPU3_LOCAL_H)
    require_re(cpu3_local_c, r"Cpu3_ClampS16Param\s*\(", "signed SI temperature clamp helper", CPU3_LOCAL_C)
    require_re(cpu3_si, r"si_holding_to_s16\s*\(", "signed SI temperature holding conversion", CPU3_SI)
    require_re(cpu3_si, r"si_absdiff_s16\s*\(", "signed SI profile temperature deviation difference", CPU3_SI)
    reject_re(cpu3_si, r"si_absdiff_u16\s*\(\s*si_temp_raw_to_si_s16", "unsigned SI profile temperature deviation difference", CPU3_SI)

    require_re(cpu2_measure, r"case\s+CMD_SI_PROFILE\s*:", "CPU2 SI profile command dispatch", CPU2_MEASURE)
    require_re(cpu2_density, r"CMD_SiProfile\s*\(", "CPU2 SI profile implementation", CPU2_DENSITY)
    require_re(cpu2_density, r"PROFILE_SOURCE_STANDARD", "CPU2 marks standard profile source", CPU2_DENSITY)
    require_re(cpu2_density, r"PROFILE_SOURCE_GB", "CPU2 marks GB profile source", CPU2_DENSITY)
    require_re(cpu2_density, r"PROFILE_SOURCE_METER", "CPU2 marks meter profile source", CPU2_DENSITY)
    require_re(cpu2_density, r"PROFILE_SOURCE_INTERVAL", "CPU2 marks interval profile source", CPU2_DENSITY)
    require_re(cpu2_density, r"PROFILE_SOURCE_SI", "CPU2 marks SI profile source", CPU2_DENSITY)
    require_re(cpu2_measure, r"PROFILE_SOURCE_WARTSILA", "CPU2 marks Wartsila profile source", CPU2_MEASURE)
    fallback_match = re.search(
        r"if\s*\(\s*s_si_profile_bottom_ref_valid\s*!=\s*0U\s*\)\s*\{"
        r".*?return\s+s_si_profile_bottom_position_01mm\s*;\s*"
        r"\}(?P<fallback>.*?)return\s+current_position\s*;",
        cpu2_density,
        re.S,
    )
    if fallback_match is None:
        raise AssertionError(f"{CPU2_DENSITY}: missing SI no-old-bottom fallback branch")
    fallback_body = fallback_match.group("fallback")
    if re.search(r"s_si_profile_bottom_position_01mm\s*=", fallback_body):
        raise AssertionError(f"{CPU2_DENSITY}: no-old-bottom fallback must not update saved bottom position")
    if re.search(r"s_si_profile_bottom_ref_valid\s*=\s*1U\s*;", fallback_body):
        raise AssertionError(f"{CPU2_DENSITY}: no-old-bottom fallback must not mark bottom reference valid")
    if re.search(r"s_si_profile_first_run\s*=\s*0U\s*;", fallback_body):
        raise AssertionError(f"{CPU2_DENSITY}: no-old-bottom fallback must keep first-run bottom detection active")
    cmd_match = re.search(r"void\s+CMD_SiProfile\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}", cpu2_density, re.S)
    if cmd_match is None:
        raise AssertionError(f"{CPU2_DENSITY}: missing CMD_SiProfile body")
    if "SearchOilLevel(" in cmd_match.group("body"):
        raise AssertionError(f"{CPU2_DENSITY}: SI profile must not pre-run SearchOilLevel before profile")
    reject_re(cpu2_density, r"SiProfile_BuildPoints\s*\([^)]*oil_level_01mm", "SI point generation clipped by pre-found oil level", CPU2_DENSITY)
    reject_re(cpu2_density, r"target\s*<=\s*oil_level", "SI point generation depends on pre-found oil level", CPU2_DENSITY)
    build_points_match = re.search(
        r"static\s+uint32_t\s+SiProfile_BuildPoints\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        cpu2_density,
        re.S,
    )
    if build_points_match is None:
        raise AssertionError(f"{CPU2_DENSITY}: missing SiProfile_BuildPoints body")
    build_points_body = build_points_match.group("body")
    if re.search(r"target\s*=\s*bottom\s*\+\s*\(int64_t\)\s*first_point_01mm\s*;", build_points_body):
        raise AssertionError(
            f"{CPU2_DENSITY}: SI 40001 Profile First Point must be absolute, not bottom-relative"
        )
    require_re(
        build_points_body,
        r"target\s*=\s*\(int64_t\)\s*first_point_01mm\s*;",
        "SI 40001 Profile First Point is used as an absolute point",
        CPU2_DENSITY,
    )
    reject_re(
        build_points_body,
        r"while\s*\(\s*target\s*<=\s*bottom\s*\)",
        "SI configured first point skipped by real bottom position",
        CPU2_DENSITY,
    )
    require_re(cpu2_density, r"SiProfile_RunPoints01mmWithDwell\s*\(", "SI runtime point runner", CPU2_DENSITY)
    require_re(cpu2_density, r"SiProfile_ReportPosition01mm\s*\(", "SI output position coordinate mapping", CPU2_DENSITY)
    require_re(
        cpu2_density,
        r"if\s*\(\s*point_index\s*==\s*0U\s*\)\s*\{[\s\S]{0,80}return\s+0U\s*;",
        "SI Point0 reported position is fixed to zero",
        CPU2_DENSITY,
    )
    require_re(
        cpu2_density,
        r"return\s+Density_ValueToU01mmClamped\s*\(\s*movement_position_01mm\s*,\s*\"SI Profile position\"\s*\)\s*;",
        "SI nonzero points report movement position",
        CPU2_DENSITY,
    )
    require_re(
        cpu2_density,
        r"single_density_data\[valid\]\.temperature_position\s*=\s*report_position_01mm\s*;",
        "SI profile output point position uses reported coordinate",
        CPU2_DENSITY,
    )
    require_re(cpu2_density, r"SiProfile_ReadPointAndClassify\s*\(", "SI runtime above-liquid detection", CPU2_DENSITY)
    si_read_match = re.search(
        r"static\s+uint32_t\s+SiProfile_ReadPointAndClassify\s*\([^)]*\)\s*\{(?P<body>.*?)\n\}",
        cpu2_density,
        re.S,
    )
    if si_read_match is None:
        raise AssertionError(f"{CPU2_DENSITY}: missing SiProfile_ReadPointAndClassify body")
    si_read_body = si_read_match.group("body")
    air_index = si_read_body.find("SiProfile_IsAirPoint(cur_density, cur_freq, &air_reason)")
    invalid_freq_index = si_read_body.find("if (cur_freq <= 0.0f)")
    if (air_index < 0) or (invalid_freq_index < 0) or (air_index > invalid_freq_index):
        raise AssertionError(
            f"{CPU2_DENSITY}: SI profile must classify density-low air before waiting on invalid frequency"
        )
    require_re(cpu2_density, r"SiProfile_AdvanceBottomDetectTriggerCount\s*\(", "SI bottom-detect interval counts accepted profile triggers", CPU2_DENSITY)
    require_re(
        cpu2_density,
        r"should_detect_bottom\s*=\s*SiProfile_ShouldDetectBottom\s*\([^;]+;\s*"
        r"SiProfile_AdvanceBottomDetectTriggerCount\s*\(\s*should_detect_bottom\s*\)\s*;\s*"
        r"if\s*\(\s*should_detect_bottom\s*!=\s*0U\s*\)",
        "SI bottom-detect trigger count advances before bottom search",
        CPU2_DENSITY,
    )
    reject_re(cpu2_density, r"bottom_search_ret\s*==\s*NO_ERROR[\s\S]{0,260}s_si_profile_count_since_bottom\s*=\s*0U\s*;", "successful SI bottom search resetting trigger counter", CPU2_DENSITY)
    reject_re(
        cpu2_density,
        r"g_measurement\.density_distribution\.profile_source\s*=\s*PROFILE_SOURCE_SI\s*;"
        r"\s*g_measurement\.density_distribution\.profile_blocked_by_process\s*=\s*0U\s*;"
        r"\s*if\s*\(\s*should_detect_bottom\s*!=\s*0U\s*\)\s*\{\s*s_si_profile_count_since_bottom\s*=\s*1U\s*;",
        "SI bottom-detect interval counted only after successful profile",
        CPU2_DENSITY,
    )
    require_re(cpu2_param_c, r"si_profile_first_point\s*=\s*1000U?", "SI first point default", CPU2_PARAM_C)
    require_re(cpu2_param_c, r"si_profile_increment\s*=\s*10000U?", "SI increment default", CPU2_PARAM_C)
    require_re(cpu2_param_c, r"si_profile_dwell_time\s*=\s*10U?", "SI dwell default", CPU2_PARAM_C)
    require_re(cpu2_param_c, r"si_profile_bottom_detect_interval\s*=\s*1U?", "SI bottom detect interval default", CPU2_PARAM_C)

    require_re(cpu3_si, r"SI_COIL_PROFILE:[\s\S]{0,200}si_profile_request_start\s*\(\s*\)", "Profile coil requests CMD_SI_PROFILE through shared API", CPU3_SI)
    reject_re(cpu3_si, r"SI_COIL_PROFILE:[\s\S]{0,200}CMD_MEASURE_DISTRIBUTED", "Profile coil bridging to CMD_MEASURE_DISTRIBUTED", CPU3_SI)
    require_re(cpu3_si, r"current_command\s*==\s*CMD_SI_PROFILE", "Profile coil state is gated by SI profile command", CPU3_SI)
    require_re(cpu3_si_h, r"void\s+si_profile_request_start\s*\(\s*void\s*\)\s*;", "shared SI profile request API", CPU3_SI_H)
    require_re(cpu3_si, r"void\s+si_profile_request_start\s*\(\s*void\s*\)\s*\{[\s\S]{0,220}si_lock_profile_timestamp_now\s*\(\s*\)\s*;[\s\S]{0,120}si_send_cpu2_command\s*\(\s*CMD_SI_PROFILE\s*\)", "shared SI profile request locks timestamp and sends command", CPU3_SI)
    require_re(cpu3_display, r"#include\s+\"si_modbus_slave\.h\"", "screen can call shared SI profile request API", CPU3_DISPLAY)
    require_re(cpu3_display_h, r"\bCOM_NUM_SI_PROFILE\b", "screen operation number for SI profile command", CPU3_DISPLAY_H)
    require_re(cpu3_display, r"\{\s*\(uint8_t\*\)\"SI Profile\"\s*,\s*COM_NUM_SI_PROFILE\s*,\s*ifsendcmd", "screen SI profile command menu item", CPU3_DISPLAY)
    require_re(cpu3_display, r"\{\s*COM_NUM_SI_PROFILE\s*,\s*CMD_SI_PROFILE\s*\}", "screen SI profile command maps to CMD_SI_PROFILE", CPU3_DISPLAY)
    require_re(cpu3_display, r"now_Opera_Num\s*==\s*COM_NUM_SI_PROFILE[\s\S]{0,140}si_profile_request_start\s*\(\s*\)", "screen SI profile locks timestamp before command", CPU3_DISPLAY)
    require_re(cpu3_display, r"case\s+COM_NUM_SI_PROFILE\s*:[\s\S]{0,160}return\s+menu_measure_density_distribution", "screen SI profile returns to density distribution menu", CPU3_DISPLAY)
    reject_re(cpu3_si, r"SI_HR_PROFILE_FIRST_POINT:[\s\S]{0,180}spreadTopLimit", "40001 writes spreadTopLimit", CPU3_SI)
    reject_re(cpu3_si, r"SI_HR_PROFILE_INCREMENT:[\s\S]{0,180}spreadMeasurementDistance", "40002 writes spreadMeasurementDistance", CPU3_SI)
    reject_re(cpu3_si, r"SI_HR_PROFILE_DWELL_TIME:[\s\S]{0,180}spreadPointHoverTime", "40003 writes spreadPointHoverTime", CPU3_SI)
    require_re(cpu3_si, r"SI_HR_AUTO_PROFILE_INTERVAL:[\s\S]{0,120}value\s*!=\s*0U", "40010 rejects zero interval", CPU3_SI)
    require_re(cpu3_si, r"s_discrete_inputs\[SI_DI_PROBE_UNCALIBRATED\]\s*=\s*0U\s*;", "Probe Un-calibrated fixed trusted", CPU3_SI)
    require_re(cpu3_si, r"si_is_si_profile_result_valid\s*\(", "SI profile result source gate", CPU3_SI)
    reject_re(cpu3_si, r"STATE_WARTSILA_DENSITY_OVER:[\s\S]{0,120}s_discrete_inputs\[SI_DI_PROFILE_COMPLETE\]\s*=\s*1U", "generic profile-over state forcing SI profile complete", CPU3_SI)
    require_re(cpu3_si, r"si_lock_profile_timestamp_now\s*\(", "profile start timestamp latch", CPU3_SI)
    require_re(cpu3_si, r"si_modbus_periodic_task\s*\(", "SI automatic profile scheduler tick", CPU3_SI)
    require_re(cpu3_si, r"si_is_leap_year\s*\(", "real calendar schedule conversion", CPU3_SI)
    reject_re(cpu3_si, r"\*\s*372U", "synthetic 31-day month schedule index", CPU3_SI)

    for field in EXPECTED_CPU3_SI_FIELDS:
        require_re(cpu3_local_h, rf"\b{field}\b", f"CPU3 local {field}", CPU3_LOCAL_H)
        require_re(cpu3_local_c, rf"\b{field}\b", f"CPU3 local {field} storage/read/write", CPU3_LOCAL_C)
    require_re(cpu3_local_c, r"CPU3_PARAM_VERSION\s+0x0006U", "CPU3 local parameter storage version V6", CPU3_LOCAL_C)


def main() -> int:
    """命令行入口，执行全部 CPU2/CPU3 共享协议契约检查。"""

    try:
        check_param_headers()
        check_state_headers()
        check_modbus_pack_unpack()
        check_si_profile_contract()
    except AssertionError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1

    print("[OK] SI CPU2/CPU3 shared protocol contract is consistent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

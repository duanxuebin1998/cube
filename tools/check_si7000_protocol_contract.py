#!/usr/bin/env python3
"""检查 CPU2/CPU3 的 SI7000 共享协议契约是否一致。"""

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

# SI7000 需要的补充状态融合进已有测量结构，不再维护独立 ProtocolAssistStatus。
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
    ("REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS", "density_distribution.profile_blocked_by_process"),
    ("REG_DEVICE_STATUS_LOADING_UNLOADING_ACTIVE", "device_status.loading_unloading_active"),
    ("REG_DEVICE_STATUS_MANUAL_ALARM_INHIBIT", "device_status.manual_alarm_inhibit"),
    ("REG_OIL_MEASUREMENT_MANUAL_LEVEL_UPDATE_INHIBIT", "oil_measurement.manual_level_update_inhibit"),
    ("REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM", "density_distribution.profile_temp_deviation_alarm"),
    ("REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM", "density_distribution.profile_density_deviation_alarm"),
]

EXPECTED_REGS = [reg for reg, _field in EXPECTED_PAIRS]


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


def parse_protocol_regs(text: str, path: Path) -> dict[str, str]:
    """提取 SI7000 共享状态依赖的寄存器宏及其地址表达式。"""

    regs: dict[str, str] = {}
    # 保留表达式文本用于 CPU2/CPU3 对比，确认两边不仅字段名一致，地址链也一致。
    for match in re.finditer(r"#define\s+(REG_(?:DEVICE_STATUS|OIL_MEASUREMENT|HEIGHT_MEASUREMENT|DENSITY_DIST)_[A-Z0-9_]+)\s+(.+)", text):
        regs[match.group(1)] = normalize_expr(match.group(2).split("//", 1)[0].strip())
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


def check_param_headers() -> None:
    """检查 CPU2/CPU3 参数头文件中的协议版本和共享结构体字段布局。"""

    cpu2_text = read_text(CPU2_PARAM)
    cpu3_text = read_text(CPU3_PARAM)

    assert_equal("DEVICE_PROTOCOL_VERSION", parse_protocol_version(cpu2_text, CPU2_PARAM), parse_protocol_version(cpu3_text, CPU3_PARAM))
    if "ProtocolAssistStatus" in cpu2_text or "ProtocolAssistStatus" in cpu3_text:
        raise AssertionError("ProtocolAssistStatus should be merged into existing measurement structs")
    for struct_name, expected_fields in EXPECTED_FIELD_GROUPS.items():
        cpu2_fields = parse_struct_fields(cpu2_text, CPU2_PARAM, struct_name)
        cpu3_fields = parse_struct_fields(cpu3_text, CPU3_PARAM, struct_name)
        assert_equal(f"{struct_name} required fields", [field for field in cpu2_fields if field in expected_fields], expected_fields)
        assert_equal(f"CPU2/CPU3 {struct_name} fields", cpu2_fields, cpu3_fields)


def check_state_headers() -> None:
    """检查 CPU2/CPU3 共享输入寄存器宏表达式和新增字段跟随关系。"""

    cpu2_regs = parse_protocol_regs(read_text(CPU2_STATE), CPU2_STATE)
    cpu3_regs = parse_protocol_regs(read_text(CPU3_STATE), CPU3_STATE)

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
            "REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS": "REG_DENSITY_DIST_PROFILE_COMPLETE_COUNTER",
            "REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM": "REG_DENSITY_DIST_PROFILE_BLOCKED_BY_PROCESS",
            "REG_DENSITY_DIST_PROFILE_DENSITY_DEVIATION_ALARM": "REG_DENSITY_DIST_PROFILE_TEMP_DEVIATION_ALARM",
        }[name]
        if expected_prev not in expr or "REG_SIZE_U32" not in expr:
            raise AssertionError(f"{CPU2_STATE}: {name} should follow {expected_prev}")


def check_modbus_pack_unpack() -> None:
    """检查 CPU2/CPU3 Modbus 打包和回读代码是否按同一字段顺序处理。"""

    for path in (CPU2_MODBUS, CPU3_MODBUS):
        text = read_text(path)
        assert_equal(f"{path} write merged SI7000 status order", parse_write_pairs(text), EXPECTED_PAIRS)
        assert_equal(f"{path} read merged SI7000 status order", parse_read_pairs(text), EXPECTED_PAIRS)


def main() -> int:
    """命令行入口，执行全部 CPU2/CPU3 共享协议契约检查。"""

    try:
        check_param_headers()
        check_state_headers()
        check_modbus_pack_unpack()
    except AssertionError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1

    print("[OK] SI7000 CPU2/CPU3 shared protocol contract is consistent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

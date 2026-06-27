#!/usr/bin/env python3
"""检查密度两位小数改造契约。

该脚本做轻量级源码契约验证，覆盖内部 x100、旧外部 x10、CPU2 迁移、
CPU3 菜单/本机参数迁移和显示小数位，避免后续改动重新引入 10 倍错位。
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

CPU2_PARAM_H = ROOT / "LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h"
CPU2_PARAM_C = ROOT / "LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.c"
CPU2_SENSOR_C = ROOT / "LTD_MAIN_CPU2/Services/Sensor/sensor.c"
CPU2_OIL_LEVEL_C = ROOT / "LTD_MAIN_CPU2/Application/Src/measure_oilLevel.c"

CPU3_PARAM_H = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h"
CPU3_PARAM_C = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/system_parameter.c"
CPU3_LOCAL_PARAM_C = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.c"
CPU3_DEVICE_SYNC_C = ROOT / "LTD_DISPLAY_CPU3/Communication/internal/main_board_modbus/device_param_sync.c"
CPU3_DISPLAY_C = ROOT / "LTD_DISPLAY_CPU3/Application/display/display.c"
DSM_C = ROOT / "LTD_DISPLAY_CPU3/Communication/external/DSM_modbus/DSM_DataAnalysis_modbus2.c"
WARTSILA_C = ROOT / "LTD_DISPLAY_CPU3/Communication/external/wartsila_modbus/wartsila_modbus_data_analysis.c"
SI7000_C = ROOT / "LTD_DISPLAY_CPU3/Communication/external/si7000_modbus/si7000_modbus_slave.c"
PROTOCOL_DOC = ROOT / "docs/01_协议与寄存器/CPU2_CPU3协议变更记录.md"


def read(path: Path, encoding: str = "utf-8") -> str:
    return path.read_text(encoding=encoding)


def require(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def require_re(text: str, pattern: str, message: str, errors: list[str]) -> None:
    require(re.search(pattern, text, re.S) is not None, message, errors)


def extract_function_body(source: str, name: str) -> str:
    match = re.search(r"\b(?:static\s+)?(?:void|int|uint32_t|int32_t)\s+" + re.escape(name) + r"\s*\([^)]*\)\s*\{", source)
    if match is None:
        return ""

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
    return ""


def extract_macro_int(text: str, name: str) -> int | None:
    match = re.search(r"#define\s+" + re.escape(name) + r"\s+([0-9]+)[uU]?\b", text)
    if match is None:
        return None
    return int(match.group(1))


def extract_macro_hex(text: str, name: str) -> int | None:
    match = re.search(r"#define\s+" + re.escape(name) + r"\s+0x([0-9A-Fa-f]+)U\b", text)
    if match is None:
        return None
    return int(match.group(1), 16)


def normalize(text: str) -> str:
    return re.sub(r"\s+", "", text)


def require_nearby(text: str, anchor: str, tokens: tuple[str, ...], message: str, errors: list[str], window: int = 1000) -> None:
    index = text.find(anchor)
    if index < 0:
        errors.append(message)
        return
    snippet = text[index:index + window]
    for token in tokens:
        if token not in snippet:
            errors.append(message)
            return


def check_numeric_examples(errors: list[str]) -> None:
    density_migrate_factor = 10
    old_density_raw = 8321
    new_density_raw = old_density_raw * density_migrate_factor
    require(new_density_raw == 83210, "832.1 kg/m3 must migrate from x10 8321 to x100 83210", errors)

    raw_x100 = 83215
    external_x10 = (raw_x100 + (density_migrate_factor // 2)) // density_migrate_factor
    require(external_x10 == 8322, "external x10 density must round 832.15 kg/m3 to 832.2", errors)

    old_base = 10000
    new_base = 100000
    old_correction = 10025
    migrated_correction = new_base + ((old_correction - old_base) * density_migrate_factor)
    require(migrated_correction == 100250, "density correction +2.5 must migrate to x100 base 100250", errors)

    external_correction = 9995
    raw_correction = new_base + ((external_correction - old_base) * density_migrate_factor)
    require(raw_correction == 99950, "DSM correction external 9995 must convert to internal 99950", errors)


def check_cpu2_contract(cpu2_h: str, cpu2_c: str, sensor_c: str, oil_level_c: str, errors: list[str]) -> None:
    cpu2_protocol = extract_macro_int(cpu2_h, "DEVICE_PROTOCOL_VERSION")
    require(cpu2_protocol == 13, "CPU2 DEVICE_PROTOCOL_VERSION must be 13 for density x100 protocol", errors)

    expected_macros = {
        "DENSITY_RAW_SCALE": 100,
        "DENSITY_EXTERNAL_SCALE_X10": 10,
        "DENSITY_PARAM_MIGRATE_FACTOR": 10,
        "DENSITY_CORRECTION_OLD_BASE_RAW": 10000,
        "DENSITY_CORRECTION_BASE_RAW": 100000,
    }
    for name, value in expected_macros.items():
        require(extract_macro_int(cpu2_h, name) == value, f"CPU2 {name} must be {value}", errors)

    require_re(cpu2_h, r"#define\s+DENSITY_TO_RAW\(d\)\s+\(\(uint32_t\)\(\(\(d\)\s*\*\s*100\.0f\)\s*\+\s*0\.5f\)\)", "DENSITY_TO_RAW must convert kg/m3 to x100 with rounding", errors)
    require_re(cpu2_h, r"#define\s+RAW_TO_DENSITY\(raw\)\s+\(\(raw\)\s*/\s*100\.0f\)", "RAW_TO_DENSITY must convert x100 raw back to kg/m3", errors)

    require_re(cpu2_c, r"#define\s+DENSITY_X100_PROTOCOL_VERSION\s+\(13u\)", "CPU2 density migration gate must be fixed at protocol 13", errors)
    require_re(cpu2_c, r"if\s*\(\s*g_deviceParams\.protocolVersion\s*>=\s*DENSITY_X100_PROTOCOL_VERSION\s*\)\s*\{\s*return\s+0;", "CPU2 density migration must skip saved protocol >= 13", errors)
    require("old_protocol == DEVICE_PROTOCOL_VERSION" in cpu2_c, "CPU2 protocol normalization must compare against current protocol", errors)
    load_body = extract_function_body(cpu2_c, "load_device_params")
    require(load_body != "", "CPU2 load_device_params body must be found", errors)
    require(load_body.find("migrate_density_params_runtime()") < load_body.find("apply_protocol_version_runtime()"), "CPU2 must migrate old density values before writing current protocol version", errors)

    for field in ("oilLevelDensity", "oilLevelThreshold", "oilLevelHysteresisThreshold"):
        require_re(cpu2_c, rf"g_deviceParams\.{field}\s*=\s*density_param_scale_x10_to_x100\(g_deviceParams\.{field}\)", f"CPU2 must migrate {field} from x10 to x100", errors)
    require_re(cpu2_c, r"g_deviceParams\.densityCorrection\s*=\s*density_correction_scale_x10_to_x100\(g_deviceParams\.densityCorrection\)", "CPU2 must migrate densityCorrection base from 10000 to 100000", errors)
    require_re(sensor_c, r"densityCorrection\s*-\s*\(float\)DENSITY_CORRECTION_BASE_RAW\)\s*/\s*100\.0f", "CPU2 sensor density correction must use x100 base and divisor", errors)
    require_re(oil_level_c, r"RAW_TO_DENSITY\s*\(\s*g_deviceParams\.oilLevelDensity\s*\)", "CPU2 oil-level density target must use RAW_TO_DENSITY", errors)
    require_re(oil_level_c, r"RAW_TO_DENSITY\s*\(\s*raw_threshold\s*\)", "CPU2 oil-level density threshold must use RAW_TO_DENSITY", errors)
    require_re(oil_level_c, r"FrequencyLevel_GetCompatThresholdHz\s*\(", "CPU2 legacy frequency paths must keep threshold compatibility helper", errors)


def check_cpu3_contract(cpu2_h: str, cpu3_h: str, cpu3_param_c: str, local_param_c: str, sync_c: str, display_c: str, errors: list[str]) -> None:
    cpu2_protocol = extract_macro_int(cpu2_h, "DEVICE_PROTOCOL_VERSION")
    cpu3_protocol = extract_macro_int(cpu3_h, "DEVICE_PROTOCOL_VERSION")
    require(cpu3_protocol == 13, "CPU3 DEVICE_PROTOCOL_VERSION must be 13 for density x100 protocol", errors)
    require(cpu2_protocol == cpu3_protocol, "CPU2 and CPU3 protocol versions must match", errors)

    cpu3_param_version = extract_macro_hex(local_param_c, "CPU3_PARAM_VERSION")
    cpu3_param_v3 = extract_macro_hex(local_param_c, "CPU3_PARAM_VERSION_V3")
    cpu3_param_v4 = extract_macro_hex(local_param_c, "CPU3_PARAM_VERSION_V4")
    require(cpu3_param_version == 0x0005, "CPU3 local FRAM version must be 0x0005 for density input migration", errors)
    require(cpu3_param_v3 == 0x0003, "CPU3 must keep V3 migration source version", errors)
    require(cpu3_param_v4 == 0x0004, "CPU3 must keep V4 migration source version", errors)
    require_re(local_param_c, r"static\s+int32_t\s+Cpu3_MigrateDensityInputX10ToX100\s*\(", "CPU3 local density input migration helper must exist", errors)
    require_re(local_param_c, r"raw\s*\*\s*\(int32_t\)DENSITY_PARAM_MIGRATE_FACTOR", "CPU3 local density input migration must multiply by factor 10", errors)
    require_re(local_param_c, r"stor\.version\s*==\s*CPU3_PARAM_VERSION_V4[\s\S]{0,220}Cpu3_MigrateDensityInputX10ToX100", "CPU3 V4 load must migrate screen_input_d to x100", errors)
    require_re(local_param_c, r"stor\.version\s*==\s*CPU3_PARAM_VERSION_V3[\s\S]{0,500}Cpu3_MigrateDensityInputX10ToX100", "CPU3 V3 load must migrate screen_input_d to x100", errors)

    frequency_threshold_rows = {
        "液位找液阈值": "COM_NUM_DEVICEPARAM_OILLEVELTHRESHOLD",
        "液位滞后阈值": "COM_NUM_DEVICEPARAM_OILLEVEL_HYSTERESIS_THRESHOLD",
    }
    for zh_name, enum_name in frequency_threshold_rows.items():
        require_re(
            cpu3_param_c,
            rf'"{zh_name}"[\s\S]{{0,180}}{enum_name}[\s\S]{{0,220}}\(uint8_t\*\)"Hz",\s*1,\s*0',
            f"CPU3 parameter {zh_name} must display Hz with point=1",
            errors,
        )

    density_rows = {
        "液位跟随密度": "COM_NUM_DEVICEPARAM_OILLEVEL_DENSITY",
    }
    for zh_name, enum_name in density_rows.items():
        require_re(
            cpu3_param_c,
            rf'"{zh_name}"[\s\S]{{0,180}}{enum_name}[\s\S]{{0,220}}\(uint8_t\*\)"kg/m3",\s*2,\s*0',
            f"CPU3 parameter {zh_name} must display kg/m3 with point=2",
            errors,
        )
    require_re(
        cpu3_param_c,
        r'"磁通量D"[\s\S]{0,220}COM_NUM_DEVICEPARAM_DENSITYCORRECTION[\s\S]{0,220}-100000,\s*100000,\s*\(uint8_t\*\)"kg/m3",\s*2,\s*-100000',
        "CPU3 density correction metadata must use x100 base/range/offset",
        errors,
    )
    require_re(
        cpu3_param_c,
        r'"密度手输值"[\s\S]{0,220}COM_NUM_SCREEN_INPUT_D[\s\S]{0,220}0,\s*200000,\s*\(uint8_t\*\)"kg/m3",\s*2,\s*0',
        "CPU3 density manual input must allow 0.00..2000.00 kg/m3",
        errors,
    )

    compact_sync = normalize(sync_c)
    require("return(uint32_t)((int32_t)h->val-(int32_t)h->offset);" in compact_sync, "CPU3 MetaValueToRaw must subtract TYPE_INT offset", errors)
    require("return(int32_t)raw+(int32_t)h->offset;" in compact_sync, "CPU3 RawToMetaValue must add TYPE_INT offset", errors)
    require_nearby(
        display_c,
        "Display_AddValueStatusSlot(snapshot,\n                                   DISPLAY_STATUS_SLOT_DENSITY",
        ("2U,", '(uint8_t*)"kg/m3"'),
        "CPU3 status density display must use two decimal places",
        errors,
    )
    require_nearby(
        display_c,
        "Display_GetDensityValue(ctx, &display_value)",
        ("OledValueDisplay((int)display_value", "\n                             2,", '(u8*)"kg/m3"'),
        "CPU3 result density display must use two decimal places",
        errors,
    )


def check_external_protocol_contract(dsm_c: str, wartsila_c: str, si7000_c: str, errors: list[str]) -> None:
    require_re(dsm_c, r"static\s+uint32_t\s+DSM_DensityRawToExternalX10\s*\(", "DSM density x100 to external x10 helper must exist", errors)
    require_re(dsm_c, r"return\s+\(raw_density\s*\+\s*\(DENSITY_PARAM_MIGRATE_FACTOR\s*/\s*2U\)\)\s*/\s*DENSITY_PARAM_MIGRATE_FACTOR", "DSM density helper must round x100 to x10", errors)
    require_re(dsm_c, r"DSM_DensityCorrectionRawToExternalX10\s*\(g_deviceParams\.densityCorrection\)", "DSM holding output must convert density correction back to external x10", errors)
    require_re(dsm_c, r"DSM_DensityCorrectionExternalX10ToRaw\s*\(\(uint32_t\)\(temp\s*&\s*0xFFFF\)\)", "DSM external write must convert density correction to internal x100", errors)
    for field in ("density", "standard_density", "weight_density", "average_density", "average_standard_density", "average_weight_density"):
        require(f"DSM_DensityRawToExternalX10(g_measurement." in dsm_c and field in dsm_c, f"DSM must convert {field} density output through x10 helper", errors)

    require_re(wartsila_c, r"static\s+int16_t\s+Wartsila_DensityRawToX10\s*\(", "Wartsila density x100 to x10 helper must exist", errors)
    require_re(wartsila_c, r"raw_x10\s*=\s*\(raw_density\s*\+\s*\(DENSITY_PARAM_MIGRATE_FACTOR\s*/\s*2U\)\)\s*/\s*DENSITY_PARAM_MIGRATE_FACTOR", "Wartsila density helper must round x100 to x10", errors)
    require_re(wartsila_c, r"raw_x10\s*>\s*32767U", "Wartsila density helper must saturate signed 16-bit output", errors)
    require_re(wartsila_c, r"density_kgm3_x10\s*=\s*\(int32_t\)\s*Wartsila_DensityRawToX10", "Wartsila current density must use x10 helper", errors)
    require_re(wartsila_c, r"density_x10\s*=\s*Wartsila_DensityRawToX10", "Wartsila distribution point density must use x10 helper", errors)

    require_re(si7000_c, r"static\s+uint16_t\s+si7000_density_raw_to_si_u16\s*\(", "SI7000 density helper must exist", errors)
    require_re(si7000_c, r"return\s+si7000_clamp_u16\(raw_density\)", "SI7000 must keep x100 density as 0.01 unit with u16 clamp", errors)


def check_docs(protocol_doc: str, errors: list[str]) -> None:
    require_re(protocol_doc, r"\|\s*13\s*\|[\s\S]{0,240}内部密度 raw 从 `kg/m3 x10` 升级为 `kg/m3 x100`", "protocol table must document density protocol 13", errors)
    require_re(protocol_doc, r"###\s+协议版本\s+13[\s\S]{0,900}CPU3 本机 FRAM 参数版本升级到 `0x0005`", "protocol version 13 section must document CPU3 V5 migration", errors)
    require_re(protocol_doc, r"DSM 外部协议输出密度和密度修正时保持原 `x10` 口径", "protocol version 13 section must document DSM x10 compatibility", errors)
    require_re(protocol_doc, r"Wartsila 外部协议密度继续保持 `scale = 10` / `x10`", "protocol version 13 section must document Wartsila x10 compatibility", errors)


def main() -> int:
    errors: list[str] = []

    cpu2_h = read(CPU2_PARAM_H, "gbk")
    cpu2_c = read(CPU2_PARAM_C, "gbk")
    sensor_c = read(CPU2_SENSOR_C, "gbk")
    oil_level_c = read(CPU2_OIL_LEVEL_C, "gbk")
    cpu3_h = read(CPU3_PARAM_H)
    cpu3_param_c = read(CPU3_PARAM_C)
    local_param_c = read(CPU3_LOCAL_PARAM_C)
    sync_c = read(CPU3_DEVICE_SYNC_C)
    display_c = read(CPU3_DISPLAY_C)
    dsm_c = read(DSM_C)
    wartsila_c = read(WARTSILA_C)
    si7000_c = read(SI7000_C)
    protocol_doc = read(PROTOCOL_DOC)

    check_numeric_examples(errors)
    check_cpu2_contract(cpu2_h, cpu2_c, sensor_c, oil_level_c, errors)
    check_cpu3_contract(cpu2_h, cpu3_h, cpu3_param_c, local_param_c, sync_c, display_c, errors)
    check_external_protocol_contract(dsm_c, wartsila_c, si7000_c, errors)
    check_docs(protocol_doc, errors)

    if errors:
        print("density precision contract check failed:")
        for error in errors:
            print(f"- {error}")
        return 1

    print("density precision contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""检查 CPU3 ret_arr_word 参数的显示枚举契约。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPU3_META = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.c"
CPU3_MENU = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display_tankopera.c"


def parse_display_arrays(text: str) -> dict[str, int]:
    arrays: dict[str, int] = {}
    pattern = re.compile(
        r"static\s+uint8_t\s+\*([A-Za-z0-9_]+)\s*\[\]\[2\]\s*=\s*\{(.*?)\};",
        re.S,
    )
    for match in pattern.finditer(text):
        name = match.group(1)
        body = match.group(2)
        entries = re.findall(
            r"\{\s*\(uint8_t\*\)\"([^\"]*)\"\s*,\s*\(uint8_t\*\)\"([^\"]*)\"\s*\}",
            body,
        )
        valid_count = 0
        for chinese, english in entries:
            if (chinese == "非法配置") or (english == "Illegal CFG"):
                break
            valid_count += 1
        arrays[name] = valid_count
    return arrays


def parse_meta_rows(text: str) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for line_no, line in enumerate(text.splitlines(), 1):
        if ("ret_arr_word" not in line) or (not line.lstrip().startswith("{")):
            continue
        parts = [part.strip() for part in line.strip().lstrip("{").split(",")]
        if len(parts) < 16:
            raise AssertionError(f"{CPU3_META}:{line_no}: cannot parse param_meta row")
        name_match = re.search(r"\"([^\"]+)\"", parts[0])
        if name_match is None:
            raise AssertionError(f"{CPU3_META}:{line_no}: missing Chinese name")
        rows.append(
            {
                "line": line_no,
                "name": name_match.group(1),
                "operanum": parts[2],
                "check": parts[5] == "true",
                "min": int(parts[6]),
                "max": int(parts[7]),
            }
        )
    return rows


def extract_dtm_disarr(text: str) -> str:
    start = text.find("uint8_t *(*dtm_disarr")
    end = text.find("/* 显示要选择的信息 */", start)
    if (start < 0) or (end < 0):
        raise AssertionError("dtm_disarr function not found")
    return text[start:end]


def explicit_case_set(dtm_text: str) -> set[str]:
    return set(re.findall(r"case\s+(COM_[A-Z0-9_]+)\s*:", dtm_text))


def mapping_for(operanum: str) -> tuple[str | None, str]:
    direct_mapping = {
        "COM_NUM_DEVICEPARAM_SPREADMEASUREMENTORDER": "arr_densitydir",
        "COM_NUM_SCREEN_SOURCE_OIL": "arr_source",
        "COM_NUM_SCREEN_SOURCE_WATER": "arr_source",
        "COM_NUM_SCREEN_SOURCE_D": "arr_source",
        "COM_NUM_SCREEN_SOURCE_T": "arr_source",
        "COM_NUM_DEVICEPARAM_REQUIREBOTTOMMEASUREMENT": "arr_IF",
        "COM_NUM_DEVICEPARAM_REQUIREWATERMEASUREMENT": "arr_IF",
        "COM_NUM_DEVICEPARAM_REQUIRESINGLEPOINTDENSITY": "arr_IF",
        "COM_NUM_DEVICEPARAM_ERROR_AUTO_BACK_ZERO": "arr_IF",
        "COM_NUM_DEVICEPARAM_ERROR_STOP_MEASUREMENT": "arr_IF",
        "COM_NUM_DEVICEPARAM_REFRESH_TANKHEIGHT_FLAG": "arr_IF",
        "COM_NUM_DEVICEPARAM_BOTTOM_ENCODER_CORRECTION_ENABLE": "arr_IF",
        "COM_NUM_SCREEN_OFF": "arr_IF",
        "COM_NUM_DEVICEPARAM_POSITION_SOURCE_AUTO_SWITCH": "arr_position_source_auto_switch",
        "COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE": "arr_ao_output_enable",
        "COM_NUM_DEVICEPARAM_POSITION_COUNT_MODE": "arr_position_count_mode",
        "COM_NUM_DEVICEPARAM_SPREADMEASUREMENTMODE": "arr_densitymode",
        "COM_NUM_SCREEN_INPUT_D_SWITCH": "arr_Switch",
        "COM_NUM_PARA_LANG": "arr_language",
        "COM_NUM_SCREEN_BRIGHTNESS": "arr_oled_brightness",
        "COM_NUM_CPU3_COM1_BAUDRATE": "arr_baudrate",
        "COM_NUM_CPU3_COM2_BAUDRATE": "arr_baudrate",
        "COM_NUM_CPU3_COM3_BAUDRATE": "arr_baudrate",
        "COM_NUM_CPU3_COM1_DATABITS": "arr_databits",
        "COM_NUM_CPU3_COM2_DATABITS": "arr_databits",
        "COM_NUM_CPU3_COM3_DATABITS": "arr_databits",
        "COM_NUM_CPU3_COM1_PARITY": "arr_parity",
        "COM_NUM_CPU3_COM2_PARITY": "arr_parity",
        "COM_NUM_CPU3_COM3_PARITY": "arr_parity",
        "COM_NUM_CPU3_COM1_STOPBITS": "arr_stopbits",
        "COM_NUM_CPU3_COM2_STOPBITS": "arr_stopbits",
        "COM_NUM_CPU3_COM3_STOPBITS": "arr_stopbits",
        "COM_NUM_DEVICEPARAM_BOTTOM_DETECT_MODE": "arr_bottom",
        "COM_NUM_DEVICEPARAM_POWER_ON_DEFAULT_COMMAND": "default_cmmand",
        "COM_NUM_DEVICEPARAM_LIQUIDLEVELMEASUREMENTMETHOD": "level_mode",
    }
    mapped_mapping = {
        "COM_NUM_CPU3_COM1_PROTOCOL": "arr_protocol",
        "COM_NUM_CPU3_COM2_PROTOCOL": "arr_protocol",
        "COM_NUM_CPU3_COM3_PROTOCOL": "arr_protocol",
    }
    normalized_mapping = {
        "COM_NUM_DEVICEPARAM_WATER_LEVEL_MODE": "water_level_mode",
    }
    relay_mapping = {
        "OPERATING_MODE": "arr_relay_operating",
        "DIGITAL_SOURCE": "arr_relay_digital",
        "CONTACT_TYPE": "arr_relay_contact",
        "ALARM_MODE": "arr_relay_alarm_mode",
        "ERROR_VALUE": "arr_relay_error",
        "ALARM_SOURCE": "arr_relay_source",
        "CLEAR_ALARM": "arr_IF",
    }

    if operanum in direct_mapping:
        return direct_mapping[operanum], "direct"
    if operanum in mapped_mapping:
        return mapped_mapping[operanum], "mapped"
    if operanum in normalized_mapping:
        return normalized_mapping[operanum], "normalized"
    for suffix, array_name in relay_mapping.items():
        if operanum.startswith("COM_NUM_DEVICEPARAM_RELAY") and operanum.endswith(suffix):
            return array_name, "direct"
    return None, "missing"


def main() -> int:
    meta_text = CPU3_META.read_text(encoding="utf-8")
    menu_text = CPU3_MENU.read_text(encoding="utf-8")
    rows = parse_meta_rows(meta_text)
    arrays = parse_display_arrays(menu_text)
    cases = explicit_case_set(extract_dtm_disarr(menu_text))

    failed: list[str] = []

    for row in rows:
        operanum = str(row["operanum"])
        array_name, mode = mapping_for(operanum)
        line = int(row["line"])
        name = str(row["name"])

        if (operanum not in cases) and (not operanum.startswith("COM_NUM_DEVICEPARAM_RELAY")):
            failed.append(f"{CPU3_MENU}: dtm_disarr missing case for {operanum} ({name})")
            continue

        if array_name is None:
            failed.append(f"{CPU3_META}:{line}: missing audit mapping for {operanum} ({name})")
            continue

        if array_name not in arrays:
            failed.append(f"{CPU3_MENU}: display array {array_name} not found for {operanum} ({name})")
            continue

        if mode != "direct":
            continue

        value_min = int(row["min"])
        value_max = int(row["max"])
        valid_count = arrays[array_name]
        if bool(row["check"]) and ((value_min < 0) or (value_max >= valid_count)):
            failed.append(
                f"{CPU3_META}:{line}: {name} range {value_min}..{value_max} exceeds "
                f"{array_name} valid 0..{valid_count - 1}"
            )

    if failed:
        print("CPU3 ret_arr_word contract check failed:")
        for item in failed:
            print(f"- {item}")
        return 1

    print("CPU3 ret_arr_word contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

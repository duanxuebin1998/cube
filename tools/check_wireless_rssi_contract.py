#!/usr/bin/env python3
"""检查读取部件参数蓝牙 RSSI 的 CPU2/CPU3 共享协议契约。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPU2_PARAM = ROOT / "LTD_MAIN_CPU2" / "Services" / "ParamStorage" / "system_parameter.h"
CPU3_PARAM = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.h"
CPU2_STATE = ROOT / "LTD_MAIN_CPU2" / "Services" / "Modbus" / "stateformodbus.h"
CPU3_STATE = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "stateformodbus.h"
CPU2_MODBUS = ROOT / "LTD_MAIN_CPU2" / "Services" / "Modbus" / "dataanalysis_modbus.c"
CPU3_MODBUS = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "internal" / "main_board_modbus" / "dataanalysis_modbus.c"
CPU2_SENSOR = ROOT / "LTD_MAIN_CPU2" / "Services" / "Sensor" / "sensor.c"
CPU2_WIRELESS = ROOT / "LTD_MAIN_CPU2" / "Services" / "Sensor" / "wireless_pairing.c"
CPU2_CH9141_AT = ROOT / "LTD_MAIN_CPU2" / "Services" / "Sensor" / "ch9141_at.c"
CPU3_DISPLAY = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display.c"
PROTOCOL_DOC = ROOT / "docs" / "01_协议与寄存器" / "CPU2_CPU3协议变更记录.md"


RSSI_FIELDS = [
    "connection_valid",
    "rssi_valid",
    "rssi",
    "connection_error_code",
    "rssi_update_counter",
]

RSSI_REGS = [
    "REG_WIRELESS_PAIRING_CONNECTION_VALID",
    "REG_WIRELESS_PAIRING_RSSI_VALID",
    "REG_WIRELESS_PAIRING_RSSI",
    "REG_WIRELESS_PAIRING_CONNECTION_ERROR_CODE",
    "REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER",
]


def read_text(path: Path, encoding: str) -> str:
    return path.read_text(encoding=encoding)


def compact(text: str) -> str:
    return re.sub(r"\s+", "", text)


def parse_protocol_version(text: str, path: Path) -> int:
    match = re.search(r"#define\s+DEVICE_PROTOCOL_VERSION\s+(\d+)u?\b", text)
    if match is None:
        raise AssertionError(f"{path}: missing DEVICE_PROTOCOL_VERSION")
    return int(match.group(1))


def extract_struct_body(text: str, name: str, path: Path) -> str:
    match = re.search(r"typedef\s+struct\s*\{(?P<body>.*?)\}\s*" + re.escape(name) + r"\s*;", text, re.DOTALL)
    if match is None:
        raise AssertionError(f"{path}: missing {name}")
    return match.group("body")


def field_order(body: str) -> list[str]:
    fields: list[str] = []
    for match in re.finditer(r"\b(?:uint32_t|int32_t)\s+([A-Za-z_][A-Za-z0-9_]*)\s*;", body):
        fields.append(match.group(1))
    return fields


def require(condition: bool, message: str, failed: list[str]) -> None:
    if not condition:
        failed.append(message)


def main() -> int:
    cpu2_param = read_text(CPU2_PARAM, "gbk")
    cpu3_param = read_text(CPU3_PARAM, "utf-8")
    cpu2_state = read_text(CPU2_STATE, "gbk")
    cpu3_state = read_text(CPU3_STATE, "utf-8")
    cpu2_modbus = read_text(CPU2_MODBUS, "gbk")
    cpu3_modbus = read_text(CPU3_MODBUS, "utf-8")
    cpu2_sensor = read_text(CPU2_SENSOR, "gbk")
    cpu2_wireless = read_text(CPU2_WIRELESS, "gbk")
    cpu2_ch9141_at = read_text(CPU2_CH9141_AT, "gbk")
    cpu3_display = read_text(CPU3_DISPLAY, "utf-8")
    protocol_doc = read_text(PROTOCOL_DOC, "utf-8")

    failed: list[str] = []

    cpu2_version = parse_protocol_version(cpu2_param, CPU2_PARAM)
    cpu3_version = parse_protocol_version(cpu3_param, CPU3_PARAM)
    require(cpu2_version == 9, "CPU2 DEVICE_PROTOCOL_VERSION must be 9", failed)
    require(cpu3_version == 9, "CPU3 DEVICE_PROTOCOL_VERSION must be 9", failed)

    cpu2_fields = field_order(extract_struct_body(cpu2_param, "WirelessPairingStatus", CPU2_PARAM))
    cpu3_fields = field_order(extract_struct_body(cpu3_param, "WirelessPairingStatus", CPU3_PARAM))
    require(cpu2_fields == cpu3_fields, "CPU2/CPU3 WirelessPairingStatus field order must match", failed)
    for field in RSSI_FIELDS:
        require(field in cpu2_fields, f"missing WirelessPairingStatus.{field}", failed)

    for reg in RSSI_REGS:
        require(reg in cpu2_state, f"CPU2 missing {reg}", failed)
        require(reg in cpu3_state, f"CPU3 missing {reg}", failed)
        require(reg in cpu2_modbus, f"CPU2 Modbus mapping missing {reg}", failed)
        require(reg in cpu3_modbus, f"CPU3 Modbus mapping missing {reg}", failed)

    cpu2_state_compact = compact(cpu2_state)
    cpu3_state_compact = compact(cpu3_state)
    expected_tail = "REG_ENG(REG_WIRELESS_PAIRING_RSSI_UPDATE_COUNTER+REG_SIZE_U32)"
    require(expected_tail in cpu2_state_compact, "CPU2 REG_ENG must end after RSSI update counter", failed)
    require(expected_tail in cpu3_state_compact, "CPU3 REG_ENG must end after RSSI update counter", failed)

    sensor_compact = compact(cpu2_sensor)
    require("READ_PART_PARAMS_RSSI_REFRESH_INTERVAL_MS5000U" in sensor_compact, "read-part-params RSSI refresh interval must be 5000 ms", failed)
    require("WirelessPairing_UpdateConnectionStatusSnapshot()" in sensor_compact, "read-part-params must update wireless RSSI snapshot", failed)

    wireless_compact = compact(cpu2_wireless)
    require("WirelessPairing_ReadConnectionStatus(WirelessConnectionStatus*status)" in wireless_compact, "missing wireless connection status reader", failed)
    require("AT+RSSI=ON,%lu" in cpu2_wireless, "wireless reader must enable RSSI reporting", failed)
    require("AT+RSSI=OFF" in cpu2_wireless, "wireless reader must disable RSSI reporting", failed)

    ch9141_compact = compact(cpu2_ch9141_at)
    require("CH9141_AT_ResponseHasRssiText" in cpu2_ch9141_at, "CH9141 AT layer must detect RSSI async text", failed)
    require("*raw_end=='\\0'" in ch9141_compact, "RSSI async wait must not finish before line ending", failed)
    require("has_rssi_token" in cpu2_ch9141_at, "RSSI async wait must identify RSSI text lines", failed)

    require("Para_wireless_rssi" in cpu3_display, "CPU3 display must include RSSI parameter slot", failed)
    require("DISPLAY_STATUS_SLOT_WIRELESS_RSSI" in cpu3_display, "CPU3 display must include RSSI highlight slot", failed)
    require("RSSI:N/A" in cpu3_display, "CPU3 display must show RSSI invalid state", failed)

    require("### 协议版本 9" in protocol_doc, "protocol change record must document protocol version 9", failed)
    require("REG_WIRELESS_PAIRING_RSSI" in protocol_doc, "protocol change record must document RSSI register", failed)

    if failed:
        print("Wireless RSSI contract check failed:")
        for item in failed:
            print(f"- {item}")
        return 1

    print("Wireless RSSI contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

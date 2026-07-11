#!/usr/bin/env python3
"""检查 LTD 共享 Modbus 分发、读写闭环和参考帧。"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
APP_MAIN = ROOT / "LTD_DISPLAY_CPU3/Application/app_main.c"
LTD_HEADER = ROOT / "LTD_DISPLAY_CPU3/Communication/external/ltd_modbus/ltd_modbus_slave.h"
LTD_SOURCE = ROOT / "LTD_DISPLAY_CPU3/Communication/external/ltd_modbus/ltd_modbus_slave.c"
CPU2_COMM = ROOT / "LTD_DISPLAY_CPU3/Communication/internal/main_board_modbus/cpu2_communicate.c"
CPU2_PROTOCOL = ROOT / "LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h"
CPU3_PROTOCOL = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h"


@dataclass(frozen=True)
class FrameCase:
    name: str
    payload: tuple[int, ...]
    expected: str


def read(path: Path) -> str:
    data = path.read_bytes()
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError:
        return data.decode("gb18030")


def crc16_modbus(data: tuple[int, ...]) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
            crc &= 0xFFFF
    return crc


def frame_hex(payload: tuple[int, ...]) -> str:
    crc = crc16_modbus(payload)
    frame = (*payload, crc & 0xFF, (crc >> 8) & 0xFF)
    return " ".join(f"{byte:02X}" for byte in frame)


FRAME_CASES = (
    FrameCase("FC03 读命令请求", (0x01, 0x03, 0, 0, 0, 2), "01 03 00 00 00 02 C4 0B"),
    FrameCase("FC03 零命令响应", (0x01, 0x03, 4, 0, 0, 0, 0), "01 03 04 00 00 00 00 FA 33"),
    FrameCase("FC04 读状态请求", (0x01, 0x04, 0, 0, 0, 2), "01 04 00 00 00 02 71 CB"),
    FrameCase("FC04 零状态响应", (0x01, 0x04, 4, 0, 0, 0, 0), "01 04 04 00 00 00 00 FB 84"),
    FrameCase("FC10 写命令 6 请求", (0x01, 0x10, 0, 0, 0, 2, 4, 0, 0, 0, 6), "01 10 00 00 00 02 04 00 00 00 06 73 AD"),
    FrameCase("FC10 写成功响应", (0x01, 0x10, 0, 0, 0, 2), "01 10 00 00 00 02 41 C8"),
    FrameCase("FC03 设备忙", (0x01, 0x83, 0x06), "01 83 06 C1 32"),
    FrameCase("FC04 非法地址", (0x01, 0x84, 0x02), "01 84 02 C2 C1"),
    FrameCase("FC10 非法值", (0x01, 0x90, 0x03), "01 90 03 0C 01"),
    FrameCase("FC10 设备忙", (0x01, 0x90, 0x06), "01 90 06 CC 02"),
    FrameCase("FC06 非法功能", (0x01, 0x86, 0x01), "01 86 01 83 A0"),
)


def require(text: str, pattern: str, message: str) -> None:
    if re.search(pattern, text, re.S) is None:
        raise AssertionError(message)


def protocol_version(text: str) -> int:
    match = re.search(r"#define\s+DEVICE_PROTOCOL_VERSION\s+(\d+)[uU]?", text)
    if match is None:
        raise AssertionError("DEVICE_PROTOCOL_VERSION not found")
    return int(match.group(1))


def check_source_contract() -> None:
    app = read(APP_MAIN)
    header = read(LTD_HEADER)
    source = read(LTD_SOURCE)
    cpu2_comm = read(CPU2_COMM)

    require(app, r"\[COM_PROTO_LTD\]\s*=\s*\{\s*proto_ltd_process", "LTD dispatcher is not connected")
    require(app, r"return\s+ltd_modbus_process_for_dispatch\s*\(", "LTD dispatcher wrapper is missing")

    expected_defines = {
        "LTD_MODBUS_FUNC_READ_HOLDING_REGS": "0x03U",
        "LTD_MODBUS_FUNC_READ_INPUT_REGS": "0x04U",
        "LTD_MODBUS_FUNC_WRITE_MULTI_REGS": "0x10U",
        "LTD_MODBUS_EX_ILLEGAL_FUNCTION": "0x01U",
        "LTD_MODBUS_EX_ILLEGAL_ADDRESS": "0x02U",
        "LTD_MODBUS_EX_ILLEGAL_VALUE": "0x03U",
        "LTD_MODBUS_EX_SLAVE_DEVICE_BUSY": "0x06U",
    }
    for name, value in expected_defines.items():
        require(header, rf"#define\s+{name}\s+{value}\b", f"{name} changed")

    require(source, r"CPU2_CommReadHoldingSnapshot\s*\(", "FC03 does not use the CPU2 snapshot")
    require(source, r"CPU2_CommReadInputSnapshot\s*\(", "FC04 does not use the CPU2 snapshot")
    require(source, r"CPU2_CommWriteHoldingRegisters\s*\(", "FC10 does not write through to CPU2")
    require(source, r"\(start\s*&\s*1U\).*?\(count\s*&\s*1U\)", "FC10 32-bit alignment guard is missing")
    require(cpu2_comm, r"CPU2_CommWriteHoldingRegisters[\s\S]*?CPU2_CombinatePackage_Send", "CPU2 write gateway is missing")
    require(cpu2_comm, r"if\s*\(\s*!ret\s*\)\s*\{\s*return\s+false", "CPU2 ACK failure is not propagated")
    require(cpu2_comm, r"CPU2_CommRequestParameterRefresh\s*\(\s*\)", "parameter snapshot refresh is missing")

    cpu2_version = protocol_version(read(CPU2_PROTOCOL))
    cpu3_version = protocol_version(read(CPU3_PROTOCOL))
    if (cpu2_version, cpu3_version) != (14, 14):
        raise AssertionError(f"protocol version mismatch: CPU2={cpu2_version}, CPU3={cpu3_version}")


def check_frames() -> None:
    for case in FRAME_CASES:
        actual = frame_hex(case.payload)
        if actual != case.expected:
            raise AssertionError(f"{case.name}: expected {case.expected}, got {actual}")


def main() -> int:
    try:
        check_source_contract()
        check_frames()
    except (AssertionError, UnicodeDecodeError) as exc:
        print(f"[FAIL] LTD Modbus contract: {exc}", file=sys.stderr)
        return 1
    print("[OK] LTD Modbus dispatcher, CPU2 ACK path, protocol version and golden frames are consistent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

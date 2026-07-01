#!/usr/bin/env python3
"""检查 SI Modbus 地址表常量和主机侧参考帧。"""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.h"
SOURCE = ROOT / "LTD_DISPLAY_CPU3/Communication/external/si_modbus/si_modbus_slave.c"

# 这里固定的是协议地址窗口和功能码，不依赖运行时状态；一旦手册映射变化，应先改文档再改本表。
EXPECTED_DEFINES = {
    "SI_COIL_COUNT": 16,
    "SI_DISCRETE_INPUT_COUNT": 32,
    "SI_HOLDING_REG_COUNT": 23,
    "SI_INPUT_REG_COUNT": 620,
    "SI_FUNC_READ_COILS": 0x01,
    "SI_FUNC_READ_DISCRETE_INPUTS": 0x02,
    "SI_FUNC_READ_HOLDING_REGS": 0x03,
    "SI_FUNC_READ_INPUT_REGS": 0x04,
    "SI_FUNC_WRITE_SINGLE_COIL": 0x05,
    "SI_FUNC_WRITE_SINGLE_REG": 0x06,
}

# 枚举值必须等于 SI Modbus offset，不能因 C 代码整理而自动重排。
EXPECTED_ENUMS = {
    "SI_COIL_MANUAL": 0,
    "SI_COIL_CALIBRATE": 1,
    "SI_COIL_AUTO": 2,
    "SI_COIL_PROFILE": 3,
    "SI_COIL_STOP": 8,
    "SI_COIL_UP_SLOW": 9,
    "SI_COIL_UP_MEDIUM": 10,
    "SI_COIL_UP_FAST": 11,
    "SI_COIL_DOWN_SLOW": 12,
    "SI_COIL_DOWN_MEDIUM": 13,
    "SI_COIL_DOWN_FAST": 14,
    "SI_DI_BOTTOM_REFERENCE": 0,
    "SI_DI_LOWER_LEVEL_SENSOR": 1,
    "SI_DI_UPPER_LEVEL_SENSOR": 2,
    "SI_DI_INTERLOCK": 3,
    "SI_DI_PROFILE_COMPLETE": 4,
    "SI_DI_UNIT_IS_METRIC": 5,
    "SI_DI_REEL_ALARM": 8,
    "SI_DI_PROBE_UNCALIBRATED": 9,
    "SI_DI_INTERVAL_TIMER": 11,
    "SI_DI_PROBE_AT_LIQUID_LEVEL": 12,
    "SI_DI_LOW_DENSITY_ALARM": 16,
    "SI_DI_HIGH_DENSITY_ALARM": 17,
    "SI_DI_LOW_TEMP_ALARM": 18,
    "SI_DI_HIGH_TEMP_ALARM": 19,
    "SI_DI_LL_LEVEL_ALARM": 20,
    "SI_DI_HH_LEVEL_ALARM": 21,
    "SI_DI_LOW_LEVEL_ALARM": 22,
    "SI_DI_HIGH_LEVEL_ALARM": 23,
    "SI_DI_PROFILE_TEMP_DEVIATION_ALARM": 24,
    "SI_DI_PROFILE_DENSITY_DEVIATION_ALARM": 25,
    "SI_DI_PROFILE_LOW_TEMP_ALARM": 28,
    "SI_DI_PROFILE_HIGH_TEMP_ALARM": 29,
    "SI_DI_PROFILE_LOW_DENSITY_ALARM": 30,
    "SI_DI_PROFILE_HIGH_DENSITY_ALARM": 31,
    "SI_IR_CURRENT_PROBE_POSITION": 0,
    "SI_IR_CURRENT_TEMPERATURE": 1,
    "SI_IR_CURRENT_DENSITY": 2,
    "SI_IR_LIQUID_LEVEL": 3,
    "SI_IR_NUMBER_OF_POINTS": 5,
    "SI_IR_PROFILE_TIMESTAMP_MONTH": 6,
    "SI_IR_PROFILE_TIMESTAMP_DAY": 7,
    "SI_IR_PROFILE_TIMESTAMP_HOUR": 8,
    "SI_IR_PROFILE_TIMESTAMP_MINUTE": 9,
    "SI_IR_CURRENT_TIME_HOUR": 10,
    "SI_IR_CURRENT_TIME_MINUTE": 11,
    "SI_IR_CURRENT_TIME_SECOND": 12,
    "SI_IR_PROFILE_POINT0_POSITION": 20,
    "SI_HR_PROFILE_FIRST_POINT": 0,
    "SI_HR_PROFILE_INCREMENT": 1,
    "SI_HR_PROFILE_DWELL_TIME": 2,
    "SI_HR_AUTO_PROFILE_INTERVAL": 9,
    "SI_HR_AUTO_PROFILE_ENABLE": 10,
    "SI_HR_AUTO_PROFILE_HOUR": 11,
    "SI_HR_AUTO_PROFILE_MINUTE": 12,
    "SI_HR_LOW_DENSITY_SETPOINT": 13,
    "SI_HR_HIGH_DENSITY_SETPOINT": 14,
    "SI_HR_LOW_TEMPERATURE_SETPOINT": 15,
    "SI_HR_HIGH_TEMPERATURE_SETPOINT": 16,
    "SI_HR_LL_LEVEL_SETPOINT": 17,
    "SI_HR_HH_LEVEL_SETPOINT": 18,
    "SI_HR_LOW_LEVEL_SETPOINT": 19,
    "SI_HR_HIGH_LEVEL_SETPOINT": 20,
    "SI_HR_TEMP_DEVIATION_SETPOINT": 21,
    "SI_HR_DENSITY_DEVIATION_SETPOINT": 22,
}


@dataclass(frozen=True)
class FrameCase:
    """一组 SI 参考帧，包含帧字节、期望十六进制和联调用途说明。"""

    name: str
    frame: list[int]
    expected_hex: str
    note: str


def read_text(path: Path) -> str:
    """按 UTF-8 读取新增 SI 源码，借此尽早发现编码异常。"""

    # 新增 SI 模块统一按 UTF-8 维护，脚本直接用 UTF-8 读取，便于发现编码异常。
    return path.read_text(encoding="utf-8")


def parse_c_int(value: str) -> int:
    """解析 C 常量文本，支持十进制、十六进制和 u/l 后缀。"""

    value = value.strip().rstrip(",")
    value = re.sub(r"[uUlL]+$", "", value)
    return int(value, 0)


def parse_defines(text: str) -> dict[str, int]:
    """提取 SI_* 宏常量，避免无关宏影响协议契约检查。"""

    values: dict[str, int] = {}
    # 只解析 SI_* 常量，避免把无关宏纳入契约检查导致误报。
    for match in re.finditer(r"#define\s+(SI_[A-Z0-9_]+)\s+([0-9A-Fa-fxXuUlL]+)\b", text):
        values[match.group(1)] = parse_c_int(match.group(2))
    return values


def parse_enums(text: str) -> dict[str, int]:
    """提取 SI 枚举值，同时支持显式赋值和 C 语言隐式自增。"""

    values: dict[str, int] = {}
    for enum_body in re.findall(r"enum\s*\{(.*?)\};", text, re.S):
        current = -1
        for raw_line in enum_body.splitlines():
            # 支持显式赋值和隐式自增两种 C 枚举写法，保持脚本与源码写法解耦。
            line = raw_line.split("//", 1)[0].strip().rstrip(",")
            if not line or not line.startswith("SI_"):
                continue
            if "=" in line:
                name, raw_value = [part.strip() for part in line.split("=", 1)]
                current = parse_c_int(raw_value)
            else:
                name = line.strip()
                current += 1
            values[name] = current
    return values


def crc16_modbus(data: list[int]) -> int:
    """按 Modbus RTU 多项式复算 CRC，防止 golden frame 尾字节抄错。"""

    # Python 侧复算 Modbus CRC，避免 golden frame 只比较人工抄写的尾两个字节。
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def with_crc(payload: list[int]) -> list[int]:
    """给一段不含 CRC 的 RTU 载荷追加低字节在前的 Modbus CRC。"""

    crc = crc16_modbus(payload)
    return payload + [crc & 0xFF, (crc >> 8) & 0xFF]


def hex_bytes(data: list[int]) -> str:
    """将帧字节格式化为 PLC 联调可直接抄用的十六进制字符串。"""

    return " ".join(f"{byte:02X}" for byte in data)


FRAME_CASES = [
    # 请求帧和参考响应帧都放在这里，便于 PLC 联调时用 --dump 直接导出。
    FrameCase("FC01 读 00001~00016 请求", with_crc([0x01, 0x01, 0x00, 0x00, 0x00, 0x10]), "01 01 00 00 00 10 3D C6", "读取全部线圈"),
    FrameCase("FC01 默认安全态响应", with_crc([0x01, 0x01, 0x02, 0x01, 0x01]), "01 01 02 01 01 79 AC", "Manual+Stop 位按 Modbus bit 顺序打包"),
    FrameCase("FC02 读 10001~10032 请求", with_crc([0x01, 0x02, 0x00, 0x00, 0x00, 0x20]), "01 02 00 00 00 20 79 D2", "读取全部离散输入"),
    FrameCase("FC02 默认公制响应", with_crc([0x01, 0x02, 0x04, 0x20, 0x00, 0x00, 0x00]), "01 02 04 20 00 00 00 F0 22", "Unit Is Metric 位为 1"),
    FrameCase("FC03 读 40010~40013 请求", with_crc([0x01, 0x03, 0x00, 0x09, 0x00, 0x04]), "01 03 00 09 00 04 94 0B", "读取 CPU3 自动 profile 参数"),
    FrameCase("FC03 自动 profile 默认响应", with_crc([0x01, 0x03, 0x08, 0x00, 0x3C, 0, 0, 0, 0, 0, 0]), "01 03 08 00 3C 00 00 00 00 00 00 69 D4", "默认周期 60min，默认关闭"),
    FrameCase("FC04 读 30001~30013 请求", with_crc([0x01, 0x04, 0x00, 0x00, 0x00, 0x0D]), "01 04 00 00 00 0D 31 CF", "读取实时值和时间寄存器"),
    FrameCase("FC04 零快照响应", with_crc([0x01, 0x04, 0x1A, 0x00, 0x00, 0xB1, 0xE0] + [0] * 22), "01 04 1A 00 00 B1 E0 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 77 4B", "参考零快照，温度无效值为 -200.00C，真实时间可非零"),
    FrameCase("FC05 写 Profile ON 请求", with_crc([0x01, 0x05, 0x00, 0x03, 0xFF, 0x00]), "01 05 00 03 FF 00 7C 3A", "写单线圈标准格式"),
    FrameCase("FC05 写 Profile ON 回显", with_crc([0x01, 0x05, 0x00, 0x03, 0xFF, 0x00]), "01 05 00 03 FF 00 7C 3A", "回包显式重建功能码、地址和值"),
    FrameCase("FC06 写 40012=23 请求", with_crc([0x01, 0x06, 0x00, 0x0B, 0x00, 0x17]), "01 06 00 0B 00 17 B8 06", "Automatic Profile Hour 最大合法值"),
    FrameCase("FC06 写 40012=23 回显", with_crc([0x01, 0x06, 0x00, 0x0B, 0x00, 0x17]), "01 06 00 0B 00 17 B8 06", "合法值回显"),
    FrameCase("FC06 写 40016=-20.00C 请求", with_crc([0x01, 0x06, 0x00, 0x0F, 0xF8, 0x30]), "01 06 00 0F F8 30 FA 1D", "低温限值按 int16 补码写入"),
    FrameCase("FC06 写 40016=-20.00C 回显", with_crc([0x01, 0x06, 0x00, 0x0F, 0xF8, 0x30]), "01 06 00 0F F8 30 FA 1D", "合法负温度限值回显"),
    FrameCase("FC03 非法地址请求", with_crc([0x01, 0x03, 0x00, 0x17, 0x00, 0x01]), "01 03 00 17 00 01 34 0E", "40024 超出保持寄存器表"),
    FrameCase("FC03 非法地址响应", with_crc([0x01, 0x83, 0x02]), "01 83 02 C0 F1", "Illegal Data Address"),
    FrameCase("FC06 非法 40011=2 请求", with_crc([0x01, 0x06, 0x00, 0x0A, 0x00, 0x02]), "01 06 00 0A 00 02 28 09", "Automatic Profile Enable 只允许 0/1"),
    FrameCase("FC06 非法 40011=2 响应", with_crc([0x01, 0x86, 0x03]), "01 86 03 02 61", "Illegal Data Value"),
    FrameCase("FC16 非法功能请求", with_crc([0x01, 0x10, 0, 0, 0, 1, 2, 0, 0]), "01 10 00 00 00 01 02 00 00 A6 50", "未支持写多寄存器"),
    FrameCase("FC16 非法功能响应", with_crc([0x01, 0x90, 0x01]), "01 90 01 8D C0", "Illegal Function"),
]


def check_source_constants() -> None:
    """检查源码中的地址窗口、功能码和枚举 offset 是否仍匹配手册映射。"""

    defines = parse_defines(read_text(HEADER) + "\n" + read_text(SOURCE))
    enums = parse_enums(read_text(SOURCE))
    values = {**defines, **enums}
    for name, expected in {**EXPECTED_DEFINES, **EXPECTED_ENUMS}.items():
        # 任何地址偏移变化都需要同步更新映射文档和 golden frame，因此这里直接失败。
        actual = values.get(name)
        if actual != expected:
            raise AssertionError(f"{name}: expected {expected}, got {actual}")


def check_frames() -> None:
    """检查所有参考帧的功能码、地址、长度、数据和 CRC 是否保持一致。"""

    for case in FRAME_CASES:
        # 帧字节全量比较，能同时覆盖功能码、地址、数据长度和 CRC。
        actual = hex_bytes(case.frame)
        if actual != case.expected_hex:
            raise AssertionError(f"{case.name}: expected {case.expected_hex}, got {actual}")


def dump_frames() -> None:
    """按联调清单格式输出全部参考帧，便于现场复制到主机工具。"""

    for case in FRAME_CASES:
        print(f"{case.name}: {hex_bytes(case.frame)}  # {case.note}")


def main() -> int:
    """命令行入口，默认执行一致性检查，--dump 时额外输出参考帧。"""

    parser = argparse.ArgumentParser(description="Check SI Modbus constants and golden frames.")
    parser.add_argument("--dump", action="store_true", help="print golden frames")
    args = parser.parse_args()

    try:
        check_source_constants()
        check_frames()
    except AssertionError as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1

    if args.dump:
        dump_frames()
    print("[OK] SI Modbus constants and golden frames are consistent.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""轮询 CPU3 标准串口参数并发送统一外部协议切换帧。"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass


if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")


FUNCTION_CODE = 0x46
MAGIC = b"LTD"
ACK_STATUS_OK = 0x00


@dataclass(frozen=True)
class ProtocolProfile:
    value: int
    baudrate: int
    parity: str
    label: str


PROFILES = {
    "dsm": ProtocolProfile(0, 4800, "N", "DSM 4800 8N1"),
    "wartsila": ProtocolProfile(1, 2400, "E", "Wartsila 2400 8E1"),
    "ltd": ProtocolProfile(2, 115200, "N", "LTD 115200 8N1"),
    "si": ProtocolProfile(5, 9600, "O", "SI 9600 8O1"),
}

SCAN_ORDER = ("dsm", "wartsila", "ltd", "si")


def modbus_crc16(data: bytes) -> int:
    """计算 Modbus RTU CRC16。"""

    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def build_switch_frame(address: int, target: int) -> bytes:
    """生成固定 8 字节协议切换请求。"""

    payload = bytes((address, FUNCTION_CODE)) + MAGIC + bytes((target,))
    crc = modbus_crc16(payload)
    return payload + bytes((crc & 0xFF, crc >> 8))


def build_ack_frame(address: int, target: int) -> bytes:
    """生成设备接受切换请求后的 6 字节独立 ACK。"""

    payload = bytes((address, FUNCTION_CODE, ACK_STATUS_OK, target))
    crc = modbus_crc16(payload)
    return payload + bytes((crc & 0xFF, crc >> 8))


def exchange(serial_module, port: str, profile: ProtocolProfile,
             frame: bytes, expected_response: bytes, timeout: float) -> bool:
    """使用一组物理参数发送切换帧并检查独立 ACK。"""

    deadline = time.monotonic() + timeout
    received = bytearray()

    try:
        with serial_module.Serial(
            port=port,
            baudrate=profile.baudrate,
            bytesize=serial_module.EIGHTBITS,
            parity=profile.parity,
            stopbits=serial_module.STOPBITS_ONE,
            timeout=min(timeout, 0.05),
            write_timeout=timeout,
        ) as connection:
            connection.reset_input_buffer()
            connection.reset_output_buffer()
            connection.write(frame)
            connection.flush()

            while time.monotonic() < deadline:
                waiting = connection.in_waiting
                chunk = connection.read(waiting if waiting > 0 else 1)
                if chunk:
                    received.extend(chunk)
                    if expected_response in received:
                        return True
                    if len(received) > 64:
                        del received[:-64]
    except (serial_module.SerialException, OSError) as exc:
        print(f"  串口访问失败：{exc}")

    return False


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="轮询四组标准串口参数，把 CPU3 当前 COM 口切换到目标协议。"
    )
    parser.add_argument("--port", required=True, help="串口名称，例如 COM5")
    parser.add_argument(
        "--target", required=True, choices=tuple(PROFILES), help="目标协议"
    )
    parser.add_argument(
        "--address", type=int, default=1, help="Modbus 从站地址，默认 1"
    )
    parser.add_argument(
        "--attempts", type=int, default=2, help="每组参数发送次数，默认 2"
    )
    parser.add_argument(
        "--timeout", type=float, default=0.3, help="每次等待应答秒数，默认 0.3"
    )
    parser.add_argument(
        "--settle", type=float, default=1.2, help="切换后等待设备重配秒数，默认 1.2"
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if not 1 <= args.address <= 247:
        print("从站地址必须在 1~247 范围内。", file=sys.stderr)
        return 2
    if args.attempts < 1:
        print("每组参数发送次数必须大于 0。", file=sys.stderr)
        return 2
    if args.timeout <= 0 or args.settle < 0:
        print("超时必须大于 0，切换等待时间不能为负数。", file=sys.stderr)
        return 2

    try:
        import serial
    except ImportError:
        print("缺少 pyserial，请先安装：py -m pip install pyserial", file=sys.stderr)
        return 2

    target_profile = PROFILES[args.target]
    frame = build_switch_frame(args.address, target_profile.value)
    expected_response = build_ack_frame(args.address, target_profile.value)
    print(f"切换帧：{frame.hex(' ').upper()}")
    print(f"预期应答：{expected_response.hex(' ').upper()}")

    for profile_name in SCAN_ORDER:
        current_profile = PROFILES[profile_name]
        print(f"尝试当前参数：{current_profile.label}")
        for attempt in range(1, args.attempts + 1):
            if exchange(serial, args.port, current_profile,
                        frame, expected_response, args.timeout):
                print(f"  第 {attempt} 次收到独立 ACK，设备已接受切换。")
                time.sleep(args.settle)

                for verify_attempt in range(1, args.attempts + 1):
                    if exchange(serial, args.port, target_profile,
                                frame, expected_response, args.timeout):
                        print(f"切换成功并已在目标参数验证：{target_profile.label}")
                        return 0
                    print(f"  目标参数验证第 {verify_attempt} 次无响应。")

                print(
                    "设备已返回切换应答，但目标参数验证失败；请检查线路并通过本机菜单确认。",
                    file=sys.stderr,
                )
                return 1
            print(f"  第 {attempt} 次无匹配 ACK。")

    print("四组标准参数均未收到切换应答。", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""检查传感器和无线链路的专用故障码、日志与显示契约。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SENSOR_DIR = ROOT / "LTD_MAIN_CPU2" / "Services" / "Sensor"
LTD_DRIVER = SENSOR_DIR / "ltd_sensor_communication.c"
DSM_DRIVER = SENSOR_DIR / "dsm_sensor_communication.c"
CH9141_DRIVER = SENSOR_DIR / "ch9141_at.c"
SENSOR_SERVICE = SENSOR_DIR / "sensor.c"
WIRELESS = SENSOR_DIR / "wireless_pairing.c"
CPU3_DISPLAY = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display.c"


def read_source(path: Path) -> str:
    data = path.read_bytes()
    for encoding in ("utf-8", "gb18030"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", data, 0, 1, str(path))


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def require_pattern(text: str, pattern: str, message: str, failures: list[str]) -> None:
    require(re.search(pattern, text, re.DOTALL) is not None, message, failures)


def main() -> int:
    failures: list[str] = []
    ltd = read_source(LTD_DRIVER)
    dsm = read_source(DSM_DRIVER)
    ch9141 = read_source(CH9141_DRIVER)
    sensor = read_source(SENSOR_SERVICE)
    wireless = read_source(WIRELESS)
    display = read_source(CPU3_DISPLAY)

    generic_users: list[str] = []
    for path in SENSOR_DIR.glob("*.c"):
        if "OTHER_PERIPHERAL_CONFIG_ERROR" in read_source(path):
            generic_users.append(path.name)
    require(
        not generic_users,
        "传感器责任域仍使用 OTHER_PERIPHERAL_CONFIG_ERROR: " + ", ".join(generic_users),
        failures,
    )

    require_pattern(
        ltd,
        r"HAL_UART_Transmit_DMA[\s\S]{0,400}return\s+COMM_UART_TRANSFER_ERROR",
        "LTD/V2 发送启动失败未归入串口传输错误",
        failures,
    )
    require_pattern(
        ltd,
        r"got\s*==\s*0U\)\s*\?\s*SENSOR_DEVICE_COMM_TIMEOUT\s*:\s*SENSOR_RESP_FORMAT_ERROR",
        "LTD/V2 未区分完全无响应和短帧",
        failures,
    )
    require("DSM_V2_LogRetry" in ltd, "LTD/V2 重试日志缺少请求、应答和 UART 现场", failures)
    require("应答求和校验错误" in ltd, "LTD/V2 校验失败未保存独立阶段", failures)
    require("应答功能码不匹配" in ltd, "LTD/V2 功能码错误未保存独立阶段", failures)
    require("应答参数码不匹配" in ltd, "LTD/V2 参数码错误未保存独立阶段", failures)

    require_pattern(
        dsm,
        r"recvLen\s*==\s*0U\)\s*\?\s*SENSOR_DEVICE_COMM_TIMEOUT\s*:\s*SENSOR_RESP_FORMAT_ERROR",
        "DSM 文本协议未区分完全无响应和不完整响应",
        failures,
    )
    require("SENSOR_DEVICE_REPORTED_ERROR" in dsm, "DSM 设备主动报错未保留专用错误码", failures)
    require("UART6_FormatHexDetail" in dsm, "DSM 重试日志缺少原始帧和 UART 错误详情", failures)

    require_pattern(
        ch9141,
        r"HAL_UART_Transmit\(&huart6[\s\S]{0,300}ret\s*=\s*COMM_UART_TRANSFER_ERROR",
        "CH9141 AT 发送失败未归入串口传输错误",
        failures,
    )
    require_pattern(
        wireless,
        r"ret\s*==\s*SENSOR_DEVICE_COMM_TIMEOUT[\s\S]{0,100}"
        r"return\s+WIRELESS_HOST_COMM_TIMEOUT",
        "CH9141 无响应未映射为蓝牙主机超时",
        failures,
    )
    require("WIRELESS_SLAVE_COMM_TIMEOUT" in wireless, "蓝牙从机未连接未保留专用错误码", failures)
    require(
        "WIRELESS_RESP_FORMAT_ERROR" in wireless and "SENSOR_RESP_FORMAT_ERROR" not in wireless,
        "无线响应解析仍与传感器响应格式错误混用",
        failures,
    )

    require_pattern(
        sensor,
        r"ltd_ret\s*!=\s*NO_ERROR[\s\S]{0,500}dsm_ret\s*!=\s*NO_ERROR",
        "传感器探测未优先保留 LTD/DSM 具体错误",
        failures,
    )
    for code in (
        "SENSOR_DEVICE_COMM_TIMEOUT",
        "SENSOR_RESP_FORMAT_ERROR",
        "COMM_UART_TRANSFER_ERROR",
        "WIRELESS_RESP_FORMAT_ERROR",
        "SENSOR_DEVICE_REPORTED_ERROR",
        "WIRELESS_HOST_COMM_TIMEOUT",
        "WIRELESS_SLAVE_COMM_TIMEOUT",
    ):
        require(
            re.search(rf"case\s+{code}\s*:", display) is not None,
            f"CPU3 显示缺少 {code}",
            failures,
        )

    if failures:
        print("Sensor fault contract check failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("Sensor fault contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""检查 DSM 对外兼容契约，防止调试区寄存器口径回退。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DSM_DATA_ANALYSIS = (
    ROOT
    / "LTD_DISPLAY_CPU3"
    / "Communication"
    / "external"
    / "DSM_modbus"
    / "DSM_DataAnalysis_modbus2.c"
)
DSM_SLAVE = (
    ROOT
    / "LTD_DISPLAY_CPU3"
    / "Communication"
    / "external"
    / "DSM_modbus"
    / "DSM_SlaveModbus_modbus2.c"
)
DSM_COMMUNICATION = (
    ROOT
    / "LTD_DISPLAY_CPU3"
    / "Communication"
    / "external"
    / "DSM_modbus"
    / "DSM_communication.c"
)


def extract_function_body(source: str, name: str) -> str:
    """按括号层级截取 C 函数体，避免用整文件字符串误判寄存器写入位置。"""
    match = re.search(
        rf"\b(?:void|int|bool)\s+{re.escape(name)}\s*\([^)]*\)\s*\{{",
        source,
    )
    if match is None:
        raise AssertionError(f"{name} not found")

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

    raise AssertionError(f"{name} body is incomplete")


def normalize(text: str) -> str:
    """压缩空白字符，使契约检查不受换行和缩进影响。"""
    return re.sub(r"\s+", "", text)


def check_dsm_x_angle_offset(source: str) -> None:
    """确认 DSM 0x0104 仍按一代协议输出 X 角度偏移值。"""
    body = extract_function_body(source, "Input_Write")
    compact_body = normalize(body)

    expected = normalize(
        "WriteOneInputRegister(INPUTREGISTER_CIRCLE, 1, g_measurement.debug_data.angle_x + 0x8000)"
    )
    if expected not in compact_body:
        raise AssertionError(
            "DSM input register 0x0104 must publish X angle as angle_x + 0x8000 for first-generation compatibility"
        )


def _assert_in_order(body: str, *needles: str) -> None:
    """确认关键调用顺序，防止本地影子先于 CPU2 确认被对外暴露。"""
    positions = []
    for needle in needles:
        compact_needle = normalize(needle)
        if compact_needle not in body:
            raise AssertionError(f"DSM contract fragment not found: {needle}")
        positions.append(body.index(compact_needle))
    if positions != sorted(positions) or len(set(positions)) != len(positions):
        raise AssertionError(f"DSM contract order is invalid: {' -> '.join(needles)}")


def check_dsm_parameter_snapshot_contract(source: str) -> None:
    """确认 DSM 参数读写不会暴露未获 CPU2 ACK 的本地影子。"""
    response03 = normalize(extract_function_body(source, "Response03"))
    response16 = normalize(extract_function_body(source, "Response16"))

    read_gate = normalize(
        "!IsHoldingRegisterZeroSegment((unsigned int)startaddress, "
        "(unsigned int)registeramount) && !CPU2_CommIsAvailable()"
    )
    if read_gate not in response03:
        raise AssertionError(
            "DSM FC03 must reject CPU2-backed parameter reads while the parameter snapshot is invalid"
        )
    _assert_in_order(
        response03,
        read_gate,
        "sendframe[2] = EXCEPTIONCODE_ERRORDEVIVEBUSY",
        "ReadHoldingRegister(startaddress, registeramount, TempBuffer)",
    )

    _assert_in_order(
        response16,
        "!CPU2_CommIsAvailable()",
        "ReadHoldingRegister(startaddress, registeramount, s_dsm_holding_register_snapshot)",
        "memcpy(&s_dsm_parameter_snapshot",
        "parameter_snapshot_taken = true",
        "WriteHoldingRegister(startaddress, registeramount, TempBuffer)",
        "UpdateDeviceParamsFromLegacyRegs(startaddress, registeramount)",
        "if ((ret != 0) && parameter_snapshot_taken)",
        "memcpy((void *)&g_deviceParams",
        "WriteHoldingRegister(startaddress, registeramount, s_dsm_holding_register_snapshot)",
        "SystemParameterSet()",
        "if (ret == PARAMETER_WRITE_FAIL)",
        "sendframe[2] = EXCEPTIONCODE_ERRORDEVIVEBUSY",
    )

    if "elseif(IsHoldingRegisterZeroSegment" not in response16:
        raise AssertionError("DSM FC16 zero-filled placeholder segment must bypass CPU2 writes")


def check_dsm_function_code_contract(slave_source: str, communication_source: str) -> None:
    """确认 DSM 未意外开放 FC06，非法功能继续返回 0x86/0x01。"""
    function_body = normalize(extract_function_body(slave_source, "GetFunctioncode"))
    allowed = set(re.findall(r"\*funcode!=([A-Za-z_][A-Za-z0-9_]*)", function_body))
    expected = {
        "readcoilfuncode",
        "readholdingregisterfuncode",
        "readinputregisterfuncode",
        "presetsinglecoilfuncode",
        "presetmultipleregisterfuncode",
    }
    if allowed != expected:
        raise AssertionError(f"DSM supported function-code set changed: {sorted(allowed)}")

    dispatch_body = normalize(
        extract_function_body(communication_source, "DSM_CommunicationProcess")
    )
    _assert_in_order(
        dispatch_body,
        "if (GetFunctioncode(rcvbuff, &functioncode) == false)",
        "tx[1] = 0x80 + functioncode",
        "tx[2] = 0x01",
    )


def _request_length_reference(function_code: int, frame_length: int, byte_count: int = 0) -> bool:
    """按 DSM 分发层契约计算帧长，用于固化截短/超长 golden case。"""
    if function_code in (0x01, 0x03, 0x04, 0x05):
        return frame_length == 8
    if function_code == 0x10:
        return frame_length >= 9 and frame_length == 9 + byte_count
    return True


def check_dsm_request_length_contract(communication_source: str) -> None:
    """确认 CRC 合法的截短/超长请求不会进入无长度参数的 Response 函数。"""
    length_body = normalize(
        extract_function_body(communication_source, "DSM_IsRequestLengthValid")
    )
    for fragment in (
        "case FUNCTIONCODE_READ_COIL",
        "case FUNCTIONCODE_READ_HOLDREGISTER",
        "case FUNCTIONCODE_READ_INPUTREGISTER",
        "case FUNCTIONCODE_WRITE_COIL",
        "return (rcvcount == 8)",
        "case FUNCTIONCODE_WRITE_MULREGISTER",
        "if (rcvcount < 9)",
        "return (rcvcount == (9 + (int)rcvbuff[6]))",
    ):
        if normalize(fragment) not in length_body:
            raise AssertionError(f"DSM request-length guard missing: {fragment}")

    dispatch_body = normalize(
        extract_function_body(communication_source, "DSM_CommunicationProcess")
    )
    _assert_in_order(
        dispatch_body,
        "SlaveCheckCRC(rcvbuff, rcvcount)",
        "if (!DSM_IsRequestLengthValid(rcvbuff, rcvcount))",
        "ResponseException(rcvbuff[1], EXCEPTIONCODE_ERRORDATA, tx)",
        "if (GetFunctioncode(rcvbuff, &functioncode) == false)",
    )

    golden_cases = (
        (0x03, 8, 0, True),
        (0x03, 7, 0, False),
        (0x03, 9, 0, False),
        (0x10, 11, 2, True),
        (0x10, 9, 2, False),
        (0x10, 12, 2, False),
        (0x06, 8, 0, True),
    )
    for function_code, frame_length, byte_count, expected in golden_cases:
        actual = _request_length_reference(function_code, frame_length, byte_count)
        if actual != expected:
            raise AssertionError(
                f"DSM golden length case failed: fc=0x{function_code:02X}, "
                f"length={frame_length}, byte_count={byte_count}"
            )


def main() -> int:
    """读取 DSM 协议实现并执行当前需要固化的一代兼容检查。"""
    source = DSM_DATA_ANALYSIS.read_text(encoding="utf-8")
    slave_source = DSM_SLAVE.read_text(encoding="utf-8")
    communication_source = DSM_COMMUNICATION.read_text(encoding="utf-8")
    try:
        check_dsm_x_angle_offset(source)
        check_dsm_parameter_snapshot_contract(slave_source)
        check_dsm_function_code_contract(slave_source, communication_source)
        check_dsm_request_length_contract(communication_source)
    except AssertionError as exc:
        print(f"DSM compatibility contract check failed: {exc}")
        return 1

    print("DSM compatibility contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

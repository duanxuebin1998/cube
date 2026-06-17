#!/usr/bin/env python3
"""检查 AO 输出使能的 CPU2/CPU3 共享参数契约。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPU2_PARAM = ROOT / "LTD_MAIN_CPU2" / "Services" / "ParamStorage" / "system_parameter.h"
CPU3_PARAM = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.h"
CPU2_STATE = ROOT / "LTD_MAIN_CPU2" / "Services" / "Modbus" / "stateformodbus.h"
CPU3_STATE = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "stateformodbus.h"
CPU2_PARAM_C = ROOT / "LTD_MAIN_CPU2" / "Services" / "ParamStorage" / "system_parameter.c"
CPU2_MODBUS = ROOT / "LTD_MAIN_CPU2" / "Services" / "Modbus" / "dataanalysis_modbus.c"
CPU3_MODBUS = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "internal" / "main_board_modbus" / "dataanalysis_modbus.c"
CPU3_META = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.c"
CPU3_MENU_H = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display_tankopera.h"
CPU3_MENU_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display_tankopera.c"
CPU3_SYNC = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "internal" / "main_board_modbus" / "device_param_sync.c"
CPU2_AO = ROOT / "LTD_MAIN_CPU2" / "Services" / "AoOutput" / "ao_output.c"
PROTOCOL_DOC = ROOT / "docs" / "01_协议与寄存器" / "CPU2_CPU3协议变更记录.md"
DEFAULT_DOC = ROOT / "docs" / "01_协议与寄存器" / "系统参数出厂默认值.md"


def compact(text: str) -> str:
    return re.sub(r"\s+", "", text)


def parse_protocol_version(text: str, path: Path) -> int:
    match = re.search(r"#define\s+DEVICE_PROTOCOL_VERSION\s+(\d+)u?\b", text)
    if match is None:
        raise AssertionError(f"{path}: missing DEVICE_PROTOCOL_VERSION")
    return int(match.group(1))


def require(condition: bool, message: str, failed: list[str]) -> None:
    if not condition:
        failed.append(message)


def require_re(text: str, pattern: str, message: str, failed: list[str]) -> None:
    require(re.search(pattern, text, re.S) is not None, message, failed)


def extract_update_disabled_branch(compact_text: str) -> str:
    function_marker = "uint32_tAoOutput_Update(void){"
    function_start = compact_text.find(function_marker)
    if function_start < 0:
        return ""
    marker = "if(AoOutput_IsEnabled()==0U){"
    start = compact_text.find(marker, function_start)
    if start < 0:
        return ""
    end = compact_text.find("returnNO_ERROR;}", start)
    if end < 0:
        return ""
    return compact_text[start:end + len("returnNO_ERROR;}")]


def main() -> int:
    cpu2_param = CPU2_PARAM.read_text(encoding="gbk")
    cpu3_param = CPU3_PARAM.read_text(encoding="utf-8")
    cpu2_state = CPU2_STATE.read_text(encoding="gbk")
    cpu3_state = CPU3_STATE.read_text(encoding="utf-8")
    cpu2_param_c = CPU2_PARAM_C.read_text(encoding="gbk")
    cpu2_modbus = CPU2_MODBUS.read_text(encoding="gbk")
    cpu3_modbus = CPU3_MODBUS.read_text(encoding="utf-8")
    cpu3_meta = CPU3_META.read_text(encoding="utf-8")
    cpu3_menu_h = CPU3_MENU_H.read_text(encoding="utf-8")
    cpu3_menu_c = CPU3_MENU_C.read_text(encoding="utf-8")
    cpu3_sync = CPU3_SYNC.read_text(encoding="utf-8")
    cpu2_ao = CPU2_AO.read_text(encoding="utf-8")
    protocol_doc = PROTOCOL_DOC.read_text(encoding="utf-8")
    default_doc = DEFAULT_DOC.read_text(encoding="utf-8")

    failed: list[str] = []

    require(parse_protocol_version(cpu2_param, CPU2_PARAM) == 10, "CPU2 DEVICE_PROTOCOL_VERSION must be 10", failed)
    require(parse_protocol_version(cpu3_param, CPU3_PARAM) == 10, "CPU3 DEVICE_PROTOCOL_VERSION must be 10", failed)

    for text, name in ((cpu2_param, "CPU2"), (cpu3_param, "CPU3")):
        require("uint32_t AoOutputEnable;" in text, f"{name} DeviceParameters must expose AoOutputEnable", failed)
        require("reserved26" not in text, f"{name} DeviceParameters must not keep reserved26 field", failed)

    for state, name in ((cpu2_state, "CPU2"), (cpu3_state, "CPU3")):
        state_compact = compact(state)
        require(
            "HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE=HOLDREGISTER_DEVICEPARAM_DEBUG_CURRENT_mA+REG_STRIDE" in state_compact,
            f"{name} AO enable register must follow DebugCurrent",
            failed,
        )
        require(
            "HOLDREGISTER_DEVICEPARAM_RESERVED26=HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE" in state_compact,
            f"{name} reserved26 alias must stay on AO enable register",
            failed,
        )
        require(
            "HOLDREGISTER_DEVICEPARAM_RESERVED27=HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE+REG_STRIDE" in state_compact,
            f"{name} reserved27 must follow AO enable without shifting later fields",
            failed,
        )

    require_re(cpu2_modbus, r"HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE[\s\S]{0,120}g_deviceParams\.AoOutputEnable", "CPU2 Modbus write must publish AoOutputEnable", failed)
    require_re(cpu2_modbus, r"g_deviceParams\.AoOutputEnable\s*=\s*\(read_u32_from_regs\(regs,\s*HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE\)\s*==\s*0U\)\s*\?\s*0U\s*:\s*1U", "CPU2 Modbus read must normalize AoOutputEnable", failed)
    require_re(cpu3_modbus, r"HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE[\s\S]{0,120}g_deviceParams\.AoOutputEnable", "CPU3 Modbus write must publish AoOutputEnable", failed)
    require_re(cpu3_modbus, r"g_deviceParams\.AoOutputEnable\s*=\s*\(read_u32_from_regs\(regs,\s*HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_ENABLE\)\s*==\s*0U\)\s*\?\s*0U\s*:\s*1U", "CPU3 Modbus read must normalize AoOutputEnable", failed)

    cpu2_param_compact = compact(cpu2_param_c)
    require("g_deviceParams.AoOutputEnable=0U;" in cpu2_param_compact, "CPU2 factory/protocol upgrade must default AoOutputEnable to disabled", failed)
    require("if(g_deviceParams.AoOutputEnable>1U){g_deviceParams.AoOutputEnable=0U;" in cpu2_param_compact, "CPU2 runtime normalization must clamp AoOutputEnable", failed)

    require("COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE" in cpu3_meta, "CPU3 metadata must expose AO output enable", failed)
    require_re(cpu3_meta, r'"AO输出使能"[\s\S]{0,180}true,\s*0,\s*1', "CPU3 metadata must constrain AO output enable to 0..1", failed)
    require("COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE" in cpu3_menu_h, "CPU3 menu enum must define AO output enable", failed)
    require("COM_NUM_DEVICEPARAM_RESERVED26 = COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE" in cpu3_menu_h, "CPU3 menu reserved26 alias must be preserved", failed)
    require("AO使能" in cpu3_menu_c and "COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE" in cpu3_menu_c, "CPU3 AO menu must include enable item", failed)
    require("arr_ao_output_enable" in cpu3_menu_c, "CPU3 AO menu must define AO enable display words", failed)
    require_re(
        cpu3_menu_c,
        r"case\s+COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE\s*:\s*\{[\s\S]{0,220}arr_ao_output_enable",
        "CPU3 AO enable must map ret_arr_word to AO enable display words",
        failed,
    )
    require_re(cpu3_sync, r"case\s+COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE\s*:\s*return\s+&g_deviceParams\.AoOutputEnable;", "CPU3 device_param_sync must map AO output enable", failed)

    cpu2_ao_compact = compact(cpu2_ao)
    disabled_branch = extract_update_disabled_branch(cpu2_ao_compact)
    require("AoOutput_IsEnabled" in cpu2_ao, "CPU2 AO service must check enable flag", failed)
    require("AO_OUTPUT_SOURCE_DISABLED" in cpu2_ao, "CPU2 AO runtime must expose disabled source", failed)
    require("if(AoOutput_IsEnabled()==0U){AoOutput_SetDisabledRuntime(now);returnNO_ERROR;}" in cpu2_ao_compact, "CPU2 AO init must bypass AD5421 when disabled", failed)
    require("ao_output_driver_ready=0U;" in disabled_branch, "CPU2 AO update disabled branch must clear driver-ready state", failed)
    require("AoOutput_SetDisabledRuntime(now);" in disabled_branch, "CPU2 AO update disabled branch must publish disabled runtime", failed)
    require(("AD5421_" not in disabled_branch) and ("Ad5421Init" not in disabled_branch), "CPU2 AO update must bypass AD5421 when disabled", failed)

    require("### 协议版本 10" in protocol_doc, "protocol change record must document version 10", failed)
    require("AoOutputEnable" in protocol_doc and "0x00C2-0x00C3" in protocol_doc, "protocol change record must document AO enable address", failed)
    require("AoOutputEnable" in default_doc and "默认关闭" in default_doc, "factory defaults doc must document AO enable default", failed)

    if failed:
        print("AO output enable contract check failed:")
        for item in failed:
            print(f"- {item}")
        return 1

    print("AO output enable contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

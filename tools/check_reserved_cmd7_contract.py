#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查命令 115 保留契约，避免强制提零点执行入口被恢复。"""

from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]

CPU2_PARAM = ROOT / "LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h"
CPU2_MEASURE = ROOT / "LTD_MAIN_CPU2/Application/Src/measure.c"
CPU3_PARAM = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h"
CPU3_MENU = ROOT / "LTD_DISPLAY_CPU3/Application/display/display_tankopera.c"
CPU3_DISPLAY = ROOT / "LTD_DISPLAY_CPU3/Application/display/display.c"
CPU3_MENU_HEADER = ROOT / "LTD_DISPLAY_CPU3/Application/display/display_tankopera.h"
PROTOCOL_DOC = ROOT / "docs/01_协议与寄存器/CPU2_CPU3协议变更记录.md"


def read_text(path: Path, encoding: str = "utf-8") -> str:
    return path.read_text(encoding=encoding)


def require(condition: bool, message: str, failed: list[str]) -> None:
    if not condition:
        failed.append(message)


def main() -> int:
    failed: list[str] = []
    cpu2_param = read_text(CPU2_PARAM, "gbk")
    cpu2_measure = read_text(CPU2_MEASURE, "gbk")
    cpu3_param = read_text(CPU3_PARAM)
    cpu3_menu = read_text(CPU3_MENU)
    cpu3_display = read_text(CPU3_DISPLAY)
    cpu3_menu_header = read_text(CPU3_MENU_HEADER)
    protocol_doc = read_text(PROTOCOL_DOC)

    for text, name in ((cpu2_param, "CPU2"), (cpu3_param, "CPU3")):
        require("#define DEVICE_PROTOCOL_VERSION 12u" in text, f"{name} protocol version must be 12", failed)
        require("CMD_RESERVED_CMD7" in text, f"{name} command 115 must be reserved", failed)
        require("CMD_FORCE_LIFT_ZERO" not in text, f"{name} must not expose CMD_FORCE_LIFT_ZERO", failed)
        require("STATE_RESERVED_002F" in text, f"{name} state 0x002F must be reserved", failed)
        require("STATE_RESERVED_802F" in text, f"{name} state 0x802F must be reserved", failed)
        require("STATE_FORCE_LIFT_ZERO" not in text, f"{name} must not expose force-lift-zero states", failed)

    require("CMD_ForceLiftZero" not in cpu2_measure, "CPU2 measure.c must not keep CMD_ForceLiftZero()", failed)
    require("case CMD_RESERVED_CMD7:" in cpu2_measure, "CPU2 must route command 115 to reserved handling", failed)
    require("CMD_FORCE_LIFT_ZERO" not in cpu2_measure, "CPU2 measure.c must not dispatch CMD_FORCE_LIFT_ZERO", failed)
    require("STATE_FORCE_LIFT_ZERO" not in cpu2_measure, "CPU2 measure.c must not publish force-lift-zero states", failed)

    for text, name in ((cpu3_menu, "CPU3 menu"), (cpu3_display, "CPU3 display"), (cpu3_menu_header, "CPU3 menu header")):
        require("CMD_FORCE_LIFT_ZERO" not in text, f"{name} must not refer to CMD_FORCE_LIFT_ZERO", failed)
        require("COM_NUM_FORCE_LIFT_ZERO" not in text, f"{name} must not expose COM_NUM_FORCE_LIFT_ZERO", failed)
        require("STATE_FORCE_LIFT_ZERO" not in text, f"{name} must not display force-lift-zero states", failed)
        require("ForceLiftZero" not in text, f"{name} must not expose ForceLiftZero menu text", failed)

    require("### 协议版本 12" in protocol_doc, "protocol change record must document protocol version 12", failed)
    require("CMD_RESERVED_CMD7" in protocol_doc, "protocol change record must document command 115 reservation", failed)

    if failed:
        for item in failed:
            print(f"FAIL: {item}")
        return 1

    print("reserved command 115 contract OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

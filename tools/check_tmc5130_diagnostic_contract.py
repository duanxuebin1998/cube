#!/usr/bin/env python3
"""检查 TMC5130 通信故障快照和最终错误详情契约。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "LTD_MAIN_CPU2" / "BSP" / "Peripherals" / "inc" / "TMC5130.h"
DRIVER = ROOT / "LTD_MAIN_CPU2" / "BSP" / "Peripherals" / "src" / "TMC5130.c"
FAULT_MANAGER = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "fault_manager.c"
RECOVERY = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "fault_recovery.c"


def read_source(path: Path) -> str:
    return path.read_bytes().decode("gb18030")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def require_pattern(text: str, pattern: str, message: str, failures: list[str]) -> None:
    require(re.search(pattern, text, re.DOTALL) is not None, message, failures)


def main() -> int:
    failures: list[str] = []
    header = read_source(HEADER)
    driver = read_source(DRIVER)
    fault_manager = read_source(FAULT_MANAGER)
    recovery = read_source(RECOVERY)

    for marker in (
        "TMC5130_DIAG_STAGE_ACCESS_BUSY",
        "TMC5130_DIAG_STAGE_SPI_READ_TRIGGER",
        "TMC5130_DIAG_STAGE_SPI_READ_DATA",
        "TMC5130_DIAG_STAGE_SPI_WRITE",
        "TMC5130_DIAG_STAGE_XACTUAL_UNSTABLE",
        "TMC5130_DIAG_STAGE_CONFIGURATION_LOST",
        "TMC5130_DIAG_STAGE_CHIP_RESET",
        "TMC5130DiagnosticSnapshot",
        "TMC5130_GetDiagnosticSnapshot",
    ):
        require(marker in header, f"TMC5130 头文件缺少 {marker}", failures)

    require_pattern(
        driver,
        r"HAL_SPI_TransmitReceive[\s\S]{0,900}tmc5130_saveDiagnostic\(stage,",
        "TMC5130 SPI 读取失败未保存 HAL 状态和读帧阶段",
        failures,
    )
    require_pattern(
        driver,
        r"TMC5130_DIAG_STAGE_SPI_READ_TRIGGER[\s\S]{0,500}"
        r"TMC5130_DIAG_STAGE_SPI_READ_DATA",
        "TMC5130 两帧读取未区分触发帧和数据帧",
        failures,
    )
    require_pattern(
        driver,
        r"HAL_SPI_Transmit\([\s\S]{0,900}TMC5130_DIAG_STAGE_SPI_WRITE",
        "TMC5130 SPI 写入失败未保存寄存器和值",
        failures,
    )
    require_pattern(
        driver,
        r"XACTUAL连续读取不稳定[\s\S]{0,1000}"
        r"TMC5130_DIAG_STAGE_XACTUAL_UNSTABLE",
        "XACTUAL 三次不稳定读数未进入故障快照",
        failures,
    )
    require_pattern(
        driver,
        r"chopconf\s*==\s*0[\s\S]{0,1300}"
        r"TMC5130_DIAG_STAGE_CONFIGURATION_LOST[\s\S]{0,1000}"
        r"return\s+MOTOR_TMC_CONFIG_LOST",
        "驱动配置丢失未使用独立故障码",
        failures,
    )
    require_pattern(
        driver,
        r"if\s*\(reset_flag\)[\s\S]{0,4000}TMC5130_DIAG_STAGE_CHIP_RESET",
        "芯片复位未保存独立故障阶段",
        failures,
    )

    require("FaultManager_AppendTmcDiagnostic" in fault_manager, "最终错误详情未附加 TMC 快照", failures)
    require_pattern(
        fault_manager,
        r"FaultManager_ReportErrorExit[\s\S]{0,1800}"
        r"FaultManager_AppendTmcDiagnostic\(error_code,\s*detail",
        "普通最终错误出口未附加 TMC 快照",
        failures,
    )
    require_pattern(
        fault_manager,
        r"FaultManager_ReportGlobalErrorExit[\s\S]{0,2200}"
        r"FaultManager_AppendTmcDiagnostic\(error_code,\s*detail",
        "全局错误出口未附加 TMC 快照",
        failures,
    )
    require_pattern(
        recovery,
        r"case\s+MOTOR_TMC_COMM_ERROR\s*:",
        "TMC5130 通信故障未纳入自动重建驱动配置",
        failures,
    )
    require_pattern(
        recovery,
        r"case\s+MOTOR_TMC_CONFIG_LOST\s*:",
        "TMC5130 配置丢失未纳入自动重建驱动配置",
        failures,
    )
    require("MOTOR_TMC_CONFIG_LOST" in fault_manager, "配置丢失最终出口未附加 TMC 快照", failures)

    if failures:
        print("TMC5130 diagnostic contract check failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("TMC5130 diagnostic contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

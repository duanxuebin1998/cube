#!/usr/bin/env python3
"""检查 AD5421 故障快照、错误映射和任务态延后日志契约。"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DRIVER_HEADER = ROOT / "LTD_MAIN_CPU2" / "BSP" / "Peripherals" / "inc" / "ad5421.h"
DRIVER_SOURCE = ROOT / "LTD_MAIN_CPU2" / "BSP" / "Peripherals" / "src" / "ad5421.c"
AO_HEADER = ROOT / "LTD_MAIN_CPU2" / "Services" / "AoOutput" / "ao_output.h"
AO_SOURCE = ROOT / "LTD_MAIN_CPU2" / "Services" / "AoOutput" / "ao_output.c"
APP_SOURCE = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "app_main.c"


def read_source(path: Path) -> str:
    return path.read_bytes().decode("gb18030")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def require_pattern(text: str, pattern: str, message: str, failures: list[str]) -> None:
    require(re.search(pattern, text, re.DOTALL) is not None, message, failures)


def main() -> int:
    failures: list[str] = []
    driver_header = read_source(DRIVER_HEADER)
    driver_source = read_source(DRIVER_SOURCE)
    ao_header = read_source(AO_HEADER)
    ao_source = read_source(AO_SOURCE)
    app_source = read_source(APP_SOURCE)

    for marker in (
        "AD5421_DIAG_STAGE_ACCESS_BUSY",
        "AD5421_DIAG_STAGE_SPI_WRITE",
        "AD5421_DIAG_STAGE_SPI_READ_COMMAND",
        "AD5421_DIAG_STAGE_SPI_READ_DATA",
        "AD5421_DIAG_STAGE_CONTROL_READBACK",
        "AD5421_DIAG_STAGE_FAULT_STATUS",
        "AD5421DiagnosticSnapshot",
        "AD5421_GetDiagnosticSnapshot",
    ):
        require(marker in driver_header, f"AD5421 头文件缺少 {marker}", failures)

    require_pattern(
        driver_source,
        r"HAL_SPI_Transmit\(&hspi3,\s*commandData[\s\S]{0,500}"
        r"AD5421_DIAG_STAGE_SPI_READ_COMMAND",
        "SPI 读命令失败未保存独立诊断阶段",
        failures,
    )
    require_pattern(
        driver_source,
        r"HAL_SPI_TransmitReceive\(&hspi3,\s*dummyData[\s\S]{0,500}"
        r"AD5421_DIAG_STAGE_SPI_READ_DATA",
        "SPI 读数据失败未保存独立诊断阶段",
        failures,
    )
    require_pattern(
        driver_source,
        r"readback\s*!=\s*controldata[\s\S]{0,900}"
        r"AD5421_DIAG_STAGE_CONTROL_READBACK",
        "控制寄存器不一致未保存期望值和实际值",
        failures,
    )
    require_pattern(
        driver_source,
        r"fault_reg\s*!=\s*0U[\s\S]{0,900}AD5421_DIAG_STAGE_FAULT_STATUS",
        "芯片主动故障未与读取失败分开保存",
        failures,
    )
    require_pattern(
        driver_source,
        r"AD5421_DIAG_STAGE_FAULT_STATUS[\s\S]{0,700}return\s+AD5421_FAULT_STATUS_ERROR",
        "芯片主动报警仍与故障寄存器读取失败共用错误码",
        failures,
    )
    require_pattern(
        driver_source,
        r"AD5421_PromoteDiagnostic\(AD5421_WRITE_CURRENT_ERROR,\s*ret\)",
        "写电流兼容错误码未保留底层根因",
        failures,
    )
    require_pattern(
        driver_source,
        r"AD5421_PromoteDiagnostic\(AD5421_READFAULT_ERROR,\s*ret\)",
        "故障寄存器兼容错误码未保留底层根因",
        failures,
    )
    require_pattern(
        driver_source,
        r"ret\s*=\s*AD5421_WriteReg\(RESETAD5421REG,[^;]+\);[\s\S]{0,400}"
        r"AD5421_PromoteDiagnostic\(AD5421_INIT_ERROR,\s*ret\)",
        "初始化复位写失败仍可能泄漏通用外设错误码",
        failures,
    )
    require_pattern(
        driver_source,
        r"AD5421_WriteRegRaw\(WRITECONTROL,[^;]+\);[\s\S]{0,400}"
        r"AD5421_PromoteDiagnostic\(AD5421_INIT_ERROR,\s*ret\)",
        "控制寄存器写失败未收敛为 AD5421 初始化错误",
        failures,
    )
    require_pattern(
        driver_source,
        r"AD5421_ReadRegCheckedRaw\(READCONTROL,[^;]+\);[\s\S]{0,400}"
        r"AD5421_PromoteDiagnostic\(AD5421_INIT_ERROR,\s*ret\)",
        "控制寄存器读取失败未收敛为 AD5421 初始化错误",
        failures,
    )

    require(
        "AoOutput_ProcessDeferredDiagnostics" in ao_header,
        "AO 头文件缺少任务态延后日志接口",
        failures,
    )
    require("AoOutput_QueueDriverError" in ao_source, "AO 未保存驱动故障日志事件", failures)
    require("AoOutput_QueueDriverRecovery" in ao_source, "AO 未保存驱动恢复日志事件", failures)
    require("AoOutput_IsSameDiagnostic" in ao_source, "AO 未抑制相同持续故障日志", failures)
    require("ErrorLog_WarnDetail" in ao_source, "AO 故障未进入统一详细日志接口", failures)
    require("ErrorLog_RecoverDetail" in ao_source, "AO 恢复未进入统一恢复日志接口", failures)
    require(
        "OTHER_PERIPHERAL_CONFIG_ERROR" not in ao_source,
        "AO 服务公开出口仍兼容吞掉通用外设错误码",
        failures,
    )
    require("AD5421_FAULT_STATUS_ERROR" in ao_source, "AO 运行态未识别芯片主动报警码", failures)
    require_pattern(
        ao_source,
        r"recover_ret\s*!=\s*AD5421_FAULT_STATUS_ERROR",
        "AO 恢复边界仍把读取失败当成芯片主动报警",
        failures,
    )
    require_pattern(
        app_source,
        r"HostCommu_ProcessDeferredLogs\(\);\s*AoOutput_ProcessDeferredDiagnostics\(\);",
        "AD5421 诊断日志未由应用主循环任务态消费",
        failures,
    )

    if failures:
        print("AD5421 diagnostic contract check failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("AD5421 diagnostic contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""检查参数、测量和位置故障码的静态语义约束。"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read_ascii_safe(path: Path) -> str:
    """按单字节读取源码，避免历史 GBK/混合编码影响英文符号检查。"""

    return path.read_bytes().decode("latin-1")


def require_pattern(path: Path, pattern: str, message: str) -> list[str]:
    text = read_ascii_safe(path)
    if re.search(pattern, text, re.MULTILINE | re.DOTALL) is None:
        return [f"{path.relative_to(ROOT)}: {message}"]
    return []


def main() -> int:
    failures: list[str] = []

    cpu2_root = ROOT / "LTD_MAIN_CPU2"
    measure_file = cpu2_root / "Application/Src/measure.c"
    tank_file = cpu2_root / "Application/Src/measure_tank_height.c"
    wartsila_file = cpu2_root / "Application/Src/wartsila_density_measurement.c"
    motion_file = cpu2_root / "Services/MotorControl/motor_ctrl_motion_api.c"
    param_file = cpu2_root / "Services/ParamStorage/system_parameter.c"
    test_file = cpu2_root / "Application/Src/test.c"

    failures += require_pattern(
        measure_file,
        r"calibrateTankHeight\s*==\s*0[^{}]*\{[^{}]*SET_ERROR\(PARAM_RANGE_ERROR\)",
        "罐高标定值为 0 时必须上报参数范围错误",
    )
    failures += require_pattern(
        measure_file,
        r"raw_real_height\s*==\s*0U[^{}]*\{[^{}]*SET_ERROR\(MEASUREMENT_POSITION_ERROR\)",
        "原始实高为 0 时必须上报位置测量错误",
    )
    failures += require_pattern(
        tank_file,
        r"status\s*==\s*NULL[^{}]*\{\s*return\s+PARAM_ADDRESS_OVERFLOW;",
        "罐底状态输出指针为空时必须上报参数地址越界",
    )
    failures += require_pattern(
        tank_file,
        r"!g_gyro_zero_ref\.valid[^{}]*\{[^{}]*return\s+MEASUREMENT_ZERO_REPEAT_FAIL;",
        "陀螺仪零点基准无效时必须留在测量零点责任域",
    )
    failures += require_pattern(
        wartsila_file,
        r"Wartsila_MoveToDensityPoint[\s\S]*?return\s+MEASUREMENT_POSITION_ERROR;",
        "瓦锡兰测点定位偏差必须保留位置测量错误",
    )
    failures += require_pattern(
        motion_file,
        r"MotorMotion_IsPositionSnapshotValid[\s\S]*?return\s+MEASUREMENT_POSITION_ERROR;",
        "电机位置快照异常必须保留位置测量错误",
    )
    failures += require_pattern(
        param_file,
        r"DEVICE_PARAM_SLOT_CRC_ERROR[\s\S]*?return\s+PARAM_CRC_ERROR;",
        "A/B 参数分区校验失败必须优先上报参数 CRC 错误",
    )
    failures += require_pattern(
        param_file,
        r"DEVICE_PARAM_SLOT_UNINITIALIZED\)\s*&&[\s\S]{0,160}"
        r"DEVICE_PARAM_SLOT_UNINITIALIZED\)\)\s*\{\s*return\s+PARAM_UNINITIALIZED;",
        "A/B 参数分区均无初始化标志时必须上报参数未初始化",
    )
    failures += require_pattern(
        param_file,
        r"ErrorLog_Retry\([\s\S]{0,500}ErrorLog_GetReasonByCode\(load_error_code\)"
        r"[\s\S]{0,300}load_error_code\);",
        "参数加载重试日志不得把具体分类重新压成参数存储异常",
    )
    failures += require_pattern(
        test_file,
        r"g_measurement\.device_status\.error_code\s*==\s*PARAM_UNINITIALIZED"
        r"[\s\S]{0,300}printf\([^;]*PARAM_UNINITIALIZED",
        "FRAM A/B 现有台架测试未同步参数未初始化分类",
    )

    pointer_guard = re.compile(
        r"if\s*\([^{};]*\bNULL\b[^{};]*\)\s*\{\s*"
        r"[^{}]*?return\s+(?:\([^;]*\)\s*\?\s*[^:;]+\s*:\s*)?PARAM_ERROR;",
        re.MULTILINE | re.DOTALL,
    )
    for path in cpu2_root.rglob("*.c"):
        text = read_ascii_safe(path)
        for match in pointer_guard.finditer(text):
            line = text.count("\n", 0, match.start()) + 1
            failures.append(
                f"{path.relative_to(ROOT)}:{line}: 空指针不得上报 PARAM_ERROR，应使用 PARAM_ADDRESS_OVERFLOW"
            )

    forbidden_dirs = [
        cpu2_root / "Application/Src",
        cpu2_root / "Services/Sensor",
    ]
    for directory in forbidden_dirs:
        for path in directory.rglob("*.c"):
            text = read_ascii_safe(path)
            for match in re.finditer(r"\bOTHER_PERIPHERAL_CONFIG_ERROR\b", text):
                line = text.count("\n", 0, match.start()) + 1
                failures.append(
                    f"{path.relative_to(ROOT)}:{line}: 业务链路不得使用外设配置通用兜底码"
                )

    if failures:
        print("参数/测量/位置故障码契约检查失败：")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("参数/测量/位置故障码契约检查通过")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

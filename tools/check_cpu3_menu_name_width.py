#!/usr/bin/env python3
"""检查 CPU3 参数列表页短名不会被数值列裁剪。"""

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
DISPLAY_TANKOPERA_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display_tankopera.c"
SYSTEM_PARAMETER_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.c"

OLED_LINE8_6 = 35
ASCII_WIDTH = 4
HANZI_WIDTH = 7


EXPECTED_SHORT_NAMES = {
    "COM_NUM_DEVICEPARAM_SENSOR_SOFTWARE_VERSION": "传感器版本",
    "COM_NUM_DEVICEPARAM_MAGIC": "魔术字",
    "COM_NUM_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM": "编码轮周长",
    "COM_NUM_DEVICEPARAM_WEIGHT_IGNORE_ZONE": "称重忽略区",
    "COM_NUM_DEVICEPARAM_FINDZERO_DOWN_DISTANCE": "找零距离",
    "COM_NUM_DEVICEPARAM_WATER_CAP_THRESHOLD": "跟随阈值",
    "COM_NUM_DEVICEPARAM_WATER_FIND_CAP_THRESHOLD": "寻找阈值",
    "COM_NUM_DEVICEPARAM_MAXDOWNDISTANCE": "下行距离",
    "COM_NUM_DEVICEPARAM_WATER_STABLE_THRESHOLD": "稳定阈值",
    "COM_NUM_DEVICEPARAM_WATER_LAG_CAP_THRESHOLD": "滞后阈值",
    "COM_NUM_DEVICEPARAM_BOTTOM_ANGLE_THRESHOLD": "角度阈值",
    "COM_NUM_DEVICEPARAM_BOTTOM_WEIGHT_THRESHOLD": "称重阈值",
    "COM_NUM_DEVICEPARAM_SPREAD_POINT_HOVER_TIME": "悬停时间",
    "COM_NUM_DEVICEPARAM_SPREADTOPLIMIT": "最高距液面",
    "COM_NUM_DEVICEPARAM_SPREADBOTTOMLIMIT": "最低距罐底",
    "COM_NUM_DEVICEPARAM_WARTSILA_MAX_HEIGHT_ABOVE_SURFACE": "最高距液面",
    "COM_NUM_DEVICEPARAM_TAPE_EXPANSION_COEFFICIENT": "尺带伸缩率",
    "COM_NUM_DEVICEPARAM_AO_START_LEVEL": "起点液位",
    "COM_NUM_DEVICEPARAM_AO_END_LEVEL": "终点液位",
    "COM_NUM_DEVICEPARAM_AO_NORMAL_CURRENT_START_mA": "起点电流",
    "COM_NUM_DEVICEPARAM_AO_NORMAL_CURRENT_END_mA": "终点电流",
    "COM_NUM_DEVICEPARAM_AO_HIGH_ALARM_LEVEL": "高报液位",
    "COM_NUM_DEVICEPARAM_AO_LOW_ALARM_LEVEL": "低报液位",
    "COM_NUM_DEVICEPARAM_INITIAL_CURRENT_mA": "初始电流",
    "COM_NUM_DEVICEPARAM_AO_HIGH_CURRENT_mA": "高位电流",
    "COM_NUM_DEVICEPARAM_AO_LOW_CURRENT_mA": "低位电流",
    "COM_NUM_DEVICEPARAM_FAULT_CURRENT_mA": "故障电流",
    "COM_NUM_DEVICEPARAM_DEBUG_CURRENT_mA": "调试电流",
    "COM_NUM_DEVICEPARAM_SP_MEAS_POSITION": "测量位置",
    "COM_NUM_DEVICEPARAM_SP_MONITOR_POSITION": "监测位置",
    "COM_NUM_DEVICEPARAM_MOTOR_COMMAND_DISTANCE": "指令距离",
}


COMMAND_PARAMETER_OPERAS = {
    "COM_NUM_SINGLE_POINT",
    "COM_NUM_SP_TEST",
    "COM_NUM_RUN_TO_POSITION",
    "COM_NUM_RUNUP",
    "COM_NUM_RUNDOWN",
    "COM_NUM_FORCE_RUNUP",
    "COM_NUM_FORCE_RUNDOWN",
}

RELAY_FIELD_SHORT_NAMES = {
    "OPERATING_MODE": "工作模式",
    "DIGITAL_SOURCE": "报警组合",
    "CONTACT_TYPE": "接点类型",
    "ALARM_MODE": "报警模式",
    "ERROR_VALUE": "无效报警",
    "ALARM_SOURCE": "报警源",
    "HH_ALARM_VALUE": "HH阈值",
    "H_ALARM_VALUE": "H阈值",
    "L_ALARM_VALUE": "L阈值",
    "LL_ALARM_VALUE": "LL阈值",
    "ALARM_HYSTERESIS": "报警滞回",
    "DAMPING_FACTOR": "阻尼系数",
    "CLEAR_ALARM": "清除锁存",
}

for relay_channel in range(1, 5):
    for field_name, short_name in RELAY_FIELD_SHORT_NAMES.items():
        EXPECTED_SHORT_NAMES[f"COM_NUM_DEVICEPARAM_RELAY{relay_channel}_{field_name}"] = short_name


def text_width(text: str) -> int:
    """按 display_tankopera.c 中 oled_text_width() 的规则估算列宽。"""
    width = 0
    for char in text:
        width += ASCII_WIDTH if ord(char) < 128 else HANZI_WIDTH
    return width


def extract_function_body(source: str, name: str) -> str:
    """截取 C 函数体，用于轻量级契约检查。"""
    match = re.search(
        rf"\bstatic\s+[A-Za-z_][\w\s]*\*?\s*{re.escape(name)}\s*\([^)]*\)\s*\{{",
        source,
    )
    if match is None:
        raise AssertionError(f"function not found: {name}")

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

    raise AssertionError(f"function body not closed: {name}")


def parse_short_names(source: str) -> dict[str, str]:
    body = extract_function_body(source, "dtm_operaname_short")
    entries = re.findall(r'\{\s*(COM_NUM_[A-Za-z0-9_]+)\s*,\s*\(uint8_t\*\)"([^"]+)"', body)

    if not entries:
        raise AssertionError("未找到 dtm_operaname_short 短名映射")

    return dict(entries)


def parse_parameter_names(source: str) -> dict[str, str]:
    entries: dict[str, str] = {}

    for line in source.splitlines():
        if "{(uint8_t*)" not in line or "COM_NUM_" not in line:
            continue

        name_match = re.search(r'\{\s*\(uint8_t\*\)"([^"]+)"', line)
        opera_match = re.search(r'\b(COM_NUM_[A-Za-z0-9_]+)\b', line)
        if name_match is None or opera_match is None:
            continue

        name = name_match.group(1)
        if name.startswith("保留"):
            continue

        entries[opera_match.group(1)] = name

    if not entries:
        raise AssertionError("未找到参数名称表")

    return entries


def check_expected_short_names(short_names: dict[str, str]) -> list[tuple[str, str, str]]:
    return [
        (opera, expected, short_names.get(opera, ""))
        for opera, expected in sorted(EXPECTED_SHORT_NAMES.items())
        if short_names.get(opera) != expected
    ]


def main() -> int:
    tankopera_source = DISPLAY_TANKOPERA_C.read_text(encoding="utf-8")
    parameter_source = SYSTEM_PARAMETER_C.read_text(encoding="utf-8")
    short_names = parse_short_names(tankopera_source)
    parameter_names = parse_parameter_names(parameter_source)
    effective_names = {
        opera: short_names.get(opera, name)
        for opera, name in parameter_names.items()
    }

    mismatched = check_expected_short_names(short_names)
    if mismatched:
        print("CPU3 参数列表短名期望检查失败:")
        for opera, expected, actual in mismatched:
            print(f"- {opera}: expected={expected}, actual={actual or '<missing>'}")
        return 1

    too_wide = [
        (opera, name, text_width(name))
        for opera, name in sorted(effective_names.items())
        if opera not in COMMAND_PARAMETER_OPERAS and text_width(name) > OLED_LINE8_6
    ]

    if too_wide:
        print("CPU3 参数列表短名宽度检查失败:")
        for opera, name, width in too_wide:
            print(f"- {opera}: {name} width={width}, limit={OLED_LINE8_6}")
        return 1

    print("CPU3 参数列表短名宽度检查通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())

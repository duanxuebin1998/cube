#!/usr/bin/env python3
"""核对 CPU2/CPU3 故障码目录、日志名称和显示原因覆盖。"""

from __future__ import annotations

import re
import zipfile
from collections import Counter
from pathlib import Path
from xml.etree import ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
CPU2_ENUM = ROOT / "LTD_MAIN_CPU2/Services/ParamStorage/system_parameter.h"
CPU3_ENUM = ROOT / "LTD_DISPLAY_CPU3/Application/system_param/system_parameter.h"
CPU2_ERROR_LOG = ROOT / "LTD_MAIN_CPU2/Services/Utilities/error_log.c"
CPU3_DISPLAY = ROOT / "LTD_DISPLAY_CPU3/Application/display/display.c"
FORMAL_WORKBOOK = ROOT / "docs/00_构建与版本/故障码/LTD故障代码统一表.xlsx"
CPU3_FLOW_GENERATOR = ROOT / "tools/generate_cpu3_flow_docs.py"
CPU3_LOCAL_CODES = {"CPU2_COMM_TIMEOUT"}
NON_FAULT_CODES = {"NO_ERROR", "STATE_SWITCH"}
RESERVED_CODES = {
    "MOTOR_FAIL_SETTING",
    "MOTOR_RESET_FAIL",
    "ENCODER_POWERON_CHANGE",
    "SENSOR_TEMPERATURE_ERROR",
    "SENSOR_VOLTAGE_ERROR",
    "SLIPRING_BCC_ERROR",
    "SLIPRING_PACKET_LOSS",
    "DENSITY_UNSTABLE",
    "MEASUREMENT_HEIGHT_DEVIATION",
    "MEASUREMENT_DENSITY_RANGE_INVALID",
    "WEIGHT_SENSOR_SATURATION",
    "OTHER_UNKNOWN_ERROR",
    "OTHER_ADDRESS_READ_ERROR",
    "OTHER_POWER_FLUCTUATION",
    "AD5421_FAULT_PIN_ERROR",
}
RESERVED_REFERENCE_ALLOWLIST = {
    "AD5421_FAULT_PIN_ERROR": {"Services/AoOutput/ao_output.c"},
}
RUNTIME_REFERENCE_EXCLUDES = {
    "Application/Src/test.c",
    "Services/ParamStorage/system_parameter.h",
    "Services/Utilities/error_log.c",
}

SHEET_NS = "http://schemas.openxmlformats.org/spreadsheetml/2006/main"
OFFICE_REL_NS = "http://schemas.openxmlformats.org/officeDocument/2006/relationships"
PACKAGE_REL_NS = "http://schemas.openxmlformats.org/package/2006/relationships"


def read_ascii_safe(path: Path) -> str:
    return path.read_bytes().decode("latin-1")


def parse_error_codes(path: Path) -> dict[str, int]:
    text = read_ascii_safe(path)
    end = text.find("} ErrorCode;")
    anchor = text.rfind("typedef enum", 0, end)
    if anchor < 0 or end < 0:
        raise ValueError(f"{path}: 未找到 ErrorCode 枚举")
    block = text[anchor:end]
    entries = re.findall(
        r"^\s*([A-Z][A-Z0-9_]*)\s*=\s*(0x[0-9A-Fa-f]+|[0-9]+)\s*,?",
        block,
        re.MULTILINE,
    )
    if not entries:
        raise ValueError(f"{path}: ErrorCode 枚举为空")
    return {name: int(value, 0) for name, value in entries}


def parse_integer_macro(path: Path, macro_name: str) -> int:
    """读取带可选无符号后缀的整数宏。"""

    text = read_ascii_safe(path)
    match = re.search(
        rf"^\s*#define\s+{re.escape(macro_name)}\s+(0x[0-9A-Fa-f]+|[0-9]+)[uUlL]*\b",
        text,
        re.MULTILINE,
    )
    if match is None:
        raise ValueError(f"{path}: 未找到整数宏 {macro_name}")
    return int(match.group(1), 0)


def check_current_protocol_documents(protocol_version: int) -> list[str]:
    """检查现行文档是否跟随源码协议版本，历史记录不参与强制替换。"""

    contracts = (
        (
            ROOT / "docs/00_构建与版本/版本总览.md",
            (f"| 共享协议版本 | `{protocol_version}` |", "协议 14 与协议 15 的故障码不能混用"),
        ),
        (
            ROOT / "docs/00_构建与版本/README.md",
            (f"当前共享协议版本 {protocol_version}",),
        ),
        (
            ROOT / "docs/01_协议与寄存器/README.md",
            (f"`DEVICE_PROTOCOL_VERSION = {protocol_version}`",),
        ),
        (
            ROOT / "docs/01_协议与寄存器/LTD共享Modbus协议/README.md",
            (f"`DEVICE_PROTOCOL_VERSION = {protocol_version}`", "32 位故障码"),
        ),
        (
            ROOT / "docs/01_协议与寄存器/LTD共享Modbus协议/LTD共享Modbus协议卷.md",
            (f"`DEVICE_PROTOCOL_VERSION = {protocol_version}`", "协议 14 与协议 15 的故障码解释不兼容"),
        ),
        (
            ROOT / "docs/04_界面与菜单/CPU3状态页不同状态显示信息确认表.md",
            (f"当前源码共享协议版本 `{protocol_version}`",),
        ),
        (
            ROOT / "docs/04_界面与菜单/CPU3参数菜单优化方案.md",
            (f"`DEVICE_PROTOCOL_VERSION` 为 {protocol_version}", "协议 14 及以前的故障编号不能用于解释协议 15"),
        ),
        (
            ROOT / "docs/01_协议与寄存器/CPU2通信与解耦/CPU2串口调试命令协议卷.md",
            (f"共享协议版本 `{protocol_version}`", "`ERR?` 和统一错误日志输出完整 32 位内部故障码"),
        ),
        (
            ROOT / "docs/01_协议与寄存器/SI协议适配/02_协议映射/README.md",
            (f"`DEVICE_PROTOCOL_VERSION = {protocol_version}`", "SI Profile 功能从协议 14 起支持"),
        ),
        (
            ROOT / "docs/01_协议与寄存器/SI协议适配/02_协议映射/SI协议兼容映射表.md",
            (f"`DEVICE_PROTOCOL_VERSION = {protocol_version}`",),
        ),
        (
            ROOT / "docs/01_协议与寄存器/DSM协议适配/02_协议映射/DSM_V1.228一代协议卷整理.md",
            (f"`DEVICE_PROTOCOL_VERSION = {protocol_version}`", "INPUTREGISTER_ERRORNUM", "设备内部故障码不是同一类编号"),
        ),
    )

    failures: list[str] = []
    for path, required_fragments in contracts:
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            failures.append(f"现行协议文档读取失败 {path.relative_to(ROOT)}: {exc}")
            continue
        for fragment in required_fragments:
            if fragment not in text:
                failures.append(
                    f"现行协议文档未同步 {path.relative_to(ROOT)}: 缺少 {fragment}"
                )

    try:
        generator_text = CPU3_FLOW_GENERATOR.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as exc:
        failures.append(f"CPU3 流程生成器读取失败: {exc}")
    else:
        dynamic_fragment = "当前 DEVICE_PROTOCOL_VERSION 为 {DEVICE_PROTOCOL_VERSION}"
        if dynamic_fragment not in generator_text:
            failures.append("CPU3 流程生成器未使用 DEVICE_PROTOCOL_VERSION 动态生成协议版本说明")
        if re.search(r"当前 DEVICE_PROTOCOL_VERSION 为 \d+", generator_text):
            failures.append("CPU3 流程生成器仍硬编码协议版本数字")

    return failures


def duplicate_values(codes: dict[str, int]) -> dict[int, list[str]]:
    grouped: dict[int, list[str]] = {}
    for name, value in codes.items():
        grouped.setdefault(value, []).append(name)
    return {value: names for value, names in grouped.items() if len(names) > 1}


def strip_c_comments_and_strings(text: str) -> str:
    """移除注释和字符串，同时保留换行，避免把说明文字误判为运行引用。"""

    def keep_newlines(match: re.Match[str]) -> str:
        return "\n" * match.group(0).count("\n")

    text = re.sub(r"/\*.*?\*/", keep_newlines, text, flags=re.DOTALL)
    text = re.sub(r"//[^\r\n]*", "", text)
    text = re.sub(r'"(?:\\.|[^"\\])*"', "", text)
    return re.sub(r"'(?:\\.|[^'\\])*'", "", text)


def find_reserved_runtime_references() -> dict[str, list[str]]:
    """查找预留码在 CPU2 运行源码中的引用，防止代码启用后正式表仍标成保留。"""

    references: dict[str, list[str]] = {name: [] for name in RESERVED_CODES}
    cpu2_root = ROOT / "LTD_MAIN_CPU2"
    paths = sorted(cpu2_root.rglob("*.c")) + sorted(cpu2_root.rglob("*.h"))
    for path in paths:
        relative = path.relative_to(cpu2_root).as_posix()
        if relative in RUNTIME_REFERENCE_EXCLUDES or "Drivers/" in relative:
            continue
        text = strip_c_comments_and_strings(read_ascii_safe(path))
        for name in RESERVED_CODES:
            if re.search(rf"\b{re.escape(name)}\b", text):
                references[name].append(relative)
    return {name: paths for name, paths in references.items() if paths}


def read_xlsx_shared_strings(archive: zipfile.ZipFile) -> list[str]:
    """读取工作簿共享字符串；没有 sharedStrings 时返回空列表。"""

    try:
        root = ET.fromstring(archive.read("xl/sharedStrings.xml"))
    except KeyError:
        return []
    return ["".join(node.text or "" for node in item.iter(f"{{{SHEET_NS}}}t")) for item in root]


def resolve_sheet_path(archive: zipfile.ZipFile, sheet_name: str) -> str:
    """根据工作表名称解析到 xlsx 压缩包内的 worksheet XML。"""

    workbook = ET.fromstring(archive.read("xl/workbook.xml"))
    relation_id = None
    for sheet in workbook.iter(f"{{{SHEET_NS}}}sheet"):
        if sheet.attrib.get("name") == sheet_name:
            relation_id = sheet.attrib.get(f"{{{OFFICE_REL_NS}}}id")
            break
    if relation_id is None:
        raise ValueError(f"正式工作簿缺少工作表：{sheet_name}")

    relationships = ET.fromstring(archive.read("xl/_rels/workbook.xml.rels"))
    for relation in relationships.iter(f"{{{PACKAGE_REL_NS}}}Relationship"):
        if relation.attrib.get("Id") != relation_id:
            continue
        target = relation.attrib.get("Target", "").replace("\\", "/")
        if target.startswith("/"):
            return target.lstrip("/")
        return f"xl/{target}"
    raise ValueError(f"正式工作簿无法解析工作表关系：{sheet_name}")


def read_cell_text(cell: ET.Element, shared_strings: list[str]) -> str:
    """读取字符串、内联字符串或普通数值单元格。"""

    cell_type = cell.attrib.get("t")
    if cell_type == "inlineStr":
        return "".join(node.text or "" for node in cell.iter(f"{{{SHEET_NS}}}t"))
    value = cell.find(f"{{{SHEET_NS}}}v")
    if value is None or value.text is None:
        return ""
    if cell_type == "s":
        return shared_strings[int(value.text)]
    return value.text


def read_formal_workbook_statuses() -> dict[str, str]:
    """读取正式表中的屏幕显示码和使用状态两列。"""

    with zipfile.ZipFile(FORMAL_WORKBOOK) as archive:
        shared_strings = read_xlsx_shared_strings(archive)
        sheet_path = resolve_sheet_path(archive, "故障代码")
        sheet = ET.fromstring(archive.read(sheet_path))

    statuses: dict[str, str] = {}
    for row in sheet.iter(f"{{{SHEET_NS}}}row"):
        values: dict[str, str] = {}
        for cell in row.findall(f"{{{SHEET_NS}}}c"):
            reference = cell.attrib.get("r", "")
            column_match = re.match(r"([A-Z]+)", reference)
            if column_match is not None:
                values[column_match.group(1)] = read_cell_text(cell, shared_strings)
        display_code = values.get("D", "")
        status = values.get("H", "")
        if re.fullmatch(r"\d+-\d+", display_code):
            if display_code in statuses:
                raise ValueError(f"正式工作簿屏幕显示码重复：{display_code}")
            statuses[display_code] = status
    return statuses


def display_code(value: int) -> str:
    """把共享数值转换为屏幕十进制“类别代码-故障码”。"""

    return f"{value >> 16}-{value & 0xFFFF}"


def main() -> int:
    failures: list[str] = []
    protocol_version: int | None = None
    try:
        cpu2_codes = parse_error_codes(CPU2_ENUM)
        cpu3_codes = parse_error_codes(CPU3_ENUM)
    except ValueError as exc:
        print(exc)
        return 1

    try:
        cpu2_protocol_version = parse_integer_macro(CPU2_ENUM, "DEVICE_PROTOCOL_VERSION")
        cpu3_protocol_version = parse_integer_macro(CPU3_ENUM, "DEVICE_PROTOCOL_VERSION")
    except ValueError as exc:
        failures.append(str(exc))
    else:
        if cpu2_protocol_version != cpu3_protocol_version:
            failures.append(
                "CPU2/CPU3 共享协议版本不一致: "
                f"CPU2={cpu2_protocol_version}, CPU3={cpu3_protocol_version}"
            )
        else:
            protocol_version = cpu2_protocol_version

    cpu2_names = set(cpu2_codes)
    cpu3_names = set(cpu3_codes)
    missing_on_cpu3 = sorted(cpu2_names - cpu3_names)
    unexpected_cpu3 = sorted((cpu3_names - cpu2_names) - CPU3_LOCAL_CODES)
    if missing_on_cpu3:
        failures.append(f"CPU3 缺少共享故障码: {', '.join(missing_on_cpu3)}")
    if unexpected_cpu3:
        failures.append(f"CPU3 出现未登记的本机故障码: {', '.join(unexpected_cpu3)}")

    for name in sorted(cpu2_names & cpu3_names):
        if cpu2_codes[name] != cpu3_codes[name]:
            failures.append(
                f"{name} 数值不一致: CPU2=0x{cpu2_codes[name]:08X}, CPU3=0x{cpu3_codes[name]:08X}"
            )

    for label, codes in (("CPU2", cpu2_codes), ("CPU3", cpu3_codes)):
        for value, names in duplicate_values(codes).items():
            failures.append(f"{label} 故障码数值重复 0x{value:08X}: {', '.join(names)}")

    cpu2_log_text = read_ascii_safe(CPU2_ERROR_LOG)
    cpu2_case_counts = Counter(re.findall(r"\bcase\s+([A-Z][A-Z0-9_]*)\s*:", cpu2_log_text))
    for name in sorted(cpu2_names - NON_FAULT_CODES):
        if cpu2_case_counts[name] < 2:
            failures.append(
                f"CPU2 日志映射未完整覆盖 {name}: case 出现 {cpu2_case_counts[name]} 次，至少需要原因和名称两处"
            )

    cpu3_display_text = read_ascii_safe(CPU3_DISPLAY)
    cpu3_display_cases = set(
        re.findall(r"\bcase\s+([A-Z][A-Z0-9_]*)\s*:", cpu3_display_text)
    )
    for name in sorted(cpu3_names - NON_FAULT_CODES):
        if name not in cpu3_display_cases:
            failures.append(f"CPU3 显示原因未覆盖 {name}")

    display_contracts = (
        r"err_type\s*=\s*\(g_measurement\.device_status\.error_code\s*>>\s*16\)",
        r"err_pos\s*=\s*g_measurement\.device_status\.error_code\s*&\s*0xFFFF",
        r'snprintf\([^;]*"%d-%d"',
    )
    for pattern in display_contracts:
        if re.search(pattern, cpu3_display_text, re.MULTILINE | re.DOTALL) is None:
            failures.append("CPU3 主状态页未保持十进制“故障类别-故障码”显示契约")
            break

    reserved_references = find_reserved_runtime_references()
    for name, paths in sorted(reserved_references.items()):
        allowed = RESERVED_REFERENCE_ALLOWLIST.get(name, set())
        unexpected = sorted(set(paths) - allowed)
        if unexpected:
            failures.append(f"预留码 {name} 已出现运行引用，应复核并改为使用中: {', '.join(unexpected)}")

    try:
        workbook_statuses = read_formal_workbook_statuses()
    except (KeyError, OSError, ValueError, zipfile.BadZipFile, ET.ParseError) as exc:
        failures.append(f"正式工作簿读取失败: {exc}")
        workbook_statuses = {}

    expected_statuses = {
        display_code(value): ("保留" if name in RESERVED_CODES else "使用中")
        for name, value in cpu2_codes.items()
        if name not in NON_FAULT_CODES
    }
    for name in CPU3_LOCAL_CODES:
        if name in cpu3_codes:
            expected_statuses[display_code(cpu3_codes[name])] = "使用中"

    missing_workbook_codes = sorted(set(expected_statuses) - set(workbook_statuses))
    unexpected_workbook_codes = sorted(set(workbook_statuses) - set(expected_statuses))
    if missing_workbook_codes:
        failures.append(f"正式工作簿缺少故障码: {', '.join(missing_workbook_codes)}")
    if unexpected_workbook_codes:
        failures.append(f"正式工作簿出现未登记故障码: {', '.join(unexpected_workbook_codes)}")
    for code, expected_status in sorted(expected_statuses.items()):
        actual_status = workbook_statuses.get(code)
        if actual_status is not None and actual_status != expected_status:
            failures.append(
                f"正式工作簿 {code} 状态不一致: 实际={actual_status or '空'}, 期望={expected_status}"
            )

    if protocol_version is not None:
        failures.extend(check_current_protocol_documents(protocol_version))

    if failures:
        print("故障码目录契约检查失败：")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print(
        "故障码目录契约检查通过："
        f"CPU2 共享码 {len(cpu2_codes)} 个，CPU3 本机码 {len(cpu3_names - cpu2_names)} 个，"
        f"正式表使用中 {sum(status == '使用中' for status in expected_statuses.values())} 项，"
        f"保留 {sum(status == '保留' for status in expected_statuses.values())} 项"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

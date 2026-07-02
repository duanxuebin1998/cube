# -*- coding: utf-8 -*-
"""Audit the readable SI-7000 Chinese translation against the source PDF text."""

from __future__ import annotations

import json
import re
from collections import Counter
from pathlib import Path

import pdfplumber


ROOT = Path(__file__).resolve().parents[1]
SOURCE_PDF = ROOT.parent / "00_原始资料" / "SI7000官方资料包_含Modbus规范.pdf"
OUTPUT_DIR = ROOT / "tools" / "audit"
SOURCE_TEXT = OUTPUT_DIR / "source_pdf_pages.json"
REPORT = OUTPUT_DIR / "translation_audit_report.md"

CHINESE_DOCS = [
    ROOT / "00_资料包目录与整理规则.md",
    ROOT / "01_DCS_Modbus接口规范_中文整理.md",
    ROOT / "02_产品规格认证与包装_中文整理.md",
    ROOT / "03_机械布置与现场接线_中文整理.md",
    ROOT / "04_安装与调试手册_中文整理.md",
    ROOT / "05_操作手册_中文整理.md",
    ROOT / "06_维修手册与保修信息_中文整理.md",
]

EXPECTED_TOPICS = [
    ("Online Informational Catalog", "在线信息目录"),
    ("Cryogenic Temperature Sensors", "低温温度传感器"),
    ("LNG Tank Gauging", "LNG 储罐计量"),
    ("DCS Modbus Interface Specification", "DCS Modbus 接口规范"),
    ("RS485", "RS485"),
    ("Odd parity", "奇校验"),
    ("Function 1", "Function 1"),
    ("Function 2", "Function 2"),
    ("Function 3", "Function 3"),
    ("Function 4", "Function 4"),
    ("Function 5", "Function 5"),
    ("Function 6", "Function 6"),
    ("Manual", "Manual"),
    ("Calibrate", "校准"),
    ("Auto", "Auto"),
    ("Profile", "Profile"),
    ("Bottom Reference", "Bottom Reference"),
    ("Upper Level Sensor", "上液位传感器"),
    ("Lower Level Sensor", "下液位传感器"),
    ("Probe Un-calibrated", "Probe Un-calibrated"),
    ("Alarm Disable", "报警禁用"),
    ("Start Position", "起始位置"),
    ("Stop Position", "停止位置"),
    ("Interval Timer", "间隔定时"),
    ("Auto Profile", "自动 Profile"),
    ("Density Alarm", "密度报警"),
    ("Temperature Alarm", "温度报警"),
    ("Level", "液位"),
    ("Temperature", "温度"),
    ("Density", "密度"),
    ("Packaging Specifications", "包装规格"),
    ("General Arrangement", "总体布置"),
    ("Field Wiring Diagram", "现场接线图"),
    ("Installation & Commissioning Manual", "安装与调试手册"),
    ("Warnings, Cautions, and Notices", "警告、注意和通知"),
    ("Installation Requirements", "安装要求"),
    ("Pre-Installation Check Lists", "安装前检查清单"),
    ("Unpacking and Inspection", "开箱和检查"),
    ("Connecting System Wiring", "连接系统接线"),
    ("Initial Power Up", "初始上电"),
    ("Motor Driver Test", "电机驱动测试"),
    ("Bench Marking", "Bench Marking"),
    ("Operational Manual", "操作手册"),
    ("Theory of Operation", "工作原理"),
    ("System Specifications", "系统规格"),
    ("Home Screen", "Home Screen"),
    ("System Configuration Menu", "System Configuration Menu"),
    ("Command Menu", "Command Menu"),
    ("Service Manual", "维修手册"),
    ("Periodic Maintenance Procedure", "周期性维护流程"),
    ("Basic Functionality Testing", "基本功能测试"),
    ("Systems Warranty Policy", "系统保修政策"),
]

HIGH_RISK_TERMS = [
    "rollover",
    "stilling well",
    "pinch valve",
    "flameproof joints",
    "lock out / tag out",
    "lightning",
    "Manual drive mode",
    "Probe At Liquid Level",
    "Reference Pulse Count",
    "Zero Level Calibration",
    "Vapor Temperature",
    "Liquid Temperature",
    "Density Standard Deviation",
    "Sensor Isolation",
    "Explosion proof",
    "intrinsically safe",
    "proof test",
    "MESG",
]


def normalize_text(text: str) -> str:
    text = text.replace("\u00ad", "")
    text = re.sub(r"\s+", " ", text)
    return text.strip()


def extract_source_pages() -> list[dict[str, object]]:
    pages: list[dict[str, object]] = []
    with pdfplumber.open(str(SOURCE_PDF)) as pdf:
        for index, page in enumerate(pdf.pages, start=1):
            text = normalize_text(page.extract_text(x_tolerance=1, y_tolerance=3) or "")
            pages.append({"page": index, "chars": len(text), "text": text})
    return pages


def read_chinese_text() -> str:
    return "\n\n".join(path.read_text(encoding="utf-8") for path in CHINESE_DOCS)


def find_topic_coverage(source_text: str, chinese_text: str) -> list[tuple[str, str, bool, bool]]:
    source_lower = source_text.lower()
    chinese_lower = chinese_text.lower()
    rows = []
    for original, translated in EXPECTED_TOPICS:
        source_hit = original.lower() in source_lower
        chinese_hit = translated.lower() in chinese_lower
        rows.append((original, translated, source_hit, chinese_hit))
    return rows


def english_residue(chinese_text: str) -> Counter[str]:
    tokens = re.findall(r"\b[A-Za-z][A-Za-z0-9&/#.+-]{2,}\b", chinese_text)
    allow_prefixes = {
        "SI",
        "LTD",
        "DCS",
        "Modbus",
        "Function",
        "Profile",
        "Auto",
        "Manual",
        "Cal",
        "RS485",
        "RS232",
        "RTU",
        "IEC",
        "IECEx",
        "EN",
        "ISO",
        "JTAG",
        "PIM",
        "AC",
        "VAC",
        "VDC",
        "LNG",
        "TGIM",
        "HART",
        "Ethernet",
        "Home",
        "Screen",
        "System",
        "Menu",
        "Command",
        "Density",
        "Level",
        "Temperature",
        "Pinch",
        "Valve",
        "Zone",
        "WARNING",
        "NOTICE",
        "CAUTION",
        "Rev",
        "PDF",
        "USA",
        "www",
    }
    counter = Counter(token for token in tokens if token not in allow_prefixes)
    return counter


def lines_with_terms(chinese_text: str, terms: list[str]) -> list[str]:
    lines = chinese_text.splitlines()
    hits: list[str] = []
    for line in lines:
        lower = line.lower()
        if any(term.lower() in lower for term in terms):
            hits.append(line.strip())
    return hits[:80]


def page_text_summary(pages: list[dict[str, object]]) -> list[str]:
    summary = []
    for page in pages:
        text = str(page["text"])
        words = re.findall(r"[A-Za-z][A-Za-z0-9/&#+.-]+", text)
        common = ", ".join(word for word, _ in Counter(words).most_common(8))
        summary.append(f"| {page['page']} | {page['chars']} | {common} |")
    return summary


def write_report() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    pages = extract_source_pages()
    SOURCE_TEXT.write_text(json.dumps(pages, ensure_ascii=False, indent=2), encoding="utf-8")

    source_text = "\n".join(str(page["text"]) for page in pages)
    chinese_text = read_chinese_text()
    coverage = find_topic_coverage(source_text, chinese_text)
    residues = english_residue(chinese_text)
    risk_lines = lines_with_terms(chinese_text, HIGH_RISK_TERMS)
    low_text_pages = [page for page in pages if int(page["chars"]) < 80]

    report_lines = [
        "# SI-7000 中文翻译覆盖审计报告",
        "",
        f"- 原始 PDF 页数：{len(pages)}",
        f"- 原始 PDF 可抽取文本总字符数：{sum(int(page['chars']) for page in pages)}",
        f"- 中文章节稿字符数：{len(chinese_text)}",
        "",
        "## 可抽取文本较少的页面",
        "",
        "这些页面通常是图片、扫描页、图纸或封面，需要以图片和人工译注核对。",
        "",
        "| PDF 页码 | 可抽取字符数 | 文本片段 |",
        "| --- | ---: | --- |",
    ]
    for page in low_text_pages:
        snippet = str(page["text"])[:80].replace("|", "/")
        report_lines.append(f"| {page['page']} | {page['chars']} | {snippet} |")

    report_lines.extend(
        [
            "",
            "## 主题覆盖检查",
            "",
            "| 原文主题 | 中文检查词 | 原文出现 | 中文出现 |",
            "| --- | --- | --- | --- |",
        ]
    )
    for original, translated, source_hit, chinese_hit in coverage:
        report_lines.append(
            f"| {original} | {translated} | {'是' if source_hit else '否'} | {'是' if chinese_hit else '否'} |"
        )

    report_lines.extend(
        [
            "",
            "## 英文残留高频词",
            "",
            "以下仅用于人工判断，专业术语和界面英文可能需要保留。",
            "",
            "| 英文词 | 次数 |",
            "| --- | ---: |",
        ]
    )
    for token, count in residues.most_common(80):
        report_lines.append(f"| {token} | {count} |")

    report_lines.extend(
        [
            "",
            "## 高风险术语所在中文行",
            "",
        ]
    )
    for line in risk_lines:
        report_lines.append(f"- {line}")

    report_lines.extend(
        [
            "",
            "## 原 PDF 页级文本摘要",
            "",
            "| PDF 页码 | 可抽取字符数 | 高频英文词 |",
            "| --- | ---: | --- |",
            *page_text_summary(pages),
        ]
    )

    REPORT.write_text("\n".join(report_lines) + "\n", encoding="utf-8", newline="\n")
    print(f"wrote {SOURCE_TEXT}")
    print(f"wrote {REPORT}")


if __name__ == "__main__":
    write_report()

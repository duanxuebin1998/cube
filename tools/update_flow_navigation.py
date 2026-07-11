# -*- coding: utf-8 -*-
"""Update cross-document navigation for CPU2/CPU3 flow HTML docs.

The script is intentionally data-driven and idempotent. It plans the global
program-flow entrance, the cross-CPU business route page, and each relationship
navigator in memory. The default mode only checks; ``--write`` explicitly
applies a fully validated plan.
"""

from __future__ import annotations

import argparse
import codecs
import html
import json
import os
import re
import tempfile
import time
from dataclasses import dataclass
from datetime import date
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence, Tuple


ROOT = Path(__file__).resolve().parents[1]
DOC_NAV_DIR = ROOT / "docs" / "00_程序流程导航"
GLOBAL_INDEX = DOC_NAV_DIR / "index.html"
CROSS_ROUTE = DOC_NAV_DIR / "跨CPU业务链路.html"
ROUTE_MANIFEST = DOC_NAV_DIR / "业务链路清单.json"
EVIDENCE_MANIFEST = DOC_NAV_DIR / "流程证据清单.json"
CPU2_DIR = DOC_NAV_DIR / "CPU2"
CPU3_DIR = DOC_NAV_DIR / "CPU3"

REPLACE_RETRY_DELAYS_SECONDS = (0.05, 0.1, 0.2, 0.4, 0.8)
CLOSURE_ROLE_IDS = ("entry", "dispatch", "execution", "readback", "exception", "verification")

STYLE_MARK = '<style id="cross-flow-nav-style">'
EMBED_STYLESHEET_NAME = "网站嵌入增强.css"
EMBED_SCRIPT_NAME = "网站嵌入增强.js"
NAV_START = "<!-- CROSS-FLOW-NAV-START -->"
NAV_END = "<!-- CROSS-FLOW-NAV-END -->"
LEGACY_START = "<!-- FLOW-LEGACY-NOTICE-START -->"
LEGACY_END = "<!-- FLOW-LEGACY-NOTICE-END -->"
SVG_METADATA_RE = re.compile(
    r'(?P<opening><svg\b[^>]*>)'
    r'(?P<metadata>\s*<title\b[^>]*\bid\s*=\s*(?P<title_quote>["\'])'
    r'(?P<title_id>[^"\']+)(?P=title_quote)[^>]*>.*?</title>'
    r'\s*<desc\b[^>]*\bid\s*=\s*(?P<desc_quote>["\'])'
    r'(?P<desc_id>[^"\']+)(?P=desc_quote)[^>]*>.*?</desc>)',
    re.IGNORECASE | re.DOTALL,
)
SVG_OPENING_TAG_RE = re.compile(r"<svg\b[^>]*>", re.IGNORECASE | re.DOTALL)
ARIA_LABEL_ATTR_RE = re.compile(
    r"\s+aria-label\s*=\s*(?P<quote>[\"'])(?P<value>.*?)(?P=quote)",
    re.IGNORECASE | re.DOTALL,
)
ARIA_ACCESSIBILITY_ATTR_RE = re.compile(
    r"\s+aria-(?:hidden|label|labelledby|describedby)\s*=\s*"
    r"(?P<quote>[\"'])(?P<value>.*?)(?P=quote)",
    re.IGNORECASE | re.DOTALL,
)
SVG_ROOT_PRESENTATION_ATTR_RE = re.compile(
    r"\s+(?:role|focusable|tabindex)\s*=\s*(?P<quote>[\"'])(?P<value>.*?)(?P=quote)",
    re.IGNORECASE | re.DOTALL,
)
ARIA_REFERENCE_ATTR_RE = re.compile(
    r"\s+(?P<name>aria-(?:labelledby|describedby))\s*=\s*"
    r"(?P<quote>[\"'])(?P<value>.*?)(?P=quote)",
    re.IGNORECASE | re.DOTALL,
)
HEADING_RE = re.compile(
    r"<h(?P<level>[1-6])\b[^>]*>(?P<body>.*?)</h(?P=level)\s*>",
    re.IGNORECASE | re.DOTALL,
)
HEADING_NUMBER_PREFIX_RE = re.compile(
    r"^(?:(?:\d+(?:\.\d+)*\.?)|(?:[一二三四五六七八九十百]+、))\s*"
)
SOURCE_CARD_H4_RE = re.compile(
    r"(?P<prefix><(?:article|div)\b[^>]*\bclass\s*=\s*[\"'][^\"']*"
    r"\bsource-card\b[^\"']*[\"'][^>]*>\s*)"
    r"<h4(?P<attrs>\b[^>]*)>(?P<body>.*?)</h4\s*>",
    re.IGNORECASE | re.DOTALL,
)
TABLE_RE = re.compile(
    r"(?P<opening><table\b[^>]*>)(?P<body>.*?)(?P<closing></table\s*>)",
    re.IGNORECASE | re.DOTALL,
)
TABLE_OPENING_RE = re.compile(r"<table\b", re.IGNORECASE)
CAPTION_RE = re.compile(
    r"<caption\b(?P<attrs>[^>]*)>(?P<body>.*?)</caption\s*>",
    re.IGNORECASE | re.DOTALL,
)
THEAD_RE = re.compile(
    r"(?P<opening><thead\b[^>]*>)(?P<body>.*?)(?P<closing></thead\s*>)",
    re.IGNORECASE | re.DOTALL,
)
TH_OPENING_RE = re.compile(r"<th\b(?P<attrs>[^>]*)>", re.IGNORECASE | re.DOTALL)
SCOPE_ATTR_RE = re.compile(r"\bscope\s*=", re.IGNORECASE)
COLSPAN_ATTR_RE = re.compile(
    r"\bcolspan\s*=\s*(?:[\"']\s*)?(?P<value>\d+)",
    re.IGNORECASE,
)
HTML_TAG_RE = re.compile(r"<[^>]+>", re.DOTALL)


class ConcurrentSourceChangeError(RuntimeError):
    """Raised when a planned target changed after it was read."""


@dataclass(frozen=True)
class FileSnapshot:
    path: Path
    original_bytes: bytes | None
    newline: str
    trailing_newline_count: int
    has_utf8_bom: bool


def _decode_utf8(raw: bytes, path: Path) -> tuple[str, bool]:
    has_bom = raw.startswith(codecs.BOM_UTF8)
    payload = raw[len(codecs.BOM_UTF8) :] if has_bom else raw
    try:
        return payload.decode("utf-8"), has_bom
    except UnicodeDecodeError as exc:
        raise RuntimeError(f"File is not valid UTF-8: {path}") from exc


def _normalize_newlines(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\r", "\n")


def _detect_newline(text: str) -> str:
    crlf_count = text.count("\r\n")
    without_crlf = text.replace("\r\n", "")
    lf_count = without_crlf.count("\n")
    cr_count = without_crlf.count("\r")
    counts = {"\r\n": crlf_count, "\n": lf_count, "\r": cr_count}
    if not any(counts.values()):
        return "\n"
    return max(counts, key=counts.__getitem__)


def _trailing_newline_count(text: str) -> int:
    normalized = _normalize_newlines(text)
    return len(normalized) - len(normalized.rstrip("\n"))


class InMemoryUpdatePlan:
    """Collect every generated result before validating or touching the disk."""

    def __init__(self) -> None:
        self._snapshots: dict[Path, FileSnapshot] = {}
        self._planned_bytes: dict[Path, bytes] = {}

    def _snapshot(self, path: Path) -> FileSnapshot:
        path = Path(path)
        if path not in self._snapshots:
            raw = path.read_bytes() if path.exists() else None
            if raw is None:
                snapshot = FileSnapshot(path, None, "\n", 1, False)
            else:
                text, has_bom = _decode_utf8(raw, path)
                snapshot = FileSnapshot(
                    path=path,
                    original_bytes=raw,
                    newline=_detect_newline(text),
                    trailing_newline_count=_trailing_newline_count(text),
                    has_utf8_bom=has_bom,
                )
            self._snapshots[path] = snapshot
        return self._snapshots[path]

    def exists(self, path: Path) -> bool:
        path = Path(path)
        snapshot = self._snapshot(path)
        return path in self._planned_bytes or snapshot.original_bytes is not None

    def read_text(self, path: Path) -> str:
        path = Path(path)
        snapshot = self._snapshot(path)
        raw = self._planned_bytes.get(path, snapshot.original_bytes)
        if raw is None:
            raise FileNotFoundError(path)
        text, _ = _decode_utf8(raw, path)
        return _normalize_newlines(text)

    def stage_text(self, path: Path, text: str) -> None:
        path = Path(path)
        snapshot = self._snapshot(path)
        normalized = _normalize_newlines(text)
        if snapshot.original_bytes is not None:
            original_text, _ = _decode_utf8(snapshot.original_bytes, path)
            if normalized == _normalize_newlines(original_text):
                self._planned_bytes.pop(path, None)
                return
            normalized = normalized.rstrip("\n")
            normalized += "\n" * snapshot.trailing_newline_count
        rendered = normalized.replace("\n", snapshot.newline)
        payload = rendered.encode("utf-8")
        if snapshot.has_utf8_bom:
            payload = codecs.BOM_UTF8 + payload
        if payload == snapshot.original_bytes:
            self._planned_bytes.pop(path, None)
        else:
            self._planned_bytes[path] = payload

    @property
    def changed_paths(self) -> tuple[Path, ...]:
        return tuple(sorted(self._planned_bytes, key=lambda item: str(item).casefold()))

    def _assert_sources_unchanged(self) -> None:
        changed_sources = []
        for path in self.changed_paths:
            current = path.read_bytes() if path.exists() else None
            if current != self._snapshots[path].original_bytes:
                changed_sources.append(path)
        if changed_sources:
            joined = ", ".join(str(path) for path in changed_sources)
            raise ConcurrentSourceChangeError(
                "Source files changed while the update was being planned: " + joined
            )

    def apply(self) -> tuple[Path, ...]:
        changed_paths = self.changed_paths
        if not changed_paths:
            return ()

        self._assert_sources_unchanged()
        temporary_paths: dict[Path, Path] = {}
        try:
            for path in changed_paths:
                path.parent.mkdir(parents=True, exist_ok=True)
                descriptor, temporary_name = tempfile.mkstemp(
                    prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
                )
                temporary_path = Path(temporary_name)
                temporary_paths[path] = temporary_path
                with os.fdopen(descriptor, "wb") as stream:
                    stream.write(self._planned_bytes[path])
                    stream.flush()
                    os.fsync(stream.fileno())

            # Detect every stale source before replacing the first target. This
            # prevents a known concurrent edit from producing a partial update.
            self._assert_sources_unchanged()
            for path in changed_paths:
                temporary_path = temporary_paths[path]
                self._replace_with_retry(temporary_path, path)
                del temporary_paths[path]
        finally:
            for temporary_path in temporary_paths.values():
                try:
                    temporary_path.unlink()
                except FileNotFoundError:
                    pass
        return changed_paths

    @staticmethod
    def _replace_with_retry(temporary_path: Path, path: Path) -> None:
        for delay_seconds in REPLACE_RETRY_DELAYS_SECONDS:
            try:
                os.replace(temporary_path, path)
                return
            except PermissionError:
                time.sleep(delay_seconds)
        os.replace(temporary_path, path)


PAGE_DEFS: Dict[str, Dict[str, str]] = {
    "global": {"title": "程序流程统一入口", "path": str(GLOBAL_INDEX), "cpu": "全局"},
    "cross": {"title": "跨 CPU 业务链路", "path": str(CROSS_ROUTE), "cpu": "全局"},
    "cpu2_total": {
        "title": "CPU2 程序流程总览",
        "path": str(CPU2_DIR / "CPU2程序流程总览.html"),
        "cpu": "CPU2",
    },
    "cpu2_issue": {
        "title": "CPU2 问题点总表",
        "path": str(CPU2_DIR / "问题点总表.html"),
        "cpu": "CPU2",
    },
    "cpu2_01": {
        "title": "启动主循环与中断",
        "path": str(CPU2_DIR / "01_启动主循环与中断.html"),
        "cpu": "CPU2",
    },
    "cpu2_02": {
        "title": "测量命令总入口",
        "path": str(CPU2_DIR / "02_测量命令总入口.html"),
        "cpu": "CPU2",
    },
    "cpu2_03": {
        "title": "回零与罐底罐高",
        "path": str(CPU2_DIR / "03_回零与罐底罐高.html"),
        "cpu": "CPU2",
    },
    "cpu2_04": {
        "title": "液位测量与跟随",
        "path": str(CPU2_DIR / "04_液位测量与跟随.html"),
        "cpu": "CPU2",
    },
    "cpu2_05": {
        "title": "水位测量与跟随",
        "path": str(CPU2_DIR / "05_水位测量与跟随.html"),
        "cpu": "CPU2",
    },
    "cpu2_06": {
        "title": "密度与单点测量",
        "path": str(CPU2_DIR / "06_密度与单点测量.html"),
        "cpu": "CPU2",
    },
    "cpu2_07": {
        "title": "故障管理与恢复",
        "path": str(CPU2_DIR / "07_故障管理与恢复.html"),
        "cpu": "CPU2",
    },
    "cpu2_08": {
        "title": "电机与位置模型",
        "path": str(CPU2_DIR / "08_电机与位置模型.html"),
        "cpu": "CPU2",
    },
    "cpu2_09": {
        "title": "传感器与无线通信",
        "path": str(CPU2_DIR / "09_传感器与无线通信.html"),
        "cpu": "CPU2",
    },
    "cpu2_10": {
        "title": "Modbus 与 CPU3 通信",
        "path": str(CPU2_DIR / "10_Modbus与CPU3通信.html"),
        "cpu": "CPU2",
    },
    "cpu2_11": {
        "title": "HART 接口",
        "path": str(CPU2_DIR / "11_HART接口.html"),
        "cpu": "CPU2",
    },
    "cpu2_12": {
        "title": "参数存储与系统配置",
        "path": str(CPU2_DIR / "12_参数存储与系统配置.html"),
        "cpu": "CPU2",
    },
    "cpu2_13": {
        "title": "扭力与继电器输出",
        "path": str(CPU2_DIR / "13_扭力与继电器输出.html"),
        "cpu": "CPU2",
    },
    "cpu2_14": {
        "title": "BSP 外设驱动",
        "path": str(CPU2_DIR / "14_BSP外设驱动.html"),
        "cpu": "CPU2",
    },
    "cpu2_15": {
        "title": "公共工具与算法",
        "path": str(CPU2_DIR / "15_公共工具与算法.html"),
        "cpu": "CPU2",
    },
    "cpu2_16": {
        "title": "其它项目代码",
        "path": str(CPU2_DIR / "16_其它项目代码.html"),
        "cpu": "CPU2",
    },
    "cpu3_total": {
        "title": "CPU3 程序流程总览",
        "path": str(CPU3_DIR / "CPU3程序流程总览.html"),
        "cpu": "CPU3",
    },
    "cpu3_01": {
        "title": "启动主循环与调度",
        "path": str(CPU3_DIR / "01_启动主循环与调度.html"),
        "cpu": "CPU3",
    },
    "cpu3_02": {
        "title": "CPU2 内部通信与轮询",
        "path": str(CPU3_DIR / "02_CPU2内部通信与轮询.html"),
        "cpu": "CPU3",
    },
    "cpu3_03": {
        "title": "外部 COM 协议分发与 RS485 发送",
        "path": str(CPU3_DIR / "03_外部COM协议分发与RS485发送.html"),
        "cpu": "CPU3",
    },
    "cpu3_04": {
        "title": "DSM 协议与命令映射",
        "path": str(CPU3_DIR / "04_DSM协议与命令映射.html"),
        "cpu": "CPU3",
    },
    "cpu3_05": {
        "title": "Wartsila 与 SI协议适配",
        "path": str(CPU3_DIR / "05_Wartsila与SI协议适配.html"),
        "cpu": "CPU3",
    },
    "cpu3_06": {
        "title": "显示刷新与按键事件",
        "path": str(CPU3_DIR / "06_显示刷新与按键事件.html"),
        "cpu": "CPU3",
    },
    "cpu3_07": {
        "title": "菜单参数与指令下发",
        "path": str(CPU3_DIR / "07_菜单参数与指令下发.html"),
        "cpu": "CPU3",
    },
    "cpu3_08": {
        "title": "本机参数、FRAM、时钟与外设恢复",
        "path": str(CPU3_DIR / "08_本机参数FRAM时钟与外设恢复.html"),
        "cpu": "CPU3",
    },
}


CPU2_ORDER = [
    "cpu2_01",
    "cpu2_02",
    "cpu2_03",
    "cpu2_04",
    "cpu2_05",
    "cpu2_06",
    "cpu2_07",
    "cpu2_08",
    "cpu2_09",
    "cpu2_10",
    "cpu2_11",
    "cpu2_12",
    "cpu2_13",
    "cpu2_14",
    "cpu2_15",
    "cpu2_16",
]

CPU3_ORDER = [
    "cpu3_01",
    "cpu3_02",
    "cpu3_03",
    "cpu3_04",
    "cpu3_05",
    "cpu3_06",
    "cpu3_07",
    "cpu3_08",
]

LINEAR_ORDERS = {
    "CPU2": ["cpu2_total", *CPU2_ORDER, "cpu2_issue"],
    "CPU3": ["cpu3_total", *CPU3_ORDER],
}

LEGACY_DOCS: Sequence[Dict[str, str]] = [
    {
        "title": "串口 B 指令程序流程梳理",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "串口B指令详细执行过程.html"),
        "status": "专题历史页",
        "current": "cpu2_08",
        "note": "串口 B 入口和运动执行细节的旧专题梳理；当前权威流程以 CPU2 电机与位置模型页为准。",
    },
    {
        "title": "CPU2 电机程序、函数与运动流程综合梳理",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "电机程序与函数梳理.html"),
        "status": "专题历史页",
        "current": "cpu2_08",
        "note": "电机函数层次、运动入口和保护逻辑的历史综合页；当前权威流程以 CPU2 电机与位置模型页为准。",
    },
    {
        "title": "电机运动函数层次梳理已合并",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "电机运动函数层次梳理.html"),
        "status": "已合并跳转页",
        "current": "cpu2_08",
        "note": "该页已指向电机综合梳理，当前流程体系统一指向 CPU2 电机与位置模型。",
    },
    {
        "title": "电机运动函数第一轮改动点梳理",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "电机运动函数本次改动点梳理.html"),
        "status": "改动记录页",
        "current": "cpu2_08",
        "note": "电机运动第一轮改动记录，保留改动背景；当前流程以 CPU2 电机与位置模型页为准。",
    },
    {
        "title": "电机运动程序详细流程图已合并",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "电机运动程序详细流程图.html"),
        "status": "已合并跳转页",
        "current": "cpu2_08",
        "note": "该页为旧流程图跳转页，当前权威流程以 CPU2 电机与位置模型页为准。",
    },
    {
        "title": "CPU2 HART 接口程序梳理",
        "path": str(ROOT / "docs" / "01_协议与寄存器" / "CPU2通信与解耦" / "CPU2_HART接口程序梳理.html"),
        "status": "专题历史页",
        "current": "cpu2_11",
        "note": "HART 接口旧专题梳理；当前权威流程以 CPU2 HART 接口程序流程页为准。",
    },
    {
        "title": "CPU2 HART 旧栈适配与 E+H 兼容方案",
        "path": str(ROOT / "docs" / "01_协议与寄存器" / "CPU2通信与解耦" / "CPU2_HART旧栈适配与E+H兼容方案.html"),
        "status": "方案参考页",
        "current": "cpu2_11",
        "note": "HART 旧栈适配和兼容性方案，保留方案背景；当前流程以 CPU2 HART 接口程序流程页为准。",
    },
    {
        "title": "CPU2 找液位详细流程梳理",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "2026-06-11_CPU2找液位详细流程梳理.html"),
        "status": "问题分析历史页",
        "current": "cpu2_04",
        "note": "找液位问题分析和旧流程梳理；当前权威流程以 CPU2 液位测量与跟随页为准。",
    },
    {
        "title": "瓦锡兰分布测量详细流程梳理",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "未处理" / "瓦锡兰分布测量详细流程梳理.html"),
        "status": "问题分析历史页",
        "current": "cpu2_06",
        "note": "瓦锡兰分布测量专题梳理；当前 CPU2 执行流程看密度与单点测量，CPU3 协议入口看 Wartsila 与 SI协议适配。",
    },
    {
        "title": "瓦锡兰分布测量改前流程与已实现优化方案对比",
        "path": str(ROOT / "docs" / "02_需求与计划" / "已实现" / "2026-06-11_瓦锡兰分布测量当前流程与优化方案对比.html"),
        "status": "需求实现记录",
        "current": "cpu2_06",
        "note": "瓦锡兰优化前后对比记录，保留实现依据；当前权威流程以 CPU2 密度与单点测量页为准。",
    },
    {
        "title": "瓦锡兰分布测量点间移动与慢速下行识别液位优化需求方案",
        "path": str(ROOT / "docs" / "02_需求与计划" / "已实现" / "2026-06-11_瓦锡兰分布测量点间移动与慢速下行识别液位优化需求方案.html"),
        "status": "需求实现记录",
        "current": "cpu2_06",
        "note": "瓦锡兰点间移动和空气点液位识别的需求方案，当前权威流程以 CPU2 密度与单点测量页为准。",
    },
    {
        "title": "读取部件参数指令优化记录",
        "path": str(ROOT / "docs" / "02_需求与计划" / "已实现" / "2026-06-10_读取部件参数指令优化记录.html"),
        "status": "需求实现记录",
        "current": "cpu2_09",
        "note": "读取部件参数、RSSI 与 AO 运行态链路的历史优化记录；当前执行流程以 CPU2 传感器与无线通信页为准，跨 CPU 入口看 readparams 业务链路。",
        "route": "readparams",
    },
    {
        "title": "四路继电器报警输出逻辑对照与问题分析",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "2026-06-06_四路继电器报警输出逻辑对照与问题分析.html"),
        "status": "问题分析历史页",
        "current": "cpu2_13",
        "note": "四路继电器报警输出逻辑的历史问题分析；当前权威流程以 CPU2 扭力与继电器输出页为准。",
    },
    {
        "title": "CPU2 电流输出问题与 v1.563 处理方式对比",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "2026-06-13_CPU2电流输出问题与CPU2_v1.563处理方式对比.html"),
        "status": "问题分析历史页",
        "current": "cpu2_13",
        "note": "CPU2 电流输出问题和 v1.563 处理方式的历史对比；当前 AO 输出与运行态流程以 CPU2 扭力与继电器输出页为准。",
        "route": "ao",
    },
    {
        "title": "AD5421 控制寄存器回读 FFFF 问题分析与现场验证",
        "path": str(ROOT / "docs" / "03_问题分析与整改" / "未处理" / "2026-06-16_AD5421控制寄存器回读FFFF问题分析与现场验证.html"),
        "status": "问题分析历史页",
        "current": "cpu2_13",
        "note": "AD5421 控制寄存器回读 FFFF 的历史现场验证；当前 AO/AD5421 输出流程以 CPU2 扭力与继电器输出页为准。",
        "route": "ao",
    },
]


def load_route_manifest(path: Path = ROUTE_MANIFEST) -> Dict[str, object]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"Route manifest does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Route manifest is not valid JSON: {path}: {exc}") from exc

    if not isinstance(data, dict) or data.get("schemaVersion") != 2:
        raise RuntimeError(f"Route manifest schemaVersion must be 2: {path}")
    closure_roles = data.get("closureRoles")
    if not isinstance(closure_roles, list) or len(closure_roles) != len(CLOSURE_ROLE_IDS):
        raise RuntimeError(f"Route manifest must define six closureRoles: {path}")
    closure_role_ids = []
    for index, role in enumerate(closure_roles, start=1):
        if not isinstance(role, dict):
            raise RuntimeError(f"Closure role {index} must be an object: {path}")
        role_id = role.get("id")
        label = role.get("label")
        if not isinstance(role_id, str) or not isinstance(label, str) or not label.strip():
            raise RuntimeError(f"Closure role {index} is invalid: {path}")
        closure_role_ids.append(role_id)
    if tuple(closure_role_ids) != CLOSURE_ROLE_IDS:
        raise RuntimeError(
            "Route manifest closureRoles must use the required order: "
            + ", ".join(CLOSURE_ROLE_IDS)
        )
    routes = data.get("routes")
    if not isinstance(routes, list) or not routes:
        raise RuntimeError(f"Route manifest must contain a non-empty routes array: {path}")

    allowed_owners = {"external", "cpu3", "cpu2", "evidence"}
    route_ids: set[str] = set()
    for route_index, route in enumerate(routes, start=1):
        if not isinstance(route, dict):
            raise RuntimeError(f"Route {route_index} must be an object: {path}")
        route_id = route.get("id")
        if not isinstance(route_id, str) or re.fullmatch(r"[a-z][a-z0-9-]*", route_id) is None:
            raise RuntimeError(f"Route {route_index} has an invalid id: {route_id!r}")
        if route_id in route_ids:
            raise RuntimeError(f"Route manifest has duplicate id: {route_id}")
        route_ids.add(route_id)
        for field in ("title", "tag", "summary"):
            if not isinstance(route.get(field), str) or not route[field].strip():
                raise RuntimeError(f"Route {route_id} has an empty {field}")

        steps = route.get("steps")
        if not isinstance(steps, list) or not steps:
            raise RuntimeError(f"Route {route_id} must contain steps")
        map_steps = 0
        formal_steps = 0
        formal_page_keys: set[str] = set()
        for step_index, step in enumerate(steps, start=1):
            if not isinstance(step, dict):
                raise RuntimeError(f"Route {route_id} step {step_index} must be an object")
            owner = step.get("owner")
            if owner not in allowed_owners:
                raise RuntimeError(f"Route {route_id} step {step_index} has invalid owner: {owner!r}")
            for field in ("role", "label", "detail"):
                if not isinstance(step.get(field), str) or not step[field].strip():
                    raise RuntimeError(f"Route {route_id} step {step_index} has an empty {field}")

            formal = step.get("formal") is True
            map_visible = step.get("map") is True
            if not formal and not map_visible:
                raise RuntimeError(f"Route {route_id} step {step_index} is not visible in any output")
            if formal:
                formal_steps += 1
                page_key = step.get("pageKey")
                if page_key not in PAGE_DEFS:
                    raise RuntimeError(
                        f"Route {route_id} step {step_index} references unknown pageKey: {page_key!r}"
                    )
                page = Path(PAGE_DEFS[page_key]["path"])
                formal_page_keys.add(page_key)
                if not page.is_file():
                    raise RuntimeError(
                        f"Route {route_id} step {step_index} references missing page: {page}"
                    )
            if map_visible:
                map_steps += 1
                site_href = step.get("siteHref")
                if not isinstance(site_href, str) or not site_href.startswith("/"):
                    raise RuntimeError(
                        f"Route {route_id} step {step_index} has invalid siteHref: {site_href!r}"
                    )

        if formal_steps < 2:
            raise RuntimeError(f"Route {route_id} must contain at least two formal steps")
        if map_steps != 4:
            raise RuntimeError(f"Route {route_id} must expose exactly four map steps, found {map_steps}")
        closure = route.get("closure")
        if not isinstance(closure, dict) or set(closure) != set(CLOSURE_ROLE_IDS):
            raise RuntimeError(f"Route {route_id} closure must define all six required roles")
        for role_id in CLOSURE_ROLE_IDS:
            page_keys = closure.get(role_id)
            if not isinstance(page_keys, list) or not page_keys:
                raise RuntimeError(f"Route {route_id} closure role {role_id} must be a non-empty array")
            if not all(isinstance(page_key, str) for page_key in page_keys):
                raise RuntimeError(f"Route {route_id} closure role {role_id} must contain page keys")
            if len(page_keys) != len(set(page_keys)):
                raise RuntimeError(f"Route {route_id} closure role {role_id} contains duplicate page keys")
            unknown_page_keys = [
                page_key
                for page_key in page_keys
                if page_key not in formal_page_keys
            ]
            if unknown_page_keys:
                raise RuntimeError(
                    f"Route {route_id} closure role {role_id} references pages outside formal steps: "
                    + ", ".join(repr(item) for item in unknown_page_keys)
                )
        if not any(str(page_key).startswith("cpu2_") for page_key in closure["execution"]):
            raise RuntimeError(f"Route {route_id} closure execution must include a CPU2 page")

    return data


ROUTE_MANIFEST_DATA = load_route_manifest()
ROUTES: List[Dict[str, object]] = ROUTE_MANIFEST_DATA["routes"]  # type: ignore[assignment]


def _repo_file(relative_path: str, *, field: str) -> Path:
    candidate = Path(relative_path)
    if candidate.is_absolute() or ".." in candidate.parts:
        raise RuntimeError(f"Evidence {field} must be a repository-relative path: {relative_path!r}")
    resolved = (ROOT / candidate).resolve()
    try:
        resolved.relative_to(ROOT.resolve())
    except ValueError as exc:
        raise RuntimeError(f"Evidence {field} escapes the repository: {relative_path!r}") from exc
    if not resolved.is_file():
        raise RuntimeError(f"Evidence {field} does not exist: {relative_path}")
    return resolved


def _firmware_version_from_header(path: Path, macro_prefix: str) -> str:
    raw = path.read_bytes()
    parts: list[int] = []
    for suffix in ("MAJOR", "MINOR", "PATCH", "BUILD"):
        pattern = rb"#define\s+" + re.escape(f"{macro_prefix}_{suffix}".encode("ascii")) + rb"\s+(\d+)u?\b"
        match = re.search(pattern, raw)
        if match is None:
            raise RuntimeError(f"Cannot read {macro_prefix}_{suffix} from {path}")
        parts.append(int(match.group(1)))
    return "V" + ".".join(str(part) for part in parts)


def load_evidence_manifest(
    path: Path = EVIDENCE_MANIFEST,
    routes: Sequence[Mapping[str, object]] = ROUTES,
) -> Dict[str, object]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"Evidence manifest does not exist: {path}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"Evidence manifest is not valid JSON: {path}: {exc}") from exc

    if not isinstance(data, dict) or data.get("schemaVersion") != 1:
        raise RuntimeError(f"Evidence manifest schemaVersion must be 1: {path}")

    firmware = data.get("firmwareBaseline")
    if not isinstance(firmware, dict) or set(firmware) != {"cpu2", "cpu3"}:
        raise RuntimeError(f"Evidence manifest firmwareBaseline must define cpu2 and cpu3: {path}")
    firmware_specs = {
        "cpu2": ("CPU2_APP_VERSION", "LTD_MAIN_CPU2/"),
        "cpu3": ("CPU3_APP_VERSION", "LTD_DISPLAY_CPU3/"),
    }
    for cpu, (macro_prefix, expected_root) in firmware_specs.items():
        item = firmware[cpu]
        if not isinstance(item, dict):
            raise RuntimeError(f"Evidence firmwareBaseline.{cpu} must be an object")
        version = item.get("version")
        source_path = item.get("sourcePath")
        if not isinstance(version, str) or re.fullmatch(r"V\d+\.\d+\.\d+\.\d+", version) is None:
            raise RuntimeError(f"Evidence firmwareBaseline.{cpu}.version is invalid: {version!r}")
        if not isinstance(source_path, str) or not source_path.startswith(expected_root):
            raise RuntimeError(f"Evidence firmwareBaseline.{cpu}.sourcePath is invalid: {source_path!r}")
        current_version = _firmware_version_from_header(
            _repo_file(source_path, field=f"firmwareBaseline.{cpu}.sourcePath"),
            macro_prefix,
        )
        if current_version != version:
            raise RuntimeError(
                f"Evidence firmware baseline is stale for {cpu}: manifest {version}, source {current_version}"
            )

    default_validations = data.get("defaultValidationPaths")
    if not isinstance(default_validations, list) or not default_validations:
        raise RuntimeError("Evidence manifest defaultValidationPaths must be a non-empty array")
    _validate_evidence_paths(
        default_validations,
        field="defaultValidationPaths",
        expected_root="docs/",
        allowed_suffixes={".md", ".html"},
    )

    pages = data.get("pages")
    if not isinstance(pages, list) or not pages:
        raise RuntimeError("Evidence manifest pages must be a non-empty array")
    core_page_keys = {
        str(step["pageKey"])
        for route in routes
        for step in route["steps"]  # type: ignore[index]
        if isinstance(step, dict) and step.get("formal") is True
    }
    page_keys: set[str] = set()
    for index, page in enumerate(pages, start=1):
        if not isinstance(page, dict):
            raise RuntimeError(f"Evidence page {index} must be an object")
        page_key = page.get("pageKey")
        if not isinstance(page_key, str) or page_key not in PAGE_DEFS:
            raise RuntimeError(f"Evidence page {index} has unknown pageKey: {page_key!r}")
        if page_key in page_keys:
            raise RuntimeError(f"Evidence manifest has duplicate pageKey: {page_key}")
        page_keys.add(page_key)
        if page.get("status") not in {"verified", "needs-review", "draft"}:
            raise RuntimeError(f"Evidence page {page_key} has invalid status: {page.get('status')!r}")
        reviewed_at = page.get("reviewedAt")
        if not isinstance(reviewed_at, str) or re.fullmatch(r"\d{4}-\d{2}-\d{2}", reviewed_at) is None:
            raise RuntimeError(f"Evidence page {page_key} has invalid reviewedAt: {reviewed_at!r}")

        cpu = PAGE_DEFS[page_key]["cpu"]
        expected_root = "LTD_MAIN_CPU2/" if cpu == "CPU2" else "LTD_DISPLAY_CPU3/"
        source_paths = page.get("sourcePaths")
        if not isinstance(source_paths, list) or not source_paths:
            raise RuntimeError(f"Evidence page {page_key} must contain sourcePaths")
        _validate_evidence_paths(
            source_paths,
            field=f"pages.{page_key}.sourcePaths",
            expected_root=expected_root,
            allowed_suffixes={".c", ".h"},
        )
        validation_paths = page.get("validationPaths")
        if not isinstance(validation_paths, list):
            raise RuntimeError(f"Evidence page {page_key} validationPaths must be an array")
        _validate_evidence_paths(
            validation_paths,
            field=f"pages.{page_key}.validationPaths",
            expected_root="docs/",
            allowed_suffixes={".md", ".html"},
        )

    missing = sorted(core_page_keys - page_keys)
    extra = sorted(page_keys - core_page_keys)
    if missing or extra:
        raise RuntimeError(
            "Evidence page coverage must exactly match formal route pages; "
            f"missing={missing or 'none'}, extra={extra or 'none'}"
        )
    pages_by_key = {str(page["pageKey"]): page for page in pages}
    for route in routes:
        route_id = str(route["id"])
        closure = route.get("closure")
        if not isinstance(closure, dict):
            raise RuntimeError(f"Route {route_id} is missing closure metadata")
        for page_key in closure.get("verification", []):
            page = pages_by_key.get(str(page_key))
            if page is None or page.get("status") != "verified":
                raise RuntimeError(
                    f"Route {route_id} verification page is not verified: {page_key!r}"
                )
    return data


def _validate_evidence_paths(
    paths: Sequence[object],
    *,
    field: str,
    expected_root: str,
    allowed_suffixes: set[str],
) -> None:
    seen: set[str] = set()
    for relative_path in paths:
        if not isinstance(relative_path, str) or not relative_path.startswith(expected_root):
            raise RuntimeError(f"Evidence {field} has invalid path: {relative_path!r}")
        if relative_path in seen:
            raise RuntimeError(f"Evidence {field} has duplicate path: {relative_path}")
        seen.add(relative_path)
        if Path(relative_path).suffix.lower() not in allowed_suffixes:
            raise RuntimeError(f"Evidence {field} has unsupported file type: {relative_path}")
        _repo_file(relative_path, field=field)


EVIDENCE_MANIFEST_DATA = load_evidence_manifest()
EVIDENCE_PAGES_BY_KEY: Dict[str, Dict[str, object]] = {
    str(item["pageKey"]): item
    for item in EVIDENCE_MANIFEST_DATA["pages"]  # type: ignore[index]
}


RELATIONS: Dict[str, Dict[str, object]] = {
    "cpu2_total": {
        "focus": "CPU2 所有流程页面的本地总览，适合从 CPU2 角度查启动、测量、通信、故障、外设和问题点。",
        "upstream": ["global", "cross"],
        "downstream": ["cpu2_01", "cpu2_02", "cpu2_04", "cpu2_06", "cpu2_10", "cpu2_issue"],
        "route": ["global", "cross", "cpu3_total"],
    },
    "cpu2_issue": {
        "focus": "集中查看 CPU2 各流程页整理出来的问题点、风险等级和整改建议。",
        "upstream": ["cpu2_total", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_07"],
        "downstream": ["cpu2_02", "cpu2_08", "cpu2_10", "cpu2_12"],
        "route": ["global", "cross", "cpu2_total"],
    },
    "cpu2_01": {
        "focus": "CPU2 上电后初始化外设、进入主循环，并为命令处理、故障恢复和通信刷新提供调度基础。",
        "upstream": ["global", "cpu2_total"],
        "downstream": ["cpu2_02", "cpu2_07", "cpu2_10", "cpu2_14"],
        "route": ["global", "cpu2_total", "cpu2_01", "cpu2_02"],
    },
    "cpu2_02": {
        "focus": "CPU2 正式测量命令总入口，负责公共准备、命令分发和各测量过程的状态切换。",
        "upstream": ["cpu3_07", "cpu3_04", "cpu3_05", "cpu3_02"],
        "downstream": ["cpu2_03", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_13", "cpu2_11"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_02", "cpu2_04"],
    },
    "cpu2_03": {
        "focus": "回零、罐底和罐高建立机械位置基准，是液位、水位、密度测量前后共同依赖的位置基础。",
        "upstream": ["cpu2_02", "cpu2_08"],
        "downstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_10"],
        "route": ["cpu2_total", "cpu2_02", "cpu2_03", "cpu2_04"],
    },
    "cpu2_04": {
        "focus": "液位找液、阈值确定、精找、跟随和方式二/连续方式闭环，是液位业务的 CPU2 核心页。",
        "upstream": ["cpu3_07", "cpu3_04", "cpu3_05", "cpu3_02", "cpu2_02", "cpu2_03"],
        "downstream": ["cpu2_07", "cpu2_10", "cpu3_02", "cpu3_06", "cpu3_04", "cpu2_issue"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_02", "cpu2_04", "cpu2_10", "cpu3_06"],
    },
    "cpu2_05": {
        "focus": "水位找水、跟随、盲区和状态输出，和液位流程同属测量主链路但判断阈值不同。",
        "upstream": ["cpu3_07", "cpu3_02", "cpu2_02", "cpu2_03"],
        "downstream": ["cpu2_07", "cpu2_10", "cpu3_02", "cpu3_06", "cpu2_issue"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_02", "cpu2_05", "cpu2_10"],
    },
    "cpu2_06": {
        "focus": "单点密度、区间密度、分布密度和瓦锡兰场景测量，决定 profile 结果和点表输出。",
        "upstream": ["cpu3_07", "cpu3_05", "cpu3_02", "cpu2_02"],
        "downstream": ["cpu2_10", "cpu3_02", "cpu3_05", "cpu2_issue"],
        "route": ["cross", "cpu3_05", "cpu3_02", "cpu2_02", "cpu2_06", "cpu3_05"],
    },
    "cpu2_07": {
        "focus": "CPU2 统一故障状态、错误码、恢复尝试和异常出口，是测量、电机、传感和通信的安全汇聚点。",
        "upstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_08", "cpu2_09", "cpu2_13"],
        "downstream": ["cpu2_10", "cpu3_02", "cpu3_06", "cpu2_issue"],
        "route": ["cross", "cpu2_08", "cpu2_07", "cpu2_10", "cpu3_06"],
    },
    "cpu2_08": {
        "focus": "电机方向、速度、位置源、记步和丢步/碰撞判断，为所有测量闭环提供运动基础。",
        "upstream": ["cpu2_01", "cpu2_02", "cpu2_03", "cpu2_04", "cpu2_05", "cpu2_06"],
        "downstream": ["cpu2_07", "cpu2_10", "cpu2_14", "cpu2_issue"],
        "route": ["cpu2_total", "cpu2_02", "cpu2_08", "cpu2_07"],
    },
    "cpu2_09": {
        "focus": "传感器采集、无线通信、读取部件参数和蓝牙 RSSI 快照，为测量判断、状态输出和故障管理提供输入。",
        "upstream": ["cpu2_01", "cpu2_14"],
        "downstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_07", "cpu2_10"],
        "route": ["cpu2_total", "cpu2_09", "cpu2_10", "cpu3_02", "cpu3_06"],
    },
    "cpu2_10": {
        "focus": "CPU2 与 CPU3 的共享寄存器和 Modbus 接口，是 CPU2 测量结果、无线 RSSI、协议版本 11 起 AO 运行态尾部字段返回 CPU3 的主通道。",
        "upstream": ["cpu2_02", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_07", "cpu3_02"],
        "downstream": ["cpu3_02", "cpu3_04", "cpu3_05", "cpu3_06", "cpu2_12"],
        "route": ["cross", "cpu2_04", "cpu2_10", "cpu3_02", "cpu3_06"],
    },
    "cpu2_11": {
        "focus": "HART 对外接口和 AO 电流/百分比状态输出，适合和 CPU2 液位结果、AoOutput 服务、AD5421 驱动和状态发布一起阅读。",
        "upstream": ["cpu2_02", "cpu2_10", "cpu2_12", "cpu2_13"],
        "downstream": ["cpu2_04", "cpu2_07", "cpu2_13", "cpu3_06"],
        "route": ["cross", "cpu3_07", "cpu2_13", "cpu2_11", "cpu3_06"],
    },
    "cpu2_12": {
        "focus": "CPU2 参数默认值、范围、保存和系统配置，是测量算法、通信协议和现场配置的共同数据源。",
        "upstream": ["cpu3_07", "cpu3_02", "cpu2_10"],
        "downstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_09", "cpu2_13"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_10", "cpu2_12", "cpu2_04"],
    },
    "cpu2_13": {
        "focus": "扭力、继电器和 AO 电流输出把测量状态转换为硬件输出，是现场报警、模拟量输出和 AD5421 运行态发布的关键链路。",
        "upstream": ["cpu2_01", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_12"],
        "downstream": ["cpu2_07", "cpu2_10", "cpu2_11"],
        "route": ["cross", "cpu3_07", "cpu2_12", "cpu2_13", "cpu2_10", "cpu3_06"],
    },
    "cpu2_14": {
        "focus": "BSP 外设驱动为电机、串口、传感器、FRAM、继电器和看门狗提供底层硬件能力。",
        "upstream": ["cpu2_01"],
        "downstream": ["cpu2_08", "cpu2_09", "cpu2_10", "cpu2_12", "cpu2_13"],
        "route": ["cpu2_total", "cpu2_14", "cpu2_08"],
    },
    "cpu2_15": {
        "focus": "公共算法和工具函数支撑多个业务页，适合在看具体问题时回查共用计算逻辑。",
        "upstream": ["cpu2_total"],
        "downstream": ["cpu2_04", "cpu2_05", "cpu2_06", "cpu2_10", "cpu2_12"],
        "route": ["cpu2_total", "cpu2_15", "cpu2_04"],
    },
    "cpu2_16": {
        "focus": "测试、调试和其它项目代码入口，主要用于现场排查和模拟流程，不作为正常业务主链路起点。",
        "upstream": ["cpu2_total", "cpu2_14"],
        "downstream": ["cpu2_07", "cpu2_08", "cpu2_10", "cpu2_issue"],
        "route": ["cpu2_total", "cpu2_16", "cpu2_issue"],
    },
    "cpu3_total": {
        "focus": "CPU3 所有流程页面的本地总览，适合从显示端、外部协议和 CPU2 轮询角度查找入口。",
        "upstream": ["global", "cross"],
        "downstream": ["cpu3_01", "cpu3_02", "cpu3_03", "cpu3_06", "cpu3_07"],
        "route": ["global", "cross", "cpu2_total"],
    },
    "cpu3_01": {
        "focus": "CPU3 上电初始化、主循环调度、显示任务、外部 COM 和 CPU2 轮询的优先级关系。",
        "upstream": ["global", "cpu3_total"],
        "downstream": ["cpu3_02", "cpu3_03", "cpu3_06", "cpu3_07", "cpu3_08"],
        "route": ["cpu3_total", "cpu3_01", "cpu3_02"],
    },
    "cpu3_02": {
        "focus": "CPU3 作为 CPU2 Modbus 主站，负责写指令/参数、读输入/保持寄存器、密度点分批回读；状态、参数、当前连接协议快照、连续请求失败和故障恢复锁存独立，非命令写失败会立即关闭参数门禁并强制补读，连续第 10 个请求未获得合法响应时置本机通信故障。",
        "upstream": ["cpu3_01", "cpu3_03", "cpu3_04", "cpu3_05", "cpu3_07"],
        "downstream": ["cpu2_10", "cpu2_02", "cpu2_04", "cpu2_05", "cpu2_06", "cpu3_06"],
        "route": ["cross", "cpu3_07", "cpu3_02", "cpu2_02", "cpu2_04", "cpu2_10"],
    },
    "cpu3_03": {
        "focus": "三路外部 COM 的协议选择、帧分发、RS485 发送和异常恢复，是外部系统进入 CPU3 的入口。",
        "upstream": ["cpu3_01", "cpu3_08"],
        "downstream": ["cpu3_04", "cpu3_05", "cpu3_02"],
        "route": ["cross", "cpu3_03", "cpu3_04", "cpu3_02", "cpu2_02"],
    },
    "cpu3_04": {
        "focus": "DSM 协议读写映射，把外部读写线圈/寄存器转换为 CPU3 缓存访问或 CPU2 指令下发。",
        "upstream": ["cpu3_03"],
        "downstream": ["cpu3_02", "cpu2_02", "cpu2_04", "cpu2_05", "cpu2_12"],
        "route": ["cross", "cpu3_03", "cpu3_04", "cpu3_02", "cpu2_02"],
    },
    "cpu3_05": {
        "focus": "Wartsila 和 SI协议适配，将 CPU2 测量结果、参数和密度点映射给外部系统。",
        "upstream": ["cpu3_03", "cpu3_02", "cpu2_10"],
        "downstream": ["cpu3_02", "cpu2_06", "cpu2_04", "cpu2_05"],
        "route": ["cross", "cpu3_05", "cpu3_02", "cpu2_06", "cpu3_05"],
    },
    "cpu3_06": {
        "focus": "显示刷新、按键事件、状态页展示和调试等待页；状态与当前连接协议快照均建立前保持通讯尝试页，第 10 次请求失败切换 CPU2 通信故障页，已建立通信后偶发失败不回退，恢复同步完成前不显示伪协议结果或失败请求的成功反馈。",
        "upstream": ["cpu3_01", "cpu3_02", "cpu3_07", "cpu3_08", "cpu2_10"],
        "downstream": ["cpu3_07", "cpu2_02", "cpu2_12"],
        "route": ["cross", "cpu2_10", "cpu3_02", "cpu3_06", "cpu3_07"],
    },
    "cpu3_07": {
        "focus": "菜单参数、指令确认、保护确认、测量/调试菜单分组、AO 输出使能和 CPU2 指令/参数下发，是人工操作进入测量链路的主入口。",
        "upstream": ["cpu3_06", "cpu3_08"],
        "downstream": ["cpu3_02", "cpu2_02", "cpu2_04", "cpu2_05", "cpu2_06", "cpu2_12"],
        "route": ["cross", "cpu3_06", "cpu3_07", "cpu3_02", "cpu2_02"],
    },
    "cpu3_08": {
        "focus": "CPU3 本机通信/显示参数、FRAM、RTC、OLED 和 UART 恢复，为显示端稳定运行提供支撑。",
        "upstream": ["cpu3_01"],
        "downstream": ["cpu3_03", "cpu3_06", "cpu3_07"],
        "route": ["cpu3_total", "cpu3_08", "cpu3_03"],
    },
}


def analyze_flow_governance(
    routes: Sequence[Mapping[str, object]] = ROUTES,
    evidence: Mapping[str, object] = EVIDENCE_MANIFEST_DATA,
    relations: Mapping[str, Mapping[str, object]] = RELATIONS,
    *,
    review_window_days: int = 90,
) -> Dict[str, object]:
    """Return high-level flow governance findings without duplicating business data."""
    core_page_keys: set[str] = set()
    duplicate_steps: list[dict[str, str]] = []
    intentional_reuse: list[dict[str, object]] = []
    closure_issues: list[dict[str, str]] = []
    for route in routes:
        route_id = str(route.get("id", "unknown"))
        steps = route.get("steps", [])
        formal_steps = [
            step
            for step in steps
            if isinstance(step, dict)
            and step.get("formal") is True
            and isinstance(step.get("pageKey"), str)
        ] if isinstance(steps, list) else []
        formal_page_keys = {str(step["pageKey"]) for step in formal_steps}
        core_page_keys.update(formal_page_keys)

        signatures_by_page: dict[str, list[tuple[str, ...]]] = {}
        for step in formal_steps:
            page_key = str(step["pageKey"])
            signature = tuple(str(step.get(field, "")) for field in ("owner", "role", "label", "detail", "siteHref"))
            signatures_by_page.setdefault(page_key, []).append(signature)
        for page_key, signatures in signatures_by_page.items():
            if len(signatures) < 2:
                continue
            if len(set(signatures)) == len(signatures):
                intentional_reuse.append({"routeId": route_id, "pageKey": page_key, "count": len(signatures)})
            else:
                duplicate_steps.append({"routeId": route_id, "pageKey": page_key})

        closure = route.get("closure")
        if not isinstance(closure, dict):
            closure_issues.append({"routeId": route_id, "roleId": "all"})
            continue
        for role_id in CLOSURE_ROLE_IDS:
            page_keys = closure.get(role_id)
            if not isinstance(page_keys, list) or not page_keys:
                closure_issues.append({"routeId": route_id, "roleId": role_id})
                continue
            if any(str(page_key) not in formal_page_keys for page_key in page_keys):
                closure_issues.append({"routeId": route_id, "roleId": role_id})

    evidence_pages = evidence.get("pages", [])
    evidence_page_keys = {
        str(page["pageKey"])
        for page in evidence_pages
        if isinstance(page, dict) and isinstance(page.get("pageKey"), str)
    } if isinstance(evidence_pages, list) else set()
    orphan_pages = sorted(evidence_page_keys - core_page_keys)
    missing_evidence = sorted(core_page_keys - evidence_page_keys)

    relation_gaps: list[dict[str, object]] = []
    for page_key in sorted(core_page_keys):
        relation = relations.get(page_key)
        upstream = relation.get("upstream") if isinstance(relation, Mapping) else None
        downstream = relation.get("downstream") if isinstance(relation, Mapping) else None
        missing_upstream = not isinstance(upstream, list) or not upstream
        missing_downstream = not isinstance(downstream, list) or not downstream
        if missing_upstream or missing_downstream:
            relation_gaps.append(
                {
                    "pageKey": page_key,
                    "missingUpstream": missing_upstream,
                    "missingDownstream": missing_downstream,
                }
            )

    updated = evidence.get("updated")
    try:
        reference_date = date.fromisoformat(str(updated))
    except ValueError:
        reference_date = date.today()
    stale_reviews: list[str] = []
    if isinstance(evidence_pages, list):
        for page in evidence_pages:
            if not isinstance(page, dict):
                continue
            try:
                reviewed_at = date.fromisoformat(str(page.get("reviewedAt")))
            except ValueError:
                continue
            if (reference_date - reviewed_at).days > review_window_days:
                stale_reviews.append(str(page.get("pageKey")))

    issues: list[dict[str, str]] = []
    issues.extend({"rule": "orphan-page", "message": f"{page_key} is not used by any route"} for page_key in orphan_pages)
    issues.extend({"rule": "missing-evidence", "message": f"{page_key} has no evidence entry"} for page_key in missing_evidence)
    issues.extend({"rule": "duplicate-step", "message": f"{item['routeId']} duplicates {item['pageKey']}"} for item in duplicate_steps)
    issues.extend({"rule": "closure", "message": f"{item['routeId']} has invalid {item['roleId']} closure"} for item in closure_issues)
    issues.extend({"rule": "relation-gap", "message": f"{item['pageKey']} has an upstream/downstream gap"} for item in relation_gaps)
    issues.extend({"rule": "stale-review", "message": f"{page_key} review is older than {review_window_days} days"} for page_key in stale_reviews)
    return {
        "issues": issues,
        "intentionalReuse": intentional_reuse,
        "metrics": {
            "routes": len(routes),
            "corePages": len(core_page_keys),
            "closureRoles": len(routes) * len(CLOSURE_ROLE_IDS),
            "verifiedPages": sum(
                1
                for page in evidence_pages
                if isinstance(page, dict) and page.get("status") == "verified"
            ) if isinstance(evidence_pages, list) else 0,
            "blockingIssues": len(issues) - len(stale_reviews),
            "warnings": len(stale_reviews),
        },
    }


INJECT_CSS = """
.cross-flow-nav{margin:16px 0 20px;padding:16px 18px;border:1px solid #cfe0f2;border-radius:10px;background:#fff;box-shadow:0 8px 22px rgba(31,59,91,.07)}
.cross-flow-nav *{box-sizing:border-box}
.cross-flow-nav__head{display:flex;justify-content:space-between;gap:12px;align-items:flex-start;margin-bottom:12px}
.cross-flow-nav__kicker{display:block;color:#4f6b86;font-size:12px;font-weight:700;margin-bottom:3px}
.cross-flow-nav__title{font-size:18px;font-weight:850;color:#17233a}
.cross-flow-nav__crumbs{display:flex;flex-wrap:wrap;gap:6px;align-items:center;margin:0 0 10px;color:#5a6f86;font-size:12px}
.cross-flow-nav__crumbs a{color:#245d96;text-decoration:none;font-weight:700}
.cross-flow-nav__crumbs span{overflow-wrap:anywhere}
.cross-flow-evidence{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;margin:0 0 14px;padding:10px;border:1px solid #cfe0f2;border-radius:8px;background:#f5f9fd}
.cross-flow-evidence__item{min-width:0;padding:8px 9px;border-left:3px solid #248b70;background:#fff}
.cross-flow-evidence__item span{display:block;margin-bottom:4px;color:#5a6f86;font-size:11px;font-weight:700}
.cross-flow-evidence__item strong,.cross-flow-evidence__links{display:block;color:#17233a;font-size:12px;line-height:1.55;overflow-wrap:anywhere}
.cross-flow-evidence__links a{color:#245d96;text-decoration:none;font-weight:700}
.cross-flow-evidence__links a+a::before{content:" · ";color:#8393a5}
.cross-flow-nav__quick{display:flex;flex-wrap:wrap;gap:7px;justify-content:flex-end}
.cross-flow-nav__quick a,.cross-flow-nav__links a{display:inline-flex;align-items:center;min-height:28px;padding:4px 9px;border:1px solid #d7e5f3;border-radius:999px;background:#f6faff;color:#245d96;text-decoration:none;font-size:12px;font-weight:700;max-width:100%;white-space:normal;overflow-wrap:anywhere;word-break:break-word}
.cross-flow-nav__grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:10px}
.cross-flow-nav__card{border:1px solid #dce7f2;border-radius:8px;background:#fbfdff;padding:12px}
.cross-flow-nav__card b{display:block;margin-bottom:5px;color:#16263b}
.cross-flow-nav__card p{margin:0 0 8px;color:#43556b;line-height:1.62;font-size:13px}
.cross-flow-nav__links{display:flex;flex-wrap:wrap;gap:6px}
.cross-flow-nav__linear{display:flex;justify-content:space-between;gap:10px;margin-top:12px;padding-top:12px;border-top:1px solid #e2ebf5}
.cross-flow-nav__linear a{flex:1 1 0;min-width:0;padding:9px 11px;border:1px solid #d7e5f3;border-radius:8px;background:#f8fbff;color:#214f7d;text-decoration:none;font-size:13px;font-weight:800;overflow-wrap:anywhere}
.cross-flow-nav__linear a:last-child{text-align:right}
@media(max-width:920px){.cross-flow-evidence{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media(max-width:720px){.cross-flow-nav{padding:14px;max-width:100%;overflow:hidden}.cross-flow-nav__head{display:block}.cross-flow-nav__title{font-size:16px}.cross-flow-nav__quick{justify-content:flex-start;margin-top:10px}.cross-flow-evidence{grid-template-columns:1fr}.cross-flow-nav__grid{grid-template-columns:1fr}.cross-flow-nav__linear{flex-direction:column}.cross-flow-nav__linear a:last-child{text-align:left}}
""".strip()


GLOBAL_CSS = """
:root{--ink:#17233a;--muted:#5c6f85;--line:#d5e1ed;--panel:#fff;--bg:#f2f6fb;--blue:#2f78c8;--green:#248b70;--amber:#b97816;--red:#b84a55;--violet:#6658bd;--shadow:0 12px 32px rgba(31,59,91,.10)}
*{box-sizing:border-box}
html{overflow-x:hidden}
body{margin:0;background:linear-gradient(180deg,#f6f9fd 0,#edf3f9 100%);color:var(--ink);font:15px/1.72 "Microsoft YaHei",Segoe UI,Arial,sans-serif;overflow-x:hidden}
a{color:#1f65aa;text-decoration:none}
code{font-family:Consolas,"Courier New",monospace;background:#eef4fb;border:1px solid #d7e4f2;border-radius:5px;padding:1px 5px;overflow-wrap:anywhere;word-break:break-all}
.wrap{max-width:1240px;margin:0 auto;padding:28px 28px 64px}
.hero{background:linear-gradient(135deg,#163a62,#0a7775);color:#fff;border-radius:12px;padding:30px 34px;box-shadow:var(--shadow)}
.hero h1{margin:0 0 10px;font-size:31px;line-height:1.25}
.hero p{margin:0;max-width:980px;color:#e6f4ff}
.meta-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;margin-top:22px}
.meta{border:1px solid rgba(255,255,255,.24);background:rgba(255,255,255,.09);border-radius:8px;padding:12px 14px}
.meta span{display:block;color:#cbe7ff;font-size:12px;font-weight:700}
.meta strong{display:block;margin-top:4px;color:#fff}
.topnav{position:sticky;top:0;z-index:5;margin:18px 0;padding:10px;background:rgba(255,255,255,.94);border:1px solid #d8e4f0;border-radius:10px;box-shadow:0 6px 18px rgba(38,65,96,.08);display:flex;flex-wrap:wrap;gap:8px}
.topnav a{padding:7px 10px;border-radius:6px;background:#f4f8fd;border:1px solid #dce8f6;color:#23527d;font-size:13px;font-weight:700}
section{background:var(--panel);border:1px solid #d7e2ef;border-radius:10px;margin:18px 0;padding:22px 24px;box-shadow:0 8px 24px rgba(35,58,92,.06)}
h2{margin:0 0 10px;font-size:24px}
h3{margin:16px 0 8px;font-size:18px}
.lead{color:var(--muted);margin:0 0 14px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px}
.card{border:1px solid #dbe6f2;border-radius:8px;background:#fbfdff;padding:13px}
.card strong{display:block;margin-bottom:5px;color:#15263c}
.card p{margin:0 0 8px;color:#46596f}
.links{display:flex;flex-wrap:wrap;gap:7px}
.pill{display:inline-flex;align-items:center;min-height:28px;padding:4px 9px;border-radius:999px;background:#f4f8fd;border:1px solid #d8e5f4;color:#245d96;font-size:12px;font-weight:700}
.split{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.flow-wrap{width:100%;max-width:100%;min-width:0;overflow-x:auto;overflow-y:hidden;border:1px solid #d7e2ef;border-radius:10px;background:#fbfdff;margin-top:12px;-webkit-overflow-scrolling:touch}
.route-svg{display:block;min-width:0!important;max-width:100%!important;width:100%!important;height:auto!important}
.node{stroke-width:2}
.node.cpu3{fill:#eaf4ff;stroke:#5d9be2}
.node.cpu2{fill:#eaf8f1;stroke:#39a97e}
.node.ext{fill:#fff6df;stroke:#d39a31}
.node.state{fill:#f3f1ff;stroke:#7464c9}
.node.error{fill:#fff0f1;stroke:#c55c66}
.edge{fill:none;stroke:#61758e;stroke-width:2}
.edge-red{fill:none;stroke:#b84a55;stroke-width:2}
.nt{text-anchor:middle;font-size:16px;font-weight:800;fill:#17233a}
.ns{text-anchor:middle;font-size:12px;fill:#5c6f85}
.route-card{border-left:4px solid #2f78c8}
.route-card:nth-child(2n){border-left-color:#248b70}
.route-card:nth-child(3n){border-left-color:#b97816}
.steps{display:grid;gap:8px;margin-top:10px}
.step{display:grid;grid-template-columns:minmax(0,116px) minmax(0,1fr);gap:8px;align-items:start;padding:9px;border:1px solid #e0e8f2;border-radius:8px;background:#fff;min-width:0}
.step b{font-size:13px;color:#28476a}
.step span,.step b{overflow-wrap:anywhere;word-break:break-word}
.step span{display:block;color:#4d6076;font-size:13px}
.route-closure{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px;margin:12px 0;padding:10px;border:1px solid #d7e2ef;border-radius:8px;background:#f5f9fd}
.route-closure__item{min-width:0;padding:8px 9px;border-left:3px solid #248b70;background:#fff}
.route-closure__item b{display:block;margin-bottom:4px;color:#28476a;font-size:12px}
.route-closure__item span{display:flex;flex-wrap:wrap;gap:4px;color:#4d6076;font-size:12px;line-height:1.5}
.route-closure__item a{font-weight:700}
.note{border:1px solid #bad8fb;background:#eef7ff;border-radius:8px;padding:11px 13px}
@media(max-width:800px){.wrap{padding:16px;max-width:100vw}.hero{padding:24px 22px;overflow:hidden}.hero h1{font-size:24px}.meta-grid,.split,.route-closure{grid-template-columns:1fr}.meta{min-width:0;overflow-wrap:anywhere;word-break:break-word}section{padding:18px;max-width:100%}.step{grid-template-columns:1fr}.topnav{position:static}.topnav a{white-space:normal;overflow-wrap:anywhere}.flow-wrap{overflow-x:visible}.route-svg{width:100%!important;min-width:0!important;max-width:100%!important}.nt{font-size:13px}.ns{font-size:10px}}
""".strip()


def replace_legacy_svg_labels(text: str) -> str:
    def replace(match):  # noqa: ANN001, ANN202
        opening = match.group("opening")
        if ARIA_LABEL_ATTR_RE.search(opening) is None:
            return match.group(0)
        cleaned_opening = ARIA_ACCESSIBILITY_ATTR_RE.sub("", opening)
        cleaned_opening = SVG_ROOT_PRESENTATION_ATTR_RE.sub("", cleaned_opening)
        cleaned_opening = cleaned_opening[:-1].rstrip()
        return (
            f'{cleaned_opening} aria-hidden="true" focusable="false">'
            f'{match.group("metadata")}'
        )

    return SVG_METADATA_RE.sub(replace, text)


def normalize_duplicate_svg_references(text: str, source: Path | None = None) -> str:
    def normalize_opening(match):  # noqa: ANN001, ANN202
        opening = match.group(0)
        seen: dict[str, str] = {}
        duplicate_spans: list[tuple[int, int]] = []
        for attribute in ARIA_REFERENCE_ATTR_RE.finditer(opening):
            name = attribute.group("name").lower()
            value = attribute.group("value")
            if name not in seen:
                seen[name] = value
                continue
            if seen[name] != value:
                location = f" in {source}" if source is not None else ""
                raise RuntimeError(
                    f"Conflicting duplicate {name}{location}: "
                    f'{seen[name]!r} != {value!r}'
                )
            duplicate_spans.append(attribute.span())

        if not duplicate_spans:
            return opening
        parts = []
        cursor = 0
        for start, end in duplicate_spans:
            parts.append(opening[cursor:start])
            cursor = end
        parts.append(opening[cursor:])
        return "".join(parts)

    return SVG_OPENING_TAG_RE.sub(normalize_opening, text)


def _plain_html_text(fragment: str) -> str:
    """提取短标题文字，仅用于已有标题派生表格 caption。"""

    return " ".join(html.unescape(HTML_TAG_RE.sub(" ", fragment)).split())


def _caption_from_heading(fragment: str) -> str:
    heading = _plain_html_text(fragment)
    concise = HEADING_NUMBER_PREFIX_RE.sub("", heading).strip()
    return concise or heading


def normalize_source_card_headings(text: str) -> str:
    """源码卡片是章节的直接子主题，统一使用 h3，避免 h2 跳到 h4。"""

    def replace(match):  # noqa: ANN001, ANN202
        return (
            f'{match.group("prefix")}<h3{match.group("attrs")}>'
            f'{match.group("body")}</h3>'
        )

    return SOURCE_CARD_H4_RE.sub(replace, text)


def _add_column_scope(match):  # noqa: ANN001, ANN202
    opening = match.group(0)
    if SCOPE_ATTR_RE.search(opening):
        return opening
    colspan = COLSPAN_ATTR_RE.search(opening)
    if colspan and int(colspan.group("value")) > 1:
        # 跨多列的表头可能是 colgroup，也可能需要 headers/id；留给作者判断。
        return opening
    return opening[:-1].rstrip() + ' scope="col">'


def normalize_table_semantics(text: str, source: Path | None = None) -> str:
    """从紧邻章节标题补 caption，并为确定的列表头补 scope。"""

    tables = list(TABLE_RE.finditer(text))
    if not tables:
        return text

    headings = list(HEADING_RE.finditer(text))
    contexts: list[tuple[int, str]] = []
    for table in tables:
        if TABLE_OPENING_RE.search(table.group("body")):
            location = f" in {source}" if source is not None else ""
            raise RuntimeError(f"Nested table needs author review{location}")
        heading = next(
            (candidate for candidate in reversed(headings) if candidate.end() <= table.start()),
            None,
        )
        if heading is None:
            location = f" in {source}" if source is not None else ""
            raise RuntimeError(f"Table has no preceding heading{location}")
        caption = _caption_from_heading(heading.group("body"))
        if not caption:
            location = f" in {source}" if source is not None else ""
            raise RuntimeError(f"Table heading is empty{location}")
        contexts.append((heading.start(), caption))

    context_totals: dict[int, int] = {}
    for heading_start, _ in contexts:
        context_totals[heading_start] = context_totals.get(heading_start, 0) + 1
    context_ordinals: dict[int, int] = {}
    table_index = 0

    def replace_table(match):  # noqa: ANN001, ANN202
        nonlocal table_index
        heading_start, caption = contexts[table_index]
        table_index += 1
        context_ordinals[heading_start] = context_ordinals.get(heading_start, 0) + 1
        if context_totals[heading_start] > 1:
            caption += f'（表 {context_ordinals[heading_start]}）'

        body = match.group("body")
        caption_matches = list(CAPTION_RE.finditer(body))
        if not caption_matches:
            body = f'<caption>{html.escape(caption)}</caption>' + body
        elif len(caption_matches) == 1 and not _plain_html_text(
            caption_matches[0].group("body")
        ):
            empty_caption = caption_matches[0]
            replacement = (
                f'<caption{empty_caption.group("attrs")}>'
                f'{html.escape(caption)}</caption>'
            )
            body = (
                body[: empty_caption.start()]
                + replacement
                + body[empty_caption.end() :]
            )

        def normalize_thead(thead_match):  # noqa: ANN001, ANN202
            normalized_body = TH_OPENING_RE.sub(
                _add_column_scope,
                thead_match.group("body"),
            )
            return (
                thead_match.group("opening")
                + normalized_body
                + thead_match.group("closing")
            )

        body = THEAD_RE.sub(normalize_thead, body)
        return match.group("opening") + body + match.group("closing")

    return TABLE_RE.sub(replace_table, text)


def normalize_flow_semantics(text: str, source: Path | None = None) -> str:
    """幂等规范化标题和表格语义；不推断跨列表头等含糊结构。"""

    text = normalize_source_card_headings(text)
    return normalize_table_semantics(text, source)


def page_path(key: str) -> Path:
    return Path(PAGE_DEFS[key]["path"])


def rel_href(from_file: Path, to_key: str, anchor: str = "") -> str:
    target = page_path(to_key)
    href = Path(os.path.relpath(target, from_file.parent)).as_posix()
    return href + anchor


def link(from_file: Path, key: str, label: str | None = None, anchor: str = "") -> str:
    title = label if label is not None else PAGE_DEFS[key]["title"]
    return f'<a href="{html.escape(rel_href(from_file, key, anchor))}">{html.escape(title)}</a>'


def link_pill(from_file: Path, key: str, label: str | None = None, anchor: str = "") -> str:
    title = label if label is not None else PAGE_DEFS[key]["title"]
    return f'<a class="pill" href="{html.escape(rel_href(from_file, key, anchor))}">{html.escape(title)}</a>'


def rel_path_to_file(from_file: Path, target: Path) -> str:
    return Path(os.path.relpath(target, from_file.parent)).as_posix()


def legacy_link(from_file: Path, item: Mapping[str, str]) -> str:
    return html.escape(rel_path_to_file(from_file, Path(item["path"])))


def linear_neighbors(key: str) -> Tuple[str | None, str | None]:
    cpu = PAGE_DEFS[key]["cpu"]
    order = LINEAR_ORDERS.get(cpu)
    if not order or key not in order:
        return None, None
    index = order.index(key)
    prev_key = order[index - 1] if index > 0 else None
    next_key = order[index + 1] if index < len(order) - 1 else None
    return prev_key, next_key


def breadcrumb(from_file: Path, key: str) -> str:
    cpu = PAGE_DEFS[key]["cpu"]
    parts = [link(from_file, "global", "统一入口")]
    if cpu == "CPU2" and key != "cpu2_total":
        parts.append(link(from_file, "cpu2_total", "CPU2 总览"))
    elif cpu == "CPU3" and key != "cpu3_total":
        parts.append(link(from_file, "cpu3_total", "CPU3 总览"))
    parts.append(f'<span>{html.escape(PAGE_DEFS[key]["title"])}</span>')
    return '<div class="cross-flow-nav__crumbs">' + "<span>/</span>".join(parts) + "</div>"


def _evidence_links(from_file: Path, paths: Sequence[str], labels: Sequence[str] | None = None) -> str:
    anchors = []
    for index, relative_path in enumerate(paths):
        target = ROOT / relative_path
        label = labels[index] if labels is not None else relative_path.split("/", 1)[-1]
        anchors.append(
            f'<a href="{html.escape(rel_path_to_file(from_file, target))}" '
            f'title="{html.escape(relative_path)}">{html.escape(label)}</a>'
        )
    return '<span class="cross-flow-evidence__links">' + "".join(anchors) + "</span>"


def make_evidence_head(from_file: Path, key: str) -> str:
    evidence = EVIDENCE_PAGES_BY_KEY.get(key)
    if evidence is None:
        return ""
    status_labels = {
        "verified": "已核对",
        "needs-review": "需复核",
        "draft": "草稿",
    }
    status = str(evidence["status"])
    reviewed_at = str(evidence["reviewedAt"])
    firmware = EVIDENCE_MANIFEST_DATA["firmwareBaseline"]
    cpu2_version = str(firmware["cpu2"]["version"])  # type: ignore[index]
    cpu3_version = str(firmware["cpu3"]["version"])  # type: ignore[index]
    source_paths = [str(item) for item in evidence["sourcePaths"]]  # type: ignore[index]
    validation_paths = [
        *[str(item) for item in EVIDENCE_MANIFEST_DATA["defaultValidationPaths"]],  # type: ignore[index]
        *[str(item) for item in evidence["validationPaths"]],  # type: ignore[index]
    ]
    validation_labels = ["当前版本验证"] + [
        f"专项验证 {index}"
        for index in range(1, len(validation_paths))
    ]
    return (
        f'<div class="cross-flow-evidence" data-evidence-page="{html.escape(key)}" '
        f'data-evidence-status="{html.escape(status)}">'
        '<div class="cross-flow-evidence__item"><span>可信状态</span>'
        f'<strong>{html.escape(status_labels[status])} · {html.escape(reviewed_at)}</strong></div>'
        '<div class="cross-flow-evidence__item"><span>适用固件基线</span>'
        f'<strong>CPU2 {html.escape(cpu2_version)} / CPU3 {html.escape(cpu3_version)}</strong></div>'
        '<div class="cross-flow-evidence__item"><span>主源码入口</span>'
        + _evidence_links(from_file, source_paths)
        + '</div><div class="cross-flow-evidence__item"><span>验证资料</span>'
        + _evidence_links(from_file, validation_paths, validation_labels)
        + "</div></div>"
    )


def pills(from_file: Path, keys: Sequence[str]) -> str:
    return "".join(link_pill(from_file, key) for key in keys)


def route_href(from_file: Path, route_id: str) -> str:
    return rel_href(from_file, "cross", f"#{route_id}")


def ensure_style(text: str) -> str:
    text = re.sub(
        r"\s*<style id=\"cross-flow-nav-style\">.*?</style>",
        "",
        text,
        flags=re.S,
    )
    style = f"\n<style id=\"cross-flow-nav-style\">\n{INJECT_CSS}\n</style>"
    if "</head>" not in text:
        raise RuntimeError("HTML page has no </head> marker")
    enhancement_link = re.search(
        rf'<link\b[^>]*\bhref=["\'][^"\']*{re.escape(EMBED_STYLESHEET_NAME)}(?:\?[^"\']*)?["\'][^>]*>',
        text,
        flags=re.I,
    )
    if enhancement_link:
        return (
            text[: enhancement_link.start()].rstrip()
            + style
            + "\n"
            + text[enhancement_link.start() :]
        )
    head_end = text.find("</head>")
    return text[:head_end].rstrip() + style + "\n" + text[head_end:]


def remove_nav_block(text: str) -> str:
    pattern = re.escape(NAV_START) + r".*?" + re.escape(NAV_END)
    return re.sub(r"\s*" + pattern + r"\s*", "\n", text, flags=re.S)


def remove_legacy_notice(text: str) -> str:
    pattern = re.escape(LEGACY_START) + r".*?" + re.escape(LEGACY_END)
    return re.sub(r"\s*" + pattern + r"\s*", "\n", text, flags=re.S)


def insert_after_first(text: str, marker: str, block: str) -> str:
    idx = text.find(marker)
    if idx < 0:
        return ""
    idx += len(marker)
    return text[:idx] + block + text[idx:]


def make_relation_block(key: str) -> str:
    from_file = page_path(key)
    relation = RELATIONS.get(key, {})
    focus = str(relation.get("focus", "本页属于程序流程文档体系，可通过统一入口、跨 CPU 链路和本 CPU 总览继续定位上下游。"))
    upstream = list(relation.get("upstream", ["global"]))
    downstream = list(relation.get("downstream", []))
    route = list(relation.get("route", ["global", "cross"]))
    current_title = PAGE_DEFS[key]["title"]
    quick_links = [
        link(from_file, "global", "统一入口"),
        link(from_file, "cross", "跨 CPU 链路"),
    ]
    if PAGE_DEFS[key]["cpu"] == "CPU2":
        quick_links.extend(
            [
                link(from_file, "cpu2_total", "CPU2 总览"),
                link(from_file, "cpu3_total", "CPU3 总览"),
            ]
        )
    elif PAGE_DEFS[key]["cpu"] == "CPU3":
        quick_links.extend(
            [
                link(from_file, "cpu3_total", "CPU3 总览"),
                link(from_file, "cpu2_total", "CPU2 总览"),
            ]
        )
    else:
        quick_links.extend(
            [
                link(from_file, "cpu2_total", "CPU2 总览"),
                link(from_file, "cpu3_total", "CPU3 总览"),
            ]
        )

    cards = [
        (
            "上游入口",
            "先看这些页面，可以知道本页由哪个菜单、协议、主循环或测量状态触发。",
            upstream,
        ),
        (
            "本页定位",
            focus,
            [key],
        ),
        (
            "下游输出",
            "本页处理完成后，通常会影响这些测量、通信、显示、故障或参数页面。",
            downstream,
        ),
        (
            "建议阅读路线",
            "按业务链路阅读时，建议按这些页面顺序前后跳转。",
            route,
        ),
    ]

    card_html = []
    for title, desc, keys in cards:
        links = "".join(link(from_file, item) for item in keys if item in PAGE_DEFS)
        if not links:
            links = link(from_file, "cross", "查看跨 CPU 链路")
        card_html.append(
            f'<article class="cross-flow-nav__card"><b>{html.escape(title)}</b>'
            f"<p>{html.escape(desc)}</p><div class=\"cross-flow-nav__links\">{links}</div></article>"
        )

    prev_key, next_key = linear_neighbors(key)
    linear_links = []
    if prev_key:
        linear_links.append(link(from_file, prev_key, "上一页：" + PAGE_DEFS[prev_key]["title"]))
    else:
        linear_links.append(link(from_file, "global", "上一层：程序流程统一入口"))
    if next_key:
        linear_links.append(link(from_file, next_key, "下一页：" + PAGE_DEFS[next_key]["title"]))
    else:
        linear_links.append(link(from_file, "cross", "下一步：跨 CPU 业务链路"))

    head = (
        '<div class="cross-flow-nav__head"><div>'
        '<span class="cross-flow-nav__kicker">程序流程关联导航</span>'
        f'<div class="cross-flow-nav__title">{html.escape(current_title)} 在整机链路中的位置</div>'
        '</div><div class="cross-flow-nav__quick">'
        + "".join(quick_links)
        + "</div></div>"
    )
    grid = '<div class="cross-flow-nav__grid">' + "".join(card_html) + "</div>"
    linear = '<div class="cross-flow-nav__linear">' + "".join(linear_links) + "</div>"
    return (
        f"\n{NAV_START}\n"
        '<section class="cross-flow-nav" id="cross-flow-nav" aria-label="程序流程关联导航">'
        + breadcrumb(from_file, key)
        + make_evidence_head(from_file, key)
        + head
        + grid
        + linear
        + "</section>\n"
        f"{NAV_END}\n"
    )


def inject_page(plan: InMemoryUpdatePlan, key: str) -> None:
    path = page_path(key)
    if not plan.exists(path):
        return
    text = replace_legacy_svg_labels(plan.read_text(path))
    text = normalize_duplicate_svg_references(text, path)
    text = ensure_style(remove_nav_block(text))
    block = make_relation_block(key)
    updated = insert_after_first(text, "</nav>", block)
    if not updated:
        updated = insert_after_first(text, "</header>", block)
    if not updated:
        raise RuntimeError(f"Cannot find insertion point in {path}")
    plan.stage_text(path, updated)


def make_legacy_notice(item: Mapping[str, str]) -> str:
    from_file = Path(item["path"])
    current_key = item["current"]
    global_href = html.escape(rel_href(from_file, "global"))
    current_href = html.escape(rel_href(from_file, current_key))
    route_id = item.get("route")

    links = [
        f'<a style="color:#1f65aa;font-weight:700;text-decoration:none" href="{current_href}">'
        f'当前权威流程：{html.escape(PAGE_DEFS[current_key]["title"])}'
        "</a>",
        f'<a style="color:#1f65aa;font-weight:700;text-decoration:none" href="{global_href}">'
        "程序流程统一入口</a>",
    ]
    if route_id:
        route_href_value = html.escape(route_href(from_file, route_id))
        links.append(
            f'<a style="color:#1f65aa;font-weight:700;text-decoration:none" href="{route_href_value}">'
            "跨 CPU 业务链路</a>"
        )

    parts = [
        f"\n{LEGACY_START}\n",
        '<section class="flow-legacy-notice" ',
        'style="margin:16px auto;padding:14px 16px;border:1px solid #bad8fb;',
        'border-radius:10px;background:#eef7ff;color:#17233a;',
        "font-size:14px;line-height:1.65;font-family:'Microsoft YaHei','Segoe UI',Arial,sans-serif;",
        'max-width:min(1180px,calc(100% - 32px));box-sizing:border-box">',
        '<b style="display:block;margin-bottom:4px">历史/专题流程提示</b>',
        f'<p style="margin:0 0 8px">{html.escape(item["note"])}</p>',
        '<div style="display:flex;flex-wrap:wrap;gap:8px">',
        "".join(links),
        "</div></section>\n",
        f"{LEGACY_END}\n",
    ]
    return "".join(parts)

def insert_after_opening_body(text: str, block: str) -> str:
    match = re.search(r"<body\b[^>]*>", text, flags=re.I)
    if not match:
        return ""
    return text[: match.end()] + block + text[match.end() :]


def inject_legacy_notice(plan: InMemoryUpdatePlan, item: Mapping[str, str]) -> None:
    path = Path(item["path"])
    if not plan.exists(path):
        return
    text = remove_legacy_notice(plan.read_text(path))
    text = normalize_duplicate_svg_references(text, path)
    block = make_legacy_notice(item)
    updated = insert_after_first(text, "</header>", block)
    if not updated:
        updated = insert_after_first(text, "<body>", block)
    if not updated:
        updated = insert_after_opening_body(text, block)
    if not updated:
        raise RuntimeError(f"Cannot find legacy notice insertion point in {path}")
    plan.stage_text(path, updated)


def inject_legacy_notices(plan: InMemoryUpdatePlan) -> None:
    for item in LEGACY_DOCS:
        inject_legacy_notice(plan, item)


def normalize_all_flow_documents(plan: InMemoryUpdatePlan) -> None:
    """规范化正式流程源；docs-site 仍只通过同步消费这些结果。"""

    paths = set(DOC_NAV_DIR.rglob("*.html")) | {GLOBAL_INDEX, CROSS_ROUTE}
    for path in sorted(paths, key=lambda item: str(item).casefold()):
        if not plan.exists(path):
            continue
        plan.stage_text(path, normalize_flow_semantics(plan.read_text(path), path))


def formal_route_steps(route: Mapping[str, object]) -> List[Mapping[str, object]]:
    steps = route.get("steps")
    if not isinstance(steps, list):
        return []
    return [step for step in steps if isinstance(step, dict) and step.get("formal") is True]


def route_steps(from_file: Path, route: Mapping[str, object]) -> str:
    items = []
    for index, step in enumerate(formal_route_steps(route), start=1):
        stage = str(step["role"])
        key = str(step["pageKey"])
        desc = str(step["detail"])
        items.append(
            '<div class="step">'
            f"<b>{index}. {html.escape(stage)}</b>"
            f"<span>{link(from_file, key)}：{html.escape(desc)}</span>"
            "</div>"
        )
    return "".join(items)


def route_closure(from_file: Path, route: Mapping[str, object]) -> str:
    closure = route.get("closure")
    if not isinstance(closure, dict):
        return ""
    items = []
    closure_roles = ROUTE_MANIFEST_DATA.get("closureRoles", [])
    for role in closure_roles:
        if not isinstance(role, dict):
            continue
        role_id = str(role["id"])
        page_keys = closure.get(role_id, [])
        role_links = "".join(
            link(from_file, str(page_key))
            for page_key in page_keys
            if str(page_key) in PAGE_DEFS
        )
        items.append(
            '<div class="route-closure__item">'
            f'<b>{html.escape(str(role["label"]))}</b>'
            f"<span>{role_links}</span>"
            "</div>"
        )
    return (
        f'<div class="route-closure" aria-label="{html.escape(str(route["title"]))}闭环覆盖">'
        + "".join(items)
        + "</div>"
    )


def make_global_index() -> str:
    from_file = GLOBAL_INDEX
    cpu2_links = "".join(link_pill(from_file, key) for key in CPU2_ORDER)
    cpu3_links = "".join(link_pill(from_file, key) for key in CPU3_ORDER)
    route_cards = []
    for route in ROUTES:
        first_steps = formal_route_steps(route)[:2]
        route_cards.append(
            '<article class="card route-card">'
            f'<strong>{html.escape(route["title"])}</strong>'
            '<span class="badge green">闭环 6/6</span>'
            f'<p>{html.escape(route["summary"])}</p>'
            f'<div class="links"><a class="pill" href="{html.escape(rel_href(from_file, "cross", "#" + str(route["id"])))}">查看链路</a>'
            + "".join(link_pill(from_file, str(step["pageKey"])) for step in first_steps)
            + "</div></article>"
        )
    legacy_cards = []
    for item in LEGACY_DOCS:
        legacy_cards.append(
            '<article class="card">'
            f'<strong>{html.escape(item["title"])}</strong>'
            f'<p><b>{html.escape(item["status"])}</b>：{html.escape(item["note"])}</p>'
            '<div class="links">'
            f'<a class="pill" href="{legacy_link(from_file, item)}">打开原文</a>'
            f'{link_pill(from_file, item["current"], "当前权威流程")}'
            "</div></article>"
        )
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CUBE 程序流程统一入口</title>
<style>
{GLOBAL_CSS}
</style>
<link rel="stylesheet" href="assets/网站嵌入增强.css">
</head>
<body class="cube-flow-page" data-cube-flow-page="cube-flow-index">
<div class="wrap">
<header class="hero">
<h1>CUBE 程序流程统一入口</h1>
<p>把 CPU2 测量执行程序、CPU3 显示/协议程序和跨 CPU 业务链路放到同一个阅读入口。单页仍按源码单独梳理，本页只负责告诉你从哪里进入、往哪里继续看。</p>
<div class="meta-grid">
<div class="meta"><span>覆盖对象</span><strong>CPU2 + CPU3</strong></div>
<div class="meta"><span>阅读方式</span><strong>先业务链路，后单页细节</strong></div>
<div class="meta"><span>页面关系</span><strong>上游 / 本页 / 下游</strong></div>
<div class="meta"><span>维护脚本</span><strong>tools/update_flow_navigation.py</strong></div>
</div>
</header>
<nav class="topnav">
<a href="#path">阅读路径</a>
<a href="#route">跨 CPU 业务链路</a>
<a href="#cpu2">CPU2 页面</a>
<a href="#cpu3">CPU3 页面</a>
<a href="#legacy">历史/专题流程</a>
<a href="#maintain">维护规则</a>
</nav>
<!-- CUBE_EMBED_START -->
<section id="path">
<h2>1. 推荐阅读路径</h2>
<p class="lead">不要从某个 HTML 孤立跳转。先按整机业务链路确定“谁触发、谁执行、谁上报”，再进入 CPU2/CPU3 的详细流程图。</p>
<div class="grid">
<article class="card"><strong>第一步：看整机链路</strong><p>用于判断某个功能跨了 CPU3 菜单、外部协议、CPU2 命令入口和 CPU2 测量核心中的哪些页面。</p><div class="links">{link_pill(from_file, "cross", "跨 CPU 业务链路")}</div></article>
<article class="card"><strong>第二步：看 CPU3 入口</strong><p>如果入口来自按键菜单、DSM、Wartsila、SI 或外部 COM，先看 CPU3 侧如何映射到 CPU2。</p><div class="links">{link_pill(from_file, "cpu3_total", "CPU3 总览")}{link_pill(from_file, "cpu3_07", "菜单入口")}{link_pill(from_file, "cpu3_03", "外部 COM")}</div></article>
<article class="card"><strong>第三步：看 CPU2 执行</strong><p>确认 CPU2 命令入口、测量细节、故障出口、通信寄存器和问题点。</p><div class="links">{link_pill(from_file, "cpu2_total", "CPU2 总览")}{link_pill(from_file, "cpu2_02", "命令入口")}{link_pill(from_file, "cpu2_issue", "问题总表")}</div></article>
</div>
</section>
<section id="route">
<h2>2. 按业务链路跳转</h2>
<p class="lead">这些路线把“显示端/外部协议入口 -> CPU3 内部通信 -> CPU2 测量执行 -> 状态回读”串起来，避免在两个 CPU 的目录之间硬切。</p>
<div class="grid">
{''.join(route_cards)}
</div>
</section>
<section id="cpu2">
<h2>3. CPU2 流程页面</h2>
<p class="lead">CPU2 是测量执行、运动控制、故障管理、参数保存和对 CPU3 寄存器发布的核心。</p>
<div class="links">{link_pill(from_file, "cpu2_total", "CPU2 程序流程总览")}{link_pill(from_file, "cpu2_issue", "CPU2 问题点总表")}{cpu2_links}</div>
</section>
<section id="cpu3">
<h2>4. CPU3 流程页面</h2>
<p class="lead">CPU3 是显示端、外部协议网关、CPU2 Modbus 主站和本机参数维护入口。</p>
<div class="links">{link_pill(from_file, "cpu3_total", "CPU3 程序流程总览")}{cpu3_links}</div>
</section>
<section id="legacy">
<h2>5. 历史/专题流程文档</h2>
<p class="lead">这些页面保留问题分析、需求方案或旧专题梳理价值，但不再作为当前程序流程的唯一入口。阅读时先看“当前权威流程”，再回到原文查背景。</p>
<div class="grid">
{''.join(legacy_cards)}
</div>
</section>
<section id="maintain">
<h2>6. 维护规则</h2>
<div class="grid">
<article class="card"><strong>单页继续按源码单独整理</strong><p>每个功能页仍需要独立阅读源码、单独画业务级 SVG，不用统一模板批量凑图。</p></article>
<article class="card"><strong>跨页只表达业务关系</strong><p>统一入口和跨 CPU 链路页不替代详细流程图，只说明上下游、触发源和结果去向。</p></article>
<article class="card"><strong>重新生成后恢复导航</strong><p>如果 CPU2/CPU3 流程 HTML 被重新生成，执行 <code>py tools/update_flow_navigation.py --write</code> 恢复统一入口和页面关联导航。</p></article>
</div>
</section>
<!-- CUBE_EMBED_END -->
</div>
<script src="assets/{EMBED_SCRIPT_NAME}"></script>
</body>
</html>
"""


def make_cross_route() -> str:
    from_file = CROSS_ROUTE
    route_cards = []
    for route in ROUTES:
        route_cards.append(
            f'<article class="card route-card" id="{html.escape(str(route["id"]))}">'
            f'<h3>{html.escape(route["title"])}</h3>'
            f'<p>{html.escape(route["summary"])}</p>'
            f'{route_closure(from_file, route)}'
            f'<div class="steps">{route_steps(from_file, route)}</div>'
            "</article>"
        )
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>跨 CPU 业务链路导航</title>
<style>
{GLOBAL_CSS}
</style>
<link rel="stylesheet" href="assets/网站嵌入增强.css">
</head>
<body class="cube-flow-page" data-cube-flow-page="cube-flow-跨cpu业务链路">
<div class="wrap">
<header class="hero">
<h1>跨 CPU 业务链路导航</h1>
<p>按整机业务把 CPU3 入口、内部 Modbus、CPU2 命令入口、测量执行、状态回读和显示/协议响应串成连续路线。这里解决“从一个 HTML 跳到另一个 HTML 太生硬”的问题。</p>
<div class="meta-grid">
<div class="meta"><span>入口侧</span><strong>菜单 / 外部协议 / 显示</strong></div>
<div class="meta"><span>通道</span><strong>CPU3 UART5 Modbus</strong></div>
<div class="meta"><span>执行侧</span><strong>CPU2 测量状态机</strong></div>
<div class="meta"><span>闭环契约</span><strong>入口 / 分发 / 执行 / 回读 / 异常 / 验证</strong></div>
</div>
</header>
<nav class="topnav">
<a href="{html.escape(rel_href(from_file, "global"))}">统一入口</a>
<a href="#map">整机闭环图</a>
<a href="#oil">液位</a>
<a href="#water">水位</a>
<a href="#density">密度</a>
<a href="#param">参数</a>
<a href="#fault">故障</a>
<a href="#external">外部协议</a>
</nav>
<!-- CUBE_EMBED_START -->
<section id="map">
<h2>1. 整机业务闭环图</h2>
<p class="lead">图中只写业务动作，不写函数名。函数名和源码证据保留在各自详细页面里。</p>
<figure class="cube-flow-figure"><figcaption id="cube-flow-跨cpu业务链路-diagram-01-caption">CPU2 CPU3 跨 CPU 业务闭环</figcaption><div class="flow-wrap cube-flow__viewport" data-flow-width="standard" tabindex="0" role="region" aria-labelledby="cube-flow-跨cpu业务链路-diagram-01-caption" aria-describedby="cube-flow-跨cpu业务链路-diagram-01-desc">
<svg class="route-svg" viewBox="0 0 1200 760" aria-hidden="true" focusable="false">
<title id="cube-flow-跨cpu业务链路-diagram-01-title">CPU2 CPU3 跨 CPU 业务闭环</title><desc id="cube-flow-跨cpu业务链路-diagram-01-desc">用户或上位机发起的菜单、DSM、Wartsila 或 SI 请求先由 CPU3 完成入口映射和参数检查，再通过内部 Modbus 写入 CPU2 命令或参数；CPU2 通信入口分发命令，结合电机、位置、传感和扭力执行液位、水位、密度等测量。异常进入 SET_ERROR 和状态恢复，正常结果与错误码写入输入寄存器；CPU3 轮询刷新 g_measurement、参数镜像和点表后用于状态页及外部协议响应，并继续下一轮交互。</desc>
<defs>
<marker id="route-arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#61758e"/></marker>
<marker id="route-arrow-red" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#b84a55"/></marker>
</defs>
<rect class="node ext" x="70" y="55" width="230" height="78" rx="10"/><text class="nt" x="185" y="88">用户/上位机</text><text class="ns" x="185" y="111">按键菜单、DSM、Wartsila、SI</text>
<rect class="node cpu3" x="390" y="55" width="260" height="78" rx="10"/><text class="nt" x="520" y="88">CPU3 入口映射</text><text class="ns" x="520" y="111">菜单确认、协议解析、参数检查</text>
<rect class="node cpu3" x="770" y="55" width="260" height="78" rx="10"/><text class="nt" x="900" y="88">CPU3 内部 Modbus 主站</text><text class="ns" x="900" y="111">写 CPU2 命令/参数，轮询输入结果</text>
<rect class="node cpu2" x="770" y="215" width="260" height="82" rx="10"/><text class="nt" x="900" y="250">CPU2 通信寄存器入口</text><text class="ns" x="900" y="273">保持寄存器写入、输入寄存器发布</text>
<rect class="node cpu2" x="390" y="215" width="260" height="82" rx="10"/><text class="nt" x="520" y="250">CPU2 测量命令入口</text><text class="ns" x="520" y="273">公共准备、命令分发、状态切换</text>
<rect class="node cpu2" x="70" y="215" width="230" height="82" rx="10"/><text class="nt" x="185" y="250">CPU2 运动/传感支撑</text><text class="ns" x="185" y="273">电机、位置、传感、扭力</text>
<rect class="node cpu2" x="160" y="395" width="260" height="92" rx="10"/><text class="nt" x="290" y="430">CPU2 测量执行</text><text class="ns" x="290" y="453">液位、水位、密度、回零、输出</text>
<rect class="node error" x="490" y="395" width="230" height="92" rx="10"/><text class="nt" x="605" y="430">故障/异常出口</text><text class="ns" x="605" y="453">SET_ERROR、命令切换、状态恢复</text>
<rect class="node state" x="800" y="395" width="250" height="92" rx="10"/><text class="nt" x="925" y="430">CPU2 状态发布</text><text class="ns" x="925" y="453">测量结果、设备状态、错误码</text>
<rect class="node cpu3" x="800" y="575" width="250" height="82" rx="10"/><text class="nt" x="925" y="610">CPU3 缓存刷新</text><text class="ns" x="925" y="633">g_measurement、参数镜像、点表</text>
<rect class="node cpu3" x="390" y="575" width="260" height="82" rx="10"/><text class="nt" x="520" y="610">显示和外部协议响应</text><text class="ns" x="520" y="633">状态页刷新、DSM/Wartsila/SI 读出</text>
<path class="edge" d="M300 94 L390 94" marker-end="url(#route-arrow)"/>
<path class="edge" d="M650 94 L770 94" marker-end="url(#route-arrow)"/>
<path class="edge" d="M900 133 L900 215" marker-end="url(#route-arrow)"/>
<path class="edge" d="M770 256 L650 256" marker-end="url(#route-arrow)"/>
<path class="edge" d="M390 256 L300 256" marker-end="url(#route-arrow)"/>
<path class="edge" d="M185 297 C195 342 235 365 290 395" marker-end="url(#route-arrow)"/>
<path class="edge" d="M520 297 C450 335 360 360 290 395" marker-end="url(#route-arrow)"/>
<path class="edge-red" d="M420 441 L490 441" marker-end="url(#route-arrow-red)"/>
<path class="edge" d="M720 441 L800 441" marker-end="url(#route-arrow)"/>
<path class="edge" d="M925 487 L925 575" marker-end="url(#route-arrow)"/>
<path class="edge" d="M800 616 L650 616" marker-end="url(#route-arrow)"/>
<path class="edge" d="M520 575 C470 505 370 485 290 487" marker-end="url(#route-arrow)"/>
<path class="edge" d="M900 575 C1010 505 1030 300 1030 133" marker-end="url(#route-arrow)"/>
</svg>
</div></figure>
</section>
<section>
<h2>2. 分业务路线</h2>
<p class="lead">每条路线都按“入口 -> 通道 -> CPU2 执行 -> 回读/显示”展开。点击步骤可直接进入对应的详细流程图页面。</p>
<div class="grid">
{''.join(route_cards)}
</div>
</section>
<section>
<h2>3. 使用建议</h2>
<div class="grid">
<article class="card"><strong>查一个命令为什么没执行</strong><p>先看 CPU3 是否把命令写入内部 Modbus，再看 CPU2 命令入口是否接收，最后看对应测量页的异常出口。</p></article>
<article class="card"><strong>查结果为什么没显示</strong><p>先看 CPU2 是否发布输入寄存器，再看 CPU3 内部轮询是否刷新缓存，最后看显示页或外部协议页。</p></article>
<article class="card"><strong>查参数为什么没生效</strong><p>区分 CPU3 本机参数、CPU2 设备参数和协议缓存，按参数链路逐页核对。</p></article>
</div>
</section>
<!-- CUBE_EMBED_END -->
</div>
<script src="assets/{EMBED_SCRIPT_NAME}"></script>
</body>
</html>
"""


def update_root_readme(plan: InMemoryUpdatePlan) -> None:
    path = ROOT / "docs" / "README.md"
    if not plan.exists(path):
        return
    text = plan.read_text(path)
    if "`00_程序流程导航`" not in text:
        text = text.replace(
            "| `00_构建与版本` | CPU2/CPU3 构建说明、升级日志、版本改动与测试方案等版本交付资料 |",
            "| `00_程序流程导航` | CPU2/CPU3 程序流程统一入口、跨 CPU 业务链路和流程文档跳转导航 |\n"
            "| `00_构建与版本` | CPU2/CPU3 构建说明、升级日志、版本改动与测试方案等版本交付资料 |",
            1,
        )
    if "`00_程序流程导航/index.html`" not in text:
        marker = "| 文档 | 用途 |\n| --- | --- |\n"
        entry = (
            "| `00_程序流程导航/index.html` | CPU2/CPU3 程序流程统一入口，按整机业务链路跳转到 CPU2 和 CPU3 详细流程页 |\n"
            "| `00_程序流程导航/跨CPU业务链路.html` | 液位、水位、密度、参数、故障和外部协议的跨 CPU 路线图 |\n"
        )
        text = text.replace(marker, marker + entry, 1)
    plan.stage_text(path, text)


def generate_pages(plan: InMemoryUpdatePlan) -> None:
    plan.stage_text(
        GLOBAL_INDEX,
        normalize_duplicate_svg_references(make_global_index(), GLOBAL_INDEX),
    )
    plan.stage_text(
        CROSS_ROUTE,
        normalize_duplicate_svg_references(make_cross_route(), CROSS_ROUTE),
    )


def inject_all_pages(plan: InMemoryUpdatePlan) -> None:
    keys = ["cpu2_total", "cpu2_issue", *CPU2_ORDER, "cpu3_total", *CPU3_ORDER]
    for key in keys:
        inject_page(plan, key)
    inject_legacy_notices(plan)


def validate_files(plan: InMemoryUpdatePlan) -> None:
    governance = analyze_flow_governance()
    blocking_issues = [
        issue
        for issue in governance["issues"]  # type: ignore[index]
        if isinstance(issue, dict) and issue.get("rule") != "stale-review"
    ]
    if blocking_issues:
        raise RuntimeError(
            "Flow governance checks failed: "
            + "; ".join(str(issue.get("message")) for issue in blocking_issues)
        )
    missing = [
        key
        for key, meta in PAGE_DEFS.items()
        if key not in {"global", "cross"} and not plan.exists(Path(meta["path"]))
    ]
    if missing:
        raise RuntimeError("Missing expected pages: " + ", ".join(missing))
    bad_encoding = []
    for path in [GLOBAL_INDEX, CROSS_ROUTE, *[page_path(key) for key in ["cpu2_total", "cpu2_issue", *CPU2_ORDER, "cpu3_total", *CPU3_ORDER]]]:
        text = plan.read_text(path)
        if "\ufffd" in text or "锟" in text:
            bad_encoding.append(str(path))
    if bad_encoding:
        raise RuntimeError("Encoding replacement characters found: " + ", ".join(bad_encoding))
    missing_nav = []
    for key in ["cpu2_total", "cpu2_issue", *CPU2_ORDER, "cpu3_total", *CPU3_ORDER]:
        text = plan.read_text(page_path(key))
        if NAV_START not in text:
            missing_nav.append(key)
    if missing_nav:
        raise RuntimeError("Missing cross-flow navigation block: " + ", ".join(missing_nav))
    missing_evidence = []
    for key in EVIDENCE_PAGES_BY_KEY:
        text = plan.read_text(page_path(key))
        if f'data-evidence-page="{key}"' not in text:
            missing_evidence.append(key)
    if missing_evidence:
        raise RuntimeError("Missing generated evidence head: " + ", ".join(missing_evidence))


def build_update_plan() -> InMemoryUpdatePlan:
    plan = InMemoryUpdatePlan()
    generate_pages(plan)
    inject_all_pages(plan)
    normalize_all_flow_documents(plan)
    update_root_readme(plan)
    validate_files(plan)
    return plan


def _display_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check or update cross-document flow navigation."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument(
        "--check",
        action="store_true",
        help="check whether generated navigation is current (default)",
    )
    mode.add_argument(
        "--write",
        action="store_true",
        help="write validated changes using atomic file replacement",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    plan = build_update_plan()
    changed_paths = plan.changed_paths
    if not changed_paths:
        print("flow navigation is up to date")
        return 0

    if not args.write:
        print(f"flow navigation needs update: {len(changed_paths)} file(s)")
        for path in changed_paths:
            print(f"  {_display_path(path)}")
        print("run with --write to apply the validated update")
        return 1

    written_paths = plan.apply()
    print(f"updated flow navigation: {len(written_paths)} file(s)")
    for path in written_paths:
        print(f"  {_display_path(path)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

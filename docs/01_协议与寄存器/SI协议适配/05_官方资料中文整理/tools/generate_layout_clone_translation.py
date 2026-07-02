#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Generate a Chinese layout-clone PDF for the SI-7000 official package.

The original PDF is only available as a PDF package, not as an editable source
document. This generator therefore uses the rendered original page as the
background, removes detected English text areas, and draws translated Chinese
text back into the original positions.
"""

from __future__ import annotations

import argparse
import json
import re
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pdfplumber
from PIL import Image
from reportlab.lib.colors import black, white
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.pdfgen import canvas


ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[3]
SOURCE_PDF = REPO / "docs/01_协议与寄存器/SI协议适配/00_原始资料/SI7000官方资料包_含Modbus规范.pdf"
ASSETS = ROOT / "assets"
OUT_PDF = ROOT / "SI7000官方资料包_中文全文翻译_版式复刻.pdf"
OUT_MD = ROOT / "SI7000官方资料包_中文全文翻译_版式复刻源稿.md"
CACHE_JSON = ROOT / "tools/translation_cache.json"
FONT_PATH = Path(r"C:\Windows\Fonts\simhei.ttf")
FONT_NAME = "SimHeiLocal"

SCAN_MANUAL_PAGES = {7, 8, 9, 101}

TERM_MAP = [
    "Scientific Instruments",
    "SI-7000",
    "Modbus",
    "DCS",
    "LTD",
    "RS485",
    "RS232",
    "RTU",
    "Profile",
    "Auto",
    "Manual",
    "Cal",
    "Pinch Valve",
    "Tank ID",
    "Device ID",
    "TGIM",
    "COM",
    "PWR",
    "LCD",
    "LED",
    "MESG",
    "LNG",
    "DVM",
    "IECEx",
    "IECEX",
    "Configuration Worksheet",
    "Bottom Reference",
    "Zoom Scan",
    "Lock-out Tag-out",
]

POST_REPLACEMENTS = [
    ("配置文件", "Profile"),
    ("档案", "Profile"),
    ("轮廓", "Profile"),
    ("坦克", "储罐"),
    ("油箱", "储罐"),
    ("探测器", "探头"),
    ("探针", "探头"),
    ("密度仪", "密度计"),
    ("电机", "电机"),
    ("函数 ", "功能码 "),
    ("功能 ", "功能码 "),
    ("修订版", "Rev"),
    ("华氏度", "deg F"),
    ("摄氏度", "deg C"),
]

CHAR_REPLACEMENTS = {
    "•": "·",
    "≥": ">=",
    "≤": "<=",
    "℃": "deg C",
    "℉": "deg F",
    "–": "-",
    "—": "-",
}


@dataclass
class WordBox:
    text: str
    x0: float
    x1: float
    top: float
    bottom: float
    size: float


@dataclass
class Segment:
    page: int
    text: str
    x0: float
    x1: float
    top: float
    bottom: float
    size: float
    word_boxes: list[WordBox]
    avail_x1: float = 0.0
    translated: str = ""


def load_cache() -> dict[str, str]:
    if CACHE_JSON.exists():
        return json.loads(CACHE_JSON.read_text(encoding="utf-8"))
    return {}


def save_cache(cache: dict[str, str]) -> None:
    CACHE_JSON.parent.mkdir(parents=True, exist_ok=True)
    CACHE_JSON.write_text(
        json.dumps(cache, ensure_ascii=False, indent=2, sort_keys=True),
        encoding="utf-8",
    )


def protect_terms(text: str) -> tuple[str, dict[str, str]]:
    replacements: dict[str, str] = {}
    protected = text
    for idx, term in enumerate(sorted(TERM_MAP, key=len, reverse=True)):
        token = f"<<<T{idx}>>>"
        if term in protected:
            protected = protected.replace(term, token)
            replacements[token] = term
    return protected, replacements


def restore_terms(text: str, replacements: dict[str, str]) -> str:
    restored = text
    for token, term in replacements.items():
        restored = restored.replace(token, term)
    for old, new in POST_REPLACEMENTS:
        restored = restored.replace(old, new)
    restored = re.sub(r"功能码码\s*", "功能码 ", restored)
    restored = re.sub(r"\s+", " ", restored).strip()
    restored = restored.replace(" :", "：").replace(" ;", "；")
    return normalize_draw_text(restored)


def normalize_draw_text(text: str) -> str:
    normalized = text
    for old, new in CHAR_REPLACEMENTS.items():
        normalized = normalized.replace(old, new)
    return normalized


def should_translate(text: str) -> bool:
    stripped = text.strip()
    if not stripped:
        return False
    if re.search(r"\bRev\b", stripped) and re.search(r"\d", stripped):
        return False
    if re.fullmatch(r"[\d\s.,:/\\\-+()[\]#°%*='\"<>]+", stripped):
        return False
    if "@" in stripped or stripped.lower().startswith(("www.", "http")):
        return False
    if re.fullmatch(r"(?:[A-Z]{1,5}[-/]?)*\d+(?:[-/][A-Z0-9.]+)*", stripped):
        return False
    return bool(re.search(r"[A-Za-z]", stripped))


def local_translation(text: str) -> str | None:
    stripped = text.strip()
    page_match = re.fullmatch(r"Page\s+(\d+)", stripped, re.I)
    if page_match:
        return f"第 {page_match.group(1)} 页"
    direct = {
        "Table of Contents": "目录",
        "REVISION HISTORY": "修订记录",
        "Parameter": "参数",
        "Address": "地址",
        "Activity": "活动",
        "ACTIVITY": "活动",
        "Remarks": "备注",
        "REMARKS": "备注",
        "Warning": "警告",
        "WARNING": "警告",
        "Notice": "通知",
        "NOTICE": "通知",
        "Introduction": "引言",
        "General Description": "总体说明",
        "Definition": "定义",
        "Addressing": "寻址",
        "Scaling": "缩放",
        "Hardware Specifications": "硬件规格",
        "Programming Suggestions": "编程建议",
        "Service Manual": "维修手册",
        "Operational Manual": "操作手册",
        "Installation &": "安装与",
        "Commissioning Manual": "调试手册",
        "DCS Modbus": "DCS Modbus",
        "Interface Specification": "接口规范",
        "Tank Gauging System": "储罐计量系统",
        "SCIENTIFIC INSTRUMENTS, INC.": "SCIENTIFIC INSTRUMENTS, INC.",
    }
    return direct.get(stripped)


def translate_many(texts: Iterable[str], cache: dict[str, str]) -> dict[str, str]:
    unique = []
    for text in texts:
        if text not in cache and should_translate(text):
            unique.append(text)

    if not unique:
        return cache

    print(f"Translating {len(unique)} unique text segments...")
    chunk: list[str] = []
    chunk_len = 0
    translated_count = 0

    def flush(items: list[str]) -> None:
        nonlocal translated_count
        if not items:
            return
        marker_to_text: dict[int, str] = {}
        marker_to_restore: dict[int, dict[str, str]] = {}
        lines = []
        for idx, item in enumerate(items):
            protected, restore = protect_terms(item)
            marker_to_text[idx] = item
            marker_to_restore[idx] = restore
            lines.append(f"<<<{idx}>>> {protected}")
        query = "\n".join(lines)
        translated = request_translate(query)
        parsed = parse_marker_translation(translated)
        if len(parsed) != len(items):
            for idx, item in enumerate(items):
                protected, restore = protect_terms(item)
                single = request_translate(protected)
                cache[item] = restore_terms(single, restore)
                translated_count += 1
                time.sleep(0.08)
        else:
            for idx, item in marker_to_text.items():
                cache[item] = restore_terms(parsed[idx], marker_to_restore[idx])
                translated_count += 1
        save_cache(cache)
        print(f"  cached {translated_count}/{len(unique)}")
        time.sleep(0.15)

    for item in unique:
        local = local_translation(item)
        if local is not None:
            cache[item] = local
            translated_count += 1
            continue
        added = len(item) + 16
        if chunk and chunk_len + added > 2600:
            flush(chunk)
            chunk = []
            chunk_len = 0
        chunk.append(item)
        chunk_len += added
    flush(chunk)
    save_cache(cache)
    return cache


def request_translate(text: str) -> str:
    params = urllib.parse.urlencode(
        {"client": "gtx", "sl": "en", "tl": "zh-CN", "dt": "t", "q": text}
    )
    url = "https://translate.googleapis.com/translate_a/single?" + params
    last_error: Exception | None = None
    for attempt in range(4):
        try:
            with urllib.request.urlopen(url, timeout=30) as response:
                data = response.read().decode("utf-8")
            parsed = json.loads(data)
            return "".join(part[0] for part in parsed[0] if part and part[0])
        except Exception as exc:  # pragma: no cover - network fallback
            last_error = exc
            time.sleep(1.0 + attempt)
    raise RuntimeError(f"translation request failed: {last_error}")


def parse_marker_translation(text: str) -> dict[int, str]:
    result: dict[int, str] = {}
    pattern = re.compile(r"<<<(\d+)>>>\s*(.*?)(?=(?:<<<\d+>>>|$))", re.S)
    for match in pattern.finditer(text):
        result[int(match.group(1))] = match.group(2).strip()
    return result


def group_words(words: list[dict], page_no: int, page_width: float) -> list[Segment]:
    boxes = [
        WordBox(
            text=w["text"],
            x0=float(w["x0"]),
            x1=float(w["x1"]),
            top=float(w["top"]),
            bottom=float(w["bottom"]),
            size=float(w.get("size", max(6.0, float(w["bottom"]) - float(w["top"])))),
        )
        for w in words
        if w.get("text", "").strip()
    ]
    boxes.sort(key=lambda w: (w.top, w.x0))

    lines: list[list[WordBox]] = []
    for box in boxes:
        if not lines:
            lines.append([box])
            continue
        current = lines[-1]
        current_mid = sum((w.top + w.bottom) / 2 for w in current) / len(current)
        box_mid = (box.top + box.bottom) / 2
        tolerance = max(3.0, box.size * 0.35)
        if abs(box_mid - current_mid) <= tolerance:
            current.append(box)
        else:
            lines.append([box])

    segments: list[Segment] = []
    for line in lines:
        line.sort(key=lambda w: w.x0)
        parts: list[list[WordBox]] = [[]]
        for idx, word in enumerate(line):
            if idx == 0:
                parts[-1].append(word)
                continue
            prev = line[idx - 1]
            gap = word.x0 - prev.x1
            threshold = max(18.0, min(42.0, word.size * 2.8))
            if gap > threshold:
                parts.append([word])
            parts[-1].append(word)

        row_segments: list[Segment] = []
        for part in parts:
            text = " ".join(w.text for w in part).strip()
            if not text:
                continue
            x0 = min(w.x0 for w in part)
            x1 = max(w.x1 for w in part)
            top = min(w.top for w in part)
            bottom = max(w.bottom for w in part)
            size = sum(w.size for w in part) / len(part)
            row_segments.append(Segment(page_no, text, x0, x1, top, bottom, size, part))

        for idx, seg in enumerate(row_segments):
            if idx + 1 < len(row_segments):
                seg.avail_x1 = row_segments[idx + 1].x0 - 2
            else:
                width = seg.x1 - seg.x0
                right_margin = 36 if seg.x0 < page_width - 220 else 16
                if len(row_segments) == 1 and width < 170:
                    seg.avail_x1 = min(seg.x0 + max(170, width + 60), page_width - right_margin)
                else:
                    seg.avail_x1 = min(max(seg.x1 + 10, seg.x0 + width), page_width - right_margin)
            segments.append(seg)
    return segments


def get_page_image(page_no: int) -> Path:
    matches = sorted(ASSETS.glob(f"p{page_no:03d}_*.png"))
    if not matches:
        raise FileNotFoundError(f"missing rendered image for page {page_no}")
    return matches[0]


def page_image_size_points(image_path: Path, page_width: float, page_height: float) -> tuple[float, float]:
    with Image.open(image_path) as img:
        return page_width, page_height


def cover_words(c: canvas.Canvas, page_height: float, words: list[WordBox]) -> None:
    c.setFillColor(white)
    for word in words:
        pad_x = max(1.6, word.size * 0.16)
        pad_y = max(1.4, word.size * 0.22)
        x = word.x0 - pad_x
        y = page_height - word.bottom - pad_y
        w = max(1.0, word.x1 - word.x0 + pad_x * 2)
        h = max(1.0, word.bottom - word.top + pad_y * 2)
        c.rect(x, y, w, h, stroke=0, fill=1)


def draw_fit_text(
    c: canvas.Canvas,
    text: str,
    x: float,
    top: float,
    bottom: float,
    width: float,
    page_height: float,
    base_size: float,
    bold: bool = False,
) -> None:
    if not text:
        return
    text = normalize_draw_text(text)
    c.setFillColor(black)
    font_size = max(3.6, min(13.5, base_size * 0.92))
    if base_size > 15:
        font_size = min(24, base_size * 0.9)
    if bold:
        font_size *= 1.04
    text_width = pdfmetrics.stringWidth(text, FONT_NAME, font_size)
    min_font = 3.7 if base_size < 12 else 5.0
    while width > 4 and text_width > width and font_size > min_font:
        font_size *= 0.92
        text_width = pdfmetrics.stringWidth(text, FONT_NAME, font_size)
    scale = 1.0
    if width > 4 and text_width > width:
        scale = max(0.55, width / text_width)
    baseline = page_height - bottom + max(0.6, (bottom - top - font_size) / 2)
    c.saveState()
    c.translate(x, baseline)
    c.scale(scale, 1.0)
    c.setFont(FONT_NAME, font_size)
    c.drawString(0, 0, text)
    c.restoreState()


def draw_wrapped_text(
    c: canvas.Canvas,
    text: str,
    x: float,
    top: float,
    width: float,
    page_height: float,
    font_size: float,
    leading: float | None = None,
) -> None:
    text = normalize_draw_text(text)
    if leading is None:
        leading = font_size * 1.35
    c.setFont(FONT_NAME, font_size)
    c.setFillColor(black)
    lines: list[str] = []
    for paragraph in text.splitlines():
        if not paragraph:
            lines.append("")
            continue
        current = ""
        for char in paragraph:
            trial = current + char
            if current and pdfmetrics.stringWidth(trial, FONT_NAME, font_size) > width:
                lines.append(current)
                current = char
            else:
                current = trial
        if current:
            lines.append(current)
    y = page_height - top - font_size
    for line in lines:
        if line:
            c.drawString(x, y, line)
        y -= leading


def draw_manual_scan_page(c: canvas.Canvas, page_no: int, page_width: float, page_height: float) -> list[str]:
    entries = MANUAL_SCAN_TRANSLATIONS[page_no]
    rendered_lines: list[str] = []
    for entry in entries:
        x, top, w, h = entry["box"]
        text = entry["text"]
        c.setFillColor(white)
        c.rect(x, page_height - top - h, w, h, stroke=0, fill=1)
        draw_wrapped_text(
            c,
            text,
            x + 2,
            top + 2,
            w - 4,
            page_height,
            float(entry.get("font_size", 8.5)),
            float(entry.get("leading", entry.get("font_size", 8.5) * 1.35)),
        )
        rendered_lines.append(text)
    return rendered_lines


def draw_page_edges(c: canvas.Canvas, page: pdfplumber.page.Page) -> None:
    c.setStrokeColor(black)
    for edge in page.edges:
        width = float(edge.get("linewidth") or 0.5)
        if width <= 0:
            width = 0.35
        c.setLineWidth(min(max(width, 0.25), 1.2))
        x0 = float(edge["x0"])
        x1 = float(edge["x1"])
        y0 = float(page.height - edge["top"])
        y1 = float(page.height - edge["bottom"])
        c.line(x0, y0, x1, y1)


MANUAL_SCAN_TRANSLATIONS = {
    7: [
        {
            "box": (250, 45, 245, 75),
            "font_size": 18,
            "leading": 25,
            "text": "IECEx 符合性证书\n国际电工委员会\nIEC 防爆环境认证体系",
        },
        {
            "box": (58, 178, 470, 245),
            "font_size": 8.5,
            "text": (
                "证书编号：IECEx SEV 19.0035X    签发号：0    证书历史：Issue No. 0 (2019-09-11)\n"
                "状态：Current    签发日期：2019-09-11    第 1 页，共 3 页\n"
                "申请人：Scientific Instruments，4400 West Tiffany Drive，West Palm Beach，FL 33407，United States of America\n"
                "设备：Tank Gauging System\n"
                "保护型式：d, I, h\n"
                "标志：电气舱 Ex db [ia IIC Ga] IIB T4 Gb；机械舱 Ex h IIC T4 Ga；探头组件 Ex ia IIC T4 Ga"
            ),
        },
        {
            "box": (58, 448, 475, 135),
            "font_size": 8,
            "text": (
                "代表 IECEx 认证机构批准签发：Martin Plüss\n"
                "职位：产品认证经理\n"
                "签名：见原签名图像\n"
                "日期：2019-09-11"
            ),
        },
        {
            "box": (58, 590, 480, 90),
            "font_size": 7.6,
            "text": (
                "1. 本证书及其附表只能完整复制。\n"
                "2. 本证书不可转让，并仍归签发机构所有。\n"
                "3. 本证书的状态和真实性可通过 IECEx 官方网站验证。\n"
                "证书签发机构：Eurofins Electric & Electronic Product Testing AG，Fehraltorf，Switzerland。"
            ),
        },
    ],
    8: [
        {
            "box": (250, 45, 245, 70),
            "font_size": 18,
            "leading": 25,
            "text": "IECEx 符合性证书",
        },
        {
            "box": (58, 122, 470, 160),
            "font_size": 8.2,
            "text": (
                "证书编号：IECEx SEV 19.0035X    签发号：0    第 2 页，共 3 页\n"
                "签发日期：2019-09-11\n"
                "制造商：Scientific Instruments，4400 West Tiffany Drive，West Palm Beach，FL 33407，United States of America\n"
                "其他制造地点：无。\n"
                "本证书用于确认生产代表样品已评估和测试，符合下列 IEC 标准；制造商与本证书所覆盖 Ex 产品相关的质量体系也已评估并符合 IECEx 质量体系要求。"
            ),
        },
        {
            "box": (58, 308, 480, 260),
            "font_size": 7.7,
            "text": (
                "标准：\n"
                "IEC 60079-0:2011 Edition 6.0：爆炸性环境 - 第 0 部分：通用要求。\n"
                "IEC 60079-1:2014-06 Edition 7.0：爆炸性环境 - 第 1 部分：隔爆外壳 d 设备保护。\n"
                "IEC 60079-11:2011 Edition 6.0：爆炸性环境 - 第 11 部分：本质安全 i 设备保护。\n"
                "ISO 80079-36:2016 Edition 1.0：爆炸性环境用非电气设备 - 基本方法和要求。\n"
                "ISO 80079-37:2016 Edition 1.0：爆炸性环境用非电气设备 - 非电气保护型式，包括结构安全 c、点燃源控制 b、液浸 k。\n"
                "本证书不表示符合上述标准明确列出内容之外的电气安全和性能要求。"
            ),
        },
        {
            "box": (58, 588, 360, 88),
            "font_size": 7.8,
            "text": (
                "测试与评估报告：所列设备样品已按记录成功满足检验和测试要求。\n"
                "测试报告：CH/SEV/EXTR19.0036/00\n"
                "质量评估报告：GB/BAS/QAR14.0004/04"
            ),
        },
    ],
    9: [
        {
            "box": (250, 45, 245, 70),
            "font_size": 18,
            "leading": 25,
            "text": "IECEx 符合性证书",
        },
        {
            "box": (58, 122, 470, 90),
            "font_size": 8.3,
            "text": (
                "证书编号：IECEx SEV 19.0035X    签发号：0    第 3 页，共 3 页\n"
                "签发日期：2019-09-11\n"
                "附表"
            ),
        },
        {
            "box": (58, 228, 480, 220),
            "font_size": 8.2,
            "text": (
                "设备：\n"
                "本证书覆盖的设备和系统如下：Tank Gauging System。\n"
                "型号：SI-7000。\n"
                "SI-7000 Tank Gauging System 是一种过程监测仪表，可监测深度最高 200 ft 储罐内的液体介质，包括 LNG 和其他燃料。\n"
                "其他信息见 Annexe。"
            ),
        },
        {
            "box": (58, 455, 430, 110),
            "font_size": 8.2,
            "text": (
                "特殊使用条件：YES，如下所示。\n"
                "Specific Conditions of Use / Schedule of Limitations：\n"
                "隔爆接合面不应修理。\n"
                "Annex：IECEx SEV 19.0035 Annexe Issue 0.pdf"
            ),
        },
    ],
    101: [
        {
            "box": (190, 145, 245, 42),
            "font_size": 17,
            "text": "系统保修政策",
        },
        {
            "box": (48, 205, 520, 430),
            "font_size": 8.1,
            "leading": 11.0,
            "text": (
                "Scientific Instruments, Inc. (SII) 对其 LTD Tank Gauging System 的材料和工艺缺陷提供保修；保修期为货物从 SII 位于 4400 West Tiffany Drive, West Palm Beach, Florida 的装运码头发出之日起 24 个月。\n"
                "SII 不对买方指定的原材料、工作方法设计，或买方设备与 SII 产品不兼容造成的损害负责；也不对合同方或第三方疏忽造成的损害负责；不对将 SII 产品用于非预期用途造成的损害负责。\n"
                "作为 SII 系统一部分但非 SII 制造的子组件，例如软件、打印机、终端、调制解调器等，按产品供应商延伸的销售条款保修。\n"
                "SII 不对间接损失、纯经济损失、利润损失或其他后果性经济损失负责。\n"
                "如果缺陷在保修期内显现，SII 或经过适当培训和认证的人员将在客户现场执行现场维修。所需现场服务人员的交通费和生活费由客户按成本承担。\n"
                "对于可实物返回 SII 的故障子组件或部件，在客户承担返运产生的所有运费、外国关税等费用的前提下，所有保修维修或更换免费完成。产品进入美国时产生的美国关税或费用由 SII 支付。\n"
                "如果系统安装和调试不是由 SII 人员执行，或不是由 SII 正式培训和认证的人员执行，则本保修政策视为无效。独立承包商或顾问成功完成培训后，SII 会签发培训证书。"
            ),
        },
        {
            "box": (48, 650, 420, 68),
            "font_size": 8.3,
            "text": (
                "授权签署：Leigh Ann Hoey\n"
                "姓名/职务：Leigh Ann Hoey / President\n"
                "CSF0030 Rev. -    7/01/18"
            ),
        },
    ],
}


def build_segments(pages: set[int] | None) -> tuple[list[Segment], dict[int, tuple[float, float]]]:
    all_segments: list[Segment] = []
    page_sizes: dict[int, tuple[float, float]] = {}
    with pdfplumber.open(SOURCE_PDF) as pdf:
        for page_no, page in enumerate(pdf.pages, 1):
            if pages is not None and page_no not in pages:
                continue
            page_sizes[page_no] = (float(page.width), float(page.height))
            if page_no in SCAN_MANUAL_PAGES:
                continue
            words = page.extract_words(
                x_tolerance=1.2,
                y_tolerance=3,
                keep_blank_chars=False,
                use_text_flow=False,
                extra_attrs=["size"],
            )
            all_segments.extend(group_words(words, page_no, float(page.width)))
    return all_segments, page_sizes


def generate_pdf(segments: list[Segment], page_sizes: dict[int, tuple[float, float]], pages: set[int] | None) -> None:
    pdfmetrics.registerFont(TTFont(FONT_NAME, str(FONT_PATH)))
    seg_by_page: dict[int, list[Segment]] = {}
    for seg in segments:
        seg_by_page.setdefault(seg.page, []).append(seg)

    output = OUT_PDF if pages is None else ROOT / "tmp_layout_clone_sample.pdf"
    c = canvas.Canvas(str(output), pageCompression=1)
    rendered_source: list[str] = ["# SI-7000 官方资料包中文全文翻译（版式复刻源稿）", ""]

    with pdfplumber.open(SOURCE_PDF) as pdf:
        page_range = range(1, len(pdf.pages) + 1)
        for page_no in page_range:
            if pages is not None and page_no not in pages:
                continue
            width, height = float(pdf.pages[page_no - 1].width), float(pdf.pages[page_no - 1].height)
            c.setPageSize((width, height))
            image_path = get_page_image(page_no)
            page_image_size_points(image_path, width, height)
            c.drawImage(str(image_path), 0, 0, width=width, height=height, preserveAspectRatio=False, mask="auto")

            rendered_source.append(f"## PDF 第 {page_no} 页")
            rendered_source.append("")
            if page_no in SCAN_MANUAL_PAGES:
                manual_lines = draw_manual_scan_page(c, page_no, width, height)
                rendered_source.extend(manual_lines)
                rendered_source.append("")
                c.showPage()
                continue

            page_segments = [
                seg
                for seg in seg_by_page.get(page_no, [])
                if should_translate(seg.text)
                and (seg.translated or seg.text).strip() != seg.text.strip()
            ]
            for seg in page_segments:
                cover_words(c, height, seg.word_boxes)
            draw_page_edges(c, pdf.pages[page_no - 1])
            for seg in page_segments:
                translated = seg.translated or seg.text
                available_width = max(12.0, seg.avail_x1 - seg.x0)
                is_bold = seg.text.isupper() or seg.size >= 13
                draw_fit_text(
                    c,
                    translated,
                    seg.x0,
                    seg.top,
                    seg.bottom,
                    available_width,
                    height,
                    seg.size,
                    bold=is_bold,
                )
                rendered_source.append(f"- {seg.text} => {translated}")
            rendered_source.append("")
            c.showPage()
    c.save()
    if pages is None:
        OUT_MD.write_text("\n".join(rendered_source), encoding="utf-8")
    print(f"Generated {output}")


def parse_pages(raw: str | None) -> set[int] | None:
    if not raw:
        return None
    pages: set[int] = set()
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start, end = part.split("-", 1)
            pages.update(range(int(start), int(end) + 1))
        else:
            pages.add(int(part))
    return pages


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pages", help="Optional comma/range page list, e.g. 1,7,10-12")
    parser.add_argument("--no-translate", action="store_true", help="Use existing cache only")
    args = parser.parse_args()

    pages = parse_pages(args.pages)
    if not SOURCE_PDF.exists():
        raise FileNotFoundError(SOURCE_PDF)
    if not FONT_PATH.exists():
        raise FileNotFoundError(FONT_PATH)

    segments, page_sizes = build_segments(pages)
    cache = load_cache()
    text_items = [seg.text for seg in segments if should_translate(seg.text)]
    if not args.no_translate:
        cache = translate_many(text_items, cache)
    for seg in segments:
        seg.translated = cache.get(seg.text, local_translation(seg.text) or seg.text)
    generate_pdf(segments, page_sizes, pages)


if __name__ == "__main__":
    main()

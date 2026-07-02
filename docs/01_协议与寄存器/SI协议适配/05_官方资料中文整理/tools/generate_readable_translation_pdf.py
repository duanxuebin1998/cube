# -*- coding: utf-8 -*-
"""Generate the readable Chinese translation PDF for the SI-7000 document pack."""

from __future__ import annotations

import html
import io
import re
import tempfile
from pathlib import Path

from PIL import Image as PilImage
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    Image,
    LongTable,
    PageBreak,
    Paragraph,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)


ROOT = Path(__file__).resolve().parents[1]
ASSET_DIR = ROOT / "assets"
OUTPUT_MD = ROOT / "SI7000官方资料包_中文全文翻译_优化排版源稿.md"
OUTPUT_PDF = ROOT / "SI7000官方资料包_中文全文翻译_优化排版_无附录.pdf"
SOURCE_PDF = "../00_原始资料/SI7000官方资料包_含Modbus规范.pdf"

INPUT_DOCS = [
    ROOT / "00_资料包目录与整理规则.md",
    ROOT / "01_DCS_Modbus接口规范_中文整理.md",
    ROOT / "02_产品规格认证与包装_中文整理.md",
    ROOT / "03_机械布置与现场接线_中文整理.md",
    ROOT / "04_安装与调试手册_中文整理.md",
    ROOT / "05_操作手册_中文整理.md",
    ROOT / "06_维修手册与保修信息_中文整理.md",
]

SKIP_SECTION_TITLES = {
    "整理目标",
    "整理说明",
    "资料包页码结构",
    "原始页面索引",
    "与现有项目文档的关系",
    "图片和图纸处理规则",
    "分批计划",
    "当前已知整理备注",
}

DROP_LINE_MARKERS = (
    "当前 CUBE",
    "当前项目",
    "本项目当前",
    "CUBE 项目",
    "CUBE 的",
    "CUBE 适配",
    "源码和联调记录",
    "../02_协议映射",
)


def register_font() -> str:
    font_path = Path("C:/Windows/Fonts/simhei.ttf")
    if not font_path.exists():
        raise FileNotFoundError("Cannot find Chinese font: C:/Windows/Fonts/simhei.ttf")
    pdfmetrics.registerFont(TTFont("SimHei", str(font_path)))
    return "SimHei"


def heading_level_and_title(line: str) -> tuple[int, str] | None:
    match = re.match(r"^(#{1,6})\s+(.+?)\s*$", line)
    if not match:
        return None
    return len(match.group(1)), match.group(2).strip()


def normalized_heading_title(title: str) -> str:
    title = re.sub(r"^\d+(?:\.\d+)*\.?\s*", "", title.strip())
    title = title.replace("中文整理", "中文翻译")
    title = title.replace("中文译文", "中文翻译")
    return title.strip()


def should_skip_heading(title: str) -> bool:
    normalized = normalized_heading_title(title)
    if normalized in SKIP_SECTION_TITLES:
        return True
    if "对 CUBE" in normalized or "CUBE 适配" in normalized:
        return True
    return False


def clean_source_line(line: str) -> str | None:
    if any(marker in line for marker in DROP_LINE_MARKERS):
        return None
    if line.startswith("#"):
        line = line.replace("SI-7000 官方资料包目录与整理规则", "资料包封面与总目录")
        line = line.replace("中文整理", "中文翻译")
    else:
        line = line.replace("中文整理", "中文译文")
    line = line.replace("整理备注：", "译注：")
    line = line.replace("整理备注：", "译注：")
    line = line.replace("本文整理", "本章翻译")
    line = line.replace("本文是官方资料中文译稿", "本章为官方资料中文译稿")
    line = line.replace("本文件", "本章")
    return line.rstrip()


def clean_markdown_document(path: Path) -> list[str]:
    lines = path.read_text(encoding="utf-8").splitlines()
    output: list[str] = []
    skip_level: int | None = None

    for raw_line in lines:
        heading = heading_level_and_title(raw_line)
        if heading:
            level, title = heading
            if skip_level is not None and level <= skip_level:
                skip_level = None
            if should_skip_heading(title):
                skip_level = level
                continue
        if skip_level is not None:
            continue

        cleaned = clean_source_line(raw_line)
        if cleaned is None:
            continue
        output.append(cleaned)

    return collapse_blank_lines(output)


def collapse_blank_lines(lines: list[str]) -> list[str]:
    result: list[str] = []
    blank_count = 0
    for line in lines:
        if line.strip():
            blank_count = 0
            result.append(line.rstrip())
        else:
            blank_count += 1
            if blank_count <= 1:
                result.append("")
    while result and not result[-1].strip():
        result.pop()
    return result


def build_source_markdown() -> str:
    lines: list[str] = [
        "# SI-7000 官方资料包中文全文翻译（优化排版）",
        "",
        f"- 原始资料：`{SOURCE_PDF}`",
        "- 正文来源：本目录内按官方资料逐章校订的中文译稿。",
        "- 排版口径：新版 PDF 采用可读性优先的章节化排版，不强制与原 PDF 页数逐页对应；原图、图纸、照片和扫描页仅在正文需要对照的位置保留。",
        "- 术语口径：专业术语、协议名、功能码、寄存器地址、单位、部件英文名和界面英文按必要程度保留。",
        "",
        "## 原资料包结构",
        "",
        "| PDF 页码 | 原始资料标题或内容 | 中文译文所在章节 |",
        "| --- | --- | --- |",
        "| 1 | SI-7000 Online Informational Catalog 封面 | 资料包封面与总目录 |",
        "| 2 | Table of Contents | 资料包封面与总目录 |",
        "| 3-6 | SI-7000 LTD System Specification Sheet | 产品规格、认证与包装 |",
        "| 7-9 | IECEx Certificates of Conformity | 产品规格、认证与包装 |",
        "| 10-24 | `020-701 Rev B` DCS Modbus Interface Specification | DCS Modbus 接口规范 |",
        "| 25 | SI-7000 Standard Packaging Specifications | 产品规格、认证与包装 |",
        "| 26-29 | SI-7000 General Arrangement | 机械布置与现场接线 |",
        "| 30-31 | SI-7000 Field Wiring Diagram | 机械布置与现场接线 |",
        "| 32-51 | `090-7000-1 Rev A` Installation & Commissioning Manual | 安装与调试手册 |",
        "| 52-85 | `090-7000-2A/2B` Operational Manual | 操作手册 |",
        "| 86-100 | `090-7000-3 Rev -` Service Manual | 维修手册与保修信息 |",
        "| 101 | Warranty Information | 维修手册与保修信息 |",
        "",
    ]

    for index, doc_path in enumerate(INPUT_DOCS):
        if index:
            lines.append("")
            lines.append("\\newpage")
            lines.append("")
        lines.extend(clean_markdown_document(doc_path))

    return "\n".join(collapse_blank_lines(lines)) + "\n"


def make_styles(font_name: str):
    sample = getSampleStyleSheet()
    base = ParagraphStyle(
        "ChineseBase",
        parent=sample["Normal"],
        fontName=font_name,
        fontSize=9.5,
        leading=14.2,
        textColor=colors.HexColor("#1f2933"),
        alignment=TA_LEFT,
        wordWrap="CJK",
        spaceAfter=5,
    )
    return {
        "title": ParagraphStyle(
            "TitleChinese",
            parent=base,
            fontSize=21,
            leading=28,
            alignment=TA_CENTER,
            spaceBefore=58,
            spaceAfter=16,
            textColor=colors.HexColor("#0f172a"),
        ),
        "h1": ParagraphStyle(
            "Heading1Chinese",
            parent=base,
            fontSize=16.2,
            leading=21,
            spaceBefore=12,
            spaceAfter=8,
            textColor=colors.HexColor("#0f172a"),
        ),
        "h2": ParagraphStyle(
            "Heading2Chinese",
            parent=base,
            fontSize=13.2,
            leading=17.2,
            spaceBefore=10,
            spaceAfter=6,
            textColor=colors.HexColor("#1d4ed8"),
        ),
        "h3": ParagraphStyle(
            "Heading3Chinese",
            parent=base,
            fontSize=11,
            leading=14.6,
            spaceBefore=8,
            spaceAfter=4,
            textColor=colors.HexColor("#334155"),
        ),
        "body": base,
        "bullet": ParagraphStyle(
            "BulletChinese",
            parent=base,
            leftIndent=16,
            firstLineIndent=-10,
            spaceAfter=3,
        ),
        "caption": ParagraphStyle(
            "CaptionChinese",
            parent=base,
            fontSize=8.1,
            leading=10.6,
            alignment=TA_CENTER,
            textColor=colors.HexColor("#475569"),
            spaceBefore=2,
            spaceAfter=8,
        ),
        "table": ParagraphStyle(
            "TableChinese",
            parent=base,
            fontSize=7.6,
            leading=9.6,
            wordWrap="CJK",
            spaceAfter=0,
        ),
        "table_header": ParagraphStyle(
            "TableHeaderChinese",
            parent=base,
            fontSize=7.6,
            leading=9.6,
            textColor=colors.white,
            wordWrap="CJK",
            spaceAfter=0,
        ),
        "code": ParagraphStyle(
            "CodeChinese",
            parent=base,
            fontSize=8,
            leading=10.5,
            leftIndent=10,
            backColor=colors.HexColor("#f1f5f9"),
            borderColor=colors.HexColor("#cbd5e1"),
            borderWidth=0.4,
            borderPadding=5,
        ),
    }


def clean_inline_markdown(text: str) -> str:
    text = re.sub(r"!\[([^\]]*)\]\([^)]+\)", r"图：\1", text)
    text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)
    text = text.replace("**", "")
    text = text.replace("__", "")
    text = text.replace("`", "")
    text = re.sub(r"\s+", " ", text).strip()
    return text


def paragraph_xml(text: str) -> str:
    text = clean_inline_markdown(text)
    text = html.escape(text)
    text = text.replace(" / ", " / ")
    return text


def image_references(line: str) -> list[tuple[str, Path]]:
    refs: list[tuple[str, Path]] = []
    for alt, rel in re.findall(r"!\[([^\]]*)\]\(([^)]+)\)", line):
        path = (ROOT / rel).resolve()
        refs.append((alt or path.name, path))
    return refs


def make_image_file(path: Path, draw_width: float, draw_height: float, tmp_dir: Path, quality: int) -> Path:
    with PilImage.open(path) as image:
        image = image.convert("RGB")
        target_width = max(1, int(draw_width / inch * 170))
        target_height = max(1, int(draw_height / inch * 170))
        image.thumbnail((target_width, target_height), PilImage.Resampling.LANCZOS)
        buffer = io.BytesIO()
        image.save(buffer, format="JPEG", quality=quality, optimize=True)

    out_path = tmp_dir / f"{path.stem}_{len(list(tmp_dir.iterdir()))}.jpg"
    out_path.write_bytes(buffer.getvalue())
    return out_path


def image_flowable(path: Path, alt: str, max_width: float, max_height: float, styles, tmp_dir: Path, quality: int = 88):
    if not path.exists():
        return [Paragraph(paragraph_xml(f"图片缺失：{path.name}"), styles["caption"])]

    with PilImage.open(path) as image:
        width_px, height_px = image.size
    ratio = min(max_width / width_px, max_height / height_px, 1.0)
    draw_width = width_px * ratio
    draw_height = height_px * ratio
    img_path = make_image_file(path, draw_width, draw_height, tmp_dir, quality)
    return [
        Spacer(1, 4),
        Image(str(img_path), width=draw_width, height=draw_height),
        Paragraph(paragraph_xml(alt), styles["caption"]),
    ]


def is_table_separator(line: str) -> bool:
    return bool(re.match(r"^\s*\|?\s*:?-{3,}:?\s*(\|\s*:?-{3,}:?\s*)+\|?\s*$", line))


def split_table_row(line: str) -> list[str]:
    line = line.strip()
    if line.startswith("|"):
        line = line[1:]
    if line.endswith("|"):
        line = line[:-1]
    return [clean_inline_markdown(cell.strip()) for cell in line.split("|")]


def table_column_widths(rows: list[list[str]], available_width: float) -> list[float]:
    col_count = max(len(row) for row in rows)
    weights = [1.0] * col_count
    for col in range(col_count):
        lengths = [len(row[col]) if col < len(row) else 0 for row in rows]
        weights[col] = min(max(max(lengths, default=1), 8), 42)
    total = sum(weights) or 1.0
    widths = [available_width * weight / total for weight in weights]
    min_width = available_width * 0.1
    widths = [max(width, min_width) for width in widths]
    total = sum(widths)
    return [width * available_width / total for width in widths]


def table_flowable(rows: list[list[str]], available_width: float, styles, font_name: str):
    col_count = max(len(row) for row in rows)
    normalized_rows = [row + [""] * (col_count - len(row)) for row in rows]
    data = []
    for row_index, row in enumerate(normalized_rows):
        style = styles["table_header"] if row_index == 0 else styles["table"]
        data.append([Paragraph(paragraph_xml(cell), style) for cell in row])

    table_cls = LongTable if len(rows) > 10 else Table
    table = table_cls(data, colWidths=table_column_widths(normalized_rows, available_width), repeatRows=1)
    table.setStyle(
        TableStyle(
            [
                ("FONTNAME", (0, 0), (-1, -1), font_name),
                ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#334155")),
                ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
                ("GRID", (0, 0), (-1, -1), 0.25, colors.HexColor("#cbd5e1")),
                ("VALIGN", (0, 0), (-1, -1), "TOP"),
                ("LEFTPADDING", (0, 0), (-1, -1), 4),
                ("RIGHTPADDING", (0, 0), (-1, -1), 4),
                ("TOPPADDING", (0, 0), (-1, -1), 4),
                ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
                ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#f8fafc")]),
            ]
        )
    )
    return [Spacer(1, 3), table, Spacer(1, 7)]


def append_paragraph(buffer: list[str], story: list, styles):
    if not buffer:
        return
    text = " ".join(line.strip() for line in buffer if line.strip())
    if text:
        story.append(Paragraph(paragraph_xml(text), styles["body"]))
    buffer.clear()


def markdown_to_story(markdown_text: str, doc_width: float, doc_height: float, styles, font_name: str, tmp_dir: Path):
    story: list = []
    lines = markdown_text.splitlines()
    paragraph_buffer: list[str] = []
    in_code = False
    code_lines: list[str] = []
    first_h1 = True
    i = 0

    while i < len(lines):
        line = lines[i]
        heading = heading_level_and_title(line)

        if line.strip() == "\\newpage":
            append_paragraph(paragraph_buffer, story, styles)
            story.append(PageBreak())
            i += 1
            continue

        if line.startswith("```"):
            append_paragraph(paragraph_buffer, story, styles)
            if in_code:
                story.append(Paragraph(html.escape("\n".join(code_lines)).replace("\n", "<br/>"), styles["code"]))
                code_lines = []
                in_code = False
            else:
                in_code = True
            i += 1
            continue
        if in_code:
            code_lines.append(line)
            i += 1
            continue

        if heading:
            append_paragraph(paragraph_buffer, story, styles)
            level, title = heading
            title = normalized_heading_title(title)
            if level == 1:
                if not first_h1:
                    story.append(PageBreak())
                first_h1 = False
                style = styles["title"] if len(story) == 0 else styles["h1"]
            elif level == 2:
                style = styles["h2"]
            else:
                style = styles["h3"]
            story.append(Paragraph(paragraph_xml(title), style))
            i += 1
            continue

        if i + 1 < len(lines) and line.strip().startswith("|") and is_table_separator(lines[i + 1]):
            append_paragraph(paragraph_buffer, story, styles)
            table_lines = [line]
            i += 1
            table_lines.append(lines[i])
            i += 1
            while i < len(lines) and lines[i].strip().startswith("|"):
                table_lines.append(lines[i])
                i += 1
            rows = [split_table_row(row) for idx, row in enumerate(table_lines) if idx != 1]
            if rows:
                story.extend(table_flowable(rows, doc_width, styles, font_name))
            continue

        refs = image_references(line)
        if refs:
            append_paragraph(paragraph_buffer, story, styles)
            for alt, path in refs:
                story.extend(image_flowable(path, alt, doc_width, doc_height * 0.52, styles, tmp_dir))
            leftover = re.sub(r"!\[[^\]]*\]\([^)]+\)", "", line).strip()
            if leftover:
                story.append(Paragraph(paragraph_xml(leftover), styles["body"]))
            i += 1
            continue

        stripped = line.strip()
        if not stripped:
            append_paragraph(paragraph_buffer, story, styles)
            i += 1
            continue

        if re.match(r"^(\s*)([-*]|\d+\.)\s+", line):
            append_paragraph(paragraph_buffer, story, styles)
            content = re.sub(r"^\s*([-*]|\d+\.)\s+", "", line)
            marker = line.strip().split()[0]
            bullet = "-" if marker in {"-", "*"} else marker
            story.append(Paragraph(f"{html.escape(bullet)} {paragraph_xml(content)}", styles["bullet"]))
            i += 1
            continue

        paragraph_buffer.append(line)
        i += 1

    append_paragraph(paragraph_buffer, story, styles)
    return story


def draw_page(canvas, doc):
    canvas.saveState()
    canvas.setStrokeColor(colors.HexColor("#cbd5e1"))
    canvas.setLineWidth(0.4)
    canvas.line(doc.leftMargin, 0.62 * inch, doc.pagesize[0] - doc.rightMargin, 0.62 * inch)
    canvas.setFont("SimHei", 7.5)
    canvas.setFillColor(colors.HexColor("#475569"))
    canvas.drawString(doc.leftMargin, 0.42 * inch, "SI-7000 官方资料包中文全文翻译（优化排版）")
    canvas.drawRightString(doc.pagesize[0] - doc.rightMargin, 0.42 * inch, f"第 {doc.page} 页")
    canvas.restoreState()


def build_pdf(markdown_text: str):
    font_name = register_font()
    styles = make_styles(font_name)
    doc = SimpleDocTemplate(
        str(OUTPUT_PDF),
        pagesize=A4,
        leftMargin=0.58 * inch,
        rightMargin=0.58 * inch,
        topMargin=0.56 * inch,
        bottomMargin=0.82 * inch,
        title="SI-7000 官方资料包中文全文翻译（优化排版）",
        author="Codex",
    )
    with tempfile.TemporaryDirectory(prefix="si7000_pdf_images_") as tmp_name:
        tmp_dir = Path(tmp_name)
        story = markdown_to_story(markdown_text, doc.width, doc.height, styles, font_name, tmp_dir)
        doc.build(story, onFirstPage=draw_page, onLaterPages=draw_page)


def main() -> None:
    markdown_text = build_source_markdown()
    OUTPUT_MD.write_text(markdown_text, encoding="utf-8", newline="\n")
    build_pdf(markdown_text)
    print(f"Generated: {OUTPUT_MD}")
    print(f"Generated: {OUTPUT_PDF}")


if __name__ == "__main__":
    main()

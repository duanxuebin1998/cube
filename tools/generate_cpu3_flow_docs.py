from __future__ import annotations

import html
import json
import re
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPU3 = ROOT / "LTD_DISPLAY_CPU3"
DOC_DIR = ROOT / "docs" / "00_程序流程导航" / "CPU3"
ASSET_DIR = DOC_DIR / "assets"
CPU2_ASSET_DIR = ROOT / "docs" / "00_程序流程导航" / "CPU2" / "assets"
CSS_VERSION = "cpu3-20260710-protocol-snapshot-contract"
DOC_DATE = "2026-07-10"
CPU2_COMM_SOURCE = "Communication/internal/main_board_modbus/cpu2_communicate.c"
MENU_SOURCE = "Application/display/display_tankopera.c"
FRAM_SOURCE = "Application/system_param/cpu3_comm_display_params.c"
DSM_COMM_SOURCE = "Communication/external/DSM_modbus/DSM_communication.c"
DSM_SLAVE_SOURCE = "Communication/external/DSM_modbus/DSM_SlaveModbus_modbus2.c"
DSM_DATA_SOURCE = "Communication/external/DSM_modbus/DSM_DataAnalysis_modbus2.c"


def read_version_macro(path: Path, macro: str, fallback: str) -> str:
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(rf'#define\s+{re.escape(macro)}\s+"([^"]+)"', text)
    if match:
        return match.group(1)
    return fallback


CPU3_VERSION = read_version_macro(CPU3 / "Application" / "app_version.h", "CPU3_APP_VERSION_STRING", "V1.15.0.0")


def read_integer_macro(path: Path, macro: str, fallback: int) -> int:
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(rf"#define\s+{re.escape(macro)}\s+(\d+)[uUlL]*", text)
    if match:
        return int(match.group(1))
    return fallback


DEVICE_PROTOCOL_VERSION = read_integer_macro(
    CPU3 / "Application" / "system_param" / "system_parameter.h",
    "DEVICE_PROTOCOL_VERSION",
    14,
)


CPU3_STYLE_APPEND = r"""

/* CPU3 flow overview refinements */
.overview-grid {
  display: grid;
  grid-template-columns: repeat(4, minmax(0, 1fr));
  gap: 14px;
  margin-top: 16px;
}

.overview-card {
  position: relative;
  display: block;
  min-height: 172px;
  padding: 18px 18px 16px;
  border: 1px solid #cbd8e8;
  border-radius: var(--radius);
  background: linear-gradient(180deg, #ffffff 0%, #f7fbff 100%);
  box-shadow: 0 10px 22px rgba(24, 39, 75, 0.06);
  color: var(--ink);
  text-decoration: none;
  overflow: hidden;
}

.overview-card::before {
  content: "";
  position: absolute;
  inset: 0 auto 0 0;
  width: 5px;
  background: linear-gradient(180deg, var(--blue), var(--teal));
}

.overview-card::after {
  content: "查看流程";
  position: absolute;
  right: 14px;
  bottom: 12px;
  color: var(--blue);
  font-size: 12px;
  font-weight: 700;
}

.overview-card:hover {
  transform: translateY(-2px);
  border-color: #9bb9dc;
  box-shadow: 0 16px 28px rgba(24, 39, 75, 0.10);
  text-decoration: none;
}

.overview-card strong {
  display: block;
  min-height: 42px;
  padding-right: 16px;
  color: #142033;
  font-size: 17px;
  line-height: 1.35;
}

.overview-card span {
  display: block;
  margin-top: 10px;
  padding-bottom: 24px;
  color: #4c5f78;
  font-size: 13px;
  line-height: 1.65;
}

.doc-search {
  width: min(720px, 100%);
  height: 44px;
  margin: 8px 0 10px;
  padding: 0 14px;
  border: 1px solid #b9cce2;
  border-radius: var(--radius);
  background: #fff;
  color: var(--ink);
  font: inherit;
  box-shadow: 0 8px 18px rgba(24, 39, 75, 0.05);
}

.doc-search:focus {
  outline: 3px solid rgba(31, 95, 153, 0.16);
  border-color: var(--blue);
}

.source-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px;
}

.source-card {
  min-width: 0;
  padding: 14px;
  border: 1px solid var(--line);
  border-radius: var(--radius);
  background: #fbfdff;
  overflow-wrap: anywhere;
  word-break: break-word;
}

.source-card p {
  margin: 6px 0 0;
  overflow-wrap: anywhere;
  word-break: break-word;
}

.issue-filter {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  margin: 10px 0 14px;
}

.issue-filter button {
  min-height: 34px;
  padding: 6px 12px;
  border: 1px solid #b9cce2;
  border-radius: var(--radius);
  background: #fff;
  color: #183553;
  font: inherit;
  font-size: 13px;
  cursor: pointer;
}

.issue-filter button.is-active {
  border-color: var(--blue);
  background: var(--soft-blue);
  color: var(--blue);
  font-weight: 700;
}

@media (max-width: 1180px) {
  .overview-grid {
    grid-template-columns: repeat(2, minmax(0, 1fr));
  }
}

@media (max-width: 720px) {
  .overview-grid,
  .source-grid {
    grid-template-columns: 1fr;
  }

  .overview-card {
    min-height: 150px;
  }
}
"""


def esc(text: object) -> str:
    return html.escape(str(text), quote=True)


def code(text: str) -> str:
    return f"<code>{esc(text)}</code>"


def slugify(text: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_\-\u4e00-\u9fff]+", "-", text).strip("-")
    return s or "item"


def flow_page_identifier(file_name: str) -> str:
    """Return the stable website-embed identifier used by committed CPU3 pages."""

    stem = Path(file_name).stem
    return "cube-flow-cpu3-" + slugify(stem).replace("_", "-").lower()


def read_text(rel: str) -> str:
    return (CPU3 / rel).read_text(encoding="utf-8", errors="replace")


SOURCE_CACHE: dict[str, list[str]] = {}


def line_snippet(rel: str, start: int, end: int | None = None) -> str:
    path = CPU3 / rel
    if rel not in SOURCE_CACHE:
        SOURCE_CACHE[rel] = path.read_text(encoding="utf-8", errors="replace").splitlines()
    lines = SOURCE_CACHE[rel]
    if end is None:
        end = start
    start = max(1, start)
    end = min(len(lines), end)
    body = "\n".join(f"{idx:>5}: {lines[idx - 1]}" for idx in range(start, end + 1))
    return f"{rel}:{start}-{end}\n{body}"


def func_line(rel: str, name: str) -> str:
    text = read_text(rel)
    m = re.search(
        rf"(?m)^[ \t]*[^#\n(){{}};=]+?\b{re.escape(name)}\s*"
        rf"\([^;{{}}]*?\)\s*\{{",
        text,
    )
    if not m:
        return f"{rel}:?"
    line = text[: m.start()].count("\n") + 1
    return f"{rel}:{line}"


def path_link(rel: str, label: str | None = None) -> str:
    p = (CPU3 / rel).resolve().as_posix()
    return f'<a href="file:///{esc(p)}">{esc(label or rel)}</a>'


def split_label(label: str, width: int = 18) -> list[str]:
    label = str(label)
    if "\n" in label:
        return [part for part in label.split("\n") if part]
    if len(label) <= width:
        return [label]
    pieces: list[str] = []
    current = ""
    for token in re.split(r"(\s+|/|、|，|；|,)", label):
        if token == "":
            continue
        candidate = current + token
        if len(candidate) > width and current:
            pieces.append(current.strip())
            current = token.strip()
        else:
            current = candidate
    if current.strip():
        pieces.append(current.strip())
    if len(pieces) <= 1:
        pieces = [label[i : i + width] for i in range(0, len(label), width)]
    return pieces[:4]


class SvgBuilder:
    def __init__(
        self,
        diagram_id: str,
        accessible_id: str,
        width: int = 1120,
        height: int = 760,
    ):
        self.diagram_id = diagram_id
        self.accessible_id = accessible_id
        self.width = width
        self.height = height
        self.parts: list[str] = []

    def defs(self) -> str:
        aid = esc(self.diagram_id)
        return (
            f'<defs><marker id="{aid}-arrow" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            '<path d="M 0 0 L 10 5 L 0 10 z" fill="#6a7c93"></path></marker>'
            f'<marker id="{aid}-arrow-red" viewBox="0 0 10 10" refX="9" refY="5" '
            'markerWidth="7" markerHeight="7" orient="auto-start-reverse">'
            '<path d="M 0 0 L 10 5 L 0 10 z" fill="#b73737"></path></marker></defs>'
        )

    def add_node(self, node_id: str, kind: str, x: int, y: int, w: int, h: int, label: str):
        dom_id = f"{self.diagram_id}-{node_id}"
        lines = split_label(label, 19 if w < 260 else 24)
        cx = x + w / 2
        cy = y + h / 2
        if kind in {"start", "end"}:
            shape = f'<ellipse class="node-shape" cx="{cx:.0f}" cy="{cy:.0f}" rx="{w/2:.0f}" ry="{h/2:.0f}"></ellipse>'
        elif kind == "decision":
            shape = (
                f'<path class="node-shape" d="M {cx:.0f} {y} L {x+w} {cy:.0f} '
                f'L {cx:.0f} {y+h} L {x} {cy:.0f} Z"></path>'
            )
        else:
            shape = f'<rect class="node-shape" x="{x}" y="{y}" width="{w}" height="{h}" rx="8"></rect>'
        text_y = cy - ((len(lines) - 1) * 10)
        tspans = "".join(
            f'<tspan x="{cx:.0f}" dy="{0 if i == 0 else 20}">{esc(line)}</tspan>'
            for i, line in enumerate(lines)
        )
        self.parts.append(
            f'<g id="{esc(dom_id)}" class="{esc(kind)}">{shape}'
            f'<text x="{cx:.0f}" y="{text_y:.0f}" text-anchor="middle">{tspans}</text></g>'
        )

    def add_edge(self, d: str, label: str | None = None, red: bool = False):
        aid = f"{esc(self.diagram_id)}-arrow-red" if red else f"{esc(self.diagram_id)}-arrow"
        cls = "edge error-edge" if red else "edge"
        self.parts.append(f'<path class="{cls}" d="{esc(d)}" marker-end="url(#{aid})"></path>')
        if label:
            m = re.search(r"M\s*([0-9.]+)\s*([0-9.]+).*?([0-9.]+)\s*([0-9.]+)$", d)
            if m:
                x = (float(m.group(1)) + float(m.group(3))) / 2
                y = (float(m.group(2)) + float(m.group(4))) / 2 - 8
            else:
                x, y = 40, 40
            self.parts.append(f'<text class="small label" x="{x:.0f}" y="{y:.0f}" text-anchor="middle">{esc(label)}</text>')

    def render(self, title: str, description: str) -> str:
        title_id = f"{self.accessible_id}-title"
        desc_id = f"{self.accessible_id}-desc"
        return (
            f'<svg class="biz-flow" '
            f'viewBox="0 0 {self.width} {self.height}" '
            'aria-hidden="true" focusable="false">'
            f'<title id="{esc(title_id)}">{esc(title)}</title>'
            f'<desc id="{esc(desc_id)}">{esc(description)}</desc>'
            f"{self.defs()}{''.join(self.parts)}</svg>"
        )


def svg_required_height(nodes: list[dict], edges: list[dict], minimum: int) -> int:
    max_y = 0.0
    for node in nodes:
        max_y = max(max_y, float(node["y"]) + float(node.get("h", 80)))
    for edge in edges:
        values = [float(value) for value in re.findall(r"[0-9.]+", edge.get("d", ""))]
        max_y = max(max_y, *(values[1::2] or [0.0]))
    return int(max(minimum, max_y + 90))


def flow_svg(
    diagram_id: str,
    title: str,
    nodes: list[dict],
    edges: list[dict],
    width: int = 1120,
    height: int = 760,
    *,
    accessible_id: str,
    description: str,
) -> str:
    b = SvgBuilder(
        diagram_id,
        accessible_id,
        width,
        svg_required_height(nodes, edges, height),
    )
    for node in nodes:
        b.add_node(
            node["id"],
            node.get("kind", "process"),
            node["x"],
            node["y"],
            node.get("w", 250),
            node.get("h", 80),
            node["label"],
        )
    for edge in edges:
        b.add_edge(edge["d"], edge.get("label"), edge.get("red", False))
    caption_id = f"{accessible_id}-caption"
    desc_id = f"{accessible_id}-desc"
    return (
        '<figure class="cube-flow-figure">'
        f'<figcaption id="{esc(caption_id)}">{esc(title)}</figcaption>'
        '<div class="svg-flow-wrap cube-flow__viewport" '
        'data-flow-width="standard" tabindex="0" role="region" '
        f'aria-labelledby="{esc(caption_id)}" aria-describedby="{esc(desc_id)}">'
        f"{b.render(title, description)}</div></figure>"
    )


def source_index(items: list[dict]) -> str:
    rows = []
    for item in items:
        refs = " ".join(code(ref) for ref in item.get("refs", []))
        rows.append(
            f'<div class="source-card"><h3>{esc(item["title"])}</h3>'
            f'<p>{esc(item["desc"])}</p><p>{refs}</p></div>'
        )
    return '<div class="source-grid">' + "".join(rows) + "</div>"


def evidence(items: list[dict]) -> str:
    rows = []
    for item in items:
        rows.append(
            f'<div class="evidence-item"><strong>{esc(item["title"])}</strong>'
            f'<span>{item["text"]}</span></div>'
        )
    return '<div class="flow-evidence">' + "".join(rows) + "</div>"


def issues(items: list[dict]) -> str:
    rows = []
    for idx, item in enumerate(items, 1):
        level = item.get("level", "mid")
        rows.append(
            f'<article class="issue {esc(level)}" data-issue-level="{esc(level)}">'
            f'<h3>{idx}. {esc(item["title"])}</h3><p>{esc(item["desc"])}</p>'
            f'<p><strong>建议：</strong>{esc(item["suggest"])}</p>'
            f'<p class="muted">{esc(item["ref"])}</p></article>'
        )
    return '<div class="issue-list">' + "".join(rows) + "</div>"


def page_html(page: dict, all_pages: list[dict]) -> str:
    page_identifier = flow_page_identifier(page["file"])
    flow_evidence_count = sum(len(flow.get("evidence", [])) for flow in page.get("flows", []))
    nav = (
        '<nav class="topnav">'
        '<a href="CPU3程序流程总览.html">总览</a>'
        '<a href="#layer">系统边界</a>'
        '<a href="#flow">功能总览</a>'
        '<a href="#detailflows">详细流程图</a>'
        '<a href="#risk">问题点</a>'
        '<a href="#source">源码索引</a>'
        '</nav>'
    )
    domain_links = " ".join(
        f'<a href="{esc(p["file"])}">{esc(p["short"])}</a>' for p in all_pages if p["file"] != page["file"]
    )
    commands = " ".join(code(c) for c in page.get("commands", [])) or "无直接外部指令"
    files = " ".join(code(f) for f in page.get("source_files", []))
    layer = (
        '<section id="layer"><h2>1. 系统边界与阅读顺序</h2>'
        '<p class="lead">本页按“外部入口/内部触发 → CPU3 业务动作 → CPU2 或外部接口输出 → 异常出口”的顺序整理。图中优先写业务动作，函数名只作为源码依据。</p>'
        '<div class="layer-map">'
        f'<div class="layer-name">入口/触发</div><div class="layer-body">{esc(page["entry"])}</div>'
        f'<div class="layer-name">业务处理</div><div class="layer-body">{esc(page["summary"])}</div>'
        f'<div class="layer-name">相关指令</div><div class="layer-body">{commands}</div>'
        f'<div class="layer-name">源码边界</div><div class="layer-body">{files}</div>'
        f'<div class="layer-name">关联页面</div><div class="layer-body">{domain_links}</div>'
        '</div>'
        f'<div class="metric-strip"><div class="metric"><span>流程图</span><strong>{len(page["flows"]) + 1}</strong></div>'
        f'<div class="metric"><span>源码文件</span><strong>{len(page.get("source_files", []))}</strong></div>'
        f'<div class="metric"><span>流程证据</span><strong>{flow_evidence_count}</strong></div>'
        f'<div class="metric"><span>源码索引</span><strong>{len(page.get("sources", []))}</strong></div>'
        f'<div class="metric"><span>问题点</span><strong>{len(page.get("issues", []))}</strong></div>'
        f'<div class="metric"><span>整理日期</span><strong>{DOC_DATE}</strong></div></div></section>'
    )
    overview = (
        '<section id="flow"><h2>2. 功能域业务总览 SVG</h2>'
        f'<p>{esc(page["overview_text"])}</p>'
        + flow_svg(
            page["id"] + "-overview",
            page["title"] + "总览",
            page["overview_nodes"],
            page["overview_edges"],
            height=page.get("overview_height", 720),
            accessible_id=f"{page_identifier}-diagram-01",
            description=page["overview_description"],
        )
        + "</section>"
    )
    flow_parts = [
        '<section id="detailflows"><h2>3. 业务级详细程序流程图</h2>'
        '<p class="lead">本节只保留业务级流程图，不再把“命令”和“代码梳理”拆成两套。每张图后都列出对应源码证据和已识别的问题点。</p>'
        '<div class="legend"><span><i class="dot start"></i>入口/正常出口</span><span><i class="dot action"></i>处理动作</span><span><i class="dot decision"></i>判断分支</span><span><i class="dot loop"></i>循环/轮询</span><span><i class="dot state"></i>状态/缓存更新</span><span><i class="dot error"></i>异常出口</span></div>'
    ]
    for idx, flow in enumerate(page["flows"], 1):
        flow_parts.append(f'<h3>3.{idx} {esc(flow["title"])}</h3>')
        flow_parts.append(f'<p class="flow-caption">{esc(flow["caption"])}</p>')
        flow_parts.append(
            flow_svg(
                page["id"] + f"-flow-{idx}",
                flow["title"],
                flow["nodes"],
                flow["edges"],
                height=flow.get("height", 860),
                accessible_id=f"{page_identifier}-diagram-{idx + 1:02d}",
                description=flow["caption"],
            )
        )
        flow_parts.append(evidence(flow.get("evidence", [])))
    flow_parts.append("</section>")
    risk = (
        '<section id="risk"><h2>4. 问题点与优化建议</h2>'
        '<div class="issue-filter" data-issue-filter-group><button class="is-active" data-issue-filter="all">全部</button><button data-issue-filter="high">高</button><button data-issue-filter="mid">中</button><button data-issue-filter="low">低</button></div>'
        + issues(page.get("issues", []))
        + "</section>"
    )
    src = (
        '<section id="source"><h2>5. 关键源码索引</h2>'
        '<p class="lead">源码索引用于回查实现位置；建议先看流程图，再按索引核对函数。</p>'
        + source_index(page.get("sources", []))
        + "</section>"
    )
    return (
        '<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1">'
        f'<title>{esc(page["title"])}</title><link rel="stylesheet" href="assets/流程文档样式.css?v={CSS_VERSION}">'
        '<link rel="stylesheet" href="../assets/网站嵌入增强.css"></head>'
        f'<body class="cube-flow-page" data-cube-flow-page="{esc(page_identifier)}"><div class="wrap">'
        f'<header class="hero"><h1>{esc(page["title"])}</h1><p>{esc(page["hero"])}</p>'
        '<div class="meta-grid">'
        f'<div class="meta"><span>项目</span><strong>LTD_DISPLAY_CPU3 / CPU3 {esc(page.get("version", CPU3_VERSION))}</strong></div>'
        f'<div class="meta"><span>功能域</span><strong>{esc(page["short"])}</strong></div>'
        f'<div class="meta"><span>核心源码</span><strong>{esc(", ".join(page.get("source_files", [])[:2]))}</strong></div>'
        f'<div class="meta"><span>本页流程图</span><strong>{len(page["flows"]) + 1} 张业务级 SVG</strong></div>'
        '</div></header>'
        + nav
        + '\n<!-- CUBE_EMBED_START -->\n'
        + layer
        + overview
        + "".join(flow_parts)
        + risk
        + src
        + '\n<!-- CUBE_EMBED_END -->\n'
        + '</div><script src="assets/流程文档交互.js"></script>'
        + '<script src="../assets/网站嵌入增强.js"></script></body></html>'
    )


def overview_page(pages: list[dict]) -> str:
    page_identifier = flow_page_identifier("CPU3程序流程总览.html")
    description = (
        "CPU3 上电初始化显示、RTC、FRAM 参数、串口和 DSM 地址后进入主循环；"
        "主循环调度 CPU2 轮询、显示按键与状态刷新，并把 COM1/2/3 请求分发到 DSM、Wartsila 或 SI。"
        "轮询结果回写测量、RSSI、AO 运行态和参数镜像，菜单或外部写入再桥接 CPU2；"
        "串口、忙队列、CRC 或地址异常则恢复接收、返回异常或不回包，随后等待下一轮。"
    )
    nodes = [
        {"id": "n1", "kind": "start", "x": 440, "y": 35, "w": 240, "h": 62, "label": "CPU3 上电进入主循环"},
        {"id": "n2", "kind": "process", "x": 90, "y": 160, "w": 260, "h": 80, "label": "初始化显示、RTC、FRAM 参数、串口和 DSM 地址"},
        {"id": "n3", "kind": "loop", "x": 430, "y": 160, "w": 260, "h": 80, "label": "主循环调度显示任务、外部 COM 和 CPU2 轮询"},
        {"id": "n4", "kind": "process", "x": 770, "y": 160, "w": 260, "h": 80, "label": "外部协议请求通过 COM1/2/3 分发到 DSM、Wartsila、SI"},
        {"id": "n5", "kind": "process", "x": 90, "y": 320, "w": 260, "h": 80, "label": "CPU2 轮询刷新测量结果、RSSI、AO运行态缓存和保持参数镜像"},
        {"id": "n6", "kind": "process", "x": 430, "y": 320, "w": 260, "h": 80, "label": "显示任务消费按键，按节拍刷新状态页和菜单页"},
        {"id": "n7", "kind": "process", "x": 770, "y": 320, "w": 260, "h": 80, "label": "菜单或外部协议写参数后下发 CPU2，含 AO 使能"},
        {"id": "n8", "kind": "decision", "x": 430, "y": 480, "w": 260, "h": 110, "label": "串口错误、忙队列覆盖或 CRC/地址异常？"},
        {"id": "n9", "kind": "error", "x": 760, "y": 500, "w": 260, "h": 82, "label": "恢复接收、异常响应或不回包"},
        {"id": "n10", "kind": "end", "x": 430, "y": 640, "w": 260, "h": 62, "label": "等待下一轮主循环或中断事件"},
    ]
    edges = [
        {"d": "M 560 97 L 560 160"},
        {"d": "M 440 66 C 260 92 220 120 220 160"},
        {"d": "M 680 66 C 860 92 900 120 900 160"},
        {"d": "M 560 240 L 220 320"},
        {"d": "M 560 240 L 560 320"},
        {"d": "M 900 240 L 900 320"},
        {"d": "M 220 400 C 320 455 420 480 560 480"},
        {"d": "M 560 400 L 560 480"},
        {"d": "M 900 400 C 800 455 700 480 560 480"},
        {"d": "M 690 535 L 760 541", "label": "是", "red": True},
        {"d": "M 560 590 L 560 640", "label": "否"},
    ]
    cards = "".join(
        f'<a class="overview-card" href="{esc(p["file"])}"><strong>{esc(p["short"])}</strong><span>{esc(p["summary"])}</span></a>'
        for p in pages
    )
    search_data = []
    for p in pages:
        source_text = " ".join(
            " ".join(str(src.get(key, "")) for key in ("title", "desc")) + " " + " ".join(src.get("refs", []))
            for src in p.get("sources", [])
        )
        flow_text = " ".join(
            " ".join(str(flow.get(key, "")) for key in ("title", "caption")) +
            " " +
            " ".join(" ".join(str(item.get(k, "")) for k in ("title", "text")) for item in flow.get("evidence", []))
            for flow in p.get("flows", [])
        )
        issue_text = " ".join(
            " ".join(str(issue.get(key, "")) for key in ("title", "text", "suggestion", "source"))
            for issue in p.get("issues", [])
        )
        page_keywords = " ".join(
            [
                p.get("hero", ""),
                p.get("entry", ""),
                p.get("overview_text", ""),
                " ".join(p.get("commands", [])),
                " ".join(p.get("source_files", [])),
                source_text,
                flow_text,
                issue_text,
            ]
        )
        search_data.append({
            "type": "CPU3流程",
            "title": p["title"],
            "domain": p["short"],
            "file": ", ".join(p.get("source_files", [])),
            "summary": p["summary"],
            "keywords": page_keywords,
            "href": p["file"],
        })
        for src in p.get("sources", []):
            search_data.append({
                "type": "源码",
                "title": src["title"],
                "domain": p["short"],
                "file": " ".join(src.get("refs", [])),
                "summary": src["desc"],
                "keywords": " ".join([p["summary"], page_keywords]),
                "href": p["file"] + "#source",
            })
    return (
        '<!doctype html><html lang="zh-CN"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width, initial-scale=1">'
        f'<title>CPU3 程序流程总览</title><link rel="stylesheet" href="assets/流程文档样式.css?v={CSS_VERSION}">'
        '<link rel="stylesheet" href="../assets/网站嵌入增强.css"></head>'
        f'<body class="cube-flow-page" data-cube-flow-page="{esc(page_identifier)}"><div class="wrap"><header class="hero"><h1>CPU3 程序流程总览</h1>'
        '<p>按 CPU2 文档同等标准整理 CPU3 显示端、协议网关、参数镜像、按键菜单和外设恢复流程。所有页面均保留一套业务级程序流程图，不把命令流程和代码梳理拆开。</p>'
        f'<div class="meta-grid"><div class="meta"><span>项目</span><strong>LTD_DISPLAY_CPU3 / CPU3 {esc(CPU3_VERSION)} / 协议 {DEVICE_PROTOCOL_VERSION}</strong></div><div class="meta"><span>页面数量</span><strong>'
        + str(len(pages))
        + f'</strong></div><div class="meta"><span>整理日期</span><strong>{DOC_DATE}</strong></div><div class="meta"><span>文档风格</span><strong>业务级 SVG + 源码证据</strong></div></div></header>'
        '<nav class="topnav"><a href="#overview">总览图</a><a href="#pages">页面入口</a><a href="#search">全文索引</a><a href="../../00_构建与版本/CPU3文档索引.md">CPU3 索引</a></nav>'
        '\n<!-- CUBE_EMBED_START -->\n'
        '<section id="overview"><h2>1. CPU3 总体业务流</h2><p class="lead">CPU3 的核心职责不是直接测量，而是在显示端把 CPU2 状态、外部协议、菜单参数和本地持久化连接起来。</p>'
        + flow_svg(
            "cpu3-all",
            "CPU3 总体业务流",
            nodes,
            edges,
            height=740,
            accessible_id=f"{page_identifier}-diagram-01",
            description=description,
        )
        + '</section><section id="pages"><h2>2. 文档入口</h2><div class="overview-grid">'
        + cards
        + '</div></section><section id="search"><h2>3. 全文索引</h2><input class="doc-search" data-global-search placeholder="输入指令、函数、文件、状态或问题关键词"><div class="search-results" data-global-search-results></div></section>'
        '\n<!-- CUBE_EMBED_END -->\n'
        f'<script type="application/json" id="doc-search-data">{json.dumps(search_data, ensure_ascii=False).replace("</", "<\\/")}</script>'
        '</div><script src="assets/流程文档交互.js"></script>'
        '<script src="../assets/网站嵌入增强.js"></script></body></html>'
    )


def readme(pages: list[dict]) -> str:
    rows = "\n".join(f"| [{p['short']}]({p['file']}) | {p['summary']} |" for p in pages)
    return (
        "# CPU3 程序流程文档\n\n"
        f"更新日期：{DOC_DATE}\n\n"
        "本目录按 CPU2 程序流程文档同等标准整理 `LTD_DISPLAY_CPU3`。每个页面只保留一套业务级程序流程，命令入口、代码处理、异常出口和源码依据放在同一页面中。\n\n"
        "| 文档 | 内容 |\n| --- | --- |\n"
        f"| [CPU3 程序流程总览](CPU3程序流程总览.html) | CPU3 上电、主循环、外部协议、CPU2 轮询、显示和参数持久化总览 |\n{rows}\n\n"
        "说明：文档为静态 HTML，样式和交互资源位于 `assets/`。流程图以业务动作和判断条件为主，函数名保留在源码索引和证据段中。\n"
    )


PAGES: list[dict] = [
    {
        "id": "cpu3-01",
        "file": "01_启动主循环与调度.html",
        "title": "CPU3 启动主循环与调度程序流程",
        "short": "启动主循环与调度",
        "hero": "梳理 CPU3 从 HAL/CubeMX 初始化到 App_Init、App_MainLoop 的调度顺序，以及显示任务、外部 COM 和 CPU2 轮询之间的优先级。",
        "entry": "main() 上电后初始化外设并进入 while(1)，每轮执行 App_MainLoop()。",
        "summary": "主循环先处理串口重配、显示任务和 COM1/COM2/COM3 外部协议帧，再检查 CPU2 的 100 ms 调度门限；门限可防止纯外部流量永久饿死轮询，但同步请求阻塞期间不保证实际间隔。",
        "overview_text": "CPU3 主循环是显示端的调度中心。它不是 RTOS 任务，而是单个 while(1) 中按固定优先级轮询不同业务。",
        "overview_description": "CPU3 上电完成 HAL、时钟与外设初始化后，App_Init 加载显示、RTC、FRAM 通信参数并配置三路串口和 DSM 地址；随后 App_MainLoop 每轮先处理串口重配、显示和三路外部协议。距上次 CPU2 轮询达到 100 ms 才发起同步请求，单次最长等待 1000 ms；未到门限且无其它工作时延时 1 ms，再进入下一轮。",
        "commands": [],
        "source_files": ["Core/Src/main.c", "Application/app_main.c", "Core/Src/usart.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 440, "y": 40, "w": 240, "h": 62, "label": "CPU3 上电进入启动入口"},
            {"id": "b", "kind": "process", "x": 350, "y": 145, "w": 420, "h": 82, "label": "HAL、系统时钟和 GPIO/DMA/UART/SPI/TIM/IWDG 初始化"},
            {"id": "c", "kind": "process", "x": 350, "y": 275, "w": 420, "h": 82, "label": "应用初始化显示、RTC、FRAM 参数、串口配置和 DSM 地址"},
            {"id": "d", "kind": "loop", "x": 350, "y": 405, "w": 420, "h": 82, "label": "主循环周期执行应用调度"},
            {"id": "e", "kind": "decision", "x": 350, "y": 535, "w": 420, "h": 110, "label": "主循环可调度时，距离上次 CPU2 轮询是否达到 100 ms 门限？"},
            {"id": "f", "kind": "process", "x": 90, "y": 690, "w": 300, "h": 86, "label": "达到门限：轮询 CPU2；单次请求最长可同步等待 1000 ms"},
            {"id": "g", "kind": "process", "x": 730, "y": 690, "w": 300, "h": 86, "label": "未达门限：本轮无工作时延时 1 ms"},
        ],
        "overview_edges": [
            {"d": "M 560 102 L 560 145"},
            {"d": "M 560 227 L 560 275"},
            {"d": "M 560 357 L 560 405"},
            {"d": "M 560 487 L 560 535"},
            {"d": "M 350 590 C 250 620 230 650 240 690", "label": "达到"},
            {"d": "M 770 590 C 870 620 890 650 880 690", "label": "未达到"},
            {"d": "M 240 776 C 300 825 500 840 560 645"},
            {"d": "M 880 776 C 820 825 620 840 560 645"},
        ],
        "flows": [
            {
                "title": "上电初始化到主循环",
                "caption": "把 main() 中的 CubeMX 外设初始化和 App_Init 的应用初始化串起来，区分硬件层初始化和 CPU3 应用状态初始化。",
                "height": 980,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "复位后进入启动入口"},
                    {"id": "n1", "kind": "process", "x": 360, "y": 125, "w": 400, "h": 78, "label": "清 HAL 状态并配置系统时钟"},
                    {"id": "n2", "kind": "process", "x": 360, "y": 245, "w": 400, "h": 86, "label": "初始化 GPIO、DMA、五路 UART、两路 SPI、TIM、CRC、IWDG"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 375, "w": 310, "h": 108, "label": "任一外设初始化失败？"},
                    {"id": "err", "kind": "error", "x": 780, "y": 392, "w": 250, "h": 80, "label": "Error_Handler 关中断后死循环"},
                    {"id": "n3", "kind": "process", "x": 360, "y": 535, "w": 400, "h": 86, "label": "应用初始化：显示 Logo、RTC 初始化、读取 CPU3 FRAM 参数"},
                    {"id": "n4", "kind": "process", "x": 360, "y": 675, "w": 400, "h": 86, "label": "按 FRAM 参数重配 COM1/2/3 并启动 DMA 接收"},
                    {"id": "n5", "kind": "process", "x": 360, "y": 815, "w": 400, "h": 78, "label": "DSM 初始化地址和寄存器边界，延时等待外设稳定"},
                    {"id": "e", "kind": "end", "x": 430, "y": 930, "w": 260, "h": 62, "label": "进入应用主循环"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 125"},
                    {"d": "M 560 203 L 560 245"},
                    {"d": "M 560 331 L 560 375"},
                    {"d": "M 715 429 L 780 432", "label": "是", "red": True},
                    {"d": "M 560 483 L 560 535", "label": "否"},
                    {"d": "M 560 621 L 560 675"},
                    {"d": "M 560 761 L 560 815"},
                    {"d": "M 560 893 L 560 930"},
                ],
                "evidence": [
                    {"title": "硬件初始化", "text": f"{code('Core/Src/main.c:70-116')} 中 main() 依次初始化 GPIO/DMA/UART/SPI/TIM/CRC/IWDG 后调用 App_Init。"},
                    {"title": "应用初始化", "text": f"{code('Application/app_main.c:367-381')} 显示 Logo、初始化 RTC、从 FRAM 读取 CPU3 本机参数、重配串口、初始化 DSM 通信。"},
                    {"title": "错误出口", "text": f"{code('Core/Src/main.c:185-193')} Error_Handler 关闭中断并进入 while(1)。"},
                ],
            },
            {
                "title": "App_MainLoop 调度优先级",
                "caption": "主循环先处理串口重配置、SI 周期任务、显示和三路外部 COM；完成这些工作后检查 CPU2 的 100 ms 调度门限。门限能防止持续外部流量永久饿死轮询，但同步请求自身仍可阻塞 1000 ms。",
                "height": 940,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 440, "y": 35, "w": 240, "h": 62, "label": "进入应用主循环"},
                    {"id": "n1", "kind": "process", "x": 360, "y": 120, "w": 400, "h": 78, "label": "处理串口重配、SI 周期任务和显示任务"},
                    {"id": "p1", "kind": "process", "x": 360, "y": 235, "w": 400, "h": 78, "label": "处理就绪的 COM1 外部协议帧（如有）"},
                    {"id": "p2", "kind": "process", "x": 360, "y": 350, "w": 400, "h": 78, "label": "处理就绪的 COM2 外部协议帧（如有）"},
                    {"id": "p3", "kind": "process", "x": 360, "y": 465, "w": 400, "h": 78, "label": "处理就绪的 COM3 外部协议帧（如有）"},
                    {"id": "d4", "kind": "decision", "x": 350, "y": 590, "w": 420, "h": 110, "label": "距离上次 CPU2 轮询是否达到 100 ms 调度门限？"},
                    {"id": "poll", "kind": "process", "x": 70, "y": 760, "w": 300, "h": 90, "label": "达到：轮询 CPU2；同步请求最长可阻塞 1000 ms"},
                    {"id": "e", "kind": "end", "x": 750, "y": 770, "w": 300, "h": 70, "label": "未达到：无工作则延时 1 ms，返回下一轮"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 120"},
                    {"d": "M 560 198 L 560 235"},
                    {"d": "M 560 313 L 560 350"},
                    {"d": "M 560 428 L 560 465"},
                    {"d": "M 560 543 L 560 590"},
                    {"d": "M 350 645 C 250 690 220 720 220 760", "label": "达到"},
                    {"d": "M 770 645 C 870 690 900 730 900 770", "label": "未达到"},
                    {"d": "M 370 805 C 500 880 700 880 750 805"},
                ],
                "evidence": [
                    {"title": "调度顺序", "text": f"{code('Application/app_main.c:387-539')} 依次处理串口重配、SI 周期任务、Display_Task 和 COM1/COM2/COM3，再检查 CPU2 的 100 ms 调度门限。"},
                    {"title": "门限边界", "text": f"{code('Application/app_main.c:529-538')} 只在主循环可继续运行时检查门限；CPU2 同步请求执行期间不保证实际轮询间隔，did_work 为 0 时仅延时 1 ms。"},
                    {"title": "协议分发", "text": f"{code('Application/app_main.c:342-360')} 根据端口参数选择 DSM/Wartsila/SI/LTD 协议处理函数。"},
                ],
            },
        ],
        "issues": [
            {"level": "high", "title": "CPU2 轮询是同步阻塞等待响应", "desc": "CPU2_CombinatePackage_Send 内部使用 CPU2_RESPONSE_TIMEOUT_MS=1000ms 同步等待 wait_response；轮询期间外部 COM 和显示刷新都会被主循环阻塞。", "suggest": "把 CPU2 轮询改成非阻塞状态机，发送完成、接收完成和超时分别推进状态。", "ref": func_line(CPU2_COMM_SOURCE, "CPU2_CombinatePackage_Send")},
            {"level": "mid", "title": "100 ms 门限不是实时最大轮询间隔", "desc": "持续外部流量不会再永久饿死 CPU2 轮询，但外部协议处理发生在门限检查前，CPU2 同步请求本身也可阻塞 1000 ms，因此实际请求起始间隔可能明显超过 100 ms。", "suggest": "把 CPU2 请求改成非阻塞状态机，并在持续外部流量和断线场景记录实际最坏轮询间隔。", "ref": "Application/app_main.c:387-539"},
            {"level": "mid", "title": "1 ms 空闲节拍的实机负载尚未量化", "desc": "主循环空闲延时由约 100 ms 缩短为 1 ms 后，显示任务、SI 自动调度和 RTC 检查频率明显提高；静态检查和构建不能证明 CPU 占用、OLED 刷新或看门狗余量满足现场要求。", "suggest": "在空闲、持续外部流量、CPU2 离线和菜单刷新场景记录 CPU 占用、OLED/RTC 访问频率和看门狗状态。", "ref": "Application/app_main.c:387-539"},
            {"level": "mid", "title": "串口重配依赖三路端口完全空闲", "desc": "g_cpu3_uart_reinit_pending 在任一端口收发忙或 pending 时直接返回，如果端口持续有流量，重配可能长期不生效。", "suggest": "显示重配等待状态，增加超时或维护窗口机制，避免用户以为参数已应用。", "ref": "Application/app_main.c:182-221"},
            {"level": "low", "title": "启动延时固定 1000ms", "desc": "App_Init 最后固定 HAL_Delay(1000)，没有和外设实际就绪状态绑定。", "suggest": "如果现场启动时间敏感，可以改为检测显示/串口/CPU2 首次响应条件。", "ref": "Application/app_main.c:379-381"},
        ],
        "sources": [
            {"title": "main 主入口", "desc": "硬件初始化和 while(1) 调度入口。", "refs": ["Core/Src/main.c:70-116"]},
            {"title": "App_Init", "desc": "CPU3 应用层初始化入口。", "refs": ["Application/app_main.c:367-381"]},
            {"title": "App_MainLoop", "desc": "显示、外部 COM、CPU2 轮询的主调度函数。", "refs": ["Application/app_main.c:387-555"]},
            {"title": "cpu3_port_process", "desc": "按端口配置选择外部协议处理器。", "refs": ["Application/app_main.c:342-360"]},
        ],
    },
    {
        "id": "cpu3-02",
        "file": "02_CPU2内部通信与轮询.html",
        "title": "CPU3 与 CPU2 内部 Modbus 通信程序流程",
        "short": "CPU2 内部通信与轮询",
        "hero": "梳理 UART5/RS485 与 CPU2 的同步请求、响应解析、上电全量读取、运行输入轮询、参数更新补读、密度分布点读取、连续通信失败故障置位与状态帧恢复，以及协议 10 起的 RSSI 和协议 11 起的 AO 运行态尾段解析。",
        "entry": "App_MainLoop 在主循环可调度时按 100 ms 门限检查 CPU2 轮询；同步请求阻塞期间不保证实际间隔。菜单和外部协议写参数时通过内部 Modbus 组帧下发。",
        "summary": "CPU3 作为 CPU2 的 Modbus 主站，周期读取输入寄存器和保持寄存器，写指令/参数时通过 0x10 下发到 CPU2；连续第 10 个请求未获得合法响应时由 CPU3 本机置故障。完整状态 0x04 负责解除故障，覆盖协议字段的合法 0x03 负责建立当前连接协议快照，两者不能用掉线前缓存互相替代，且均完成后才退出通讯尝试页。",
        "overview_text": "内部通信链路由 UART5 + RS485 实现。功能码、帧形、CRC 和地址均有效的响应会清零连续失败；只有覆盖设备状态和错误码的 0x04 响应才建立状态快照或解除故障，只有覆盖协议字段的合法 0x03 才建立当前连接协议快照，二者均完成后才退出通讯尝试页。依赖 CPU2 的普通写入口必须等待状态、完整参数、当前连接协议三类快照完整且共享协议版本相等；参数刷新期间仅屏幕 CMD_CANCEL_MEASUREMENT 可绕过参数快照。任何已发起的非命令 0x10 写失败会立即关闭参数快照并强制补读四组，不依赖参数更新标志。超时、非法帧、UART 错误和 TX DMA 启动失败均累计，第 10 次设置 CPU2_COMM_TIMEOUT；上电分组只在成功后推进。",
        "overview_description": "CPU3 需要轮询或写入 CPU2 时，分别组装 0x03/0x04 读取或 0x10 写入帧，经 UART5 RS485 同步等待合法响应。0x03 回写参数并在覆盖协议字段时建立协议快照，完整 0x04 回写测量与运行状态并可解除通信故障；超时、非法帧、UART 或发送失败会累计，连续第 10 次置 CPU2 通信超时故障。",
        "commands": ["FUNCTIONCODE_READ_HOLDREGISTER", "FUNCTIONCODE_READ_INPUTREGISTER", "FUNCTIONCODE_WRITE_MULREGISTER"],
        "source_files": ["Communication/internal/main_board_modbus/cpu2_communicate.c", "Communication/internal/main_board_modbus/dataanalysis_modbus.c", "Application/system_param/stateformodbus.h"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "CPU3 需要访问 CPU2"},
            {"id": "b", "kind": "decision", "x": 405, "y": 145, "w": 310, "h": 108, "label": "是上电/运行轮询还是写参数/指令？"},
            {"id": "c", "kind": "process", "x": 90, "y": 310, "w": 300, "h": 90, "label": "轮询：按分组发送 03/04 读寄存器"},
            {"id": "d", "kind": "process", "x": 730, "y": 310, "w": 300, "h": 90, "label": "写入：组 0x10 多寄存器帧下发命令或参数"},
            {"id": "e", "kind": "process", "x": 360, "y": 485, "w": 400, "h": 86, "label": "UART5 切发送 DMA，等待 CPU2 响应"},
            {"id": "f", "kind": "decision", "x": 405, "y": 625, "w": 310, "h": 108, "label": "收到功能码、帧形、CRC 和地址均有效的响应？"},
            {"id": "g", "kind": "process", "x": 90, "y": 800, "w": 300, "h": 90, "label": "读保持：刷新参数缓存；覆盖协议字段时建立当前连接协议快照"},
            {"id": "h", "kind": "process", "x": 730, "y": 800, "w": 300, "h": 90, "label": "读输入：刷新测量状态缓存和运行态"},
            {"id": "i", "kind": "error", "x": 430, "y": 800, "w": 260, "h": 90, "label": "请求失败计数 +1；第 1～9 次重试，第 10 次置 CPU3 本机故障"},
        ],
        "overview_edges": [
            {"d": "M 560 102 L 560 145"},
            {"d": "M 405 199 C 260 230 230 270 240 310", "label": "轮询"},
            {"d": "M 715 199 C 860 230 890 270 880 310", "label": "写入"},
            {"d": "M 240 400 C 320 455 470 465 560 485"},
            {"d": "M 880 400 C 800 455 650 465 560 485"},
            {"d": "M 560 571 L 560 625"},
            {"d": "M 405 679 C 300 720 250 760 240 800", "label": "03"},
            {"d": "M 715 679 C 820 720 870 760 880 800", "label": "04"},
            {"d": "M 560 733 L 560 800", "label": "否/超时", "red": True},
        ],
        "flows": [
            {
                "title": "CPU2 分组轮询",
                "caption": "上电阶段读 7 组，包含输入寄存器和保持寄存器；运行阶段只读输入寄存器；参数更新标志变化或已发起的非命令 0x10 写失败后补读保持寄存器。失败写会同步关闭参数快照，强制补读不依赖参数更新标志；完整参数补读完成后刷新 Wartsila 寄存器镜像，批量参数同步永久跳过一次性命令字段。",
                "height": 1180,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "主循环可调度且达到 100 ms 门限时轮询 CPU2"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "上电全量读取完成？"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 285, "w": 300, "h": 90, "label": "上电未完成：读取当前启动组；仅合法响应后推进索引"},
                    {"id": "p2", "kind": "process", "x": 90, "y": 420, "w": 300, "h": 90, "label": "最后一组完成后建立参数快照，刷新 Wartsila 镜像并记录更新标志"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "有保持寄存器补读任务？"},
                    {"id": "p3", "kind": "process", "x": 730, "y": 285, "w": 300, "h": 90, "label": "按队列补读保持参数；完成后恢复参数快照并刷新 Wartsila 镜像"},
                    {"id": "p4", "kind": "process", "x": 405, "y": 520, "w": 310, "h": 90, "label": "运行轮询：设备状态、测量结果、继电器、RSSI 和 AO 运行态"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 660, "w": 310, "h": 108, "label": "CPU2 参数更新标志是否变化？"},
                    {"id": "p5", "kind": "state", "x": 730, "y": 675, "w": 300, "h": 80, "label": "标记需要补读保持参数，下轮开始刷新镜像"},
                    {"id": "d4", "kind": "decision", "x": 405, "y": 815, "w": 310, "h": 108, "label": "CPU2 状态是否为分布/区间测量完成？"},
                    {"id": "p6", "kind": "process", "x": 730, "y": 830, "w": 300, "h": 90, "label": "按测点数分批读取密度分布点，每帧最多 100 寄存器"},
                    {"id": "e", "kind": "end", "x": 430, "y": 1030, "w": 260, "h": 62, "label": "本轮轮询结束"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 405 184 C 280 220 240 250 240 285", "label": "否"},
                    {"d": "M 240 375 L 240 420"},
                    {"d": "M 390 465 C 480 485 520 500 560 520"},
                    {"d": "M 560 238 L 560 285", "label": "是"},
                    {"d": "M 715 339 L 730 330", "label": "是"},
                    {"d": "M 560 393 L 560 520", "label": "否"},
                    {"d": "M 560 610 L 560 660"},
                    {"d": "M 715 714 L 730 715", "label": "是"},
                    {"d": "M 560 768 L 560 815", "label": "否"},
                    {"d": "M 715 869 L 730 875", "label": "是"},
                    {"d": "M 560 923 L 560 1030", "label": "否"},
                    {"d": "M 880 920 C 820 990 700 1020 560 1030"},
                    {"d": "M 880 755 C 780 790 680 790 560 815"},
                ],
                "evidence": [
                    {"title": "轮询分组", "text": f"{code(f'{CPU2_COMM_SOURCE}:287-347')} 定义上电、运行和参数更新补读三类 PollGroup。"},
                    {"title": "状态机", "text": f"{code(func_line(CPU2_COMM_SOURCE, 'PollingInputData'))} PollingInputData 维护 poweron_done、runtime_index、hold_refresh_pending 等静态状态，请求失败不推进分组；故障恢复优先读取设备状态，随后重新建立参数和当前连接协议快照；补读完成刷新 Wartsila 镜像。"},
                    {"title": "密度点读取", "text": f"{code(func_line(CPU2_COMM_SOURCE, 'RequestDensityDistPoints_ByCount'))} RequestDensityDistPoints_ByCount 按点数和每点寄存器数分批读取。"},
                ],
            },
            {
                "title": "CPU2 请求发送与响应解析",
                "caption": "CPU3 组帧后切 UART5 到 RS485 发送；1000 ms 超时、非法响应、UART 错误和 TX DMA 启动失败均按一次请求失败累计；合法响应清零连续失败，但已锁存故障仍需设备状态 0x04 响应恢复。故障会清参数和当前连接协议快照，状态恢复后必须重新收到覆盖协议字段的合法 0x03。已发起的非命令 0x10 写失败还会立即关闭参数快照并排队强制全量补读，阻止同轮其它 COM 普通命令。",
                "height": 1120,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "需要向 CPU2 发读/写请求"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 135, "w": 420, "h": 86, "label": "按地址、功能码、起始地址、数量和数据组 Modbus RTU 帧"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 275, "w": 310, "h": 108, "label": "UART5 TX DMA 启动成功？"},
                    {"id": "err1", "kind": "error", "x": 790, "y": 290, "w": 250, "h": 82, "label": "失败：切回接收并累计一次请求失败"},
                    {"id": "p2", "kind": "process", "x": 350, "y": 440, "w": 420, "h": 86, "label": "等待 CPU2 响应或超时"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 580, "w": 310, "h": 108, "label": "超时、UART 错误或响应帧非法？"},
                    {"id": "err2", "kind": "error", "x": 790, "y": 596, "w": 250, "h": 82, "label": "失败计数 +1；非命令写另关闭参数快照并强制补读"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 735, "w": 310, "h": 108, "label": "响应功能码是 03/04/10？"},
                    {"id": "p3", "kind": "process", "x": 70, "y": 900, "w": 300, "h": 90, "label": "03：刷新参数；覆盖协议字段时建立当前连接协议快照"},
                    {"id": "p4", "kind": "process", "x": 410, "y": 900, "w": 300, "h": 90, "label": "04：解析输入寄存器，刷新测量状态缓存"},
                    {"id": "p5", "kind": "state", "x": 750, "y": 900, "w": 300, "h": 90, "label": "10：校验写回地址/数量，标记有效响应并清失败计数"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 135"},
                    {"d": "M 560 221 L 560 275"},
                    {"d": "M 715 329 L 790 331", "label": "否", "red": True},
                    {"d": "M 560 383 L 560 440", "label": "是"},
                    {"d": "M 560 526 L 560 580"},
                    {"d": "M 715 634 L 790 637", "label": "是", "red": True},
                    {"d": "M 560 688 L 560 735", "label": "否"},
                    {"d": "M 405 789 C 260 830 220 860 220 900", "label": "03"},
                    {"d": "M 560 843 L 560 900", "label": "04"},
                    {"d": "M 715 789 C 860 830 900 860 900 900", "label": "10"},
                ],
                "evidence": [
                    {"title": "通信状态", "text": f"{code('cpu2_communicate.c:21-204')} 状态快照、完整参数快照、当前连接协议快照、连续请求失败和通信故障锁存由通信模块私有维护；普通写要求三类快照完整且协议版本相等，屏幕取消仅放宽参数快照。"},
                    {"title": "组帧与等待", "text": f"{code(func_line(CPU2_COMM_SOURCE, 'CPU2_CombinatePackage_Send'))} CPU2_CombinatePackage_Send 返回本次请求结果；超时、非法帧、UART 错误和 TX DMA 启动失败统一调用 CPU2_CommRecordFailure。"},
                    {"title": "中断边界", "text": f"{code('Application/app_main.c:650-705')} UART5 错误回调只置待处理标志并重启接收，故障计数和状态更新在主循环完成。"},
                    {"title": "响应与恢复", "text": f"{code(func_line(CPU2_COMM_SOURCE, 'HostCommuProcess'))} 分发合法响应；{code(func_line(CPU2_COMM_SOURCE, 'CPU2_Response03Process'))} 和 {code(func_line(CPU2_COMM_SOURCE, 'CPU2_Response04Process'))} 分别重建协议快照与处理状态恢复。通信故障已清参数和协议快照，随后必须重跑完整参数同步。"},
                    {"title": "写结果不确定", "text": f"{code(func_line(CPU2_COMM_SOURCE, 'CPU2_CommFinishFailedRequest'))} 和 {code(func_line(CPU2_COMM_SOURCE, 'CPU2_CombinatePackage_Send'))} 在已发起的非命令 0x10 写任一失败出口关闭参数快照并请求四组补读；精确命令地址加 2 寄存器不触发参数补读。"},
                ],
            },
            {
                "title": "输入寄存器尾段解析",
                "caption": "CPU2 在继电器运行态后追加无线 RSSI，协议 11 起再追加 AO 运行态；CPU3 必须按相同顺序读尾段，否则后续字段整体错位。",
                "height": 1160,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "04 响应进入输入寄存器解析"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 125, "w": 420, "h": 82, "label": "先解析设备状态、液位/水位/密度、调试数据和继电器运行态"},
                    {"id": "p2", "kind": "state", "x": 350, "y": 250, "w": 420, "h": 86, "label": "读取无线配对状态：连接、RSSI、错误码和更新计数"},
                    {"id": "p3", "kind": "state", "x": 350, "y": 385, "w": 420, "h": 86, "label": "继续读取 AO 输出运行态 9 个字段"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 535, "w": 310, "h": 108, "label": "CPU2/CPU3 协议版本是否严格一致？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 550, "w": 250, "h": 82, "label": "不一致：状态页提示协议不兼容，不可信尾段数据"},
                    {"id": "p4", "kind": "process", "x": 70, "y": 720, "w": 300, "h": 94, "label": "读取部件参数页使用 RSSI 字段显示 RSSI 或 N/A"},
                    {"id": "p5", "kind": "process", "x": 410, "y": 720, "w": 300, "h": 94, "label": "CPU3 缓存 AO 运行态；状态页未独立显示目标电流/来源"},
                    {"id": "p6", "kind": "process", "x": 750, "y": 720, "w": 300, "h": 94, "label": "AD5421 故障码由 CPU3 文案表翻译显示"},
                    {"id": "end", "kind": "end", "x": 430, "y": 940, "w": 260, "h": 62, "label": "RSSI 供状态页显示；AO运行态先进入缓存"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 125"},
                    {"d": "M 560 207 L 560 250"},
                    {"d": "M 560 336 L 560 385"},
                    {"d": "M 560 471 L 560 535"},
                    {"d": "M 715 589 L 790 591", "label": "否", "red": True},
                    {"d": "M 405 589 C 270 635 230 680 220 720", "label": "是/RSSI"},
                    {"d": "M 560 643 L 560 720", "label": "是/AO"},
                    {"d": "M 715 589 C 825 635 890 680 900 720", "label": "是/错误码"},
                    {"d": "M 220 814 C 320 900 470 930 560 940"},
                    {"d": "M 560 814 L 560 940"},
                    {"d": "M 900 814 C 800 900 650 930 560 940"},
                ],
                "evidence": [
                    {"title": "AO 尾段定义", "text": f"{code('Application/system_param/stateformodbus.h:402-415')} 在 RSSI 更新计数之后定义 REG_AO_OUTPUT_RUNTIME_*，REG_ENG 顺延到 AO 尾段末尾。"},
                    {"title": "CPU3 尾段解析", "text": f"{code('dataanalysis_modbus.c:103-113,493-623')} 先解析 RSSI 字段，再调用 read_ao_output_runtime_from_regs 写入 g_measurement.ao_output_runtime。"},
                    {"title": "协议版本", "text": f"{code('Application/system_param/system_parameter.h:29')} 当前 DEVICE_PROTOCOL_VERSION 为 {DEVICE_PROTOCOL_VERSION}，CPU3 使用严格相等判断协议兼容。"},
                ],
            },
        ],
        "issues": [
            {"level": "high", "title": "等待响应期间阻塞主循环", "desc": "CPU2_CombinatePackage_Send 使用 while(wait_response) 同步等待，CPU2 断线时单次最多阻塞 1000 ms，外部协议响应、显示刷新和按键处理都会延迟；100 ms 调度门限只能防止轮询被纯外部流量永久饿死，不能保证实际请求间隔。", "suggest": "改成 UART5 请求状态机：发送完成、接收完成、超时分别由状态推进；主循环每轮只推进一次。", "ref": func_line(CPU2_COMM_SOURCE, "CPU2_CombinatePackage_Send")},
            {"level": "high", "title": "密度点读取在一个函数内连续发送多帧", "desc": "RequestDensityDistPoints_ByCount 的 while(total_regs > 0) 会连续调用同步发送，多点数据量大时可能长时间占用主循环。", "suggest": "把密度点读取拆成分帧状态机，每轮只发一帧，并记录已读 offset。", "ref": func_line(CPU2_COMM_SOURCE, "RequestDensityDistPoints_ByCount")},
            {"level": "mid", "title": "DSM/Wartsila 多字段写可能部分生效", "desc": "外部写失败反馈已统一：DSM、Wartsila 和 SI 在 CPU2 不可用或写入失败时返回设备忙 0x06，SI 单字段影子只在 CPU2 ACK 后提交。但 DSM/Wartsila 多字段写仍是逐字段同步，若中途失败，失败前已被 CPU2 接受的字段无法事务回滚。", "suggest": "主站收到 0x06 后应重试并回读确认；现场尽量避免把多个独立参数放在一个写事务中。若后续要求原子语义，需要在 CPU2 增加批量校验和一次性提交机制。", "ref": "Communication/external/DSM_modbus; Communication/external/wartsila_modbus; Communication/external/si_modbus"},
            {"level": "mid", "title": "输入寄存器尾段强依赖两端协议版本一致", "desc": "协议 10 起增加 RSSI，协议 11 起在 RSSI 后继续追加 AO 运行态。只要 CPU2/CPU3 任何一端协议不一致，尾段字段都会错位。", "suggest": "保持协议版本严格相等提示，并在现场升级清单里要求 CPU2/CPU3 成对升级。", "ref": "stateformodbus.h:397-415"},
            {"level": "mid", "title": "03 响应解析前先 WriteDeviceParamsToHoldingRegisters", "desc": "03 处理先把本地 g_deviceParams 写入 HoldingRegisterArray，再覆盖响应区间，若响应只是局部参数，未读区仍是本地旧镜像。", "suggest": "明确 HoldingRegisterArray 的主数据源，避免局部读时混入旧值；必要时增加脏区标记。", "ref": func_line(CPU2_COMM_SOURCE, "CPU2_Response03Process")},
        ],
        "sources": [
            {"title": "PollingInputData", "desc": "CPU2 上电、参数/当前连接协议快照和运行轮询状态机。", "refs": [func_line(CPU2_COMM_SOURCE, "PollingInputData")]},
            {"title": "CPU2_CombinatePackage_Send", "desc": "内部 Modbus 主站组帧、发送、等待、请求结果和连续失败故障置位入口。", "refs": [func_line(CPU2_COMM_SOURCE, "CPU2_CombinatePackage_Send")]},
            {"title": "HostCommuProcess", "desc": "CPU2 响应帧 CRC/地址/功能码分发。", "refs": [func_line(CPU2_COMM_SOURCE, "HostCommuProcess")]},
            {"title": "read_measurement_result_from_InputRegisters", "desc": "输入寄存器到 g_measurement 的字段映射。", "refs": ["Communication/internal/main_board_modbus/dataanalysis_modbus.c:493-626"]},
        ],
    },
]


def append_remaining_pages() -> None:
    PAGES.extend([
        external_com_page(),
        dsm_page(),
        wartsila_si_page(),
        display_page(),
        menu_page(),
        param_fram_io_page(),
    ])


def external_com_page() -> dict:
    return {
        "id": "cpu3-03",
        "file": "03_外部COM协议分发与RS485发送.html",
        "title": "CPU3 外部 COM 协议分发与 RS485 发送程序流程",
        "short": "外部 COM 协议分发",
        "hero": "梳理 COM1/COM2/COM3 从 DMA 空闲收帧、统一协议切换管理帧优先解析、业务协议分发，到 DMA 发送、切换落盘和错误恢复的完整流程。",
        "entry": "USART6/USART2/USART3 空闲中断置 comX_rx_ready，App_MainLoop 先识别统一协议切换管理帧，未命中时才调用当前业务协议处理器。",
        "summary": "三路外部 COM 在当前业务协议之前共用统一切换帧入口；合法请求先按旧串口参数返回独立 ACK，最后一帧发送完成后保持 RX 停止，主循环完成 FRAM 写后读回再只重初始化当前 COM 口。普通帧继续按本地配置分发到 DSM、Wartsila、SI 或 LTD。",
        "overview_text": "外部 COM 是 CPU3 对上位系统的主要入口。统一管理帧不依赖当前业务协议，未命中管理帧时才按端口配置选择业务协议。",
        "overview_description": "COM1/2/3 收到完整帧后，CPU3 先调用 ProtocolSwitchFrame_Process 校验统一管理帧。合法目标返回与请求不同的独立 ACK 并暂存切换，非法目标返回统一异常帧；两者均不进入业务协议。切换 ACK 最后一帧按旧参数发送完成后，中断只标记 APPLY 并保持 RX DMA 停止，主循环完成 FRAM 写后读回；校验通过才启用新参数，失败则恢复旧配置。",
        "commands": ["COM_PROTO_DSM", "COM_PROTO_WARTSILA", "COM_PROTO_SI", "COM_PROTO_LTD"],
        "source_files": ["Application/app_main.c", "Communication/common/protocol_switch_frame.c", "Communication/external/ltd_modbus/ltd_modbus_slave.c", "Application/system_param/cpu3_comm_display_params.c", "Core/Src/stm32f4xx_it.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "COM1/2/3 收到完整 RTU 帧"},
            {"id": "b", "kind": "process", "x": 350, "y": 150, "w": 420, "h": 82, "label": "先检查挂起重配安全点\n再优先解析统一协议切换管理帧"},
            {"id": "c", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "已有管理帧响应，或业务协议处理器存在？"},
            {"id": "d", "kind": "error", "x": 790, "y": 300, "w": 250, "h": 82, "label": "无协议/越界：不回包并恢复接收"},
            {"id": "e", "kind": "process", "x": 350, "y": 450, "w": 420, "h": 82, "label": "管理帧独立 ACK/异常，或调用 DSM/Wartsila/SI/LTD 生成响应帧"},
            {"id": "f", "kind": "decision", "x": 405, "y": 585, "w": 310, "h": 108, "label": "tx_len 是否大于 0？"},
            {"id": "g", "kind": "process", "x": 90, "y": 750, "w": 300, "h": 82, "label": "无响应：立即切接收并重启 DMA"},
            {"id": "h", "kind": "process", "x": 730, "y": 750, "w": 300, "h": 82, "label": "有响应：尝试 DMA 发送，忙则进入单帧 pending"},
        ],
        "overview_edges": [
            {"d": "M 560 102 L 560 150"},
            {"d": "M 560 232 L 560 285"},
            {"d": "M 715 339 L 790 341", "label": "否", "red": True},
            {"d": "M 560 393 L 560 450", "label": "是"},
            {"d": "M 560 532 L 560 585"},
            {"d": "M 405 639 C 300 690 250 720 240 750", "label": "否"},
            {"d": "M 715 639 C 820 690 870 720 880 750", "label": "是"},
        ],
        "flows": [
            {
                "title": "三路 COM 收帧与协议分发",
                "caption": "COM1/COM2/COM3 都先识别统一管理帧；命中后直接生成独立 ACK 或异常响应，未命中才读取当前协议并进入 DSM/Wartsila/SI/LTD 业务处理器。",
                "height": 1240,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "外部端口收到完整帧？"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 130, "w": 420, "h": 82, "label": "清接收标志，标记本轮已处理并取接收缓存"},
                    {"id": "d0", "kind": "decision", "x": 405, "y": 260, "w": 310, "h": 108, "label": "统一协议切换管理帧是否命中？"},
                    {"id": "sw", "kind": "state", "x": 70, "y": 430, "w": 320, "h": 98, "label": "合法：返回独立 ACK 并暂存目标\n非法目标：返回 0xC6/0x03 异常"},
                    {"id": "p2", "kind": "process", "x": 730, "y": 430, "w": 300, "h": 82, "label": "未命中：读取当前端口协议配置"},
                    {"id": "d1", "kind": "decision", "x": 725, "y": 570, "w": 310, "h": 108, "label": "协议号越界或 handler 为空？"},
                    {"id": "err", "kind": "error", "x": 70, "y": 585, "w": 320, "h": 82, "label": "处理失败：不回包并恢复接收 DMA"},
                    {"id": "p3", "kind": "process", "x": 670, "y": 730, "w": 420, "h": 82, "label": "执行业务协议处理：DSM / Wartsila / SI / LTD"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 880, "w": 310, "h": 108, "label": "是否生成响应帧？"},
                    {"id": "p4", "kind": "process", "x": 90, "y": 1050, "w": 300, "h": 82, "label": "无响应：切回接收模式并重启 DMA"},
                    {"id": "p5", "kind": "process", "x": 730, "y": 1050, "w": 300, "h": 82, "label": "有响应：立即发送或进入待发队列"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 560 212 L 560 260"},
                    {"d": "M 405 314 C 300 360 250 400 230 430", "label": "是"},
                    {"d": "M 715 314 C 820 360 860 400 880 430", "label": "否"},
                    {"d": "M 880 512 L 880 570"},
                    {"d": "M 725 624 C 600 640 430 640 390 626", "label": "是", "red": True},
                    {"d": "M 880 678 L 880 730", "label": "否"},
                    {"d": "M 880 812 C 790 840 680 860 560 880"},
                    {"d": "M 230 528 C 260 760 360 840 560 880"},
                    {"d": "M 405 934 C 300 980 250 1020 240 1050", "label": "否"},
                    {"d": "M 715 934 C 820 980 870 1020 880 1050", "label": "是"},
                ],
                "evidence": [
                    {"title": "统一管理帧入口", "text": f"{code(func_line('Application/app_main.c', 'cpu3_port_process'))} 先调用 ProtocolSwitchFrame_Process；命中后不再进入当前业务协议。"},
                    {"title": "切换帧解析", "text": f"{code(func_line('Communication/common/protocol_switch_frame.c', 'ProtocolSwitchFrame_Process'))} 校验固定帧、站号、目标协议和 CRC，并生成与请求不同的独立 ACK 或异常响应。"},
                    {"title": "业务协议表", "text": f"{code('Application/app_main.c:330-340')} g_handlers 将 DSM、Wartsila、SI、LTD 分别绑定到处理函数；仅管理帧未命中时使用。"},
                ],
            },
            {
                "title": "挂起串口重配安全点",
                "caption": "CPU3 菜单或参数写入可能只置位 UART 重配请求；真正重配在 App_MainLoop 开头执行，且必须确认三路外部 COM 没有接收就绪、发送忙和 pending 待发帧。",
                "height": 1120,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "主循环发现串口重配请求"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 135, "w": 310, "h": 108, "label": "任一 COM 接收帧已就绪？"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 300, "w": 310, "h": 108, "label": "任一 COM 正在发送？"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 465, "w": 310, "h": 108, "label": "任一 pending_len 仍有待发帧？"},
                    {"id": "hold", "kind": "state", "x": 780, "y": 300, "w": 260, "h": 90, "label": "不安全：保留 pending 标志，推迟到下一轮主循环"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 640, "w": 420, "h": 82, "label": "安全：清重配标志，停止三路 RX DMA"},
                    {"id": "p2", "kind": "process", "x": 350, "y": 775, "w": 420, "h": 82, "label": "按 CPU3 本机通信参数重配 COM1/2/3"},
                    {"id": "p3", "kind": "process", "x": 350, "y": 910, "w": 420, "h": 82, "label": "三路切回接收模式并重启 DMA 接收"},
                    {"id": "end", "kind": "end", "x": 430, "y": 1040, "w": 260, "h": 62, "label": "继续显示任务和外部 COM 分发"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 135"},
                    {"d": "M 715 189 C 755 210 790 250 910 300", "label": "是"},
                    {"d": "M 560 243 L 560 300", "label": "否"},
                    {"d": "M 715 354 L 780 345", "label": "是"},
                    {"d": "M 560 408 L 560 465", "label": "否"},
                    {"d": "M 715 519 C 755 500 810 430 910 390", "label": "是"},
                    {"d": "M 560 573 L 560 640", "label": "否"},
                    {"d": "M 560 722 L 560 775"},
                    {"d": "M 560 857 L 560 910"},
                    {"d": "M 560 992 L 560 1040"},
                    {"d": "M 910 390 C 910 720 780 940 560 1040"},
                ],
                "evidence": [
                    {"title": "安全条件", "text": f"{code('Application/app_main.c:182-205')} cpu3_apply_uart_reinit_if_pending 依次检查接收 ready、发送 busy 和 pending_len。"},
                    {"title": "调度位置", "text": f"{code('Application/app_main.c:398-405')} App_MainLoop 在 Display_Task 和 COM 分发之前先处理挂起重配。"},
                    {"title": "重配动作", "text": f"{code(func_line(FRAM_SOURCE, 'Cpu3_ReinitAllUarts'))} Cpu3_ReinitAllUarts 根据本机通信参数重配三路外部 COM。"},
                ],
            },
            {
                "title": "DMA 发送、pending 队列与回接收",
                "caption": "发送通道忙时只缓存一帧 pending；续发失败会取消待切换状态。切换 ACK 最后一帧完成后，中断只标记 APPLY 并保持 RX 停止，主循环再保存并重配当前端口。",
                "height": 1220,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "外部协议是否生成响应帧？"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 140, "w": 310, "h": 108, "label": "当前发送通道是否空闲？"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 310, "w": 300, "h": 90, "label": "空闲：置 busy，停 RX DMA，切发送模式，启动 TX DMA"},
                    {"id": "p2", "kind": "state", "x": 730, "y": 310, "w": 300, "h": 90, "label": "忙：复制到 pending_buf，若已有 pending 则覆盖并计数"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 470, "w": 310, "h": 108, "label": "TX DMA 启动或续发成功？"},
                    {"id": "err", "kind": "error", "x": 790, "y": 485, "w": 250, "h": 82, "label": "失败：取消待切换状态，释放 busy 并恢复接收 DMA"},
                    {"id": "p3", "kind": "process", "x": 350, "y": 640, "w": 420, "h": 82, "label": "HAL_UART_TxCpltCallback 进入发送完成处理"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 780, "w": 310, "h": 108, "label": "是否还有 pending_len？"},
                    {"id": "p4", "kind": "process", "x": 90, "y": 935, "w": 300, "h": 82, "label": "有待发帧：继续发送，不切回接收"},
                    {"id": "p5", "kind": "process", "x": 730, "y": 935, "w": 300, "h": 82, "label": "切换 ACK 完成：清忙、标记 APPLY，保持 RX DMA 停止"},
                    {"id": "p6", "kind": "end", "x": 730, "y": 1080, "w": 300, "h": 82, "label": "主循环保存目标协议并只重配当前 COM 口"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 140"},
                    {"d": "M 405 194 C 300 245 250 280 240 310", "label": "是"},
                    {"d": "M 715 194 C 820 245 870 280 880 310", "label": "否"},
                    {"d": "M 240 400 C 330 430 450 445 560 470"},
                    {"d": "M 880 400 C 790 430 670 445 560 470"},
                    {"d": "M 715 524 L 790 526", "label": "否", "red": True},
                    {"d": "M 560 578 L 560 640", "label": "是"},
                    {"d": "M 560 722 L 560 780"},
                    {"d": "M 405 834 C 300 875 250 910 240 935", "label": "是"},
                    {"d": "M 715 834 C 820 875 870 910 880 935", "label": "否"},
                    {"d": "M 240 935 C 270 780 360 740 560 780"},
                    {"d": "M 880 1017 L 880 1080", "label": "有切换"},
                ],
                "evidence": [
                    {"title": "发送或排队", "text": f"{code('Application/app_main.c:226-279')} uart_try_send_or_queue 实现空闲发送、忙时单帧 pending 和覆盖计数。"},
                    {"title": "发送完成", "text": f"{code(func_line('Application/app_main.c', 'HAL_UART_TxCpltCallback'))} TxCplt 回调先续发 pending；最后一帧完成后才把切换状态标记为 APPLY。"},
                    {"title": "主循环应用", "text": f"{code(func_line('Application/app_main.c', 'cpu3_apply_ready_protocol_switches'))} 在非中断上下文保存目标协议、套用标准串口参数并只重配当前端口。"},
                    {"title": "错误恢复", "text": f"{code('Application/app_main.c:662-700')} ErrorCallback 对三路外部 COM 和 UART5 分别恢复。"},
                ],
            },
        ],
        "issues": [
            {"level": "mid", "title": "pending 队列只有一帧且覆盖旧帧", "desc": "外部请求密集时，新响应会覆盖旧 pending，虽然有覆盖计数，但上位机只会丢响应。", "suggest": "将单帧 pending 改成环形队列，或在忙时直接返回设备忙异常帧。", "ref": "Application/app_main.c:226-279"},
            {"level": "mid", "title": "协议号依赖枚举连续作为数组下标", "desc": "g_handlers 用 COM_PROTO_* 作为数组下标，若枚举不连续或后续新增值未同步表项，会进入不回包路径。", "suggest": "改成 switch 分发或增加编译期断言/默认错误响应。", "ref": "Application/app_main.c:318-360"},
            {"level": "low", "title": "UART_TX_POST_DELAY_LOOP 是固定空转延时", "desc": "发送完成后用固定 NOP 循环等待 RS485 电平方向恢复，和主频/优化级别相关。", "suggest": "优先使用 TC 标志或定时器，固定循环只作为最后兜底。", "ref": "Application/app_main.c:18-31"},
            {"level": "low", "title": "串口重配可能被持续外部流量推迟", "desc": "重配只在三路接收 ready、发送 busy 和 pending 都为空时执行；如果外部请求持续不断，g_cpu3_uart_reinit_pending 会一直保留。", "suggest": "增加重配延期计数或维护窗口，在现场能看到串口参数已保存但尚未生效的原因。", "ref": "Application/app_main.c:182-205"},
        ],
        "sources": [
            {"title": "ProtocolSwitchFrame_Process", "desc": "在当前业务协议之前识别统一切换帧并生成独立 ACK 或异常响应。", "refs": [func_line("Communication/common/protocol_switch_frame.c", "ProtocolSwitchFrame_Process")]},
            {"title": "cpu3_port_process", "desc": "优先解析统一切换帧，未命中时按端口配置选择业务协议处理器。", "refs": [func_line("Application/app_main.c", "cpu3_port_process")]},
            {"title": "cpu3_apply_ready_protocol_switches", "desc": "切换应答发送完成后，在主循环保存目标协议并只重配当前端口。", "refs": [func_line("Application/app_main.c", "cpu3_apply_ready_protocol_switches")]},
            {"title": "ltd_modbus_process", "desc": "LTD 共享 Modbus 的 FC03/FC04 快照读取、FC10 转发和异常响应。", "refs": ["Communication/external/ltd_modbus/ltd_modbus_slave.c:68-232"]},
            {"title": "uart_try_send_or_queue", "desc": "三路外部 COM 的发送和单帧 pending 管理。", "refs": ["Application/app_main.c:226-279"]},
            {"title": "HAL_UART_TxCpltCallback", "desc": "发送完成后续发 pending 或恢复接收。", "refs": ["Application/app_main.c:559-660"]},
            {"title": "Cpu3_ReinitAllUarts", "desc": "按本地参数重配三路外部 COM。", "refs": [func_line(FRAM_SOURCE, "Cpu3_ReinitAllUarts")]},
        ],
    }


def dsm_page() -> dict:
    return {
        "id": "cpu3-04",
        "file": "04_DSM协议与命令映射.html",
        "title": "CPU3 DSM 协议与命令映射程序流程",
        "short": "DSM 协议与命令映射",
        "hero": "梳理 DSM 外部 Modbus 从最小帧、地址、CRC、已知功能码实际长度校验和 01/03/04/05/10 分支，到保持寄存器读门禁、线圈命令映射、参数写入双快照回滚和 CPU2 指令/参数下发。",
        "entry": "外部 COM 选择 COM_PROTO_DSM 后调用 DSM_CommunicationProcess。",
        "summary": "DSM 作为兼容协议层：地址和 CRC 通过后，FC01/03/04/05 必须恰好 8 字节，FC10 必须至少 9 字节且实际长度等于 9+byteCount；长度错误返回 0x03，未知功能码（含 06）仍返回 0x01。普通保持寄存器只在 CPU2 状态/参数/协议快照可用时读出；普通参数写先保存参数与受影响寄存器双快照，CPU2 ACK 或解析失败时回滚并返回异常；第 6 段全零占位继续正常读写。",
        "overview_text": "DSM 协议页面重点看四条链：已知功能码的实际请求长度门禁、保持寄存器读门禁、第 6 段全零占位豁免、写多个保持寄存器的 CPU2 ACK 与本地双快照回滚。帧长不符经 ResponseException 返回功能码加 0x80/0x03，未知功能码（含 06）仍走 0x01；参数回滚只修正 CPU3 影子，CPU2 实值由既有四组强制补读最终校准。",
        "overview_description": "DSM RTU 帧通过最小长度、地址、CRC 和功能码长度校验后，按 FC03/04 读取、FC05 命令映射或 FC10 参数写入分支处理。普通 FC03 需 CPU2 状态、参数和协议快照可用且版本兼容，第 6 段全零占位豁免；FC10 保存参数与寄存器双快照，CPU2 ACK 或解析失败时回滚并返回异常。短帧、地址或 CRC 错误不回包，长度错误回 0x03，未知功能码回 0x01。",
        "commands": ["DSM_IsRequestLengthValid", "ResponseException", "Response01", "Response03", "Response04", "Response05", "Response16"],
        "source_files": ["Communication/external/DSM_modbus/DSM_communication.c", "Communication/external/DSM_modbus/DSM_SlaveModbus_modbus2.c", "Communication/external/DSM_modbus/DSM_DataAnalysis_modbus2.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "DSM RTU 帧进入"},
            {"id": "b", "kind": "decision", "x": 405, "y": 145, "w": 310, "h": 108, "label": "最小帧、地址、CRC 和已知功能码实际长度有效？"},
            {"id": "e", "kind": "error", "x": 790, "y": 160, "w": 250, "h": 82, "label": "短帧/地址/CRC 不回包；长度错回 0x03；未知码回 0x01"},
            {"id": "c", "kind": "process", "x": 90, "y": 320, "w": 300, "h": 90, "label": "03：普通段无有效 CPU2 快照时回 0x83/0x06，第6段占位正常；04 读输入"},
            {"id": "d", "kind": "process", "x": 410, "y": 320, "w": 300, "h": 90, "label": "05：线圈地址映射为 CPU2 指令或占位动作"},
            {"id": "f", "kind": "process", "x": 730, "y": 320, "w": 300, "h": 90, "label": "10：普通段双快照后下发 CPU2，失败回滚；第6段占位成功"},
            {"id": "g", "kind": "end", "x": 430, "y": 505, "w": 260, "h": 62, "label": "返回 DSM 响应帧给 COM 层"},
        ],
        "overview_edges": [
            {"d": "M 560 102 L 560 145"},
            {"d": "M 715 199 L 790 201", "label": "否", "red": True},
            {"d": "M 560 253 C 390 270 260 290 240 320", "label": "03/04"},
            {"d": "M 560 253 L 560 320", "label": "05"},
            {"d": "M 560 253 C 730 270 860 290 880 320", "label": "10"},
            {"d": "M 240 410 C 320 470 470 485 560 505"},
            {"d": "M 560 410 L 560 505"},
            {"d": "M 880 410 C 800 470 650 485 560 505"},
        ],
        "flows": [
            {
                "title": "DSM 顶层功能码分发",
                "caption": "DSM_CommunicationProcess 先筛除过短帧、地址不匹配和 CRC 错误；随后 DSM_IsRequestLengthValid 对 FC01/03/04/05 要求实际长度恰好为 8，对 FC10 先确认至少 9 字节，再要求 rcvcount = 9 + byteCount。已知功能码长度不符经 ResponseException 返回功能码加 0x80/0x03；通过长度门禁后，GetFunctioncode 才把未知功能码（含 06）作为非法功能 0x01，再按 01/03/04/05/10 分发。Response03 对非第 6 段执行 CPU2 可用性门禁；Response16 对普通参数写执行双快照、ACK 判定和失败回滚。",
                "height": 1180,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "收到 DSM 请求帧"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 125, "w": 310, "h": 108, "label": "实际长度大于 3？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 140, "w": 250, "h": 82, "label": "过短帧：返回 -1，不回包"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 260, "w": 310, "h": 108, "label": "地址匹配本机或广播？"},
                    {"id": "e2", "kind": "error", "x": 790, "y": 275, "w": 250, "h": 82, "label": "地址不匹配：返回 -1，不回包"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 395, "w": 310, "h": 108, "label": "CRC 正确？"},
                    {"id": "e3", "kind": "error", "x": 790, "y": 410, "w": 250, "h": 82, "label": "CRC 错误：返回 -1，不回包"},
                    {"id": "d4", "kind": "decision", "x": 405, "y": 530, "w": 310, "h": 108, "label": "已知功能码实际长度有效？01/03/04/05=8；10=9+byteCount"},
                    {"id": "e4", "kind": "error", "x": 790, "y": 545, "w": 250, "h": 82, "label": "长度错误：ResponseException 返回功能码+0x80/0x03"},
                    {"id": "d5", "kind": "decision", "x": 405, "y": 665, "w": 310, "h": 108, "label": "GetFunctioncode 支持 01/03/04/05/10？"},
                    {"id": "e5", "kind": "error", "x": 790, "y": 680, "w": 250, "h": 82, "label": "未知功能码（含 06）：返回非法功能 0x01"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 835, "w": 250, "h": 86, "label": "01 读线圈"},
                    {"id": "p2", "kind": "process", "x": 360, "y": 835, "w": 250, "h": 86, "label": "03 普通段门禁/第6段占位；04 读输入"},
                    {"id": "p3", "kind": "process", "x": 630, "y": 835, "w": 250, "h": 86, "label": "05 写线圈转命令"},
                    {"id": "p4", "kind": "process", "x": 900, "y": 835, "w": 180, "h": 86, "label": "10 双快照写参数或第6段占位"},
                    {"id": "end", "kind": "end", "x": 430, "y": 1035, "w": 260, "h": 62, "label": "返回 tx_len 给 COM 层"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 125"},
                    {"d": "M 715 179 L 790 181", "label": "否", "red": True},
                    {"d": "M 560 233 L 560 260", "label": "是"},
                    {"d": "M 715 314 L 790 316", "label": "否", "red": True},
                    {"d": "M 560 368 L 560 395", "label": "是"},
                    {"d": "M 715 449 L 790 451", "label": "否", "red": True},
                    {"d": "M 560 503 L 560 530", "label": "是"},
                    {"d": "M 715 584 L 790 586", "label": "否", "red": True},
                    {"d": "M 560 638 L 560 665", "label": "是"},
                    {"d": "M 715 719 L 790 721", "label": "否", "red": True},
                    {"d": "M 560 773 C 360 790 230 810 215 835", "label": "01"},
                    {"d": "M 560 773 L 485 835", "label": "03/04"},
                    {"d": "M 560 773 L 755 835", "label": "05"},
                    {"d": "M 560 773 C 760 790 980 810 990 835", "label": "10"},
                    {"d": "M 215 921 C 300 995 450 1015 560 1035"},
                    {"d": "M 485 921 C 510 975 535 1005 560 1035"},
                    {"d": "M 755 921 C 700 985 630 1015 560 1035"},
                    {"d": "M 990 921 C 850 1005 700 1025 560 1035"},
                ],
                "evidence": [
                    {"title": "请求长度契约", "text": f"{code(func_line(DSM_COMM_SOURCE, 'DSM_IsRequestLengthValid'))} 对 FC01/03/04/05 要求 rcvcount == 8；FC10 先要求 rcvcount >= 9，确认可读取 byteCount 后再要求 rcvcount == 9 + byteCount。未知功能码在此返回 true，留给非法功能分支处理。"},
                    {"title": "顶层校验与分发", "text": f"{code(func_line(DSM_COMM_SOURCE, 'DSM_CommunicationProcess'))} 在地址和 CRC 通过后、GetFunctioncode 与各 Response 分发前检查实际长度；长度错经 ResponseException 返回 0x03，未知功能码（含 06）仍返回 0x01。"},
                    {"title": "03 读门禁", "text": f"{code(func_line(DSM_SLAVE_SOURCE, 'Response03'))} 普通段在 CPU2_CommIsAvailable 为 false 时返回 0x83/0x06；第 6 段全零占位继续正常响应。"},
                    {"title": "异常帧", "text": f"{code(func_line(DSM_SLAVE_SOURCE, 'ResponseException'))} 统一把请求功能码加 0x80，并携带 0x03 数据异常或其它异常码。"},
                ],
            },
            {
                "title": "写线圈转 CPU2 命令",
                "caption": "Response05 根据线圈地址查表，只有合法动作并写入 0xFF00 时才下发 CPU2 命令；部分线圈按占位成功或无效命令处理。",
                "height": 1040,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "DSM 0x05 写单线圈"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 130, "w": 420, "h": 82, "label": "解析线圈地址和线圈值，匹配线圈命令映射表"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 270, "w": 310, "h": 108, "label": "线圈地址和写入值合法？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 285, "w": 250, "h": 82, "label": "非法地址/数据：组异常响应"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 430, "w": 310, "h": 108, "label": "映射动作类型是什么？"},
                    {"id": "p2", "kind": "process", "x": 80, "y": 600, "w": 260, "h": 86, "label": "SEND_CMD：映射为 CMD_xxx"},
                    {"id": "p3", "kind": "state", "x": 430, "y": 600, "w": 260, "h": 86, "label": "NOOP 或 SELF_CHECK：只回成功或记录占位"},
                    {"id": "e2", "kind": "error", "x": 780, "y": 600, "w": 260, "h": 86, "label": "INVALID_CMD：返回非法数据异常"},
                    {"id": "d3", "kind": "decision", "x": 105, "y": 750, "w": 310, "h": 108, "label": "线圈值是否 0xFF00？"},
                    {"id": "p4", "kind": "process", "x": 80, "y": 920, "w": 300, "h": 82, "label": "下发 HOLDREGISTER_DEVICEPARAM_COMMAND 给 CPU2"},
                    {"id": "end", "kind": "end", "x": 600, "y": 920, "w": 260, "h": 62, "label": "返回 DSM 正常或异常响应"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 560 212 L 560 270"},
                    {"d": "M 715 324 L 790 326", "label": "否", "red": True},
                    {"d": "M 560 378 L 560 430", "label": "是"},
                    {"d": "M 405 484 C 260 520 210 560 210 600", "label": "发命令"},
                    {"d": "M 560 538 L 560 600", "label": "占位"},
                    {"d": "M 715 484 C 840 520 900 560 910 600", "label": "无效", "red": True},
                    {"d": "M 210 686 L 260 750"},
                    {"d": "M 260 858 L 230 920", "label": "是"},
                    {"d": "M 415 804 C 520 850 630 880 730 920", "label": "否"},
                    {"d": "M 380 961 L 600 951"},
                    {"d": "M 560 686 C 610 760 680 840 730 920"},
                    {"d": "M 910 686 C 860 760 790 840 730 920", "red": True},
                ],
                "evidence": [
                    {"title": "映射表", "text": f"{code('DSM_SlaveModbus_modbus2.c:750-807')} g_coil_cmd_map 将 DSM 线圈地址映射为 CPU2 CMD 或占位动作。"},
                    {"title": "写线圈响应", "text": f"{code('DSM_SlaveModbus_modbus2.c:834-914')} Response05 检查地址、线圈值、动作类型，并在合法 0xFF00 时下发 CPU2。"},
                    {"title": "自检占位", "text": f"{code('DSM_SlaveModbus_modbus2.c:57-74')} DSM_RequestSelfCheckPlaceholder/Consume 用模块变量模拟自检状态。"},
                ],
            },
            {
                "title": "写保持寄存器转参数下发",
                "caption": "Response16 先校验数量、byte count 和地址段。第 6 段全零占位直接回成功；其它段先检查 CPU2 可用性，再保存 g_deviceParams 与受影响 DSM 寄存器双快照。写池、参数解析或 CPU2 ACK 失败时恢复双快照并调用 SystemParameterSet，参数非法返回 0x90/0x03，CPU2 不可用或 ACK 失败返回 0x90/0x06；既有四组强制补读最终校准 CPU2 实值。",
                "height": 1520,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "DSM 0x10 写多个保持寄存器"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "数量 1..123 且 byteCount = 数量*2？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 145, "w": 250, "h": 82, "label": "非法数量或字节数：返回数据异常"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 285, "w": 420, "h": 82, "label": "计算起止地址，按 0x000/0x100/0x180/0x200/0x280/0x300 地址段校验"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 420, "w": 310, "h": 108, "label": "地址段和结束地址可写？"},
                    {"id": "e2", "kind": "error", "x": 790, "y": 435, "w": 250, "h": 82, "label": "非法地址：返回地址异常"},
                    {"id": "d4", "kind": "decision", "x": 405, "y": 575, "w": 310, "h": 108, "label": "是否第 6 段全零占位？"},
                    {"id": "p3", "kind": "state", "x": 70, "y": 750, "w": 300, "h": 90, "label": "第 6 段：直接回写成功，不落参数不下发 CPU2"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 750, "w": 310, "h": 108, "label": "普通段 CPU2 三类快照、协议和故障门禁可用？"},
                    {"id": "e3", "kind": "error", "x": 790, "y": 765, "w": 250, "h": 82, "label": "不可用：不提交影子，返回 0x90/0x06"},
                    {"id": "p2", "kind": "process", "x": 350, "y": 920, "w": 420, "h": 86, "label": "保存 g_deviceParams 与受影响 DSM 寄存器双快照"},
                    {"id": "p4", "kind": "process", "x": 350, "y": 1060, "w": 420, "h": 90, "label": "写寄存器池，解析为设备参数并逐项同步 CPU2"},
                    {"id": "d5", "kind": "decision", "x": 405, "y": 1210, "w": 310, "h": 108, "label": "写池、参数解析和 CPU2 ACK 均成功？"},
                    {"id": "e4", "kind": "error", "x": 770, "y": 1225, "w": 300, "h": 90, "label": "失败：恢复双快照并 SystemParameterSet；返回 0x03 或 0x06"},
                    {"id": "end", "kind": "end", "x": 430, "y": 1400, "w": 260, "h": 62, "label": "成功回显；失败后由强制补读最终校准"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 715 184 L 790 186", "label": "否", "red": True},
                    {"d": "M 560 238 L 560 285", "label": "是"},
                    {"d": "M 560 367 L 560 420"},
                    {"d": "M 715 474 L 790 476", "label": "否", "red": True},
                    {"d": "M 560 528 L 560 575", "label": "是"},
                    {"d": "M 405 629 C 300 675 235 710 220 750", "label": "是"},
                    {"d": "M 560 683 L 560 750", "label": "否"},
                    {"d": "M 715 804 L 790 806", "label": "否", "red": True},
                    {"d": "M 560 858 L 560 920", "label": "是"},
                    {"d": "M 560 1006 L 560 1060"},
                    {"d": "M 560 1150 L 560 1210"},
                    {"d": "M 715 1264 L 770 1270", "label": "否", "red": True},
                    {"d": "M 560 1318 L 560 1400", "label": "是"},
                    {"d": "M 220 840 C 300 1320 430 1370 560 1400"},
                    {"d": "M 920 1315 C 820 1370 690 1390 560 1400", "red": True},
                ],
                "evidence": [
                    {"title": "请求和地址合法性", "text": f"{code(func_line(DSM_SLAVE_SOURCE, 'Response16'))} 校验数量、byteCount 和六个地址段；第 6 段全零占位直接成功，普通段才进入 CPU2 写链路。"},
                    {"title": "双快照与失败回滚", "text": f"{code(f'{DSM_SLAVE_SOURCE}:1024-1085')} 普通段写前保存 g_deviceParams 和受影响 DSM 寄存器；写池、解析或 ACK 失败后恢复两份快照并调用 SystemParameterSet。"},
                    {"title": "旧寄存器转设备参数", "text": f"{code(func_line(DSM_DATA_SOURCE, 'UpdateDeviceParamsFromLegacyRegs'))} 先复核 CPU2 可用性，再把 DSM 旧寄存器映射到 g_deviceParams 并通过 DeviceParams_SyncAllToCPU2 逐项同步。"},
                    {"title": "最终校准", "text": f"{code(func_line(CPU2_COMM_SOURCE, 'CPU2_CommFinishFailedRequest'))} 已发起的非命令写失败会关闭参数快照并排队四组强制补读，最终以 CPU2 实值校准 CPU3 镜像。"},
                ],
            },
        ],
        "issues": [
            {"level": "high", "title": "Response05 在外部协议处理路径中同步下发 CPU2", "desc": "线圈命令会直接调用 CPU2_CombinatePackage_Send，外部 COM 响应等待 CPU2 同步写完成，可能导致上位机响应时间不可控。", "suggest": "将 CPU2 指令下发改为主循环命令队列，DSM 先回接收确认，再异步下发 CPU2。", "ref": func_line(DSM_SLAVE_SOURCE, "Response05")},
            {"level": "high", "title": "Response16 普通参数写仍是同步且非原子", "desc": "本轮已在 CPU2 不可用或 ACK 失败时回滚 CPU3 参数与 DSM 寄存器影子并返回 0x90/0x06，消除了失败后的伪成功回读；但多字段仍逐项同步，失败前字段可能已被 CPU2 接受，直到四组强制补读完成前不能把本地回滚等同于 CPU2 原子回滚。", "suggest": "主站收到 0x06 后重试并回读确认；如需事务语义，应在 CPU2 增加批量校验与一次提交。", "ref": func_line(DSM_SLAVE_SOURCE, "Response16")},
            {"level": "mid", "title": "广播地址 0 会进入处理并可能回包", "desc": "DSM_CommunicationProcess 接受 rcvbuff[0] == 0，但后续 Response 函数统一用 SlaveAddress 组帧，可能违反 Modbus 广播不响应习惯。", "suggest": "明确 DSM 广播口径；如按标准广播，应执行写动作但 tx_len=0。", "ref": func_line(DSM_COMM_SOURCE, "DSM_CommunicationProcess")},
            {"level": "mid", "title": "部分无效命令是业务口径而非地址不存在", "desc": "例如 SET_ZEROCIRCLE/SET_ZEROANGLE 映射为 INVALID_CMD，地址存在但返回数据异常，现场联调需要文档化。", "suggest": "在协议说明表中标记“占位成功/无效命令/下发 CPU2”的三类动作。", "ref": "DSM_SlaveModbus_modbus2.c:750-807"},
            {"level": "mid", "title": "第 6 段写保持寄存器当前只做协议占位", "desc": "0x300 段通过 IsHoldingRegisterZeroSegment 判定后回正常响应，但注释明确不落参数、不下发 CPU2，容易被上位机误认为参数已生效。", "suggest": "协议表中单独标记 0x300 段为占位成功，并在联调时验证上位机是否依赖这些参数。", "ref": "DSM_SlaveModbus_modbus2.c:980-1018"},
            {"level": "low", "title": "03/04 读前全量刷新寄存器", "desc": "03 读前仍执行 SystemParameterSet，04 读前执行 Input_Write；Response03 已对非第 6 段增加 CPU2 可用性门禁，但频繁读仍会全量刷新寄存器数组。", "suggest": "可按脏标志或固定节拍刷新缓存，读请求只做拷贝。", "ref": func_line(DSM_COMM_SOURCE, "DSM_CommunicationProcess")},
        ],
        "sources": [
            {"title": "DSM_CommunicationProcess", "desc": "DSM 顶层校验和功能码分发。", "refs": [func_line(DSM_COMM_SOURCE, "DSM_CommunicationProcess")]},
            {"title": "DSM_IsRequestLengthValid", "desc": "已知功能码实际请求长度契约；FC10 在读取 byteCount 前先做最小长度门禁。", "refs": [func_line(DSM_COMM_SOURCE, "DSM_IsRequestLengthValid")]},
            {"title": "Response03", "desc": "普通保持寄存器读门禁与第 6 段全零占位豁免。", "refs": [func_line(DSM_SLAVE_SOURCE, "Response03")]},
            {"title": "Response05", "desc": "线圈命令映射到 CPU2 CMD。", "refs": [func_line(DSM_SLAVE_SOURCE, "Response05")]},
            {"title": "Response16", "desc": "写保持寄存器、双快照、失败回滚和异常响应。", "refs": [func_line(DSM_SLAVE_SOURCE, "Response16")]},
            {"title": "UpdateDeviceParamsFromLegacyRegs", "desc": "DSM 旧寄存器写入后更新 CPU3/CPU2 参数。", "refs": [func_line(DSM_DATA_SOURCE, "UpdateDeviceParamsFromLegacyRegs")]},
        ],
    }


def wartsila_si_page() -> dict:
    return {
        "id": "cpu3-05",
        "file": "05_Wartsila与SI协议适配.html",
        "title": "CPU3 Wartsila 与 SI协议适配程序流程",
        "short": "Wartsila 与 SI协议适配",
        "hero": "梳理 Wartsila 寄存器池、写保持寄存器下发 CPU2、SI 快照同步、读写线圈/寄存器和异常响应。",
        "entry": "外部 COM 选择 COM_PROTO_WARTSILA 或 COM_PROTO_SI 后进入对应协议处理器。",
        "summary": "Wartsila 主要通过保持寄存器池映射参数和结果，SI 通过快照数组对外提供线圈、离散输入、保持寄存器和输入寄存器。",
        "overview_text": "这两个协议都是 CPU3 的对外适配层：Wartsila 仅在写入 0x0006 或 0x005A~0x005D 时桥接 CPU2，SI 写入在 CPU2 ACK 后才提交本地影子；状态、完整参数或当前连接协议快照未完成、共享协议不兼容、通信故障或写失败时均返回设备忙 0x06。Wartsila 参数失败会恢复写前镜像并强制补读，未确认期间读取 0x005A~0x005D 也返回 0x06；非零命令影子在一次下发尝试后即清零，失败后的写 0 不会重放旧命令。CPU2 参数补读完成会刷新 Wartsila 镜像，批量参数同步跳过命令字段。",
        "overview_description": "外部帧按端口配置进入 Wartsila 或 SI：Wartsila 读写保持寄存器池，仅命中命令或四个参数地址时桥接 CPU2；SI 校验帧后刷新快照，并在 CPU2 ACK 成功后才提交线圈或参数影子。正常路径返回响应帧；CPU2 不可用或写失败返回设备忙 0x06，Wartsila 参数失败还会恢复写前镜像并强制补读，非零命令影子在一次尝试后清零。",
        "commands": ["modbus_rtu_process", "si_modbus_process"],
        "source_files": ["Communication/external/wartsila_modbus/wartsila_modbus_communication.c", "Communication/external/wartsila_modbus/wartsila_modbus_data_analysis.c", "Communication/external/si_modbus/si_modbus_slave.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "外部协议帧进入"},
            {"id": "b", "kind": "decision", "x": 405, "y": 145, "w": 310, "h": 108, "label": "端口协议为 Wartsila 还是 SI？"},
            {"id": "c", "kind": "process", "x": 90, "y": 320, "w": 300, "h": 90, "label": "Wartsila：地址/CRC 校验后读写 g_holding_regs"},
            {"id": "d", "kind": "process", "x": 730, "y": 320, "w": 300, "h": 90, "label": "SI：地址/CRC 通过后刷新快照数组"},
            {"id": "e", "kind": "decision", "x": 90, "y": 485, "w": 300, "h": 108, "label": "Wartsila 是否写保持寄存器？"},
            {"id": "f", "kind": "process", "x": 90, "y": 655, "w": 300, "h": 90, "label": "参数先确认 ACK，再下发非零命令；命令影子一次消费，失败回 0x06"},
            {"id": "g", "kind": "process", "x": 730, "y": 500, "w": 300, "h": 90, "label": "SI 写入先等 CPU2 ACK，再提交影子；失败回设备忙"},
            {"id": "h", "kind": "end", "x": 430, "y": 815, "w": 260, "h": 62, "label": "返回响应帧或错误码给 COM 层"},
        ],
        "overview_edges": [
            {"d": "M 560 102 L 560 145"},
            {"d": "M 405 199 C 260 240 240 280 240 320", "label": "Wartsila"},
            {"d": "M 715 199 C 860 240 880 280 880 320", "label": "SI"},
            {"d": "M 240 410 L 240 485"},
            {"d": "M 240 593 L 240 655", "label": "是"},
            {"d": "M 390 539 C 500 600 560 700 560 815", "label": "否/0x03"},
            {"d": "M 880 410 L 880 500"},
            {"d": "M 240 745 C 320 800 460 810 560 815"},
            {"d": "M 880 590 C 800 760 660 805 560 815"},
        ],
        "flows": [
            {
                "title": "Wartsila 读写保持寄存器",
                "caption": "Wartsila 只支持 0x03 和 0x10。读前把设备参数/测量值刷到 g_holding_regs，CPU2 完整参数补读结束也会刷新该镜像；0x10 只有覆盖 0x0006 或 0x005A~0x005D 时才桥接 CPU2。单帧同时覆盖参数区与命令区时，先逐项确认四参数 ACK，再下发 0x0006 非零命令；任一步失败返回设备忙 0x06。参数阶段失败会恢复 CPU3 写前四参数镜像、立即关闭参数快照并强制补读，补读前读四参数也返回 0x06；命令影子无论 ACK 成败都在本次尝试后清零，批量参数同步也永久跳过命令字段。",
                "height": 1080,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "Wartsila RTU 帧进入"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "地址和 CRC 有效？"},
                    {"id": "err", "kind": "error", "x": 790, "y": 145, "w": 250, "h": 82, "label": "地址不匹配或 CRC 错误：不生成响应"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "功能码是 0x03 还是 0x10？"},
                    {"id": "p1", "kind": "process", "x": 80, "y": 455, "w": 300, "h": 90, "label": "0x03：四参数未确认时回 0x06，否则刷新寄存器池后返回"},
                    {"id": "p2", "kind": "process", "x": 730, "y": 455, "w": 300, "h": 90, "label": "0x10：校验数量、字节数和地址后写入寄存器池"},
                    {"id": "p3", "kind": "process", "x": 730, "y": 620, "w": 300, "h": 90, "label": "写入寄存器池并判断是否命中 CPU2 桥接地址"},
                    {"id": "d3", "kind": "decision", "x": 730, "y": 770, "w": 300, "h": 108, "label": "覆盖 0x0006 或 0x005A~0x005D？"},
                    {"id": "p4", "kind": "process", "x": 420, "y": 925, "w": 300, "h": 82, "label": "是：参数先确认 ACK，再下发非零命令；命令影子一次消费，失败回 0x06"},
                    {"id": "p5", "kind": "state", "x": 780, "y": 925, "w": 300, "h": 82, "label": "否：仅保留 Wartsila 本地寄存器写入"},
                    {"id": "end", "kind": "end", "x": 80, "y": 925, "w": 260, "h": 62, "label": "返回正常/异常响应"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 715 184 L 790 186", "label": "否", "red": True},
                    {"d": "M 560 238 L 560 285", "label": "是"},
                    {"d": "M 405 339 C 280 385 240 420 230 455", "label": "03"},
                    {"d": "M 715 339 C 840 385 880 420 880 455", "label": "10"},
                    {"d": "M 230 545 L 210 925"},
                    {"d": "M 880 545 L 880 620"},
                    {"d": "M 880 710 L 880 770"},
                    {"d": "M 730 824 C 630 860 570 890 570 925", "label": "是"},
                    {"d": "M 880 878 L 930 925", "label": "否"},
                    {"d": "M 420 966 L 340 956"},
                    {"d": "M 930 1007 C 760 1060 420 1060 210 987"},
                ],
                "evidence": [
                    {"title": "Wartsila 顶层", "text": f"{code('wartsila_modbus_communication.c:173-228')} modbus_rtu_process 校验地址/CRC，处理 0x03/0x10，并在写后回调失败时覆盖为设备忙响应。"},
                    {"title": "写后回调", "text": f"{code('wartsila_modbus_communication.c:249-306')} modbus_on_holding_written 只处理 0x0006 命令和 0x005A~0x005D 参数；跨区帧先确认参数 ACK，再下发命令。参数失败恢复写前四参数镜像并清命令，非零命令影子无论 ACK 成败都在本次尝试后清零。"},
                    {"title": "未确认参数读门禁", "text": f"{code('wartsila_modbus_communication.c:51-116')} 参数快照无效时，覆盖 0x005A~0x005D 的 0x03 返回设备忙 0x06，不把失败写留下的目标值作为成功回读。"},
                    {"title": "参数转发", "text": f"{code('wartsila_modbus_communication.c:274-304')} ForwardParamsToLowerDevice 逐项转发四个瓦锡兰密度区间参数，任一失败即返回 false。"},
                ],
            },
            {
                "title": "SI 快照同步与功能码处理",
                "caption": "SI 在地址和 CRC 通过后才刷新 CPU3 快照。FC05 ON 和 CPU2 参数 40001~40003 需等待 CPU2 ACK，失败返回设备忙 0x06 且不提交影子；CPU3 本机参数 40010~40023 合法时直接保存 FRAM。分发层只要有正常或异常响应帧就视为本帧已处理。",
                "height": 1040,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "SI RTU 帧进入"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "接收/发送缓存有效且长度足够？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 145, "w": 250, "h": 82, "label": "参数或长度错误：返回 BADLEN"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "地址匹配且 CRC 正确？"},
                    {"id": "e2", "kind": "error", "x": 790, "y": 300, "w": 250, "h": 82, "label": "地址不匹配或 CRC 错误：不回包"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 455, "w": 420, "h": 86, "label": "刷新线圈、保持寄存器、离散输入、输入寄存器快照"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 600, "w": 310, "h": 108, "label": "功能码属于 01/02/03/04/05/06？"},
                    {"id": "p2", "kind": "process", "x": 90, "y": 760, "w": 300, "h": 90, "label": "读类：从快照数组返回位或寄存器"},
                    {"id": "p3", "kind": "process", "x": 730, "y": 760, "w": 300, "h": 90, "label": "写类：CPU2项等ACK；本机项直接保存；失败回0x06"},
                    {"id": "e3", "kind": "error", "x": 410, "y": 760, "w": 300, "h": 90, "label": "未知功能码：组非法功能异常"},
                    {"id": "end", "kind": "end", "x": 430, "y": 935, "w": 260, "h": 62, "label": "有响应帧则分发层返回成功"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 715 184 L 790 186", "label": "否", "red": True},
                    {"d": "M 560 238 L 560 285", "label": "是"},
                    {"d": "M 715 339 L 790 341", "label": "否", "red": True},
                    {"d": "M 560 393 L 560 455", "label": "是"},
                    {"d": "M 560 541 L 560 600"},
                    {"d": "M 405 654 C 300 700 250 730 240 760", "label": "01-04"},
                    {"d": "M 715 654 C 820 700 870 730 880 760", "label": "05/06"},
                    {"d": "M 560 708 L 560 760", "label": "其它", "red": True},
                    {"d": "M 240 850 C 330 910 470 925 560 935"},
                    {"d": "M 880 850 C 790 910 650 925 560 935"},
                    {"d": "M 560 850 L 560 935", "red": True},
                ],
                "evidence": [
                    {"title": "写入分流", "text": f"{code('si_modbus_slave.c:1166-1257')} FC05/FC06 区分 CPU2 命令/参数与 CPU3 本机参数，CPU2 项返回成功后才提交影子。"},
                    {"title": "快照刷新", "text": f"{code('si_modbus_slave.c:1266-1275')} si_modbus_sync_from_system 汇总刷新四类数组。"},
                    {"title": "处理入口", "text": f"{code('si_modbus_slave.c:1279-1339')} 地址/CRC 通过后才同步系统状态并按功能码处理。"},
                    {"title": "分发表适配", "text": f"{code('si_modbus_slave.c:1346-1360')} 只要已生成响应帧，COM 分发层就按成功处理并发送。"},
                ],
            },
        ],
        "issues": [
            {"level": "low", "title": "Wartsila 0x10 保留现场非标准 byteCount", "desc": "现场抓包已确认 Wartsila 写帧使用 byteCount=qty，而数据区仍为每寄存器 2 字节；当前实现按该现场兼容口径校验，不应直接改成标准 Modbus 的 qty*2。", "suggest": "保留非标准行为并用 golden frame 固化；若后续需要兼容通用 Modbus 工具，应另行设计双格式识别，不能破坏现场主机。", "ref": "wartsila_modbus_communication.c:102-125; 当前兼容情况与协议栈.md"},
            {"level": "mid", "title": "Wartsila 四参数同步不是原子事务", "desc": "0x005A~0x005D 通过四次 CPU2 请求顺序下发。若中途失败，外部主机收到设备忙 0x06，但失败前已被 CPU2 接受的字段可能已经生效，不能自动回滚。", "suggest": "主站收到 0x06 后重试并回读；如需原子更新，在 CPU2 增加批量参数校验和一次性提交。", "ref": "wartsila_modbus_communication.c:274-304"},
            {"level": "low", "title": "SI 写失败反馈已收口但仍需现场重试验证", "desc": "SI FC05 ON 和 CPU2 参数 40001~40003 在状态/完整参数/当前连接协议快照未完成、共享协议不兼容、通信故障或 ACK 失败时返回设备忙 0x06，相关线圈/保持寄存器影子只在 ACK 成功后更新；40010~40023 是 CPU3 本机参数，合法写入直接保存。SI 00009 Stop 映射维护模式，不使用屏幕取消的参数快照绕过。", "suggest": "联调时覆盖离线、延迟上线、协议快照未建立、参数同步未完成、协议不匹配和参数刷新场景，确认 PLC 按设备忙重试，同时确认本机参数不受 CPU2 离线影响。", "ref": "si_modbus_slave.c:942-1257"},
            {"level": "low", "title": "Wartsila 特殊地址 0x0672 返回固定版本字符串", "desc": "0x0672 读取固定返回一段版本文本，不走寄存器池，后续版本变化时容易遗漏同步。", "suggest": "把固定字符串集中成宏或从版本定义生成。", "ref": "wartsila_modbus_communication.c:55-76"},
        ],
        "sources": [
            {"title": "modbus_rtu_process", "desc": "Wartsila 顶层 RTU 处理。", "refs": ["Communication/external/wartsila_modbus/wartsila_modbus_communication.c:173-228"]},
            {"title": "modbus_on_holding_written", "desc": "Wartsila 写后解析、CPU2 下发和命令影子一次性消费。", "refs": ["Communication/external/wartsila_modbus/wartsila_modbus_communication.c:230-275"]},
            {"title": "si_modbus_process", "desc": "SI 地址/CRC/快照/功能码处理。", "refs": ["Communication/external/si_modbus/si_modbus_slave.c:1279-1360"]},
            {"title": "DeviceParams_StoreToRegisters", "desc": "把 CPU3 参数/测量结果整理到 Wartsila 寄存器池。", "refs": ["Communication/external/wartsila_modbus/wartsila_modbus_data_analysis.c:198"]},
        ],
    }


def display_page() -> dict:
    return {
        "id": "cpu3-06",
        "file": "06_显示刷新与按键事件.html",
        "title": "CPU3 显示刷新与按键事件程序流程",
        "short": "显示刷新与按键事件",
        "hero": "梳理 OLED 初始化、显示任务、状态页 2 秒数据采样、协议兼容提示、CPU2 通信超时和 AD5421/AO 故障显示、长按进入菜单、按键队列、长按释放保护和 SPI 恢复。",
        "entry": "App_MainLoop 每轮调用 Display_Task；按键 EXTI 中断只入队或启动长按计时，不直接绘制。",
        "summary": "显示模块在主循环中消费按键和刷新屏幕；状态快照与当前连接协议快照均建立后才退出通讯尝试页，连续第 10 次未获得合法响应切换 CPU2 通信故障页，并结合状态页采样节流、协议版本提示、AD5421/AO 故障文案和 SPI 恢复保护 OLED 绘制。",
        "overview_text": "显示流程的关键是把中断输入、状态数据采样和 OLED 绘制解耦：中断只记录事件，状态页按变化/2 秒节拍采样，Display_Task 在主循环里统一处理。",
        "overview_description": "按键中断或刷新请求到达后，EXTI 只做消抖、入队或启动长按，Display_Task 在主循环消费事件并选择菜单、确认等前景整屏绘制或状态页刷新。状态页在首次、关键变化或 2 秒节拍时更新显示快照；状态或协议快照未就绪时保留通讯尝试页，连续第 10 次失败切换通信故障页，SPI 错误或周期恢复触发 OLED 清屏重绘。",
        "commands": ["LONG_PRESS_KEY_SURE", "LONG_PRESS_KEY_BACK", "USE_KEY_UP", "USE_KEY_DOWN", "USE_KEY_SURE", "USE_KEY_BACK"],
        "source_files": ["Application/display/display.c", "Application/display/exit.c", "Application/display/display_tankopera.c", "Application/display/hgs.c", "Communication/internal/main_board_modbus/cpu2_communicate.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "按键中断或显示刷新请求"},
            {"id": "b", "kind": "process", "x": 90, "y": 175, "w": 300, "h": 86, "label": "EXTI 消抖：菜单态入普通按键队列，状态页启动长按"},
            {"id": "c", "kind": "process", "x": 730, "y": 175, "w": 300, "h": 86, "label": "定时器或状态变化请求屏幕刷新"},
            {"id": "d", "kind": "loop", "x": 350, "y": 340, "w": 420, "h": 90, "label": "显示任务更新释放保护，消费长按和普通按键"},
            {"id": "e", "kind": "decision", "x": 405, "y": 500, "w": 310, "h": 108, "label": "是否进入前景页面或状态刷新？"},
            {"id": "f", "kind": "process", "x": 90, "y": 670, "w": 300, "h": 90, "label": "前景页面：菜单/确认/故障原因整屏绘制"},
            {"id": "g", "kind": "process", "x": 730, "y": 670, "w": 300, "h": 90, "label": "状态页按需要局部或全屏刷新"},
            {"id": "h", "kind": "error", "x": 410, "y": 830, "w": 300, "h": 90, "label": "SPI 错误或周期恢复：执行 OLED 恢复清屏"},
        ],
        "overview_edges": [
            {"d": "M 560 102 C 390 125 260 145 240 175"},
            {"d": "M 560 102 C 730 125 860 145 880 175"},
            {"d": "M 240 261 C 330 310 450 325 560 340"},
            {"d": "M 880 261 C 790 310 670 325 560 340"},
            {"d": "M 560 430 L 560 500"},
            {"d": "M 405 554 C 300 600 250 635 240 670", "label": "前景"},
            {"d": "M 715 554 C 820 600 870 635 880 670", "label": "状态"},
            {"d": "M 560 608 L 560 830", "label": "恢复", "red": True},
        ],
        "flows": [
            {
                "title": "按键中断到主循环消费",
                "caption": "EXTI 回调只做亮屏、消抖、普通按键入队或启动长按计时；实际页面跳转和绘制在显示任务中完成。",
                "height": 1080,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "按键 EXTI 触发"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 130, "w": 420, "h": 82, "label": "点亮屏幕，并按键位读取 GPIO 电平"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 270, "w": 310, "h": 108, "label": "50ms 内是否重复下降沿？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 285, "w": 250, "h": 82, "label": "抖动：丢弃本次按键"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 430, "w": 310, "h": 108, "label": "当前是否在罐上菜单操作？"},
                    {"id": "p2", "kind": "state", "x": 80, "y": 600, "w": 300, "h": 90, "label": "菜单态：按键入 8 项环形队列，队列满丢弃最新"},
                    {"id": "p3", "kind": "state", "x": 730, "y": 600, "w": 300, "h": 90, "label": "状态页：确认/返回长按启动 TIM1 计时"},
                    {"id": "p4", "kind": "loop", "x": 350, "y": 760, "w": 420, "h": 82, "label": "显示任务读取长按动作或按键队列"},
                    {"id": "p5", "kind": "process", "x": 350, "y": 900, "w": 420, "h": 90, "label": "长按确认进入主菜单；长按返回进入取消测量或故障原因页；普通按键交当前页面处理"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 560 212 L 560 270"},
                    {"d": "M 715 324 L 790 326", "label": "是", "red": True},
                    {"d": "M 560 378 L 560 430", "label": "否"},
                    {"d": "M 405 484 C 280 520 240 560 230 600", "label": "是"},
                    {"d": "M 715 484 C 840 520 880 560 880 600", "label": "否"},
                    {"d": "M 230 690 C 320 735 440 750 560 760"},
                    {"d": "M 880 690 C 800 735 680 750 560 760"},
                    {"d": "M 560 842 L 560 900"},
                ],
                "evidence": [
                    {"title": "EXTI 回调", "text": f"{code('Application/display/exit.c:318-363')} 按键中断按 FlagofTankOpera 区分菜单按键和状态页长按。"},
                    {"title": "按键队列", "text": f"{code('Application/display/exit.c:181-246')} Display_RequestKey/TakePendingKey 实现 8 项队列和临界区保护。"},
                    {"title": "主循环消费", "text": f"{code('Application/display/display.c:2024-2073')} Display_ProcessLongPressAction 和 Display_ProcessPendingInput 处理页面跳转和按键。"},
                ],
            },
            {
                "title": "显示刷新与 OLED 恢复",
                "caption": "Display_Task 根据刷新请求、状态高亮计时和 SPI 错误计数决定是否重绘；前景页面会要求状态页下次全屏刷新。",
                "height": 1040,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "显示任务被主循环调用"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 130, "w": 420, "h": 82, "label": "更新长按释放保护并消费输入事件"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 270, "w": 310, "h": 108, "label": "有状态高亮超时或刷新请求？"},
                    {"id": "end1", "kind": "end", "x": 790, "y": 285, "w": 250, "h": 82, "label": "无刷新：返回主循环"},
                    {"id": "p2", "kind": "process", "x": 350, "y": 440, "w": 420, "h": 82, "label": "清屏幕刷新请求，记录刷新开始时间"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 580, "w": 310, "h": 108, "label": "是否需要 OLED 恢复或全屏清屏？"},
                    {"id": "p3", "kind": "error", "x": 790, "y": 595, "w": 250, "h": 82, "label": "SPI 错误/周期恢复：执行 OLED 恢复清屏"},
                    {"id": "p4", "kind": "process", "x": 350, "y": 750, "w": 420, "h": 82, "label": "根据状态和数据源绘制状态页"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 880, "w": 310, "h": 108, "label": "刷新耗时是否超过 500ms？"},
                    {"id": "e2", "kind": "error", "x": 790, "y": 895, "w": 250, "h": 82, "label": "超时计数 +1"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 560 212 L 560 270"},
                    {"d": "M 715 324 L 790 326", "label": "否"},
                    {"d": "M 560 378 L 560 440", "label": "是"},
                    {"d": "M 560 522 L 560 580"},
                    {"d": "M 715 634 L 790 636", "label": "是", "red": True},
                    {"d": "M 560 688 L 560 750", "label": "否/恢复后"},
                    {"d": "M 790 677 C 720 715 650 735 560 750", "red": True},
                    {"d": "M 560 832 L 560 880"},
                    {"d": "M 715 934 L 790 936", "label": "是", "red": True},
                ],
                "evidence": [
                    {"title": "显示初始化", "text": f"{code('Application/display/display.c:1871-1905')} DisplayInit 初始化 OLED、计数器、TIM3 和页面状态。"},
                    {"title": "恢复条件", "text": f"{code('Application/display/display.c:1912-2005')} Display_ShouldRecoverBeforeDraw 监控强制恢复、SPI 错误和 60s 周期恢复。"},
                    {"title": "刷新任务", "text": f"{code('Application/display/display.c:2057-2073')} Display_Task 执行刷新并统计超过 500ms 的耗时。"},
                ],
            },
            {
                "title": "上电通讯尝试页与连续通信失败故障切换",
                "caption": "状态快照、完整参数快照、当前连接协议快照、连续请求失败和故障恢复锁存相互独立：冷启动第 1～9 次未获得合法响应仍显示通讯尝试页，第 10 次转故障；状态 0x04 快照负责建立状态/解除故障，覆盖协议字段的合法 0x03 才建立当前连接协议快照，二者均建立后才退出通讯尝试页。普通菜单写入口必须等待三类快照完整且共享协议版本相等。",
                "height": 900,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "状态页开始刷新"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 145, "w": 310, "h": 108, "label": "尚未建立首次状态快照且通信故障未锁存？"},
                    {"id": "p1", "kind": "process", "x": 70, "y": 330, "w": 300, "h": 90, "label": "是：冷启动失败少于 10 次，显示“通讯尝试中...”"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 500, "w": 310, "h": 108, "label": "当前是否为 CPU2 通信超时故障？"},
                    {"id": "p2", "kind": "process", "x": 70, "y": 690, "w": 300, "h": 90, "label": "否：绘制正常 CPU2 状态或其它故障状态"},
                    {"id": "p3", "kind": "error", "x": 750, "y": 690, "w": 300, "h": 90, "label": "是：连续第 10 次未获合法响应，显示 CPU2通信超时故障"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 145"},
                    {"d": "M 405 199 C 280 240 230 290 220 330", "label": "是"},
                    {"d": "M 560 253 L 560 500", "label": "否"},
                    {"d": "M 405 554 C 280 600 230 650 220 690", "label": "否"},
                    {"d": "M 715 554 C 840 600 890 650 900 690", "label": "是", "red": True},
                ],
                "evidence": [
                    {"title": "通信状态查询", "text": f"{code('Communication/internal/main_board_modbus/cpu2_communicate.c:21-225')} CPU2_CommShouldShowStartup 等待首次状态和当前连接协议快照，故障锁存时由故障页接管；CPU2_CommIsAvailable 另要求完整参数快照和共享协议相等。"},
                    {"title": "页面选择", "text": f"{code('Application/display/display.c:3066-3079')} RefreshScreen 只通过通信模块查询接口选择通讯尝试页，不再读取计数哨兵。"},
                    {"title": "菜单门禁", "text": f"{code(func_line(MENU_SOURCE, 'send_cpu2_command'))} 和 {code(func_line(MENU_SOURCE, 'Display_RequestCancelMeasurement'))} 分别执行普通写入与屏幕取消门禁；普通写要求状态/参数/当前连接协议快照完整、协议相等且无通信故障，同步请求失败后不显示旧参数或成功反馈。屏幕取消仅放宽参数快照，SI Stop 不使用该绕过。"},
                ],
            },
            {
                "title": "状态页数据采样、协议兼容和通信故障显示",
                "caption": "状态页不是每次绘制都重新采样。当前程序在首次、强制刷新、关键状态变化或 2 秒采样节拍到期时更新快照，否则只恢复高亮区域。",
                "height": 1180,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "进入状态页绘制"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 135, "w": 310, "h": 108, "label": "是否首次、全屏恢复或快照无效？"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 300, "w": 300, "h": 90, "label": "立即采样 CPU2 缓存，生成状态页快照"},
                    {"id": "d2", "kind": "decision", "x": 730, "y": 300, "w": 300, "h": 108, "label": "状态/错误/协议/语言是否变化或已到 2 秒？"},
                    {"id": "p2", "kind": "process", "x": 730, "y": 475, "w": 300, "h": 90, "label": "需要采样：刷新液位、水位、RSSI、协议状态、CPU2超时和故障原因"},
                    {"id": "p3", "kind": "state", "x": 350, "y": 635, "w": 420, "h": 86, "label": "不采样：只恢复状态高亮，不重新读取 CPU2 数据"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 780, "w": 310, "h": 108, "label": "CPU2/CPU3 协议版本严格一致？"},
                    {"id": "e1", "kind": "error", "x": 90, "y": 945, "w": 300, "h": 90, "label": "不一致：显示协议不兼容，提示成对升级"},
                    {"id": "d4", "kind": "decision", "x": 730, "y": 945, "w": 300, "h": 108, "label": "错误码是否为 CPU2通信超时或 AD5421/AO 相关？"},
                    {"id": "p4", "kind": "process", "x": 730, "y": 1110, "w": 300, "h": 90, "label": "按 CPU2通信超时或 AD5421/AO 文案显示现场可读故障原因"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 135"},
                    {"d": "M 405 189 C 300 240 250 275 240 300", "label": "是"},
                    {"d": "M 715 189 C 820 240 870 275 880 300", "label": "否"},
                    {"d": "M 880 408 L 880 475", "label": "是"},
                    {"d": "M 730 354 C 650 520 610 600 560 635", "label": "否"},
                    {"d": "M 240 390 C 330 540 450 600 560 635"},
                    {"d": "M 880 565 C 800 605 680 620 560 635"},
                    {"d": "M 560 721 L 560 780"},
                    {"d": "M 405 834 C 300 880 250 920 240 945", "label": "否", "red": True},
                    {"d": "M 715 834 C 820 880 870 920 880 945", "label": "是"},
                    {"d": "M 880 1053 L 880 1110", "label": "是", "red": True},
                ],
                "evidence": [
                    {"title": "采样节流", "text": f"{code('Application/display/display.c:16,2633-2652')} 状态页使用 DISPLAY_STATUS_DATA_REFRESH_MS=2000，并由 Display_ShouldSampleStatusData 判断是否重新采样。"},
                    {"title": "状态页绘制", "text": f"{code('Application/display/display.c:2968-3120')} RefreshScreen 根据快照有效性、全屏恢复和状态变化决定绘制路径。"},
                    {"title": "协议与故障文案", "text": f"{code('Application/display/display.c:650-816,3076-3084')} 状态页严格判断协议版本，并把 CPU2 通信超时和 AD5421/AO 错误码翻译成现场可读故障原因。"},
                ],
            },
        ],
        "issues": [
            {"level": "mid", "title": "按键队列满时丢弃最新按键", "desc": "pending_key_queue 满时只增加 overflow_count，用户快速操作时最新按键会丢失但界面无提示。", "suggest": "在调试页显示 overflow_count，或改为丢弃最旧按键并给界面节流。", "ref": "Application/display/exit.c:181-199"},
            {"level": "mid", "title": "前景绘制和协议轮询共用主循环", "desc": "菜单绘制、确认页和状态页刷新都在 App_MainLoop 中执行，刷新耗时过长会影响外部 COM 响应和 CPU2 轮询。", "suggest": "保留 display_last_refresh_ms 统计，并对状态页分帧/局部刷新做节流。", "ref": "Application/display/display.c:2057-2073"},
            {"level": "low", "title": "长按释放保护会临时屏蔽按键", "desc": "长按动作后 pending_key_tail=head，并等待释放保护时间，用户可能感觉按键短暂失效。", "suggest": "在交互说明或界面状态上体现长按确认后的释放等待。", "ref": "Application/display/exit.c:254-314"},
            {"level": "low", "title": "OLED 周期恢复固定 60s", "desc": "无论现场 SPI 状态如何，60s 到期会触发恢复策略，可能造成偶发刷新停顿。", "suggest": "将周期恢复做成参数或只在错误计数变化后触发。", "ref": "Application/display/display.c:13-16,1912-2005"},
            {"level": "mid", "title": "状态页数据最多 2 秒才重新采样一次", "desc": "当前版本为降低闪烁和绘制负担，非强制刷新时按 2 秒采样状态数据；AO 电流、RSSI 或瞬时故障变化可能不会立即显示。", "suggest": "把 AO/故障字段纳入强制采样触发，或在状态页显示最近采样时间/更新计数。", "ref": "Application/display/display.c:16,2633-3120"},
            {"level": "mid", "title": "协议不兼容、AO 故障和 CPU2 通信故障来源不同", "desc": "协议版本严格一致既避免尾段错位，也参与 CPU2 写门禁；AD5421/AO 故障原因来自 CPU2 设备错误码；CPU2_COMM_TIMEOUT 由 CPU3 在连续第 10 个请求未获得合法响应后本机置位，状态、完整参数、当前连接协议快照和状态帧恢复锁存另行维护。", "suggest": "升级流程仍要求 CPU2/CPU3 成对升级；排查故障时区分 CPU2 上报错误码、CPU3 本机通信错误码、参数或协议快照未完成和冷启动通讯尝试状态。", "ref": "Application/display/display.c:650-816,3066-3079; cpu2_communicate.c:21-204"},
        ],
        "sources": [
            {"title": "Display_Task", "desc": "显示任务主入口。", "refs": ["Application/display/display.c:2057-2073"]},
            {"title": "HAL_GPIO_EXTI_Callback", "desc": "按键中断入口。", "refs": ["Application/display/exit.c:318-363"]},
            {"title": "Display_ProcessPendingInput", "desc": "主循环消费长按和普通按键。", "refs": ["Application/display/display.c:2024-2046"]},
            {"title": "Display_ShouldSampleStatusData", "desc": "状态页采样节流和强制刷新判断。", "refs": ["Application/display/display.c:2633-2652"]},
            {"title": "OLED_RecoverAndClear", "desc": "OLED SPI 异常后的恢复清屏。", "refs": ["Application/display/hgs.c:591"]},
        ],
    }


def menu_page() -> dict:
    return {
        "id": "cpu3-07",
        "file": "07_菜单参数与指令下发.html",
        "title": "CPU3 菜单参数与指令下发程序流程",
        "short": "菜单参数与指令下发",
        "hero": "梳理主菜单、测量命令、读取部件参数、AO 输出使能、调试指令、参数配置、密码、数值输入、保护确认、本机参数保存和 CPU2 参数/指令下发。",
        "entry": "长按确认进入主菜单；菜单按键由 KeyProcess 根据 func_index 调用对应页面函数。",
        "summary": "菜单既能发 CPU2 测量/调试命令，也能修改 CPU2 设备参数和 CPU3 本机显示/通信参数；CPU2 普通写入口由状态、完整参数、当前连接协议三类快照、共享协议相等和通信故障锁存共同门禁。参数刷新期间仅屏幕 CMD_CANCEL_MEASUREMENT 可放宽参数快照，请求失败不会进入成功或运动监控页面。",
        "overview_text": "菜单流程必须和参数/命令下发合在一起看：页面选择、输入、确认、保护确认、CPU2 可用性门禁、ACK 结果、本地保存和失败反馈都在一条链路中。",
        "overview_description": "长按确认进入主菜单后，用户选择命令或参数，按是否需要密码、数值输入及保护操作进入普通确认或二次确认。CPU3 本机参数直接保存 FRAM 并按需请求串口重配；CPU2 命令或参数必须通过状态、参数、协议快照和版本门禁并取得合法 ACK，成功后更新反馈，失败则提示通信故障并返回，不进入成功或运动监控页。",
        "commands": ["COM_NUM_FIND_OIL", "COM_NUM_FIND_WATER", "COM_NUM_WARTSILA_DENSITY", "COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE", "CMD_READ_PART_PARAMS", "CMD_CANCEL_MEASUREMENT"],
        "source_files": ["Application/display/display_tankopera.c", "Communication/internal/main_board_modbus/device_param_sync.c", "Application/system_param/cpu3_comm_display_params.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "长按确认进入主菜单"},
            {"id": "b", "kind": "process", "x": 350, "y": 150, "w": 420, "h": 82, "label": "主菜单选择测量命令、参数配置、调试指令、语言或退出"},
            {"id": "c", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "是否需要密码或数值输入？"},
            {"id": "d", "kind": "process", "x": 90, "y": 455, "w": 300, "h": 90, "label": "无参命令：进入是否下发确认页"},
            {"id": "e", "kind": "process", "x": 730, "y": 455, "w": 300, "h": 90, "label": "带参/参数：输入数值、小数点、符号或密码"},
            {"id": "f", "kind": "decision", "x": 405, "y": 620, "w": 310, "h": 108, "label": "是否属于保护操作？"},
            {"id": "g", "kind": "process", "x": 90, "y": 790, "w": 300, "h": 90, "label": "普通确认：直接执行命令或参数写入"},
            {"id": "h", "kind": "process", "x": 730, "y": 790, "w": 300, "h": 90, "label": "保护确认：二次确认后才执行"},
            {"id": "i", "kind": "end", "x": 430, "y": 950, "w": 260, "h": 62, "label": "ACK 成功后反馈；失败提示通信故障并返回"},
        ],
        "overview_edges": [
            {"d": "M 560 102 L 560 150"},
            {"d": "M 560 232 L 560 285"},
            {"d": "M 405 339 C 280 380 240 420 240 455", "label": "否"},
            {"d": "M 715 339 C 840 380 880 420 880 455", "label": "是"},
            {"d": "M 240 545 C 330 580 450 600 560 620"},
            {"d": "M 880 545 C 790 580 670 600 560 620"},
            {"d": "M 405 674 C 300 720 250 760 240 790", "label": "否"},
            {"d": "M 715 674 C 820 720 870 760 880 790", "label": "是"},
            {"d": "M 240 880 C 330 930 470 945 560 950"},
            {"d": "M 880 880 C 790 930 650 945 560 950"},
        ],
        "flows": [
            {
                "title": "菜单页面选择与按键分发",
                "caption": "func_index 指向当前页面，KeyProcess 根据 keymenu 表里的 back/up/down/sure 函数指针执行页面动作。",
                "height": 980,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "长按确认进入主菜单"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 130, "w": 420, "h": 82, "label": "主菜单显示测量命令、参数配置、调试指令、语言和退出"},
                    {"id": "p2", "kind": "process", "x": 350, "y": 260, "w": 420, "h": 82, "label": "菜单选择根据上下键改变当前选项和页码"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 400, "w": 310, "h": 108, "label": "按确认进入哪类菜单？"},
                    {"id": "m1", "kind": "process", "x": 40, "y": 570, "w": 240, "h": 90, "label": "测量命令：回零、找液位、水位、密度、分布等"},
                    {"id": "m2", "kind": "process", "x": 320, "y": 570, "w": 240, "h": 90, "label": "参数配置：密码通过后按分组构建参数菜单"},
                    {"id": "m3", "kind": "process", "x": 600, "y": 570, "w": 240, "h": 90, "label": "调试指令：运动、标定、维护、无线匹配等"},
                    {"id": "m4", "kind": "state", "x": 880, "y": 570, "w": 200, "h": 90, "label": "语言/退出：修改本地显示或退出菜单"},
                    {"id": "end", "kind": "end", "x": 430, "y": 780, "w": 260, "h": 62, "label": "等待下一次按键分发"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 560 212 L 560 260"},
                    {"d": "M 560 342 L 560 400"},
                    {"d": "M 405 454 C 260 500 170 530 160 570"},
                    {"d": "M 500 508 L 440 570"},
                    {"d": "M 620 508 L 720 570"},
                    {"d": "M 715 454 C 840 500 960 530 980 570"},
                    {"d": "M 160 660 C 260 730 430 760 560 780"},
                    {"d": "M 440 660 C 485 720 520 755 560 780"},
                    {"d": "M 720 660 C 670 720 620 755 560 780"},
                    {"d": "M 980 660 C 850 740 690 770 560 780"},
                ],
                "evidence": [
                    {"title": "主菜单", "text": f"{code(func_line(MENU_SOURCE, 'mainmenu'))} mainmenu 定义五个主入口。"},
                    {"title": "按键分发", "text": f"{code(func_line(MENU_SOURCE, 'DisplayTankOpera_CanProcessKey'))} 和 {code(func_line(MENU_SOURCE, 'KeyProcess'))} 使用 keymenu 表进行函数指针分发。"},
                    {"title": "菜单构建", "text": f"{code(func_line(MENU_SOURCE, 'menu_build_by_group'))} 和 {code(func_line(MENU_SOURCE, 'menu_build_by_filter'))} 按参数元数据动态构建参数页。"},
                ],
            },
            {
                "title": "参数/指令输入、确认和下发",
                "caption": "无参指令直接确认，带参指令和参数先输入数值；关键参数会进入保护确认。CPU3 本机参数直接保存，CPU2 普通参数/命令必须先通过状态、完整参数、当前连接协议快照、共享协议和故障门禁，再以合法 ACK 作为成功依据；参数刷新期间仅屏幕 CMD_CANCEL_MEASUREMENT 可放宽参数快照。失败时提示通信故障且不进入成功或运动监控页面。",
                "height": 1280,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "选择命令或参数项"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "是否无参指令？"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 300, "w": 300, "h": 90, "label": "无参：进入二次确认页"},
                    {"id": "p2", "kind": "process", "x": 730, "y": 300, "w": 300, "h": 90, "label": "带参/参数：进入数值输入页，设置位数、小数点和符号"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 470, "w": 310, "h": 108, "label": "密码或确认是否通过？"},
                    {"id": "err", "kind": "error", "x": 790, "y": 485, "w": 250, "h": 82, "label": "取消或密码错误：返回上级菜单"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 630, "w": 310, "h": 108, "label": "是否需要保护确认？"},
                    {"id": "p3", "kind": "process", "x": 730, "y": 645, "w": 300, "h": 90, "label": "保护确认页：再次确认后执行"},
                    {"id": "d4", "kind": "decision", "x": 405, "y": 800, "w": 310, "h": 108, "label": "操作对象是 CPU3 本机参数？"},
                    {"id": "p4", "kind": "process", "x": 90, "y": 965, "w": 300, "h": 90, "label": "是：更新 CPU3 本机参数，保存 FRAM，必要时置串口重配"},
                    {"id": "d5", "kind": "decision", "x": 730, "y": 950, "w": 300, "h": 108, "label": "否：CPU2 三类快照完整、协议相等且本次请求获得合法 ACK？"},
                    {"id": "ok", "kind": "state", "x": 410, "y": 1135, "w": 300, "h": 82, "label": "成功：提交镜像并进入成功页/监控页或返回"},
                    {"id": "e2", "kind": "error", "x": 780, "y": 1135, "w": 300, "h": 82, "label": "失败：提示 CPU2 通信故障，不显示旧值或成功反馈"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 405 184 C 290 230 250 260 240 300", "label": "是"},
                    {"d": "M 715 184 C 830 230 870 260 880 300", "label": "否"},
                    {"d": "M 240 390 C 330 430 450 450 560 470"},
                    {"d": "M 880 390 C 790 430 670 450 560 470"},
                    {"d": "M 715 524 L 790 526", "label": "否", "red": True},
                    {"d": "M 560 578 L 560 630", "label": "是"},
                    {"d": "M 715 684 L 730 690", "label": "是"},
                    {"d": "M 560 738 L 560 800", "label": "否/保护后"},
                    {"d": "M 730 735 C 650 760 590 780 560 800"},
                    {"d": "M 405 854 C 300 900 250 930 240 965", "label": "是"},
                    {"d": "M 715 854 C 820 900 870 925 880 950", "label": "否"},
                    {"d": "M 730 1004 C 660 1060 610 1100 560 1135", "label": "是"},
                    {"d": "M 880 1058 L 930 1135", "label": "否", "red": True},
                    {"d": "M 240 1055 C 330 1100 430 1120 560 1135"},
                ],
                "evidence": [
                    {"title": "数值输入", "text": f"{code(func_line(MENU_SOURCE, 'inputcmdpara'))} inputcmdpara 按位数、小数点、符号输入参数值。"},
                    {"title": "确认页", "text": f"{code(func_line(MENU_SOURCE, 'ifsendcmd'))} ifsendcmd 根据操作类型显示确认内容，并用 timesure/timeback 做二次确认。"},
                    {"title": "保护确认", "text": f"{code(func_line(MENU_SOURCE, 'operation_needs_protect_confirm'))} operation_needs_protect_confirm 列出关键操作和通信参数。"},
                    {"title": "CPU2 写门禁和失败反馈", "text": f"{code(func_line(MENU_SOURCE, 'send_cpu2_command'))}、{code(func_line(MENU_SOURCE, 'cmd_nopara_process'))} 和 {code(func_line(MENU_SOURCE, 'para_mainprocess'))} 先检查 CPU2 状态/参数/当前连接协议快照、协议和故障门禁，再以同步请求返回值决定是否进入成功/运动监控页；已发起的参数写失败会由底层立即关闭参数快照并强制补读，参数写后读回失败同样走通信故障提示。"},
                    {"title": "屏幕取消门禁与幂等", "text": f"{code(func_line(MENU_SOURCE, 'Display_RequestCancelMeasurement'))} 在流程已自然结束时按无需取消的成功结果退出；仍需取消时调用 CPU2_CommCanSendCommand，仅在状态和当前连接协议快照有效、协议相等且无故障时绕过参数刷新。"},
                ],
            },
            {
                "title": "AO 输出使能与读取部件参数下发",
                "caption": "当前程序把 AO 输出使能做成 CPU2 设备参数，把读取部件参数做成 CPU2 命令；两者都从菜单进入，但一个写保持寄存器，一个写命令寄存器。",
                "height": 1180,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "用户进入菜单维护/参数相关页面"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 140, "w": 310, "h": 108, "label": "选择 AO 使能还是读取部件参数？"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 310, "w": 300, "h": 90, "label": "AO使能：显示为 0/1 布尔参数，归属 AO 输出分组"},
                    {"id": "p2", "kind": "process", "x": 730, "y": 310, "w": 300, "h": 90, "label": "读取部件参数：进入确认页，下发 CMD_READ_PART_PARAMS"},
                    {"id": "d2", "kind": "decision", "x": 90, "y": 475, "w": 300, "h": 108, "label": "输入值是否在 0..1 范围内？"},
                    {"id": "e1", "kind": "error", "x": 90, "y": 650, "w": 300, "h": 82, "label": "非法值：提示范围错误并留在输入页"},
                    {"id": "p3", "kind": "process", "x": 410, "y": 650, "w": 300, "h": 90, "label": "合法：映射为 CPU2 的 AO 输出使能参数"},
                    {"id": "p4", "kind": "process", "x": 730, "y": 650, "w": 300, "h": 90, "label": "命令确认：写 command，CPU2 进入读取部件参数状态"},
                    {"id": "p5", "kind": "process", "x": 350, "y": 820, "w": 420, "h": 90, "label": "先检查 CPU2 可用性并等待 0x10 ACK；失败提示且不进入成功页"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 990, "w": 310, "h": 108, "label": "下发成功且 CPU2 已发布 RSSI/AO 运行态？"},
                    {"id": "end", "kind": "end", "x": 430, "y": 1140, "w": 260, "h": 62, "label": "状态页显示 RSSI 或故障原因；AO 运行态进入缓存"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 140"},
                    {"d": "M 405 194 C 300 245 250 280 240 310", "label": "AO使能"},
                    {"d": "M 715 194 C 820 245 870 280 880 310", "label": "读取部件"},
                    {"d": "M 240 400 L 240 475"},
                    {"d": "M 240 583 L 240 650", "label": "否", "red": True},
                    {"d": "M 390 529 C 450 570 500 610 560 650", "label": "是"},
                    {"d": "M 880 400 L 880 650"},
                    {"d": "M 560 740 L 560 820"},
                    {"d": "M 880 740 C 800 785 680 805 560 820"},
                    {"d": "M 560 910 L 560 990"},
                    {"d": "M 560 1098 L 560 1140", "label": "是/下一轮"},
                ],
                "evidence": [
                    {"title": "菜单枚举", "text": f"{code('Application/display/display_tankopera.h:394')} 定义 COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE，菜单短名为 AO使能。"},
                    {"title": "AO 分组和范围", "text": f"{code('Application/system_param/system_parameter.c:266')} AO输出使能范围为 0..1，按布尔枚举显示。"},
                    {"title": "同步到 CPU2", "text": f"{code('Communication/internal/main_board_modbus/device_param_sync.c:258')} CPU3 参数镜像把 AoOutputEnable 映射到 CPU2 设备参数；{code(func_line('Communication/internal/main_board_modbus/device_param_sync.c', 'DeviceParams_SyncAllToCPU2'))} 通过内部 Modbus 同步。"},
                    {"title": "读取部件参数", "text": f"{code(func_line(MENU_SOURCE, 'cmd_nopara_process'))} 菜单触发 CMD_READ_PART_PARAMS 后，状态页使用后续轮询到的 RSSI/传感器快照。"},
                ],
            },
        ],
        "issues": [
            {"level": "mid", "title": "确认逻辑依赖 timesure/timeback 计数", "desc": "ifsendcmd 需要连续确认/返回操作，计数变量跨页面使用，状态未清理时可能造成误判。", "suggest": "把确认状态封装为独立结构，进入确认页时统一初始化。", "ref": func_line(MENU_SOURCE, "ifsendcmd")},
            {"level": "mid", "title": "参数范围判断依赖枚举区间", "desc": "dtm_operaname、dtm_points 等多处用 COM_NUM_* START/STOP 区间判断，枚举新增或命名不一致会影响显示和输入。", "suggest": "以 param_meta/显式表为主，减少区间判断；新增操作时要求表驱动校验。", "ref": func_line(MENU_SOURCE, "inputcmdpara")},
            {"level": "mid", "title": "本机通信参数修改后异步重配", "desc": "菜单保存 CPU3 COM 参数后需要等待主循环安全点重配，用户确认保存和实际生效之间存在时间差。", "suggest": "保存后显示“等待串口空闲后生效”，并在重配完成后给出状态反馈。", "ref": "Application/app_main.c:182-221"},
            {"level": "low", "title": "密码错误固定延时阻塞主循环", "desc": "密码错误时 HAL_Delay(500)，会暂停协议处理和 CPU2 轮询。", "suggest": "改成显示超时状态，由 Display_Task 周期返回主菜单。", "ref": func_line(MENU_SOURCE, "ifsendcmd")},
            {"level": "mid", "title": "AO 使能复用了原 reserved26 寄存器位置", "desc": "CPU2/CPU3 当前把 AO_OUTPUT_ENABLE 放在原保留保持寄存器位置；如果现场旧版本仍把该地址当保留位，升级前后可能出现配置含义变化。", "suggest": "在升级说明和参数表里明确 reserved26 已变更为 AO 输出使能，并要求 CPU2/CPU3 成对升级。", "ref": "Application/system_param/stateformodbus.h:378-384"},
            {"level": "mid", "title": "读取部件参数和 AO 运行态缓存都依赖运行轮询刷新", "desc": "菜单下发 ACK 只证明 CPU2 接受了请求，并不代表 RSSI 或 AO 运行态已经更新；仍需等待 CPU2 执行并由 CPU3 下一轮输入寄存器状态帧刷新。", "suggest": "状态页显示等待/更新时间；若后续要显示 AO 状态，需要补状态页快照和绘制字段。", "ref": func_line(CPU2_COMM_SOURCE, "PollingInputData")},
        ],
        "sources": [
            {"title": "mainmenu / measuremenu", "desc": "主菜单和测量命令菜单定义。", "refs": [func_line(MENU_SOURCE, "mainmenu"), func_line(MENU_SOURCE, "measuremenu")]},
            {"title": "KeyProcess", "desc": "按键分发到当前页面函数。", "refs": [func_line(MENU_SOURCE, "KeyProcess")]},
            {"title": "ifsendcmd", "desc": "指令/参数确认页。", "refs": [func_line(MENU_SOURCE, "ifsendcmd")]},
            {"title": "COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE", "desc": "AO 输出使能菜单项。", "refs": ["Application/display/display_tankopera.h:394"]},
            {"title": "DeviceParams_SyncAllToCPU2", "desc": "本地参数与 CPU2 参数差异同步。", "refs": [func_line("Communication/internal/main_board_modbus/device_param_sync.c", "DeviceParams_SyncAllToCPU2")]},
        ],
    }


def param_fram_io_page() -> dict:
    return {
        "id": "cpu3-08",
        "file": "08_本机参数FRAM时钟与外设恢复.html",
        "title": "CPU3 本机参数、FRAM、时钟与外设恢复程序流程",
        "short": "本机参数与外设恢复",
        "hero": "梳理 CPU3 本机通信/显示参数默认值、FRAM V3/V4/V5 到当前 V6（0x0006）的迁移、协议切换落盘与单端口重配、RTC 初始化和外设错误恢复。",
        "entry": "App_Init 调用 Cpu3Clock_Init、读取 CPU3 本机 FRAM 参数、Cpu3_ReinitAllUarts；运行期菜单、协议切换应答完成或错误回调触发保存和恢复。",
        "summary": "CPU3 本机参数独立于 CPU2 设备参数并存储在 FRAM 中。统一协议切换 ACK 按旧参数完成后保持 RX 停止，主循环通过 Cpu3Local_WriteValueChecked 保存并读回校验；成功才重配当前 COM 口，失败恢复旧配置。",
        "overview_text": "本页是 CPU3 的本地支撑层：它决定显示/通信参数如何加载、保存、生效和恢复，也承接统一协议切换从旧参数应答到新参数启用的安全边界。",
        "overview_description": "CPU3 应用初始化时从 FRAM 加载或迁移本机参数，无效时恢复默认并重新保存。运行期协议切换先用旧串口参数发完独立 ACK，中断标记 APPLY 后保持 RX DMA 停止；主循环调用 Cpu3Local_WriteValueChecked 写入目标协议、套用标准串口参数并读回校验。通过后只重初始化收到命令的端口，失败则恢复旧配置和旧串口参数。",
        "commands": ["CPU3_PARAM_VERSION=0x0006", "CPU3_PARAM_VERSION_V3", "CPU3_PARAM_VERSION_V4", "CPU3_PARAM_VERSION_V5", "CPU3_CLOCK_BKP_MARKER"],
        "source_files": ["Application/system_param/cpu3_comm_display_params.c", "Application/system_param/mb85rs2m.c", "Application/system_param/cpu3_clock.c", "Application/display/hgs.c", "Application/app_main.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "CPU3 初始化、参数保存或协议切换应用"},
            {"id": "b", "kind": "process", "x": 90, "y": 170, "w": 300, "h": 90, "label": "RTC 优先使用 LSE，失败回退 LSI，并按 BKP 标记保留校时时间"},
            {"id": "c", "kind": "process", "x": 410, "y": 170, "w": 300, "h": 90, "label": "FRAM 读取 CPU3 参数镜像，校验 magic/version/CRC"},
            {"id": "d", "kind": "process", "x": 730, "y": 170, "w": 300, "h": 90, "label": "OLED/UART 运行期按错误回调或错误计数恢复"},
            {"id": "e", "kind": "decision", "x": 410, "y": 345, "w": 300, "h": 108, "label": "FRAM 参数是否有效或可迁移？"},
            {"id": "f", "kind": "process", "x": 90, "y": 530, "w": 300, "h": 90, "label": "无效：加载默认参数、应用显示运行参数并保存"},
            {"id": "g", "kind": "process", "x": 730, "y": 530, "w": 300, "h": 90, "label": "有效：加载参数，补齐固件版本和协议推荐串口参数"},
            {"id": "h", "kind": "end", "x": 430, "y": 720, "w": 260, "h": 62, "label": "重配 COM1/2/3 并进入运行"},
        ],
        "overview_edges": [
            {"d": "M 560 102 C 390 130 260 145 240 170"},
            {"d": "M 560 102 L 560 170"},
            {"d": "M 560 102 C 730 130 860 145 880 170"},
            {"d": "M 560 260 L 560 345"},
            {"d": "M 410 399 C 300 450 250 500 240 530", "label": "否"},
            {"d": "M 710 399 C 820 450 870 500 880 530", "label": "是"},
            {"d": "M 240 620 C 330 690 470 710 560 720"},
            {"d": "M 880 620 C 790 690 650 710 560 720"},
        ],
        "flows": [
            {
                "title": "CPU3 FRAM 参数加载、迁移和保存",
                "caption": "FRAM 镜像包含 magic、version、params、crc；V3/V4/V5 可迁移到当前 V6（0x0006），无效或 CRC 错误时使用默认值并写回。",
                "height": 1160,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "读取 CPU3 本机 FRAM 参数"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 130, "w": 420, "h": 82, "label": "从 CPU3 参数地址读取本机参数镜像"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 270, "w": 310, "h": 108, "label": "参数标识正确且为 V3/V4/V5 旧格式？"},
                    {"id": "p2", "kind": "process", "x": 780, "y": 285, "w": 260, "h": 90, "label": "按旧版结构校验 CRC，迁移字段并补新增默认值"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 430, "w": 310, "h": 108, "label": "参数标识、版本和 CRC 是否为当前 V6 有效？"},
                    {"id": "p3", "kind": "error", "x": 80, "y": 600, "w": 300, "h": 90, "label": "无效：初始化默认参数，应用显示运行参数"},
                    {"id": "p4", "kind": "process", "x": 730, "y": 600, "w": 300, "h": 90, "label": "有效：加载为 CPU3 本机运行参数"},
                    {"id": "p5", "kind": "process", "x": 350, "y": 760, "w": 420, "h": 86, "label": "补齐固件版本、归一化端口参数、应用显示运行参数"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 900, "w": 310, "h": 108, "label": "是否需要写回 FRAM？"},
                    {"id": "p6", "kind": "state", "x": 790, "y": 915, "w": 250, "h": 82, "label": "构造镜像并 CRC32；与现有有效镜像相同则跳过"},
                    {"id": "end", "kind": "end", "x": 430, "y": 1060, "w": 260, "h": 62, "label": "加载完成"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 560 212 L 560 270"},
                    {"d": "M 715 324 L 780 330", "label": "是"},
                    {"d": "M 560 378 L 560 430", "label": "否"},
                    {"d": "M 405 484 C 280 530 240 570 230 600", "label": "否", "red": True},
                    {"d": "M 715 484 C 820 530 870 570 880 600", "label": "是"},
                    {"d": "M 230 690 C 330 735 450 750 560 760"},
                    {"d": "M 880 690 C 790 735 670 750 560 760"},
                    {"d": "M 910 375 C 900 520 750 670 560 760"},
                    {"d": "M 560 846 L 560 900"},
                    {"d": "M 715 954 L 790 956", "label": "是"},
                    {"d": "M 560 1008 L 560 1060", "label": "否"},
                    {"d": "M 790 997 C 700 1030 620 1045 560 1060"},
                ],
                "evidence": [
                    {"title": "存储结构", "text": f"{code(f'{FRAM_SOURCE}:794-948')} 定义当前 V6（0x0006）、V3/V4/V5 旧版参数镜像、CRC 字段和版本校验。"},
                    {"title": "加载流程", "text": f"{code(func_line(FRAM_SOURCE, 'Cpu3_Params_LoadFromFRAM'))} 读取 CPU3 本机 FRAM 参数，处理 V3/V4/V5 到 V6 的迁移、V6 校验和默认值回退。"},
                    {"title": "保存与读回", "text": f"{code(func_line(FRAM_SOURCE, 'Cpu3_Params_SaveToFRAM'))} 保存前构造 V6 目标镜像并判重；真实写入后重新读取，校验存储 CRC 和参数镜像。"},
                ],
            },
            {
                "title": "统一协议切换的保存与单端口生效",
                "caption": "切换请求只在 RAM 中暂存；独立 ACK 发送成功后保持 RX 停止。主循环保存并读回校验，通过后启用新参数，失败时恢复旧配置。",
                "height": 1240,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "合法协议切换帧已接受"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 140, "w": 420, "h": 90, "label": "暂存目标协议，按旧串口参数发送独立 ACK"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 300, "w": 310, "h": 108, "label": "切换应答最后一帧是否发送成功？"},
                    {"id": "err", "kind": "error", "x": 70, "y": 470, "w": 320, "h": 90, "label": "DMA 启动失败或发送期 UART 错误：取消尚未完成的切换"},
                    {"id": "p2", "kind": "state", "x": 730, "y": 470, "w": 300, "h": 90, "label": "Tx 完成中断标记 APPLY，并保持目标端口 RX DMA 停止"},
                    {"id": "p3", "kind": "process", "x": 350, "y": 650, "w": 420, "h": 96, "label": "Cpu3Local_WriteValueChecked 保存目标协议并执行 FRAM 写后读回"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 810, "w": 310, "h": 108, "label": "FRAM 读回 CRC 与参数是否一致？"},
                    {"id": "rollback", "kind": "error", "x": 70, "y": 980, "w": 320, "h": 90, "label": "失败：恢复旧配置、回写旧参数并按旧参数恢复接收"},
                    {"id": "p4", "kind": "process", "x": 730, "y": 980, "w": 300, "h": 90, "label": "成功：Cpu3_ReinitPortUart 只重配当前 COM 口"},
                    {"id": "end", "kind": "end", "x": 430, "y": 1130, "w": 260, "h": 62, "label": "新协议生效或安全保持旧协议"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 140"},
                    {"d": "M 560 230 L 560 300"},
                    {"d": "M 405 354 C 300 400 250 440 230 470", "label": "否", "red": True},
                    {"d": "M 715 354 C 820 400 870 440 880 470", "label": "是"},
                    {"d": "M 880 560 C 790 610 680 630 560 650"},
                    {"d": "M 560 746 L 560 810"},
                    {"d": "M 405 864 C 300 910 250 950 230 980", "label": "否", "red": True},
                    {"d": "M 715 864 C 820 910 870 950 880 980", "label": "是"},
                    {"d": "M 230 1070 C 330 1110 450 1120 560 1130"},
                    {"d": "M 880 1070 C 790 1110 670 1120 560 1130"},
                ],
                "evidence": [
                    {"title": "应答完成边界", "text": f"{code(func_line('Application/app_main.c', 'cpu3_mark_protocol_switch_tx_complete'))} 最后一帧发送完成后改为 PENDING_APPLY，Tx 回调不再按旧参数重启 RX DMA。"},
                    {"title": "主循环保存", "text": f"{code(func_line('Application/app_main.c', 'cpu3_apply_ready_protocol_switches'))} 调用 Cpu3Local_WriteValueChecked 保存并检查读回结果，失败时恢复旧配置。"},
                    {"title": "单端口重配", "text": f"{code(func_line(FRAM_SOURCE, 'Cpu3_ReinitPortUart'))} 仅选择目标端口的 UART、配置和 RX DMA 缓冲区，不中断其它 COM 口。"},
                ],
            },
            {
                "title": "RTC 与外设错误恢复",
                "caption": "RTC 初始化只服务协议时间展示；LSE 缺失时回退 LSI，屏幕菜单可校时；UART 和 OLED 错误恢复分别由 HAL 回调和显示任务触发，避免外设错误长期卡住显示端。",
                "height": 1040,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "进入本地外设支撑流程"},
                    {"id": "p1", "kind": "process", "x": 70, "y": 170, "w": 300, "h": 90, "label": "RTC：打开 PWR/备份域，优先等待 LSE，失败启用 LSI"},
                    {"id": "p2", "kind": "process", "x": 410, "y": 170, "w": 300, "h": 90, "label": "UART：Tx 完成或错误回调恢复接收 DMA"},
                    {"id": "p3", "kind": "process", "x": 750, "y": 170, "w": 300, "h": 90, "label": "OLED：检测 SPI 错误计数或周期恢复请求"},
                    {"id": "d1", "kind": "decision", "x": 70, "y": 340, "w": 300, "h": 108, "label": "RTC BKP 标记和寄存器时间是否有效？"},
                    {"id": "p4", "kind": "state", "x": 70, "y": 515, "w": 300, "h": 90, "label": "无效：写默认日期 2026-01-01；菜单校时后写 SET 标记"},
                    {"id": "d2", "kind": "decision", "x": 410, "y": 340, "w": 300, "h": 108, "label": "UART 错误来自哪一路？"},
                    {"id": "p5", "kind": "state", "x": 410, "y": 515, "w": 300, "h": 90, "label": "COM1/2/3 释放 busy/pending；UART5 清 wait_response"},
                    {"id": "d3", "kind": "decision", "x": 750, "y": 340, "w": 300, "h": 108, "label": "是否需要 OLED 恢复清屏？"},
                    {"id": "p6", "kind": "state", "x": 750, "y": 515, "w": 300, "h": 90, "label": "恢复清屏并标记本帧绘制完成"},
                    {"id": "end", "kind": "end", "x": 430, "y": 760, "w": 260, "h": 62, "label": "返回运行流程"},
                ],
                "edges": [
                    {"d": "M 560 97 C 390 120 240 140 220 170"},
                    {"d": "M 560 97 L 560 170"},
                    {"d": "M 560 97 C 730 120 880 140 900 170"},
                    {"d": "M 220 260 L 220 340"},
                    {"d": "M 220 448 L 220 515", "label": "否"},
                    {"d": "M 560 260 L 560 340"},
                    {"d": "M 560 448 L 560 515"},
                    {"d": "M 900 260 L 900 340"},
                    {"d": "M 900 448 L 900 515", "label": "是"},
                    {"d": "M 220 605 C 330 700 460 735 560 760"},
                    {"d": "M 560 605 L 560 760"},
                    {"d": "M 900 605 C 790 700 660 735 560 760"},
                ],
                "evidence": [
                    {"title": "RTC 初始化", "text": f"{code('cpu3_clock.c:251-405')} Cpu3Clock_Init 优先使用 LSE，失败回退 LSI；只有 BKP 标记或时间非法时写默认日期。"},
                    {"title": "UART 错误恢复", "text": f"{code('Application/app_main.c:662-700')} HAL_UART_ErrorCallback 分别恢复外部 COM 和 UART5。"},
                    {"title": "OLED 恢复", "text": f"{code('display.c:1872-1903')} Display_ShouldRecoverBeforeDraw 根据请求、SPI 错误和周期条件恢复。"},
                ],
            },
        ],
        "issues": [
            {"level": "mid", "title": "FRAM 无效会静默恢复默认通信参数", "desc": "magic/version/CRC 无效时直接初始化默认并保存，现场串口协议配置可能回到默认值。", "suggest": "在显示端增加“参数恢复默认”提示或错误计数，便于现场定位 FRAM/升级问题。", "ref": func_line(FRAM_SOURCE, "Cpu3_Params_LoadFromFRAM")},
            {"level": "low", "title": "LSI 兜底时时间精度仍有限", "desc": "新板优先使用 LSE，旧板或 LSE 启动失败时会回退 LSI；LSI 长期漂移仍可能影响时间戳精度。", "suggest": "现场测试时应确认菜单显示的 RTC 状态，LSE 异常时按硬件问题排查晶振和 VBAT。", "ref": "cpu3_clock.c:251-335"},
            {"level": "low", "title": "UART5 空闲期错误不单独置故障", "desc": "UART5 错误只在存在 CPU2 同步请求时置待处理标志并计入本次失败；空闲期硬件错误本身不直接置故障，后续请求失败时才进入连续失败判定。", "suggest": "如需区分物理层空闲错误和请求失败，可增加独立诊断计数，但保持 ISR 只记录、主循环处理。", "ref": "Application/app_main.c:650-705"},
            {"level": "low", "title": "FRAM 保存判重只比较 params", "desc": "magic/version/CRC 有效且 params 相同就跳过写入，若保留字段策略变化需确认是否仍满足升级需求。", "suggest": "版本升级时复核 Cpu3_Params_BuildStorage 和判重条件。", "ref": func_line(FRAM_SOURCE, "Cpu3_Params_SaveToFRAM")},
        ],
        "sources": [
            {"title": "读取 CPU3 本机 FRAM 参数", "desc": "CPU3 本机参数 V3/V4/V5 到 V6 的迁移、加载和默认回退。", "refs": [func_line(FRAM_SOURCE, "Cpu3_Params_LoadFromFRAM")]},
            {"title": "Cpu3_Params_SaveToFRAM", "desc": "CPU3 本机参数 V6 镜像构造、判重、写入和写后读回校验。", "refs": [func_line(FRAM_SOURCE, "Cpu3_Params_SaveToFRAM"), func_line(FRAM_SOURCE, "Cpu3_Params_BuildStorage")]},
            {"title": "Cpu3Local_WriteValueChecked", "desc": "保存目标协议、套用标准串口参数并返回 FRAM 持久化结果。", "refs": [func_line(FRAM_SOURCE, "Cpu3Local_WriteValueChecked")]},
            {"title": "Cpu3_ReinitPortUart", "desc": "协议切换应答完成后只重初始化收到命令的外部 COM 口。", "refs": [func_line(FRAM_SOURCE, "Cpu3_ReinitPortUart")]},
            {"title": "Cpu3Clock_Init", "desc": "CPU3 本机 RTC 初始化、LSE/LSI 选择和备份标记处理。", "refs": ["Application/system_param/cpu3_clock.c:344-405"]},
            {"title": "ReadMultiData / WriteMultiData", "desc": "MB85RS2M FRAM 多字节读写。", "refs": ["Application/system_param/mb85rs2m.c:112-150"]},
        ],
    }


def main() -> None:
    append_remaining_pages()
    DOC_DIR.mkdir(parents=True, exist_ok=True)
    ASSET_DIR.mkdir(parents=True, exist_ok=True)
    for asset in ["流程文档样式.css", "流程文档交互.js"]:
        shutil.copy2(CPU2_ASSET_DIR / asset, ASSET_DIR / asset)
    css_path = ASSET_DIR / "流程文档样式.css"
    css_text = css_path.read_text(encoding="utf-8")
    if CPU3_STYLE_APPEND not in css_text:
        css_path.write_text(css_text + CPU3_STYLE_APPEND, encoding="utf-8", newline="\n")

    for page in PAGES:
        (DOC_DIR / page["file"]).write_text(page_html(page, PAGES), encoding="utf-8", newline="\n")

    (DOC_DIR / "CPU3程序流程总览.html").write_text(overview_page(PAGES), encoding="utf-8", newline="\n")
    (DOC_DIR / "README.md").write_text(readme(PAGES), encoding="utf-8", newline="\n")

    root_readme = ROOT / "docs" / "00_构建与版本" / "CPU3文档索引.md"
    if root_readme.exists():
        text = root_readme.read_text(encoding="utf-8", errors="replace")
        text = re.sub(r"更新日期：\d{4}-\d{2}-\d{2}", f"更新日期：{DOC_DATE}", text)
        root_readme.write_text(text, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()

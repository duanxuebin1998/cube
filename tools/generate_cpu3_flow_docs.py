from __future__ import annotations

import html
import json
import re
import shutil
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPU3 = ROOT / "LTD_DISPLAY_CPU3"
DOC_DIR = CPU3 / "docs" / "00_程序流程"
ASSET_DIR = DOC_DIR / "assets"
CPU2_ASSET_DIR = ROOT / "LTD_MAIN_CPU2" / "docs" / "00_程序流程" / "assets"
CSS_VERSION = "cpu3-20260616-flow-audit"
DOC_DATE = "2026-06-16"


def read_version_macro(path: Path, macro: str, fallback: str) -> str:
    text = path.read_text(encoding="utf-8", errors="replace")
    match = re.search(rf'#define\s+{re.escape(macro)}\s+"([^"]+)"', text)
    if match:
        return match.group(1)
    return fallback


CPU3_VERSION = read_version_macro(CPU3 / "Application" / "app_version.h", "CPU3_APP_VERSION_STRING", "V1.15.0.0")


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
    m = re.search(rf"(?m)^.*\b{name}\s*\([^;]*\)\s*\{{", text)
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
    def __init__(self, diagram_id: str, width: int = 1120, height: int = 760):
        self.diagram_id = diagram_id
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

    def render(self, title: str) -> str:
        return (
            f'<div class="svg-flow-wrap"><svg class="biz-flow" '
            f'viewBox="0 0 {self.width} {self.height}" role="img" aria-label="{esc(title)}">'
            f"{self.defs()}{''.join(self.parts)}</svg></div>"
        )


def svg_required_height(nodes: list[dict], edges: list[dict], minimum: int) -> int:
    max_y = 0.0
    for node in nodes:
        max_y = max(max_y, float(node["y"]) + float(node.get("h", 80)))
    for edge in edges:
        values = [float(value) for value in re.findall(r"[0-9.]+", edge.get("d", ""))]
        max_y = max(max_y, *(values[1::2] or [0.0]))
    return int(max(minimum, max_y + 90))


def flow_svg(diagram_id: str, title: str, nodes: list[dict], edges: list[dict], width: int = 1120, height: int = 760) -> str:
    b = SvgBuilder(diagram_id, width, svg_required_height(nodes, edges, height))
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
    return b.render(title)


def source_index(items: list[dict]) -> str:
    rows = []
    for item in items:
        refs = " ".join(code(ref) for ref in item.get("refs", []))
        rows.append(
            f'<div class="source-card"><h4>{esc(item["title"])}</h4>'
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
        + flow_svg(page["id"] + "-overview", page["title"] + "总览", page["overview_nodes"], page["overview_edges"], height=page.get("overview_height", 720))
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
        flow_parts.append(flow_svg(page["id"] + f"-flow-{idx}", flow["title"], flow["nodes"], flow["edges"], height=flow.get("height", 860)))
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
        f'<title>{esc(page["title"])}</title><link rel="stylesheet" href="assets/流程文档样式.css?v={CSS_VERSION}"></head>'
        '<body><div class="wrap">'
        f'<header class="hero"><h1>{esc(page["title"])}</h1><p>{esc(page["hero"])}</p>'
        '<div class="meta-grid">'
        f'<div class="meta"><span>项目</span><strong>LTD_DISPLAY_CPU3 / CPU3 {esc(page.get("version", CPU3_VERSION))}</strong></div>'
        f'<div class="meta"><span>功能域</span><strong>{esc(page["short"])}</strong></div>'
        f'<div class="meta"><span>核心源码</span><strong>{esc(", ".join(page.get("source_files", [])[:2]))}</strong></div>'
        f'<div class="meta"><span>本页流程图</span><strong>{len(page["flows"]) + 1} 张业务级 SVG</strong></div>'
        '</div></header>'
        + nav
        + layer
        + overview
        + "".join(flow_parts)
        + risk
        + src
        + '<script src="assets/流程文档交互.js"></script></div></body></html>'
    )


def overview_page(pages: list[dict]) -> str:
    nodes = [
        {"id": "n1", "kind": "start", "x": 440, "y": 35, "w": 240, "h": 62, "label": "CPU3 上电进入主循环"},
        {"id": "n2", "kind": "process", "x": 90, "y": 160, "w": 260, "h": 80, "label": "初始化显示、RTC、FRAM 参数、串口和 DSM 地址"},
        {"id": "n3", "kind": "loop", "x": 430, "y": 160, "w": 260, "h": 80, "label": "主循环调度显示任务、外部 COM 和 CPU2 轮询"},
        {"id": "n4", "kind": "process", "x": 770, "y": 160, "w": 260, "h": 80, "label": "外部协议请求通过 COM1/2/3 分发到 DSM、Wartsila、SI7000"},
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
        f'<title>CPU3 程序流程总览</title><link rel="stylesheet" href="assets/流程文档样式.css?v={CSS_VERSION}"></head>'
        '<body><div class="wrap"><header class="hero"><h1>CPU3 程序流程总览</h1>'
        '<p>按 CPU2 文档同等标准整理 CPU3 显示端、协议网关、参数镜像、按键菜单和外设恢复流程。所有页面均保留一套业务级程序流程图，不把命令流程和代码梳理拆开。</p>'
        f'<div class="meta-grid"><div class="meta"><span>项目</span><strong>LTD_DISPLAY_CPU3 / CPU3 {esc(CPU3_VERSION)} / 协议 11</strong></div><div class="meta"><span>页面数量</span><strong>'
        + str(len(pages))
        + f'</strong></div><div class="meta"><span>整理日期</span><strong>{DOC_DATE}</strong></div><div class="meta"><span>文档风格</span><strong>业务级 SVG + 源码证据</strong></div></div></header>'
        '<nav class="topnav"><a href="#overview">总览图</a><a href="#pages">页面入口</a><a href="#search">全文索引</a><a href="../README.md">CPU3 docs</a></nav>'
        '<section id="overview"><h2>1. CPU3 总体业务流</h2><p class="lead">CPU3 的核心职责不是直接测量，而是在显示端把 CPU2 状态、外部协议、菜单参数和本地持久化连接起来。</p>'
        + flow_svg("cpu3-all", "CPU3 总体业务流", nodes, edges, height=740)
        + '</section><section id="pages"><h2>2. 文档入口</h2><div class="overview-grid">'
        + cards
        + '</div></section><section id="search"><h2>3. 全文索引</h2><input class="doc-search" data-global-search placeholder="输入指令、函数、文件、状态或问题关键词"><div class="search-results" data-global-search-results></div></section>'
        f'<script type="application/json" id="doc-search-data">{json.dumps(search_data, ensure_ascii=False).replace("</", "<\\/")}</script>'
        '<script src="assets/流程文档交互.js"></script></div></body></html>'
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
        "summary": "主循环先处理串口重配和显示任务，再处理 COM1/COM2/COM3 外部协议帧；只有没有外部帧时才轮询 CPU2。",
        "overview_text": "CPU3 主循环是显示端的调度中心。它不是 RTOS 任务，而是单个 while(1) 中按固定优先级轮询不同业务。",
        "commands": [],
        "source_files": ["Core/Src/main.c", "Application/app_main.c", "Core/Src/usart.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 440, "y": 40, "w": 240, "h": 62, "label": "CPU3 上电进入启动入口"},
            {"id": "b", "kind": "process", "x": 350, "y": 145, "w": 420, "h": 82, "label": "HAL、系统时钟和 GPIO/DMA/UART/SPI/TIM/IWDG 初始化"},
            {"id": "c", "kind": "process", "x": 350, "y": 275, "w": 420, "h": 82, "label": "应用初始化显示、RTC、FRAM 参数、串口配置和 DSM 地址"},
            {"id": "d", "kind": "loop", "x": 350, "y": 405, "w": 420, "h": 82, "label": "主循环周期执行应用调度"},
            {"id": "e", "kind": "decision", "x": 350, "y": 535, "w": 420, "h": 110, "label": "本轮是否有外部 COM 帧或显示按键事件？"},
            {"id": "f", "kind": "process", "x": 90, "y": 690, "w": 300, "h": 86, "label": "有事件：优先处理显示/协议，不执行 CPU2 轮询延时"},
            {"id": "g", "kind": "process", "x": 730, "y": 690, "w": 300, "h": 86, "label": "无事件：轮询 CPU2 并延时 10ms"},
        ],
        "overview_edges": [
            {"d": "M 560 102 L 560 145"},
            {"d": "M 560 227 L 560 275"},
            {"d": "M 560 357 L 560 405"},
            {"d": "M 560 487 L 560 535"},
            {"d": "M 350 590 C 250 620 230 650 240 690", "label": "是"},
            {"d": "M 770 590 C 870 620 890 650 880 690", "label": "否"},
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
                "caption": "主循环先消化串口重配置和显示输入，再处理外部 COM 帧；只有本轮没有外部工作时才访问 CPU2，避免协议响应被 CPU2 轮询阻塞。",
                "height": 1040,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 440, "y": 35, "w": 240, "h": 62, "label": "进入应用主循环"},
                    {"id": "n1", "kind": "process", "x": 360, "y": 125, "w": 400, "h": 78, "label": "检查串口重配请求，只在安全点重配"},
                    {"id": "n2", "kind": "process", "x": 360, "y": 245, "w": 400, "h": 78, "label": "执行显示任务，消费按键并刷新状态页"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 365, "w": 310, "h": 108, "label": "COM1 是否收到完整帧？"},
                    {"id": "p1", "kind": "process", "x": 790, "y": 378, "w": 250, "h": 82, "label": "按 COM1 配置分发协议并发送响应"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 525, "w": 310, "h": 108, "label": "COM2 是否收到完整帧？"},
                    {"id": "p2", "kind": "process", "x": 790, "y": 538, "w": 250, "h": 82, "label": "按 COM2 配置分发协议并发送响应"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 685, "w": 310, "h": 108, "label": "COM3 是否收到完整帧？"},
                    {"id": "p3", "kind": "process", "x": 790, "y": 698, "w": 250, "h": 82, "label": "按 COM3 配置分发协议并发送响应"},
                    {"id": "d4", "kind": "decision", "x": 405, "y": 845, "w": 310, "h": 108, "label": "本轮是否没有处理任何事件？"},
                    {"id": "poll", "kind": "process", "x": 90, "y": 858, "w": 250, "h": 82, "label": "轮询 CPU2 并延时 10ms"},
                    {"id": "e", "kind": "end", "x": 800, "y": 910, "w": 230, "h": 62, "label": "返回主循环下一轮"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 125"},
                    {"d": "M 560 203 L 560 245"},
                    {"d": "M 560 323 L 560 365"},
                    {"d": "M 715 419 L 790 419", "label": "是"},
                    {"d": "M 560 473 L 560 525", "label": "否"},
                    {"d": "M 715 579 L 790 579", "label": "是"},
                    {"d": "M 560 633 L 560 685", "label": "否"},
                    {"d": "M 715 739 L 790 739", "label": "是"},
                    {"d": "M 560 793 L 560 845", "label": "否"},
                    {"d": "M 405 899 L 340 899", "label": "是"},
                    {"d": "M 715 899 L 800 941", "label": "否"},
                    {"d": "M 1040 419 C 1080 560 1080 850 1030 941"},
                    {"d": "M 1040 579 C 1080 690 1080 850 1030 941"},
                    {"d": "M 1040 739 C 1080 800 1080 880 1030 941"},
                    {"d": "M 340 899 C 480 985 650 995 800 941"},
                ],
                "evidence": [
                    {"title": "调度顺序", "text": f"{code('Application/app_main.c:387-555')} 中依次调用串口重配、Display_Task、COM1/COM2/COM3、空闲 CPU2 轮询。"},
                    {"title": "空闲轮询", "text": f"{code('Application/app_main.c:550-555')} did_work 为 0 才执行 PollingInputData 和 10ms 延时。"},
                    {"title": "协议分发", "text": f"{code('Application/app_main.c:342-360')} 根据端口参数选择 DSM/Wartsila/SI7000/LTD 协议处理函数。"},
                ],
            },
        ],
        "issues": [
            {"level": "high", "title": "CPU2 轮询是同步阻塞等待响应", "desc": "CPU2_CombinatePackage_Send 内部等待 wait_response，超时时间注释写 100ms 但判断为 1000 tick；轮询期间外部 COM 和显示刷新都会被主循环阻塞。", "suggest": "把 CPU2 轮询改成非阻塞状态机，或至少将超时常量、注释和实际 tick 对齐，并统计超时次数。", "ref": "Communication/internal/main_board_modbus/cpu2_communicate.c:300-352"},
            {"level": "mid", "title": "主循环优先级会让 CPU2 数据刷新受外部协议流量影响", "desc": "只要 COM1/2/3 持续有帧，本轮 did_work 就不为 0，CPU2 轮询会一直后移，显示数据可能滞后。", "suggest": "增加 CPU2 轮询最大间隔保护，或把 CPU2 状态刷新拆成固定节拍任务。", "ref": "Application/app_main.c:387-555"},
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
        "hero": "梳理 UART5/RS485 与 CPU2 的同步请求、响应解析、上电全量读取、运行输入轮询、参数更新补读、密度分布点读取，以及协议 10 起的 RSSI 和协议 11 起的 AO 运行态尾段解析。",
        "entry": "App_MainLoop 空闲时轮询 CPU2；菜单和外部协议写参数时通过内部 Modbus 组帧下发。",
        "summary": "CPU3 作为 CPU2 的 Modbus 主站，周期读取输入寄存器和保持寄存器，写指令/参数时通过 0x10 下发到 CPU2。",
        "overview_text": "内部通信链路由 UART5 + RS485 实现。读响应刷新测量状态缓存和参数镜像；协议 10 起解析 RSSI，协议 11 起继续解析 AO 运行态缓存。",
        "commands": ["FUNCTIONCODE_READ_HOLDREGISTER", "FUNCTIONCODE_READ_INPUTREGISTER", "FUNCTIONCODE_WRITE_MULREGISTER"],
        "source_files": ["Communication/internal/main_board_modbus/cpu2_communicate.c", "Communication/internal/main_board_modbus/dataanalysis_modbus.c", "Application/system_param/stateformodbus.h"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "CPU3 需要访问 CPU2"},
            {"id": "b", "kind": "decision", "x": 405, "y": 145, "w": 310, "h": 108, "label": "是上电/运行轮询还是写参数/指令？"},
            {"id": "c", "kind": "process", "x": 90, "y": 310, "w": 300, "h": 90, "label": "轮询：按分组发送 03/04 读寄存器"},
            {"id": "d", "kind": "process", "x": 730, "y": 310, "w": 300, "h": 90, "label": "写入：组 0x10 多寄存器帧下发命令或参数"},
            {"id": "e", "kind": "process", "x": 360, "y": 485, "w": 400, "h": 86, "label": "UART5 切发送 DMA，等待 CPU2 响应"},
            {"id": "f", "kind": "decision", "x": 405, "y": 625, "w": 310, "h": 108, "label": "响应地址、CRC、功能码有效？"},
            {"id": "g", "kind": "process", "x": 90, "y": 800, "w": 300, "h": 90, "label": "读保持：刷新 CPU2 参数镜像和本地参数缓存"},
            {"id": "h", "kind": "process", "x": 730, "y": 800, "w": 300, "h": 90, "label": "读输入：刷新测量状态缓存和运行态"},
            {"id": "i", "kind": "error", "x": 430, "y": 800, "w": 260, "h": 90, "label": "无效/超时：打印并恢复接收，不更新缓存"},
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
                "caption": "上电阶段读 7 组，包含输入寄存器和保持寄存器；运行阶段只读输入寄存器；参数更新标志变化后补读保持寄存器。",
                "height": 1180,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "主循环空闲时轮询 CPU2"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "上电全量读取完成？"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 285, "w": 300, "h": 90, "label": "上电未完成：每轮读取一组启动参数/状态"},
                    {"id": "p2", "kind": "process", "x": 90, "y": 420, "w": 300, "h": 90, "label": "最后一组完成后同步 Wartsila 保持寄存器并记录参数更新标志"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "有保持寄存器补读任务？"},
                    {"id": "p3", "kind": "process", "x": 730, "y": 285, "w": 300, "h": 90, "label": "按补读队列每次读取一组保持参数"},
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
                    {"title": "轮询分组", "text": f"{code('cpu2_communicate.c:85-154')} 定义上电、运行和参数更新补读三类 PollGroup。"},
                    {"title": "状态机", "text": f"{code('cpu2_communicate.c:156-254')} PollingInputData 维护 poweron_done、runtime_index、hold_refresh_pending 等静态状态。"},
                    {"title": "密度点读取", "text": f"{code('cpu2_communicate.c:256-298')} RequestDensityDistPoints_ByCount 按点数和每点寄存器数分批读取。"},
                ],
            },
            {
                "title": "CPU2 请求发送与响应解析",
                "caption": "CPU3 组帧后切 UART5 到 RS485 发送，等待响应完成，再根据功能码把 CPU2 数据写入保持/输入缓存。",
                "height": 1120,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "需要向 CPU2 发读/写请求"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 135, "w": 420, "h": 86, "label": "按地址、功能码、起始地址、数量和数据组 Modbus RTU 帧"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 275, "w": 310, "h": 108, "label": "UART5 TX DMA 启动成功？"},
                    {"id": "err1", "kind": "error", "x": 790, "y": 290, "w": 250, "h": 82, "label": "失败：切回接收、清等待响应状态并返回"},
                    {"id": "p2", "kind": "process", "x": 350, "y": 440, "w": 420, "h": 86, "label": "等待 CPU2 响应或超时"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 580, "w": 310, "h": 108, "label": "超时或响应帧非法？"},
                    {"id": "err2", "kind": "error", "x": 790, "y": 596, "w": 250, "h": 82, "label": "打印超时/CRC/地址错误，不更新缓存"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 735, "w": 310, "h": 108, "label": "响应功能码是 03/04/10？"},
                    {"id": "p3", "kind": "process", "x": 70, "y": 900, "w": 300, "h": 90, "label": "03：解析保持寄存器，刷新参数元数据和参数缓存"},
                    {"id": "p4", "kind": "process", "x": 410, "y": 900, "w": 300, "h": 90, "label": "04：解析输入寄存器，刷新测量状态缓存"},
                    {"id": "p5", "kind": "state", "x": 750, "y": 900, "w": 300, "h": 90, "label": "10：当前空实现，仅结束等待"},
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
                    {"title": "组帧与等待", "text": f"{code('cpu2_communicate.c:300-353')} CPU2_CombinatePackage_Send 组帧、发送、保存 RCV_* 上下文并等待响应。"},
                    {"title": "发送失败恢复", "text": f"{code('cpu2_communicate.c:355-376')} sendToCPU2 在 TX DMA 启动失败时恢复 UART5 接收。"},
                    {"title": "响应解析", "text": f"{code('cpu2_communicate.c:379-452')} 03/04 分别刷新保持/输入缓存，10 响应暂未解析。"},
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
                    {"title": "协议版本", "text": f"{code('Application/system_param/system_parameter.h:29')} 当前 DEVICE_PROTOCOL_VERSION 为 12，CPU3 使用严格相等判断协议兼容。"},
                ],
            },
        ],
        "issues": [
            {"level": "high", "title": "等待响应期间阻塞主循环", "desc": "CPU2_CombinatePackage_Send 使用 while(wait_response) 同步等待，外部协议响应、显示刷新和按键处理都会被阻塞。", "suggest": "改成 UART5 请求状态机：发送完成、接收完成、超时分别由状态推进；主循环每轮只推进一次。", "ref": "cpu2_communicate.c:300-353"},
            {"level": "high", "title": "密度点读取在一个函数内连续发送多帧", "desc": "RequestDensityDistPoints_ByCount 的 while(total_regs > 0) 会连续调用同步发送，多点数据量大时可能长时间占用主循环。", "suggest": "把密度点读取拆成分帧状态机，每轮只发一帧，并记录已读 offset。", "ref": "cpu2_communicate.c:256-298"},
            {"level": "mid", "title": "0x10 响应未校验写入地址和数量", "desc": "CPU2_Response10Process 当前为空，无法确认 CPU2 回显的起始地址/数量是否与本次写入一致。", "suggest": "解析 0x10 回显并和 RCV_startaddress/RCV_registercnt 对比，失败时设置通信错误计数。", "ref": "cpu2_communicate.c:432-437"},
            {"level": "mid", "title": "输入寄存器尾段强依赖两端协议版本一致", "desc": "协议 10 起增加 RSSI，协议 11 起在 RSSI 后继续追加 AO 运行态。只要 CPU2/CPU3 任何一端协议不一致，尾段字段都会错位。", "suggest": "保持协议版本严格相等提示，并在现场升级清单里要求 CPU2/CPU3 成对升级。", "ref": "stateformodbus.h:397-415"},
            {"level": "mid", "title": "03 响应解析前先 WriteDeviceParamsToHoldingRegisters", "desc": "03 处理先把本地 g_deviceParams 写入 HoldingRegisterArray，再覆盖响应区间，若响应只是局部参数，未读区仍是本地旧镜像。", "suggest": "明确 HoldingRegisterArray 的主数据源，避免局部读时混入旧值；必要时增加脏区标记。", "ref": "cpu2_communicate.c:379-412"},
        ],
        "sources": [
            {"title": "PollingInputData", "desc": "CPU2 上电和运行轮询状态机。", "refs": ["Communication/internal/main_board_modbus/cpu2_communicate.c:156-254"]},
            {"title": "CPU2_CombinatePackage_Send", "desc": "内部 Modbus 主站组帧、发送和等待入口。", "refs": ["Communication/internal/main_board_modbus/cpu2_communicate.c:300-353"]},
            {"title": "HostCommuProcess", "desc": "CPU2 响应帧 CRC/地址/功能码分发。", "refs": ["Communication/internal/main_board_modbus/cpu2_communicate.c:40-83"]},
            {"title": "read_measurement_result_from_InputRegisters", "desc": "输入寄存器到 g_measurement 的字段映射。", "refs": ["Communication/internal/main_board_modbus/dataanalysis_modbus.c:480-650"]},
        ],
    },
]


def append_remaining_pages() -> None:
    PAGES.extend([
        external_com_page(),
        dsm_page(),
        wartsila_si7000_page(),
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
        "hero": "梳理 COM1/COM2/COM3 从 DMA 空闲收帧、按端口协议分发，到 DMA 发送、单帧 pending 队列和错误恢复的完整流程。",
        "entry": "USART6/USART2/USART3 空闲中断置 comX_rx_ready，App_MainLoop 读取端口配置并调用对应协议处理器。",
        "summary": "三路外部 COM 共用一套分发和发送逻辑，端口协议由 CPU3 本地参数决定，可选择 DSM、Wartsila、SI7000 或占位 LTD。",
        "overview_text": "外部 COM 是 CPU3 对上位系统的主要入口。CPU3 先按端口配置选择协议，再决定是否通过 DMA 回包。",
        "commands": ["COM_PROTO_DSM", "COM_PROTO_WARTSILA", "COM_PROTO_SI7000", "COM_PROTO_LTD"],
        "source_files": ["Application/app_main.c", "Application/system_param/cpu3_comm_display_params.c", "Core/Src/stm32f4xx_it.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "COM1/2/3 收到完整 RTU 帧"},
            {"id": "b", "kind": "process", "x": 350, "y": 150, "w": 420, "h": 82, "label": "先检查挂起重配安全点\n再读取端口协议配置"},
            {"id": "c", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "协议处理器是否存在？"},
            {"id": "d", "kind": "error", "x": 790, "y": 300, "w": 250, "h": 82, "label": "无协议/越界：不回包并恢复接收"},
            {"id": "e", "kind": "process", "x": 350, "y": 450, "w": 420, "h": 82, "label": "调用 DSM/Wartsila/SI7000 生成响应帧"},
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
                "caption": "COM1/COM2/COM3 的主循环处理逻辑基本一致：清 ready 标志、调用 cpu3_port_process、根据返回值和 tx_len 决定恢复接收或发送。",
                "height": 1040,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "外部端口收到完整帧？"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 130, "w": 420, "h": 82, "label": "清接收标志，标记本轮已处理并取接收缓存"},
                    {"id": "p2", "kind": "process", "x": 350, "y": 260, "w": 420, "h": 82, "label": "读取当前端口协议配置"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 400, "w": 310, "h": 108, "label": "协议号越界或 handler 为空？"},
                    {"id": "err", "kind": "error", "x": 790, "y": 415, "w": 250, "h": 82, "label": "处理失败：打印结果并恢复接收 DMA"},
                    {"id": "p3", "kind": "process", "x": 350, "y": 560, "w": 420, "h": 82, "label": "执行协议处理：DSM / Wartsila / SI7000 / LTD占位"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 700, "w": 310, "h": 108, "label": "是否生成响应帧？"},
                    {"id": "p4", "kind": "process", "x": 90, "y": 860, "w": 300, "h": 82, "label": "无响应：切回接收模式并重启 DMA"},
                    {"id": "p5", "kind": "process", "x": 730, "y": 860, "w": 300, "h": 82, "label": "有响应：立即发送或进入待发队列"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 560 212 L 560 260"},
                    {"d": "M 560 342 L 560 400"},
                    {"d": "M 715 454 L 790 456", "label": "是", "red": True},
                    {"d": "M 560 508 L 560 560", "label": "否"},
                    {"d": "M 560 642 L 560 700"},
                    {"d": "M 405 754 C 300 800 250 830 240 860", "label": "否"},
                    {"d": "M 715 754 C 820 800 870 830 880 860", "label": "是"},
                ],
                "evidence": [
                    {"title": "分发入口", "text": f"{code('Application/app_main.c:342-360')} cpu3_port_process 根据端口 protocol 索引 g_handlers。"},
                    {"title": "COM 主循环", "text": f"{code('Application/app_main.c:398-545')} COM1/2/3 均按 ret 和 send_len 分支处理。"},
                    {"title": "协议表", "text": f"{code('Application/app_main.c:318-323')} g_handlers 将 DSM、Wartsila、SI7000、LTD 占位绑定到处理函数。"},
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
                    {"title": "重配动作", "text": f"{code('Application/system_param/cpu3_comm_display_params.c:523-540')} Cpu3_ReinitAllUarts 根据本机通信参数重配三路外部 COM。"},
                ],
            },
            {
                "title": "DMA 发送、pending 队列与回接收",
                "caption": "发送通道忙时只缓存一帧 pending，新帧覆盖旧帧并计数；Tx 完成回调如果有 pending 就续发，否则延时后恢复接收 DMA。",
                "height": 1080,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "外部协议是否生成响应帧？"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 140, "w": 310, "h": 108, "label": "当前发送通道是否空闲？"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 310, "w": 300, "h": 90, "label": "空闲：置 busy，停 RX DMA，切发送模式，启动 TX DMA"},
                    {"id": "p2", "kind": "state", "x": 730, "y": 310, "w": 300, "h": 90, "label": "忙：复制到 pending_buf，若已有 pending 则覆盖并计数"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 470, "w": 310, "h": 108, "label": "TX DMA 启动或续发成功？"},
                    {"id": "err", "kind": "error", "x": 790, "y": 485, "w": 250, "h": 82, "label": "失败：释放 busy，恢复接收 DMA"},
                    {"id": "p3", "kind": "process", "x": 350, "y": 640, "w": 420, "h": 82, "label": "HAL_UART_TxCpltCallback 进入发送完成处理"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 780, "w": 310, "h": 108, "label": "是否还有 pending_len？"},
                    {"id": "p4", "kind": "process", "x": 90, "y": 935, "w": 300, "h": 82, "label": "有待发帧：继续发送，不切回接收"},
                    {"id": "p5", "kind": "process", "x": 730, "y": 935, "w": 300, "h": 82, "label": "无待发帧：短延时后清忙标志并恢复接收"},
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
                ],
                "evidence": [
                    {"title": "发送或排队", "text": f"{code('Application/app_main.c:226-279')} uart_try_send_or_queue 实现空闲发送、忙时单帧 pending 和覆盖计数。"},
                    {"title": "发送完成", "text": f"{code('Application/app_main.c:559-660')} TxCplt 回调按端口续发 pending 或恢复接收。"},
                    {"title": "错误恢复", "text": f"{code('Application/app_main.c:662-700')} ErrorCallback 对三路外部 COM 和 UART5 分别恢复。"},
                ],
            },
        ],
        "issues": [
            {"level": "mid", "title": "pending 队列只有一帧且覆盖旧帧", "desc": "外部请求密集时，新响应会覆盖旧 pending，虽然有覆盖计数，但上位机只会丢响应。", "suggest": "将单帧 pending 改成环形队列，或在忙时直接返回设备忙异常帧。", "ref": "Application/app_main.c:226-279"},
            {"level": "mid", "title": "协议号依赖枚举连续作为数组下标", "desc": "g_handlers 用 COM_PROTO_* 作为数组下标，若枚举不连续或后续新增值未同步表项，会进入不回包路径。", "suggest": "改成 switch 分发或增加编译期断言/默认错误响应。", "ref": "Application/app_main.c:318-360"},
            {"level": "low", "title": "UART_TX_POST_DELAY_LOOP 是固定空转延时", "desc": "发送完成后用固定 NOP 循环等待 RS485 电平方向恢复，和主频/优化级别相关。", "suggest": "优先使用 TC 标志或定时器，固定循环只作为最后兜底。", "ref": "Application/app_main.c:18-31"},
            {"level": "low", "title": "LTD 协议当前是不回包占位", "desc": "COM_PROTO_LTD 绑定 proto_no_reply，如果现场配置为 LTD，上位机会表现为无响应。", "suggest": "在菜单中标注未实现，或配置层禁止选择 LTD。", "ref": "Application/app_main.c:298-323"},
            {"level": "low", "title": "串口重配可能被持续外部流量推迟", "desc": "重配只在三路接收 ready、发送 busy 和 pending 都为空时执行；如果外部请求持续不断，g_cpu3_uart_reinit_pending 会一直保留。", "suggest": "增加重配延期计数或维护窗口，在现场能看到串口参数已保存但尚未生效的原因。", "ref": "Application/app_main.c:182-205"},
        ],
        "sources": [
            {"title": "cpu3_port_process", "desc": "按端口配置选择协议处理器。", "refs": ["Application/app_main.c:342-360"]},
            {"title": "uart_try_send_or_queue", "desc": "三路外部 COM 的发送和单帧 pending 管理。", "refs": ["Application/app_main.c:226-279"]},
            {"title": "HAL_UART_TxCpltCallback", "desc": "发送完成后续发 pending 或恢复接收。", "refs": ["Application/app_main.c:559-660"]},
            {"title": "Cpu3_ReinitAllUarts", "desc": "按本地参数重配三路外部 COM。", "refs": ["Application/system_param/cpu3_comm_display_params.c:523-540"]},
        ],
    }


def dsm_page() -> dict:
    return {
        "id": "cpu3-04",
        "file": "04_DSM协议与命令映射.html",
        "title": "CPU3 DSM 协议与命令映射程序流程",
        "short": "DSM 协议与命令映射",
        "hero": "梳理 DSM 外部 Modbus 从地址/CRC 校验、01/03/04/05/10 分支，到线圈命令映射、保持寄存器写入和 CPU2 指令/参数下发。",
        "entry": "外部 COM 选择 COM_PROTO_DSM 后调用 DSM_CommunicationProcess。",
        "summary": "DSM 作为兼容协议层，读请求从 CPU3 缓存返回，写线圈或写保持寄存器时按映射下发 CPU2。",
        "overview_text": "DSM 协议页面重点看三条链：读输入/保持寄存器、写线圈转命令、写多个保持寄存器转参数。",
        "commands": ["Response01", "Response03", "Response04", "Response05", "Response16"],
        "source_files": ["Communication/external/DSM_modbus/DSM_communication.c", "Communication/external/DSM_modbus/DSM_SlaveModbus_modbus2.c", "Communication/external/DSM_modbus/DSM_DataAnalysis_modbus2.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "DSM RTU 帧进入"},
            {"id": "b", "kind": "decision", "x": 405, "y": 145, "w": 310, "h": 108, "label": "长度、地址、CRC、功能码有效？"},
            {"id": "e", "kind": "error", "x": 790, "y": 160, "w": 250, "h": 82, "label": "无效：不回包或异常响应"},
            {"id": "c", "kind": "process", "x": 90, "y": 320, "w": 300, "h": 90, "label": "03/04：刷新系统参数或输入寄存器后组读响应"},
            {"id": "d", "kind": "process", "x": 410, "y": 320, "w": 300, "h": 90, "label": "05：线圈地址映射为 CPU2 指令或占位动作"},
            {"id": "f", "kind": "process", "x": 730, "y": 320, "w": 300, "h": 90, "label": "10：写保持寄存器并解析参数下发 CPU2"},
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
                "caption": "DSM_CommunicationProcess 先做长度、地址、CRC、功能码校验，再按功能码进入 01/03/04/05/10。",
                "height": 980,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "收到 DSM 请求帧"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "长度大于 3？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 145, "w": 250, "h": 82, "label": "长度过短：返回 -1 不回包"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "地址匹配本机或广播？"},
                    {"id": "e2", "kind": "error", "x": 790, "y": 300, "w": 250, "h": 82, "label": "地址不匹配：返回 -1"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 440, "w": 310, "h": 108, "label": "CRC 和功能码有效？"},
                    {"id": "e3", "kind": "error", "x": 790, "y": 455, "w": 250, "h": 82, "label": "CRC 错误不回包；功能码错误组 0x01 异常"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 620, "w": 250, "h": 86, "label": "01 读线圈"},
                    {"id": "p2", "kind": "process", "x": 360, "y": 620, "w": 250, "h": 86, "label": "03/04 读保持/输入"},
                    {"id": "p3", "kind": "process", "x": 630, "y": 620, "w": 250, "h": 86, "label": "05 写线圈转命令"},
                    {"id": "p4", "kind": "process", "x": 900, "y": 620, "w": 180, "h": 86, "label": "10 写保持参数"},
                    {"id": "end", "kind": "end", "x": 430, "y": 820, "w": 260, "h": 62, "label": "返回 tx_len 给 COM 层"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 715 184 L 790 186", "label": "否", "red": True},
                    {"d": "M 560 238 L 560 285", "label": "是"},
                    {"d": "M 715 339 L 790 341", "label": "否", "red": True},
                    {"d": "M 560 393 L 560 440", "label": "是"},
                    {"d": "M 715 494 L 790 496", "label": "否", "red": True},
                    {"d": "M 560 548 C 360 570 230 590 215 620", "label": "01"},
                    {"d": "M 560 548 L 485 620", "label": "03/04"},
                    {"d": "M 560 548 L 755 620", "label": "05"},
                    {"d": "M 560 548 C 760 570 980 590 990 620", "label": "10"},
                    {"d": "M 215 706 C 300 780 450 800 560 820"},
                    {"d": "M 485 706 C 510 760 535 790 560 820"},
                    {"d": "M 755 706 C 700 770 630 800 560 820"},
                    {"d": "M 990 706 C 850 790 700 810 560 820"},
                ],
                "evidence": [
                    {"title": "顶层分发", "text": f"{code('DSM_communication.c:39-97')} 先校验长度/地址/CRC/功能码，再分发到 Response01/03/04/05/16。"},
                    {"title": "读前刷新", "text": f"{code('DSM_communication.c:70-79')} 03 前 SystemParameterSet，04 前 Input_Write。"},
                    {"title": "异常帧", "text": f"{code('DSM_SlaveModbus_modbus2.c:1069-1085')} ResponseException 统一组异常响应。"},
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
                "caption": "Response16 不是简单写数组：它先校验数量和 byte count，再按地址段判定可写范围。合法写入后，第 6 段当前只作为协议占位回成功，其它地址段会解析为设备参数并尝试同步 CPU2。",
                "height": 1290,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "DSM 0x10 写多个保持寄存器"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "数量 1..123 且 byteCount = 数量*2？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 145, "w": 250, "h": 82, "label": "非法数量或字节数：返回数据异常"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 285, "w": 420, "h": 82, "label": "计算起止地址，按 0x000/0x100/0x180/0x200/0x280/0x300 地址段校验"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 420, "w": 310, "h": 108, "label": "地址段和结束地址可写？"},
                    {"id": "e2", "kind": "error", "x": 790, "y": 435, "w": 250, "h": 82, "label": "非法地址：返回地址异常"},
                    {"id": "p2", "kind": "process", "x": 350, "y": 575, "w": 420, "h": 82, "label": "把请求数据复制到 TempBuffer 并写入 DSM 保持寄存器池"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 710, "w": 310, "h": 108, "label": "写寄存器池成功？"},
                    {"id": "e3", "kind": "error", "x": 790, "y": 725, "w": 250, "h": 82, "label": "写入失败：返回数据异常"},
                    {"id": "d4", "kind": "decision", "x": 405, "y": 865, "w": 310, "h": 108, "label": "是否第 6 段零段占位？"},
                    {"id": "p3", "kind": "state", "x": 80, "y": 1030, "w": 300, "h": 90, "label": "第 6 段：只回写成功，不落参数不下发 CPU2"},
                    {"id": "p4", "kind": "process", "x": 410, "y": 1030, "w": 300, "h": 90, "label": "其它段：解析旧寄存器到 g_deviceParams 并下发差异参数"},
                    {"id": "e4", "kind": "error", "x": 740, "y": 1030, "w": 300, "h": 90, "label": "参数越界返回数据异常；设备忙/写失败返回设备忙"},
                    {"id": "end", "kind": "end", "x": 430, "y": 1200, "w": 260, "h": 62, "label": "正常时回显起始地址和寄存器数量"},
                ],
                "edges": [
                    {"d": "M 560 97 L 560 130"},
                    {"d": "M 715 184 L 790 186", "label": "否", "red": True},
                    {"d": "M 560 238 L 560 285", "label": "是"},
                    {"d": "M 560 367 L 560 420"},
                    {"d": "M 715 474 L 790 476", "label": "否", "red": True},
                    {"d": "M 560 528 L 560 575", "label": "是"},
                    {"d": "M 560 657 L 560 710"},
                    {"d": "M 715 764 L 790 766", "label": "否", "red": True},
                    {"d": "M 560 818 L 560 865", "label": "是"},
                    {"d": "M 405 919 C 300 960 230 990 230 1030", "label": "是"},
                    {"d": "M 560 973 L 560 1030", "label": "否"},
                    {"d": "M 710 1075 L 740 1075", "label": "异常", "red": True},
                    {"d": "M 230 1120 C 300 1170 410 1185 560 1200"},
                    {"d": "M 560 1120 L 560 1200"},
                    {"d": "M 890 1120 C 780 1180 680 1200 560 1200", "red": True},
                ],
                "evidence": [
                    {"title": "请求合法性", "text": f"{code('DSM_SlaveModbus_modbus2.c:917-943')} Response16 校验寄存器数量和 byteCount，不满足直接返回 EXCEPTIONCODE_ERRORDATA。"},
                    {"title": "地址段边界", "text": f"{code('DSM_SlaveModbus_modbus2.c:945-985')} 按 0x000/0x100/0x180/0x200/0x280/0x300 分段限制写入范围。"},
                    {"title": "参数解析与异常", "text": f"{code('DSM_SlaveModbus_modbus2.c:1001-1058')} 写池失败、参数错误、设备忙和正常回显分别走不同响应。"},
                    {"title": "旧寄存器转设备参数", "text": f"{code('DSM_DataAnalysis_modbus2.c:170')} UpdateDeviceParamsFromLegacyRegs 把 DSM 旧寄存器映射到 g_deviceParams 并同步 CPU2。"},
                ],
            },
        ],
        "issues": [
            {"level": "high", "title": "Response05 在外部协议处理路径中同步下发 CPU2", "desc": "线圈命令会直接调用 CPU2_CombinatePackage_Send，外部 COM 响应等待 CPU2 同步写完成，可能导致上位机响应时间不可控。", "suggest": "将 CPU2 指令下发改为主循环命令队列，DSM 先回接收确认，再异步下发 CPU2。", "ref": "DSM_SlaveModbus_modbus2.c:834-914"},
            {"level": "high", "title": "Response16 也可能在外部协议路径同步下发 CPU2 参数", "desc": "写保持寄存器成功后会进入 UpdateDeviceParamsFromLegacyRegs，解析参数并尝试同步 CPU2；外部 COM 响应时间会受到 CPU2 内部通信状态影响。", "suggest": "把参数同步拆成异步队列或至少为每次 0x10 写入记录 CPU2 下发耗时和失败原因。", "ref": "DSM_SlaveModbus_modbus2.c:1001-1058; DSM_DataAnalysis_modbus2.c:170"},
            {"level": "mid", "title": "广播地址 0 会进入处理并可能回包", "desc": "DSM_CommunicationProcess 接受 rcvbuff[0] == 0，但后续 Response 函数统一用 SlaveAddress 组帧，可能违反 Modbus 广播不响应习惯。", "suggest": "明确 DSM 广播口径；如按标准广播，应执行写动作但 tx_len=0。", "ref": "DSM_communication.c:55-63"},
            {"level": "mid", "title": "部分无效命令是业务口径而非地址不存在", "desc": "例如 SET_ZEROCIRCLE/SET_ZEROANGLE 映射为 INVALID_CMD，地址存在但返回数据异常，现场联调需要文档化。", "suggest": "在协议说明表中标记“占位成功/无效命令/下发 CPU2”的三类动作。", "ref": "DSM_SlaveModbus_modbus2.c:750-807"},
            {"level": "mid", "title": "第 6 段写保持寄存器当前只做协议占位", "desc": "0x300 段通过 IsHoldingRegisterZeroSegment 判定后回正常响应，但注释明确不落参数、不下发 CPU2，容易被上位机误认为参数已生效。", "suggest": "协议表中单独标记 0x300 段为占位成功，并在联调时验证上位机是否依赖这些参数。", "ref": "DSM_SlaveModbus_modbus2.c:980-1018"},
            {"level": "low", "title": "03/04 读前全量刷新寄存器", "desc": "03 读前 SystemParameterSet、04 读前 Input_Write 全量写寄存器数组，读频繁时 CPU3 主循环负担较大。", "suggest": "可按脏标志或固定节拍刷新缓存，读请求只做拷贝。", "ref": "DSM_communication.c:70-79"},
        ],
        "sources": [
            {"title": "DSM_CommunicationProcess", "desc": "DSM 顶层校验和功能码分发。", "refs": ["Communication/external/DSM_modbus/DSM_communication.c:39-97"]},
            {"title": "Response05", "desc": "线圈命令映射到 CPU2 CMD。", "refs": ["Communication/external/DSM_modbus/DSM_SlaveModbus_modbus2.c:834-914"]},
            {"title": "Response16", "desc": "写保持寄存器并触发参数解析/下发。", "refs": ["Communication/external/DSM_modbus/DSM_SlaveModbus_modbus2.c:917-1066"]},
            {"title": "UpdateDeviceParamsFromLegacyRegs", "desc": "DSM 旧寄存器写入后更新 CPU3/CPU2 参数。", "refs": ["Communication/external/DSM_modbus/DSM_DataAnalysis_modbus2.c:170"]},
        ],
    }


def wartsila_si7000_page() -> dict:
    return {
        "id": "cpu3-05",
        "file": "05_Wartsila与SI7000协议适配.html",
        "title": "CPU3 Wartsila 与 SI7000 协议适配程序流程",
        "short": "Wartsila 与 SI7000 适配",
        "hero": "梳理 Wartsila 寄存器池、写保持寄存器下发 CPU2、SI7000 快照同步、读写线圈/寄存器和异常响应。",
        "entry": "外部 COM 选择 COM_PROTO_WARTSILA 或 COM_PROTO_SI7000 后进入对应协议处理器。",
        "summary": "Wartsila 主要通过保持寄存器池映射参数和结果，SI7000 通过快照数组对外提供线圈、离散输入、保持寄存器和输入寄存器。",
        "overview_text": "这两个协议都是 CPU3 的对外适配层：Wartsila 写入会向 CPU2 下发参数，SI7000 读写先刷新本地快照。",
        "commands": ["modbus_rtu_process", "si7000_modbus_process"],
        "source_files": ["Communication/external/wartsila_modbus/wartsila_modbus_communication.c", "Communication/external/wartsila_modbus/wartsila_modbus_data_analysis.c", "Communication/external/si7000_modbus/si7000_modbus_slave.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "外部协议帧进入"},
            {"id": "b", "kind": "decision", "x": 405, "y": 145, "w": 310, "h": 108, "label": "端口协议为 Wartsila 还是 SI7000？"},
            {"id": "c", "kind": "process", "x": 90, "y": 320, "w": 300, "h": 90, "label": "Wartsila：地址/CRC 校验后读写 g_holding_regs"},
            {"id": "d", "kind": "process", "x": 730, "y": 320, "w": 300, "h": 90, "label": "SI7000：地址/CRC 通过后刷新快照数组"},
            {"id": "e", "kind": "decision", "x": 90, "y": 485, "w": 300, "h": 108, "label": "Wartsila 是否写保持寄存器？"},
            {"id": "f", "kind": "process", "x": 90, "y": 655, "w": 300, "h": 90, "label": "解析保持寄存器到参数，指令或密度参数下发 CPU2"},
            {"id": "g", "kind": "process", "x": 730, "y": 500, "w": 300, "h": 90, "label": "SI7000 按功能码读写线圈/寄存器并生成异常或正常响应"},
            {"id": "h", "kind": "end", "x": 430, "y": 815, "w": 260, "h": 62, "label": "返回响应帧或错误码给 COM 层"},
        ],
        "overview_edges": [
            {"d": "M 560 102 L 560 145"},
            {"d": "M 405 199 C 260 240 240 280 240 320", "label": "Wartsila"},
            {"d": "M 715 199 C 860 240 880 280 880 320", "label": "SI7000"},
            {"d": "M 240 410 L 240 485"},
            {"d": "M 240 593 L 240 655", "label": "是"},
            {"d": "M 880 410 L 880 500"},
            {"d": "M 240 745 C 320 800 460 810 560 815"},
            {"d": "M 880 590 C 800 760 660 805 560 815"},
        ],
        "flows": [
            {
                "title": "Wartsila 读写保持寄存器",
                "caption": "Wartsila 只支持 0x03 和 0x10。读前把设备参数/测量值刷到 g_holding_regs，写后解析寄存器并下发 CPU2。",
                "height": 1080,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "Wartsila RTU 帧进入"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "地址和 CRC 有效？"},
                    {"id": "err", "kind": "error", "x": 790, "y": 145, "w": 250, "h": 82, "label": "地址不匹配或 CRC 错误：不生成响应"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "功能码是 0x03 还是 0x10？"},
                    {"id": "p1", "kind": "process", "x": 80, "y": 455, "w": 300, "h": 90, "label": "0x03：刷新参数寄存器池后返回数据"},
                    {"id": "p2", "kind": "process", "x": 730, "y": 455, "w": 300, "h": 90, "label": "0x10：校验数量、字节数和地址后写入寄存器池"},
                    {"id": "p3", "kind": "process", "x": 730, "y": 620, "w": 300, "h": 90, "label": "写保持寄存器后解析命令或参数"},
                    {"id": "d3", "kind": "decision", "x": 730, "y": 770, "w": 300, "h": 108, "label": "写入范围包含 0x0006 指令？"},
                    {"id": "p4", "kind": "process", "x": 420, "y": 925, "w": 300, "h": 82, "label": "是：下发命令到 CPU2 后清空命令缓存"},
                    {"id": "p5", "kind": "process", "x": 780, "y": 925, "w": 300, "h": 82, "label": "否：转发 Wartsila 密度区间参数到 CPU2"},
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
                    {"d": "M 720 966 L 780 966"},
                    {"d": "M 420 966 L 340 956"},
                ],
                "evidence": [
                    {"title": "Wartsila 顶层", "text": f"{code('wartsila_modbus_communication.c:151-207')} modbus_rtu_process 校验地址/CRC，处理 0x03/0x10。"},
                    {"title": "写后回调", "text": f"{code('wartsila_modbus_communication.c:211-247')} modbus_on_holding_written 将写入解析为命令或参数下发。"},
                    {"title": "参数转发", "text": f"{code('wartsila_modbus_communication.c:249-272')} ForwardParamsToLowerDevice 只转发瓦锡兰密度区间相关参数。"},
                ],
            },
            {
                "title": "SI7000 快照同步与功能码处理",
                "caption": "SI7000 在地址和 CRC 通过后才刷新 CPU3 快照，读写函数码都会生成正常或异常响应；分发层只要有响应帧就视为成功。",
                "height": 1040,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "SI7000 RTU 帧进入"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 130, "w": 310, "h": 108, "label": "接收/发送缓存有效且长度足够？"},
                    {"id": "e1", "kind": "error", "x": 790, "y": 145, "w": 250, "h": 82, "label": "参数或长度错误：返回 BADLEN"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 285, "w": 310, "h": 108, "label": "地址匹配且 CRC 正确？"},
                    {"id": "e2", "kind": "error", "x": 790, "y": 300, "w": 250, "h": 82, "label": "地址不匹配或 CRC 错误：不回包"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 455, "w": 420, "h": 86, "label": "刷新线圈、保持寄存器、离散输入、输入寄存器快照"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 600, "w": 310, "h": 108, "label": "功能码属于 01/02/03/04/05/06？"},
                    {"id": "p2", "kind": "process", "x": 90, "y": 760, "w": 300, "h": 90, "label": "读类：从快照数组返回位或寄存器"},
                    {"id": "p3", "kind": "process", "x": 730, "y": 760, "w": 300, "h": 90, "label": "写类：更新单线圈或单寄存器并回显"},
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
                    {"title": "快照刷新", "text": f"{code('si7000_modbus_slave.c:997-1012')} si7000_modbus_sync_from_system 汇总刷新四类数组。"},
                    {"title": "处理入口", "text": f"{code('si7000_modbus_slave.c:1016-1080')} 地址/CRC 通过后才同步系统状态并按功能码处理。"},
                    {"title": "分发表适配", "text": f"{code('si7000_modbus_slave.c:1083-1096')} 只要已生成响应帧，COM 分发层就按成功处理并发送。"},
                ],
            },
        ],
        "issues": [
            {"level": "high", "title": "Wartsila 0x10 字节数校验疑似错误", "desc": "handle_0x10 中检查 bytes != qty，但后续又用 pdu_len == 6 + 2*bytes，并按 2*i 取数据；标准 Modbus 的 byteCount 应为 qty*2。", "suggest": "确认现场 Wartsila 协议是否非标；如果不是非标，应改为 bytes == qty*2 并补充测试帧。", "ref": "wartsila_modbus_communication.c:102-113"},
            {"level": "mid", "title": "Wartsila 参数转发范围很窄", "desc": "非 0x0006 写入只转发 4 个密度区间参数，其他写入会解析到 g_deviceParams 但不一定同步 CPU2。", "suggest": "列出 Wartsila 可写寄存器清单，逐项确认是否需要下发 CPU2。", "ref": "wartsila_modbus_communication.c:249-272"},
            {"level": "mid", "title": "SI7000 写操作是否同步 CPU2 需要单独核对", "desc": "SI7000 写单线圈/单寄存器在模块内处理，页面证据显示主入口负责快照和响应，写后对 CPU2 的影响需要按具体 handler 继续核对。", "suggest": "把 SI7000 05/06 的业务动作整理成协议表，区分只改本地快照、下发 CPU2、占位回显三类。", "ref": "si7000_modbus_slave.c:1016-1080"},
            {"level": "low", "title": "Wartsila 特殊地址 0x0672 返回固定版本字符串", "desc": "0x0672 读取固定返回一段版本文本，不走寄存器池，后续版本变化时容易遗漏同步。", "suggest": "把固定字符串集中成宏或从版本定义生成。", "ref": "wartsila_modbus_communication.c:55-76"},
        ],
        "sources": [
            {"title": "modbus_rtu_process", "desc": "Wartsila 顶层 RTU 处理。", "refs": ["Communication/external/wartsila_modbus/wartsila_modbus_communication.c:151-207"]},
            {"title": "modbus_on_holding_written", "desc": "Wartsila 写后解析和 CPU2 下发。", "refs": ["Communication/external/wartsila_modbus/wartsila_modbus_communication.c:211-247"]},
            {"title": "si7000_modbus_process", "desc": "SI7000 地址/CRC/快照/功能码处理。", "refs": ["Communication/external/si7000_modbus/si7000_modbus_slave.c:1016-1080"]},
            {"title": "DeviceParams_StoreToRegisters", "desc": "把 CPU3 参数/测量结果整理到 Wartsila 寄存器池。", "refs": ["Communication/external/wartsila_modbus/wartsila_modbus_data_analysis.c:198"]},
        ],
    }


def display_page() -> dict:
    return {
        "id": "cpu3-06",
        "file": "06_显示刷新与按键事件.html",
        "title": "CPU3 显示刷新与按键事件程序流程",
        "short": "显示刷新与按键事件",
        "hero": "梳理 OLED 初始化、显示任务、状态页 2 秒数据采样、协议兼容提示、AD5421 故障显示、长按进入菜单、按键队列、长按释放保护和 SPI 恢复。",
        "entry": "App_MainLoop 每轮调用 Display_Task；按键 EXTI 中断只入队或启动长按计时，不直接绘制。",
        "summary": "显示模块在主循环中消费按键和刷新屏幕，通过状态页采样节流、协议版本提示、AD5421 故障文案、SPI 错误计数和超时计数保护 OLED 绘制。",
        "overview_text": "显示流程的关键是把中断输入、状态数据采样和 OLED 绘制解耦：中断只记录事件，状态页按变化/2 秒节拍采样，Display_Task 在主循环里统一处理。",
        "commands": ["LONG_PRESS_KEY_SURE", "LONG_PRESS_KEY_BACK", "USE_KEY_UP", "USE_KEY_DOWN", "USE_KEY_SURE", "USE_KEY_BACK"],
        "source_files": ["Application/display/display.c", "Application/display/exit.c", "Application/display/display_tankopera.c", "Application/display/hgs.c"],
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
                "title": "状态页数据采样、协议兼容和 AD5421 故障显示",
                "caption": "状态页不是每次绘制都重新采样。当前程序在首次、强制刷新、关键状态变化或 2 秒采样节拍到期时更新快照，否则只恢复高亮区域。",
                "height": 1180,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "进入状态页绘制"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 135, "w": 310, "h": 108, "label": "是否首次、全屏恢复或快照无效？"},
                    {"id": "p1", "kind": "process", "x": 90, "y": 300, "w": 300, "h": 90, "label": "立即采样 CPU2 缓存，生成状态页快照"},
                    {"id": "d2", "kind": "decision", "x": 730, "y": 300, "w": 300, "h": 108, "label": "状态/错误/协议/语言是否变化或已到 2 秒？"},
                    {"id": "p2", "kind": "process", "x": 730, "y": 475, "w": 300, "h": 90, "label": "需要采样：刷新液位、水位、RSSI、协议状态和故障原因"},
                    {"id": "p3", "kind": "state", "x": 350, "y": 635, "w": 420, "h": 86, "label": "不采样：只恢复状态高亮，不重新读取 CPU2 数据"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 780, "w": 310, "h": 108, "label": "CPU2/CPU3 协议版本严格一致？"},
                    {"id": "e1", "kind": "error", "x": 90, "y": 945, "w": 300, "h": 90, "label": "不一致：显示协议不兼容，提示成对升级"},
                    {"id": "d4", "kind": "decision", "x": 730, "y": 945, "w": 300, "h": 108, "label": "错误码是否为 AD5421/AO 输出相关？"},
                    {"id": "p4", "kind": "process", "x": 730, "y": 1110, "w": 300, "h": 90, "label": "按 AD5421 故障文案显示初始化、写电流、FAULT、READFAULT 或回读错误"},
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
                    {"title": "协议与 AD5421 文案", "text": f"{code('Application/display/display.c:796-805,2968-3120')} 状态页严格判断协议版本，并把 AD5421/AO 错误码翻译成现场可读故障原因。"},
                ],
            },
        ],
        "issues": [
            {"level": "mid", "title": "按键队列满时丢弃最新按键", "desc": "pending_key_queue 满时只增加 overflow_count，用户快速操作时最新按键会丢失但界面无提示。", "suggest": "在调试页显示 overflow_count，或改为丢弃最旧按键并给界面节流。", "ref": "Application/display/exit.c:181-199"},
            {"level": "mid", "title": "前景绘制和协议轮询共用主循环", "desc": "菜单绘制、确认页和状态页刷新都在 App_MainLoop 中执行，刷新耗时过长会影响外部 COM 响应和 CPU2 轮询。", "suggest": "保留 display_last_refresh_ms 统计，并对状态页分帧/局部刷新做节流。", "ref": "Application/display/display.c:2057-2073"},
            {"level": "low", "title": "长按释放保护会临时屏蔽按键", "desc": "长按动作后 pending_key_tail=head，并等待释放保护时间，用户可能感觉按键短暂失效。", "suggest": "在交互说明或界面状态上体现长按确认后的释放等待。", "ref": "Application/display/exit.c:254-314"},
            {"level": "low", "title": "OLED 周期恢复固定 60s", "desc": "无论现场 SPI 状态如何，60s 到期会触发恢复策略，可能造成偶发刷新停顿。", "suggest": "将周期恢复做成参数或只在错误计数变化后触发。", "ref": "Application/display/display.c:13-16,1912-2005"},
            {"level": "mid", "title": "状态页数据最多 2 秒才重新采样一次", "desc": "当前版本为降低闪烁和绘制负担，非强制刷新时按 2 秒采样状态数据；AO 电流、RSSI 或瞬时故障变化可能不会立即显示。", "suggest": "把 AO/故障字段纳入强制采样触发，或在状态页显示最近采样时间/更新计数。", "ref": "Application/display/display.c:16,2633-3120"},
            {"level": "mid", "title": "协议不兼容和 AO 故障依赖 CPU2 运行态同步", "desc": "协议版本严格一致可以避免尾段错位；当前状态页的 AD5421/AO 故障原因来自 CPU2 设备错误码，AO runtime 虽已缓存但未作为独立状态字段显示。", "suggest": "升级流程要求 CPU2/CPU3 成对升级；若后续要显示 AO 目标电流/来源，需要在状态页快照中显式接入 ao_output_runtime。", "ref": "Communication/internal/main_board_modbus/dataanalysis_modbus.c:493-623"},
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
        "summary": "菜单既能发 CPU2 测量/调试命令，也能修改 CPU2 设备参数和 CPU3 本机显示/通信参数；当前版本新增 AO 输出使能，并保留读取部件参数触发 RSSI/传感器快照。",
        "overview_text": "菜单流程必须和参数/命令下发合在一起看：页面选择、输入、确认、保护确认、下发和本地保存都在一条链路中。",
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
            {"id": "i", "kind": "end", "x": 430, "y": 950, "w": 260, "h": 62, "label": "下发 CPU2、保存 CPU3 本机参数或返回菜单"},
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
                    {"title": "主菜单", "text": f"{code('display_tankopera.c:3386-3401')} mainmenu 定义五个主入口。"},
                    {"title": "按键分发", "text": f"{code('display_tankopera.c:698-759')} CanProcessKey 和 KeyProcess 使用 keymenu 表进行函数指针分发。"},
                    {"title": "菜单构建", "text": f"{code('display_tankopera.c:4073-4146')} menu_build_by_group/filter 按参数元数据动态构建参数页。"},
                ],
            },
            {
                "title": "参数/指令输入、确认和下发",
                "caption": "无参指令直接确认，带参指令和参数先输入数值；关键参数会进入保护确认，最后按本机参数或 CPU2 参数走不同保存/下发路径。",
                "height": 1120,
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
                    {"id": "p5", "kind": "process", "x": 730, "y": 965, "w": 300, "h": 90, "label": "否：写 CPU2 保持寄存器或命令寄存器，等待响应"},
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
                    {"d": "M 715 854 C 820 900 870 930 880 965", "label": "否"},
                ],
                "evidence": [
                    {"title": "数值输入", "text": f"{code('display_tankopera.c:1132-1180')} inputcmdpara 按位数、小数点、符号输入参数值。"},
                    {"title": "确认页", "text": f"{code('display_tankopera.c:2193-2258')} ifsendcmd 根据操作类型显示确认内容，并用 timesure/timeback 做二次确认。"},
                    {"title": "保护确认", "text": f"{code('display_tankopera.c:2265-2300')} operation_needs_protect_confirm 列出关键操作和通信参数。"},
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
                    {"id": "p5", "kind": "process", "x": 350, "y": 820, "w": 420, "h": 90, "label": "CPU3 通过内部 0x10 写 CPU2；后续轮询输入寄存器确认状态"},
                    {"id": "d3", "kind": "decision", "x": 405, "y": 990, "w": 310, "h": 108, "label": "CPU2 是否已发布 RSSI/AO 运行态？"},
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
                    {"title": "菜单枚举", "text": f"{code('display_tankopera.h:193')} 定义 COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE，菜单短名为 AO使能。"},
                    {"title": "AO 分组和范围", "text": f"{code('system_parameter.c:238')} AO输出使能范围为 0..1，按布尔枚举显示。"},
                    {"title": "同步到 CPU2", "text": f"{code('device_param_sync.c:248,512')} CPU3 参数镜像把 AoOutputEnable 映射到 CPU2 设备参数，并通过内部 Modbus 同步。"},
                    {"title": "读取部件参数", "text": f"{code('display_tankopera.c:1235,2796,3911')} 菜单触发 CMD_READ_PART_PARAMS 后，状态页使用后续轮询到的 RSSI/传感器快照。"},
                ],
            },
        ],
        "issues": [
            {"level": "mid", "title": "确认逻辑依赖 timesure/timeback 计数", "desc": "ifsendcmd 需要连续确认/返回操作，计数变量跨页面使用，状态未清理时可能造成误判。", "suggest": "把确认状态封装为独立结构，进入确认页时统一初始化。", "ref": "display_tankopera.c:2193-2258"},
            {"level": "mid", "title": "参数范围判断依赖枚举区间", "desc": "dtm_operaname、dtm_points 等多处用 COM_NUM_* START/STOP 区间判断，枚举新增或命名不一致会影响显示和输入。", "suggest": "以 param_meta/显式表为主，减少区间判断；新增操作时要求表驱动校验。", "ref": "display_tankopera.c:846-980"},
            {"level": "mid", "title": "本机通信参数修改后异步重配", "desc": "菜单保存 CPU3 COM 参数后需要等待主循环安全点重配，用户确认保存和实际生效之间存在时间差。", "suggest": "保存后显示“等待串口空闲后生效”，并在重配完成后给出状态反馈。", "ref": "Application/app_main.c:182-221"},
            {"level": "low", "title": "密码错误固定延时阻塞主循环", "desc": "密码错误时 HAL_Delay(500)，会暂停协议处理和 CPU2 轮询。", "suggest": "改成显示超时状态，由 Display_Task 周期返回主菜单。", "ref": "display_tankopera.c:2193-2258"},
            {"level": "mid", "title": "AO 使能复用了原 reserved26 寄存器位置", "desc": "CPU2/CPU3 当前把 AO_OUTPUT_ENABLE 放在原保留保持寄存器位置；如果现场旧版本仍把该地址当保留位，升级前后可能出现配置含义变化。", "suggest": "在升级说明和参数表里明确 reserved26 已变更为 AO 输出使能，并要求 CPU2/CPU3 成对升级。", "ref": "Application/system_param/stateformodbus.h:378-384"},
            {"level": "mid", "title": "读取部件参数和 AO 运行态缓存都依赖运行轮询刷新", "desc": "菜单下发后并不是立即得到 RSSI 或 AO 运行态，而是等待 CPU2 执行并由 CPU3 下一轮输入寄存器轮询刷新；当前 AO 运行态只是缓存，未独立显示在状态页。", "suggest": "状态页显示等待/更新时间；若后续要显示 AO 状态，需要补状态页快照和绘制字段。", "ref": "Communication/internal/main_board_modbus/cpu2_communicate.c:156-254"},
        ],
        "sources": [
            {"title": "mainmenu / measuremenu", "desc": "主菜单和测量命令菜单定义。", "refs": ["Application/display/display_tankopera.c:3886-4070"]},
            {"title": "KeyProcess", "desc": "按键分发到当前页面函数。", "refs": ["Application/display/display_tankopera.c:778-830"]},
            {"title": "ifsendcmd", "desc": "指令/参数确认页。", "refs": ["Application/display/display_tankopera.c:2193-2258"]},
            {"title": "COM_NUM_DEVICEPARAM_AO_OUTPUT_ENABLE", "desc": "AO 输出使能菜单项。", "refs": ["Application/display/display_tankopera.h:193"]},
            {"title": "DeviceParams_SyncAllToCPU2", "desc": "本地参数与 CPU2 参数差异同步。", "refs": ["Communication/internal/main_board_modbus/device_param_sync.c:512"]},
        ],
    }


def param_fram_io_page() -> dict:
    return {
        "id": "cpu3-08",
        "file": "08_本机参数FRAM时钟与外设恢复.html",
        "title": "CPU3 本机参数、FRAM、时钟与外设恢复程序流程",
        "short": "本机参数与外设恢复",
        "hero": "梳理 CPU3 本机通信/显示参数默认值、FRAM V3 到 V4 迁移、CRC 校验、串口重配、RTC 初始化、OLED SPI 恢复和 UART 错误恢复。",
        "entry": "App_Init 调用 Cpu3Clock_Init、读取 CPU3 本机 FRAM 参数、Cpu3_ReinitAllUarts；运行期菜单或错误回调触发保存和恢复。",
        "summary": "CPU3 本机参数独立于 CPU2 设备参数，存储在 FRAM 中；RTC 和 OLED/UART 恢复用于保证协议显示和现场交互稳定。",
        "overview_text": "本页是 CPU3 的本地支撑层：它不直接改变测量算法，但决定显示/通信参数是否能正确加载、生效和恢复。",
        "commands": ["CPU3_PARAM_VERSION", "CPU3_PARAM_VERSION_V3", "CPU3_CLOCK_BKP_MARKER"],
        "source_files": ["Application/system_param/cpu3_comm_display_params.c", "Application/system_param/mb85rs2m.c", "Application/system_param/cpu3_clock.c", "Application/display/hgs.c", "Application/app_main.c"],
        "overview_nodes": [
            {"id": "a", "kind": "start", "x": 430, "y": 40, "w": 260, "h": 62, "label": "CPU3 应用初始化或参数保存"},
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
                "caption": "FRAM 镜像包含 magic、version、params、crc；V3 可迁移到 V4，无效或 CRC 错误时使用默认值并写回。",
                "height": 1160,
                "nodes": [
                    {"id": "s", "kind": "start", "x": 430, "y": 35, "w": 260, "h": 62, "label": "读取 CPU3 本机 FRAM 参数"},
                    {"id": "p1", "kind": "process", "x": 350, "y": 130, "w": 420, "h": 82, "label": "从 CPU3 参数地址读取本机参数镜像"},
                    {"id": "d1", "kind": "decision", "x": 405, "y": 270, "w": 310, "h": 108, "label": "参数标识正确且为 V3 旧格式？"},
                    {"id": "p2", "kind": "process", "x": 780, "y": 285, "w": 260, "h": 90, "label": "校验 V3 CRC，迁移字段并补默认亮度"},
                    {"id": "d2", "kind": "decision", "x": 405, "y": 430, "w": 310, "h": 108, "label": "参数标识、版本和 CRC 是否为当前 V4 有效？"},
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
                    {"title": "存储结构", "text": f"{code('cpu3_comm_display_params.c:550-590')} 定义 magic/version/V3/V4 参数镜像和 CRC 字段。"},
                    {"title": "加载流程", "text": f"{code('cpu3_comm_display_params.c:712-782')} 读取 CPU3 本机 FRAM 参数 处理 V3 迁移、V4 校验和默认值回退。"},
                    {"title": "保存判重", "text": f"{code('cpu3_comm_display_params.c:682-710')} 保存前构造目标镜像，现有镜像有效且相同则跳过写入。"},
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
            {"level": "mid", "title": "FRAM 无效会静默恢复默认通信参数", "desc": "magic/version/CRC 无效时直接初始化默认并保存，现场串口协议配置可能回到默认值。", "suggest": "在显示端增加“参数恢复默认”提示或错误计数，便于现场定位 FRAM/升级问题。", "ref": "cpu3_comm_display_params.c:712-782"},
            {"level": "low", "title": "LSI 兜底时时间精度仍有限", "desc": "新板优先使用 LSE，旧板或 LSE 启动失败时会回退 LSI；LSI 长期漂移仍可能影响时间戳精度。", "suggest": "现场测试时应确认菜单显示的 RTC 状态，LSE 异常时按硬件问题排查晶振和 VBAT。", "ref": "cpu3_clock.c:251-335"},
            {"level": "mid", "title": "UART5 错误会清 wait_response", "desc": "内部 CPU2 通信错误回调直接 wait_response=false，调用方只看到等待结束，缺少明确错误状态。", "suggest": "增加 UART5 错误码/计数并让 CPU2_CombinatePackage_Send 返回错误。", "ref": "Application/app_main.c:693-700"},
            {"level": "low", "title": "FRAM 保存判重只比较 params", "desc": "magic/version/CRC 有效且 params 相同就跳过写入，若保留字段策略变化需确认是否仍满足升级需求。", "suggest": "版本升级时复核 Cpu3_Params_BuildStorage 和判重条件。", "ref": "cpu3_comm_display_params.c:682-710"},
        ],
        "sources": [
            {"title": "读取 CPU3 本机 FRAM 参数", "desc": "CPU3 本机参数加载、迁移和默认回退。", "refs": ["Application/system_param/cpu3_comm_display_params.c:712-782"]},
            {"title": "Cpu3_Params_SaveToFRAM", "desc": "CPU3 本机参数 CRC 构造、判重和写入。", "refs": ["Application/system_param/cpu3_comm_display_params.c:682-710"]},
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
    css_path.write_text(css_path.read_text(encoding="utf-8") + CPU3_STYLE_APPEND, encoding="utf-8", newline="\n")

    for page in PAGES:
        (DOC_DIR / page["file"]).write_text(page_html(page, PAGES), encoding="utf-8", newline="\n")

    (DOC_DIR / "CPU3程序流程总览.html").write_text(overview_page(PAGES), encoding="utf-8", newline="\n")
    (DOC_DIR / "README.md").write_text(readme(PAGES), encoding="utf-8", newline="\n")

    root_readme = CPU3 / "docs" / "README.md"
    text = root_readme.read_text(encoding="utf-8", errors="replace")
    row = "| `00_程序流程` | CPU3 启动主循环、CPU2 内部通信、外部协议、显示菜单、本机参数和外设恢复程序流程 HTML 文档 |\n"
    if "`00_程序流程`" not in text:
        text = text.replace("| `03_故障码` | CPU3 显示侧使用的 LTD 故障代码表、设备参数和保持寄存器辅助表 |\n", row + "| `03_故障码` | CPU3 显示侧使用的 LTD 故障代码表、设备参数和保持寄存器辅助表 |\n")
    text = re.sub(r"更新日期：\d{4}-\d{2}-\d{2}", f"更新日期：{DOC_DATE}", text)
    root_readme.write_text(text, encoding="utf-8", newline="\n")


if __name__ == "__main__":
    main()

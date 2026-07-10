#!/usr/bin/env python3
"""检查正式程序流程 HTML 的网站嵌入与 SVG 可访问性契约。"""

from __future__ import annotations

import re
import sys
import unicodedata
from collections import Counter
from dataclasses import dataclass, field
from html.parser import HTMLParser
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FLOW_ROOT = ROOT / "docs" / "00_程序流程导航"
OUTPUT_ENCODING = "utf-8"
EMBED_START = "CUBE_EMBED_START"
EMBED_END = "CUBE_EMBED_END"
ENHANCEMENT_STYLESHEET = "网站嵌入增强.css"
FLOW_WIDTHS = {"compact", "standard", "wide"}
VOID_TAGS = {
    "area",
    "base",
    "br",
    "col",
    "embed",
    "hr",
    "img",
    "input",
    "link",
    "meta",
    "param",
    "source",
    "track",
    "wbr",
}
ID_REFERENCE_ATTRS = {
    "aria-controls",
    "aria-describedby",
    "aria-labelledby",
    "headers",
}
FLOAT_PATTERN = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
VIEWBOX_PATTERN = re.compile(
    rf"^\s*({FLOAT_PATTERN})[\s,]+({FLOAT_PATTERN})[\s,]+"
    rf"({FLOAT_PATTERN})[\s,]+({FLOAT_PATTERN})\s*$"
)
URL_FRAGMENT_PATTERN = re.compile(r"url\(\s*['\"]?#([^)'\"\s]+)['\"]?\s*\)", re.IGNORECASE)
MIN_NORMALIZED_SUMMARY_LENGTH = 12
MIN_DISTINCT_SUMMARY_LENGTH = 6


@dataclass
class Element:
    """HTMLParser 事件构成的最小元素树。"""

    tag: str
    attrs: dict[str, str]
    parent: int | None
    line: int
    start_event: int
    end_event: int | None = None
    children: list[int] = field(default_factory=list)
    text_parts: list[str] = field(default_factory=list)


@dataclass
class FlowPageAudit:
    """单个流程页的检查结果。"""

    path: Path
    svg_count: int
    table_count: int
    page_identifier: str
    errors: list[str]


class FlowContractParser(HTMLParser):
    """保留父子关系、位置和嵌入标记的轻量 HTML 解析器。"""

    def __init__(self) -> None:
        super().__init__(convert_charrefs=True)
        self.elements: list[Element] = []
        self.stack: list[int] = []
        self.event = 0
        self.embed_starts: list[tuple[int, int]] = []
        self.embed_ends: list[tuple[int, int]] = []
        self.duplicate_attributes: list[tuple[int, str, str, tuple[str, ...]]] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self._add_element(tag, attrs, push=tag.lower() not in VOID_TAGS)

    def handle_startendtag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        self._add_element(tag, attrs, push=False)

    def handle_endtag(self, tag: str) -> None:
        self.event += 1
        normalized = tag.lower()
        matching_position = next(
            (
                position
                for position in range(len(self.stack) - 1, -1, -1)
                if self.elements[self.stack[position]].tag == normalized
            ),
            None,
        )
        if matching_position is None:
            return
        for index in self.stack[matching_position:]:
            self.elements[index].end_event = self.event
        del self.stack[matching_position:]

    def handle_data(self, data: str) -> None:
        self.event += 1
        if self.stack:
            self.elements[self.stack[-1]].text_parts.append(data)

    def handle_comment(self, data: str) -> None:
        self.event += 1
        marker = data.strip()
        if marker == EMBED_START:
            self.embed_starts.append((self.event, self.getpos()[0]))
        elif marker == EMBED_END:
            self.embed_ends.append((self.event, self.getpos()[0]))

    def finish(self) -> None:
        self.event += 1
        for index in self.stack:
            if self.elements[index].end_event is None:
                self.elements[index].end_event = self.event
        self.stack.clear()

    def _add_element(
        self,
        tag: str,
        attrs: list[tuple[str, str | None]],
        *,
        push: bool,
    ) -> None:
        self.event += 1
        attribute_names = [name.lower() for name, _ in attrs]
        for name, count in Counter(attribute_names).items():
            if count <= 1:
                continue
            values = tuple(
                value if value is not None else ""
                for attribute_name, value in attrs
                if attribute_name.lower() == name
            )
            self.duplicate_attributes.append((self.getpos()[0], tag.lower(), name, values))
        normalized_attrs = {
            name.lower(): value if value is not None else ""
            for name, value in attrs
        }
        parent = self.stack[-1] if self.stack else None
        node = Element(
            tag=tag.lower(),
            attrs=normalized_attrs,
            parent=parent,
            line=self.getpos()[0],
            start_event=self.event,
            end_event=self.event if not push else None,
        )
        index = len(self.elements)
        self.elements.append(node)
        if parent is not None:
            self.elements[parent].children.append(index)
        if push:
            self.stack.append(index)


def class_tokens(node: Element) -> set[str]:
    return set(node.attrs.get("class", "").split())


def node_text(parser: FlowContractParser, index: int) -> str:
    node = parser.elements[index]
    parts = list(node.text_parts)
    for child in node.children:
        parts.append(node_text(parser, child))
    return " ".join(" ".join(parts).split())


def normalize_summary_text(text: str) -> str:
    """折叠 Unicode 兼容字符、大小写、空白和标点，用于作者摘要门禁。"""

    normalized = unicodedata.normalize("NFKC", text).casefold()
    return re.sub(r"[\W_]+", "", normalized)


def normalized_summary_references(*reference_texts: str) -> set[str]:
    """归一化非空标题与图注，供重复和有效信息检查复用。"""

    return {
        normalized_reference
        for reference_text in reference_texts
        if (normalized_reference := normalize_summary_text(reference_text))
    }


def summary_without_references(summary_text: str, *reference_texts: str) -> str:
    """移除摘要中完整出现的标题或图注，保留真正补充的信息。"""

    normalized_summary = normalize_summary_text(summary_text)
    normalized_references = normalized_summary_references(*reference_texts)
    for reference_text in sorted(normalized_references, key=len, reverse=True):
        normalized_summary = normalized_summary.replace(reference_text, "")
    return normalized_summary


def ancestor_indices(parser: FlowContractParser, index: int):  # noqa: ANN201
    parent = parser.elements[index].parent
    while parent is not None:
        yield parent
        parent = parser.elements[parent].parent


def descendant_indices(parser: FlowContractParser, index: int):  # noqa: ANN201
    """按文档顺序遍历元素后代。"""

    stack = list(reversed(parser.elements[index].children))
    while stack:
        child = stack.pop()
        yield child
        stack.extend(reversed(parser.elements[child].children))


def is_between(node: Element, start_event: int, end_event: int) -> bool:
    return start_event < node.start_event < end_event


def is_valid_viewbox(value: str) -> bool:
    match = VIEWBOX_PATTERN.fullmatch(value)
    if not match:
        return False
    _, _, width, height = (float(part) for part in match.groups())
    return width > 0 and height > 0


def validate_flow_html(text: str, path: Path) -> FlowPageAudit:
    """检查单页契约，不修改输入。"""

    parser = FlowContractParser()
    errors: list[str] = []

    def add_error(message: str, node: Element | None = None) -> None:
        location = f":{node.line}" if node is not None else ""
        errors.append(f"{path.as_posix()}{location} {message}")

    try:
        parser.feed(text.removeprefix("\ufeff"))
        parser.close()
        parser.finish()
    except Exception as exc:  # HTMLParser 的异常必须转成可定位的门禁错误
        add_error(f"HTML 解析失败：{exc}")
        return FlowPageAudit(
            path=path,
            svg_count=0,
            table_count=0,
            page_identifier="",
            errors=errors,
        )

    for line, tag, attribute, values in parser.duplicate_attributes:
        distinct_values = tuple(dict.fromkeys(values))
        detail = (
            f"（值：{' | '.join(distinct_values)}）"
            if distinct_values
            else ""
        )
        errors.append(
            f"{path.as_posix()}:{line} {tag} 属性重复：{attribute}{detail}"
        )

    by_tag: dict[str, list[int]] = {}
    for index, node in enumerate(parser.elements):
        by_tag.setdefault(node.tag, []).append(index)

    html_nodes = by_tag.get("html", [])
    head_nodes = by_tag.get("head", [])
    body_nodes = by_tag.get("body", [])
    if len(html_nodes) != 1:
        add_error(f"必须且只能有一个 html 元素，当前为 {len(html_nodes)} 个")
    elif parser.elements[html_nodes[0]].attrs.get("lang", "").lower() != "zh-cn":
        add_error('html 必须声明 lang="zh-CN"', parser.elements[html_nodes[0]])

    if len(head_nodes) != 1:
        add_error(f"必须且只能有一个 head 元素，当前为 {len(head_nodes)} 个")
    if len(body_nodes) != 1:
        add_error(f"必须且只能有一个 body 元素，当前为 {len(body_nodes)} 个")

    if head_nodes:
        head = parser.elements[head_nodes[0]]
        head_children = [parser.elements[index] for index in head.children]
        title_indices = [index for index in head.children if parser.elements[index].tag == "title"]
        charset_ok = any(
            child.tag == "meta" and child.attrs.get("charset", "").lower() == "utf-8"
            for child in head_children
        )
        viewport_ok = any(
            child.tag == "meta"
            and child.attrs.get("name", "").lower() == "viewport"
            and "width=device-width" in child.attrs.get("content", "").replace(" ", "").lower()
            for child in head_children
        )
        if not charset_ok:
            add_error('head 缺少 <meta charset="utf-8">', head)
        if not viewport_ok:
            add_error("head 缺少移动端 viewport 声明", head)
        if len(title_indices) != 1 or not node_text(parser, title_indices[0]):
            add_error("head 必须包含一个非空 title", head)

        final_element = head_children[-1] if head_children else None
        final_href = final_element.attrs.get("href", "") if final_element else ""
        final_rel = set(final_element.attrs.get("rel", "").lower().split()) if final_element else set()
        if (
            final_element is None
            or final_element.tag != "link"
            or "stylesheet" not in final_rel
            or not final_href.split("?", 1)[0].replace("\\", "/").endswith(
                f"assets/{ENHANCEMENT_STYLESHEET}"
            )
        ):
            add_error("统一增强样式必须是 head 的最后一个元素", head)

    page_identifier = ""
    if body_nodes:
        body = parser.elements[body_nodes[0]]
        if "cube-flow-page" not in class_tokens(body):
            add_error("body 缺少 cube-flow-page 类", body)
        page_identifier = body.attrs.get("data-cube-flow-page", "").strip()
        if not page_identifier:
            add_error("body 缺少稳定的 data-cube-flow-page 标识", body)

    h1_nodes = by_tag.get("h1", [])
    if len(h1_nodes) != 1:
        add_error(f"独立页必须且只能有一个 h1，当前为 {len(h1_nodes)} 个")

    heading_indices = sorted(
        (
            index
            for level in range(1, 7)
            for index in by_tag.get(f"h{level}", [])
        ),
        key=lambda index: parser.elements[index].start_event,
    )
    if heading_indices and parser.elements[heading_indices[0]].tag != "h1":
        add_error("页面首个标题必须是 h1", parser.elements[heading_indices[0]])
    previous_level: int | None = None
    for heading_index in heading_indices:
        heading = parser.elements[heading_index]
        level = int(heading.tag[1])
        if not node_text(parser, heading_index):
            add_error(f"{heading.tag} 标题文字不能为空", heading)
        if previous_level is not None and level > previous_level + 1:
            add_error(
                f"标题层级跳级：h{previous_level} 后直接使用 h{level}",
                heading,
            )
        previous_level = level

    start_event = -1
    end_event = -1
    if len(parser.embed_starts) != 1 or len(parser.embed_ends) != 1:
        add_error(
            f"嵌入标记必须各有一个，当前 start={len(parser.embed_starts)}、"
            f"end={len(parser.embed_ends)}"
        )
    else:
        start_event = parser.embed_starts[0][0]
        end_event = parser.embed_ends[0][0]
        if start_event >= end_event:
            add_error("嵌入标记顺序错误")
        wraps = [
            node
            for node in parser.elements
            if "wrap" in class_tokens(node)
            and node.start_event < start_event
            and (node.end_event or parser.event) > end_event
        ]
        if not wraps:
            add_error("嵌入标记必须位于 .wrap 内容容器内")
        for node in parser.elements:
            if not is_between(node, start_event, end_event):
                continue
            forbidden_class = class_tokens(node) & {"hero", "topnav", "tool-panel"}
            if node.tag == "script" or forbidden_class:
                label = node.tag if node.tag == "script" else ", ".join(sorted(forbidden_class))
                add_error(f"独立页专用内容位于嵌入区域内：{label}", node)

    id_nodes: dict[str, list[Element]] = {}
    for node in parser.elements:
        identifier = node.attrs.get("id", "").strip()
        if identifier:
            id_nodes.setdefault(identifier, []).append(node)
    for identifier, nodes in id_nodes.items():
        if len(nodes) > 1:
            add_error(f"页面 ID 重复：{identifier}", nodes[1])
    known_ids = set(id_nodes)

    for node in parser.elements:
        for attr_name in ID_REFERENCE_ATTRS:
            for identifier in node.attrs.get(attr_name, "").split():
                if identifier not in known_ids:
                    add_error(f"{attr_name} 引用了不存在的 ID：{identifier}", node)
        label_for = node.attrs.get("for", "").strip()
        if label_for and label_for not in known_ids:
            add_error(f"for 引用了不存在的 ID：{label_for}", node)
        for attr_name in ("href", "xlink:href"):
            value = node.attrs.get(attr_name, "").strip()
            if value.startswith("#") and len(value) > 1 and value[1:] not in known_ids:
                add_error(f"{attr_name} 引用了不存在的 ID：{value[1:]}", node)
        for attr_value in node.attrs.values():
            for identifier in URL_FRAGMENT_PATTERN.findall(attr_value):
                if identifier not in known_ids:
                    add_error(f"SVG url() 引用了不存在的 ID：{identifier}", node)

    table_nodes = by_tag.get("table", [])
    for table_index in table_nodes:
        table = parser.elements[table_index]
        captions = [
            index
            for index in table.children
            if parser.elements[index].tag == "caption"
        ]
        if len(captions) != 1:
            add_error(f"table 必须有一个直接 caption，当前为 {len(captions)} 个", table)
        else:
            caption_index = captions[0]
            if not node_text(parser, caption_index):
                add_error("table caption 文字不能为空", parser.elements[caption_index])
            if not table.children or table.children[0] != caption_index:
                add_error("table caption 必须是首个元素子节点", parser.elements[caption_index])

        table_headers = []
        for index in descendant_indices(parser, table_index):
            node = parser.elements[index]
            if node.tag != "th":
                continue
            nearest_table = next(
                (
                    ancestor
                    for ancestor in ancestor_indices(parser, index)
                    if parser.elements[ancestor].tag == "table"
                ),
                None,
            )
            if nearest_table == table_index:
                table_headers.append(index)
        if not table_headers:
            add_error("table 缺少 th 表头", table)

        for header_index in table_headers:
            header = parser.elements[header_index]
            scope = header.attrs.get("scope", "").strip().lower()
            if not scope:
                add_error("th 缺少 scope", header)
                continue
            section = next(
                (
                    parser.elements[ancestor].tag
                    for ancestor in ancestor_indices(parser, header_index)
                    if parser.elements[ancestor].tag in {"thead", "tbody", "tfoot"}
                ),
                "",
            )
            if section == "thead" and scope not in {"col", "colgroup"}:
                add_error("thead 中的 th scope 必须是 col 或 colgroup", header)
            elif section in {"tbody", "tfoot"} and scope not in {"row", "rowgroup"}:
                add_error("tbody/tfoot 中的 th scope 必须是 row 或 rowgroup", header)
            elif not section:
                add_error("th 必须位于 thead、tbody 或 tfoot 中", header)

    svg_nodes = by_tag.get("svg", [])
    for svg_index in svg_nodes:
        svg = parser.elements[svg_index]
        if start_event >= 0 and end_event >= 0 and not is_between(svg, start_event, end_event):
            add_error("SVG 位于嵌入区域之外", svg)
        if not is_valid_viewbox(svg.attrs.get("viewbox", "")):
            add_error("SVG 缺少有效且宽高为正的 viewBox", svg)
        if "width" in svg.attrs or "height" in svg.attrs:
            add_error("SVG 不应使用固定 width/height，尺寸应由 viewBox 和容器控制", svg)
        if svg.attrs.get("role", "").lower() != "img":
            add_error('SVG 缺少 role="img"', svg)

        direct_titles = [index for index in svg.children if parser.elements[index].tag == "title"]
        direct_descriptions = [index for index in svg.children if parser.elements[index].tag == "desc"]
        if len(direct_titles) != 1:
            add_error(f"SVG 必须有一个直接 title，当前为 {len(direct_titles)} 个", svg)
        if len(direct_descriptions) != 1:
            add_error(f"SVG 必须有一个直接 desc，当前为 {len(direct_descriptions)} 个", svg)

        title_id = ""
        title_text = ""
        description_id = ""
        description_text = ""
        normalized_description = ""
        if direct_titles:
            title_node = parser.elements[direct_titles[0]]
            title_id = title_node.attrs.get("id", "").strip()
            title_text = node_text(parser, direct_titles[0])
            if not title_id or not title_text:
                add_error("SVG title 必须有 ID 且文字非空", title_node)
        if direct_descriptions:
            description_node = parser.elements[direct_descriptions[0]]
            description_id = description_node.attrs.get("id", "").strip()
            description_text = node_text(parser, direct_descriptions[0])
            normalized_description = normalize_summary_text(description_text)
            if not description_id:
                add_error("SVG desc 必须有 ID", description_node)
            if not normalized_description:
                add_error(
                    "SVG desc 归一化后不能为空；作者摘要需概述入口、关键判断和结果",
                    description_node,
                )

        summary_figure_index = next(
            (
                ancestor
                for ancestor in ancestor_indices(parser, svg_index)
                if parser.elements[ancestor].tag == "figure"
            ),
            None,
        )
        summary_caption_text = ""
        if summary_figure_index is not None:
            summary_captions = [
                index
                for index in parser.elements[summary_figure_index].children
                if parser.elements[index].tag == "figcaption"
            ]
            if len(summary_captions) == 1:
                summary_caption_text = node_text(parser, summary_captions[0])

        if normalized_description:
            normalized_references = normalized_summary_references(
                title_text,
                summary_caption_text,
            )
            if normalized_description in normalized_references:
                add_error(
                    "SVG desc 归一化后不得与直接 title 或所属 figcaption 完全相同；"
                    "作者摘要需概述入口、关键判断和结果",
                    parser.elements[direct_descriptions[0]],
                )
            elif len(normalized_description) < MIN_NORMALIZED_SUMMARY_LENGTH:
                add_error(
                    f"SVG desc 归一化后至少需要 {MIN_NORMALIZED_SUMMARY_LENGTH} 个字符；"
                    "作者摘要需概述入口、关键判断和结果",
                    parser.elements[direct_descriptions[0]],
                )
            elif len(
                summary_without_references(
                    description_text,
                    title_text,
                    summary_caption_text,
                )
            ) < MIN_DISTINCT_SUMMARY_LENGTH:
                add_error(
                    "SVG desc 去除完整 title 与 figcaption 后至少需要保留 "
                    f"{MIN_DISTINCT_SUMMARY_LENGTH} 个归一化字符；"
                    "作者摘要不得只是图题的简短后缀",
                    parser.elements[direct_descriptions[0]],
                )

        svg_labelled_by = svg.attrs.get("aria-labelledby", "").split()
        if not svg_labelled_by:
            add_error("SVG 必须用 aria-labelledby 关联 title 与 desc", svg)
        else:
            for required_id, label in ((title_id, "title"), (description_id, "desc")):
                if required_id and required_id not in svg_labelled_by:
                    add_error(f"SVG aria-labelledby 未关联 {label}：{required_id}", svg)

        viewport_index = svg.parent
        viewport = parser.elements[viewport_index] if viewport_index is not None else None
        if viewport is None or "cube-flow__viewport" not in class_tokens(viewport):
            add_error("SVG 的直接父元素必须是 cube-flow__viewport", svg)
            continue
        if viewport.attrs.get("data-flow-width", "") not in FLOW_WIDTHS:
            add_error("viewport 的 data-flow-width 必须是 compact/standard/wide", viewport)
        if viewport.attrs.get("role", "").lower() != "region":
            add_error('viewport 缺少 role="region"', viewport)
        if viewport.attrs.get("tabindex", "") != "0":
            add_error('viewport 缺少 tabindex="0"', viewport)

        figure_index = next(
            (
                ancestor
                for ancestor in ancestor_indices(parser, viewport_index)
                if parser.elements[ancestor].tag == "figure"
            ),
            None,
        )
        if figure_index is None or "cube-flow-figure" not in class_tokens(parser.elements[figure_index]):
            add_error("SVG 缺少 cube-flow-figure 容器", svg)
            continue
        figure = parser.elements[figure_index]
        captions = [index for index in figure.children if parser.elements[index].tag == "figcaption"]
        if len(captions) != 1:
            add_error(f"figure 必须有一个直接 figcaption，当前为 {len(captions)} 个", figure)
            continue
        caption = parser.elements[captions[0]]
        caption_id = caption.attrs.get("id", "").strip()
        viewport_labels = viewport.attrs.get("aria-labelledby", "").split()
        if not caption_id or caption_id not in viewport_labels:
            add_error("viewport 必须通过 aria-labelledby 关联可见 figcaption", viewport)

    for node in parser.elements:
        if "mermaid" in class_tokens(node) or "mermaid" in node.attrs.get("src", "").lower():
            add_error("正式流程页不得依赖运行时 Mermaid", node)

    return FlowPageAudit(
        path=path,
        svg_count=len(svg_nodes),
        table_count=len(table_nodes),
        page_identifier=page_identifier,
        errors=errors,
    )


def audit_flow_root(flow_root: Path = FLOW_ROOT) -> tuple[list[FlowPageAudit], list[str]]:
    """严格读取并检查流程目录，返回逐页结果和跨页错误。"""

    if not flow_root.is_dir():
        return [], [f"流程目录不存在：{flow_root.as_posix()}"]

    audits: list[FlowPageAudit] = []
    cross_page_errors: list[str] = []
    for html_path in sorted(flow_root.rglob("*.html")):
        relative = html_path.relative_to(ROOT) if html_path.is_relative_to(ROOT) else html_path
        try:
            raw = html_path.read_bytes()
            text = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            audits.append(
                FlowPageAudit(
                    path=relative,
                    svg_count=0,
                    table_count=0,
                    page_identifier="",
                    errors=[f"{relative.as_posix()} 不是严格 UTF-8：{exc}"],
                )
            )
            continue
        if "\ufffd" in text:
            audits.append(
                FlowPageAudit(
                    path=relative,
                    svg_count=0,
                    table_count=0,
                    page_identifier="",
                    errors=[f"{relative.as_posix()} 包含 Unicode 替换字符 U+FFFD"],
                )
            )
            continue
        audits.append(validate_flow_html(text, relative))

    identifiers = Counter(audit.page_identifier for audit in audits if audit.page_identifier)
    for identifier, count in identifiers.items():
        if count > 1:
            cross_page_errors.append(f"data-cube-flow-page 跨页重复：{identifier}（{count} 页）")
    return audits, cross_page_errors


def configure_output(stdout=sys.stdout, stderr=sys.stderr) -> None:  # noqa: ANN001
    for stream in (stdout, stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding=OUTPUT_ENCODING, errors="replace")


def main() -> int:
    configure_output()
    audits, cross_page_errors = audit_flow_root()
    errors = [error for audit in audits for error in audit.errors] + cross_page_errors
    if errors:
        print("程序流程 HTML 契约检查失败：")
        for error in errors:
            print(f"- {error}")
        return 1

    svg_count = sum(audit.svg_count for audit in audits)
    table_count = sum(audit.table_count for audit in audits)
    print(
        f"程序流程 HTML 契约检查通过：{len(audits)} 页，"
        f"{svg_count} 张 SVG，{table_count} 张表格。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

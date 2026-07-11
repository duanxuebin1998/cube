#!/usr/bin/env python3
"""check_flow_docs.py 的流程页契约测试。"""

from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_flow_docs.py")
VALID_DESCRIPTION = "从启动入口读取参数，根据校验结果进入执行或错误出口。"


def load_module():
    spec = importlib.util.spec_from_file_location("check_flow_docs", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_flow_docs.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def valid_page(
    page_identifier: str = "cpu2-demo",
    description: str = VALID_DESCRIPTION,
) -> str:
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>示例流程</title>
<link rel="stylesheet" href="../assets/网站嵌入增强.css">
</head>
<body class="cube-flow-page" data-cube-flow-page="{page_identifier}">
<div class="wrap">
<header class="hero"><h1>示例流程</h1></header>
<!-- CUBE_EMBED_START -->
<main>
<h2>1. 示例参数</h2>
<div class="cube-flow-table"><div class="cube-flow-table__hint" id="demo-table-hint">横向滚动，键盘 ← → 查看完整表格</div>
<div class="cube-flow-table__viewport" tabindex="0" role="region" aria-labelledby="demo-table-caption" aria-describedby="demo-table-hint">
<table>
<caption id="demo-table-caption">示例参数表</caption>
<thead><tr><th scope="col">参数</th><th scope="col">说明</th></tr></thead>
<tbody><tr><td>示例值</td><td>示例说明</td></tr></tbody>
</table></div></div>
<figure class="cube-flow-figure">
<figcaption id="demo-caption">示例流程图</figcaption>
<div class="cube-flow__viewport" data-flow-width="standard" role="region"
     aria-labelledby="demo-caption" aria-describedby="demo-desc" tabindex="0">
<svg viewBox="0 0 1120 640" aria-hidden="true" focusable="false">
<title id="demo-title">示例流程图</title>
<desc id="demo-desc">{description}</desc>
<defs><marker id="arrow"><path d="M0 0 L10 5 L0 10"></path></marker></defs>
<path d="M0 0 L10 10" marker-end="url(#arrow)"></path>
</svg>
</div>
</figure>
</main>
<!-- CUBE_EMBED_END -->
</div>
<script src="../assets/网站嵌入增强.js"></script>
</body>
</html>"""


class CheckFlowDocsTests(unittest.TestCase):
    def test_accepts_complete_flow_contract(self) -> None:
        module = load_module()

        audit = module.validate_flow_html(valid_page(), Path("docs/示例.html"))

        self.assertEqual([], audit.errors)
        self.assertEqual(1, audit.svg_count)
        self.assertEqual(1, audit.table_count)
        self.assertEqual("cpu2-demo", audit.page_identifier)

    def test_rejects_missing_viewbox_and_unreferenced_description(self) -> None:
        module = load_module()
        html = valid_page().replace(' viewBox="0 0 1120 640"', "").replace(
            ' aria-describedby="demo-desc"', ""
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        joined = "\n".join(audit.errors)
        self.assertIn("缺少有效且宽高为正的 viewBox", joined)
        self.assertIn("viewport 必须通过 aria-describedby 关联 SVG desc", joined)

    def test_rejects_script_controls_inside_embed_region(self) -> None:
        module = load_module()
        html = valid_page().replace(
            "<main>",
            '<section class="tool-panel">筛选</section><script src="filter.js"></script><main>',
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        joined = "\n".join(audit.errors)
        self.assertIn("独立页专用内容位于嵌入区域内：tool-panel", joined)
        self.assertIn("独立页专用内容位于嵌入区域内：script", joined)

    def test_rejects_duplicate_html_attributes_even_when_values_match(self) -> None:
        module = load_module()
        html = valid_page().replace(
            'aria-hidden="true" focusable="false"',
            'aria-hidden="true" focusable="false" aria-hidden="true"',
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        self.assertIn(
            "svg 属性重复：aria-hidden（值：true）",
            "\n".join(audit.errors),
        )

    def test_rejects_svg_exposed_directly_to_assistive_technology(self) -> None:
        module = load_module()
        html = valid_page().replace(
            'aria-hidden="true" focusable="false"',
            'role="img" aria-labelledby="demo-title demo-desc"',
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        joined = "\n".join(audit.errors)
        self.assertIn('SVG 视觉画布缺少 aria-hidden="true"', joined)
        self.assertIn('SVG 视觉画布缺少 focusable="false"', joined)
        self.assertIn("SVG 视觉画布不应直接暴露辅助技术属性", joined)

    def test_reports_duplicate_page_identifiers_across_files(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "a.html").write_text(valid_page("same-page"), encoding="utf-8")
            (root / "b.html").write_text(valid_page("same-page"), encoding="utf-8")

            audits, cross_page_errors = module.audit_flow_root(root)

        self.assertEqual(2, len(audits))
        self.assertIn("data-cube-flow-page 跨页重复：same-page（2 页）", cross_page_errors)

    def test_rejects_missing_table_caption_scope_and_heading_jump(self) -> None:
        module = load_module()
        html = (
            valid_page()
            .replace("<h2>1. 示例参数</h2>", "<h4>1. 示例参数</h4>")
            .replace('<caption id="demo-table-caption">示例参数表</caption>\n', "")
            .replace(' scope="col"', "")
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        joined = "\n".join(audit.errors)
        self.assertIn("标题层级跳级：h1 后直接使用 h4", joined)
        self.assertIn("table 必须有一个直接 caption，当前为 0 个", joined)
        self.assertIn("th 缺少 scope", joined)

    def test_rejects_table_without_labelled_keyboard_scroll_viewport(self) -> None:
        module = load_module()
        html = (
            valid_page()
            .replace(
                '<div class="cube-flow-table"><div class="cube-flow-table__hint" '
                'id="demo-table-hint">横向滚动，键盘 ← → 查看完整表格</div>\n'
                '<div class="cube-flow-table__viewport" tabindex="0" role="region" '
                'aria-labelledby="demo-table-caption" aria-describedby="demo-table-hint">\n',
                "",
            )
            .replace("</table></div></div>", "</table>")
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        self.assertIn(
            "table 必须位于可聚焦的 cube-flow-table__viewport 中",
            "\n".join(audit.errors),
        )

    def test_rejects_missing_shared_keyboard_scroll_script(self) -> None:
        module = load_module()
        html = valid_page().replace(
            '<script src="../assets/网站嵌入增强.js"></script>\n',
            "",
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        self.assertIn(
            "统一增强脚本必须是 body 的最后一个元素",
            "\n".join(audit.errors),
        )

    def test_rejects_column_scope_on_body_row_header(self) -> None:
        module = load_module()
        html = valid_page().replace(
            "<tbody><tr><td>示例值</td><td>示例说明</td></tr></tbody>",
            '<tbody><tr><th scope="col">示例值</th><td>示例说明</td></tr></tbody>',
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        self.assertIn(
            "tbody/tfoot 中的 th scope 必须是 row 或 rowgroup",
            "\n".join(audit.errors),
        )

    def test_rejects_svg_description_empty_after_normalization(self) -> None:
        module = load_module()
        html = valid_page(description="　——……___")

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        self.assertIn(
            "SVG desc 归一化后不能为空；作者摘要需概述入口、关键判断和结果",
            "\n".join(audit.errors),
        )

    def test_rejects_svg_description_matching_title_or_caption(self) -> None:
        module = load_module()
        cases = {
            "direct title": valid_page(description="示例　流程图！"),
            "figcaption": (
                valid_page(description="示例　流程图！")
                .replace(
                    '<title id="demo-title">示例流程图</title>',
                    '<title id="demo-title">不同的可访问标题</title>',
                )
            ),
        }

        for label, html in cases.items():
            with self.subTest(label=label):
                audit = module.validate_flow_html(html, Path("docs/示例.html"))
                self.assertIn(
                    "SVG desc 归一化后不得与直接 title 或所属 figcaption 完全相同；"
                    "作者摘要需概述入口、关键判断和结果",
                    "\n".join(audit.errors),
                )

    def test_rejects_svg_description_with_only_a_short_caption_suffix(self) -> None:
        module = load_module()

        audit = module.validate_flow_html(
            valid_page(description="示例流程图说明"),
            Path("docs/示例.html"),
        )

        self.assertIn(
            "SVG desc 归一化后至少需要 12 个字符",
            "\n".join(audit.errors),
        )

    def test_rejects_long_copied_caption_with_trivial_suffix(self) -> None:
        module = load_module()
        long_caption = "程序启动参数校验与执行结果流程图"
        html = (
            valid_page(description=f"{long_caption}说明")
            .replace(
                '<figcaption id="demo-caption">示例流程图</figcaption>',
                f'<figcaption id="demo-caption">{long_caption}</figcaption>',
            )
            .replace(
                '<title id="demo-title">示例流程图</title>',
                f'<title id="demo-title">{long_caption}</title>',
            )
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        self.assertGreaterEqual(
            len(module.normalize_summary_text(f"{long_caption}说明")),
            module.MIN_NORMALIZED_SUMMARY_LENGTH,
        )
        self.assertIn(
            "SVG desc 去除完整 title 与 figcaption 后至少需要保留 6 个归一化字符",
            "\n".join(audit.errors),
        )

    def test_accepts_summary_at_inclusive_quality_boundaries(self) -> None:
        module = load_module()
        caption = "参数校验流程"
        description = f"{caption}入口判断结果"
        html = (
            valid_page(description=description)
            .replace(
                '<figcaption id="demo-caption">示例流程图</figcaption>',
                f'<figcaption id="demo-caption">{caption}</figcaption>',
            )
            .replace(
                '<title id="demo-title">示例流程图</title>',
                f'<title id="demo-title">{caption}</title>',
            )
        )

        audit = module.validate_flow_html(html, Path("docs/示例.html"))

        self.assertEqual(
            module.MIN_NORMALIZED_SUMMARY_LENGTH,
            len(module.normalize_summary_text(description)),
        )
        self.assertEqual(
            module.MIN_DISTINCT_SUMMARY_LENGTH,
            len(module.summary_without_references(description, caption, caption)),
        )
        self.assertEqual([], audit.errors)


if __name__ == "__main__":
    unittest.main()

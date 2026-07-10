#!/usr/bin/env python3
"""update_flow_navigation.py 的安全更新行为测试。"""

from __future__ import annotations

import codecs
import importlib.util
import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).with_name("update_flow_navigation.py")


def load_module():
    spec = importlib.util.spec_from_file_location("update_flow_navigation", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load update_flow_navigation.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class UpdateFlowNavigationTests(unittest.TestCase):
    def test_legacy_svg_label_conversion_is_idempotent(self) -> None:
        module = load_module()
        source = (
            '<svg role="img" aria-labelledby="title desc" aria-describedby="desc" '
            'aria-label="旧说明"><title id="title">流程</title>'
            '<desc id="desc">流程说明</desc></svg>'
        )

        converted = module.replace_legacy_svg_labels(source)
        converted = module.normalize_duplicate_svg_references(converted)

        self.assertEqual(1, converted.count('aria-labelledby="title desc"'))
        self.assertNotIn("aria-label=", converted)
        self.assertNotIn("aria-describedby=", converted)
        self.assertEqual(converted, module.replace_legacy_svg_labels(converted))
        self.assertEqual(
            converted,
            module.normalize_duplicate_svg_references(converted),
        )

    def test_collapses_matching_duplicate_svg_references(self) -> None:
        module = load_module()
        source = (
            '<svg aria-labelledby="title desc" role="img" '
            "aria-labelledby='title desc' aria-describedby=\"desc\" "
            "aria-describedby='desc'><title id=\"title\">流程</title></svg>"
        )

        normalized = module.normalize_duplicate_svg_references(source)

        self.assertEqual(1, normalized.lower().count("aria-labelledby"))
        self.assertEqual(1, normalized.lower().count("aria-describedby"))
        self.assertEqual(normalized, module.normalize_duplicate_svg_references(normalized))

    def test_rejects_conflicting_duplicate_svg_references(self) -> None:
        module = load_module()
        source = (
            '<svg aria-labelledby="title desc" aria-labelledby="other">'
            '<title id="title">流程</title></svg>'
        )

        with self.assertRaisesRegex(
            RuntimeError,
            r"Conflicting duplicate aria-labelledby in fixture\.html",
        ):
            module.normalize_duplicate_svg_references(source, Path("fixture.html"))

    def test_normalizes_table_semantics_from_existing_heading_idempotently(self) -> None:
        module = load_module()
        source = (
            '<h2>3.0 找液位/跟随函数总览与方式边界</h2>'
            '<table><thead><tr><th>方式</th><th style="width:40%">职责</th></tr></thead>'
            '<tbody><tr><td>0</td><td>步进</td></tr></tbody></table>'
            '<table><thead><tr><th>函数组</th><th>职责</th></tr></thead>'
            '<tbody><tr><td>入口</td><td>分发</td></tr></tbody></table>'
        )

        normalized = module.normalize_flow_semantics(source, Path("fixture.html"))

        self.assertIn(
            '<caption>找液位/跟随函数总览与方式边界（表 1）</caption>',
            normalized,
        )
        self.assertIn(
            '<caption>找液位/跟随函数总览与方式边界（表 2）</caption>',
            normalized,
        )
        self.assertEqual(4, normalized.count('scope="col"'))
        self.assertEqual(
            normalized,
            module.normalize_flow_semantics(normalized, Path("fixture.html")),
        )

    def test_preserves_authored_caption_scope_and_ambiguous_colgroup(self) -> None:
        module = load_module()
        source = (
            '<h2>1. 参数</h2><table><caption>作者标题</caption><thead><tr>'
            '<th scope="col">参数</th><th colspan="2">组合值</th>'
            '</tr></thead><tbody><tr><td>A</td><td>1</td><td>2</td></tr></tbody></table>'
        )

        normalized = module.normalize_table_semantics(source, Path("fixture.html"))

        self.assertEqual(source, normalized)

    def test_promotes_source_card_heading_without_changing_text(self) -> None:
        module = load_module()
        source = (
            '<section><h2>5. 关键源码索引</h2><div class="source-card">'
            '<h4 data-kind="entry">main 主入口</h4><p>说明</p></div></section>'
        )

        normalized = module.normalize_source_card_headings(source)

        self.assertIn('<h3 data-kind="entry">main 主入口</h3>', normalized)
        self.assertNotIn("<h4", normalized)
        self.assertEqual(normalized, module.normalize_source_card_headings(normalized))

    def test_refuses_to_invent_caption_without_preceding_heading(self) -> None:
        module = load_module()
        source = '<table><thead><tr><th>参数</th></tr></thead></table>'

        with self.assertRaisesRegex(
            RuntimeError,
            r"Table has no preceding heading in fixture\.html",
        ):
            module.normalize_table_semantics(source, Path("fixture.html"))

    def test_default_mode_checks_without_writing(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "flow.html"
            path.write_bytes(b"before\n")
            plan = module.InMemoryUpdatePlan()
            plan.stage_text(path, "after\n")

            output = io.StringIO()
            with mock.patch.object(module, "build_update_plan", return_value=plan):
                with redirect_stdout(output):
                    result = module.main([])

            self.assertEqual(1, result)
            self.assertEqual(b"before\n", path.read_bytes())
            self.assertIn("run with --write", output.getvalue())

    def test_write_mode_applies_the_validated_plan(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "flow.html"
            path.write_bytes(b"before\n")
            plan = module.InMemoryUpdatePlan()
            plan.stage_text(path, "after\n")

            with mock.patch.object(module, "build_update_plan", return_value=plan):
                with redirect_stdout(io.StringIO()):
                    result = module.main(["--write"])

            self.assertEqual(0, result)
            self.assertEqual(b"after\n", path.read_bytes())

    def test_write_only_replaces_files_with_real_content_changes(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            unchanged = root / "unchanged.html"
            changed = root / "changed.html"
            unchanged.write_bytes(b"same\r\n")
            changed.write_bytes(b"before\r\n")
            plan = module.InMemoryUpdatePlan()
            plan.stage_text(unchanged, "same\n")
            plan.stage_text(changed, "after\n")

            real_replace = os.replace
            with mock.patch.object(module.os, "replace", wraps=real_replace) as replace:
                written = plan.apply()

            self.assertEqual((changed,), written)
            self.assertEqual(b"same\r\n", unchanged.read_bytes())
            self.assertEqual(b"after\r\n", changed.read_bytes())
            self.assertEqual(1, replace.call_count)
            self.assertEqual(changed, Path(replace.call_args.args[1]))

    def test_retries_transient_permission_error_during_atomic_replace(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "flow.html"
            path.write_bytes(b"before\n")
            plan = module.InMemoryUpdatePlan()
            plan.stage_text(path, "after\n")
            real_replace = os.replace
            calls = 0

            def replace_after_transient_error(source, target):
                nonlocal calls
                calls += 1
                if calls == 1:
                    raise PermissionError(5, "transient file lock")
                return real_replace(source, target)

            with mock.patch.object(
                module.os,
                "replace",
                side_effect=replace_after_transient_error,
            ) as replace:
                with mock.patch.object(module.time, "sleep") as sleep:
                    written = plan.apply()

            self.assertEqual((path,), written)
            self.assertEqual(b"after\n", path.read_bytes())
            self.assertEqual(2, replace.call_count)
            sleep.assert_called_once_with(module.REPLACE_RETRY_DELAYS_SECONDS[0])
            self.assertEqual([], list(Path(temp_dir).glob(".*.tmp")))

    def test_preserves_utf8_bom_crlf_and_missing_trailing_newline(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "flow.html"
            path.write_bytes(codecs.BOM_UTF8 + "第一行\r\n第二行".encode("utf-8"))
            plan = module.InMemoryUpdatePlan()
            plan.stage_text(path, "第一行\n第二行\n第三行\n")

            plan.apply()

            self.assertEqual(
                codecs.BOM_UTF8 + "第一行\r\n第二行\r\n第三行".encode("utf-8"),
                path.read_bytes(),
            )

    def test_preserves_existing_trailing_newline(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "flow.html"
            path.write_bytes(b"before\n")
            plan = module.InMemoryUpdatePlan()
            plan.stage_text(path, "after")

            plan.apply()

            self.assertEqual(b"after\n", path.read_bytes())

    def test_preserves_multiple_trailing_newlines(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "flow.html"
            path.write_bytes(b"before\r\n\r\n")
            plan = module.InMemoryUpdatePlan()
            plan.stage_text(path, "after\n")

            plan.apply()

            self.assertEqual(b"after\r\n\r\n", path.read_bytes())

    def test_concurrent_change_is_detected_before_any_replacement(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            first = root / "first.html"
            second = root / "second.html"
            first.write_bytes(b"first-before\n")
            second.write_bytes(b"second-before\n")
            plan = module.InMemoryUpdatePlan()
            plan.stage_text(first, "first-after\n")
            plan.stage_text(second, "second-after\n")
            assert_unchanged = plan._assert_sources_unchanged
            checks = 0

            def change_between_preflight_checks() -> None:
                nonlocal checks
                checks += 1
                if checks == 2:
                    second.write_bytes(b"external-change\n")
                assert_unchanged()

            with mock.patch.object(
                plan,
                "_assert_sources_unchanged",
                side_effect=change_between_preflight_checks,
            ):
                with self.assertRaises(module.ConcurrentSourceChangeError):
                    plan.apply()

            self.assertEqual(b"first-before\n", first.read_bytes())
            self.assertEqual(b"external-change\n", second.read_bytes())
            self.assertEqual([], list(root.glob(".*.tmp")))


if __name__ == "__main__":
    unittest.main()

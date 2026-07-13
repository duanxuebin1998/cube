#!/usr/bin/env python3
"""update_flow_navigation.py 的安全更新行为测试。"""

from __future__ import annotations

import codecs
import importlib.util
import io
import json
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
    def test_route_manifest_is_the_only_cross_cpu_route_source(self) -> None:
        module = load_module()

        manifest = module.load_route_manifest()
        routes = manifest["routes"]
        self.assertEqual(2, manifest["schemaVersion"])
        self.assertEqual(
            list(module.CLOSURE_ROLE_IDS),
            [role["id"] for role in manifest["closureRoles"]],
        )
        self.assertEqual(
            ["oil", "water", "density", "readparams", "ao", "param", "fault", "external"],
            [route["id"] for route in routes],
        )
        self.assertTrue(
            all(sum(step["map"] is True for step in route["steps"]) == 4 for route in routes)
        )
        self.assertTrue(
            all(set(route["closure"]) == set(module.CLOSURE_ROLE_IDS) for route in routes)
        )
        self.assertFalse(hasattr(module, "_INLINE_ROUTES"))

    def test_route_manifest_rejects_unknown_formal_page(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.ROUTE_MANIFEST_DATA, ensure_ascii=False))
        manifest["routes"][0]["steps"][0]["pageKey"] = "missing_page"

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "routes.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "unknown pageKey"):
                module.load_route_manifest(path)

    def test_route_manifest_rejects_missing_closure_role(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.ROUTE_MANIFEST_DATA, ensure_ascii=False))
        del manifest["routes"][0]["closure"]["exception"]

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "routes.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "all six required roles"):
                module.load_route_manifest(path)

    def test_route_manifest_rejects_closure_page_outside_formal_steps(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.ROUTE_MANIFEST_DATA, ensure_ascii=False))
        manifest["routes"][0]["closure"]["exception"] = ["cpu2_08"]

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "routes.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "outside formal steps"):
                module.load_route_manifest(path)

    def test_evidence_manifest_covers_every_core_route_page(self) -> None:
        module = load_module()

        manifest = module.load_evidence_manifest()
        core_keys = {
            step["pageKey"]
            for route in module.ROUTES
            for step in route["steps"]
            if step.get("formal") is True
        }
        self.assertEqual(core_keys, {page["pageKey"] for page in manifest["pages"]})
        self.assertEqual("V1.23.0.0", manifest["firmwareBaseline"]["cpu2"]["version"])
        self.assertEqual("V1.22.0.0", manifest["firmwareBaseline"]["cpu3"]["version"])

    def test_evidence_manifest_rejects_missing_core_page(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.EVIDENCE_MANIFEST_DATA, ensure_ascii=False))
        manifest["pages"] = manifest["pages"][1:]

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "evidence.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "coverage must exactly match"):
                module.load_evidence_manifest(path)

    def test_evidence_manifest_rejects_missing_governance_policy(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.EVIDENCE_MANIFEST_DATA, ensure_ascii=False))
        del manifest["governancePolicy"]

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "evidence.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "governancePolicy"):
                module.load_evidence_manifest(path)

    def test_evidence_manifest_rejects_missing_responsibility_owner(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.EVIDENCE_MANIFEST_DATA, ensure_ascii=False))
        manifest["governancePolicy"]["responsibilities"]["validation"]["owner"] = ""

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "evidence.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "responsibility is invalid: validation"):
                module.load_evidence_manifest(path)

    def test_evidence_manifest_rejects_stale_firmware_baseline(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.EVIDENCE_MANIFEST_DATA, ensure_ascii=False))
        manifest["firmwareBaseline"]["cpu2"]["version"] = "V1.21.5.0"

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "evidence.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "baseline is stale"):
                module.load_evidence_manifest(path)

    def test_evidence_manifest_rejects_missing_source_file(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.EVIDENCE_MANIFEST_DATA, ensure_ascii=False))
        manifest["pages"][0]["sourcePaths"] = ["LTD_MAIN_CPU2/Application/Src/not-found.c"]

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "evidence.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "does not exist"):
                module.load_evidence_manifest(path)

    def test_evidence_manifest_rejects_missing_semantic_symbol(self) -> None:
        module = load_module()
        manifest = json.loads(json.dumps(module.EVIDENCE_MANIFEST_DATA, ensure_ascii=False))
        semantic_page = next(page for page in manifest["pages"] if page.get("sourceSymbols"))
        semantic_page["sourceSymbols"][0]["symbols"][0]["name"] = "SymbolThatDoesNotExist"

        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "evidence.json"
            path.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
            with self.assertRaisesRegex(RuntimeError, "symbol does not exist"):
                module.load_evidence_manifest(path)

    def test_flow_governance_distinguishes_risk_from_intentional_reuse(self) -> None:
        module = load_module()

        report = module.analyze_flow_governance()
        self.assertEqual(18, report["metrics"]["corePages"])
        self.assertEqual(48, report["metrics"]["closureRoles"])
        self.assertEqual(0, report["metrics"]["blockingIssues"])
        self.assertEqual(0, report["metrics"]["warnings"])
        self.assertEqual(2, len(report["intentionalReuse"]))

        routes = json.loads(json.dumps(module.ROUTES, ensure_ascii=False))
        routes[0]["steps"].append(dict(routes[0]["steps"][0]))
        duplicate_report = module.analyze_flow_governance(routes=routes)
        self.assertTrue(
            any(issue["rule"] == "duplicate-step" for issue in duplicate_report["issues"])
        )

        relations = json.loads(json.dumps(module.RELATIONS, ensure_ascii=False))
        relations["cpu2_02"]["upstream"] = []
        relation_report = module.analyze_flow_governance(relations=relations)
        self.assertTrue(
            any(issue["rule"] == "relation-gap" for issue in relation_report["issues"])
        )

    def test_core_page_navigation_contains_generated_evidence_head(self) -> None:
        module = load_module()

        output = module.make_relation_block("cpu2_02")

        self.assertIn('data-evidence-page="cpu2_02"', output)
        self.assertIn("CPU2 V1.23.0.0 / CPU3 V1.22.0.0", output)
        self.assertIn("LTD_MAIN_CPU2/Application/Src/measure.c", output)
        self.assertIn("当前版本验证", output)
        self.assertIn("正式来源与派生", output)
        self.assertIn("内容责任", output)
        self.assertIn("未确认边界", output)
        self.assertIn('data-evidence-owner="CPU2 固件责任人"', output)

    def test_legacy_svg_label_conversion_is_idempotent(self) -> None:
        module = load_module()
        source = (
            '<svg role="img" aria-labelledby="title desc" aria-describedby="desc" '
            'aria-label="旧说明"><title id="title">流程</title>'
            '<desc id="desc">流程说明</desc></svg>'
        )

        converted = module.replace_legacy_svg_labels(source)
        converted = module.normalize_duplicate_svg_references(converted)

        self.assertEqual(1, converted.count('aria-hidden="true"'))
        self.assertEqual(1, converted.count('focusable="false"'))
        self.assertNotIn('role="img"', converted)
        self.assertNotIn("aria-label=", converted)
        self.assertNotIn("aria-labelledby=", converted)
        self.assertNotIn("aria-describedby=", converted)
        self.assertEqual(converted, module.replace_legacy_svg_labels(converted))
        self.assertEqual(
            converted,
            module.normalize_duplicate_svg_references(converted),
        )

    def test_cross_route_exposes_summary_on_viewport_and_hides_svg_canvas(self) -> None:
        module = load_module()

        output = module.make_cross_route()

        self.assertIn(
            'aria-describedby="cube-flow-跨cpu业务链路-diagram-01-desc"',
            output,
        )
        self.assertIn('aria-hidden="true" focusable="false"', output)
        self.assertNotIn('role="img"', output)
        self.assertIn(
            '<script src="assets/网站嵌入增强.js"></script>',
            output,
        )
        self.assertEqual(8, output.count('class="route-closure"'))
        self.assertEqual(48, output.count('class="route-closure__item"'))
        self.assertIn("异常出口", output)
        self.assertIn("验证证据", output)

    def test_generated_navigation_pages_share_the_standalone_embed_controller(self) -> None:
        module = load_module()

        for output in (module.make_global_index(), module.make_cross_route()):
            self.assertEqual(1, output.count(module.EMBED_SCRIPT_NAME))
            self.assertLess(
                output.index(module.EMBED_SCRIPT_NAME),
                output.index("</body>"),
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

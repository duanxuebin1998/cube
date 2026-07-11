#!/usr/bin/env python3
"""check_flow_impact.py 的反向索引和影响分析测试。"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_flow_impact.py")


def load_module():
    spec = importlib.util.spec_from_file_location("check_flow_impact", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_flow_impact.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def fixture_data():
    routes = {
        "routes": [
            {
                "id": "measure",
                "title": "测量链路",
                "steps": [
                    {
                        "formal": True,
                        "pageKey": "cpu2_02",
                        "label": "测量入口",
                        "siteHref": "/flow/cpu2-02/",
                    },
                    {
                        "formal": True,
                        "pageKey": "cpu3_02",
                        "label": "状态回读",
                        "siteHref": "/flow/cpu3-02/",
                    },
                ],
            }
        ]
    }
    evidence = {
        "defaultValidationPaths": ["docs/validation/default.md"],
        "pages": [
            {
                "pageKey": "cpu2_02",
                "sourcePaths": ["LTD_MAIN_CPU2/Application/Src/measure.c"],
                "validationPaths": ["docs/validation/measure.md"],
            },
            {
                "pageKey": "cpu3_02",
                "sourcePaths": ["LTD_DISPLAY_CPU3/Communication/cpu2.c"],
                "validationPaths": [],
            },
        ],
    }
    documents = {
        "cpu2_02": "docs/00_程序流程导航/CPU2/02.html",
        "cpu3_02": "docs/00_程序流程导航/CPU3/02.html",
    }
    return routes, evidence, documents


class CheckFlowImpactTests(unittest.TestCase):
    def test_current_manifests_build_a_complete_reverse_index(self) -> None:
        module = load_module()

        index = module.build_flow_index(
            module.load_json(module.ROUTE_MANIFEST),
            module.load_json(module.EVIDENCE_MANIFEST),
            module.discover_flow_documents(),
        )

        self.assertEqual(18, len(index["pages"]))
        self.assertEqual(8, len(index["routes"]))
        self.assertGreaterEqual(index["sourcePathCount"], 18)
        self.assertTrue(all(page["routeIds"] for page in index["pages"]))
        self.assertTrue(all(page["documentPath"].endswith(".html") for page in index["pages"]))

    def test_maps_changed_source_to_pages_routes_and_validation(self) -> None:
        module = load_module()
        routes, evidence, documents = fixture_data()
        index = module.build_flow_index(routes, evidence, documents)

        report = module.analyze_changed_paths(
            index,
            [
                "LTD_MAIN_CPU2\\Application\\Src\\measure.c",
                "LTD_MAIN_CPU2/Application/Src/unmapped.c",
                "LTD_MAIN_CPU2/Application/Inc/app_version.h",
                "docs/00_程序流程导航/CPU2/02.html",
            ],
        )

        self.assertEqual(1, report["metrics"]["affectedPages"])
        self.assertEqual(1, report["metrics"]["affectedRoutes"])
        self.assertEqual(1, report["metrics"]["unmappedFirmwareSources"])
        self.assertEqual(2, report["metrics"]["firmwareSources"])
        self.assertEqual([], report["needsReviewPageKeys"])
        self.assertEqual("cpu2_02", report["affectedPages"][0]["pageKey"])
        self.assertTrue(report["affectedPages"][0]["reviewed"])
        self.assertEqual(
            ["docs/validation/default.md", "docs/validation/measure.md"],
            report["validationPaths"],
        )

    def test_reports_mapped_source_without_flow_review(self) -> None:
        module = load_module()
        routes, evidence, documents = fixture_data()
        index = module.build_flow_index(routes, evidence, documents)

        report = module.analyze_changed_paths(
            index,
            ["LTD_DISPLAY_CPU3/Communication/cpu2.c"],
        )

        self.assertEqual(["cpu3_02"], report["needsReviewPageKeys"])
        self.assertEqual(1, report["metrics"]["needsReview"])
        self.assertFalse(report["affectedPages"][0]["reviewed"])

    def test_evidence_manifest_change_acknowledges_impacted_pages(self) -> None:
        module = load_module()
        routes, evidence, documents = fixture_data()
        index = module.build_flow_index(routes, evidence, documents)

        report = module.analyze_changed_paths(
            index,
            [
                "LTD_DISPLAY_CPU3/Communication/cpu2.c",
                "docs/00_程序流程导航/流程证据清单.json",
            ],
        )

        self.assertEqual([], report["needsReviewPageKeys"])
        self.assertTrue(report["flowManifestChanged"])

    def test_text_report_separates_exact_impact_from_unmapped_firmware(self) -> None:
        module = load_module()
        routes, evidence, documents = fixture_data()
        index = module.build_flow_index(routes, evidence, documents)
        report = module.analyze_changed_paths(
            index,
            [
                "LTD_MAIN_CPU2/Application/Src/measure.c",
                "LTD_MAIN_CPU2/Application/Src/unmapped.c",
            ],
        )

        text = module.render_text("fixture", report)

        self.assertIn("cpu2_02 测量入口：需要复核", text)
        self.assertIn("未映射到核心流程的固件源码", text)
        self.assertIn("docs/validation/measure.md", text)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""验证工作包契约测试。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from check_validation_work_packages import (
    WorkPackageValidationError,
    build_work_package_report,
    parse_work_package,
)


METADATA = """<!-- cube-validation-result
{
  "schemaVersion": 1,
  "status": "pending",
  "executedAt": "2026-07-12",
  "environment": "计划阶段",
  "source": "测试矩阵",
  "firmwareBaseline": {"cpu2": "V1.22.0.0"},
  "summary": "待执行"
}
-->"""


def package_content(*, include_field: bool = True) -> str:
    rows = [
        "| T-A | 自动 | 条件 | 步骤 | 日志 | 失败 | 正确 | 通过 | 待执行 |",
        "| T-B | 台架 | 条件 | 步骤 | 日志 | 失败 | 正确 | 通过 | 待执行 |",
        "| T-D | 实机 | 条件 | 步骤 | 日志 | 失败 | 正确 | 通过 | 待执行 |",
    ]
    if include_field:
        rows.append("| T-F | 现场 | 条件 | 步骤 | 日志 | 失败 | 正确 | 通过 | 待执行 |")
    return "\n".join(
        [
            "# 测试工作包",
            "",
            "`gap-cpu2-01-specific-evidence` `cpu2_01` `ev-wp-test`",
            "",
            METADATA,
            "",
            "## 可执行测试矩阵",
            "",
            "| ID | 层 | 前置条件 | 操作步骤 | 观测位置 | 停止条件 | 预期结果 | 通过标准 | 状态 |",
            "| --- | --- | --- | --- | --- | --- | --- | --- | --- |",
            *rows,
            "",
            "## 通过标准",
            "",
            "四层均形成记录。",
        ]
    )


class WorkPackageTests(unittest.TestCase):
    def test_parser_requires_four_layers_and_metadata(self) -> None:
        parsed = parse_work_package(package_content(), "fixture.md")

        self.assertEqual("pending", parsed["result"]["status"])
        self.assertEqual(4, parsed["matrixCaseCount"])
        self.assertEqual(1, parsed["layerCaseCounts"]["field"])
        self.assertEqual(
            {"caseId": "T-B", "layer": "bench", "plannedStatus": "pending"},
            parsed["matrixCases"][1],
        )

    def test_parser_rejects_missing_layer(self) -> None:
        with self.assertRaisesRegex(WorkPackageValidationError, "缺少验证层"):
            parse_work_package(package_content(include_field=False), "fixture.md")

    def test_report_keeps_relationship_closure_separate_from_pending_result(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            relative_path = "docs/00_程序流程导航/验证工作包/test.md"
            target = root / relative_path
            target.parent.mkdir(parents=True)
            target.write_text(package_content(), encoding="utf-8")
            identity = {
                "documents": [{
                    "id": "ev-wp-test",
                    "path": relative_path,
                    "documentKind": "validation-plan",
                    "declaredLayers": ["automatic", "bench", "device", "field"],
                }]
            }
            manifest = {
                "schemaVersion": 2,
                "pages": [{"pageKey": "cpu2_01", "validationPaths": [relative_path]}],
            }
            gaps = {
                "schemaVersion": 1,
                "gaps": [{
                    "id": "gap-cpu2-01-specific-evidence",
                    "pageKey": "cpu2_01",
                    "events": [{
                        "state": "closed",
                        "owner": "文档治理",
                        "explanationPath": relative_path,
                        "relatedEvidenceIds": ["ev-wp-test"],
                    }],
                }],
            }

            report = build_work_package_report(identity, manifest, gaps, root=root)

        self.assertEqual("pending-execution", report["state"])
        self.assertEqual(1, report["summary"]["relationshipsClosed"])
        self.assertEqual(1, report["summary"]["pending"])
        self.assertTrue(report["method"]["relationshipClosureIsNotTestPass"])

    def test_report_rejects_unclosed_gap(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            relative_path = "docs/00_程序流程导航/验证工作包/test.md"
            target = root / relative_path
            target.parent.mkdir(parents=True)
            target.write_text(package_content(), encoding="utf-8")
            identity = {"documents": [{
                "id": "ev-wp-test", "path": relative_path, "documentKind": "validation-plan",
                "declaredLayers": ["automatic", "bench", "device", "field"],
            }]}
            manifest = {"schemaVersion": 2, "pages": [{"pageKey": "cpu2_01", "validationPaths": [relative_path]}]}
            gaps = {"schemaVersion": 1, "gaps": [{
                "id": "gap-cpu2-01-specific-evidence", "pageKey": "cpu2_01",
                "events": [{"state": "in-progress", "owner": "验证", "explanationPath": relative_path, "relatedEvidenceIds": ["ev-wp-test"]}],
            }]}

            with self.assertRaisesRegex(WorkPackageValidationError, "尚未关闭专项关系"):
                build_work_package_report(identity, manifest, gaps, root=root)


if __name__ == "__main__":
    unittest.main()

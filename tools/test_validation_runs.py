#!/usr/bin/env python3
"""真实验证运行台账契约测试。"""

from __future__ import annotations

import hashlib
import tempfile
import unittest
from pathlib import Path

from check_validation_runs import ValidationRunError, build_validation_run_report


REVISION = "6dad8cd634a2cf576a77b78bf4b12fe14d60103e"


def packages(status: str = "pending") -> dict[str, dict[str, object]]:
    return {
        "ev-wp-test": {
            "evidenceId": "ev-wp-test",
            "path": "docs/00_程序流程导航/验证工作包/test.md",
            "label": "测试工作包",
            "declaredResultStatus": status,
            "firmwareBaseline": {"cpu2": "V1.22.0.0", "cpu3": "V1.21.0.0"},
            "cases": {
                "T-A01": {"caseId": "T-A01", "layer": "automatic", "plannedStatus": "pending"},
                "T-B01": {"caseId": "T-B01", "layer": "bench", "plannedStatus": "pending"},
            },
        }
    }


def registry(runs: list[dict[str, object]] | None = None) -> dict[str, object]:
    return {
        "schemaVersion": 1,
        "kind": "cube-validation-run-registry",
        "description": "fixture",
        "runs": runs or [],
    }


def run_entry(root: Path, *, outcome: str = "passed", case_id: str = "T-A01") -> dict[str, object]:
    record = root / "docs" / "05_测试记录" / "fixture.txt"
    record.parent.mkdir(parents=True, exist_ok=True)
    record.write_text("真实观测记录\n", encoding="utf-8")
    return {
        "id": "run-test-a01-20260712-01",
        "evidenceId": "ev-wp-test",
        "caseId": case_id,
        "layer": "automatic" if case_id == "T-A01" else "bench",
        "outcome": outcome,
        "executedAt": "2026-07-12T14:30:00+08:00",
        "operator": "测试负责人",
        "environment": "测试环境",
        "firmwareBaseline": {"cpu2": "V1.22.0.0", "cpu3": "V1.21.0.0"},
        "decisionRevision": REVISION,
        "summary": "真实结果摘要",
        "records": [{
            "path": "docs/05_测试记录/fixture.txt",
            "sha256": hashlib.sha256(record.read_bytes()).hexdigest(),
        }],
    }


class ValidationRunTests(unittest.TestCase):
    def test_empty_registry_is_pending_not_pass(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            report = build_validation_run_report(registry(), packages(), root=Path(temp_dir))

        self.assertEqual("no-runs", report["state"])
        self.assertEqual(2, report["summary"]["plannedCases"])
        self.assertEqual(0, report["summary"]["registeredRuns"])
        self.assertEqual(2, report["summary"]["pendingCases"])
        self.assertEqual(1, report["summary"]["packageStatuses"]["pending"])

    def test_latest_appended_run_defines_current_case_outcome(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            first = run_entry(root, outcome="failed")
            second = {**run_entry(root, outcome="passed"), "id": "run-test-a01-20260712-02"}
            report = build_validation_run_report(
                registry([first, second]),
                packages(status="partial"),
                root=root,
            )

        case = report["packages"][0]["cases"][0]
        self.assertEqual(2, case["runCount"])
        self.assertEqual("passed", case["currentOutcome"])
        self.assertEqual("partial", report["packages"][0]["derivedResultStatus"])

    def test_rejects_case_layer_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            item = run_entry(root)
            item["layer"] = "bench"
            with self.assertRaisesRegex(ValidationRunError, "layer 与矩阵不一致"):
                build_validation_run_report(registry([item]), packages(status="partial"), root=root)

    def test_rejects_record_hash_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            item = run_entry(root)
            item["records"][0]["sha256"] = "0" * 64
            with self.assertRaisesRegex(ValidationRunError, "SHA-256 不一致"):
                build_validation_run_report(registry([item]), packages(status="partial"), root=root)

    def test_rejects_declared_status_drift(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            item = run_entry(root)
            with self.assertRaisesRegex(ValidationRunError, "运行台账派生为 partial"):
                build_validation_run_report(registry([item]), packages(status="pending"), root=root)


if __name__ == "__main__":
    unittest.main()

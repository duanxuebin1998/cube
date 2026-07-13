#!/usr/bin/env python3
"""验证批次范围冻结、生命周期和关闭决策契约测试。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from check_validation_batches import ValidationBatchError, build_validation_batch_report


REVISION = "6dad8cd634a2cf576a77b78bf4b12fe14d60103e"


def packages() -> dict[str, dict[str, object]]:
    return {
        "ev-wp-test": {
            "evidenceId": "ev-wp-test",
            "path": "docs/work-package.md",
            "label": "测试工作包",
            "declaredResultStatus": "partial",
            "firmwareBaseline": {"cpu2": "V1.22.0.0", "cpu3": "V1.21.0.0"},
            "cases": {
                "T-A01": {"caseId": "T-A01", "layer": "automatic", "plannedStatus": "pending"},
                "T-B01": {"caseId": "T-B01", "layer": "bench", "plannedStatus": "pending"},
            },
        }
    }


def run(run_id: str, case_id: str, outcome: str = "passed") -> dict[str, object]:
    return {
        "id": run_id,
        "evidenceId": "ev-wp-test",
        "caseId": case_id,
        "layer": "automatic" if case_id == "T-A01" else "bench",
        "outcome": outcome,
        "executedAt": "2026-07-13T10:00:00+08:00",
        "operator": "测试负责人",
        "environment": "测试环境",
        "firmwareBaseline": {"cpu2": "V1.22.0.0", "cpu3": "V1.21.0.0"},
        "decisionRevision": REVISION,
        "summary": "运行摘要",
        "records": [{"path": "docs/record.txt", "sha256": "0" * 64}],
    }


def event(
    state: str,
    at: str,
    run_ids: list[str] | None = None,
    *,
    decision: str | None = None,
    exceptions: list[str] | None = None,
) -> dict[str, object]:
    value: dict[str, object] = {
        "state": state,
        "recordedAt": at,
        "owner": "测试负责人",
        "decisionRevision": REVISION,
        "explanationPath": "docs/plan.md",
        "relatedRunIds": run_ids or [],
    }
    if decision is not None:
        value["decision"] = decision
    if exceptions:
        value["exceptionPaths"] = exceptions
    return value


def batch(events: list[dict[str, object]]) -> dict[str, object]:
    return {
        "id": "batch-test-202607",
        "label": "测试验证批次",
        "milestone": {"kind": "bench-campaign", "reference": "台架闭环"},
        "owner": "测试负责人",
        "createdAt": "2026-07-12T09:00:00+08:00",
        "scopeRevision": REVISION,
        "firmwareBaseline": {"cpu2": "V1.22.0.0", "cpu3": "V1.21.0.0"},
        "window": {"start": "2026-07-12", "end": "2026-07-19"},
        "caseRefs": [
            {"evidenceId": "ev-wp-test", "caseId": "T-A01"},
            {"evidenceId": "ev-wp-test", "caseId": "T-B01"},
        ],
        "exitCriteria": ["两个范围用例都有明确结果"],
        "events": events,
    }


def registry(batches: list[dict[str, object]] | None = None) -> dict[str, object]:
    return {
        "schemaVersion": 1,
        "kind": "cube-validation-batch-registry",
        "description": "fixture",
        "batches": batches or [],
    }


def write_docs(root: Path) -> None:
    docs = root / "docs"
    docs.mkdir(parents=True)
    (docs / "plan.md").write_text("# 批次说明\n", encoding="utf-8")
    (docs / "exception.md").write_text("# 例外说明\n", encoding="utf-8")


class ValidationBatchTests(unittest.TestCase):
    def test_empty_registry_is_not_complete(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            report = build_validation_batch_report(registry(), packages(), [], root=root)

        self.assertEqual("empty", report["state"])
        self.assertEqual(0, report["summary"]["batches"])
        self.assertTrue(report["method"]["emptyRegistryIsNotComplete"])

    def test_planned_batch_freezes_scope_and_baseline(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_docs(root)
            report = build_validation_batch_report(
                registry([batch([event("planned", "2026-07-12T09:00:00+08:00")])]),
                packages(),
                [],
                root=root,
            )

        current = report["batches"][0]
        self.assertEqual("planned", current["state"])
        self.assertEqual(2, current["plannedCases"])
        self.assertEqual(2, current["pendingCases"])
        self.assertFalse(current["decisionInputComplete"])

    def test_rejects_illegal_lifecycle_transition(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_docs(root)
            events = [
                event("planned", "2026-07-12T09:00:00+08:00"),
                event("closed", "2026-07-14T09:00:00+08:00"),
            ]
            with self.assertRaisesRegex(ValidationBatchError, "不允许从 planned 转换到 closed"):
                build_validation_batch_report(registry([batch(events)]), packages(), [], root=root)

    def test_decision_ready_requires_one_run_per_frozen_case(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_docs(root)
            runs = [run("run-test-a01", "T-A01")]
            events = [
                event("planned", "2026-07-12T09:00:00+08:00"),
                event("running", "2026-07-13T09:00:00+08:00"),
                event("decision-ready", "2026-07-14T09:00:00+08:00", ["run-test-a01"]),
            ]
            with self.assertRaisesRegex(ValidationBatchError, "每个冻结用例"):
                build_validation_batch_report(registry([batch(events)]), packages(), runs, root=root)

    def test_accepted_requires_all_referenced_runs_passed(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_docs(root)
            runs = [run("run-test-a01", "T-A01"), run("run-test-b01", "T-B01", "failed")]
            ids = [item["id"] for item in runs]
            events = [
                event("planned", "2026-07-12T09:00:00+08:00"),
                event("running", "2026-07-13T09:00:00+08:00"),
                event("decision-ready", "2026-07-14T09:00:00+08:00", ids),
                event("closed", "2026-07-14T10:00:00+08:00", ids, decision="accepted"),
            ]
            with self.assertRaisesRegex(ValidationBatchError, "全部通过"):
                build_validation_batch_report(registry([batch(events)]), packages(), runs, root=root)

    def test_closed_accepted_batch_preserves_explicit_decision_run_set(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_docs(root)
            runs = [run("run-test-a01", "T-A01"), run("run-test-b01", "T-B01")]
            ids = [item["id"] for item in runs]
            events = [
                event("planned", "2026-07-12T09:00:00+08:00"),
                event("running", "2026-07-13T09:00:00+08:00"),
                event("decision-ready", "2026-07-14T09:00:00+08:00", ids),
                event("closed", "2026-07-14T10:00:00+08:00", ids, decision="accepted"),
            ]
            report = build_validation_batch_report(
                registry([batch(events)]), packages(), runs, root=root
            )

        current = report["batches"][0]
        self.assertEqual("closed", current["state"])
        self.assertEqual("accepted", current["decision"])
        self.assertTrue(current["decisionInputComplete"])
        self.assertEqual(ids, current["events"][-1]["relatedRunIds"])

    def test_accepted_with_exceptions_requires_formal_exception_path(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            write_docs(root)
            runs = [run("run-test-a01", "T-A01"), run("run-test-b01", "T-B01", "failed")]
            ids = [item["id"] for item in runs]
            events = [
                event("planned", "2026-07-12T09:00:00+08:00"),
                event("running", "2026-07-13T09:00:00+08:00"),
                event("decision-ready", "2026-07-14T09:00:00+08:00", ids),
                event(
                    "closed",
                    "2026-07-14T10:00:00+08:00",
                    ids,
                    decision="accepted-with-exceptions",
                    exceptions=["docs/exception.md"],
                ),
            ]
            report = build_validation_batch_report(
                registry([batch(events)]), packages(), runs, root=root
            )

        self.assertEqual("accepted-with-exceptions", report["batches"][0]["decision"])
        self.assertEqual(["docs/exception.md"], report["batches"][0]["events"][-1]["exceptionPaths"])


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""交付证据缺口处置生命周期测试。"""

from __future__ import annotations

import copy
import sys
import tempfile
import unittest
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from check_delivery_evidence_gaps import GapValidationError, build_gap_report, validate_gap_registry


REVISION = "1" * 40


class DeliveryEvidenceGapTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        explanation = self.root / "docs" / "explanation.md"
        explanation.parent.mkdir(parents=True)
        explanation.write_text("# 说明\n", encoding="utf-8")
        self.identity = {
            "documents": [{"id": "ev-one"}],
        }
        self.coverage = {
            "documents": [{"id": "ev-one", "specificPageKeys": []}],
            "pages": [{"pageKey": "cpu2_01", "routeIds": ["route-one"], "state": "missing-specific"}],
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def registry(self, events=None):  # noqa: ANN001
        return {
            "schemaVersion": 1,
            "kind": "cube-delivery-evidence-gap-disposition-registry",
            "gaps": [{
                "id": "gap-cpu2-01-specific-evidence",
                "pageKey": "cpu2_01",
                "events": events or [{
                    "state": "new",
                    "owner": "文档治理",
                    "explanationPath": "docs/explanation.md",
                    "decisionRevision": REVISION,
                    "relatedEvidenceIds": [],
                }],
            }],
        }

    def test_valid_registry_separates_machine_gap_and_manual_state(self) -> None:
        registry = validate_gap_registry(self.registry(), self.identity, self.coverage, root=self.root)
        report = build_gap_report(registry, self.coverage)

        self.assertEqual("active-gaps", report["state"])
        self.assertEqual(1, report["summary"]["machineGaps"])
        self.assertEqual("new", report["gaps"][0]["dispositionState"])
        self.assertEqual("aligned", report["gaps"][0]["alignment"])

    def test_every_current_machine_gap_requires_a_disposition_record(self) -> None:
        registry = self.registry()
        registry["gaps"] = []

        with self.assertRaisesRegex(GapValidationError, "尚未登记"):
            validate_gap_registry(registry, self.identity, self.coverage, root=self.root)

    def test_lifecycle_rejects_overwritten_or_skipped_states(self) -> None:
        events = self.registry()["gaps"][0]["events"] + [{
            "state": "closed",
            "owner": "验证",
            "explanationPath": "docs/explanation.md",
            "decisionRevision": REVISION,
            "relatedEvidenceIds": ["ev-one"],
        }]

        with self.assertRaisesRegex(GapValidationError, "非法状态迁移"):
            validate_gap_registry(self.registry(events), self.identity, self.coverage, root=self.root)

    def test_closed_gap_requires_machine_coverage_and_related_specific_evidence(self) -> None:
        coverage = copy.deepcopy(self.coverage)
        coverage["pages"][0]["state"] = "specific"
        coverage["documents"][0]["specificPageKeys"] = ["cpu2_01"]
        events = self.registry()["gaps"][0]["events"] + [
            {
                "state": "claimed", "owner": "验证", "explanationPath": "docs/explanation.md",
                "decisionRevision": REVISION, "relatedEvidenceIds": [],
            },
            {
                "state": "in-progress", "owner": "验证", "explanationPath": "docs/explanation.md",
                "decisionRevision": REVISION, "relatedEvidenceIds": ["ev-one"],
            },
            {
                "state": "closed", "owner": "验证", "explanationPath": "docs/explanation.md",
                "decisionRevision": REVISION, "relatedEvidenceIds": ["ev-one"],
            },
        ]

        validate_gap_registry(self.registry(events), self.identity, coverage, root=self.root)

        events[-1]["relatedEvidenceIds"] = []
        with self.assertRaisesRegex(GapValidationError, "真正覆盖"):
            validate_gap_registry(self.registry(events), self.identity, coverage, root=self.root)

    def test_not_applicable_keeps_machine_gap_visible(self) -> None:
        events = self.registry()["gaps"][0]["events"] + [{
            "state": "not-applicable",
            "owner": "架构评审",
            "explanationPath": "docs/explanation.md",
            "decisionRevision": REVISION,
            "relatedEvidenceIds": [],
        }]

        registry = validate_gap_registry(self.registry(events), self.identity, self.coverage, root=self.root)
        report = build_gap_report(registry, self.coverage)

        self.assertEqual(1, report["summary"]["machineGaps"])
        self.assertEqual(1, report["summary"]["notApplicable"])
        self.assertEqual(0, report["summary"]["active"])


if __name__ == "__main__":
    unittest.main()

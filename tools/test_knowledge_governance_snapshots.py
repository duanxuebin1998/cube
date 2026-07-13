#!/usr/bin/env python3
"""知识治理月度快照契约测试。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from check_knowledge_governance_snapshots import (
    KnowledgeGovernanceSnapshotError,
    validate_registry,
)
from manage_knowledge_governance_snapshot import build_snapshot


class KnowledgeGovernanceSnapshotTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        explanation = self.root / "docs" / "00_程序流程导航" / "说明.md"
        explanation.parent.mkdir(parents=True)
        explanation.write_text("# 说明\n", encoding="utf-8")
        self.explanation = "docs/00_程序流程导航/说明.md"

    def tearDown(self) -> None:
        self.temp.cleanup()

    def empty_registry(self) -> dict:
        return {
            "schemaVersion": 1,
            "kind": "cube-knowledge-governance-snapshot-registry",
            "description": "fixture",
            "cadence": {"period": "monthly", "timezone": "Asia/Shanghai"},
            "snapshots": [],
        }

    def build_fixture_snapshot(self) -> dict:
        return build_snapshot(
            recorded_at="2026-07-12T10:00:00+08:00",
            explanation=self.explanation,
            revision="6dad8cd634a2cf576a77b78bf4b12fe14d60103e",
            root=self.root,
            queue={
                "schemaVersion": 2,
                "kind": "cube-governance-queue",
                "items": [{
                    "governanceKey": "gov-no-run-fixture",
                    "factFingerprint": "a" * 64,
                    "ownerId": "validation",
                    "kind": "no-real-run",
                    "affected": ["cpu2_05", "docs/file.md"],
                    "lifecycle": {"state": "unclaimed"},
                }],
            },
            batches={
                "kind": "cube-validation-batches",
                "batches": [{"id": "batch-closed", "state": "closed"}, {"id": "batch-open", "state": "running"}],
            },
            baselines={
                "kind": "cube-delivery-baseline-lifecycle",
                "baselines": [{"id": "baseline-a", "state": "current"}],
            },
            evidence={
                "updated": "2026-07-12",
                "governancePolicy": {"reviewWindowDays": 90},
                "pages": [
                    {"pageKey": "cpu2_05", "reviewedAt": "2026-07-11"},
                    {"pageKey": "cpu3_02", "reviewedAt": "2026-01-01"},
                ],
            },
        )

    def test_empty_registry_is_honest_and_valid(self) -> None:
        summary = validate_registry(self.empty_registry(), root=self.root)
        self.assertEqual(summary, {"snapshots": 0, "facts": 0, "months": 0})

    def test_builder_keeps_only_stable_references(self) -> None:
        snapshot = self.build_fixture_snapshot()
        self.assertEqual(snapshot["id"], "kgs-2026-07")
        self.assertEqual(snapshot["review"]["stalePageKeys"], ["cpu3_02"])
        self.assertEqual(snapshot["governanceFacts"][0]["affectedPageKeys"], ["cpu2_05"])
        self.assertEqual(snapshot["closedBatchIds"], ["batch-closed"])
        self.assertEqual(snapshot["baselineLifecycle"], [{"id": "baseline-a", "state": "current"}])
        registry = self.empty_registry()
        registry["snapshots"].append(snapshot)
        summary = validate_registry(registry, root=self.root)
        self.assertEqual(summary["facts"], 1)

    def test_duplicate_month_is_rejected(self) -> None:
        snapshot = self.build_fixture_snapshot()
        registry = self.empty_registry()
        registry["snapshots"] = [snapshot, {**snapshot}]
        with self.assertRaisesRegex(KnowledgeGovernanceSnapshotError, "重复"):
            validate_registry(registry, root=self.root)

    def test_unstable_fact_identity_is_rejected(self) -> None:
        snapshot = self.build_fixture_snapshot()
        snapshot["governanceFacts"][0]["id"] = "temporary"
        registry = self.empty_registry()
        registry["snapshots"] = [snapshot]
        with self.assertRaisesRegex(KnowledgeGovernanceSnapshotError, "id 无效"):
            validate_registry(registry, root=self.root)


if __name__ == "__main__":
    unittest.main()

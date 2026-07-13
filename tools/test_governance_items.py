#!/usr/bin/env python3
"""治理事项接手与机器事实关闭链测试。"""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from check_governance_items import (
    GovernanceItemError,
    build_governance_item_report,
    validate_queue_alignment,
)


REVISION = "6dad8cd634a2cf576a77b78bf4b12fe14d60103e"
FINGERPRINT = "a" * 64


def registry(items: list[dict[str, object]] | None = None) -> dict[str, object]:
    return {
        "schemaVersion": 1,
        "kind": "cube-governance-item-registry",
        "description": "fixture",
        "items": items or [],
    }


def event(
    state: str,
    sequence: int,
    *,
    action_kind: str | None = None,
) -> dict[str, object]:
    actions = [] if action_kind is None else [{
        "kind": action_kind,
        "id": "run-test",
        "path": "docs/explanation.md",
    }]
    return {
        "sequence": sequence,
        "state": state,
        "recordedAt": f"2026-07-1{sequence}T10:00:00+08:00",
        "ownerId": "validation",
        "owner": "测试负责人",
        "decisionRevision": REVISION,
        "explanationPath": "docs/explanation.md",
        "factFingerprint": FINGERPRINT,
        "actionRefs": actions,
    }


def item(events: list[dict[str, object]]) -> dict[str, object]:
    return {"id": "gov-no-run-ev-test", "kind": "no-real-run", "events": events}


def queue(*, present: bool, lifecycle_state: str, event_count: int) -> dict[str, object]:
    items = []
    if present:
        items.append({
            "governanceKey": "gov-no-run-ev-test",
            "kind": "no-real-run",
            "factFingerprint": FINGERPRINT,
            "lifecycle": {"state": lifecycle_state, "eventCount": event_count},
        })
    return {"schemaVersion": 2, "kind": "cube-governance-queue", "items": items}


class GovernanceItemTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root / "docs").mkdir()
        (self.root / "docs" / "explanation.md").write_text("# 说明\n", encoding="utf-8")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def test_empty_registry_is_honest(self) -> None:
        report = build_governance_item_report(registry(), root=self.root)
        self.assertEqual("empty", report["state"])
        self.assertEqual(0, report["summary"]["events"])

    def test_claimed_item_matches_current_fact(self) -> None:
        report = build_governance_item_report(
            registry([item([event("claimed", 1)])]), root=self.root
        )
        alignment = validate_queue_alignment(report, queue(
            present=True, lifecycle_state="claimed", event_count=1
        ))
        self.assertEqual(1, alignment["current"])

    def test_activity_requires_current_machine_fact(self) -> None:
        report = build_governance_item_report(
            registry([item([event("claimed", 1)])]), root=self.root
        )
        with self.assertRaisesRegex(GovernanceItemError, "机器事实已清零"):
            validate_queue_alignment(report, queue(
                present=False, lifecycle_state="claimed", event_count=1
            ))

    def test_closed_requires_contract_action(self) -> None:
        with self.assertRaisesRegex(GovernanceItemError, "关闭动作类型不匹配"):
            build_governance_item_report(
                registry([item([
                    event("claimed", 1),
                    event("closed", 2, action_kind="validation-batch"),
                ])]),
                root=self.root,
            )

    def test_closed_fact_can_reopen_without_losing_history(self) -> None:
        report = build_governance_item_report(
            registry([item([
                event("claimed", 1),
                event("closed", 2, action_kind="validation-run"),
            ])]),
            root=self.root,
        )
        alignment = validate_queue_alignment(report, queue(
            present=True, lifecycle_state="reopened", event_count=2
        ))
        self.assertEqual(1, alignment["reopened"])

    def test_activity_cannot_change_fingerprint_silently(self) -> None:
        events = [event("claimed", 1), event("in-progress", 2, action_kind="validation-run")]
        events[1]["factFingerprint"] = "b" * 64
        with self.assertRaisesRegex(GovernanceItemError, "不得静默更换事实指纹"):
            build_governance_item_report(registry([item(events)]), root=self.root)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""交付证据基线校验、保存与生命周期管理工具测试。"""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


TOOLS_DIR = Path(__file__).resolve().parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))


def load_module(name: str):
    path = TOOLS_DIR / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def make_snapshot(*, revision: str = "a" * 40, dirty: bool = False) -> dict:
    return {
        "schemaVersion": 1,
        "scope": "current-worktree",
        "git": {
            "head": revision,
            "shortHead": revision[:8],
            "dirty": dirty,
            "statusEntries": 0 if not dirty else 2,
        },
        "firmwareBaseline": {"cpu2": "V1.22.0.0", "cpu3": "V1.21.0.0"},
        "status": {"id": "evidence-gap", "label": "验证证据未闭环", "readyForHumanDecision": False},
        "metrics": {"affectedPages": 1, "currentQueueItems": 1},
        "checkpoints": [],
        "currentQueueItems": [],
        "affectedPages": [],
        "unmappedFirmwareSourcePaths": [],
        "unprojectedPageKeys": [],
        "formalEvidence": [],
        "method": {"manualDeliveryDecisionRequired": True},
    }


def empty_registry() -> dict:
    return {
        "schemaVersion": 2,
        "kind": "cube-delivery-evidence-baseline-registry",
        "selectedBaselineId": None,
        "baselines": [],
    }


class DeliveryEvidenceBaselineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.checker = load_module("check_delivery_evidence_baselines")
        cls.saver = load_module("save_delivery_evidence_baseline")
        cls.manager = load_module("manage_delivery_evidence_baseline")

    def make_workspace(self, root: Path) -> tuple[Path, Path, str]:
        snapshot_path = root / "current.json"
        registry_path = root / "docs" / "00_程序流程导航" / "交付证据基线清单.json"
        explanation_path = "docs/decision.md"
        write_json(snapshot_path, make_snapshot())
        write_json(registry_path, empty_registry())
        (root / explanation_path).write_text("# 基线决策\n", encoding="utf-8")
        return snapshot_path, registry_path, explanation_path

    def save(self, *, root: Path, baseline_id: str, revision: str, select: bool) -> tuple[dict, Path]:
        snapshot_path, registry_path, explanation_path = self.make_workspace(root) if not (root / "current.json").exists() else (
            root / "current.json",
            root / "docs" / "00_程序流程导航" / "交付证据基线清单.json",
            "docs/decision.md",
        )
        write_json(snapshot_path, make_snapshot(revision=revision))
        return self.saver.save_baseline(
            baseline_id=baseline_id,
            label=f"测试基线 {baseline_id}",
            snapshot_path=snapshot_path,
            registry_path=registry_path,
            root=root,
            owner="测试负责人",
            explanation_path=explanation_path,
            decision_revision=revision,
            select=select,
        )

    def test_empty_registry_is_an_explicit_no_baseline_state(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            registry_path = root / "docs" / "00_程序流程导航" / "交付证据基线清单.json"
            write_json(registry_path, empty_registry())
            result = self.checker.check_registry(registry_path, root=root)
            self.assertIsNone(result["selectedBaselineId"])
            self.assertEqual([], result["baselines"])
            self.assertEqual(0, self.checker.build_lifecycle_report(result)["summary"]["trendEligible"])

    def test_unknown_schema_kind_and_implicit_selection_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            registry_path = root / "registry.json"
            invalid = empty_registry()
            invalid["schemaVersion"] = 1
            write_json(registry_path, invalid)
            with self.assertRaisesRegex(self.checker.BaselineValidationError, "不受支持"):
                self.checker.check_registry(registry_path, root=root)
            invalid = empty_registry()
            invalid["kind"] = "invalid"
            write_json(registry_path, invalid)
            with self.assertRaisesRegex(self.checker.BaselineValidationError, "kind"):
                self.checker.check_registry(registry_path, root=root)
            invalid = empty_registry()
            invalid["selectedBaselineId"] = "missing"
            write_json(registry_path, invalid)
            with self.assertRaisesRegex(self.checker.BaselineValidationError, "未指向已登记基线"):
                self.checker.check_registry(registry_path, root=root)

    def test_dirty_snapshot_cannot_be_saved_as_a_baseline(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            snapshot_path, registry_path, explanation_path = self.make_workspace(root)
            write_json(snapshot_path, make_snapshot(dirty=True))
            with self.assertRaisesRegex(self.checker.BaselineValidationError, "非干净工作区"):
                self.saver.save_baseline(
                    baseline_id="fixture",
                    label="测试基线",
                    snapshot_path=snapshot_path,
                    registry_path=registry_path,
                    root=root,
                    owner="测试负责人",
                    explanation_path=explanation_path,
                )

    def test_lifecycle_metadata_is_required_and_formal(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            snapshot_path, registry_path, explanation_path = self.make_workspace(root)
            common = dict(
                baseline_id="fixture", label="测试基线", snapshot_path=snapshot_path,
                registry_path=registry_path, root=root, explanation_path=explanation_path,
            )
            with self.assertRaisesRegex(self.checker.BaselineValidationError, "owner"):
                self.saver.save_baseline(**common, owner="")
            with self.assertRaisesRegex(self.checker.BaselineValidationError, "正式说明不存在"):
                self.saver.save_baseline(**{**common, "explanation_path": "docs/missing.md"}, owner="负责人")
            with self.assertRaisesRegex(self.checker.BaselineValidationError, "decisionRevision"):
                self.saver.save_baseline(**common, owner="负责人", decision_revision="abc")

    def test_save_without_selection_is_audit_only(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            registry, _ = self.save(root=root, baseline_id="fixture", revision="a" * 40, select=False)
            self.assertIsNone(registry["selectedBaselineId"])
            self.assertEqual("audit-only", registry["baselines"][0]["lifecycle"][-1]["state"])

    def test_selecting_new_baseline_supersedes_previous_current(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.save(root=root, baseline_id="first", revision="a" * 40, select=True)
            registry, baseline_path = self.save(root=root, baseline_id="second", revision="b" * 40, select=True)
            self.assertEqual("second", registry["selectedBaselineId"])
            self.assertTrue(baseline_path.is_file())
            states = {item["id"]: item["lifecycle"][-1]["state"] for item in registry["baselines"]}
            self.assertEqual({"first": "superseded", "second": "current"}, states)
            self.assertEqual("second", registry["baselines"][0]["lifecycle"][-1]["supersededBy"])
            report = self.checker.build_lifecycle_report(registry)
            self.assertEqual(1, report["summary"]["current"])
            self.assertEqual(1, report["summary"]["superseded"])
            self.assertEqual(2, report["summary"]["trendEligible"])

    def test_manager_retire_and_audit_only_clear_current_selection(self) -> None:
        for action in ("retire", "audit-only"):
            with self.subTest(action=action), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                self.save(root=root, baseline_id="fixture", revision="a" * 40, select=True)
                registry = self.manager.update_lifecycle(
                    baseline_id="fixture", action=action, owner="治理负责人",
                    explanation_path="docs/decision.md", decision_revision="b" * 40,
                    registry_path=root / "docs" / "00_程序流程导航" / "交付证据基线清单.json",
                    root=root,
                )
                self.assertIsNone(registry["selectedBaselineId"])
                expected_state = "retired" if action == "retire" else action
                self.assertEqual(expected_state, registry["baselines"][0]["lifecycle"][-1]["state"])
                self.assertEqual(2, len(registry["baselines"][0]["lifecycle"]))

    def test_retired_baseline_cannot_be_selected_directly(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.save(root=root, baseline_id="fixture", revision="a" * 40, select=True)
            registry_path = root / "docs" / "00_程序流程导航" / "交付证据基线清单.json"
            common = dict(
                baseline_id="fixture", owner="治理负责人", explanation_path="docs/decision.md",
                registry_path=registry_path, root=root,
            )
            self.manager.update_lifecycle(action="retire", decision_revision="b" * 40, **common)
            with self.assertRaisesRegex(self.checker.BaselineValidationError, "不能直接重新选择"):
                self.manager.update_lifecycle(action="select", decision_revision="c" * 40, **common)


if __name__ == "__main__":
    unittest.main()

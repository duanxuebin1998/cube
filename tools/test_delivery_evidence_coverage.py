#!/usr/bin/env python3
"""稳定验证证据身份与覆盖报告测试。"""

from __future__ import annotations

import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_delivery_evidence_coverage.py")
TOOLS_DIR = MODULE_PATH.parent
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))


def load_module():
    spec = importlib.util.spec_from_file_location("check_delivery_evidence_coverage", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def identity_registry() -> dict:
    return {
        "schemaVersion": 1,
        "kind": "cube-validation-evidence-identity-registry",
        "documents": [
            {
                "id": "ev-shared-contract",
                "path": "docs/shared.md",
                "documentKind": "validation-plan",
                "declaredLayers": ["automatic"],
            },
            {
                "id": "ev-flow-one",
                "path": "docs/one.md",
                "documentKind": "governance-record",
                "declaredLayers": ["bench", "device"],
            },
        ],
    }


def evidence_manifest() -> dict:
    return {
        "schemaVersion": 2,
        "defaultValidationPaths": ["docs/shared.md"],
        "pages": [
            {"pageKey": "cpu2_01", "validationPaths": ["docs/one.md"]},
            {"pageKey": "cpu2_02", "validationPaths": []},
        ],
    }


def route_manifest() -> dict:
    return {
        "schemaVersion": 2,
        "routes": [
            {
                "id": "route-one",
                "title": "测试链路",
                "steps": [
                    {"pageKey": "cpu2_01"},
                    {"pageKey": "cpu2_02"},
                ],
            }
        ],
    }


def empty_baselines() -> dict:
    return {
        "schemaVersion": 2,
        "kind": "cube-delivery-evidence-baseline-registry",
        "selectedBaselineId": None,
        "baselines": [],
    }


class DeliveryEvidenceCoverageTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module()

    def make_docs(self, root: Path) -> None:
        (root / "docs").mkdir(parents=True, exist_ok=True)
        (root / "docs" / "shared.md").write_text("# 共享\n", encoding="utf-8")
        (root / "docs" / "one.md").write_text("# 专项\n", encoding="utf-8")

    def test_registry_requires_stable_unique_identity_and_declared_layers(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_docs(root)
            registry = self.module.validate_identity_registry(identity_registry(), root=root)
            self.assertEqual(2, len(registry["documents"]))

            invalid = identity_registry()
            invalid["documents"][1]["id"] = invalid["documents"][0]["id"]
            with self.assertRaisesRegex(self.module.CoverageValidationError, "重复 id"):
                self.module.validate_identity_registry(invalid, root=root)

            invalid = identity_registry()
            invalid["documents"][0]["declaredLayers"] = ["automatic", "automatic"]
            with self.assertRaisesRegex(self.module.CoverageValidationError, "declaredLayers"):
                self.module.validate_identity_registry(invalid, root=root)

    def test_report_separates_shared_identity_from_specific_coverage(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_docs(root)
            identity = self.module.validate_identity_registry(identity_registry(), root=root)
            report = self.module.build_coverage_report(
                identity, evidence_manifest(), route_manifest(), empty_baselines(), root=root
            )

            self.assertEqual("gaps-present", report["state"])
            self.assertEqual(1, report["summary"]["flowsWithSpecificEvidence"])
            self.assertEqual(1, report["summary"]["flowsMissingSpecificEvidence"])
            self.assertEqual("shared", report["documents"][0]["scope"])
            self.assertEqual(["cpu2_01", "cpu2_02"], report["documents"][0]["coveredPageKeys"])
            self.assertEqual(["cpu2_02"], report["routes"][0]["missingSpecificPageKeys"])
            self.assertEqual("not-evaluated", report["selectedBaselineCoverage"]["state"])

    def test_registry_must_exactly_cover_manifest_paths(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_docs(root)
            identity = self.module.validate_identity_registry(identity_registry(), root=root)
            identity["documents"].pop()
            with self.assertRaisesRegex(self.module.CoverageValidationError, "精确覆盖"):
                self.module.build_coverage_report(
                    identity, evidence_manifest(), route_manifest(), empty_baselines(), root=root
                )

    def test_selected_baseline_joins_formal_evidence_by_stable_id(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.make_docs(root)
            identity = self.module.validate_identity_registry(identity_registry(), root=root)
            baseline_path = root / "docs" / "baseline.json"
            write_json(
                baseline_path,
                {
                    "evidence": {
                        "affectedPages": [{"pageKey": "cpu2_01"}, {"pageKey": "cpu2_02"}],
                        "formalEvidence": [
                            {"path": "docs/one.md", "pageKeys": ["cpu2_01"]},
                            {"path": "docs/unknown.md", "pageKeys": ["cpu2_02"]},
                        ],
                    }
                },
            )
            baselines = empty_baselines()
            baselines["selectedBaselineId"] = "base"
            baselines["baselines"] = [{"id": "base", "path": "docs/baseline.json"}]
            report = self.module.build_coverage_report(
                identity, evidence_manifest(), route_manifest(), baselines, root=root
            )
            selected = report["selectedBaselineCoverage"]
            self.assertEqual("evaluated", selected["state"])
            self.assertEqual(["ev-flow-one"], selected["formalEvidenceIds"])
            self.assertEqual(["cpu2_01"], selected["coveredAffectedPageKeys"])
            self.assertEqual(["cpu2_02"], selected["missingAffectedPageKeys"])
            self.assertEqual(["docs/unknown.md"], selected["unregisteredEvidencePaths"])


if __name__ == "__main__":
    unittest.main()

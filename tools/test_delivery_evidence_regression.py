#!/usr/bin/env python3
"""交付证据退化策略校验、判定与趋势测试。"""

from __future__ import annotations

import copy
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


def make_comparison(*, state: str = "compared") -> dict:
    current_metrics = {
        "affectedPages": 3,
        "currentQueueItems": 2,
        "currentHighPriorityItems": 1,
        "missingSpecificEvidencePages": 0,
        "strictNeedsReview": 0,
        "structuredDocuments": 2,
        "unprojectedPages": 0,
    }
    comparison = {
        "schemaVersion": 1,
        "state": state,
        "current": {
            "git": {"head": "b" * 40},
            "firmwareBaseline": {"cpu2": "V2", "cpu3": "V2"},
            "status": {"id": "ready", "readyForHumanDecision": True},
            "metrics": current_metrics,
        },
        "baseline": None,
        "unmappedFirmwareSourcePaths": {"added": [], "removed": [], "persisting": []},
        "formalEvidenceTransitions": [],
    }
    if state == "compared":
        comparison["baseline"] = {
            "id": "base",
            "label": "基线",
            "gitRevision": "a" * 40,
            "firmwareBaseline": {"cpu2": "V2", "cpu3": "V2"},
            "status": {"id": "ready", "readyForHumanDecision": True},
            "metrics": copy.deepcopy(current_metrics),
        }
    return comparison


class DeliveryEvidenceRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module("check_delivery_evidence_regression")
        cls.policy = cls.module.validate_policy(
            cls.module.load_json(cls.module.DEFAULT_POLICY)
        )

    def empty_exceptions(self) -> dict:
        return {
            "schemaVersion": 1,
            "kind": "cube-delivery-regression-exceptions",
            "exceptions": [],
        }

    def test_policy_fails_closed_and_covers_all_three_severities(self) -> None:
        self.assertEqual(
            {"block", "explain", "record"},
            {rule["severity"] for rule in self.policy["rules"]},
        )
        invalid = copy.deepcopy(self.policy)
        invalid["schemaVersion"] = 2
        with self.assertRaisesRegex(self.module.RegressionPolicyError, "schemaVersion"):
            self.module.validate_policy(invalid)

        invalid = copy.deepcopy(self.policy)
        invalid["failClosed"]["unknownBaselineSchema"] = "record"
        with self.assertRaisesRegex(self.module.RegressionPolicyError, "失败关闭"):
            self.module.validate_policy(invalid)

    def test_no_baseline_is_not_evaluated_instead_of_passed(self) -> None:
        result = self.module.evaluate_comparison(
            make_comparison(state="no-baseline"),
            self.policy,
            self.empty_exceptions(),
        )
        self.assertEqual("not-evaluated", result["state"])
        self.assertEqual([], result["matches"])
        self.assertEqual(0, result["summary"]["active"]["block"])

    def test_block_explain_and_record_are_derived_from_stable_fields(self) -> None:
        comparison = make_comparison()
        comparison["current"]["metrics"]["strictNeedsReview"] = 1
        comparison["current"]["metrics"]["currentQueueItems"] = 4
        comparison["current"]["metrics"]["affectedPages"] = 5
        comparison["unmappedFirmwareSourcePaths"]["added"] = ["LTD_MAIN_CPU2/new.c"]
        comparison["formalEvidenceTransitions"] = [
            {"path": "docs/result.md", "before": "fresh", "after": "failed"}
        ]

        result = self.module.evaluate_comparison(
            comparison,
            self.policy,
            self.empty_exceptions(),
        )

        self.assertEqual("blocked", result["state"])
        self.assertEqual(2, result["summary"]["active"]["block"])
        self.assertEqual(2, result["summary"]["active"]["explain"])
        self.assertEqual(1, result["summary"]["matched"]["record"])
        self.assertIn(
            "passed-evidence-failed",
            {item["ruleId"] for item in result["matches"]},
        )

    def test_exact_revision_exception_waives_only_its_bound_rule(self) -> None:
        comparison = make_comparison()
        comparison["current"]["metrics"]["strictNeedsReview"] = 1
        exceptions = self.empty_exceptions()
        exceptions["exceptions"] = [
            {
                "id": "fixture-waiver",
                "ruleId": "strict-review-increase",
                "baselineId": "base",
                "currentGitRevision": "b" * 40,
                "action": "waive",
                "owner": "测试负责人",
                "explanation": "测试夹具验证精确绑定。",
                "evidencePaths": ["docs/result.md"],
            }
        ]

        result = self.module.evaluate_comparison(comparison, self.policy, exceptions)

        self.assertEqual("pass", result["state"])
        self.assertEqual(1, result["summary"]["waived"])
        self.assertEqual("waived", result["matches"][0]["disposition"])

        exceptions["exceptions"][0]["currentGitRevision"] = "c" * 40
        result = self.module.evaluate_comparison(comparison, self.policy, exceptions)
        self.assertEqual("blocked", result["state"])

    def test_trend_uses_registered_revisions_and_collapses_duplicates(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            evidence = {
                "firmwareBaseline": {"cpu2": "V1", "cpu3": "V1"},
                "metrics": {field: index for index, field in enumerate(self.module.TREND_METRICS)},
            }
            entries = []
            for baseline_id, state in (
                ("first", "superseded"),
                ("alias", "current"),
                ("audit", "audit-only"),
                ("retired", "retired"),
            ):
                relative_path = f"docs/00_程序流程导航/交付证据基线/{baseline_id}.json"
                write_json(root / relative_path, {"evidence": evidence})
                entries.append(
                    {
                        "id": baseline_id,
                        "label": baseline_id,
                        "path": relative_path,
                        "gitRevision": "a" * 40,
                        "lifecycle": [{
                            "state": state,
                            "owner": "测试负责人",
                            "explanationPath": "docs/decision.md",
                            "decisionRevision": "a" * 40,
                            "supersededBy": "alias" if state == "superseded" else None,
                        }],
                    }
                )
            registry = {
                "schemaVersion": 2,
                "kind": "cube-delivery-evidence-baseline-registry",
                "selectedBaselineId": "alias",
                "baselines": entries,
            }
            trend = self.module.build_baseline_trend(registry, root=root)

            self.assertEqual("insufficient", trend["state"])
            self.assertEqual(1, len(trend["points"]))
            self.assertEqual(["first", "alias"], trend["points"][0]["baselineIds"])
            self.assertEqual(["current", "superseded"], trend["method"]["lifecycleStatesIncluded"])
            self.assertTrue(trend["method"]["excludedStatesRetainedForAudit"])
            self.assertTrue(trend["method"]["datesNotInferred"])
            lifecycle = self.module.build_lifecycle_report(registry)
            self.assertEqual(4, lifecycle["summary"]["total"])
            self.assertEqual(2, lifecycle["summary"]["trendEligible"])
            self.assertEqual(1, lifecycle["summary"]["auditOnly"])
            self.assertEqual(1, lifecycle["summary"]["retired"])


if __name__ == "__main__":
    unittest.main()

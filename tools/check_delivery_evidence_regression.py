#!/usr/bin/env python3
"""校验交付证据退化策略，并按显式基线对比执行门禁。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

from check_delivery_evidence_baselines import (
    BASELINE_ID_PATTERN,
    DEFAULT_REGISTRY,
    GIT_REVISION_PATTERN,
    ROOT,
    TREND_LIFECYCLE_STATES,
    BaselineValidationError,
    build_lifecycle_report,
    check_registry,
    lifecycle_state,
    load_json,
)


DEFAULT_POLICY = ROOT / "docs" / "00_程序流程导航" / "交付证据退化策略.json"
DEFAULT_EXCEPTIONS = ROOT / "docs" / "00_程序流程导航" / "交付证据退化例外清单.json"
DEFAULT_COMPARISON = ROOT / "docs-site" / "public" / "data" / "current-delivery-comparison.json"
DEFAULT_RESULT = ROOT / "docs-site" / "public" / "data" / "current-delivery-regression.json"
DEFAULT_TREND = ROOT / "docs-site" / "public" / "data" / "delivery-baseline-trend.json"
DEFAULT_LIFECYCLE = ROOT / "docs-site" / "public" / "data" / "delivery-baseline-lifecycle.json"

POLICY_VERSION_PATTERN = re.compile(r"^[1-9][0-9]*\.[0-9]+\.[0-9]+$")
RULE_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9-]{0,79}$")
EXCEPTION_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9._-]{0,79}$")
SEVERITIES = {"block", "explain", "record"}
SOURCES = {"metric", "collection", "evidence-transition", "status-transition", "firmware-transition"}
METRIC_FIELDS = {
    "affectedPages",
    "currentQueueItems",
    "currentHighPriorityItems",
    "missingSpecificEvidencePages",
    "strictNeedsReview",
    "structuredDocuments",
    "unprojectedPages",
}
COLLECTION_FIELDS = {"unmappedFirmwareSourcePaths.added"}
EVIDENCE_STATES = {
    "fresh",
    "stale",
    "failed",
    "partial",
    "pending",
    "unrecorded",
    "invalid",
    "not-linked",
}
TREND_METRICS = (
    "affectedPages",
    "currentQueueItems",
    "currentHighPriorityItems",
    "unmappedFirmwareSources",
    "missingSpecificEvidencePages",
    "strictNeedsReview",
    "structuredDocuments",
)


class RegressionPolicyError(ValueError):
    """表示退化策略、例外或判定结果不满足契约。"""


def validate_policy(data: dict[str, Any], source: str = "交付证据退化策略") -> dict[str, Any]:
    if data.get("schemaVersion") != 1:
        raise RegressionPolicyError(f"{source} 的 schemaVersion 必须为 1")
    if data.get("kind") != "cube-delivery-regression-policy":
        raise RegressionPolicyError(f"{source} 的 kind 无效")
    policy_version = data.get("policyVersion")
    if not isinstance(policy_version, str) or not POLICY_VERSION_PATTERN.fullmatch(policy_version):
        raise RegressionPolicyError(f"{source} 的 policyVersion 无效")
    fail_closed = data.get("failClosed")
    expected_fail_closed = {
        "unknownPolicySchema": "block",
        "unknownBaselineSchema": "block",
        "unknownComparisonSchema": "block",
    }
    if fail_closed != expected_fail_closed:
        raise RegressionPolicyError(f"{source} 必须对未知 schema 全部失败关闭")
    rules = data.get("rules")
    if not isinstance(rules, list) or not rules:
        raise RegressionPolicyError(f"{source} 的 rules 必须是非空数组")

    ids: set[str] = set()
    severity_counts = {severity: 0 for severity in SEVERITIES}
    for index, rule in enumerate(rules, start=1):
        if not isinstance(rule, dict):
            raise RegressionPolicyError(f"{source} 的第 {index} 条规则必须是对象")
        rule_id = rule.get("id")
        if not isinstance(rule_id, str) or not RULE_ID_PATTERN.fullmatch(rule_id):
            raise RegressionPolicyError(f"{source} 的第 {index} 条规则 id 无效")
        if rule_id in ids:
            raise RegressionPolicyError(f"{source} 存在重复规则 id：{rule_id}")
        ids.add(rule_id)
        for field in ("label", "message"):
            if not isinstance(rule.get(field), str) or not rule[field].strip():
                raise RegressionPolicyError(f"{source} 的规则 {rule_id} 缺少 {field}")
        severity = rule.get("severity")
        if severity not in SEVERITIES:
            raise RegressionPolicyError(f"{source} 的规则 {rule_id} severity 无效")
        severity_counts[severity] += 1
        source_kind = rule.get("source")
        if source_kind not in SOURCES:
            raise RegressionPolicyError(f"{source} 的规则 {rule_id} source 无效")
        if source_kind == "metric":
            if rule.get("field") not in METRIC_FIELDS:
                raise RegressionPolicyError(f"{source} 的规则 {rule_id} metric field 无效")
            if rule.get("operator") not in {"increase", "changed"}:
                raise RegressionPolicyError(f"{source} 的规则 {rule_id} metric operator 无效")
        elif source_kind == "collection":
            if rule.get("field") not in COLLECTION_FIELDS or rule.get("operator") != "non-empty":
                raise RegressionPolicyError(f"{source} 的规则 {rule_id} collection 契约无效")
        elif source_kind == "evidence-transition":
            before_states = rule.get("beforeStates")
            after_states = rule.get("afterStates")
            if (
                not isinstance(before_states, list)
                or not before_states
                or not set(before_states) <= EVIDENCE_STATES
                or not isinstance(after_states, list)
                or not after_states
                or not set(after_states) <= EVIDENCE_STATES
            ):
                raise RegressionPolicyError(f"{source} 的规则 {rule_id} evidence states 无效")
        elif source_kind == "status-transition":
            if rule.get("operator") != "ready-to-not-ready":
                raise RegressionPolicyError(f"{source} 的规则 {rule_id} status operator 无效")
        elif source_kind == "firmware-transition":
            if rule.get("operator") != "changed":
                raise RegressionPolicyError(f"{source} 的规则 {rule_id} firmware operator 无效")
    if any(count == 0 for count in severity_counts.values()):
        raise RegressionPolicyError(f"{source} 必须同时定义 block、explain 和 record 规则")
    return data


def validate_exceptions(
    data: dict[str, Any],
    policy: dict[str, Any],
    registry: dict[str, Any],
    source: str = "交付证据退化例外清单",
) -> dict[str, Any]:
    if data.get("schemaVersion") != 1:
        raise RegressionPolicyError(f"{source} 的 schemaVersion 必须为 1")
    if data.get("kind") != "cube-delivery-regression-exceptions":
        raise RegressionPolicyError(f"{source} 的 kind 无效")
    entries = data.get("exceptions")
    if not isinstance(entries, list):
        raise RegressionPolicyError(f"{source} 的 exceptions 必须是数组")
    rules = {item["id"]: item for item in policy["rules"]}
    baseline_ids = {item["id"] for item in registry["baselines"]}
    ids: set[str] = set()
    bindings: set[tuple[str, str, str]] = set()
    for index, entry in enumerate(entries, start=1):
        if not isinstance(entry, dict):
            raise RegressionPolicyError(f"{source} 的第 {index} 条记录必须是对象")
        exception_id = entry.get("id")
        if not isinstance(exception_id, str) or not EXCEPTION_ID_PATTERN.fullmatch(exception_id):
            raise RegressionPolicyError(f"{source} 的第 {index} 条记录 id 无效")
        if exception_id in ids:
            raise RegressionPolicyError(f"{source} 存在重复 id：{exception_id}")
        ids.add(exception_id)
        rule_id = entry.get("ruleId")
        if rule_id not in rules:
            raise RegressionPolicyError(f"{source} 的记录 {exception_id} ruleId 无效")
        baseline_id = entry.get("baselineId")
        if baseline_id not in baseline_ids:
            raise RegressionPolicyError(f"{source} 的记录 {exception_id} baselineId 无效")
        revision = entry.get("currentGitRevision")
        if not isinstance(revision, str) or not GIT_REVISION_PATTERN.fullmatch(revision):
            raise RegressionPolicyError(f"{source} 的记录 {exception_id} currentGitRevision 无效")
        expected_action = "waive" if rules[rule_id]["severity"] == "block" else "acknowledge"
        if rules[rule_id]["severity"] == "record" or entry.get("action") != expected_action:
            raise RegressionPolicyError(f"{source} 的记录 {exception_id} action 与规则等级不匹配")
        for field in ("owner", "explanation"):
            if not isinstance(entry.get(field), str) or not entry[field].strip():
                raise RegressionPolicyError(f"{source} 的记录 {exception_id} 缺少 {field}")
        evidence_paths = entry.get("evidencePaths")
        if (
            not isinstance(evidence_paths, list)
            or not evidence_paths
            or any(
                not isinstance(item, str)
                or not item.startswith("docs/")
                or ".." in Path(item).parts
                for item in evidence_paths
            )
        ):
            raise RegressionPolicyError(f"{source} 的记录 {exception_id} evidencePaths 无效")
        binding = (rule_id, baseline_id, revision)
        if binding in bindings:
            raise RegressionPolicyError(f"{source} 存在重复规则、基线和 revision 绑定：{exception_id}")
        bindings.add(binding)
    return data


def _nested_value(data: dict[str, Any], dotted_path: str) -> Any:
    value: Any = data
    for part in dotted_path.split("."):
        if not isinstance(value, dict) or part not in value:
            raise RegressionPolicyError(f"当前对比缺少策略字段：{dotted_path}")
        value = value[part]
    return value


def _exception_summary(entry: dict[str, Any]) -> dict[str, Any]:
    return {
        "id": entry["id"],
        "action": entry["action"],
        "owner": entry["owner"],
        "explanation": entry["explanation"],
        "evidencePaths": entry["evidencePaths"],
    }


def evaluate_comparison(
    comparison: dict[str, Any],
    policy: dict[str, Any],
    exceptions: dict[str, Any],
) -> dict[str, Any]:
    validate_policy(policy)
    if comparison.get("schemaVersion") != 1:
        raise RegressionPolicyError("当前交付对比的 schemaVersion 必须为 1")
    comparison_state = comparison.get("state")
    if comparison_state not in {"no-baseline", "compared"}:
        raise RegressionPolicyError("当前交付对比的 state 无效")
    current = comparison.get("current")
    if not isinstance(current, dict) or not isinstance(current.get("git"), dict):
        raise RegressionPolicyError("当前交付对比缺少 current.git")
    revision = current["git"].get("head")
    if not isinstance(revision, str) or not GIT_REVISION_PATTERN.fullmatch(revision):
        raise RegressionPolicyError("当前交付对比缺少完整 Git revision")
    empty_summary = {
        "matched": {"block": 0, "explain": 0, "record": 0},
        "active": {"block": 0, "explain": 0},
        "waived": 0,
        "acknowledged": 0,
    }
    method = {
        "explicitBaselineOnly": True,
        "policyFieldsOnly": True,
        "unknownSchemaFailsClosed": True,
        "quantityIsNotQuality": True,
        "manualDeliveryDecisionRequired": True,
    }
    if comparison_state == "no-baseline":
        return {
            "schemaVersion": 1,
            "kind": "cube-delivery-regression-evaluation",
            "policyVersion": policy["policyVersion"],
            "state": "not-evaluated",
            "comparisonState": comparison_state,
            "baseline": None,
            "currentGitRevision": revision,
            "summary": empty_summary,
            "matches": [],
            "method": method,
        }

    baseline = comparison.get("baseline")
    if not isinstance(baseline, dict) or not isinstance(baseline.get("id"), str):
        raise RegressionPolicyError("当前交付对比缺少显式 baseline")
    if not isinstance(baseline.get("metrics"), dict):
        raise RegressionPolicyError("当前交付对比缺少 baseline.metrics")
    if not isinstance(current.get("metrics"), dict):
        raise RegressionPolicyError("当前交付对比缺少 current.metrics")
    exception_by_binding = {
        (item["ruleId"], item["baselineId"], item["currentGitRevision"]): item
        for item in exceptions["exceptions"]
    }
    matches: list[dict[str, Any]] = []
    for rule in policy["rules"]:
        facts: dict[str, Any] | None = None
        source = rule["source"]
        if source == "metric":
            field = rule["field"]
            baseline_value = baseline["metrics"].get(field)
            current_value = current["metrics"].get(field)
            if not isinstance(baseline_value, (int, float)) or not isinstance(current_value, (int, float)):
                raise RegressionPolicyError(f"当前交付对比的指标 {field} 必须是数字")
            delta = current_value - baseline_value
            matched = delta > 0 if rule["operator"] == "increase" else delta != 0
            if matched:
                facts = {"field": field, "baseline": baseline_value, "current": current_value, "delta": delta}
        elif source == "collection":
            items = _nested_value(comparison, rule["field"])
            if not isinstance(items, list):
                raise RegressionPolicyError(f"当前交付对比的集合 {rule['field']} 必须是数组")
            if items:
                facts = {"field": rule["field"], "count": len(items), "items": items}
        elif source == "evidence-transition":
            transitions = comparison.get("formalEvidenceTransitions")
            if not isinstance(transitions, list):
                raise RegressionPolicyError("当前交付对比缺少 formalEvidenceTransitions")
            items = [
                {"path": item.get("path"), "before": item.get("before"), "after": item.get("after")}
                for item in transitions
                if item.get("before") in rule["beforeStates"] and item.get("after") in rule["afterStates"]
            ]
            if items:
                facts = {"count": len(items), "items": items}
        elif source == "status-transition":
            before = baseline.get("status")
            after = current.get("status")
            if not isinstance(before, dict) or not isinstance(after, dict):
                raise RegressionPolicyError("当前交付对比缺少状态对象")
            if before.get("readyForHumanDecision") is True and after.get("readyForHumanDecision") is False:
                facts = {"before": before, "after": after}
        elif source == "firmware-transition":
            before = baseline.get("firmwareBaseline")
            after = current.get("firmwareBaseline")
            if not isinstance(before, dict) or not isinstance(after, dict):
                raise RegressionPolicyError("当前交付对比缺少固件基线对象")
            if before != after:
                facts = {"before": before, "after": after}
        if facts is None:
            continue

        exception = exception_by_binding.get((rule["id"], baseline["id"], revision))
        disposition = "active"
        if exception is not None:
            disposition = "waived" if exception["action"] == "waive" else "acknowledged"
        matches.append(
            {
                "ruleId": rule["id"],
                "label": rule["label"],
                "severity": rule["severity"],
                "message": rule["message"],
                "facts": facts,
                "disposition": disposition,
                "exception": _exception_summary(exception) if exception is not None else None,
            }
        )

    summary = {
        "matched": {
            severity: sum(item["severity"] == severity for item in matches)
            for severity in ("block", "explain", "record")
        },
        "active": {
            "block": sum(item["severity"] == "block" and item["disposition"] == "active" for item in matches),
            "explain": sum(item["severity"] == "explain" and item["disposition"] == "active" for item in matches),
        },
        "waived": sum(item["disposition"] == "waived" for item in matches),
        "acknowledged": sum(item["disposition"] == "acknowledged" for item in matches),
    }
    state = "blocked" if summary["active"]["block"] else (
        "explanation-required" if summary["active"]["explain"] else "pass"
    )
    return {
        "schemaVersion": 1,
        "kind": "cube-delivery-regression-evaluation",
        "policyVersion": policy["policyVersion"],
        "state": state,
        "comparisonState": comparison_state,
        "baseline": {
            "id": baseline["id"],
            "label": baseline.get("label"),
            "gitRevision": baseline.get("gitRevision"),
        },
        "currentGitRevision": revision,
        "summary": summary,
        "matches": matches,
        "method": method,
    }


def build_baseline_trend(registry: dict[str, Any], *, root: Path = ROOT) -> dict[str, Any]:
    points_by_revision: dict[str, dict[str, Any]] = {}
    for entry in registry["baselines"]:
        if lifecycle_state(entry)["state"] not in TREND_LIFECYCLE_STATES:
            continue
        envelope = load_json(root / Path(entry["path"]))
        evidence = envelope["evidence"]
        revision = entry["gitRevision"]
        metrics = {
            field: evidence["metrics"].get(field, 0)
            for field in TREND_METRICS
        }
        if any(not isinstance(value, (int, float)) for value in metrics.values()):
            raise RegressionPolicyError(f"基线 {entry['id']} 的趋势指标必须是数字")
        point = points_by_revision.get(revision)
        if point is None:
            points_by_revision[revision] = {
                "sequence": len(points_by_revision) + 1,
                "gitRevision": revision,
                "shortRevision": revision[:8],
                "baselineIds": [entry["id"]],
                "labels": [entry["label"]],
                "firmwareBaseline": evidence["firmwareBaseline"],
                "metrics": metrics,
            }
        else:
            if point["firmwareBaseline"] != evidence["firmwareBaseline"] or point["metrics"] != metrics:
                raise RegressionPolicyError(f"同一 Git revision 的基线事实不一致：{revision}")
            point["baselineIds"].append(entry["id"])
            point["labels"].append(entry["label"])
    points = list(points_by_revision.values())
    state = "empty" if not points else ("insufficient" if len(points) == 1 else "available")
    return {
        "schemaVersion": 1,
        "kind": "cube-delivery-baseline-trend",
        "state": state,
        "metricIds": list(TREND_METRICS),
        "points": points,
        "method": {
            "registeredBaselinesOnly": True,
            "lifecycleStatesIncluded": sorted(TREND_LIFECYCLE_STATES),
            "excludedStatesRetainedForAudit": True,
            "duplicateRevisionsCollapsed": True,
            "datesNotInferred": True,
            "noInterpolation": True,
            "currentDirtyWorktreeExcluded": True,
            "quantityIsNotQuality": True,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
    parser.add_argument("--exceptions", type=Path, default=DEFAULT_EXCEPTIONS)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--comparison", type=Path)
    parser.add_argument("--result", type=Path)
    parser.add_argument("--trend", type=Path)
    parser.add_argument("--lifecycle", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        policy = validate_policy(load_json(args.policy), str(args.policy))
        registry = check_registry(args.registry, root=ROOT)
        exceptions = validate_exceptions(
            load_json(args.exceptions), policy, registry, str(args.exceptions)
        )
        trend = build_baseline_trend(registry)
        lifecycle = build_lifecycle_report(registry)
        if args.trend is not None:
            actual_trend = load_json(args.trend)
            if actual_trend != trend:
                raise RegressionPolicyError(f"站点趋势结果与正式基线不一致：{args.trend}")
        if args.lifecycle is not None:
            actual_lifecycle = load_json(args.lifecycle)
            if actual_lifecycle != lifecycle:
                raise RegressionPolicyError(f"站点生命周期结果与正式基线不一致：{args.lifecycle}")
        if args.comparison is None:
            print(
                f"交付退化策略检查通过：策略 {policy['policyVersion']}，"
                f"{len(policy['rules'])} 条规则，{len(exceptions['exceptions'])} 条例外，"
                f"{len(trend['points'])} 个真实趋势点"
            )
            return 0
        comparison = load_json(args.comparison)
        result = evaluate_comparison(comparison, policy, exceptions)
        if args.result is not None:
            actual_result = load_json(args.result)
            if actual_result != result:
                raise RegressionPolicyError(f"站点退化门禁结果与正式策略不一致：{args.result}")
    except (BaselineValidationError, RegressionPolicyError) as exc:
        print(f"交付退化策略检查失败：{exc}", file=sys.stderr)
        return 1

    print(
        f"交付退化门禁：{result['state']}；"
        f"阻断 {result['summary']['active']['block']}，"
        f"待解释 {result['summary']['active']['explain']}，"
        f"仅记录 {result['summary']['matched']['record']}"
    )
    return 2 if result["state"] == "blocked" else 0


if __name__ == "__main__":
    raise SystemExit(main())

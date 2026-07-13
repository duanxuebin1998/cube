#!/usr/bin/env python3
"""校验验证批次范围、追加式生命周期、运行覆盖和里程碑关闭决策。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import date, datetime
from pathlib import Path
from typing import Any

from check_delivery_evidence_coverage import (
    DEFAULT_IDENTITY_REGISTRY,
    ROOT,
    load_json,
    validate_identity_registry,
)
from check_validation_runs import (
    ALLOWED_OUTCOMES,
    DEFAULT_RUN_REGISTRY,
    build_validation_run_report,
    load_work_package_cases,
)


DEFAULT_BATCH_REGISTRY = ROOT / "docs" / "00_程序流程导航" / "验证批次登记清单.json"
DEFAULT_SNAPSHOT = ROOT / "docs-site" / "public" / "data" / "validation-batches.json"
BATCH_ID_PATTERN = re.compile(r"batch-[a-z0-9][a-z0-9-]{2,}")
REVISION_PATTERN = re.compile(r"[0-9a-f]{40}")
VERSION_PATTERN = re.compile(r"V\d+\.\d+\.\d+\.\d+")
MILESTONE_KINDS = (
    "version",
    "bench-campaign",
    "device-campaign",
    "field-campaign",
    "release-candidate",
)
BATCH_STATES = (
    "planned",
    "running",
    "waiting-external",
    "decision-ready",
    "closed",
    "cancelled",
)
ACTIVE_STATES = {"planned", "running", "waiting-external", "decision-ready"}
TRANSITIONS = {
    "planned": {"running", "cancelled"},
    "running": {"waiting-external", "decision-ready", "cancelled"},
    "waiting-external": {"running", "cancelled"},
    "decision-ready": {"running", "closed"},
    "closed": set(),
    "cancelled": set(),
}
DECISIONS = ("accepted", "rejected", "accepted-with-exceptions")


class ValidationBatchError(ValueError):
    """表示验证批次不满足范围冻结或里程碑决策契约。"""


def _text(value: Any, field: str, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValidationBatchError(f"{context} 的 {field} 必须是非空字符串")
    return value.strip()


def _timestamp(value: Any, field: str, context: str) -> datetime:
    text = _text(value, field, context)
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError as exc:
        raise ValidationBatchError(f"{context} 的 {field} 不是有效 ISO 8601 时间") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValidationBatchError(f"{context} 的 {field} 必须包含时区")
    return parsed


def _day(value: Any, field: str, context: str) -> date:
    text = _text(value, field, context)
    try:
        parsed = date.fromisoformat(text)
    except ValueError as exc:
        raise ValidationBatchError(f"{context} 的 {field} 必须是有效日期") from exc
    if parsed.isoformat() != text:
        raise ValidationBatchError(f"{context} 的 {field} 必须使用 YYYY-MM-DD")
    return parsed


def _docs_path(value: Any, field: str, context: str, root: Path) -> str:
    relative = _text(value, field, context).replace("\\", "/").removeprefix("./")
    if not relative.startswith("docs/") or relative.lower().endswith(".pdf"):
        raise ValidationBatchError(f"{context} 的 {field} 必须指向 docs/ 下的非 PDF 文件")
    candidate = (root / Path(relative)).resolve()
    docs_root = (root / "docs").resolve()
    if docs_root not in candidate.parents or not candidate.is_file():
        raise ValidationBatchError(f"{context} 的 {field} 不存在或路径越界：{relative}")
    return relative


def _expected_baseline(
    case_refs: list[dict[str, str]], packages: dict[str, dict[str, Any]]
) -> dict[str, str]:
    expected: dict[str, str] = {}
    for case_ref in case_refs:
        package = packages[case_ref["evidenceId"]]
        for cpu, version in package["firmwareBaseline"].items():
            previous = expected.get(cpu)
            if previous is not None and previous != version:
                raise ValidationBatchError(
                    f"批次范围包含互相冲突的 {cpu} 工作包基线：{previous} / {version}"
                )
            expected[cpu] = version
    return expected


def build_validation_batch_report(
    registry: dict[str, Any],
    packages: dict[str, dict[str, Any]],
    runs: list[dict[str, Any]],
    *,
    root: Path = ROOT,
) -> dict[str, Any]:
    """从冻结范围和追加式事件构建稳定批次报告。"""

    if registry.get("schemaVersion") != 1:
        raise ValidationBatchError("验证批次登记清单 schemaVersion 必须为 1")
    if registry.get("kind") != "cube-validation-batch-registry":
        raise ValidationBatchError("验证批次登记清单 kind 无效")
    if not isinstance(registry.get("description"), str) or not registry["description"].strip():
        raise ValidationBatchError("验证批次登记清单 description 必须是非空字符串")
    raw_batches = registry.get("batches")
    if not isinstance(raw_batches, list):
        raise ValidationBatchError("验证批次登记清单 batches 必须是数组")

    run_by_id = {run["id"]: run for run in runs}
    latest_run_by_case: dict[tuple[str, str], dict[str, Any]] = {}
    for run in runs:
        latest_run_by_case[(run["evidenceId"], run["caseId"])] = run

    batch_ids: set[str] = set()
    batch_reports: list[dict[str, Any]] = []
    for batch_index, raw in enumerate(raw_batches):
        if not isinstance(raw, dict):
            raise ValidationBatchError(f"第 {batch_index + 1} 个批次必须是对象")
        context = f"第 {batch_index + 1} 个批次"
        batch_id = _text(raw.get("id"), "id", context)
        if BATCH_ID_PATTERN.fullmatch(batch_id) is None:
            raise ValidationBatchError(f"批次 ID 无效：{batch_id}")
        if batch_id in batch_ids:
            raise ValidationBatchError(f"批次 ID 重复：{batch_id}")
        batch_ids.add(batch_id)
        label = _text(raw.get("label"), "label", batch_id)
        owner = _text(raw.get("owner"), "owner", batch_id)
        created_at = _timestamp(raw.get("createdAt"), "createdAt", batch_id)
        scope_revision = _text(raw.get("scopeRevision"), "scopeRevision", batch_id)
        if REVISION_PATTERN.fullmatch(scope_revision) is None:
            raise ValidationBatchError(f"{batch_id} 的 scopeRevision 必须是 40 位小写 Git revision")

        milestone = raw.get("milestone")
        if not isinstance(milestone, dict):
            raise ValidationBatchError(f"{batch_id} 的 milestone 必须是对象")
        milestone_kind = _text(milestone.get("kind"), "milestone.kind", batch_id)
        if milestone_kind not in MILESTONE_KINDS:
            raise ValidationBatchError(f"{batch_id} 的 milestone.kind 无效：{milestone_kind}")
        milestone_reference = _text(
            milestone.get("reference"), "milestone.reference", batch_id
        )

        window = raw.get("window")
        if not isinstance(window, dict):
            raise ValidationBatchError(f"{batch_id} 的 window 必须是对象")
        window_start = _day(window.get("start"), "window.start", batch_id)
        window_end = _day(window.get("end"), "window.end", batch_id)
        if window_end < window_start:
            raise ValidationBatchError(f"{batch_id} 的 window.end 不能早于 window.start")

        raw_case_refs = raw.get("caseRefs")
        if not isinstance(raw_case_refs, list) or not raw_case_refs:
            raise ValidationBatchError(f"{batch_id} 必须至少冻结一个工作包用例")
        case_refs: list[dict[str, str]] = []
        case_keys: set[tuple[str, str]] = set()
        for raw_case in raw_case_refs:
            if not isinstance(raw_case, dict):
                raise ValidationBatchError(f"{batch_id} 的 caseRefs 项必须是对象")
            evidence_id = _text(raw_case.get("evidenceId"), "caseRefs.evidenceId", batch_id)
            case_id = _text(raw_case.get("caseId"), "caseRefs.caseId", batch_id)
            package = packages.get(evidence_id)
            if package is None:
                raise ValidationBatchError(f"{batch_id} 引用了未知工作包：{evidence_id}")
            if case_id not in package["cases"]:
                raise ValidationBatchError(f"{batch_id} 引用了工作包中不存在的用例：{case_id}")
            key = (evidence_id, case_id)
            if key in case_keys:
                raise ValidationBatchError(f"{batch_id} 重复冻结用例：{evidence_id}:{case_id}")
            case_keys.add(key)
            case_refs.append({"evidenceId": evidence_id, "caseId": case_id})

        baseline = raw.get("firmwareBaseline")
        if not isinstance(baseline, dict) or not baseline:
            raise ValidationBatchError(f"{batch_id} 的 firmwareBaseline 必须是非空对象")
        if set(baseline) - {"cpu2", "cpu3"}:
            raise ValidationBatchError(f"{batch_id} 的 firmwareBaseline 含未知 CPU 字段")
        for cpu, version in baseline.items():
            if not isinstance(version, str) or VERSION_PATTERN.fullmatch(version) is None:
                raise ValidationBatchError(f"{batch_id} 的 firmwareBaseline.{cpu} 格式无效")
        expected_baseline = _expected_baseline(case_refs, packages)
        if baseline != expected_baseline:
            raise ValidationBatchError(
                f"{batch_id} 的 firmwareBaseline 与冻结工作包范围不一致："
                f"登记 {baseline}，期望 {expected_baseline}"
            )

        criteria = raw.get("exitCriteria")
        if not isinstance(criteria, list) or not criteria:
            raise ValidationBatchError(f"{batch_id} 必须至少声明一条退出条件")
        normalized_criteria = [
            _text(value, "exitCriteria", batch_id) for value in criteria
        ]
        if len(set(normalized_criteria)) != len(normalized_criteria):
            raise ValidationBatchError(f"{batch_id} 的退出条件重复")

        raw_events = raw.get("events")
        if not isinstance(raw_events, list) or not raw_events:
            raise ValidationBatchError(f"{batch_id} 必须包含初始 planned 事件")
        events: list[dict[str, Any]] = []
        previous_state: str | None = None
        previous_time: datetime | None = None
        for event_index, raw_event in enumerate(raw_events):
            if not isinstance(raw_event, dict):
                raise ValidationBatchError(f"{batch_id} 第 {event_index + 1} 个事件必须是对象")
            event_context = f"{batch_id} 第 {event_index + 1} 个事件"
            state = _text(raw_event.get("state"), "state", event_context)
            if state not in BATCH_STATES:
                raise ValidationBatchError(f"{event_context} 的 state 无效：{state}")
            if event_index == 0 and state != "planned":
                raise ValidationBatchError(f"{batch_id} 首个事件必须是 planned")
            if previous_state is not None and state not in TRANSITIONS[previous_state]:
                raise ValidationBatchError(
                    f"{batch_id} 不允许从 {previous_state} 转换到 {state}"
                )
            recorded_at = _timestamp(raw_event.get("recordedAt"), "recordedAt", event_context)
            if recorded_at < created_at:
                raise ValidationBatchError(f"{event_context} 不能早于批次 createdAt")
            if previous_time is not None and recorded_at < previous_time:
                raise ValidationBatchError(f"{batch_id} 的事件时间必须按追加顺序非递减")
            event_owner = _text(raw_event.get("owner"), "owner", event_context)
            event_revision = _text(
                raw_event.get("decisionRevision"), "decisionRevision", event_context
            )
            if REVISION_PATTERN.fullmatch(event_revision) is None:
                raise ValidationBatchError(f"{event_context} 的 decisionRevision 格式无效")
            explanation_path = _docs_path(
                raw_event.get("explanationPath"), "explanationPath", event_context, root
            )
            raw_run_ids = raw_event.get("relatedRunIds")
            if not isinstance(raw_run_ids, list):
                raise ValidationBatchError(f"{event_context} 的 relatedRunIds 必须是数组")
            related_run_ids = [
                _text(value, "relatedRunIds", event_context) for value in raw_run_ids
            ]
            if len(set(related_run_ids)) != len(related_run_ids):
                raise ValidationBatchError(f"{event_context} 的 relatedRunIds 重复")
            event_runs: list[dict[str, Any]] = []
            event_case_keys: set[tuple[str, str]] = set()
            for run_id in related_run_ids:
                run = run_by_id.get(run_id)
                if run is None:
                    raise ValidationBatchError(f"{event_context} 引用了未知运行：{run_id}")
                key = (run["evidenceId"], run["caseId"])
                if key not in case_keys:
                    raise ValidationBatchError(f"{event_context} 引用了批次范围外运行：{run_id}")
                if key in event_case_keys:
                    raise ValidationBatchError(
                        f"{event_context} 对同一用例引用了多次运行：{run_id}"
                    )
                if datetime.fromisoformat(run["executedAt"]) > recorded_at:
                    raise ValidationBatchError(f"{event_context} 引用了事件发生后的运行：{run_id}")
                event_case_keys.add(key)
                event_runs.append(run)
            if state == "planned" and related_run_ids:
                raise ValidationBatchError(f"{batch_id} 的 planned 事件不能引用运行")
            if state in {"decision-ready", "closed"}:
                if event_case_keys != case_keys:
                    raise ValidationBatchError(f"{event_context} 必须为每个冻结用例各引用一次运行")
                unresolved = [
                    run["id"] for run in event_runs if run["outcome"] in {"partial", "blocked"}
                ]
                if unresolved:
                    raise ValidationBatchError(
                        f"{event_context} 仍含未决运行：{', '.join(unresolved)}"
                    )

            decision = raw_event.get("decision")
            raw_exception_paths = raw_event.get("exceptionPaths", [])
            if not isinstance(raw_exception_paths, list):
                raise ValidationBatchError(f"{event_context} 的 exceptionPaths 必须是数组")
            exception_paths = [
                _docs_path(value, "exceptionPaths", event_context, root)
                for value in raw_exception_paths
            ]
            if state != "closed":
                if decision is not None or exception_paths:
                    raise ValidationBatchError(f"{event_context} 只有 closed 事件可声明决策或例外")
            else:
                decision = _text(decision, "decision", event_context)
                if decision not in DECISIONS:
                    raise ValidationBatchError(f"{event_context} 的 decision 无效：{decision}")
                outcomes = [run["outcome"] for run in event_runs]
                if decision == "accepted" and any(outcome != "passed" for outcome in outcomes):
                    raise ValidationBatchError(f"{event_context} 只有全部通过才能 accepted")
                if decision == "rejected" and "failed" not in outcomes:
                    raise ValidationBatchError(f"{event_context} rejected 必须至少包含一次 failed")
                if decision == "accepted-with-exceptions":
                    if all(outcome == "passed" for outcome in outcomes) or not exception_paths:
                        raise ValidationBatchError(
                            f"{event_context} 带例外接受必须含非通过结果和正式例外说明"
                        )

            events.append(
                {
                    "state": state,
                    "recordedAt": raw_event["recordedAt"].strip(),
                    "owner": event_owner,
                    "decisionRevision": event_revision,
                    "explanationPath": explanation_path,
                    "relatedRunIds": related_run_ids,
                    **({"decision": decision} if decision is not None else {}),
                    **({"exceptionPaths": exception_paths} if exception_paths else {}),
                }
            )
            previous_state = state
            previous_time = recorded_at

        case_reports: list[dict[str, Any]] = []
        for case_ref in case_refs:
            key = (case_ref["evidenceId"], case_ref["caseId"])
            package = packages[case_ref["evidenceId"]]
            latest = latest_run_by_case.get(key)
            case_reports.append(
                {
                    **case_ref,
                    "layer": package["cases"][case_ref["caseId"]]["layer"],
                    "currentOutcome": latest["outcome"] if latest else "pending",
                    "latestRunId": latest["id"] if latest else None,
                    "runCount": sum(
                        run["evidenceId"] == key[0] and run["caseId"] == key[1]
                        for run in runs
                    ),
                }
            )
        cases_with_runs = sum(case["runCount"] > 0 for case in case_reports)
        unresolved_cases = sum(
            case["currentOutcome"] in {"pending", "partial", "blocked"}
            for case in case_reports
        )
        current_outcomes = {
            outcome: sum(case["currentOutcome"] == outcome for case in case_reports)
            for outcome in ("pending", *ALLOWED_OUTCOMES)
        }
        latest_event = events[-1]
        batch_reports.append(
            {
                "id": batch_id,
                "label": label,
                "milestone": {"kind": milestone_kind, "reference": milestone_reference},
                "owner": owner,
                "createdAt": raw["createdAt"].strip(),
                "scopeRevision": scope_revision,
                "firmwareBaseline": baseline,
                "window": {"start": window_start.isoformat(), "end": window_end.isoformat()},
                "exitCriteria": normalized_criteria,
                "state": latest_event["state"],
                "decision": latest_event.get("decision"),
                "plannedCases": len(case_reports),
                "casesWithRuns": cases_with_runs,
                "pendingCases": len(case_reports) - cases_with_runs,
                "unresolvedCases": unresolved_cases,
                "decisionInputComplete": cases_with_runs == len(case_reports)
                and unresolved_cases == 0,
                "currentOutcomes": current_outcomes,
                "cases": case_reports,
                "events": events,
            }
        )

    state_counts = {
        state: sum(batch["state"] == state for batch in batch_reports)
        for state in BATCH_STATES
    }
    milestone_counts = {
        kind: sum(batch["milestone"]["kind"] == kind for batch in batch_reports)
        for kind in MILESTONE_KINDS
    }
    referenced_run_ids = {
        run_id
        for batch in batch_reports
        for event in batch["events"]
        for run_id in event["relatedRunIds"]
    }
    return {
        "schemaVersion": 1,
        "kind": "cube-validation-batches",
        "state": "empty" if not batch_reports else "batches-recorded",
        "summary": {
            "batches": len(batch_reports),
            "active": sum(batch["state"] in ACTIVE_STATES for batch in batch_reports),
            "closed": state_counts["closed"],
            "cancelled": state_counts["cancelled"],
            "plannedCases": sum(batch["plannedCases"] for batch in batch_reports),
            "casesWithRuns": sum(batch["casesWithRuns"] for batch in batch_reports),
            "decisionReadyInputs": sum(
                batch["decisionInputComplete"] for batch in batch_reports
            ),
            "referencedRuns": len(referenced_run_ids),
            "states": state_counts,
            "milestones": milestone_counts,
        },
        "batches": batch_reports,
        "method": {
            "scopeUsesStableWorkPackageCases": True,
            "runsReferencedByStableIdentity": True,
            "scopeAndBaselineFrozen": True,
            "eventsAreAppendOnly": True,
            "decisionUsesExplicitRunSet": True,
            "acceptedRequiresAllPassed": True,
            "emptyRegistryIsNotComplete": True,
            "manualDeliveryDecisionRequired": True,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_BATCH_REGISTRY)
    parser.add_argument("--run-registry", type=Path, default=DEFAULT_RUN_REGISTRY)
    parser.add_argument("--identity-registry", type=Path, default=DEFAULT_IDENTITY_REGISTRY)
    parser.add_argument("--snapshot", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        identity = validate_identity_registry(load_json(args.identity_registry), root=ROOT)
        packages = load_work_package_cases(identity)
        run_report = build_validation_run_report(load_json(args.run_registry), packages)
        report = build_validation_batch_report(
            load_json(args.registry), packages, run_report["runs"]
        )
        if args.snapshot is not None and load_json(args.snapshot) != report:
            raise ValidationBatchError(f"站点验证批次快照与正式资料不一致：{args.snapshot}")
    except (ValidationBatchError, ValueError, KeyError, OSError, json.JSONDecodeError) as exc:
        print(f"验证批次检查失败：{exc}", file=sys.stderr)
        return 1
    print(
        "验证批次检查通过："
        f"{report['summary']['batches']} 个批次，"
        f"{report['summary']['active']} 个活动批次，"
        f"{report['summary']['closed']} 个已关闭批次。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

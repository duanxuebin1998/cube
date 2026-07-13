#!/usr/bin/env python3
"""创建验证批次或向已有批次追加生命周期事件。"""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path

from check_delivery_evidence_coverage import (
    DEFAULT_IDENTITY_REGISTRY,
    ROOT,
    load_json,
    validate_identity_registry,
)
from check_validation_batches import (
    BATCH_STATES,
    DECISIONS,
    DEFAULT_BATCH_REGISTRY,
    MILESTONE_KINDS,
    build_validation_batch_report,
)
from check_validation_runs import (
    DEFAULT_RUN_REGISTRY,
    build_validation_run_report,
    load_work_package_cases,
)


def _case_ref(value: str) -> dict[str, str]:
    evidence_id, separator, case_id = value.partition(":")
    if not separator or not evidence_id or not case_id:
        raise argparse.ArgumentTypeError("--case 必须使用 <evidenceId>:<caseId>")
    return {"evidenceId": evidence_id, "caseId": case_id}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_BATCH_REGISTRY)
    parser.add_argument("--run-registry", type=Path, default=DEFAULT_RUN_REGISTRY)
    parser.add_argument("--identity-registry", type=Path, default=DEFAULT_IDENTITY_REGISTRY)
    subparsers = parser.add_subparsers(dest="command", required=True)

    create = subparsers.add_parser("create", help="创建带初始 planned 事件的冻结批次")
    create.add_argument("--id", required=True)
    create.add_argument("--label", required=True)
    create.add_argument("--milestone-kind", required=True, choices=MILESTONE_KINDS)
    create.add_argument("--milestone-reference", required=True)
    create.add_argument("--owner", required=True)
    create.add_argument("--created-at", required=True)
    create.add_argument("--scope-revision", required=True)
    create.add_argument("--cpu2")
    create.add_argument("--cpu3")
    create.add_argument("--window-start", required=True)
    create.add_argument("--window-end", required=True)
    create.add_argument("--case", action="append", required=True, type=_case_ref)
    create.add_argument("--exit-criterion", action="append", required=True)
    create.add_argument("--explanation", required=True)
    create.add_argument("--decision-revision", required=True)
    create.add_argument("--dry-run", action="store_true")

    transition = subparsers.add_parser("transition", help="追加批次状态事件")
    transition.add_argument("--id", required=True)
    transition.add_argument(
        "--state", required=True, choices=tuple(state for state in BATCH_STATES if state != "planned")
    )
    transition.add_argument("--recorded-at", required=True)
    transition.add_argument("--owner", required=True)
    transition.add_argument("--decision-revision", required=True)
    transition.add_argument("--explanation", required=True)
    transition.add_argument("--run-id", action="append", default=[])
    transition.add_argument("--decision", choices=DECISIONS)
    transition.add_argument("--exception", action="append", default=[])
    transition.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _atomic_write_json(target: Path, payload: dict[str, object]) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{target.name}.", suffix=".tmp", dir=target.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(payload, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, target)
    finally:
        if temporary.exists():
            temporary.unlink()


def _candidate_registry(args: argparse.Namespace, registry: dict[str, object]) -> dict[str, object]:
    batches = list(registry.get("batches", []))
    if args.command == "create":
        baseline = {
            cpu: version
            for cpu, version in (("cpu2", args.cpu2), ("cpu3", args.cpu3))
            if version
        }
        batch = {
            "id": args.id,
            "label": args.label,
            "milestone": {
                "kind": args.milestone_kind,
                "reference": args.milestone_reference,
            },
            "owner": args.owner,
            "createdAt": args.created_at,
            "scopeRevision": args.scope_revision,
            "firmwareBaseline": baseline,
            "window": {"start": args.window_start, "end": args.window_end},
            "caseRefs": args.case,
            "exitCriteria": args.exit_criterion,
            "events": [{
                "state": "planned",
                "recordedAt": args.created_at,
                "owner": args.owner,
                "decisionRevision": args.decision_revision,
                "explanationPath": args.explanation,
                "relatedRunIds": [],
            }],
        }
        return {**registry, "batches": [*batches, batch]}

    matching = [index for index, batch in enumerate(batches) if batch.get("id") == args.id]
    if len(matching) != 1:
        raise ValueError(f"必须且只能找到一个批次：{args.id}")
    index = matching[0]
    batch = dict(batches[index])
    event = {
        "state": args.state,
        "recordedAt": args.recorded_at,
        "owner": args.owner,
        "decisionRevision": args.decision_revision,
        "explanationPath": args.explanation,
        "relatedRunIds": args.run_id,
    }
    if args.decision is not None:
        event["decision"] = args.decision
    if args.exception:
        event["exceptionPaths"] = args.exception
    batch["events"] = [*batch.get("events", []), event]
    batches[index] = batch
    return {**registry, "batches": batches}


def main() -> int:
    args = parse_args()
    identity = validate_identity_registry(load_json(args.identity_registry), root=ROOT)
    packages = load_work_package_cases(identity)
    run_report = build_validation_run_report(load_json(args.run_registry), packages)
    registry = load_json(args.registry)
    candidate = _candidate_registry(args, registry)
    report = build_validation_batch_report(candidate, packages, run_report["runs"])
    changed_batch = next(batch for batch in report["batches"] if batch["id"] == args.id)
    print(json.dumps(changed_batch, ensure_ascii=False, indent=2))
    if args.dry_run:
        print("dry-run：批次变更合法，未修改正式清单。")
        return 0
    _atomic_write_json(args.registry, candidate)
    print(
        f"已更新 {args.id}；当前状态 {changed_batch['state']}，"
        f"范围 {changed_batch['plannedCases']} 个用例。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

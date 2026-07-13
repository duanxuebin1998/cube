#!/usr/bin/env python3
"""向 CUBE 交付证据缺口追加一次显式生命周期事件。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any

from check_delivery_evidence_gaps import (
    DEFAULT_GAP_REGISTRY,
    GAP_ID_PATTERN,
    GIT_REVISION_PATTERN,
    ROOT,
    STATES,
    TRANSITIONS,
    GapValidationError,
    load_inputs,
    validate_gap_registry,
)


def write_json_atomic(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    handle, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(handle, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        os.replace(temporary_name, path)
    except Exception:
        Path(temporary_name).unlink(missing_ok=True)
        raise


def append_event(
    registry: dict[str, Any],
    *,
    gap_id: str,
    state: str,
    owner: str,
    explanation_path: str,
    decision_revision: str,
    related_evidence_ids: list[str],
) -> dict[str, Any]:
    if not GAP_ID_PATTERN.fullmatch(gap_id):
        raise GapValidationError("缺口 id 无效")
    if state not in STATES:
        raise GapValidationError("缺口状态无效")
    if not owner.strip():
        raise GapValidationError("owner 不能为空")
    if not GIT_REVISION_PATTERN.fullmatch(decision_revision):
        raise GapValidationError("decisionRevision 必须是完整 40 位小写 Git revision")
    target = next((item for item in registry["gaps"] if item["id"] == gap_id), None)
    if target is None:
        raise GapValidationError(f"缺口不存在：{gap_id}")
    previous_state = target["events"][-1]["state"]
    if state not in TRANSITIONS[previous_state]:
        raise GapValidationError(f"非法状态迁移：{previous_state} -> {state}")
    event = {
        "state": state,
        "owner": owner.strip(),
        "explanationPath": explanation_path,
        "decisionRevision": decision_revision,
        "relatedEvidenceIds": list(dict.fromkeys(related_evidence_ids)),
    }
    next_gaps = []
    for item in registry["gaps"]:
        next_gaps.append({**item, "events": [*item["events"], event]} if item["id"] == gap_id else item)
    return {**registry, "gaps": next_gaps}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("state", choices=STATES)
    parser.add_argument("--id", required=True, dest="gap_id")
    parser.add_argument("--owner", required=True)
    parser.add_argument("--explanation", required=True, dest="explanation_path")
    parser.add_argument("--decision-revision", required=True)
    parser.add_argument("--evidence-id", action="append", default=[], dest="related_evidence_ids")
    parser.add_argument("--registry", type=Path, default=DEFAULT_GAP_REGISTRY)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        load_args = argparse.Namespace(
            registry=args.registry,
            identity_registry=ROOT / "docs" / "00_程序流程导航" / "验证证据登记清单.json",
            evidence_manifest=ROOT / "docs" / "00_程序流程导航" / "流程证据清单.json",
            route_manifest=ROOT / "docs" / "00_程序流程导航" / "业务链路清单.json",
            baseline_registry=ROOT / "docs" / "00_程序流程导航" / "交付证据基线清单.json",
        )
        registry, identity, coverage = load_inputs(load_args)
        next_registry = append_event(
            registry,
            gap_id=args.gap_id,
            state=args.state,
            owner=args.owner,
            explanation_path=args.explanation_path,
            decision_revision=args.decision_revision,
            related_evidence_ids=args.related_evidence_ids,
        )
        validate_gap_registry(next_registry, identity, coverage, root=ROOT)
        if not args.dry_run:
            write_json_atomic(args.registry, next_registry)
    except (GapValidationError, ValueError, KeyError) as exc:
        print(f"交付证据缺口生命周期更新失败：{exc}", file=sys.stderr)
        return 1
    verb = "将追加" if args.dry_run else "已追加"
    print(f"{verb}缺口 {args.gap_id} 状态：{args.state}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

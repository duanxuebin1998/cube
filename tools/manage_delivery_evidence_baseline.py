#!/usr/bin/env python3
"""显式选择、替代、停用或审计保留 CUBE 交付证据基线。"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

from check_delivery_evidence_baselines import (
    BaselineValidationError,
    DEFAULT_REGISTRY,
    GIT_REVISION_PATTERN,
    ROOT,
    lifecycle_state,
    load_json,
    validate_formal_explanation_path,
    validate_registry,
)
from save_delivery_evidence_baseline import write_json_atomic


ACTIONS = {"select", "retire", "audit-only"}


def make_event(
    *,
    state: str,
    owner: str,
    explanation_path: str,
    decision_revision: str,
    superseded_by: str | None = None,
) -> dict[str, Any]:
    return {
        "state": state,
        "owner": owner.strip(),
        "explanationPath": explanation_path,
        "decisionRevision": decision_revision,
        "supersededBy": superseded_by,
    }


def update_lifecycle(
    *,
    baseline_id: str,
    action: str,
    owner: str,
    explanation_path: str,
    decision_revision: str,
    registry_path: Path = DEFAULT_REGISTRY,
    root: Path = ROOT,
    dry_run: bool = False,
) -> dict[str, Any]:
    if action not in ACTIONS:
        raise BaselineValidationError(f"不支持的生命周期动作：{action}")
    if not owner.strip():
        raise BaselineValidationError("生命周期 owner 不能为空")
    if not GIT_REVISION_PATTERN.fullmatch(decision_revision):
        raise BaselineValidationError("decisionRevision 必须是完整 40 位小写 Git revision")
    validate_formal_explanation_path(
        explanation_path,
        "生命周期 explanationPath",
        root=root,
    )

    registry = load_json(registry_path)
    validate_registry(registry, registry_path, root=root)
    entries_by_id = {item["id"]: item for item in registry["baselines"]}
    target = entries_by_id.get(baseline_id)
    if target is None:
        raise BaselineValidationError(f"基线不存在：{baseline_id}")
    target_state = lifecycle_state(target)["state"]
    if action == "select" and registry.get("selectedBaselineId") == baseline_id:
        raise BaselineValidationError(f"基线已经是当前选择：{baseline_id}")
    if action == "select" and target_state == "retired":
        raise BaselineValidationError("停用基线不能直接重新选择；先显式转为 audit-only")
    next_state = {
        "select": "current",
        "retire": "retired",
        "audit-only": "audit-only",
    }[action]
    if action != "select" and target_state == next_state:
        raise BaselineValidationError(f"基线已经是 {next_state} 状态：{baseline_id}")

    previous_selected = registry.get("selectedBaselineId")
    next_entries: list[dict[str, Any]] = []
    for entry in registry["baselines"]:
        lifecycle = list(entry["lifecycle"])
        if action == "select" and entry["id"] == previous_selected:
            lifecycle.append(
                make_event(
                    state="superseded",
                    owner=owner,
                    explanation_path=explanation_path,
                    decision_revision=decision_revision,
                    superseded_by=baseline_id,
                )
            )
        if entry["id"] == baseline_id:
            lifecycle.append(
                make_event(
                    state=next_state,
                    owner=owner,
                    explanation_path=explanation_path,
                    decision_revision=decision_revision,
                )
            )
        next_entries.append({**entry, "lifecycle": lifecycle})

    selected_id = baseline_id if action == "select" else previous_selected
    if action != "select" and previous_selected == baseline_id:
        selected_id = None
    next_registry = {
        **registry,
        "selectedBaselineId": selected_id,
        "baselines": next_entries,
    }
    validate_registry(next_registry, registry_path, root=root)
    if not dry_run:
        write_json_atomic(registry_path, next_registry)
    return next_registry


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=sorted(ACTIONS))
    parser.add_argument("--id", required=True, dest="baseline_id")
    parser.add_argument("--owner", required=True)
    parser.add_argument("--explanation", required=True, dest="explanation_path")
    parser.add_argument("--decision-revision", required=True)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        registry = update_lifecycle(
            baseline_id=args.baseline_id,
            action=args.action,
            owner=args.owner,
            explanation_path=args.explanation_path,
            decision_revision=args.decision_revision,
            registry_path=args.registry,
            dry_run=args.dry_run,
        )
    except BaselineValidationError as exc:
        print(f"交付证据基线生命周期更新失败：{exc}", file=sys.stderr)
        return 1
    verb = "将更新" if args.dry_run else "已更新"
    selected = registry.get("selectedBaselineId") or "未选择"
    print(f"{verb}基线 {args.baseline_id}：{args.action}；当前选择：{selected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

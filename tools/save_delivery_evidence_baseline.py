#!/usr/bin/env python3
"""把干净工作区的当前交付证据快照保存为显式 Git 基线。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import tempfile
from pathlib import Path
from typing import Any

from check_delivery_evidence_baselines import (
    BASELINE_DIRECTORY,
    BASELINE_ID_PATTERN,
    BaselineValidationError,
    DEFAULT_REGISTRY,
    GIT_REVISION_PATTERN,
    ROOT,
    load_json,
    validate_formal_explanation_path,
    validate_evidence_snapshot,
    validate_registry,
)


DEFAULT_SNAPSHOT = ROOT / "docs-site" / "public" / "data" / "current-delivery-evidence.json"


def write_json_atomic(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(data, ensure_ascii=False, indent=2) + "\n"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            stream.write(payload)
        temporary_path.replace(path)
    except Exception:
        temporary_path.unlink(missing_ok=True)
        raise


def save_baseline(
    *,
    baseline_id: str,
    label: str,
    snapshot_path: Path = DEFAULT_SNAPSHOT,
    registry_path: Path = DEFAULT_REGISTRY,
    root: Path = ROOT,
    owner: str,
    explanation_path: str,
    decision_revision: str | None = None,
    select: bool = False,
    dry_run: bool = False,
) -> tuple[dict[str, Any], Path]:
    if not BASELINE_ID_PATTERN.fullmatch(baseline_id):
        raise BaselineValidationError(
            "基线 id 只能使用小写字母、数字、点、下划线和短横线，最长 80 个字符"
        )
    if not label.strip():
        raise BaselineValidationError("基线 label 不能为空")
    if not owner.strip():
        raise BaselineValidationError("基线 owner 不能为空")
    validate_formal_explanation_path(
        explanation_path,
        "基线 explanationPath",
        root=root,
    )

    snapshot = load_json(snapshot_path)
    validate_evidence_snapshot(snapshot, str(snapshot_path), require_clean=True)
    registry = load_json(registry_path)
    validate_registry(registry, registry_path, root=root)
    if any(item.get("id") == baseline_id for item in registry["baselines"]):
        raise BaselineValidationError(f"基线 id 已存在：{baseline_id}")

    revision = snapshot["git"]["head"]
    decision_revision = decision_revision or revision
    if not GIT_REVISION_PATTERN.fullmatch(decision_revision):
        raise BaselineValidationError("基线 decisionRevision 必须是完整 40 位小写 Git revision")
    relative_path = f"{BASELINE_DIRECTORY}{baseline_id}.json"
    baseline_path = root / Path(relative_path)
    if baseline_path.exists():
        raise BaselineValidationError(f"基线文件已存在：{baseline_path}")

    envelope = {
        "schemaVersion": 1,
        "kind": "cube-delivery-evidence-baseline",
        "id": baseline_id,
        "label": label.strip(),
        "gitRevision": revision,
        "evidence": snapshot,
    }
    lifecycle_event = {
        "state": "current" if select else "audit-only",
        "owner": owner.strip(),
        "explanationPath": explanation_path,
        "decisionRevision": decision_revision,
        "supersededBy": None,
    }
    entry = {
        "id": baseline_id,
        "label": label.strip(),
        "path": relative_path,
        "gitRevision": revision,
        "lifecycle": [lifecycle_event],
    }
    next_baselines: list[dict[str, Any]] = []
    for item in registry["baselines"]:
        if select and item["id"] == registry.get("selectedBaselineId"):
            next_baselines.append(
                {
                    **item,
                    "lifecycle": [
                        *item["lifecycle"],
                        {
                            "state": "superseded",
                            "owner": owner.strip(),
                            "explanationPath": explanation_path,
                            "decisionRevision": decision_revision,
                            "supersededBy": baseline_id,
                        },
                    ],
                }
            )
        else:
            next_baselines.append(item)
    next_registry = {
        **registry,
        "selectedBaselineId": baseline_id if select else registry.get("selectedBaselineId"),
        "baselines": [*next_baselines, entry],
    }
    if not dry_run:
        write_json_atomic(baseline_path, envelope)
        try:
            validate_registry(next_registry, registry_path, root=root)
            write_json_atomic(registry_path, next_registry)
        except Exception:
            baseline_path.unlink(missing_ok=True)
            raise
    return next_registry, baseline_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--id", required=True, dest="baseline_id")
    parser.add_argument("--label", required=True)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--owner", required=True)
    parser.add_argument("--explanation", required=True, dest="explanation_path")
    parser.add_argument("--decision-revision")
    parser.add_argument("--select", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        registry, baseline_path = save_baseline(
            baseline_id=args.baseline_id,
            label=args.label,
            snapshot_path=args.snapshot,
            registry_path=args.registry,
            owner=args.owner,
            explanation_path=args.explanation_path,
            decision_revision=args.decision_revision,
            select=args.select,
            dry_run=args.dry_run,
        )
    except BaselineValidationError as exc:
        print(f"交付证据基线保存失败：{exc}", file=sys.stderr)
        return 1
    action = "将保存" if args.dry_run else "已保存"
    selected = registry.get("selectedBaselineId") or "未选择"
    print(f"{action}基线：{baseline_path}")
    print(f"当前选择：{selected}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

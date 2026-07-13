#!/usr/bin/env python3
"""校验 CUBE 交付证据基线清单与基线快照。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "docs" / "00_程序流程导航" / "交付证据基线清单.json"
BASELINE_DIRECTORY = "docs/00_程序流程导航/交付证据基线/"
SUPPORTED_REGISTRY_SCHEMA_VERSIONS = {2}
SUPPORTED_BASELINE_SCHEMA_VERSIONS = {1}
SUPPORTED_EVIDENCE_SCHEMA_VERSIONS = {1}
BASELINE_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9._-]{0,79}$")
GIT_REVISION_PATTERN = re.compile(r"^[0-9a-f]{40}$")
LIFECYCLE_STATES = {"current", "superseded", "retired", "audit-only"}
TREND_LIFECYCLE_STATES = {"current", "superseded"}


class BaselineValidationError(ValueError):
    """表示基线清单或快照不满足可审计契约。"""


def load_json(path: Path) -> dict[str, Any]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise BaselineValidationError(f"文件不存在：{path}") from exc
    except json.JSONDecodeError as exc:
        raise BaselineValidationError(f"JSON 格式无效：{path}:{exc.lineno}:{exc.colno}") from exc
    if not isinstance(data, dict):
        raise BaselineValidationError(f"JSON 根节点必须是对象：{path}")
    return data


def validate_evidence_snapshot(
    data: dict[str, Any],
    source: str,
    *,
    require_clean: bool,
) -> dict[str, Any]:
    schema_version = data.get("schemaVersion")
    if schema_version not in SUPPORTED_EVIDENCE_SCHEMA_VERSIONS:
        raise BaselineValidationError(
            f"{source} 的交付证据 schemaVersion={schema_version!r} 不受支持"
        )
    if data.get("scope") != "current-worktree":
        raise BaselineValidationError(f"{source} 的 scope 必须为 current-worktree")
    git = data.get("git")
    if not isinstance(git, dict):
        raise BaselineValidationError(f"{source} 缺少 git 对象")
    head = git.get("head")
    if not isinstance(head, str) or not GIT_REVISION_PATTERN.fullmatch(head):
        raise BaselineValidationError(f"{source} 必须记录完整 40 位小写 Git revision")
    if require_clean and git.get("dirty") is not False:
        raise BaselineValidationError(f"{source} 来自非干净工作区，不能保存为正式基线")
    for field in (
        "metrics",
        "status",
        "firmwareBaseline",
    ):
        if not isinstance(data.get(field), dict):
            raise BaselineValidationError(f"{source} 缺少对象字段 {field}")
    for field in (
        "checkpoints",
        "currentQueueItems",
        "affectedPages",
        "unmappedFirmwareSourcePaths",
        "unprojectedPageKeys",
        "formalEvidence",
    ):
        if not isinstance(data.get(field), list):
            raise BaselineValidationError(f"{source} 缺少数组字段 {field}")
    return data


def validate_baseline_envelope(
    data: dict[str, Any],
    source: str,
    *,
    expected_entry: dict[str, Any] | None = None,
) -> dict[str, Any]:
    schema_version = data.get("schemaVersion")
    if schema_version not in SUPPORTED_BASELINE_SCHEMA_VERSIONS:
        raise BaselineValidationError(
            f"{source} 的基线 schemaVersion={schema_version!r} 不受支持"
        )
    if data.get("kind") != "cube-delivery-evidence-baseline":
        raise BaselineValidationError(f"{source} 的 kind 无效")
    baseline_id = data.get("id")
    label = data.get("label")
    revision = data.get("gitRevision")
    if not isinstance(baseline_id, str) or not BASELINE_ID_PATTERN.fullmatch(baseline_id):
        raise BaselineValidationError(f"{source} 的 id 无效")
    if not isinstance(label, str) or not label.strip():
        raise BaselineValidationError(f"{source} 的 label 不能为空")
    if not isinstance(revision, str) or not GIT_REVISION_PATTERN.fullmatch(revision):
        raise BaselineValidationError(f"{source} 的 gitRevision 无效")
    evidence = data.get("evidence")
    if not isinstance(evidence, dict):
        raise BaselineValidationError(f"{source} 缺少 evidence 对象")
    validate_evidence_snapshot(evidence, f"{source}.evidence", require_clean=True)
    if evidence["git"]["head"] != revision:
        raise BaselineValidationError(f"{source} 的 gitRevision 与 evidence.git.head 不一致")
    if expected_entry is not None:
        for field in ("id", "label", "gitRevision"):
            if data.get(field) != expected_entry.get(field):
                raise BaselineValidationError(f"{source} 的 {field} 与清单不一致")
    return data


def validate_formal_explanation_path(
    value: Any,
    source: str,
    *,
    root: Path = ROOT,
) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith("docs/")
        or ".." in Path(value).parts
    ):
        raise BaselineValidationError(f"{source} 必须指向 docs/ 下的正式说明")
    docs_root = (root / "docs").resolve()
    candidate = (root / Path(value)).resolve()
    if candidate != docs_root and docs_root not in candidate.parents:
        raise BaselineValidationError(f"{source} 路径越界：{value}")
    if not candidate.is_file():
        raise BaselineValidationError(f"{source} 指向的正式说明不存在：{value}")
    return value


def validate_lifecycle_event(
    event: Any,
    source: str,
    *,
    root: Path = ROOT,
) -> dict[str, Any]:
    if not isinstance(event, dict):
        raise BaselineValidationError(f"{source} 必须是对象")
    state = event.get("state")
    if state not in LIFECYCLE_STATES:
        raise BaselineValidationError(f"{source} 的 state 无效")
    owner = event.get("owner")
    if not isinstance(owner, str) or not owner.strip():
        raise BaselineValidationError(f"{source} 缺少 owner")
    revision = event.get("decisionRevision")
    if not isinstance(revision, str) or not GIT_REVISION_PATTERN.fullmatch(revision):
        raise BaselineValidationError(f"{source} 的 decisionRevision 无效")
    validate_formal_explanation_path(
        event.get("explanationPath"),
        f"{source}.explanationPath",
        root=root,
    )
    superseded_by = event.get("supersededBy")
    if state == "superseded":
        if not isinstance(superseded_by, str) or not BASELINE_ID_PATTERN.fullmatch(superseded_by):
            raise BaselineValidationError(f"{source} 的 supersededBy 无效")
    elif superseded_by is not None:
        raise BaselineValidationError(f"{source} 只有 superseded 状态可以填写 supersededBy")
    return event


def lifecycle_state(entry: dict[str, Any]) -> dict[str, Any]:
    lifecycle = entry.get("lifecycle")
    if not isinstance(lifecycle, list) or not lifecycle:
        raise BaselineValidationError(f"基线 {entry.get('id', '<unknown>')} 缺少生命周期事件")
    return lifecycle[-1]


def validate_registry(
    registry: dict[str, Any],
    registry_path: Path,
    *,
    root: Path = ROOT,
) -> dict[str, Any]:
    schema_version = registry.get("schemaVersion")
    if schema_version not in SUPPORTED_REGISTRY_SCHEMA_VERSIONS:
        raise BaselineValidationError(
            f"{registry_path} 的 schemaVersion={schema_version!r} 不受支持"
        )
    if registry.get("kind") != "cube-delivery-evidence-baseline-registry":
        raise BaselineValidationError(f"{registry_path} 的 kind 无效")
    baselines = registry.get("baselines")
    if not isinstance(baselines, list):
        raise BaselineValidationError(f"{registry_path} 的 baselines 必须是数组")
    selected_id = registry.get("selectedBaselineId")
    if selected_id is not None and not isinstance(selected_id, str):
        raise BaselineValidationError(f"{registry_path} 的 selectedBaselineId 必须是字符串或 null")

    ids: set[str] = set()
    paths: set[str] = set()
    entries_by_id: dict[str, dict[str, Any]] = {}
    for index, entry in enumerate(baselines, start=1):
        if not isinstance(entry, dict):
            raise BaselineValidationError(f"{registry_path} 的第 {index} 个基线必须是对象")
        baseline_id = entry.get("id")
        label = entry.get("label")
        relative_path = entry.get("path")
        revision = entry.get("gitRevision")
        if not isinstance(baseline_id, str) or not BASELINE_ID_PATTERN.fullmatch(baseline_id):
            raise BaselineValidationError(f"{registry_path} 的第 {index} 个基线 id 无效")
        if baseline_id in ids:
            raise BaselineValidationError(f"{registry_path} 存在重复基线 id：{baseline_id}")
        ids.add(baseline_id)
        entries_by_id[baseline_id] = entry
        if not isinstance(label, str) or not label.strip():
            raise BaselineValidationError(f"{registry_path} 的基线 {baseline_id} 缺少 label")
        if (
            not isinstance(relative_path, str)
            or not relative_path.startswith(BASELINE_DIRECTORY)
            or not relative_path.endswith(".json")
            or ".." in Path(relative_path).parts
        ):
            raise BaselineValidationError(f"{registry_path} 的基线 {baseline_id} path 无效")
        if relative_path in paths:
            raise BaselineValidationError(f"{registry_path} 存在重复基线路径：{relative_path}")
        paths.add(relative_path)
        if not isinstance(revision, str) or not GIT_REVISION_PATTERN.fullmatch(revision):
            raise BaselineValidationError(f"{registry_path} 的基线 {baseline_id} gitRevision 无效")
        lifecycle = entry.get("lifecycle")
        if not isinstance(lifecycle, list) or not lifecycle:
            raise BaselineValidationError(f"{registry_path} 的基线 {baseline_id} lifecycle 必须是非空数组")
        previous_state: str | None = None
        for event_index, event in enumerate(lifecycle, start=1):
            validated_event = validate_lifecycle_event(
                event,
                f"{registry_path} 的基线 {baseline_id} 第 {event_index} 个生命周期事件",
                root=root,
            )
            if event_index == 1 and validated_event["state"] not in {"current", "audit-only"}:
                raise BaselineValidationError(
                    f"{registry_path} 的基线 {baseline_id} 首个生命周期状态只能是 current 或 audit-only"
                )
            if validated_event["state"] == previous_state:
                raise BaselineValidationError(
                    f"{registry_path} 的基线 {baseline_id} 存在连续重复生命周期状态"
                )
            previous_state = validated_event["state"]
        baseline_path = root / Path(relative_path)
        envelope = load_json(baseline_path)
        validate_baseline_envelope(envelope, str(baseline_path), expected_entry=entry)

    if selected_id is not None and selected_id not in ids:
        raise BaselineValidationError(
            f"{registry_path} 的 selectedBaselineId 未指向已登记基线：{selected_id}"
        )
    current_ids: list[str] = []
    for baseline_id, entry in entries_by_id.items():
        for event in entry["lifecycle"]:
            superseded_by = event.get("supersededBy")
            if superseded_by is not None:
                if superseded_by == baseline_id:
                    raise BaselineValidationError(f"{registry_path} 的基线 {baseline_id} 不能替代自身")
                if superseded_by not in ids:
                    raise BaselineValidationError(
                        f"{registry_path} 的基线 {baseline_id} supersededBy 未指向已登记基线：{superseded_by}"
                    )
        if lifecycle_state(entry)["state"] == "current":
            current_ids.append(baseline_id)
    if selected_id is None and current_ids:
        raise BaselineValidationError(f"{registry_path} 未选择基线，但仍存在 current 状态：{current_ids}")
    if selected_id is not None and current_ids != [selected_id]:
        raise BaselineValidationError(
            f"{registry_path} 的 selectedBaselineId 必须是唯一 current 基线：{selected_id}"
        )
    return registry


def build_lifecycle_report(registry: dict[str, Any]) -> dict[str, Any]:
    baselines: list[dict[str, Any]] = []
    counts = {state: 0 for state in LIFECYCLE_STATES}
    for entry in registry["baselines"]:
        current = lifecycle_state(entry)
        state = current["state"]
        counts[state] += 1
        baselines.append(
            {
                "id": entry["id"],
                "label": entry["label"],
                "gitRevision": entry["gitRevision"],
                "state": state,
                "owner": current["owner"],
                "explanationPath": current["explanationPath"],
                "decisionRevision": current["decisionRevision"],
                "supersededBy": current["supersededBy"],
                "selected": entry["id"] == registry.get("selectedBaselineId"),
                "trendEligible": state in TREND_LIFECYCLE_STATES,
                "eventCount": len(entry["lifecycle"]),
            }
        )
    trend_eligible = sum(item["trendEligible"] for item in baselines)
    return {
        "schemaVersion": 1,
        "kind": "cube-delivery-baseline-lifecycle",
        "registrySchemaVersion": registry["schemaVersion"],
        "selectedBaselineId": registry.get("selectedBaselineId"),
        "summary": {
            "total": len(baselines),
            "current": counts["current"],
            "superseded": counts["superseded"],
            "retired": counts["retired"],
            "auditOnly": counts["audit-only"],
            "trendEligible": trend_eligible,
        },
        "baselines": baselines,
        "method": {
            "explicitLifecycleOnly": True,
            "selectedMustBeCurrent": True,
            "lifecycleEvidenceRequired": True,
            "trendStates": sorted(TREND_LIFECYCLE_STATES),
            "allBaselinesRetainedForAudit": True,
        },
    }


def check_registry(registry_path: Path = DEFAULT_REGISTRY, *, root: Path = ROOT) -> dict[str, Any]:
    registry = load_json(registry_path)
    return validate_registry(registry, registry_path, root=root)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        registry = check_registry(args.registry)
    except BaselineValidationError as exc:
        print(f"交付证据基线检查失败：{exc}", file=sys.stderr)
        return 1
    selected = registry.get("selectedBaselineId") or "未建立基线"
    lifecycle = build_lifecycle_report(registry)
    print(
        f"交付证据基线检查通过：{len(registry['baselines'])} 个基线，当前选择：{selected}，"
        f"趋势允许 {lifecycle['summary']['trendEligible']} 个"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

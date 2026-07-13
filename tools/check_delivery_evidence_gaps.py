#!/usr/bin/env python3
"""检查 CUBE 交付证据缺口的人工处置生命周期与机器覆盖事实。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

from check_delivery_evidence_baselines import check_registry as check_baseline_registry
from check_delivery_evidence_coverage import (
    DEFAULT_BASELINE_REGISTRY,
    DEFAULT_EVIDENCE_MANIFEST,
    DEFAULT_IDENTITY_REGISTRY,
    DEFAULT_ROUTE_MANIFEST,
    ROOT,
    build_coverage_report,
    load_json,
    validate_identity_registry,
)


DEFAULT_GAP_REGISTRY = ROOT / "docs" / "00_程序流程导航" / "交付证据缺口处置清单.json"
GAP_ID_PATTERN = re.compile(r"gap-[a-z0-9][a-z0-9-]{1,78}")
GIT_REVISION_PATTERN = re.compile(r"[0-9a-f]{40}")
STATES = ("new", "claimed", "in-progress", "waiting-external", "closed", "not-applicable")
ACTIVE_STATES = {"new", "claimed", "in-progress", "waiting-external"}
TRANSITIONS = {
    "new": {"claimed", "waiting-external", "not-applicable"},
    "claimed": {"in-progress", "waiting-external", "not-applicable"},
    "in-progress": {"waiting-external", "closed"},
    "waiting-external": {"claimed", "in-progress", "not-applicable"},
    "closed": {"new"},
    "not-applicable": {"new"},
}


class GapValidationError(ValueError):
    """表示缺口处置清单或机器对账不满足正式契约。"""


def _formal_markdown_path(value: Any, source: str, *, root: Path) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith("docs/")
        or not value.lower().endswith(".md")
        or ".." in Path(value).parts
    ):
        raise GapValidationError(f"{source} 必须指向 docs/ 下的 Markdown")
    candidate = (root / Path(value)).resolve()
    docs_root = (root / "docs").resolve()
    if docs_root not in candidate.parents or not candidate.is_file():
        raise GapValidationError(f"{source} 指向的正式说明不存在：{value}")
    return value.replace("\\", "/")


def expected_gap_id(page_key: str) -> str:
    return f"gap-{page_key.replace('_', '-')}-specific-evidence"


def validate_gap_registry(
    registry: dict[str, Any],
    identity_registry: dict[str, Any],
    coverage: dict[str, Any],
    source: str = "交付证据缺口处置清单",
    *,
    root: Path = ROOT,
) -> dict[str, Any]:
    if registry.get("schemaVersion") != 1:
        raise GapValidationError(f"{source} 的 schemaVersion 必须为 1")
    if registry.get("kind") != "cube-delivery-evidence-gap-disposition-registry":
        raise GapValidationError(f"{source} 的 kind 无效")
    gaps = registry.get("gaps")
    if not isinstance(gaps, list):
        raise GapValidationError(f"{source}.gaps 必须是数组")

    page_by_key = {item["pageKey"]: item for item in coverage["pages"]}
    document_by_id = {item["id"]: item for item in coverage["documents"]}
    identity_ids = {item["id"] for item in identity_registry["documents"]}
    ids: set[str] = set()
    page_keys: set[str] = set()
    for index, gap in enumerate(gaps, start=1):
        if not isinstance(gap, dict):
            raise GapValidationError(f"{source} 的第 {index} 个缺口必须是对象")
        gap_id = gap.get("id")
        page_key = gap.get("pageKey")
        if not isinstance(gap_id, str) or not GAP_ID_PATTERN.fullmatch(gap_id):
            raise GapValidationError(f"{source} 的第 {index} 个缺口 id 无效")
        if not isinstance(page_key, str) or page_key not in page_by_key:
            raise GapValidationError(f"{source}.{gap_id}.pageKey 不是已登记核心流程")
        if gap_id != expected_gap_id(page_key):
            raise GapValidationError(f"{source}.{gap_id} 与 pageKey 的稳定 ID 规则不一致")
        if gap_id in ids or page_key in page_keys:
            raise GapValidationError(f"{source} 存在重复缺口：{gap_id}/{page_key}")
        ids.add(gap_id)
        page_keys.add(page_key)

        events = gap.get("events")
        if not isinstance(events, list) or not events:
            raise GapValidationError(f"{source}.{gap_id}.events 必须是非空数组")
        previous_state: str | None = None
        for event_index, event in enumerate(events, start=1):
            if not isinstance(event, dict):
                raise GapValidationError(f"{source}.{gap_id}.events[{event_index}] 必须是对象")
            state = event.get("state")
            if state not in STATES:
                raise GapValidationError(f"{source}.{gap_id}.events[{event_index}].state 无效")
            if event_index == 1 and state != "new":
                raise GapValidationError(f"{source}.{gap_id} 的首个事件必须为 new")
            if previous_state is not None and state not in TRANSITIONS[previous_state]:
                raise GapValidationError(f"{source}.{gap_id} 存在非法状态迁移：{previous_state} -> {state}")
            owner = event.get("owner")
            if not isinstance(owner, str) or not owner.strip():
                raise GapValidationError(f"{source}.{gap_id}.events[{event_index}].owner 不能为空")
            _formal_markdown_path(
                event.get("explanationPath"),
                f"{source}.{gap_id}.events[{event_index}].explanationPath",
                root=root,
            )
            revision = event.get("decisionRevision")
            if not isinstance(revision, str) or not GIT_REVISION_PATTERN.fullmatch(revision):
                raise GapValidationError(f"{source}.{gap_id}.events[{event_index}].decisionRevision 无效")
            evidence_ids = event.get("relatedEvidenceIds")
            if (
                not isinstance(evidence_ids, list)
                or len(evidence_ids) != len(set(evidence_ids))
                or any(evidence_id not in identity_ids for evidence_id in evidence_ids)
            ):
                raise GapValidationError(f"{source}.{gap_id}.events[{event_index}].relatedEvidenceIds 无效")
            previous_state = state

        current_state = events[-1]["state"]
        machine_gap = page_by_key[page_key]["state"] == "missing-specific"
        if machine_gap and current_state == "closed":
            raise GapValidationError(f"{source}.{gap_id} 已关闭，但机器仍报告专项证据缺口")
        if not machine_gap and current_state in ACTIVE_STATES:
            raise GapValidationError(f"{source}.{gap_id} 的机器缺口已消失，但人工状态仍为 {current_state}")
        if current_state == "closed":
            related = events[-1]["relatedEvidenceIds"]
            if not related or not any(page_key in document_by_id[item]["specificPageKeys"] for item in related):
                raise GapValidationError(f"{source}.{gap_id} 关闭时必须关联真正覆盖该流程的专项证据")

    machine_gap_keys = {
        item["pageKey"] for item in coverage["pages"] if item["state"] == "missing-specific"
    }
    missing_records = sorted(machine_gap_keys - page_keys)
    if missing_records:
        raise GapValidationError(f"当前机器缺口尚未登记处置记录：{missing_records}")
    return registry


def build_gap_report(registry: dict[str, Any], coverage: dict[str, Any]) -> dict[str, Any]:
    page_by_key = {item["pageKey"]: item for item in coverage["pages"]}
    gaps = []
    for entry in registry["gaps"]:
        latest = entry["events"][-1]
        page = page_by_key[entry["pageKey"]]
        machine_state = "missing-specific" if page["state"] == "missing-specific" else "covered"
        gaps.append(
            {
                "id": entry["id"],
                "pageKey": entry["pageKey"],
                "routeIds": page["routeIds"],
                "machineState": machine_state,
                "dispositionState": latest["state"],
                "owner": latest["owner"],
                "explanationPath": latest["explanationPath"],
                "decisionRevision": latest["decisionRevision"],
                "relatedEvidenceIds": latest["relatedEvidenceIds"],
                "eventCount": len(entry["events"]),
                "alignment": "aligned",
            }
        )
    gaps.sort(key=lambda item: item["pageKey"])
    counts = {state: sum(item["dispositionState"] == state for item in gaps) for state in STATES}
    machine_gaps = sum(item["machineState"] == "missing-specific" for item in gaps)
    return {
        "schemaVersion": 1,
        "kind": "cube-delivery-evidence-gap-dispositions",
        "state": "active-gaps" if machine_gaps else "clear",
        "summary": {
            "machineGaps": machine_gaps,
            "registeredGaps": len(gaps),
            "active": sum(item["dispositionState"] in ACTIVE_STATES for item in gaps),
            "waitingExternal": counts["waiting-external"],
            "closed": counts["closed"],
            "notApplicable": counts["not-applicable"],
            "states": counts,
        },
        "gaps": gaps,
        "method": {
            "machineDiscoveryFromCoverage": True,
            "manualDispositionFromRegistry": True,
            "appendOnlyEvents": True,
            "routeRelationsDerived": True,
            "manualStateDoesNotChangeCoverage": True,
            "naturalLanguageInference": False,
        },
    }


def load_inputs(args: argparse.Namespace) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    identity = validate_identity_registry(load_json(args.identity_registry), root=ROOT)
    baselines = check_baseline_registry(args.baseline_registry, root=ROOT)
    coverage = build_coverage_report(
        identity,
        load_json(args.evidence_manifest),
        load_json(args.route_manifest),
        baselines,
    )
    registry = validate_gap_registry(load_json(args.registry), identity, coverage, root=ROOT)
    return registry, identity, coverage


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_GAP_REGISTRY)
    parser.add_argument("--identity-registry", type=Path, default=DEFAULT_IDENTITY_REGISTRY)
    parser.add_argument("--evidence-manifest", type=Path, default=DEFAULT_EVIDENCE_MANIFEST)
    parser.add_argument("--route-manifest", type=Path, default=DEFAULT_ROUTE_MANIFEST)
    parser.add_argument("--baseline-registry", type=Path, default=DEFAULT_BASELINE_REGISTRY)
    parser.add_argument("--snapshot", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        registry, _identity, coverage = load_inputs(args)
        report = build_gap_report(registry, coverage)
        if args.snapshot is not None and load_json(args.snapshot) != report:
            raise GapValidationError(f"站点缺口处置快照与正式清单不一致：{args.snapshot}")
    except (GapValidationError, ValueError, KeyError) as exc:
        print(f"交付证据缺口处置检查失败：{exc}", file=sys.stderr)
        return 1
    print(
        "交付证据缺口处置检查通过："
        f"机器缺口 {report['summary']['machineGaps']}，"
        f"已登记 {report['summary']['registeredGaps']}，"
        f"活动处置 {report['summary']['active']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

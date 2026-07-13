#!/usr/bin/env python3
"""校验治理事项的追加式接手、执行、等待、关闭和重新出现契约。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "docs" / "00_程序流程导航" / "治理事项处置清单.json"
DEFAULT_QUEUE = ROOT / "docs-site" / "public" / "data" / "governance-queue.json"
ITEM_ID_PATTERN = re.compile(r"gov-[a-z0-9][a-z0-9-]{2,}")
REVISION_PATTERN = re.compile(r"[0-9a-f]{40}")
FINGERPRINT_PATTERN = re.compile(r"[0-9a-f]{64}")
OWNER_IDS = {"cpu2", "cpu3", "validation", "delivery", "docs"}
STATES = ("claimed", "in-progress", "waiting-external", "closed")
ACTIVE_STATES = {"claimed", "in-progress", "waiting-external"}
TRANSITIONS = {
    "claimed": {"in-progress", "waiting-external", "closed"},
    "in-progress": {"waiting-external", "closed"},
    "waiting-external": {"claimed", "in-progress", "closed"},
    "closed": {"claimed"},
}
ACTION_KINDS = {
    "flow-review",
    "flow-mapping",
    "flow-adjudication",
    "evidence-registration",
    "validation-run",
    "validation-batch",
    "delivery-baseline",
}
CONTRACT_ACTION_KINDS = {
    "stale-review": {"flow-review"},
    "unmapped-source": {"flow-mapping", "flow-adjudication"},
    "missing-specific-evidence": {"evidence-registration"},
    "no-real-run": {"validation-run"},
    "no-validation-batch": {"validation-batch"},
    "no-delivery-baseline": {"delivery-baseline"},
}


class GovernanceItemError(ValueError):
    """表示治理事项不满足追加式关闭链契约。"""


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise GovernanceItemError(f"无法读取 JSON：{path}: {exc}") from exc
    if not isinstance(value, dict):
        raise GovernanceItemError(f"JSON 顶层必须是对象：{path}")
    return value


def _text(value: Any, field: str, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise GovernanceItemError(f"{context} 的 {field} 必须是非空字符串")
    return value.strip()


def _timestamp(value: Any, field: str, context: str) -> datetime:
    text = _text(value, field, context)
    try:
        parsed = datetime.fromisoformat(text)
    except ValueError as exc:
        raise GovernanceItemError(f"{context} 的 {field} 不是有效 ISO 8601 时间") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise GovernanceItemError(f"{context} 的 {field} 必须包含时区")
    return parsed


def _docs_path(value: Any, field: str, context: str, root: Path) -> str:
    relative = _text(value, field, context).replace("\\", "/").removeprefix("./")
    if not relative.startswith("docs/") or relative.lower().endswith(".pdf"):
        raise GovernanceItemError(f"{context} 的 {field} 必须指向 docs/ 下的非 PDF 文件")
    candidate = (root / Path(relative)).resolve()
    docs_root = (root / "docs").resolve()
    if docs_root not in candidate.parents or not candidate.is_file():
        raise GovernanceItemError(f"{context} 的 {field} 不存在或路径越界：{relative}")
    return relative


def build_governance_item_report(
    registry: dict[str, Any], *, root: Path = ROOT
) -> dict[str, Any]:
    """验证正式处置清单并生成当前事件摘要。"""

    if registry.get("schemaVersion") != 1:
        raise GovernanceItemError("治理事项处置清单 schemaVersion 必须为 1")
    if registry.get("kind") != "cube-governance-item-registry":
        raise GovernanceItemError("治理事项处置清单 kind 无效")
    _text(registry.get("description"), "description", "治理事项处置清单")
    raw_items = registry.get("items")
    if not isinstance(raw_items, list):
        raise GovernanceItemError("治理事项处置清单 items 必须是数组")

    seen_ids: set[str] = set()
    items: list[dict[str, Any]] = []
    for item_index, raw_item in enumerate(raw_items):
        if not isinstance(raw_item, dict):
            raise GovernanceItemError(f"第 {item_index + 1} 个治理事项必须是对象")
        item_id = _text(raw_item.get("id"), "id", f"第 {item_index + 1} 个治理事项")
        if ITEM_ID_PATTERN.fullmatch(item_id) is None:
            raise GovernanceItemError(f"治理事项 ID 无效：{item_id}")
        if item_id in seen_ids:
            raise GovernanceItemError(f"治理事项 ID 重复：{item_id}")
        seen_ids.add(item_id)
        kind = _text(raw_item.get("kind"), "kind", item_id)
        if kind not in CONTRACT_ACTION_KINDS:
            raise GovernanceItemError(f"{item_id} 的 kind 无关闭契约：{kind}")
        raw_events = raw_item.get("events")
        if not isinstance(raw_events, list) or not raw_events:
            raise GovernanceItemError(f"{item_id} 必须至少包含一个接手事件")

        events: list[dict[str, Any]] = []
        previous_state: str | None = None
        previous_time: datetime | None = None
        previous_fingerprint: str | None = None
        previous_owner_id: str | None = None
        for event_index, raw_event in enumerate(raw_events):
            if not isinstance(raw_event, dict):
                raise GovernanceItemError(f"{item_id} 的第 {event_index + 1} 个事件必须是对象")
            context = f"{item_id} 事件 {event_index + 1}"
            sequence = raw_event.get("sequence")
            if sequence != event_index + 1:
                raise GovernanceItemError(f"{context} 的 sequence 必须连续从 1 递增")
            state = _text(raw_event.get("state"), "state", context)
            if state not in STATES:
                raise GovernanceItemError(f"{context} 的 state 无效：{state}")
            if previous_state is None and state != "claimed":
                raise GovernanceItemError(f"{item_id} 的首个事件必须是 claimed")
            if previous_state is not None and state not in TRANSITIONS[previous_state]:
                raise GovernanceItemError(f"{item_id} 不允许从 {previous_state} 转换到 {state}")
            recorded_at = _timestamp(raw_event.get("recordedAt"), "recordedAt", context)
            if previous_time is not None and recorded_at < previous_time:
                raise GovernanceItemError(f"{context} 的 recordedAt 不能早于上一事件")
            owner_id = _text(raw_event.get("ownerId"), "ownerId", context)
            if owner_id not in OWNER_IDS:
                raise GovernanceItemError(f"{context} 的 ownerId 无效：{owner_id}")
            owner = _text(raw_event.get("owner"), "owner", context)
            revision = _text(raw_event.get("decisionRevision"), "decisionRevision", context)
            if REVISION_PATTERN.fullmatch(revision) is None:
                raise GovernanceItemError(f"{context} 的 decisionRevision 必须是 40 位小写 revision")
            explanation = _docs_path(
                raw_event.get("explanationPath"), "explanationPath", context, root
            )
            fingerprint = _text(raw_event.get("factFingerprint"), "factFingerprint", context)
            if FINGERPRINT_PATTERN.fullmatch(fingerprint) is None:
                raise GovernanceItemError(f"{context} 的 factFingerprint 必须是 64 位小写 SHA-256")
            if previous_state != "closed" and previous_fingerprint not in (None, fingerprint):
                raise GovernanceItemError(f"{item_id} 的活动生命周期不得静默更换事实指纹")
            if previous_state != "closed" and previous_owner_id not in (None, owner_id):
                raise GovernanceItemError(f"{item_id} 的活动生命周期不得静默更换责任组")

            raw_actions = raw_event.get("actionRefs", [])
            if not isinstance(raw_actions, list):
                raise GovernanceItemError(f"{context} 的 actionRefs 必须是数组")
            actions: list[dict[str, str]] = []
            for action_index, raw_action in enumerate(raw_actions):
                if not isinstance(raw_action, dict):
                    raise GovernanceItemError(f"{context} 的动作 {action_index + 1} 必须是对象")
                action_context = f"{context} 动作 {action_index + 1}"
                action_kind = _text(raw_action.get("kind"), "kind", action_context)
                if action_kind not in ACTION_KINDS:
                    raise GovernanceItemError(f"{action_context} 的 kind 无效：{action_kind}")
                actions.append({
                    "kind": action_kind,
                    "id": _text(raw_action.get("id"), "id", action_context),
                    "path": _docs_path(raw_action.get("path"), "path", action_context, root),
                })
            if state in {"in-progress", "closed"} and not actions:
                raise GovernanceItemError(f"{context} 必须至少引用一项正式动作")
            if state == "closed" and not (
                {action["kind"] for action in actions} & CONTRACT_ACTION_KINDS[kind]
            ):
                expected = ", ".join(sorted(CONTRACT_ACTION_KINDS[kind]))
                raise GovernanceItemError(f"{context} 的关闭动作类型不匹配，期望：{expected}")

            events.append({
                "sequence": sequence,
                "state": state,
                "recordedAt": raw_event["recordedAt"],
                "ownerId": owner_id,
                "owner": owner,
                "decisionRevision": revision,
                "explanationPath": explanation,
                "factFingerprint": fingerprint,
                "actionRefs": actions,
            })
            previous_state = state
            previous_time = recorded_at
            previous_fingerprint = fingerprint
            previous_owner_id = owner_id

        items.append({
            "id": item_id,
            "kind": kind,
            "state": events[-1]["state"],
            "ownerId": events[-1]["ownerId"],
            "owner": events[-1]["owner"],
            "factFingerprint": events[-1]["factFingerprint"],
            "eventCount": len(events),
            "events": events,
        })

    return {
        "schemaVersion": 1,
        "kind": "cube-governance-item-lifecycle",
        "state": "empty" if not items else "events-recorded",
        "summary": {
            "items": len(items),
            "active": sum(item["state"] in ACTIVE_STATES for item in items),
            "closed": sum(item["state"] == "closed" for item in items),
            "events": sum(item["eventCount"] for item in items),
        },
        "items": items,
        "method": {
            "appendOnlyEvents": True,
            "machineFactsRemainAuthoritative": True,
            "closedRequiresContractAction": True,
            "reopenedPreservesHistory": True,
        },
    }


def validate_queue_alignment(report: dict[str, Any], queue: dict[str, Any]) -> dict[str, int]:
    """把正式事件与重新同步后的当前机器事实对账。"""

    if queue.get("schemaVersion") != 2 or queue.get("kind") != "cube-governance-queue":
        raise GovernanceItemError("治理队列必须先重新同步到 schema 2")
    current_items = queue.get("items")
    if not isinstance(current_items, list):
        raise GovernanceItemError("治理队列 items 必须是数组")
    current_by_id = {item.get("governanceKey"): item for item in current_items}
    if None in current_by_id or len(current_by_id) != len(current_items):
        raise GovernanceItemError("治理队列存在缺失或重复的 governanceKey")
    formal_by_id = {item["id"]: item for item in report["items"]}

    for item in report["items"]:
        current = current_by_id.get(item["id"])
        if item["state"] in ACTIVE_STATES:
            if current is None:
                raise GovernanceItemError(f"{item['id']} 的机器事实已清零，必须追加 closed 事件")
            if current.get("factFingerprint") != item["factFingerprint"]:
                raise GovernanceItemError(f"{item['id']} 的活动事件事实指纹已漂移，必须先闭环旧事件")

    reopened = 0
    unclaimed = 0
    for current in current_items:
        lifecycle = current.get("lifecycle")
        if not isinstance(lifecycle, dict):
            raise GovernanceItemError(f"{current['governanceKey']} 缺少派生生命周期")
        formal = formal_by_id.get(current["governanceKey"])
        expected = "unclaimed" if formal is None else (
            "reopened" if formal["state"] == "closed" else formal["state"]
        )
        if lifecycle.get("state") != expected:
            raise GovernanceItemError(
                f"{current['governanceKey']} 生命周期不一致：派生 {lifecycle.get('state')}，期望 {expected}"
            )
        if formal is not None and lifecycle.get("eventCount") != formal["eventCount"]:
            raise GovernanceItemError(f"{current['governanceKey']} 的事件数量与正式清单不一致")
        reopened += expected == "reopened"
        unclaimed += expected == "unclaimed"
    return {"current": len(current_items), "unclaimed": unclaimed, "reopened": reopened}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--queue", type=Path, default=DEFAULT_QUEUE)
    parser.add_argument("--skip-queue", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        report = build_governance_item_report(load_json(args.registry))
        alignment = None if args.skip_queue else validate_queue_alignment(report, load_json(args.queue))
    except GovernanceItemError as exc:
        print(f"治理事项关闭链检查失败：{exc}", file=sys.stderr)
        return 1
    suffix = "" if alignment is None else (
        f"；当前 {alignment['current']} 项，待接手 {alignment['unclaimed']} 项，重新出现 {alignment['reopened']} 项"
    )
    print(
        f"治理事项关闭链检查通过：正式事项 {report['summary']['items']} 个，"
        f"事件 {report['summary']['events']} 条{suffix}。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

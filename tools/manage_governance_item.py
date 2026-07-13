#!/usr/bin/env python3
"""向治理事项正式清单原子追加责任接手、执行、等待或关闭事件。"""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path

from check_governance_items import (
    ACTIVE_STATES,
    CONTRACT_ACTION_KINDS,
    DEFAULT_QUEUE,
    DEFAULT_REGISTRY,
    ROOT,
    build_governance_item_report,
    load_json,
)


def _action(value: str) -> dict[str, str]:
    kind, first, remainder = value.partition(":")
    action_id, second, path = remainder.partition(":")
    if not first or not second or not kind or not action_id or not path:
        raise argparse.ArgumentTypeError("--action 必须使用 <kind>:<id>:<docs/path>")
    return {"kind": kind, "id": action_id, "path": path.replace("\\", "/")}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--queue", type=Path, default=DEFAULT_QUEUE)
    subparsers = parser.add_subparsers(dest="command", required=True)
    transition = subparsers.add_parser("transition", help="追加治理事项生命周期事件")
    transition.add_argument("--id", required=True)
    transition.add_argument(
        "--state", required=True, choices=("claimed", "in-progress", "waiting-external", "closed")
    )
    transition.add_argument("--recorded-at", required=True)
    transition.add_argument("--owner-id", required=True)
    transition.add_argument("--owner", required=True)
    transition.add_argument("--decision-revision", required=True)
    transition.add_argument("--explanation", required=True)
    transition.add_argument("--action", action="append", default=[], type=_action)
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


def _candidate_registry(
    args: argparse.Namespace,
    registry: dict[str, object],
    queue: dict[str, object],
) -> dict[str, object]:
    items = [dict(item) for item in registry.get("items", [])]
    matching = [index for index, item in enumerate(items) if item.get("id") == args.id]
    current_by_id = {
        item.get("governanceKey"): item for item in queue.get("items", []) if isinstance(item, dict)
    }
    current = current_by_id.get(args.id)

    if args.state == "closed":
        if current is not None:
            raise ValueError(f"{args.id} 的机器事实仍存在，禁止手工关闭")
        if len(matching) != 1:
            raise ValueError(f"关闭前必须且只能找到一个活动治理事项：{args.id}")
        item = dict(items[matching[0]])
        events = list(item.get("events", []))
        if not events or events[-1].get("state") not in ACTIVE_STATES:
            raise ValueError(f"{args.id} 当前没有可关闭的活动事件")
        fingerprint = events[-1]["factFingerprint"]
        if not ({action["kind"] for action in args.action} & CONTRACT_ACTION_KINDS[item["kind"]]):
            expected = ", ".join(sorted(CONTRACT_ACTION_KINDS[item["kind"]]))
            raise ValueError(f"关闭动作类型不匹配，期望：{expected}")
    else:
        if current is None:
            raise ValueError(f"{args.id} 不在当前治理队列中，不能追加活动事件")
        if args.owner_id != current.get("ownerId"):
            raise ValueError(
                f"{args.id} 的责任组必须来自当前策略：{current.get('ownerId')}"
            )
        fingerprint = current["factFingerprint"]
        if not matching:
            if args.state != "claimed":
                raise ValueError(f"{args.id} 首个正式事件必须是 claimed")
            item = {"id": args.id, "kind": current["kind"], "events": []}
            items.append(item)
            matching = [len(items) - 1]
        elif len(matching) == 1:
            item = dict(items[matching[0]])
        else:
            raise ValueError(f"治理事项 ID 重复：{args.id}")
        events = list(item.get("events", []))

    if args.state == "in-progress" and not args.action:
        raise ValueError("进入 in-progress 必须至少引用一项正式动作")
    event = {
        "sequence": len(events) + 1,
        "state": args.state,
        "recordedAt": args.recorded_at,
        "ownerId": args.owner_id,
        "owner": args.owner,
        "decisionRevision": args.decision_revision,
        "explanationPath": args.explanation.replace("\\", "/"),
        "factFingerprint": fingerprint,
        "actionRefs": args.action,
    }
    item["events"] = [*events, event]
    items[matching[0]] = item
    return {**registry, "items": items}


def main() -> int:
    args = parse_args()
    registry = load_json(args.registry)
    queue = load_json(args.queue)
    candidate = _candidate_registry(args, registry, queue)
    report = build_governance_item_report(candidate, root=ROOT)
    changed = next(item for item in report["items"] if item["id"] == args.id)
    print(json.dumps(changed, ensure_ascii=False, indent=2))
    if args.dry_run:
        print("dry-run：治理事件合法，未修改正式清单。")
        return 0
    _atomic_write_json(args.registry, candidate)
    print(f"已更新 {args.id}；当前状态 {changed['state']}，事件 {changed['eventCount']} 条。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

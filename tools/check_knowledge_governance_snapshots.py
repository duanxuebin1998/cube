#!/usr/bin/env python3
"""校验跨版本知识治理月度快照的追加式、稳定身份和证据边界。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_REGISTRY = ROOT / "docs" / "00_程序流程导航" / "知识治理月度快照清单.json"
MONTH_PATTERN = re.compile(r"\d{4}-(?:0[1-9]|1[0-2])")
REVISION_PATTERN = re.compile(r"[0-9a-f]{40}")
FINGERPRINT_PATTERN = re.compile(r"[0-9a-f]{64}")
ITEM_ID_PATTERN = re.compile(r"gov-[a-z0-9][a-z0-9-]{2,}")
PAGE_KEY_PATTERN = re.compile(r"cpu[23]_\d{2}")
OWNER_IDS = {"cpu2", "cpu3", "validation", "delivery", "docs"}
ITEM_KINDS = {
    "stale-review",
    "unmapped-source",
    "missing-specific-evidence",
    "no-real-run",
    "no-validation-batch",
    "no-delivery-baseline",
}
LIFECYCLE_STATES = {
    "unclaimed",
    "claimed",
    "in-progress",
    "waiting-external",
    "reopened",
}
BASELINE_STATES = {"current", "superseded", "retired", "audit-only"}


class KnowledgeGovernanceSnapshotError(ValueError):
    """表示月度快照不满足正式历史契约。"""


def shanghai_timezone():  # noqa: ANN201
    """优先使用 IANA 时区；Windows 未安装 tzdata 时回退到当前中国标准时间。"""
    try:
        return ZoneInfo("Asia/Shanghai")
    except ZoneInfoNotFoundError:
        return timezone(timedelta(hours=8), name="Asia/Shanghai")


def load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise KnowledgeGovernanceSnapshotError(f"无法读取 JSON：{path}：{exc}") from exc
    if not isinstance(value, dict):
        raise KnowledgeGovernanceSnapshotError(f"JSON 根节点必须是对象：{path}")
    return value


def _text(value: Any, field: str, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise KnowledgeGovernanceSnapshotError(f"{context} 的 {field} 必须是非空字符串")
    return value.strip()


def _sorted_unique_texts(value: Any, field: str, context: str) -> list[str]:
    if not isinstance(value, list) or any(not isinstance(item, str) or not item for item in value):
        raise KnowledgeGovernanceSnapshotError(f"{context} 的 {field} 必须是非空字符串数组")
    if value != sorted(set(value)):
        raise KnowledgeGovernanceSnapshotError(f"{context} 的 {field} 必须排序且不得重复")
    return value


def _timestamp(value: Any, field: str, context: str) -> datetime:
    text = _text(value, field, context)
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise KnowledgeGovernanceSnapshotError(f"{context} 的 {field} 不是合法 ISO 8601 时间：{text}") from exc
    if parsed.tzinfo is None:
        raise KnowledgeGovernanceSnapshotError(f"{context} 的 {field} 必须包含时区：{text}")
    return parsed


def _docs_path(value: Any, field: str, context: str, root: Path) -> str:
    text = _text(value, field, context).replace("\\", "/")
    if not text.startswith("docs/") or ".." in Path(text).parts:
        raise KnowledgeGovernanceSnapshotError(f"{context} 的 {field} 必须指向 docs/ 正式资料：{text}")
    if not (root / Path(text)).is_file():
        raise KnowledgeGovernanceSnapshotError(f"{context} 的 {field} 不存在：{text}")
    return text


def validate_registry(registry: dict[str, Any], *, root: Path = ROOT) -> dict[str, int]:
    """验证正式清单，返回可用于门禁输出的摘要。"""

    if registry.get("schemaVersion") != 1:
        raise KnowledgeGovernanceSnapshotError("知识治理月度快照 schemaVersion 必须为 1")
    if registry.get("kind") != "cube-knowledge-governance-snapshot-registry":
        raise KnowledgeGovernanceSnapshotError("知识治理月度快照 kind 无效")
    _text(registry.get("description"), "description", "知识治理月度快照")
    cadence = registry.get("cadence")
    if not isinstance(cadence, dict) or cadence.get("period") != "monthly" or cadence.get("timezone") != "Asia/Shanghai":
        raise KnowledgeGovernanceSnapshotError("月度快照 cadence 必须为 monthly / Asia/Shanghai")
    raw_snapshots = registry.get("snapshots")
    if not isinstance(raw_snapshots, list):
        raise KnowledgeGovernanceSnapshotError("月度快照 snapshots 必须是数组")

    seen_ids: set[str] = set()
    seen_months: set[str] = set()
    previous_month = ""
    fact_count = 0
    for index, raw in enumerate(raw_snapshots):
        context = f"第 {index + 1} 个月度快照"
        if not isinstance(raw, dict):
            raise KnowledgeGovernanceSnapshotError(f"{context} 必须是对象")
        month = _text(raw.get("month"), "month", context)
        if MONTH_PATTERN.fullmatch(month) is None:
            raise KnowledgeGovernanceSnapshotError(f"{context} 的 month 无效：{month}")
        snapshot_id = _text(raw.get("id"), "id", context)
        if snapshot_id != f"kgs-{month}":
            raise KnowledgeGovernanceSnapshotError(f"{context} 的 id 必须为 kgs-{month}")
        if snapshot_id in seen_ids or month in seen_months:
            raise KnowledgeGovernanceSnapshotError(f"月度快照身份或月份重复：{snapshot_id}")
        if previous_month and month <= previous_month:
            raise KnowledgeGovernanceSnapshotError("月度快照必须按月份严格追加")
        seen_ids.add(snapshot_id)
        seen_months.add(month)
        previous_month = month
        recorded_at = _timestamp(raw.get("recordedAt"), "recordedAt", snapshot_id)
        local_month = recorded_at.astimezone(shanghai_timezone()).strftime("%Y-%m")
        if local_month != month:
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 recordedAt 与 month 不一致")
        revision = _text(raw.get("gitRevision"), "gitRevision", snapshot_id)
        if REVISION_PATTERN.fullmatch(revision) is None:
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 gitRevision 必须是 40 位小写十六进制")
        _docs_path(raw.get("explanationPath"), "explanationPath", snapshot_id, root)

        review = raw.get("review")
        if not isinstance(review, dict):
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 review 必须是对象")
        if not isinstance(review.get("windowDays"), int) or review["windowDays"] <= 0:
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 review.windowDays 必须是正整数")
        if not isinstance(review.get("totalPages"), int) or review["totalPages"] < 0:
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 review.totalPages 必须是非负整数")
        stale_pages = _sorted_unique_texts(review.get("stalePageKeys"), "review.stalePageKeys", snapshot_id)
        if len(stale_pages) > review["totalPages"] or any(PAGE_KEY_PATTERN.fullmatch(item) is None for item in stale_pages):
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的过期流程页键无效")

        raw_facts = raw.get("governanceFacts")
        if not isinstance(raw_facts, list):
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 governanceFacts 必须是数组")
        fact_ids: list[str] = []
        for fact_index, fact in enumerate(raw_facts):
            fact_context = f"{snapshot_id} 的第 {fact_index + 1} 个治理事实"
            if not isinstance(fact, dict):
                raise KnowledgeGovernanceSnapshotError(f"{fact_context} 必须是对象")
            fact_id = _text(fact.get("id"), "id", fact_context)
            if ITEM_ID_PATTERN.fullmatch(fact_id) is None:
                raise KnowledgeGovernanceSnapshotError(f"{fact_context} 的 id 无效：{fact_id}")
            fingerprint = _text(fact.get("factFingerprint"), "factFingerprint", fact_id)
            if FINGERPRINT_PATTERN.fullmatch(fingerprint) is None:
                raise KnowledgeGovernanceSnapshotError(f"{fact_id} 的 factFingerprint 必须是 SHA-256")
            if fact.get("ownerId") not in OWNER_IDS:
                raise KnowledgeGovernanceSnapshotError(f"{fact_id} 的 ownerId 无效")
            if fact.get("kind") not in ITEM_KINDS:
                raise KnowledgeGovernanceSnapshotError(f"{fact_id} 的 kind 无效")
            if fact.get("lifecycleState") not in LIFECYCLE_STATES:
                raise KnowledgeGovernanceSnapshotError(f"{fact_id} 的 lifecycleState 无效")
            page_keys = _sorted_unique_texts(fact.get("affectedPageKeys"), "affectedPageKeys", fact_id)
            if any(PAGE_KEY_PATTERN.fullmatch(item) is None for item in page_keys):
                raise KnowledgeGovernanceSnapshotError(f"{fact_id} 的 affectedPageKeys 无效")
            fact_ids.append(fact_id)
        if fact_ids != sorted(set(fact_ids)):
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 governanceFacts 必须按 id 排序且不重复")
        fact_count += len(fact_ids)

        _sorted_unique_texts(raw.get("closedBatchIds"), "closedBatchIds", snapshot_id)
        raw_baselines = raw.get("baselineLifecycle")
        if not isinstance(raw_baselines, list):
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 baselineLifecycle 必须是数组")
        baseline_ids: list[str] = []
        for baseline in raw_baselines:
            if not isinstance(baseline, dict):
                raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 baselineLifecycle 项必须是对象")
            baseline_id = _text(baseline.get("id"), "id", f"{snapshot_id} 的基线")
            if baseline.get("state") not in BASELINE_STATES:
                raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的基线 {baseline_id} 状态无效")
            baseline_ids.append(baseline_id)
        if baseline_ids != sorted(set(baseline_ids)):
            raise KnowledgeGovernanceSnapshotError(f"{snapshot_id} 的 baselineLifecycle 必须按 id 排序且不重复")

    return {"snapshots": len(raw_snapshots), "facts": fact_count, "months": len(seen_months)}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        summary = validate_registry(load_json(args.registry))
    except KnowledgeGovernanceSnapshotError as exc:
        print(f"知识治理月度快照检查失败：{exc}", file=sys.stderr)
        return 1
    print(
        f"知识治理月度快照检查通过：{summary['months']} 个月份，"
        f"{summary['snapshots']} 个正式快照，{summary['facts']} 条事实引用。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

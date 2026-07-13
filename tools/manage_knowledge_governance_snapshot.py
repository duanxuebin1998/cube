#!/usr/bin/env python3
"""从干净 Git revision 的本机派生快照原子追加知识治理月度历史。"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any
from zoneinfo import ZoneInfo, ZoneInfoNotFoundError

from check_knowledge_governance_snapshots import (
    DEFAULT_REGISTRY,
    KnowledgeGovernanceSnapshotError,
    load_json,
    validate_registry,
)


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_QUEUE = ROOT / "docs-site" / "public" / "data" / "governance-queue.json"
DEFAULT_BATCHES = ROOT / "docs-site" / "public" / "data" / "validation-batches.json"
DEFAULT_BASELINES = ROOT / "docs-site" / "public" / "data" / "delivery-baseline-lifecycle.json"
DEFAULT_DELIVERY = ROOT / "docs-site" / "public" / "data" / "current-delivery-evidence.json"
DEFAULT_EVIDENCE = ROOT / "docs" / "00_程序流程导航" / "流程证据清单.json"
SCOPE = (
    "docs/00_程序流程导航",
    "docs-site/scripts",
    "docs-site/src",
    "docs-site/astro.config.mjs",
    "tools",
)


def shanghai_timezone():  # noqa: ANN201
    """优先使用 IANA 时区；Windows 未安装 tzdata 时回退到当前中国标准时间。"""
    try:
        return ZoneInfo("Asia/Shanghai")
    except ZoneInfoNotFoundError:
        return timezone(timedelta(hours=8), name="Asia/Shanghai")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_REGISTRY)
    parser.add_argument("--queue", type=Path, default=DEFAULT_QUEUE)
    parser.add_argument("--batches", type=Path, default=DEFAULT_BATCHES)
    parser.add_argument("--baselines", type=Path, default=DEFAULT_BASELINES)
    parser.add_argument("--delivery", type=Path, default=DEFAULT_DELIVERY)
    parser.add_argument("--evidence", type=Path, default=DEFAULT_EVIDENCE)
    parser.add_argument("--recorded-at", required=True)
    parser.add_argument("--explanation", required=True)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def git_text(*args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=ROOT, text=True, encoding="utf-8", errors="replace",
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if result.returncode != 0:
        raise KnowledgeGovernanceSnapshotError(result.stderr.strip() or "Git 命令失败")
    return result.stdout.strip()


def require_clean_revision(delivery: dict[str, Any]) -> str:
    revision = git_text("rev-parse", "HEAD")
    dirty = git_text("status", "--porcelain=v1", "--", *SCOPE)
    if dirty:
        raise KnowledgeGovernanceSnapshotError("目标文档、站点或工具范围存在未提交改动，禁止保存月度历史")
    git_data = delivery.get("git")
    if not isinstance(git_data, dict) or git_data.get("revision") != revision or git_data.get("dirty") is not False:
        raise KnowledgeGovernanceSnapshotError("当前交付证据快照与干净 HEAD 不一致，请先重新同步站点")
    return revision


def build_snapshot(
    *, recorded_at: str, explanation: str, revision: str, queue: dict[str, Any],
    batches: dict[str, Any], baselines: dict[str, Any], evidence: dict[str, Any], root: Path = ROOT,
) -> dict[str, Any]:
    try:
        instant = datetime.fromisoformat(recorded_at.replace("Z", "+00:00"))
    except ValueError as exc:
        raise KnowledgeGovernanceSnapshotError("--recorded-at 必须是包含时区的 ISO 8601 时间") from exc
    if instant.tzinfo is None:
        raise KnowledgeGovernanceSnapshotError("--recorded-at 必须包含时区")
    month = instant.astimezone(shanghai_timezone()).strftime("%Y-%m")
    explanation_path = explanation.replace("\\", "/")
    if not explanation_path.startswith("docs/") or not (root / explanation_path).is_file():
        raise KnowledgeGovernanceSnapshotError("--explanation 必须指向存在的 docs/ 正式说明")
    if queue.get("schemaVersion") != 2 or queue.get("kind") != "cube-governance-queue":
        raise KnowledgeGovernanceSnapshotError("治理队列快照格式无效")
    if batches.get("kind") != "cube-validation-batches" or not isinstance(batches.get("batches"), list):
        raise KnowledgeGovernanceSnapshotError("验证批次快照格式无效")
    if baselines.get("kind") != "cube-delivery-baseline-lifecycle" or not isinstance(baselines.get("baselines"), list):
        raise KnowledgeGovernanceSnapshotError("基线生命周期快照格式无效")
    policy = evidence.get("governancePolicy")
    pages = evidence.get("pages")
    if not isinstance(policy, dict) or not isinstance(pages, list):
        raise KnowledgeGovernanceSnapshotError("流程证据清单缺少治理策略或页面")
    reference = datetime.fromisoformat(f"{evidence['updated']}T00:00:00+00:00")
    window_days = policy.get("reviewWindowDays")
    if not isinstance(window_days, int) or window_days <= 0:
        raise KnowledgeGovernanceSnapshotError("流程核对窗口无效")
    stale_page_keys = sorted(
        page["pageKey"] for page in pages
        if (reference - datetime.fromisoformat(f"{page['reviewedAt']}T00:00:00+00:00")).days > window_days
    )
    facts = []
    for item in queue.get("items", []):
        affected = sorted(value for value in item.get("affected", []) if isinstance(value, str) and value.startswith(("cpu2_", "cpu3_")))
        facts.append({
            "id": item["governanceKey"],
            "factFingerprint": item["factFingerprint"],
            "ownerId": item["ownerId"],
            "kind": item["kind"],
            "lifecycleState": item["lifecycle"]["state"],
            "affectedPageKeys": affected,
        })
    closed_batch_ids = sorted(
        batch["id"] for batch in batches["batches"]
        if (batch.get("state") or batch.get("currentState")) == "closed"
    )
    baseline_lifecycle = sorted(
        ({"id": item["id"], "state": item["state"]} for item in baselines["baselines"]),
        key=lambda item: item["id"],
    )
    return {
        "id": f"kgs-{month}",
        "month": month,
        "recordedAt": recorded_at,
        "gitRevision": revision,
        "explanationPath": explanation_path,
        "review": {
            "windowDays": window_days,
            "totalPages": len(pages),
            "stalePageKeys": stale_page_keys,
        },
        "governanceFacts": sorted(facts, key=lambda item: item["id"]),
        "closedBatchIds": closed_batch_ids,
        "baselineLifecycle": baseline_lifecycle,
    }


def atomic_write(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="\n", dir=path.parent, delete=False) as handle:
        json.dump(value, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        temporary = Path(handle.name)
    os.replace(temporary, path)


def main() -> int:
    args = parse_args()
    try:
        registry = load_json(args.registry)
        validate_registry(registry)
        delivery = load_json(args.delivery)
        revision = require_clean_revision(delivery)
        snapshot = build_snapshot(
            recorded_at=args.recorded_at,
            explanation=args.explanation,
            revision=revision,
            queue=load_json(args.queue),
            batches=load_json(args.batches),
            baselines=load_json(args.baselines),
            evidence=load_json(args.evidence),
        )
        if any(item.get("month") == snapshot["month"] for item in registry["snapshots"]):
            raise KnowledgeGovernanceSnapshotError(f"{snapshot['month']} 已有正式快照，禁止覆盖")
        updated = {**registry, "snapshots": [*registry["snapshots"], snapshot]}
        validate_registry(updated)
        if not args.dry_run:
            atomic_write(args.registry, updated)
    except KnowledgeGovernanceSnapshotError as exc:
        print(f"知识治理月度快照追加失败：{exc}")
        return 1
    action = "已校验，未写入" if args.dry_run else "已原子追加"
    print(f"{action}：{snapshot['id']} · {snapshot['gitRevision']}。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

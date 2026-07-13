#!/usr/bin/env python3
"""向正式清单原子追加一条由调用者显式提供的真实验证运行。"""

from __future__ import annotations

import argparse
import hashlib
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
from check_validation_runs import (
    DEFAULT_RUN_REGISTRY,
    build_validation_run_report,
    load_work_package_cases,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_RUN_REGISTRY)
    parser.add_argument("--identity-registry", type=Path, default=DEFAULT_IDENTITY_REGISTRY)
    parser.add_argument("--id", required=True, help="调用者分配的稳定 run-... ID")
    parser.add_argument("--evidence-id", required=True)
    parser.add_argument("--case-id", required=True)
    parser.add_argument("--layer", required=True, choices=("automatic", "bench", "device", "field"))
    parser.add_argument("--outcome", required=True, choices=("passed", "failed", "partial", "blocked"))
    parser.add_argument("--executed-at", required=True, help="含时区的 ISO 8601 时间")
    parser.add_argument("--operator", required=True)
    parser.add_argument("--environment", required=True)
    parser.add_argument("--cpu2")
    parser.add_argument("--cpu3")
    parser.add_argument("--revision", required=True)
    parser.add_argument("--summary", required=True)
    parser.add_argument("--record", action="append", required=True, type=Path)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _record_entry(record_path: Path) -> dict[str, str]:
    absolute = record_path if record_path.is_absolute() else ROOT / record_path
    absolute = absolute.resolve()
    relative = absolute.relative_to(ROOT.resolve()).as_posix()
    return {"path": relative, "sha256": hashlib.sha256(absolute.read_bytes()).hexdigest()}


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


def main() -> int:
    args = parse_args()
    identity = validate_identity_registry(load_json(args.identity_registry), root=ROOT)
    packages = load_work_package_cases(identity)
    registry = load_json(args.registry)
    baseline = {key: value for key, value in (("cpu2", args.cpu2), ("cpu3", args.cpu3)) if value}
    run = {
        "id": args.id,
        "evidenceId": args.evidence_id,
        "caseId": args.case_id,
        "layer": args.layer,
        "outcome": args.outcome,
        "executedAt": args.executed_at,
        "operator": args.operator,
        "environment": args.environment,
        "firmwareBaseline": baseline,
        "decisionRevision": args.revision,
        "summary": args.summary,
        "records": [_record_entry(record) for record in args.record],
    }
    candidate = {**registry, "runs": [*registry.get("runs", []), run]}
    report = build_validation_run_report(
        candidate, packages, enforce_declared_status=False
    )
    print(json.dumps(run, ensure_ascii=False, indent=2))
    if args.dry_run:
        print("dry-run：登记合法，未修改正式清单。")
        return 0
    _atomic_write_json(args.registry, candidate)
    print(
        f"已追加 {run['id']}；当前 {report['summary']['registeredRuns']} 次运行，"
        f"{report['summary']['pendingCases']} 个用例待执行。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

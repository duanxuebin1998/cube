#!/usr/bin/env python3
"""校验真实验证运行登记清单，并从用例级记录派生工作包执行状态。"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from datetime import datetime
from pathlib import Path
from typing import Any

from check_delivery_evidence_coverage import (
    DEFAULT_IDENTITY_REGISTRY,
    ROOT,
    load_json,
    validate_identity_registry,
)
from check_validation_work_packages import (
    WORK_PACKAGE_PREFIX,
    parse_work_package,
)


DEFAULT_RUN_REGISTRY = ROOT / "docs" / "00_程序流程导航" / "验证运行登记清单.json"
DEFAULT_SNAPSHOT = ROOT / "docs-site" / "public" / "data" / "validation-runs.json"
ALLOWED_OUTCOMES = ("passed", "failed", "partial", "blocked")
ALLOWED_LAYERS = ("automatic", "bench", "device", "field")
VERSION_PATTERN = re.compile(r"V\d+\.\d+\.\d+\.\d+")
REVISION_PATTERN = re.compile(r"[0-9a-f]{40}")
RUN_ID_PATTERN = re.compile(r"run-[a-z0-9][a-z0-9-]{2,}")


class ValidationRunError(ValueError):
    """表示真实运行登记不满足追加式审计契约。"""


def _non_empty_text(value: Any, field: str, run_id: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValidationRunError(f"{run_id} 的 {field} 必须是非空字符串")
    return value.strip()


def _normalize_repo_path(value: str) -> str:
    return value.replace("\\", "/").removeprefix("./")


def _resolve_docs_record(relative_path: str, root: Path) -> Path:
    normalized = _normalize_repo_path(relative_path)
    if not normalized.startswith("docs/") or normalized.lower().endswith(".pdf"):
        raise ValidationRunError(f"原始记录必须是 docs/ 下的非 PDF 文件：{relative_path}")
    candidate = (root / Path(normalized)).resolve()
    docs_root = (root / "docs").resolve()
    if docs_root not in candidate.parents or not candidate.is_file():
        raise ValidationRunError(f"原始记录不存在或路径越界：{relative_path}")
    return candidate


def load_work_package_cases(
    identity_registry: dict[str, Any], *, root: Path = ROOT
) -> dict[str, dict[str, Any]]:
    """从正式证据身份与 Markdown 矩阵读取稳定用例身份。"""

    packages: dict[str, dict[str, Any]] = {}
    for document in identity_registry["documents"]:
        relative_path = _normalize_repo_path(document["path"])
        if not relative_path.startswith(WORK_PACKAGE_PREFIX):
            continue
        if document["id"] in packages:
            raise ValidationRunError(f"工作包证据 ID 重复：{document['id']}")
        absolute_path = (root / Path(relative_path)).resolve()
        docs_root = (root / "docs").resolve()
        if docs_root not in absolute_path.parents or not absolute_path.is_file():
            raise ValidationRunError(f"工作包不存在或路径越界：{relative_path}")
        parsed = parse_work_package(absolute_path.read_text(encoding="utf-8"), relative_path)
        cases = {case["caseId"]: case for case in parsed["matrixCases"]}
        packages[document["id"]] = {
            "evidenceId": document["id"],
            "path": relative_path,
            "label": parsed["label"],
            "declaredResultStatus": parsed["result"]["status"],
            "firmwareBaseline": parsed["result"]["firmwareBaseline"],
            "cases": cases,
        }
    if not packages:
        raise ValidationRunError("验证证据登记清单中没有正式验证工作包")
    return packages


def validate_run_registry(
    registry: dict[str, Any],
    packages: dict[str, dict[str, Any]],
    *,
    root: Path = ROOT,
) -> list[dict[str, Any]]:
    """严格校验运行身份、引用、基线、原始记录及摘要字段。"""

    if registry.get("schemaVersion") != 1:
        raise ValidationRunError("验证运行登记清单 schemaVersion 必须为 1")
    if registry.get("kind") != "cube-validation-run-registry":
        raise ValidationRunError("验证运行登记清单 kind 无效")
    if not isinstance(registry.get("description"), str) or not registry["description"].strip():
        raise ValidationRunError("验证运行登记清单 description 必须是非空字符串")
    raw_runs = registry.get("runs")
    if not isinstance(raw_runs, list):
        raise ValidationRunError("验证运行登记清单 runs 必须是数组")

    run_ids: set[str] = set()
    normalized_runs: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_runs):
        if not isinstance(raw, dict):
            raise ValidationRunError(f"第 {index + 1} 条运行必须是对象")
        run_id = _non_empty_text(raw.get("id"), "id", f"第 {index + 1} 条运行")
        if RUN_ID_PATTERN.fullmatch(run_id) is None:
            raise ValidationRunError(f"运行 ID 无效：{run_id}")
        if run_id in run_ids:
            raise ValidationRunError(f"运行 ID 重复：{run_id}")
        run_ids.add(run_id)

        evidence_id = _non_empty_text(raw.get("evidenceId"), "evidenceId", run_id)
        package = packages.get(evidence_id)
        if package is None:
            raise ValidationRunError(f"{run_id} 引用了未知工作包：{evidence_id}")
        case_id = _non_empty_text(raw.get("caseId"), "caseId", run_id)
        planned_case = package["cases"].get(case_id)
        if planned_case is None:
            raise ValidationRunError(f"{run_id} 引用了工作包中不存在的用例：{case_id}")
        layer = _non_empty_text(raw.get("layer"), "layer", run_id)
        if layer not in ALLOWED_LAYERS or layer != planned_case["layer"]:
            raise ValidationRunError(
                f"{run_id} 的 layer 与矩阵不一致：登记 {layer}，计划 {planned_case['layer']}"
            )
        outcome = _non_empty_text(raw.get("outcome"), "outcome", run_id)
        if outcome not in ALLOWED_OUTCOMES:
            raise ValidationRunError(f"{run_id} 的 outcome 无效：{outcome}")

        executed_at = _non_empty_text(raw.get("executedAt"), "executedAt", run_id)
        try:
            parsed_time = datetime.fromisoformat(executed_at)
        except ValueError as exc:
            raise ValidationRunError(f"{run_id} 的 executedAt 不是有效 ISO 8601 时间") from exc
        if parsed_time.tzinfo is None or parsed_time.utcoffset() is None:
            raise ValidationRunError(f"{run_id} 的 executedAt 必须包含时区")

        operator = _non_empty_text(raw.get("operator"), "operator", run_id)
        environment = _non_empty_text(raw.get("environment"), "environment", run_id)
        summary = _non_empty_text(raw.get("summary"), "summary", run_id)
        revision = _non_empty_text(raw.get("decisionRevision"), "decisionRevision", run_id)
        if REVISION_PATTERN.fullmatch(revision) is None:
            raise ValidationRunError(f"{run_id} 的 decisionRevision 必须是 40 位小写 Git revision")

        baseline = raw.get("firmwareBaseline")
        if not isinstance(baseline, dict) or not baseline:
            raise ValidationRunError(f"{run_id} 的 firmwareBaseline 必须至少声明 cpu2 或 cpu3")
        if set(baseline) - {"cpu2", "cpu3"}:
            raise ValidationRunError(f"{run_id} 的 firmwareBaseline 含未知 CPU 字段")
        for cpu, version in baseline.items():
            if not isinstance(version, str) or VERSION_PATTERN.fullmatch(version) is None:
                raise ValidationRunError(f"{run_id} 的 firmwareBaseline.{cpu} 格式无效")
            planned_version = package["firmwareBaseline"].get(cpu)
            if planned_version is not None and version != planned_version:
                raise ValidationRunError(
                    f"{run_id} 的 {cpu} 基线 {version} 与工作包 {planned_version} 不一致"
                )

        records = raw.get("records")
        if not isinstance(records, list) or not records:
            raise ValidationRunError(f"{run_id} 必须至少引用一份原始记录")
        normalized_records: list[dict[str, str]] = []
        seen_paths: set[str] = set()
        for record in records:
            if not isinstance(record, dict):
                raise ValidationRunError(f"{run_id} 的 records 项必须是对象")
            record_path = _non_empty_text(record.get("path"), "records.path", run_id)
            record_path = _normalize_repo_path(record_path)
            if record_path in seen_paths:
                raise ValidationRunError(f"{run_id} 重复引用原始记录：{record_path}")
            seen_paths.add(record_path)
            record_file = _resolve_docs_record(record_path, root)
            actual_sha256 = hashlib.sha256(record_file.read_bytes()).hexdigest()
            declared_sha256 = _non_empty_text(record.get("sha256"), "records.sha256", run_id)
            if declared_sha256 != actual_sha256:
                raise ValidationRunError(f"{run_id} 的原始记录 SHA-256 不一致：{record_path}")
            normalized_records.append({"path": record_path, "sha256": declared_sha256})

        normalized_runs.append(
            {
                "id": run_id,
                "evidenceId": evidence_id,
                "caseId": case_id,
                "layer": layer,
                "outcome": outcome,
                "executedAt": executed_at,
                "operator": operator,
                "environment": environment,
                "firmwareBaseline": baseline,
                "decisionRevision": revision,
                "summary": summary,
                "records": normalized_records,
            }
        )
    return normalized_runs


def _derived_package_status(case_states: list[str | None]) -> str:
    if not any(case_states):
        return "pending"
    if any(state == "failed" for state in case_states):
        return "failed"
    if all(state == "passed" for state in case_states):
        return "passed"
    return "partial"


def build_validation_run_report(
    registry: dict[str, Any],
    packages: dict[str, dict[str, Any]],
    *,
    root: Path = ROOT,
    enforce_declared_status: bool = True,
) -> dict[str, Any]:
    """按登记顺序保留历史，以每个用例最后一次运行派生当前状态。"""

    runs = validate_run_registry(registry, packages, root=root)
    latest_by_case: dict[tuple[str, str], dict[str, Any]] = {}
    for run in runs:
        latest_by_case[(run["evidenceId"], run["caseId"])] = run

    package_reports: list[dict[str, Any]] = []
    layer_summary = {
        layer: {"plannedCases": 0, "casesWithRuns": 0, "pendingCases": 0}
        for layer in ALLOWED_LAYERS
    }
    current_outcome_counts = {outcome: 0 for outcome in ALLOWED_OUTCOMES}
    for evidence_id, package in sorted(packages.items()):
        case_reports: list[dict[str, Any]] = []
        for case_id, planned_case in package["cases"].items():
            latest = latest_by_case.get((evidence_id, case_id))
            layer = planned_case["layer"]
            layer_summary[layer]["plannedCases"] += 1
            if latest is None:
                layer_summary[layer]["pendingCases"] += 1
            else:
                layer_summary[layer]["casesWithRuns"] += 1
                current_outcome_counts[latest["outcome"]] += 1
            case_reports.append(
                {
                    "caseId": case_id,
                    "layer": layer,
                    "currentOutcome": latest["outcome"] if latest else "pending",
                    "latestRunId": latest["id"] if latest else None,
                    "runCount": sum(
                        run["evidenceId"] == evidence_id and run["caseId"] == case_id
                        for run in runs
                    ),
                }
            )
        derived_status = _derived_package_status(
            [
                latest_by_case.get((evidence_id, case_id), {}).get("outcome")
                for case_id in package["cases"]
            ]
        )
        if enforce_declared_status and package["declaredResultStatus"] != derived_status:
            raise ValidationRunError(
                f"{package['path']} 的 cube-validation-result.status 为 "
                f"{package['declaredResultStatus']}，运行台账派生为 {derived_status}"
            )
        package_reports.append(
            {
                "evidenceId": evidence_id,
                "label": package["label"],
                "path": package["path"],
                "declaredResultStatus": package["declaredResultStatus"],
                "derivedResultStatus": derived_status,
                "plannedCases": len(case_reports),
                "casesWithRuns": sum(case["runCount"] > 0 for case in case_reports),
                "runCount": sum(run["evidenceId"] == evidence_id for run in runs),
                "cases": case_reports,
            }
        )

    planned_cases = sum(item["plannedCases"] for item in package_reports)
    cases_with_runs = len(latest_by_case)
    package_status_counts = {
        status: sum(item["derivedResultStatus"] == status for item in package_reports)
        for status in ("pending", "partial", "passed", "failed")
    }
    return {
        "schemaVersion": 1,
        "kind": "cube-validation-runs",
        "state": "no-runs" if not runs else "runs-recorded",
        "summary": {
            "workPackages": len(package_reports),
            "plannedCases": planned_cases,
            "registeredRuns": len(runs),
            "casesWithRuns": cases_with_runs,
            "pendingCases": planned_cases - cases_with_runs,
            "currentOutcomes": current_outcome_counts,
            "packageStatuses": package_status_counts,
            "layers": layer_summary,
        },
        "packages": package_reports,
        "runs": runs,
        "method": {
            "workPackageCasesFromMarkdownMatrix": True,
            "appendOrderDefinesLatestRun": True,
            "recordFilesMustRemainUnderDocs": True,
            "recordIntegrityUsesSha256": True,
            "emptyRegistryIsNotPass": True,
            "packageStatusMustMatchDerivedRuns": True,
            "manualDeliveryDecisionRequired": True,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, default=DEFAULT_RUN_REGISTRY)
    parser.add_argument("--identity-registry", type=Path, default=DEFAULT_IDENTITY_REGISTRY)
    parser.add_argument("--snapshot", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        identity = validate_identity_registry(load_json(args.identity_registry), root=ROOT)
        packages = load_work_package_cases(identity)
        report = build_validation_run_report(load_json(args.registry), packages)
        if args.snapshot is not None and load_json(args.snapshot) != report:
            raise ValidationRunError(f"站点验证运行快照与正式资料不一致：{args.snapshot}")
    except (ValidationRunError, ValueError, KeyError, OSError, json.JSONDecodeError) as exc:
        print(f"验证运行台账检查失败：{exc}", file=sys.stderr)
        return 1
    print(
        "验证运行台账检查通过："
        f"{report['summary']['plannedCases']} 个计划用例，"
        f"{report['summary']['registeredRuns']} 次真实运行，"
        f"{report['summary']['pendingCases']} 个用例待执行。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

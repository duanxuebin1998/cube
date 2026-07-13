#!/usr/bin/env python3
"""校验验证工作包的稳定身份、可执行矩阵、结果元数据和缺口关闭关系。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

from check_delivery_evidence_coverage import (
    COVERAGE_LAYERS,
    DEFAULT_EVIDENCE_MANIFEST,
    DEFAULT_IDENTITY_REGISTRY,
    ROOT,
    load_json,
    validate_identity_registry,
)
from check_delivery_evidence_gaps import DEFAULT_GAP_REGISTRY, expected_gap_id
from check_validation_result_metadata import parse_result_block, validate_result_metadata


DEFAULT_SNAPSHOT = ROOT / "docs-site" / "public" / "data" / "validation-work-packages.json"
WORK_PACKAGE_PREFIX = "docs/00_程序流程导航/验证工作包/"
REQUIRED_MATRIX_COLUMNS = (
    "ID",
    "层",
    "前置条件",
    "操作步骤",
    "观测位置",
    "停止条件",
    "预期结果",
    "通过标准",
    "状态",
)
LAYER_LABELS = {
    "automatic": "自动",
    "bench": "台架",
    "device": "实机",
    "field": "现场",
}


class WorkPackageValidationError(ValueError):
    """表示验证工作包不满足正式契约。"""


def _table_cells(line: str) -> list[str]:
    """读取简单 Markdown 表格行。"""

    stripped = line.strip()
    if not stripped.startswith("|") or not stripped.endswith("|"):
        return []
    return [cell.strip() for cell in stripped[1:-1].split("|")]


def _is_separator(cells: list[str]) -> bool:
    return bool(cells) and all(re.fullmatch(r":?-{3,}:?", cell) for cell in cells)


def parse_work_package(content: str, source: str) -> dict[str, Any]:
    """解析标题、矩阵和结构化结果；任何关键字段缺失都失败关闭。"""

    title_match = re.search(r"^#\s+(.+?)\s*$", content, flags=re.MULTILINE)
    if title_match is None:
        raise WorkPackageValidationError(f"{source} 缺少一级标题")

    payload, parse_errors = parse_result_block(content)
    if payload is None:
        detail = "；".join(parse_errors) if parse_errors else "缺少 cube-validation-result 元数据"
        raise WorkPackageValidationError(f"{source}：{detail}")
    metadata_errors = validate_result_metadata(payload)
    if metadata_errors:
        raise WorkPackageValidationError(f"{source}：{'；'.join(metadata_errors)}")

    lines = content.splitlines()
    header_index: int | None = None
    for index, line in enumerate(lines):
        cells = _table_cells(line)
        if cells and all(column in cells for column in REQUIRED_MATRIX_COLUMNS):
            header_index = index
            break
    if header_index is None:
        raise WorkPackageValidationError(f"{source} 缺少完整的可执行测试矩阵表头")

    rows: list[list[str]] = []
    for line in lines[header_index + 1 :]:
        cells = _table_cells(line)
        if not cells:
            if rows:
                break
            continue
        if _is_separator(cells):
            continue
        rows.append(cells)
    if not rows:
        raise WorkPackageValidationError(f"{source} 的可执行测试矩阵没有用例")

    layer_counts = {
        layer: sum(len(row) > 1 and row[1] == label for row in rows)
        for layer, label in LAYER_LABELS.items()
    }
    missing_layers = [layer for layer, count in layer_counts.items() if count == 0]
    if missing_layers:
        raise WorkPackageValidationError(f"{source} 的测试矩阵缺少验证层：{missing_layers}")
    if "## 通过标准" not in content:
        raise WorkPackageValidationError(f"{source} 缺少独立的通过标准章节")

    label_to_layer = {label: layer for layer, label in LAYER_LABELS.items()}
    matrix_cases: list[dict[str, str]] = []
    case_ids: set[str] = set()
    for row in rows:
        if len(row) != len(REQUIRED_MATRIX_COLUMNS):
            raise WorkPackageValidationError(f"{source} 的测试矩阵存在列数不一致用例")
        case_id = row[0]
        if not re.fullmatch(r"[A-Z0-9]+(?:-[A-Z0-9]+)+", case_id):
            raise WorkPackageValidationError(f"{source} 的用例 ID 无效：{case_id}")
        if case_id in case_ids:
            raise WorkPackageValidationError(f"{source} 的用例 ID 重复：{case_id}")
        case_ids.add(case_id)
        layer = label_to_layer.get(row[1])
        if layer is None:
            raise WorkPackageValidationError(f"{source} 的用例 {case_id} 验证层无效：{row[1]}")
        if row[8] != "待执行":
            raise WorkPackageValidationError(
                f"{source} 的计划矩阵状态必须保持待执行，真实结果应登记到运行台账：{case_id}"
            )
        matrix_cases.append({"caseId": case_id, "layer": layer, "plannedStatus": "pending"})

    return {
        "label": title_match.group(1).strip(),
        "result": payload,
        "matrixCaseCount": len(rows),
        "layerCaseCounts": layer_counts,
        "matrixCases": matrix_cases,
    }


def build_work_package_report(
    identity_registry: dict[str, Any],
    evidence_manifest: dict[str, Any],
    gap_registry: dict[str, Any],
    *,
    root: Path = ROOT,
) -> dict[str, Any]:
    """从三份正式清单和工作包正文构建单一机器报告。"""

    if evidence_manifest.get("schemaVersion") != 2 or not isinstance(evidence_manifest.get("pages"), list):
        raise WorkPackageValidationError("流程证据清单格式无效")
    if gap_registry.get("schemaVersion") != 1 or not isinstance(gap_registry.get("gaps"), list):
        raise WorkPackageValidationError("交付证据缺口处置清单格式无效")

    page_paths = {
        page["pageKey"]: [str(value).replace("\\", "/") for value in page.get("validationPaths", [])]
        for page in evidence_manifest["pages"]
    }
    gaps_by_page = {gap.get("pageKey"): gap for gap in gap_registry["gaps"]}
    packages: list[dict[str, Any]] = []
    for document in identity_registry["documents"]:
        relative_path = document["path"].replace("\\", "/")
        if not relative_path.startswith(WORK_PACKAGE_PREFIX):
            continue
        if document["documentKind"] != "validation-plan":
            raise WorkPackageValidationError(f"{document['id']} 的 documentKind 必须为 validation-plan")
        if document["declaredLayers"] != list(COVERAGE_LAYERS):
            raise WorkPackageValidationError(f"{document['id']} 必须声明自动、台架、实机、现场四层")

        page_keys = sorted(page_key for page_key, paths in page_paths.items() if relative_path in paths)
        if len(page_keys) != 1:
            raise WorkPackageValidationError(f"{document['id']} 必须且只能作为一个核心流程的专项资料")
        page_key = page_keys[0]
        gap = gaps_by_page.get(page_key)
        if gap is None or gap.get("id") != expected_gap_id(page_key):
            raise WorkPackageValidationError(f"{document['id']} 缺少匹配的稳定缺口记录")
        events = gap.get("events")
        if not isinstance(events, list) or not events:
            raise WorkPackageValidationError(f"{gap['id']} 缺少生命周期事件")
        latest = events[-1]
        if latest.get("state") != "closed":
            raise WorkPackageValidationError(f"{gap['id']} 已建立工作包但尚未关闭专项关系")
        if document["id"] not in latest.get("relatedEvidenceIds", []):
            raise WorkPackageValidationError(f"{gap['id']} 关闭事件未关联 {document['id']}")
        if latest.get("explanationPath") != relative_path:
            raise WorkPackageValidationError(f"{gap['id']} 关闭说明必须指向对应工作包")

        absolute_path = (root / Path(relative_path)).resolve()
        docs_root = (root / "docs").resolve()
        if docs_root not in absolute_path.parents or not absolute_path.is_file():
            raise WorkPackageValidationError(f"工作包不存在或路径越界：{relative_path}")
        try:
            content = absolute_path.read_text(encoding="utf-8")
        except UnicodeDecodeError as exc:
            raise WorkPackageValidationError(f"工作包不是有效 UTF-8：{relative_path}") from exc
        for stable_id in (gap["id"], page_key, document["id"]):
            if f"`{stable_id}`" not in content:
                raise WorkPackageValidationError(f"{relative_path} 未显式声明稳定标识 {stable_id}")
        parsed = parse_work_package(content, relative_path)
        result = parsed["result"]
        packages.append(
            {
                "evidenceId": document["id"],
                "gapId": gap["id"],
                "pageKey": page_key,
                "path": relative_path,
                "label": parsed["label"],
                "declaredLayers": document["declaredLayers"],
                "resultStatus": result["status"],
                "executedAt": result["executedAt"],
                "owner": latest["owner"],
                "dispositionState": latest["state"],
                "eventCount": len(events),
                "matrixCaseCount": parsed["matrixCaseCount"],
                "layerCaseCounts": parsed["layerCaseCounts"],
            }
        )

    if not packages:
        raise WorkPackageValidationError("验证证据登记清单中没有正式验证工作包")
    packages.sort(key=lambda item: item["pageKey"])
    statuses = ("pending", "partial", "passed", "failed")
    status_counts = {status: sum(item["resultStatus"] == status for item in packages) for status in statuses}
    return {
        "schemaVersion": 1,
        "kind": "cube-validation-work-packages",
        "state": "pending-execution" if status_counts["pending"] else "results-recorded",
        "summary": {
            "packages": len(packages),
            "flows": len({item["pageKey"] for item in packages}),
            "relationshipsClosed": sum(item["dispositionState"] == "closed" for item in packages),
            **status_counts,
            "layerPackages": {
                layer: sum(layer in item["declaredLayers"] for item in packages)
                for layer in COVERAGE_LAYERS
            },
        },
        "packages": packages,
        "method": {
            "stableIdentityFromRegistry": True,
            "flowRelationFromEvidenceManifest": True,
            "gapClosureFromAppendOnlyRegistry": True,
            "executionResultFromDocumentMetadata": True,
            "fourLayerMatrixRequired": True,
            "relationshipClosureIsNotTestPass": True,
            "naturalLanguageInference": False,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--identity-registry", type=Path, default=DEFAULT_IDENTITY_REGISTRY)
    parser.add_argument("--evidence-manifest", type=Path, default=DEFAULT_EVIDENCE_MANIFEST)
    parser.add_argument("--gap-registry", type=Path, default=DEFAULT_GAP_REGISTRY)
    parser.add_argument("--snapshot", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        identity = validate_identity_registry(load_json(args.identity_registry), root=ROOT)
        report = build_work_package_report(
            identity,
            load_json(args.evidence_manifest),
            load_json(args.gap_registry),
        )
        if args.snapshot is not None and load_json(args.snapshot) != report:
            raise WorkPackageValidationError(f"站点验证工作包快照与正式资料不一致：{args.snapshot}")
    except (WorkPackageValidationError, ValueError, KeyError, OSError, json.JSONDecodeError) as exc:
        print(f"验证工作包检查失败：{exc}", file=sys.stderr)
        return 1
    print(
        "验证工作包检查通过："
        f"{report['summary']['packages']} 份工作包，"
        f"{report['summary']['relationshipsClosed']} 份专项关系已闭环，"
        f"{report['summary']['pending']} 份待执行"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

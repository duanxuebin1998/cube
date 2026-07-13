#!/usr/bin/env python3
"""校验稳定验证证据身份，并生成流程、链路与已选基线覆盖快照。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path
from typing import Any

from check_delivery_evidence_baselines import (
    DEFAULT_REGISTRY as DEFAULT_BASELINE_REGISTRY,
    ROOT,
    check_registry as check_baseline_registry,
    load_json,
)


DEFAULT_IDENTITY_REGISTRY = ROOT / "docs" / "00_程序流程导航" / "验证证据登记清单.json"
DEFAULT_EVIDENCE_MANIFEST = ROOT / "docs" / "00_程序流程导航" / "流程证据清单.json"
DEFAULT_ROUTE_MANIFEST = ROOT / "docs" / "00_程序流程导航" / "业务链路清单.json"
DEFAULT_SNAPSHOT = ROOT / "docs-site" / "public" / "data" / "delivery-evidence-coverage.json"
EVIDENCE_ID_PATTERN = re.compile(r"^ev-[a-z0-9][a-z0-9-]{1,78}$")
DOCUMENT_KINDS = {"validation-plan", "governance-record"}
COVERAGE_LAYERS = ("automatic", "bench", "device", "field")


class CoverageValidationError(ValueError):
    """表示证据身份或覆盖关系不满足正式契约。"""


def _repo_markdown_path(value: Any, source: str, *, root: Path) -> str:
    if (
        not isinstance(value, str)
        or not value.startswith("docs/")
        or not value.lower().endswith(".md")
        or ".." in Path(value).parts
    ):
        raise CoverageValidationError(f"{source} 必须指向 docs/ 下的 Markdown")
    docs_root = (root / "docs").resolve()
    candidate = (root / Path(value)).resolve()
    if docs_root not in candidate.parents or not candidate.is_file():
        raise CoverageValidationError(f"{source} 指向的正式资料不存在：{value}")
    return value.replace("\\", "/")


def validate_identity_registry(
    registry: dict[str, Any],
    source: str = "验证证据登记清单",
    *,
    root: Path = ROOT,
) -> dict[str, Any]:
    if registry.get("schemaVersion") != 1:
        raise CoverageValidationError(f"{source} 的 schemaVersion 必须为 1")
    if registry.get("kind") != "cube-validation-evidence-identity-registry":
        raise CoverageValidationError(f"{source} 的 kind 无效")
    documents = registry.get("documents")
    if not isinstance(documents, list) or not documents:
        raise CoverageValidationError(f"{source} 的 documents 必须是非空数组")

    ids: set[str] = set()
    paths: set[str] = set()
    alias_paths: set[str] = set()
    for index, document in enumerate(documents, start=1):
        if not isinstance(document, dict):
            raise CoverageValidationError(f"{source} 的第 {index} 份资料必须是对象")
        evidence_id = document.get("id")
        if not isinstance(evidence_id, str) or not EVIDENCE_ID_PATTERN.fullmatch(evidence_id):
            raise CoverageValidationError(f"{source} 的第 {index} 份资料 id 无效")
        if evidence_id in ids:
            raise CoverageValidationError(f"{source} 存在重复 id：{evidence_id}")
        ids.add(evidence_id)
        relative_path = _repo_markdown_path(document.get("path"), f"{source}.{evidence_id}.path", root=root)
        if relative_path in paths or relative_path in alias_paths:
            raise CoverageValidationError(f"{source} 存在重复资料路径：{relative_path}")
        paths.add(relative_path)
        if document.get("documentKind") not in DOCUMENT_KINDS:
            raise CoverageValidationError(f"{source}.{evidence_id}.documentKind 无效")
        layers = document.get("declaredLayers")
        if (
            not isinstance(layers, list)
            or not layers
            or any(layer not in COVERAGE_LAYERS for layer in layers)
            or len(layers) != len(set(layers))
        ):
            raise CoverageValidationError(f"{source}.{evidence_id}.declaredLayers 无效")
        aliases = document.get("aliases", [])
        if not isinstance(aliases, list):
            raise CoverageValidationError(f"{source}.{evidence_id}.aliases 必须是数组")
        for alias_index, alias in enumerate(aliases, start=1):
            normalized = _repo_markdown_path(
                alias,
                f"{source}.{evidence_id}.aliases[{alias_index}]",
                root=root,
            )
            if normalized in paths or normalized in alias_paths:
                raise CoverageValidationError(f"{source} 存在重复别名路径：{normalized}")
            alias_paths.add(normalized)
    return registry


def collect_manifest_paths(manifest: dict[str, Any]) -> tuple[list[str], set[str], dict[str, list[str]]]:
    if manifest.get("schemaVersion") != 2 or not isinstance(manifest.get("pages"), list):
        raise CoverageValidationError("流程证据清单格式无效")
    shared_paths = {str(path).replace("\\", "/") for path in manifest.get("defaultValidationPaths", [])}
    page_paths: dict[str, list[str]] = {}
    ordered: list[str] = []
    seen: set[str] = set()
    for path in manifest.get("defaultValidationPaths", []):
        normalized = str(path).replace("\\", "/")
        if normalized not in seen:
            seen.add(normalized)
            ordered.append(normalized)
    for page in manifest["pages"]:
        page_key = page.get("pageKey")
        paths = [str(path).replace("\\", "/") for path in page.get("validationPaths", [])]
        page_paths[page_key] = paths
        for normalized in paths:
            if normalized not in seen:
                seen.add(normalized)
                ordered.append(normalized)
    return ordered, shared_paths, page_paths


def _route_page_keys(route: dict[str, Any]) -> list[str]:
    return sorted(
        {
            step["pageKey"]
            for step in route.get("steps", [])
            if isinstance(step, dict) and isinstance(step.get("pageKey"), str)
        }
    )


def build_coverage_report(
    identity_registry: dict[str, Any],
    evidence_manifest: dict[str, Any],
    route_manifest: dict[str, Any],
    baseline_registry: dict[str, Any],
    *,
    root: Path = ROOT,
) -> dict[str, Any]:
    manifest_paths, shared_paths, page_paths = collect_manifest_paths(evidence_manifest)
    registered_paths = [item["path"].replace("\\", "/") for item in identity_registry["documents"]]
    missing = sorted(set(manifest_paths) - set(registered_paths))
    extra = sorted(set(registered_paths) - set(manifest_paths))
    if missing or extra:
        raise CoverageValidationError(
            f"身份登记必须精确覆盖流程证据清单；缺少={missing or '无'}；额外={extra or '无'}"
        )
    if route_manifest.get("schemaVersion") != 2 or not isinstance(route_manifest.get("routes"), list):
        raise CoverageValidationError("业务链路清单格式无效")

    document_by_path = {item["path"].replace("\\", "/"): item for item in identity_registry["documents"]}
    page_route_ids: dict[str, list[str]] = {}
    route_pages: dict[str, list[str]] = {}
    for route in route_manifest["routes"]:
        route_id = route.get("id")
        if not isinstance(route_id, str):
            raise CoverageValidationError("业务链路缺少 id")
        keys = _route_page_keys(route)
        route_pages[route_id] = keys
        for page_key in keys:
            page_route_ids.setdefault(page_key, []).append(route_id)

    documents: list[dict[str, Any]] = []
    for entry in identity_registry["documents"]:
        relative_path = entry["path"].replace("\\", "/")
        specific_page_keys = sorted(
            page_key for page_key, paths in page_paths.items() if relative_path in paths
        )
        covered_page_keys = sorted(page_paths) if relative_path in shared_paths else specific_page_keys
        route_ids = sorted(
            {route_id for page_key in covered_page_keys for route_id in page_route_ids.get(page_key, [])}
        )
        documents.append(
            {
                "id": entry["id"],
                "path": relative_path,
                "label": Path(relative_path).stem,
                "documentKind": entry["documentKind"],
                "declaredLayers": entry["declaredLayers"],
                "scope": "shared" if relative_path in shared_paths else "specific",
                "specificPageKeys": specific_page_keys,
                "coveredPageKeys": covered_page_keys,
                "routeIds": route_ids,
            }
        )

    document_id_by_path = {item["path"]: item["id"] for item in documents}
    pages = []
    for page_key in sorted(page_paths):
        evidence_ids = [document_id_by_path[path] for path in page_paths[page_key]]
        pages.append(
            {
                "pageKey": page_key,
                "routeIds": sorted(page_route_ids.get(page_key, [])),
                "specificEvidenceIds": evidence_ids,
                "state": "specific" if evidence_ids else "missing-specific",
            }
        )
    page_by_key = {item["pageKey"]: item for item in pages}
    routes = []
    for route in route_manifest["routes"]:
        route_id = route["id"]
        keys = route_pages[route_id]
        evidence_ids = sorted(
            {
                evidence_id
                for page_key in keys
                for evidence_id in page_by_key.get(page_key, {}).get("specificEvidenceIds", [])
            }
        )
        missing_page_keys = [
            page_key for page_key in keys if page_by_key.get(page_key, {}).get("state") == "missing-specific"
        ]
        layers = sorted(
            {
                layer
                for evidence_id in evidence_ids
                for layer in next(item for item in documents if item["id"] == evidence_id)["declaredLayers"]
            },
            key=COVERAGE_LAYERS.index,
        )
        routes.append(
            {
                "id": route_id,
                "title": route.get("title", route_id),
                "pageKeys": keys,
                "specificEvidenceIds": evidence_ids,
                "declaredLayers": layers,
                "missingSpecificPageKeys": missing_page_keys,
                "state": "covered" if not missing_page_keys else "gaps-present",
            }
        )

    path_or_alias_to_id: dict[str, str] = {}
    for entry in identity_registry["documents"]:
        path_or_alias_to_id[entry["path"].replace("\\", "/")] = entry["id"]
        for alias in entry.get("aliases", []):
            path_or_alias_to_id[alias.replace("\\", "/")] = entry["id"]
    selected_id = baseline_registry.get("selectedBaselineId")
    baseline_coverage: dict[str, Any]
    if selected_id is None:
        baseline_coverage = {
            "state": "not-evaluated",
            "baselineId": None,
            "affectedPageKeys": [],
            "formalEvidenceIds": [],
            "unregisteredEvidencePaths": [],
            "coveredAffectedPageKeys": [],
            "missingAffectedPageKeys": [],
        }
    else:
        selected = next(item for item in baseline_registry["baselines"] if item["id"] == selected_id)
        envelope = load_json(root / Path(selected["path"]))
        evidence = envelope["evidence"]
        affected = sorted({item["pageKey"] for item in evidence.get("affectedPages", [])})
        evidence_ids: list[str] = []
        unregistered: list[str] = []
        covered: set[str] = set()
        for item in evidence.get("formalEvidence", []):
            relative_path = item.get("path")
            evidence_id = path_or_alias_to_id.get(relative_path)
            if evidence_id is None:
                if isinstance(relative_path, str):
                    unregistered.append(relative_path)
                continue
            evidence_ids.append(evidence_id)
            covered.update(item.get("pageKeys", []))
        covered_affected = sorted(set(affected) & covered)
        baseline_coverage = {
            "state": "evaluated",
            "baselineId": selected_id,
            "affectedPageKeys": affected,
            "formalEvidenceIds": sorted(set(evidence_ids)),
            "unregisteredEvidencePaths": sorted(set(unregistered)),
            "coveredAffectedPageKeys": covered_affected,
            "missingAffectedPageKeys": sorted(set(affected) - set(covered_affected)),
        }

    layer_counts = {
        layer: sum(layer in document["declaredLayers"] for document in documents)
        for layer in COVERAGE_LAYERS
    }
    missing_pages = [page["pageKey"] for page in pages if page["state"] == "missing-specific"]
    return {
        "schemaVersion": 1,
        "kind": "cube-delivery-evidence-coverage",
        "state": "gaps-present" if missing_pages else "complete",
        "summary": {
            "documents": len(documents),
            "flows": len(pages),
            "flowsWithSpecificEvidence": len(pages) - len(missing_pages),
            "flowsMissingSpecificEvidence": len(missing_pages),
            "routes": len(routes),
            "routesCovered": sum(route["state"] == "covered" for route in routes),
            "routesWithGaps": sum(route["state"] == "gaps-present" for route in routes),
            "declaredLayerDocuments": layer_counts,
        },
        "documents": documents,
        "pages": pages,
        "routes": routes,
        "selectedBaselineCoverage": baseline_coverage,
        "method": {
            "stableIdentityFromRegistry": True,
            "flowRelationsFromEvidenceManifest": True,
            "routeRelationsFromRouteManifest": True,
            "selectedBaselineOnly": True,
            "sharedEvidenceDoesNotReplaceSpecificEvidence": True,
            "declaredLayersAreNotExecutionResults": True,
            "naturalLanguageInference": False,
        },
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--identity-registry", type=Path, default=DEFAULT_IDENTITY_REGISTRY)
    parser.add_argument("--evidence-manifest", type=Path, default=DEFAULT_EVIDENCE_MANIFEST)
    parser.add_argument("--route-manifest", type=Path, default=DEFAULT_ROUTE_MANIFEST)
    parser.add_argument("--baseline-registry", type=Path, default=DEFAULT_BASELINE_REGISTRY)
    parser.add_argument("--snapshot", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        identity = validate_identity_registry(
            load_json(args.identity_registry), str(args.identity_registry), root=ROOT
        )
        evidence = load_json(args.evidence_manifest)
        routes = load_json(args.route_manifest)
        baselines = check_baseline_registry(args.baseline_registry, root=ROOT)
        report = build_coverage_report(identity, evidence, routes, baselines)
        if args.snapshot is not None and load_json(args.snapshot) != report:
            raise CoverageValidationError(f"站点覆盖快照与正式关系不一致：{args.snapshot}")
    except (CoverageValidationError, ValueError) as exc:
        print(f"交付证据覆盖检查失败：{exc}", file=sys.stderr)
        return 1
    print(
        "交付证据覆盖检查通过："
        f"{report['summary']['documents']} 份稳定证据，"
        f"{report['summary']['flowsWithSpecificEvidence']}/{report['summary']['flows']} 个流程有专项证据，"
        f"{report['summary']['routesCovered']}/{report['summary']['routes']} 条链路无专项缺口"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

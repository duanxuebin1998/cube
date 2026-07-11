#!/usr/bin/env python3
"""Map changed firmware files to core flow pages, routes, and evidence."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Iterable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
FLOW_ROOT = ROOT / "docs" / "00_程序流程导航"
ROUTE_MANIFEST = FLOW_ROOT / "业务链路清单.json"
EVIDENCE_MANIFEST = FLOW_ROOT / "流程证据清单.json"
FLOW_MANIFEST_PATHS = {
    "docs/00_程序流程导航/业务链路清单.json",
    "docs/00_程序流程导航/流程证据清单.json",
}
FIRMWARE_ROOTS = ("LTD_MAIN_CPU2/", "LTD_DISPLAY_CPU3/")
FIRMWARE_SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hh", ".hpp"}
FIRMWARE_METADATA_PATHS = {
    "LTD_MAIN_CPU2/Application/Inc/app_version.h",
    "LTD_DISPLAY_CPU3/Application/app_version.h",
}


def configure_output(stdout=sys.stdout, stderr=sys.stderr) -> None:  # noqa: ANN001
    for stream in (stdout, stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding="utf-8", errors="replace")


def normalize_path(value: str) -> str:
    path = value.strip().replace("\\", "/")
    while path.startswith("./"):
        path = path[2:]
    return path


def load_json(path: Path) -> dict[str, object]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError as exc:
        raise RuntimeError(f"缺少流程影响分析数据：{path}") from exc
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"流程影响分析数据不是有效 JSON：{path}: {exc}") from exc
    if not isinstance(data, dict):
        raise RuntimeError(f"流程影响分析数据必须是对象：{path}")
    return data


def discover_flow_documents(flow_root: Path = FLOW_ROOT) -> dict[str, str]:
    result: dict[str, str] = {}
    marker = 'data-evidence-page="'
    for path in sorted(flow_root.rglob("*.html")):
        text = path.read_text(encoding="utf-8-sig", errors="strict")
        offset = 0
        while True:
            start = text.find(marker, offset)
            if start < 0:
                break
            value_start = start + len(marker)
            value_end = text.find('"', value_start)
            if value_end < 0:
                raise RuntimeError(f"流程证据页标记未闭合：{path}")
            page_key = text[value_start:value_end]
            relative_path = normalize_path(str(path.relative_to(ROOT)))
            existing = result.get(page_key)
            if existing is not None and existing != relative_path:
                raise RuntimeError(f"流程页 {page_key} 存在重复证据头：{existing}, {relative_path}")
            result[page_key] = relative_path
            offset = value_end + 1
    return result


def build_flow_index(
    route_manifest: Mapping[str, object],
    evidence_manifest: Mapping[str, object],
    page_documents: Mapping[str, str],
) -> dict[str, object]:
    routes = route_manifest.get("routes")
    pages = evidence_manifest.get("pages")
    if not isinstance(routes, list) or not routes:
        raise RuntimeError("业务链路清单缺少 routes")
    if not isinstance(pages, list) or not pages:
        raise RuntimeError("流程证据清单缺少 pages")

    metadata_by_page: dict[str, dict[str, object]] = {}
    route_by_id: dict[str, dict[str, object]] = {}
    for route in routes:
        if not isinstance(route, dict):
            raise RuntimeError("业务链路必须是对象")
        route_id = route.get("id")
        title = route.get("title")
        steps = route.get("steps")
        if not isinstance(route_id, str) or not isinstance(title, str) or not isinstance(steps, list):
            raise RuntimeError("业务链路缺少 id、title 或 steps")
        route_by_id[route_id] = {"id": route_id, "title": title}
        for step in steps:
            if not isinstance(step, dict) or step.get("formal") is not True:
                continue
            page_key = step.get("pageKey")
            if not isinstance(page_key, str):
                raise RuntimeError(f"业务链路 {route_id} 的正式步骤缺少 pageKey")
            current = metadata_by_page.setdefault(
                page_key,
                {
                    "pageKey": page_key,
                    "title": str(step.get("label", page_key)),
                    "href": str(step.get("siteHref", "")),
                    "routeIds": [],
                },
            )
            route_ids = current["routeIds"]
            if isinstance(route_ids, list) and route_id not in route_ids:
                route_ids.append(route_id)

    default_validation_paths = evidence_manifest.get("defaultValidationPaths")
    if not isinstance(default_validation_paths, list):
        raise RuntimeError("流程证据清单 defaultValidationPaths 必须是数组")
    indexed_pages: list[dict[str, object]] = []
    source_to_pages: dict[str, list[str]] = {}
    for page in pages:
        if not isinstance(page, dict):
            raise RuntimeError("流程证据页面必须是对象")
        page_key = page.get("pageKey")
        source_paths = page.get("sourcePaths")
        validation_paths = page.get("validationPaths")
        if not isinstance(page_key, str) or page_key not in metadata_by_page:
            raise RuntimeError(f"流程证据页未进入正式业务链路：{page_key!r}")
        if not isinstance(source_paths, list) or not source_paths:
            raise RuntimeError(f"流程证据页缺少源码入口：{page_key}")
        if not isinstance(validation_paths, list):
            raise RuntimeError(f"流程证据页验证资料必须是数组：{page_key}")
        document_path = page_documents.get(page_key)
        if document_path is None:
            raise RuntimeError(f"未找到带证据头的正式流程页：{page_key}")
        metadata = metadata_by_page[page_key]
        route_ids = list(metadata["routeIds"])
        normalized_sources = [normalize_path(str(item)) for item in source_paths]
        item = {
            **metadata,
            "documentPath": normalize_path(document_path),
            "sourcePaths": normalized_sources,
            "validationPaths": [
                *[normalize_path(str(value)) for value in default_validation_paths],
                *[normalize_path(str(value)) for value in validation_paths],
            ],
            "routes": [route_by_id[route_id] for route_id in route_ids],
        }
        indexed_pages.append(item)
        for source_path in normalized_sources:
            source_to_pages.setdefault(source_path, []).append(page_key)

    return {
        "pages": sorted(indexed_pages, key=lambda item: str(item["pageKey"])),
        "routes": [route_by_id[route_id] for route_id in route_by_id],
        "sourceToPages": source_to_pages,
        "sourcePathCount": len(source_to_pages),
    }


def is_firmware_source(path: str) -> bool:
    normalized = normalize_path(path)
    return (
        normalized not in FIRMWARE_METADATA_PATHS
        and normalized.startswith(FIRMWARE_ROOTS)
        and Path(normalized).suffix.lower() in FIRMWARE_SOURCE_SUFFIXES
    )


def analyze_changed_paths(index: Mapping[str, object], changed_paths: Iterable[str]) -> dict[str, object]:
    changed = sorted({normalize_path(path) for path in changed_paths if normalize_path(path)})
    source_to_pages = index.get("sourceToPages")
    pages = index.get("pages")
    if not isinstance(source_to_pages, dict) or not isinstance(pages, list):
        raise RuntimeError("流程影响索引结构无效")
    pages_by_key = {
        str(page["pageKey"]): page
        for page in pages
        if isinstance(page, dict) and isinstance(page.get("pageKey"), str)
    }
    firmware_sources = [path for path in changed if is_firmware_source(path)]
    mapped_sources = [path for path in firmware_sources if path in source_to_pages]
    unmapped_sources = [path for path in firmware_sources if path not in source_to_pages]
    affected_page_keys = sorted(
        {
            str(page_key)
            for source_path in mapped_sources
            for page_key in source_to_pages.get(source_path, [])
        }
    )
    evidence_manifest_changed = "docs/00_程序流程导航/流程证据清单.json" in changed
    affected_pages: list[dict[str, object]] = []
    needs_review: list[str] = []
    affected_route_ids: set[str] = set()
    validation_paths: set[str] = set()
    for page_key in affected_page_keys:
        page = pages_by_key[page_key]
        document_path = str(page["documentPath"])
        document_changed = document_path in changed
        reviewed = document_changed or evidence_manifest_changed
        if not reviewed:
            needs_review.append(page_key)
        route_ids = [str(route_id) for route_id in page["routeIds"]]
        affected_route_ids.update(route_ids)
        validation_paths.update(str(path) for path in page["validationPaths"])
        affected_pages.append(
            {
                **page,
                "changedSourcePaths": [path for path in mapped_sources if page_key in source_to_pages[path]],
                "documentChanged": document_changed,
                "reviewed": reviewed,
            }
        )

    route_by_id = {
        str(route["id"]): route
        for route in index.get("routes", [])
        if isinstance(route, dict) and isinstance(route.get("id"), str)
    }
    return {
        "changedPaths": changed,
        "firmwareSourcePaths": firmware_sources,
        "mappedSourcePaths": mapped_sources,
        "unmappedFirmwareSourcePaths": unmapped_sources,
        "affectedPages": affected_pages,
        "affectedRoutes": [route_by_id[route_id] for route_id in sorted(affected_route_ids)],
        "validationPaths": sorted(validation_paths),
        "needsReviewPageKeys": needs_review,
        "flowManifestChanged": any(path in FLOW_MANIFEST_PATHS for path in changed),
        "metrics": {
            "changedFiles": len(changed),
            "firmwareSources": len(firmware_sources),
            "mappedSources": len(mapped_sources),
            "unmappedFirmwareSources": len(unmapped_sources),
            "affectedPages": len(affected_pages),
            "affectedRoutes": len(affected_route_ids),
            "needsReview": len(needs_review),
        },
    }


def git_paths(arguments: Sequence[str]) -> list[str]:
    result = subprocess.run(
        ["git", *arguments],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return [normalize_path(item.decode("utf-8", errors="surrogateescape")) for item in result.stdout.split(b"\0") if item]


def working_tree_paths() -> list[str]:
    return sorted(
        {
            *git_paths(["diff", "--name-only", "-z", "--"]),
            *git_paths(["diff", "--cached", "--name-only", "-z", "--"]),
            *git_paths(["ls-files", "--others", "--exclude-standard", "-z", "--"]),
        }
    )


def collect_changed_paths(args: argparse.Namespace) -> tuple[str, list[str]]:
    if args.files:
        return "显式文件清单", [normalize_path(path) for path in args.files]
    if args.staged:
        return "暂存区", git_paths(["diff", "--cached", "--name-only", "-z", "--"])
    if args.revision_range:
        return f"Git 范围 {args.revision_range}", git_paths(["diff", "--name-only", "-z", args.revision_range, "--"])
    return "当前工作区（已暂存、未暂存和未跟踪）", working_tree_paths()


def render_text(scope: str, report: Mapping[str, object]) -> str:
    metrics = report["metrics"]
    lines = [
        f"流程变更影响分析：{scope}",
        (
            f"- 改动文件 {metrics['changedFiles']} 个；固件源码 {metrics['firmwareSources']} 个；"
            f"命中核心源码入口 {metrics['mappedSources']} 个。"
        ),
        (
            f"- 影响核心流程 {metrics['affectedPages']} 个、业务链路 {metrics['affectedRoutes']} 条；"
            f"待复核流程 {metrics['needsReview']} 个。"
        ),
    ]
    affected_pages = report["affectedPages"]
    if affected_pages:
        lines.append("受影响流程：")
        for page in affected_pages:
            state = "已同步复核" if page["reviewed"] else "需要复核"
            route_titles = "、".join(str(route["title"]) for route in page["routes"])
            lines.append(f"- {page['pageKey']} {page['title']}：{state}；链路={route_titles}")
            for path in page["changedSourcePaths"]:
                lines.append(f"  - 源码：{path}")
            lines.append(f"  - 流程页：{page['documentPath']}")
    else:
        lines.append("- 当前改动未命中 18 个核心流程登记的源码入口。")
    unmapped = report["unmappedFirmwareSourcePaths"]
    if unmapped:
        lines.append("未映射到核心流程的固件源码（仅提示，需按语义人工判断）：")
        lines.extend(f"- {path}" for path in unmapped)
    validation_paths = report["validationPaths"]
    if validation_paths:
        lines.append("建议复核的验证资料：")
        lines.extend(f"- {path}" for path in validation_paths)
    return "\n".join(lines)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="根据 Git 改动反推受影响流程、业务链路和验证资料")
    scope = parser.add_mutually_exclusive_group()
    scope.add_argument("--staged", action="store_true", help="分析暂存区")
    scope.add_argument("--range", dest="revision_range", help="分析指定 Git revision range")
    scope.add_argument("--files", nargs="+", help="分析显式文件清单")
    parser.add_argument("--json", action="store_true", help="输出 JSON")
    parser.add_argument("--strict", action="store_true", help="命中核心源码但流程未复核时返回失败")
    parser.add_argument("--validate-index", action="store_true", help="只验证反向索引，不读取 Git 改动")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    configure_output()
    args = parse_args(argv)
    try:
        route_manifest = load_json(ROUTE_MANIFEST)
        evidence_manifest = load_json(EVIDENCE_MANIFEST)
        index = build_flow_index(route_manifest, evidence_manifest, discover_flow_documents())
        if args.validate_index:
            print(
                f"流程变更影响索引有效：{len(index['pages'])} 个核心流程，"
                f"{len(index['routes'])} 条业务链路，{index['sourcePathCount']} 个源码入口。"
            )
            return 0
        scope, changed_paths = collect_changed_paths(args)
        report = analyze_changed_paths(index, changed_paths)
        if args.json:
            print(json.dumps({"scope": scope, **report}, ensure_ascii=False, indent=2))
        else:
            print(render_text(scope, report))
        if args.strict and report["needsReviewPageKeys"]:
            print("严格检查失败：存在已命中核心源码但未同步流程页或证据清单的流程。", file=sys.stderr)
            return 1
        return 0
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"流程变更影响分析失败：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

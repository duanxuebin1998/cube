#!/usr/bin/env python3
"""Map changed firmware files to core flow pages, routes, and evidence."""

from __future__ import annotations

import argparse
import json
import re
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
CURRENT_IMPACT_PATHS = ("LTD_MAIN_CPU2", "LTD_DISPLAY_CPU3", "docs/00_程序流程导航")
FIRMWARE_SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".h", ".hh", ".hpp"}
FIRMWARE_METADATA_PATHS = {
    "LTD_MAIN_CPU2/Application/Inc/app_version.h",
    "LTD_DISPLAY_CPU3/Application/app_version.h",
}
SEMANTIC_SYMBOL_KINDS = {"function", "state", "command", "register", "error-exit"}
DIFF_HEADER_RE = re.compile(r"^diff --git a/(.+) b/(.+)$")
DIFF_HUNK_RE = re.compile(
    r"^@@ -(?P<old_start>\d+)(?:,(?P<old_count>\d+))? "
    r"\+(?P<new_start>\d+)(?:,(?P<new_count>\d+))? @@(?: ?(?P<context>.*))?$"
)


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
    semantic_symbol_count = 0
    for page in pages:
        if not isinstance(page, dict):
            raise RuntimeError("流程证据页面必须是对象")
        page_key = page.get("pageKey")
        source_paths = page.get("sourcePaths")
        source_symbols = page.get("sourceSymbols", [])
        validation_paths = page.get("validationPaths")
        if not isinstance(page_key, str) or page_key not in metadata_by_page:
            raise RuntimeError(f"流程证据页未进入正式业务链路：{page_key!r}")
        if not isinstance(source_paths, list) or not source_paths:
            raise RuntimeError(f"流程证据页缺少源码入口：{page_key}")
        if not isinstance(source_symbols, list):
            raise RuntimeError(f"流程证据页 sourceSymbols 必须是数组：{page_key}")
        if not isinstance(validation_paths, list):
            raise RuntimeError(f"流程证据页验证资料必须是数组：{page_key}")
        document_path = page_documents.get(page_key)
        if document_path is None:
            raise RuntimeError(f"未找到带证据头的正式流程页：{page_key}")
        metadata = metadata_by_page[page_key]
        route_ids = list(metadata["routeIds"])
        normalized_sources = [normalize_path(str(item)) for item in source_paths]
        normalized_symbol_groups: list[dict[str, object]] = []
        symbol_paths: set[str] = set()
        for group in source_symbols:
            if not isinstance(group, dict):
                raise RuntimeError(f"流程证据页 sourceSymbols 项必须是对象：{page_key}")
            symbol_path = normalize_path(str(group.get("path", "")))
            symbols = group.get("symbols")
            if symbol_path not in normalized_sources:
                raise RuntimeError(f"流程证据页语义符号路径未进入 sourcePaths：{page_key}: {symbol_path}")
            if symbol_path in symbol_paths:
                raise RuntimeError(f"流程证据页语义符号路径重复：{page_key}: {symbol_path}")
            if not isinstance(symbols, list) or not symbols:
                raise RuntimeError(f"流程证据页语义符号列表为空：{page_key}: {symbol_path}")
            symbol_paths.add(symbol_path)
            normalized_symbols: list[dict[str, str]] = []
            symbol_names: set[str] = set()
            for symbol in symbols:
                if not isinstance(symbol, dict):
                    raise RuntimeError(f"流程证据页语义符号必须是对象：{page_key}: {symbol_path}")
                name = symbol.get("name")
                kind = symbol.get("kind")
                label = symbol.get("label")
                if not isinstance(name, str) or re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", name) is None:
                    raise RuntimeError(f"流程证据页语义符号名无效：{page_key}: {name!r}")
                if name in symbol_names:
                    raise RuntimeError(f"流程证据页语义符号重复：{page_key}: {symbol_path}: {name}")
                if kind not in SEMANTIC_SYMBOL_KINDS:
                    raise RuntimeError(f"流程证据页语义符号类型无效：{page_key}: {name}: {kind!r}")
                if not isinstance(label, str) or not label.strip():
                    raise RuntimeError(f"流程证据页语义符号说明为空：{page_key}: {name}")
                symbol_names.add(name)
                normalized_symbols.append({"name": name, "kind": str(kind), "label": label.strip()})
                semantic_symbol_count += 1
            normalized_symbol_groups.append({"path": symbol_path, "symbols": normalized_symbols})
        item = {
            **metadata,
            "documentPath": normalize_path(document_path),
            "sourcePaths": normalized_sources,
            "sourceSymbols": normalized_symbol_groups,
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
        "semanticSymbolCount": semantic_symbol_count,
    }


def strip_c_comments(line: str, in_block_comment: bool) -> tuple[str, bool]:
    """Remove C comments from one changed line while retaining string-like code text."""
    output: list[str] = []
    index = 0
    while index < len(line):
        if in_block_comment:
            end = line.find("*/", index)
            if end < 0:
                return "".join(output), True
            in_block_comment = False
            index = end + 2
            continue
        block = line.find("/*", index)
        slash = line.find("//", index)
        candidates = [value for value in (block, slash) if value >= 0]
        if not candidates:
            output.append(line[index:])
            break
        marker = min(candidates)
        output.append(line[index:marker])
        if marker == slash:
            break
        in_block_comment = True
        index = marker + 2
    return "".join(output), in_block_comment


def parse_unified_diff(diff_text: str) -> dict[str, dict[str, object]]:
    """Extract code-bearing changed lines and Git hunk function context per file."""
    result: dict[str, dict[str, object]] = {}
    current_path: str | None = None
    current_hunk: dict[str, object] | None = None
    old_comment = False
    new_comment = False

    def finish_hunk() -> None:
        nonlocal current_hunk
        if current_path is None or current_hunk is None:
            return
        changed_code = current_hunk["changedCode"]
        if isinstance(changed_code, list) and changed_code:
            context = str(current_hunk.get("context", "")).strip()
            semantic_text = [*changed_code]
            if context:
                semantic_text.append(context)
            record = result.setdefault(
                current_path,
                {"semanticText": [], "codeChanged": False, "hunks": 0, "hunkRanges": []},
            )
            record["semanticText"].extend(semantic_text)  # type: ignore[union-attr]
            record["codeChanged"] = True
            record["hunks"] = int(record["hunks"]) + 1
            record["hunkRanges"].append(current_hunk["range"])  # type: ignore[union-attr]
        current_hunk = None

    for raw_line in diff_text.splitlines():
        header = DIFF_HEADER_RE.match(raw_line)
        if header:
            finish_hunk()
            current_path = normalize_path(header.group(2))
            old_comment = False
            new_comment = False
            result.setdefault(
                current_path,
                {"semanticText": [], "codeChanged": False, "hunks": 0, "hunkRanges": []},
            )
            continue
        hunk = DIFF_HUNK_RE.match(raw_line)
        if hunk and current_path is not None:
            finish_hunk()
            current_hunk = {
                "context": hunk.group("context") or "",
                "changedCode": [],
                "range": {
                    "oldStart": int(hunk.group("old_start")),
                    "oldCount": int(hunk.group("old_count") or "1"),
                    "newStart": int(hunk.group("new_start")),
                    "newCount": int(hunk.group("new_count") or "1"),
                },
            }
            old_comment = False
            new_comment = False
            continue
        if current_hunk is None or not raw_line or raw_line.startswith(("+++", "---")):
            continue
        prefix = raw_line[0]
        content = raw_line[1:]
        if prefix == "-":
            code, old_comment = strip_c_comments(content, old_comment)
            if code.strip():
                current_hunk["changedCode"].append(code)  # type: ignore[union-attr]
        elif prefix == "+":
            code, new_comment = strip_c_comments(content, new_comment)
            if code.strip():
                current_hunk["changedCode"].append(code)  # type: ignore[union-attr]
        elif prefix == " ":
            _, old_comment = strip_c_comments(content, old_comment)
            _, new_comment = strip_c_comments(content, new_comment)
    finish_hunk()
    return result


def annotate_changed_evidence_pages(
    diff_by_path: dict[str, dict[str, object]],
    manifest_text: str,
) -> None:
    record = diff_by_path.get("docs/00_程序流程导航/流程证据清单.json")
    if record is None:
        return
    markers: list[tuple[str, int]] = []
    for line_number, line in enumerate(manifest_text.splitlines(), start=1):
        match = re.search(r'"pageKey"\s*:\s*"([^"]+)"', line)
        if match:
            markers.append((match.group(1), line_number))
    changed_keys: set[str] = set()
    ranges = record.get("hunkRanges", [])
    for index, (page_key, marker_line) in enumerate(markers):
        page_start = max(1, marker_line - 1)
        page_end = markers[index + 1][1] - 2 if index + 1 < len(markers) else len(manifest_text.splitlines())
        for value in ranges:
            if not isinstance(value, Mapping):
                continue
            start = int(value.get("newStart", 0))
            count = int(value.get("newCount", 0))
            end = start + max(count, 1) - 1
            if start <= page_end and end >= page_start:
                changed_keys.add(page_key)
                break
    record["changedPageKeys"] = sorted(changed_keys)


def symbol_matches(text: str, name: str) -> bool:
    return re.search(rf"(?<![A-Za-z0-9_]){re.escape(name)}(?![A-Za-z0-9_])", text) is not None


def is_firmware_source(path: str) -> bool:
    normalized = normalize_path(path)
    return (
        normalized not in FIRMWARE_METADATA_PATHS
        and normalized.startswith(FIRMWARE_ROOTS)
        and Path(normalized).suffix.lower() in FIRMWARE_SOURCE_SUFFIXES
    )


def analyze_changed_paths(
    index: Mapping[str, object],
    changed_paths: Iterable[str],
    diff_by_path: Mapping[str, Mapping[str, object]] | None = None,
) -> dict[str, object]:
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
    evidence_diff = diff_by_path.get("docs/00_程序流程导航/流程证据清单.json") if diff_by_path else None
    changed_evidence_page_keys = {
        str(value)
        for value in evidence_diff.get("changedPageKeys", [])
    } if isinstance(evidence_diff, Mapping) else set()
    affected_pages: list[dict[str, object]] = []
    needs_review: list[str] = []
    file_review: list[str] = []
    affected_route_ids: set[str] = set()
    validation_paths: set[str] = set()
    for page_key in affected_page_keys:
        page = pages_by_key[page_key]
        document_path = str(page["documentPath"])
        document_changed = document_path in changed
        evidence_page_changed = page_key in changed_evidence_page_keys
        reviewed = document_changed or evidence_page_changed
        if not reviewed:
            file_review.append(page_key)
        semantic_matches: list[dict[str, str]] = []
        analyzed_semantic_sources: list[str] = []
        configured_semantic_sources: list[str] = []
        for group in page.get("sourceSymbols", []):
            if not isinstance(group, dict):
                continue
            symbol_path = str(group.get("path", ""))
            if symbol_path not in mapped_sources or page_key not in source_to_pages.get(symbol_path, []):
                continue
            configured_semantic_sources.append(symbol_path)
            diff_record = diff_by_path.get(symbol_path) if diff_by_path is not None else None
            if not isinstance(diff_record, Mapping):
                continue
            analyzed_semantic_sources.append(symbol_path)
            semantic_text = "\n".join(str(value) for value in diff_record.get("semanticText", []))
            for symbol in group.get("symbols", []):
                if not isinstance(symbol, dict):
                    continue
                name = str(symbol.get("name", ""))
                if semantic_text and symbol_matches(semantic_text, name):
                    semantic_matches.append(
                        {
                            "path": symbol_path,
                            "name": name,
                            "kind": str(symbol.get("kind", "")),
                            "label": str(symbol.get("label", "")),
                        }
                    )
        if semantic_matches:
            semantic_status = "matched"
        elif configured_semantic_sources and len(analyzed_semantic_sources) == len(configured_semantic_sources):
            semantic_status = "not-matched"
        elif configured_semantic_sources:
            semantic_status = "unavailable"
        else:
            semantic_status = "not-configured"
        if semantic_matches and not reviewed:
            needs_review.append(page_key)
        route_ids = [str(route_id) for route_id in page["routeIds"]]
        affected_route_ids.update(route_ids)
        validation_paths.update(str(path) for path in page["validationPaths"])
        affected_pages.append(
            {
                **page,
                "changedSourcePaths": [path for path in mapped_sources if page_key in source_to_pages[path]],
                "documentChanged": document_changed,
                "evidencePageChanged": evidence_page_changed,
                "reviewed": reviewed,
                "semanticStatus": semantic_status,
                "semanticMatches": semantic_matches,
                "configuredSemanticSourcePaths": configured_semantic_sources,
                "analyzedSemanticSourcePaths": analyzed_semantic_sources,
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
        "fileReviewPageKeys": file_review,
        "flowManifestChanged": any(path in FLOW_MANIFEST_PATHS for path in changed),
        "metrics": {
            "changedFiles": len(changed),
            "firmwareSources": len(firmware_sources),
            "mappedSources": len(mapped_sources),
            "unmappedFirmwareSources": len(unmapped_sources),
            "affectedPages": len(affected_pages),
            "affectedRoutes": len(affected_route_ids),
            "semanticAffectedPages": sum(bool(page["semanticMatches"]) for page in affected_pages),
            "semanticMatchedSymbols": sum(len(page["semanticMatches"]) for page in affected_pages),
            "fileOnlyPages": sum(page["semanticStatus"] == "not-matched" for page in affected_pages),
            "semanticUnavailablePages": sum(
                page["semanticStatus"] in {"unavailable", "not-configured"} for page in affected_pages
            ),
            "needsReview": len(needs_review),
            "fileNeedsReview": len(file_review),
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


def git_diff(arguments: Sequence[str]) -> str:
    result = subprocess.run(
        ["git", "--no-pager", "diff", "--no-ext-diff", "--no-color", "--unified=0", *arguments],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout.decode("utf-8", errors="surrogateescape")


def untracked_semantic_records(paths: Iterable[str]) -> dict[str, dict[str, object]]:
    untracked = set(
        git_paths(
            [
                "ls-files",
                "--others",
                "--exclude-standard",
                "-z",
                "--",
                *CURRENT_IMPACT_PATHS,
            ]
        )
    )
    records: dict[str, dict[str, object]] = {}
    for value in paths:
        normalized = normalize_path(value)
        if normalized not in untracked or not is_firmware_source(normalized):
            continue
        file = ROOT / Path(normalized)
        if not file.is_file():
            continue
        in_comment = False
        semantic_text: list[str] = []
        for line in file.read_bytes().decode("latin1").splitlines():
            code, in_comment = strip_c_comments(line, in_comment)
            if code.strip():
                semantic_text.append(code)
        records[normalized] = {
            "semanticText": semantic_text,
            "codeChanged": bool(semantic_text),
            "hunks": 1 if semantic_text else 0,
            "untracked": True,
        }
    return records


def git_blob_text(spec: str) -> str:
    result = subprocess.run(
        ["git", "show", spec],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    return result.stdout.decode("utf-8", errors="strict")


def evidence_manifest_text_for_scope(args: argparse.Namespace) -> str:
    manifest_path = "docs/00_程序流程导航/流程证据清单.json"
    if args.staged:
        return git_blob_text(f":{manifest_path}")
    if args.revision_range:
        separator = "..." if "..." in args.revision_range else ".." if ".." in args.revision_range else None
        if separator is not None:
            right = args.revision_range.split(separator, maxsplit=1)[1]
            if right:
                return git_blob_text(f"{right}:{manifest_path}")
    return EVIDENCE_MANIFEST.read_text(encoding="utf-8")


def finalize_diff_records(
    args: argparse.Namespace,
    diff_by_path: dict[str, dict[str, object]],
) -> dict[str, dict[str, object]]:
    annotate_changed_evidence_pages(diff_by_path, evidence_manifest_text_for_scope(args))
    return diff_by_path


def working_tree_paths() -> list[str]:
    return sorted(
        {
            *git_paths(["diff", "--name-only", "-z", "--", *CURRENT_IMPACT_PATHS]),
            *git_paths(
                ["diff", "--cached", "--name-only", "-z", "--", *CURRENT_IMPACT_PATHS]
            ),
            *git_paths(
                [
                    "ls-files",
                    "--others",
                    "--exclude-standard",
                    "-z",
                    "--",
                    *CURRENT_IMPACT_PATHS,
                ]
            ),
        }
    )


def collect_changed_paths(args: argparse.Namespace) -> tuple[str, list[str], dict[str, dict[str, object]]]:
    if args.files:
        paths = [normalize_path(path) for path in args.files]
        diff_by_path = parse_unified_diff(git_diff(["HEAD", "--", *paths]))
        diff_by_path.update(untracked_semantic_records(paths))
        return "显式文件清单", paths, finalize_diff_records(args, diff_by_path)
    if args.staged:
        return (
            "暂存区",
            git_paths(
                ["diff", "--cached", "--name-only", "-z", "--", *CURRENT_IMPACT_PATHS]
            ),
            finalize_diff_records(
                args,
                parse_unified_diff(git_diff(["--cached", "--", *CURRENT_IMPACT_PATHS])),
            ),
        )
    if args.revision_range:
        return (
            f"Git 范围 {args.revision_range}",
            git_paths(
                [
                    "diff",
                    "--name-only",
                    "-z",
                    args.revision_range,
                    "--",
                    *CURRENT_IMPACT_PATHS,
                ]
            ),
            finalize_diff_records(
                args,
                parse_unified_diff(
                    git_diff([args.revision_range, "--", *CURRENT_IMPACT_PATHS])
                ),
            ),
        )
    paths = working_tree_paths()
    diff_by_path = parse_unified_diff(git_diff(["HEAD", "--", *CURRENT_IMPACT_PATHS]))
    diff_by_path.update(untracked_semantic_records(paths))
    return (
        "当前工作区（已暂存、未暂存和未跟踪）",
        paths,
        finalize_diff_records(args, diff_by_path),
    )


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
            f"语义命中 {metrics['semanticAffectedPages']} 个流程 / {metrics['semanticMatchedSymbols']} 个符号；"
            f"严格待复核 {metrics['needsReview']} 个。"
        ),
    ]
    affected_pages = report["affectedPages"]
    if affected_pages:
        lines.append("受影响流程：")
        for page in affected_pages:
            semantic_status = page["semanticStatus"]
            if semantic_status == "matched":
                state = "关键语义已命中；已同步复核" if page["reviewed"] else "关键语义已命中；需要复核"
            elif semantic_status == "not-matched":
                state = "仅文件级命中；未命中已登记关键语义"
            elif semantic_status == "unavailable":
                state = "文件级命中；部分差异无法做语义识别"
            else:
                state = "文件级命中；该入口尚未登记关键语义"
            route_titles = "、".join(str(route["title"]) for route in page["routes"])
            lines.append(f"- {page['pageKey']} {page['title']}：{state}；链路={route_titles}")
            for path in page["changedSourcePaths"]:
                lines.append(f"  - 源码：{path}")
            lines.append(f"  - 流程页：{page['documentPath']}")
            for match in page["semanticMatches"]:
                lines.append(f"  - 语义：{match['kind']} {match['name']}（{match['label']}）")
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
                f"{len(index['routes'])} 条业务链路，{index['sourcePathCount']} 个源码入口，"
                f"{index['semanticSymbolCount']} 个关键语义。"
            )
            return 0
        scope, changed_paths, diff_by_path = collect_changed_paths(args)
        report = analyze_changed_paths(index, changed_paths, diff_by_path)
        if args.json:
            print(json.dumps({"scope": scope, **report}, ensure_ascii=False, indent=2))
        else:
            print(render_text(scope, report))
        if args.strict and report["needsReviewPageKeys"]:
            print("严格检查失败：存在已命中关键语义但未同步流程页或证据清单的流程。", file=sys.stderr)
            return 1
        return 0
    except (RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"流程变更影响分析失败：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

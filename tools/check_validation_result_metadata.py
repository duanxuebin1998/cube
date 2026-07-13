#!/usr/bin/env python3
"""检查流程证据清单引用资料中的验证结果元数据。"""

from __future__ import annotations

import json
import re
import sys
from datetime import date
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = ROOT / "docs" / "00_程序流程导航" / "流程证据清单.json"
MARKER = "cube-validation-result"
OUTPUT_ENCODING = "utf-8"
BLOCK_PATTERN = re.compile(
    rf"<!--\s*{MARKER}\s*(\{{.*?\}})\s*-->",
    flags=re.DOTALL,
)
VERSION_PATTERN = re.compile(r"^V\d+\.\d+\.\d+\.\d+$")
ALLOWED_STATUSES = {"passed", "failed", "partial", "pending"}
REQUIRED_TEXT_FIELDS = ("environment", "source", "summary")


def configure_output(stdout=sys.stdout, stderr=sys.stderr) -> None:  # noqa: ANN001
    """统一子进程输出编码，避免 check_docs 聚合时中文乱码。"""

    for stream in (stdout, stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding=OUTPUT_ENCODING, errors="replace")


def collect_validation_paths(manifest: dict[str, Any]) -> list[str]:
    """收集默认资料和各流程专项资料，保持稳定顺序并去重。"""

    paths: list[str] = []
    seen: set[str] = set()
    candidates = list(manifest.get("defaultValidationPaths", []))
    for page in manifest.get("pages", []):
        candidates.extend(page.get("validationPaths", []))
    for raw_path in candidates:
        if not isinstance(raw_path, str):
            continue
        normalized = raw_path.replace("\\", "/")
        if normalized not in seen:
            seen.add(normalized)
            paths.append(normalized)
    return paths


def parse_result_block(content: str) -> tuple[dict[str, Any] | None, list[str]]:
    """读取单个 JSON 注释块；缺失块允许，非法块返回错误。"""

    blocks = BLOCK_PATTERN.findall(content)
    if not blocks:
        if MARKER in content:
            return None, ["存在元数据标记，但未形成完整的 HTML 注释 JSON 块"]
        return None, []
    if len(blocks) > 1:
        return None, ["同一资料只能包含一个验证结果元数据块"]
    try:
        payload = json.loads(blocks[0])
    except json.JSONDecodeError as exc:
        return None, [f"JSON 语法错误：第 {exc.lineno} 行第 {exc.colno} 列 {exc.msg}"]
    if not isinstance(payload, dict):
        return None, ["元数据顶层必须是 JSON 对象"]
    return payload, []


def validate_result_metadata(payload: dict[str, Any]) -> list[str]:
    """验证字段、日期、状态和固件基线格式。"""

    errors: list[str] = []
    if payload.get("schemaVersion") != 1:
        errors.append("schemaVersion 必须为整数 1")

    status = payload.get("status")
    if status not in ALLOWED_STATUSES:
        errors.append("status 必须是 passed、failed、partial 或 pending")

    executed_at = payload.get("executedAt")
    if not isinstance(executed_at, str):
        errors.append("executedAt 必须是 YYYY-MM-DD 日期字符串")
    else:
        try:
            parsed = date.fromisoformat(executed_at)
            if parsed.isoformat() != executed_at:
                raise ValueError
        except ValueError:
            errors.append("executedAt 必须是有效的 YYYY-MM-DD 日期")

    for field in REQUIRED_TEXT_FIELDS:
        value = payload.get(field)
        if not isinstance(value, str) or not value.strip():
            errors.append(f"{field} 必须是非空字符串")

    baseline = payload.get("firmwareBaseline")
    if not isinstance(baseline, dict) or not baseline:
        errors.append("firmwareBaseline 必须至少声明 cpu2 或 cpu3 基线")
    else:
        unknown = sorted(set(baseline) - {"cpu2", "cpu3"})
        if unknown:
            errors.append(f"firmwareBaseline 包含未知字段：{', '.join(unknown)}")
        for cpu, version in baseline.items():
            if cpu in {"cpu2", "cpu3"} and (
                not isinstance(version, str) or VERSION_PATTERN.fullmatch(version) is None
            ):
                errors.append(f"firmwareBaseline.{cpu} 必须符合 Vx.x.x.x 格式")
    return errors


def resolve_repo_path(relative_path: str) -> Path | None:
    """把清单路径解析到仓库内，拒绝越界路径。"""

    candidate = (ROOT / Path(relative_path)).resolve()
    try:
        candidate.relative_to(ROOT)
    except ValueError:
        return None
    return candidate


def check_manifest(manifest_path: Path = MANIFEST_PATH) -> tuple[list[str], int, int]:
    """返回错误、引用资料数和含结构化结果的资料数。"""

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return [f"无法读取流程证据清单：{exc}"], 0, 0

    errors: list[str] = []
    paths = collect_validation_paths(manifest)
    structured_count = 0
    for relative_path in paths:
        absolute_path = resolve_repo_path(relative_path)
        if absolute_path is None:
            errors.append(f"{relative_path}：路径超出仓库范围")
            continue
        if not absolute_path.is_file():
            errors.append(f"{relative_path}：引用资料不存在")
            continue
        if absolute_path.suffix.lower() != ".md":
            errors.append(f"{relative_path}：验证结果元数据仅支持 Markdown 资料")
            continue
        try:
            content = absolute_path.read_text(encoding="utf-8")
        except UnicodeDecodeError:
            errors.append(f"{relative_path}：资料不是有效 UTF-8，无法安全读取元数据")
            continue

        payload, parse_errors = parse_result_block(content)
        if payload is None:
            errors.extend(f"{relative_path}：{message}" for message in parse_errors)
            continue
        structured_count += 1
        errors.extend(
            f"{relative_path}：{message}"
            for message in validate_result_metadata(payload)
        )
    return errors, len(paths), structured_count


def main() -> int:
    configure_output()
    errors, document_count, structured_count = check_manifest()
    if errors:
        print("验证结果元数据检查失败：", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print(
        "验证结果元数据检查通过："
        f"{document_count} 份引用资料，{structured_count} 份已结构化，"
        f"{document_count - structured_count} 份历史资料暂未结构化。"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

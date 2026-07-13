#!/usr/bin/env python3
"""运行 CUBE 文档库本地快速自检。"""

from __future__ import annotations

import os
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ENCODING = "utf-8"

CHECKS = (
    ("文档结构", "tools/check_docs_structure.py"),
    ("程序流程导航同步", "tools/update_flow_navigation.py"),
    ("程序流程变更影响", "tools/check_flow_impact.py"),
    ("程序流程 HTML 契约", "tools/check_flow_docs.py"),
    ("验证结果元数据", "tools/check_validation_result_metadata.py"),
    ("交付证据覆盖", "tools/check_delivery_evidence_coverage.py"),
    ("交付证据缺口处置", "tools/check_delivery_evidence_gaps.py"),
    ("验证工作包", "tools/check_validation_work_packages.py"),
    ("真实验证运行台账", "tools/check_validation_runs.py"),
    ("验证批次与里程碑决策", "tools/check_validation_batches.py"),
    ("治理事项关闭链", "tools/check_governance_items.py"),
    ("跨版本知识治理", "tools/check_knowledge_governance_snapshots.py"),
    ("交付证据基线", "tools/check_delivery_evidence_baselines.py"),
    ("交付退化策略", "tools/check_delivery_evidence_regression.py"),
    ("Markdown 链接", "tools/check_markdown_links.py"),
)

Runner = Callable[..., subprocess.CompletedProcess[str]]


def configure_output(stdout=sys.stdout, stderr=sys.stderr) -> None:  # noqa: ANN001
    for stream in (stdout, stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding=OUTPUT_ENCODING, errors="replace")


def run_checks(runner: Runner = subprocess.run) -> int:
    child_env = os.environ.copy()
    child_env["PYTHONIOENCODING"] = OUTPUT_ENCODING
    for label, script in CHECKS:
        command = [sys.executable, script]
        print(f"== {label} ==")
        result = runner(
            command,
            cwd=ROOT,
            text=True,
            encoding=OUTPUT_ENCODING,
            errors="replace",
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=child_env,
            check=False,
        )
        if result.stdout:
            print(result.stdout.rstrip())
        if result.stderr:
            print(result.stderr.rstrip(), file=sys.stderr)
        if result.returncode != 0:
            print(f"{label}检查失败，退出码：{result.returncode}", file=sys.stderr)
            return result.returncode
    print("CUBE 文档快速自检通过。")
    return 0


def main() -> int:
    configure_output()
    return run_checks()


if __name__ == "__main__":
    raise SystemExit(main())

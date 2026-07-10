#!/usr/bin/env python3
"""运行 CUBE 文档库本地快速自检。"""

from __future__ import annotations

import subprocess
import sys
from collections.abc import Callable
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_ENCODING = "utf-8"

CHECKS = (
    ("文档结构", "tools/check_docs_structure.py"),
    ("程序流程导航同步", "tools/update_flow_navigation.py"),
    ("程序流程 HTML 契约", "tools/check_flow_docs.py"),
    ("Markdown 链接", "tools/check_markdown_links.py"),
)

Runner = Callable[..., subprocess.CompletedProcess[str]]


def configure_output(stdout=sys.stdout, stderr=sys.stderr) -> None:  # noqa: ANN001
    for stream in (stdout, stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if reconfigure is not None:
            reconfigure(encoding=OUTPUT_ENCODING, errors="replace")


def run_checks(runner: Runner = subprocess.run) -> int:
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

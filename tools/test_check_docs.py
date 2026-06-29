#!/usr/bin/env python3
"""check_docs.py 的聚合行为测试。"""

from __future__ import annotations

import importlib.util
import io
import subprocess
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_docs.py")


def load_module():
    spec = importlib.util.spec_from_file_location("check_docs", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_docs.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckDocsTests(unittest.TestCase):
    def test_runs_structure_then_markdown_links(self) -> None:
        module = load_module()
        calls: list[tuple[str, ...]] = []
        encodings: list[tuple[str | None, str | None]] = []

        def fake_run(command, **kwargs):  # noqa: ANN001
            calls.append(tuple(str(part) for part in command))
            encodings.append((kwargs.get("encoding"), kwargs.get("errors")))
            return subprocess.CompletedProcess(command, 0, stdout="ok\n", stderr="")

        result = module.run_checks(fake_run)

        self.assertEqual(0, result)
        self.assertEqual(
            [
                (sys.executable, "tools/check_docs_structure.py"),
                (sys.executable, "tools/check_markdown_links.py"),
            ],
            calls,
        )
        self.assertEqual(
            [(module.OUTPUT_ENCODING, "replace"), (module.OUTPUT_ENCODING, "replace")],
            encodings,
        )

    def test_returns_failure_when_a_check_fails(self) -> None:
        module = load_module()

        def fake_run(command, **_kwargs):  # noqa: ANN001
            return subprocess.CompletedProcess(command, 7, stdout="", stderr="bad\n")

        result = module.run_checks(fake_run)

        self.assertEqual(7, result)

    def test_structure_check_runs_before_link_check(self) -> None:
        module = load_module()

        self.assertEqual(
            "tools/check_docs_structure.py",
            module.CHECKS[0][1],
        )

    def test_daily_check_does_not_run_migration_staging_check(self) -> None:
        module = load_module()

        scripts = [script for _, script in module.CHECKS]

        self.assertNotIn("tools/check_docs_staging.py", scripts)

    def test_configure_output_uses_utf8_when_stream_supports_reconfigure(self) -> None:
        module = load_module()
        stdout = io.TextIOWrapper(io.BytesIO(), encoding="cp936")
        stderr = io.TextIOWrapper(io.BytesIO(), encoding="cp936")

        module.configure_output(stdout=stdout, stderr=stderr)

        self.assertEqual("utf-8", stdout.encoding.lower().replace("_", "-"))
        self.assertEqual("utf-8", stderr.encoding.lower().replace("_", "-"))


if __name__ == "__main__":
    unittest.main()

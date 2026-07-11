#!/usr/bin/env python3
"""check_version_bumped.py 的流程影响联动测试。"""

from __future__ import annotations

import importlib.util
import io
import subprocess
import sys
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock


MODULE_PATH = Path(__file__).with_name("check_version_bumped.py")


def load_module():
    spec = importlib.util.spec_from_file_location("check_version_bumped", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_version_bumped.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckVersionBumpedTests(unittest.TestCase):
    def test_flow_impact_report_uses_staged_scope(self) -> None:
        module = load_module()
        calls = []

        def fake_runner(command, **kwargs):  # noqa: ANN001
            calls.append((command, kwargs))
            return subprocess.CompletedProcess(command, 0, stdout="impact ok\n", stderr="")

        output = io.StringIO()
        with redirect_stdout(output):
            error = module.report_staged_flow_impact(fake_runner)

        self.assertIsNone(error)
        self.assertEqual(
            [sys.executable, "-X", "utf8", module.FLOW_IMPACT_SCRIPT, "--staged"],
            calls[0][0],
        )
        self.assertIn("impact ok", output.getvalue())

    def test_main_reports_impact_alongside_version_gate(self) -> None:
        module = load_module()
        staged = ["docs/00_程序流程导航/README.md"]
        output = io.StringIO()

        with mock.patch.object(module, "staged_files", return_value=staged):
            with mock.patch.object(module, "report_staged_flow_impact", return_value=None) as impact:
                with redirect_stdout(output):
                    result = module.main()

        self.assertEqual(0, result)
        impact.assert_called_once_with()
        self.assertIn("Version bump check passed", output.getvalue())

    def test_main_fails_when_impact_index_cannot_be_analyzed(self) -> None:
        module = load_module()
        staged = ["docs/00_程序流程导航/README.md"]

        with mock.patch.object(module, "staged_files", return_value=staged):
            with mock.patch.object(
                module,
                "report_staged_flow_impact",
                return_value="Flow impact analysis failed with exit code 2.",
            ):
                with redirect_stdout(io.StringIO()):
                    result = module.main()

        self.assertEqual(1, result)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""验证结果元数据检查器测试。"""

from __future__ import annotations

import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_validation_result_metadata.py")


def load_module():
    spec = importlib.util.spec_from_file_location("check_validation_result_metadata", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_validation_result_metadata.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ValidationResultMetadataTests(unittest.TestCase):
    def test_configure_output_uses_utf8(self) -> None:
        module = load_module()
        stdout = io.TextIOWrapper(io.BytesIO(), encoding="cp936")
        stderr = io.TextIOWrapper(io.BytesIO(), encoding="cp936")

        module.configure_output(stdout=stdout, stderr=stderr)

        self.assertEqual("utf-8", stdout.encoding.lower().replace("_", "-"))
        self.assertEqual("utf-8", stderr.encoding.lower().replace("_", "-"))

    def test_collects_default_and_specific_paths_once(self) -> None:
        module = load_module()
        manifest = {
            "defaultValidationPaths": ["docs/shared.md"],
            "pages": [
                {"validationPaths": ["docs/specific.md", "docs/shared.md"]},
                {"validationPaths": []},
            ],
        }

        self.assertEqual(
            ["docs/shared.md", "docs/specific.md"],
            module.collect_validation_paths(manifest),
        )

    def test_missing_metadata_is_allowed(self) -> None:
        module = load_module()

        payload, errors = module.parse_result_block("# 历史验证资料\n\n尚未结构化。")

        self.assertIsNone(payload)
        self.assertEqual([], errors)

    def test_parses_and_validates_complete_metadata(self) -> None:
        module = load_module()
        content = """<!-- cube-validation-result
{
  "schemaVersion": 1,
  "status": "partial",
  "executedAt": "2026-07-11",
  "environment": "Windows；主机测试；CPU2/CPU3 构建",
  "source": "本文测试矩阵与构建证据",
  "firmwareBaseline": {"cpu2": "V1.22.0.0", "cpu3": "V1.21.0.0"},
  "summary": "自动检查通过，实机项目待执行。"
}
-->"""

        payload, errors = module.parse_result_block(content)

        self.assertEqual([], errors)
        self.assertIsNotNone(payload)
        self.assertEqual([], module.validate_result_metadata(payload))

    def test_rejects_invalid_json_and_incomplete_fields(self) -> None:
        module = load_module()
        payload, errors = module.parse_result_block(
            "<!-- cube-validation-result {\"schemaVersion\": 1,} -->"
        )
        self.assertIsNone(payload)
        self.assertIn("JSON 语法错误", errors[0])

        errors = module.validate_result_metadata({
            "schemaVersion": 2,
            "status": "done",
            "executedAt": "2026-02-30",
            "environment": "",
            "source": "正文",
            "firmwareBaseline": {"cpu2": "1.2.3"},
            "summary": "结果",
        })
        self.assertGreaterEqual(len(errors), 5)

    def test_check_manifest_reports_malformed_block_but_not_missing_block(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            docs = root / "docs"
            docs.mkdir()
            (docs / "legacy.md").write_text("# 历史资料\n", encoding="utf-8")
            (docs / "broken.md").write_text(
                "<!-- cube-validation-result {\"status\": \"partial\",} -->",
                encoding="utf-8",
            )
            manifest_path = root / "manifest.json"
            manifest_path.write_text(json.dumps({
                "defaultValidationPaths": ["docs/legacy.md", "docs/broken.md"],
                "pages": [],
            }), encoding="utf-8")
            original_root = module.ROOT
            module.ROOT = root
            try:
                errors, document_count, structured_count = module.check_manifest(manifest_path)
            finally:
                module.ROOT = original_root

        self.assertEqual(2, document_count)
        self.assertEqual(0, structured_count)
        self.assertEqual(1, len(errors))
        self.assertIn("broken.md", errors[0])


if __name__ == "__main__":
    unittest.main()

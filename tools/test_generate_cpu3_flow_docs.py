#!/usr/bin/env python3
"""generate_cpu3_flow_docs.py 的流程图摘要契约测试。"""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("generate_cpu3_flow_docs.py")


def load_module():
    spec = importlib.util.spec_from_file_location("generate_cpu3_flow_docs", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load generate_cpu3_flow_docs.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class GenerateCpu3FlowDocsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.module = load_module()

    def test_flow_svg_requires_description_at_call_boundary(self) -> None:
        with self.assertRaises(TypeError) as raised:
            self.module.flow_svg(
                "fixture",
                "示例流程图",
                [],
                [],
                accessible_id="fixture-diagram-01",
            )

        self.assertIn("description", str(raised.exception))

    def test_existing_generator_calls_supply_descriptions(self) -> None:
        self.module.append_remaining_pages()

        for page in self.module.PAGES:
            with self.subTest(page=page["file"]):
                rendered = self.module.page_html(page, self.module.PAGES)
                self.assertEqual(
                    len(page["flows"]) + 1,
                    rendered.count('<desc id="'),
                )

        overview = self.module.overview_page(self.module.PAGES)
        self.assertEqual(1, overview.count('<desc id="'))


if __name__ == "__main__":
    unittest.main()

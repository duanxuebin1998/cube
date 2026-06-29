#!/usr/bin/env python3
"""check_markdown_links.py 的最小行为测试。"""

from __future__ import annotations

import importlib.util
import io
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("check_markdown_links.py")


def load_module():
    spec = importlib.util.spec_from_file_location("check_markdown_links", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load check_markdown_links.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class CheckMarkdownLinksTests(unittest.TestCase):
    def test_configure_output_uses_utf8_when_stream_supports_reconfigure(self) -> None:
        module = load_module()
        stdout = io.TextIOWrapper(io.BytesIO(), encoding="cp936")
        stderr = io.TextIOWrapper(io.BytesIO(), encoding="cp936")

        module.configure_output(stdout=stdout, stderr=stderr)

        self.assertEqual("utf-8", stdout.encoding.lower().replace("_", "-"))
        self.assertEqual("utf-8", stderr.encoding.lower().replace("_", "-"))

    def test_accepts_existing_relative_links_and_skips_non_file_links(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            docs = root / "docs"
            docs.mkdir()
            (docs / "guide.md").write_text("# Guide\n", encoding="utf-8")
            (docs / "asset.txt").write_text("asset\n", encoding="utf-8")
            (docs / "README.md").write_text(
                "\n".join(
                    [
                        "# Home",
                        "[Guide](guide.md)",
                        "[Asset](asset.txt)",
                        "[External](https://example.com/a.md)",
                        "[Anchor](#local-anchor)",
                        "```",
                        "[Ignored](missing.md)",
                        "```",
                        "[Reference][ref]",
                        "",
                        "[ref]: guide.md",
                    ]
                ),
                encoding="utf-8",
            )

            problems = module.check_markdown_links(docs, root)

        self.assertEqual([], problems)

    def test_reports_missing_relative_link(self) -> None:
        module = load_module()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            docs = root / "docs"
            docs.mkdir()
            (docs / "broken.md").write_text("[Missing](missing.md)\n", encoding="utf-8")

            problems = module.check_markdown_links(docs, root)

        self.assertEqual(1, len(problems))
        self.assertEqual("docs/broken.md", problems[0].source)
        self.assertEqual(1, problems[0].line)
        self.assertEqual("missing.md", problems[0].target)


if __name__ == "__main__":
    unittest.main()

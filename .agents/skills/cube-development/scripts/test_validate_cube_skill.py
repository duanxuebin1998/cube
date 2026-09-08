"""Regression tests for skill metadata and progressive reference routing."""

import tempfile
import unittest
from pathlib import Path

from validate_cube_skill import validate_description, validate_reference_routes


class SkillValidationTests(unittest.TestCase):
    def test_short_description_is_valid(self):
        validate_description("CUBE firmware and protocol workflows.")

    def test_empty_and_oversized_descriptions_fail(self):
        for description in ("", "   ", "x" * 1025):
            with self.subTest(description_length=len(description)):
                with self.assertRaises(ValueError):
                    validate_description(description)

    def test_nested_routes_and_cycles_are_reachable(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            refs = root / "references"
            (refs / "nested").mkdir(parents=True)
            (root / "SKILL.md").write_text("`references/start.md`\n", encoding="utf-8")
            (refs / "start.md").write_text("[Detail](nested/detail.md#checks)\n", encoding="utf-8")
            (refs / "nested/detail.md").write_text("[Start](../start.md)\n", encoding="utf-8")
            self.assertEqual(2, len(validate_reference_routes(root)))

    def test_missing_nested_reference_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "references").mkdir()
            (root / "SKILL.md").write_text("[Start](references/start.md)\n", encoding="utf-8")
            (root / "references/start.md").write_text("[Missing](missing.md)\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "Missing routed reference"):
                validate_reference_routes(root)

    def test_orphan_reference_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "references").mkdir()
            (root / "SKILL.md").write_text("# Entry\n", encoding="utf-8")
            (root / "references/orphan.md").write_text("# Orphan\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "not reachable"):
                validate_reference_routes(root)

    def test_reference_cannot_escape_skill(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "skill"
            root.mkdir()
            (root.parent / "outside.md").write_text("# Outside\n", encoding="utf-8")
            (root / "SKILL.md").write_text("[Outside](../outside.md)\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "leaves skill"):
                validate_reference_routes(root)

    def test_external_links_do_not_create_local_routes(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "SKILL.md").write_text("[Docs](https://example.com/doc.md)\n", encoding="utf-8")
            self.assertEqual(set(), validate_reference_routes(root))


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3
"""Tests for commit subject version checking."""

from __future__ import annotations

import unittest

from tools import check_commit_subject_versions as check


class CommitSubjectVersionTests(unittest.TestCase):
    def test_accepts_both_cpu_versions(self) -> None:
        subject = "fix: 修复编码器通信诊断（CPU2 V1.20.2.0 / CPU3 V1.18.2.0）"

        self.assertTrue(check.subject_has_version(subject))

    def test_accepts_single_cpu_version(self) -> None:
        subject = "fix: 修复固定点定位（CPU2 V1.16.2.0）"

        self.assertTrue(check.subject_has_version(subject))

    def test_rejects_missing_version(self) -> None:
        subject = "docs: 统一整机文档库治理与迁移"

        self.assertFalse(check.subject_has_version(subject))

    def test_cross_cpu_paths_require_both_versions(self) -> None:
        required = check.required_cpus_for_paths(["docs/00_构建与版本/README.md"])
        subject = "docs: 更新版本索引（CPU2 V1.20.2.0）"

        self.assertEqual({"CPU2", "CPU3"}, required)
        self.assertEqual({"CPU3"}, check.missing_required_cpu_versions(subject, required))

    def test_cpu2_only_paths_require_cpu2_version(self) -> None:
        required = check.required_cpus_for_paths(["LTD_MAIN_CPU2/Application/Src/app_main.c"])
        subject = "fix: 修复主循环门禁（CPU2 V1.20.3.0）"

        self.assertEqual({"CPU2"}, required)
        self.assertEqual(set(), check.missing_required_cpu_versions(subject, required))

    def test_unknown_paths_still_require_a_version_token(self) -> None:
        required = check.required_cpus_for_paths(["README.tmp"])
        subject = "chore: 临时整理"

        self.assertEqual(set(), required)
        self.assertEqual({"CPU2/CPU3"}, check.missing_required_cpu_versions(subject, required))

    def test_returns_missing_subjects(self) -> None:
        subjects = [
            check.CommitSubject("a1", "fix: 通过（CPU3 V1.18.2.0）"),
            check.CommitSubject("b2", "fix: 缺少版本"),
        ]

        self.assertEqual([subjects[1]], check.missing_version_subjects(subjects))

    def test_commit_message_subject_ignores_comments_and_blank_lines(self) -> None:
        message = "\n# comment\nfix: 修复菜单退出（CPU2 V1.20.1.0 / CPU3 V1.18.1.0）\n\nbody"

        self.assertEqual(
            "fix: 修复菜单退出（CPU2 V1.20.1.0 / CPU3 V1.18.1.0）",
            check.subject_from_message(message),
        )


if __name__ == "__main__":
    unittest.main()

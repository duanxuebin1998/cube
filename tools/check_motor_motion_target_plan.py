#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""检查电机运动入口是否共用目标规划 helper。"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


SOURCE_PATH = Path("LTD_MAIN_CPU2/Services/MotorControl/motor_ctrl_motion_api.c")


def decode_source(raw: bytes) -> str:
    for encoding in ("gbk", "utf-8"):
        try:
            return raw.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("source", raw, 0, 1, "source is neither GBK nor UTF-8")


def read_source(args: argparse.Namespace) -> str:
    if args.git_ref:
        blob = f"{args.git_ref}:{SOURCE_PATH.as_posix()}"
        raw = subprocess.check_output(["git", "show", blob])
        return decode_source(raw)
    return decode_source(Path(args.source).read_bytes())


def extract_function(text: str, name: str) -> str:
    marker = f"uint32_t {name}("
    start = -1
    while True:
        start = text.find(marker, start + 1)
        if start < 0:
            raise AssertionError(f"missing function: {name}")

        brace = text.find("{", start)
        if brace < 0:
            raise AssertionError(f"missing function body: {name}")

        semicolon = text.find(";", start)
        if 0 <= semicolon < brace:
            continue

        depth = 0
        for index in range(brace, len(text)):
            if text[index] == "{":
                depth += 1
            elif text[index] == "}":
                depth -= 1
                if depth == 0:
                    return text[brace:index + 1]

    raise AssertionError(f"unterminated function body: {name}")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def validate(text: str) -> list[str]:
    failures: list[str] = []

    require("} MotorMotionTargetPlan;" in text,
            "missing MotorMotionTargetPlan type", failures)
    require("MotorMotion_BuildRelativeTargetPlan(" in text,
            "missing relative target plan helper", failures)
    require("MotorMotion_BuildAbsoluteTargetPlan(" in text,
            "missing absolute target plan helper", failures)
    require("MotorMotion_CheckCommandAbortWithSpeedScope(" in text,
            "missing command abort helper with speed scope", failures)
    require("MotorMotion_CheckReadyForProtectedMotion(" in text,
            "missing protected motion readiness helper", failures)
    require("MotorMotion_WaitMoveStartObservation(" in text,
            "missing move start observation helper", failures)
    require("MotorMotion_CheckJogRuntimeGuards(" in text,
            "missing jog runtime guard helper", failures)
    require("MotorMotion_CheckAbortRefreshAndHealth(" in text,
            "missing wait-loop abort refresh health helper", failures)
    require("MotorMotion_CheckStoppedAndRefresh(" in text,
            "missing stopped refresh helper", failures)

    expected_calls = {
        "MotorCtrl_MoveAndWait": "MotorMotion_BuildRelativeTargetPlan",
        "MotorCtrl_JogMoveAndWait": "MotorMotion_BuildRelativeTargetPlan",
        "MotorCtrl_MoveToPosition": "MotorMotion_BuildAbsoluteTargetPlan",
        "MotorCtrl_JogMoveToPosition": "MotorMotion_BuildAbsoluteTargetPlan",
    }
    for function_name, helper_name in expected_calls.items():
        try:
            body = extract_function(text, function_name)
        except AssertionError as exc:
            failures.append(str(exc))
            continue
        require(helper_name in body,
                f"{function_name} does not call {helper_name}", failures)

    try:
        move_and_wait_body = extract_function(text, "MotorCtrl_MoveAndWait")
    except AssertionError as exc:
        failures.append(str(exc))
    else:
        require("MotorMotion_CheckCommandAbortWithSpeedScope" in move_and_wait_body,
                "MotorCtrl_MoveAndWait does not use command abort helper", failures)
        require("MotorMotion_WaitMoveStartObservation" in move_and_wait_body,
                "MotorCtrl_MoveAndWait does not use move start observation helper", failures)

    readiness_callers = (
        "MotorCtrl_MoveAndWait",
        "MotorCtrl_JogMoveAndWait",
        "MotorCtrl_JogMoveToPosition",
        "MotorMotion_JogMoveToTargetInternal",
        "MotorMotion_MoveBlockingNoDetectInternal",
    )
    for function_name in readiness_callers:
        try:
            body = extract_function(text, function_name)
        except AssertionError as exc:
            failures.append(str(exc))
            continue
        require("MotorMotion_CheckReadyForProtectedMotion" in body,
                f"{function_name} does not use protected motion readiness helper",
                failures)

    try:
        jog_body = extract_function(text, "MotorMotion_JogMoveToTargetInternal")
    except AssertionError as exc:
        failures.append(str(exc))
    else:
        require("MotorMotion_CheckJogRuntimeGuards" in jog_body,
                "MotorMotion_JogMoveToTargetInternal does not use jog runtime guard helper",
                failures)

    wait_loop_callers = (
        "MotorMotion_WaitTicksStartChanged",
        "MotorMotion_MoveBlockingNoDetectInternal",
        "MotorMotion_WaitStopAbortable",
    )
    for function_name in wait_loop_callers:
        try:
            body = extract_function(text, function_name)
        except AssertionError as exc:
            failures.append(str(exc))
            continue
        require("MotorMotion_CheckAbortRefreshAndHealth" in body,
                f"{function_name} does not use wait-loop abort refresh health helper",
                failures)

    stopped_refresh_callers = (
        "MotorMotion_WaitStopAbortable",
        "MotorMotion_WaitUntilStopWithTarget",
        "MotorMotion_WaitStoppedAfterStopCommand",
    )
    for function_name in stopped_refresh_callers:
        try:
            body = extract_function(text, function_name)
        except AssertionError as exc:
            failures.append(str(exc))
            continue
        require("MotorMotion_CheckStoppedAndRefresh" in body,
                f"{function_name} does not use stopped refresh helper",
                failures)

    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=str(SOURCE_PATH),
                        help="source file to check, default is motor_ctrl_motion_api.c")
    parser.add_argument("--git-ref", help="read source from git ref instead of worktree")
    args = parser.parse_args()

    failures = validate(read_source(args))
    if failures:
        print("Motor motion target plan check failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("Motor motion target plan check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

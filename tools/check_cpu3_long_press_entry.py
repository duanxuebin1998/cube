#!/usr/bin/env python3
"""Check CPU3 long-press menu-entry robustness constraints."""

from __future__ import annotations

from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]
EXIT_H = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "exit.h"
EXIT_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "exit.c"
DISPLAY_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display.c"
IT_C = ROOT / "LTD_DISPLAY_CPU3" / "Core" / "Src" / "stm32f4xx_it.c"


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def require(condition: bool, message: str, failures: list[str]) -> None:
    if not condition:
        failures.append(message)


def main() -> int:
    exit_h = read_text(EXIT_H)
    exit_c = read_text(EXIT_C)
    display_c = read_text(DISPLAY_C)
    it_c = read_text(IT_C)
    failures: list[str] = []

    require(
        re.search(r"#define\s+REQUIRED_PRESS_COUNT\s+15\b", exit_h) is not None,
        "REQUIRED_PRESS_COUNT must be 15 for about 1.5s long press",
        failures,
    )
    require(
        re.search(r"#define\s+DISPLAY_KEY_DEBOUNCE_MS\s+50U\b", exit_h) is not None,
        "DISPLAY_KEY_DEBOUNCE_MS must be 50U",
        failures,
    )
    require(
        re.search(r"#define\s+DISPLAY_LONG_PRESS_RELEASE_GUARD_MS\s+50U\b", exit_h) is not None,
        "DISPLAY_LONG_PRESS_RELEASE_GUARD_MS must be 50U",
        failures,
    )
    require(
        "Display_StartLongPress" in exit_c
        and "button_long_press_key == LONG_PRESS_KEY_NONE" in exit_c
        and "long_press_release_guard_key == LONG_PRESS_KEY_NONE" in exit_c,
        "long-press start must lock an existing target instead of resetting it",
        failures,
    )
    require(
        "Display_ReadLongPressKey(button_long_press_key) == GPIO_PIN_SET" in exit_c
        and "button_long_press_key = LONG_PRESS_KEY_NONE" in exit_c,
        "a released stale long-press target must be cleared before starting a new target",
        failures,
    )
    require(
        "__HAL_TIM_SET_COUNTER(&htim1, 0)" in exit_c
        and "__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE)" in exit_c,
        "long-press timer must be reset before start",
        failures,
    )
    require(
        "Display_ArmLongPressReleaseGuard" in exit_c
        and "Display_UpdateLongPressReleaseGuard" in exit_c
        and "Display_ArmLongPressReleaseGuard(long_press_key)" in it_c,
        "long-press trigger must arm release guard",
        failures,
    )
    require(
        "Display_ClearPendingKeys" in exit_c and "pending_key_tail = pending_key_head" in exit_c,
        "long-press trigger must clear pending normal keys",
        failures,
    )
    require(
        "Display_UpdateLongPressReleaseGuard();" in display_c,
        "Display_Task must poll release guard",
        failures,
    )
    require(
        re.search(
            r"if\s*\(long_press_key\s*!=\s*LONG_PRESS_KEY_NONE\)\s*\{[^{}]*"
            r"Display_ProcessLongPressAction\(long_press_key\);[^{}]*return;",
            display_c,
            re.S,
        )
        is not None,
        "Display_ProcessPendingInput must return after handling a long press",
        failures,
    )

    if failures:
        print("CPU3 long-press entry checks failed:")
        for failure in failures:
            print(f"- {failure}")
        return 1

    print("CPU3 long-press entry checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

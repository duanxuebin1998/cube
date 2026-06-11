from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8", errors="ignore")


def function_body(source: str, name: str) -> str:
    match = re.search(
        r"\b(?:void|bool|HAL_StatusTypeDef)\s+" + re.escape(name) + r"\s*\([^)]*\)\s*\{",
        source,
    )
    if match is None:
        raise AssertionError(f"function not found: {name}")

    start = match.end()
    depth = 1
    i = start
    while i < len(source):
        ch = source[i]
        if ch == "{":
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                return source[start:i]
        i += 1

    raise AssertionError(f"function body not closed: {name}")


def require_absent(body: str, patterns: list[str], scope: str) -> list[str]:
    errors: list[str] = []
    for pattern in patterns:
        if re.search(pattern, body):
            errors.append(f"{scope} contains forbidden pattern: {pattern}")
    return errors


def main() -> int:
    errors: list[str] = []

    it_c = read("LTD_DISPLAY_CPU3/Core/Src/stm32f4xx_it.c")
    exit_c = read("LTD_DISPLAY_CPU3/Application/display/exit.c")
    app_main_c = read("LTD_DISPLAY_CPU3/Application/app_main.c")
    display_c = read("LTD_DISPLAY_CPU3/Application/display/display.c")
    display_h = read("LTD_DISPLAY_CPU3/Application/display/display.h")
    display_tankopera_c = read("LTD_DISPLAY_CPU3/Application/display/display_tankopera.c")
    display_tankopera_h = read("LTD_DISPLAY_CPU3/Application/display/display_tankopera.h")
    exit_h = read("LTD_DISPLAY_CPU3/Application/display/exit.h")
    hgs_c = read("LTD_DISPLAY_CPU3/Application/display/hgs.c")
    hgs_h = read("LTD_DISPLAY_CPU3/Application/display/hgs.h")

    tim3_body = function_body(it_c, "TIM3_IRQHandler")
    tim1_body = function_body(it_c, "TIM1_UP_TIM10_IRQHandler")
    exti_body = function_body(exit_c, "HAL_GPIO_EXTI_Callback")
    mainloop_body = function_body(app_main_c, "App_MainLoop")
    display_task_body = function_body(display_c, "Display_Task")
    refresh_body = function_body(display_c, "RefreshScreen")
    workingdata_body = function_body(display_c, "oled_workingdata")
    keyprocess_body = function_body(display_tankopera_c, "KeyProcess")
    set_window_body = function_body(hgs_c, "OLED_SetWindow")
    write_data_buffer_body = function_body(hgs_c, "WriteDataBuffer")
    write_fill_body = function_body(hgs_c, "OLED_WriteFillData")

    errors += require_absent(
        tim3_body,
        [r"\bRefreshScreen\s*\(", r"\boled_clear\s*\(", r"\ball_screen\s*\("],
        "TIM3_IRQHandler",
    )
    if not re.search(r"\bDisplay_RequestRefresh\s*\(", tim3_body):
        errors.append("TIM3_IRQHandler should only request display refresh")

    errors += require_absent(
        tim1_body,
        [
            r"\buseKey\s*\(",
            r"\bKeyProcess\s*\(",
            r"\bexecute_opera\s*\(",
            r"\bDisplay_EnterCancelMeasurementConfirm\s*\(",
            r"\ball_screen\s*\(",
            r"\boled_clear\s*\(",
        ],
        "TIM1_UP_TIM10_IRQHandler",
    )
    if not re.search(r"\bDisplay_RequestLongPressAction\s*\(", tim1_body):
        errors.append("TIM1_UP_TIM10_IRQHandler should request long-press action")

    errors += require_absent(
        exti_body,
        [
            r"\bKeyProcess\s*\(",
            r"\bprintf\s*\(",
            r"\ball_screen\s*\(",
            r"\boled_clear\s*\(",
        ],
        "HAL_GPIO_EXTI_Callback",
    )
    if not re.search(r"\bDisplay_RequestKey\s*\(", exti_body):
        errors.append("HAL_GPIO_EXTI_Callback should request key processing")

    if not re.search(r"\bDisplay_Task\s*\(", mainloop_body):
        errors.append("App_MainLoop should call Display_Task")
    if "void Display_Task(void);" not in display_h:
        errors.append("display.h should declare Display_Task")
    if "void Display_RequestRefresh(void);" not in display_h:
        errors.append("display.h should declare Display_RequestRefresh")
    if "void Display_RequestKey(uint8_t keypress);" not in exit_h:
        errors.append("exit.h should declare Display_RequestKey")
    if "void Display_RequestLongPressAction(uint8_t long_press_key);" not in exit_h:
        errors.append("exit.h should declare Display_RequestLongPressAction")
    if "oled_shadow_buffer" not in hgs_c:
        errors.append("hgs.c should maintain an internal OLED shadow buffer")
    if "uint32_t OLED_GetShadowCrc(void);" not in hgs_h:
        errors.append("hgs.h should expose OLED_GetShadowCrc for software consistency checks")
    if "uint32_t OLED_GetRefreshSeq(void);" not in hgs_h:
        errors.append("hgs.h should expose OLED_GetRefreshSeq for refresh liveness checks")
    if "OLED_MarkFrameComplete" not in hgs_c:
        errors.append("hgs.c should record completed display frames")
    if "pending_key_mask" in exit_c:
        errors.append("exit.c should queue key events instead of coalescing them in pending_key_mask")
    if "pending_key_queue" not in exit_c:
        errors.append("exit.c should keep a FIFO queue for pending key events")
    if re.search(r"RefreshScreen\s*\(\);\s*OLED_MarkFrameComplete\s*\(", display_task_body, re.S):
        errors.append("Display_Task should not mark a frame complete when RefreshScreen does not draw")
    if "DisplayTankOpera_RedrawCurrentPage" not in refresh_body:
        errors.append("RefreshScreen should recover and redraw the current menu page while FlagofTankOpera is true")
    if "DisplayTankOpera_RedrawCurrentPage" not in display_tankopera_c:
        errors.append("display_tankopera.c should expose a side-effect-safe current page redraw helper")
    if "bool DisplayTankOpera_RedrawCurrentPage(void);" not in display_tankopera_h:
        errors.append("display_tankopera.h should declare DisplayTankOpera_RedrawCurrentPage")
    if "bool KeyProcess(uint8_t keypress);" not in display_tankopera_h:
        errors.append("display_tankopera.h should make KeyProcess return whether a page was drawn")
    if "bool DisplayTankOpera_CanProcessKey(uint8_t keypress);" not in display_tankopera_h:
        errors.append("display_tankopera.h should declare DisplayTankOpera_CanProcessKey for pre-draw wake/recover")
    if "bool Display_CanEnterCancelMeasurementConfirm(void);" not in display_tankopera_h:
        errors.append("display_tankopera.h should declare Display_CanEnterCancelMeasurementConfirm")
    if not re.search(r"func_index\s*<\s*0.*func_index\s*>=\s*KEYNUM_END", keyprocess_body, re.S):
        errors.append("KeyProcess should guard func_index before indexing keymenu")
    if "Display_PrepareForForegroundDraw" not in display_c:
        errors.append("display.c should wake/recover the OLED before foreground menu drawing")
    if "display_status_full_redraw_required" not in display_c:
        errors.append("display.c should track when foreground pages require a full status redraw")
    if not re.search(r"Display_PrepareForForegroundDraw[\s\S]*display_status_full_redraw_required\s*=\s*true", display_c):
        errors.append("foreground menu/key drawing should invalidate the status-page partial refresh cache")
    if not re.search(r"oled_workingdata[\s\S]*display_status_full_redraw_required[\s\S]*Display_DrawStatusFull", display_c):
        errors.append("oled_workingdata should force a full status redraw after non-status foreground pages")
    if not re.search(r"Display_DrawStatusFull[\s\S]*display_status_full_redraw_required\s*=\s*false", display_c):
        errors.append("Display_DrawStatusFull should clear the full-redraw-required flag")
    if "Display_FinishFrame" not in display_c or "OLED_GetSpiErrorCount() == frame_spi_error_start" not in display_c:
        errors.append("display.c should mark a frame complete only when no new SPI error occurred")
    if not re.search(r"OLED_ShadowSetWindow", set_window_body):
        errors.append("OLED_SetWindow should maintain the OLED shadow window")
    if len(re.findall(r"\bOLED_ShadowSetWindow\s*\(", hgs_c)) != 3:
        errors.append("OLED shadow window should only be changed from OLED_SetWindow")
    if not re.search(r"if\s*\(\s*status\s*==\s*HAL_OK\s*\).*OLED_ShadowWriteBuffer", write_data_buffer_body, re.S):
        errors.append("WriteDataBuffer should update the OLED shadow buffer only after HAL_OK")
    if not re.search(r"if\s*\(\s*status\s*!=\s*HAL_OK\s*\).*return\s+status\s*;", write_fill_body, re.S):
        errors.append("OLED_WriteFillData should stop on the first SPI block failure")
    if "void OLED_ClearArea(uint8_t x, uint8_t y, uint8_t width_cols, uint8_t height_rows);" not in hgs_h:
        errors.append("hgs.h should expose OLED_ClearArea for partial status-page redraws")
    if "void OLED_ClearArea" not in hgs_c:
        errors.append("hgs.c should implement OLED_ClearArea")
    if "Display_ClearBeforeDraw" in workingdata_body:
        errors.append("oled_workingdata should not clear the full screen on every ordinary status refresh")
    if "DisplayStatusSnapshot" not in display_c:
        errors.append("display.c should keep status-page snapshots for layout/delta redraw decisions")
    if "Display_DrawStatusFull" not in display_c:
        errors.append("display.c should have a full status-page redraw path")
    if "Display_DrawStatusDelta" not in display_c:
        errors.append("display.c should have a delta status-page redraw path")
    if "Display_StatusLayoutChanged" not in display_c:
        errors.append("display.c should compare status-page layouts before clearing the full screen")
    if "DISPLAY_VALUE_HIGHLIGHT_FRAMES" in display_c:
        errors.append("display.c should not time value highlights by refresh frames")
    if "DISPLAY_VALUE_HIGHLIGHT_MS 500U" not in display_c:
        errors.append("display.c should highlight changed values for 500ms")
    if "highlight_until_tick" not in display_c or "highlight_visible" not in display_c:
        errors.append("display.c should track value highlight by visible flag and expiry tick")
    if "Display_ShouldRequestStatusHighlightRefresh" not in display_c:
        errors.append("display.c should request a status refresh when a value highlight expires")
    if "Display_StatusTickBefore" not in display_c:
        errors.append("display.c should compare highlight expiry ticks with wraparound-safe logic")
    if "HAL_GetTick()" not in function_body(display_c, "Display_UpdateStatusHighlights"):
        errors.append("Display_UpdateStatusHighlights should use HAL_GetTick for 500ms highlights")
    if "Display_ShouldRequestStatusHighlightRefresh" not in display_task_body:
        errors.append("Display_Task should schedule a refresh when 500ms value highlight expires")
    if "DISPLAY_SCREEN_OFF_IDLE_MS" not in display_c:
        errors.append("display.c should use an idle timeout for screen-off instead of clearing brightness every refresh")

    if errors:
        for error in errors:
            print(error)
        return 1

    print("CPU3 display ISR boundary checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

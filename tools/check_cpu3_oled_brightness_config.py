from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[1]


def read(path: str) -> str:
    return (ROOT / path).read_text(encoding="utf-8", errors="ignore")


def function_body(source: str, name: str) -> str:
    match = re.search(
        r"\b(?:void|bool|uint8_t|static\s+void|static\s+uint8_t)\s+"
        + re.escape(name)
        + r"\s*\([^)]*\)\s*\{",
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


def expect(condition: bool, message: str, errors: list[str]) -> None:
    if not condition:
        errors.append(message)


def main() -> int:
    errors: list[str] = []

    hgs_c = read("LTD_DISPLAY_CPU3/Application/display/hgs.c")
    hgs_h = read("LTD_DISPLAY_CPU3/Application/display/hgs.h")
    display_h = read("LTD_DISPLAY_CPU3/Application/display/display.h")
    tank_h = read("LTD_DISPLAY_CPU3/Application/display/display_tankopera.h")
    tank_c = read("LTD_DISPLAY_CPU3/Application/display/display_tankopera.c")
    state_h = read("LTD_DISPLAY_CPU3/Application/system_param/stateformodbus.h")
    system_param_c = read("LTD_DISPLAY_CPU3/Application/system_param/system_parameter.c")
    cpu3_param_h = read("LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.h")
    cpu3_param_c = read("LTD_DISPLAY_CPU3/Application/system_param/cpu3_comm_display_params.c")

    expect("COM_NUM_SCREEN_BRIGHTNESS" in tank_h, "missing COM_NUM_SCREEN_BRIGHTNESS enum", errors)
    expect(
        re.search(r"COM_NUM_CPU3_COM3_PROTOCOL,.*\n\s*COM_NUM_SCREEN_BRIGHTNESS,.*\n\s*COM_NUM_PARA_LOCAL_STOP", tank_h) is not None,
        "COM_NUM_SCREEN_BRIGHTNESS should append after existing CPU3 COM params to avoid shifting old operation numbers",
        errors,
    )
    expect("HOLDREGISTER_CPU3_BRIGHTNESS" in state_h, "missing HOLDREGISTER_CPU3_BRIGHTNESS register", errors)
    expect(
        "HOLDREGISTER_CPU3_BASE + 0x1C" in state_h,
        "CPU3 brightness should use the 0x021C gap before COM1",
        errors,
    )
    expect("screen_brightness" in cpu3_param_h, "Cpu3CommAndDisplayParams should persist screen_brightness", errors)
    expect(
        re.search(r"#define\s+CPU3_PARAM_VERSION\s+0x0004U", cpu3_param_c) is not None,
        "CPU3 FRAM parameter version should be bumped to 0x0004 for the new persisted field",
        errors,
    )
    expect(
        re.search(r"screen_brightness\s*=\s*OLED_BRIGHTNESS_LEVEL_LOW", cpu3_param_c) is not None,
        "default brightness level should preserve current 0x20 brightness",
        errors,
    )
    expect(
        re.search(r"case\s+COM_NUM_SCREEN_BRIGHTNESS\s*:\s*return\s+g_cpu3_comm_display_params\.screen_brightness", cpu3_param_c, re.S) is not None,
        "Cpu3Local_ReadValue should return screen_brightness",
        errors,
    )
    expect(
        re.search(r"case\s+COM_NUM_SCREEN_BRIGHTNESS\s*:.*OLED_SetBrightnessLevel", cpu3_param_c, re.S) is not None,
        "Cpu3Local_WriteValue should apply brightness immediately",
        errors,
    )
    expect(
        re.search(r"void\s+Cpu3Local_ApplyDisplayRuntimeParams\s*\(\s*void\s*\)\s*;", cpu3_param_h) is not None,
        "missing runtime display parameter apply API",
        errors,
    )
    expect("Cpu3Local_ApplyDisplayRuntimeParams();" in cpu3_param_c, "FRAM load/default path should apply brightness", errors)
    expect("Cpu3Local_ApplyDisplayRuntimeParams();" in system_param_c, "InputValueInit should refresh screen_parameter and brightness", errors)

    expect("OLED_BRIGHTNESS_LEVEL_COUNT" in display_h, "display.h should define OLED brightness level constants", errors)
    for value in ("0x10U", "0x20U", "0x30U", "0x40U", "0x50U"):
        expect(value in hgs_c, f"missing SSD1325 brightness mapping value {value}", errors)
    expect("void OLED_SetContrast(uint8_t contrast);" in hgs_h, "hgs.h should expose OLED_SetContrast", errors)
    expect("void OLED_SetBrightnessLevel(uint8_t level);" in hgs_h, "hgs.h should expose OLED_SetBrightnessLevel", errors)
    expect("OLED_WriteInitCommands" in hgs_c, "OLED init commands should be centralized", errors)
    expect("OLED_ConfiguredContrast" in hgs_c, "OLED init should use configured brightness instead of a literal contrast", errors)

    try:
        init_body = function_body(hgs_c, "OLED_Init")
    except AssertionError as exc:
        errors.append(str(exc))
        init_body = ""
    expect("all_screen(" not in init_body, "OLED_Init should not call all_screen and reset/init twice", errors)
    expect(len(re.findall(r"\bWriteCommand\s*\(\s*0x81\s*\)", init_body)) == 0, "OLED_Init should not hard-code 0x81 manually", errors)

    expect("arr_oled_brightness" in tank_c, "menu should provide brightness level names", errors)
    expect("COM_NUM_SCREEN_BRIGHTNESS" in tank_c, "brightness parameter should be routed through display/menu code", errors)
    expect("屏幕亮度" in system_param_c, "parameter table should expose Chinese name 屏幕亮度", errors)
    expect("Brightness" in system_param_c, "parameter table should expose English name Brightness", errors)

    if errors:
        for error in errors:
            print(error)
        return 1

    print("CPU3 OLED brightness configuration checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())

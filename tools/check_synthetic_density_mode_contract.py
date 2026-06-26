from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "measure.c"


def require(text: str, pattern: str, message: str) -> None:
    if re.search(pattern, text, re.S) is None:
        raise AssertionError(message)


def main() -> None:
    source = SOURCE.read_text(encoding="gbk")

    require(
        source,
        r"static\s+DensitySpreadModeId\s+CMD_SyntheticDensityModeFromParam\s*\(\s*void\s*\)",
        "synthetic density mode mapper is missing",
    )
    require(
        source,
        r"switch\s*\(\s*g_deviceParams\.spreadMeasurementMode\s*\)",
        "mapper must use spreadMeasurementMode",
    )
    require(source, r"case\s+1U\s*:\s*return\s+DENS_MODE_GB\s*;", "mode 1 must select GB measurement")
    require(source, r"case\s+2U\s*:\s*return\s+DENS_MODE_METER\s*;", "mode 2 must select meter measurement")
    require(source, r"case\s+3U\s*:\s*return\s+DENS_MODE_INTERVAL\s*;", "mode 3 must select interval measurement")
    require(source, r"case\s+0U\s*:\s*default\s*:\s*return\s+DENS_MODE_SPREAD\s*;", "mode 0/default must select spread measurement")

    require(
        source,
        r"DensitySpreadModeId\s+density_mode\s*=\s*CMD_SyntheticDensityModeFromParam\s*\(\s*\)\s*;",
        "CMD_SyntheticMeasurement must compute density mode from parameter",
    )
    require(
        source,
        r"Density_MeasureByMode_Exact\s*\(\s*density_mode\s*,\s*&temp\s*\)",
        "CMD_SyntheticMeasurement must call density core with selected mode",
    )
    require(
        source,
        r"if\s*\(\s*ret\s*==\s*STATE_SWITCH\s*\)\s*\{\s*return\s*;\s*\}",
        "CMD_SyntheticMeasurement must return directly on command switch",
    )
    require(
        source,
        r"if\s*\(\s*ret\s*!=\s*NO_ERROR\s*\)\s*\{[^}]*SET_ERROR\s*\(\s*ret\s*\)\s*;[^}]*return\s*;",
        "CMD_SyntheticMeasurement must not publish failed temporary results",
    )

    if "Density_MeasureByMode_Exact(DENS_MODE_SPREAD, &temp)" in source:
        raise AssertionError("CMD_SyntheticMeasurement must not hard-code DENS_MODE_SPREAD")


if __name__ == "__main__":
    main()

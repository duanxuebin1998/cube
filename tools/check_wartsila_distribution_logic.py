from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "wartsila_density_measurement.c"
MEASURE_SOURCE = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "measure.c"


def read_text(path: Path) -> str:
    return path.read_text(encoding="gbk")


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise AssertionError(f"missing {label}: {needle}")


def forbid(text: str, needle: str, label: str) -> None:
    if needle in text:
        raise AssertionError(f"forbidden {label}: {needle}")


def main() -> int:
    source = read_text(SOURCE)
    measure_source = read_text(MEASURE_SOURCE)

    require(source, "Wartsila_IsAirPoint", "air classification helper")
    require(source, "density_value < WARTSILA_AIR_DENSITY_THRESHOLD", "actual density air threshold")
    require(source, "frequency_hz > (float)g_deviceParams.oilLevelFrequency", "float frequency air threshold")
    require(source, "Wartsila_MoveDownToLiquidAfterAirPoint", "slow downward liquid recognition helper")
    require(source, "MotorCtrl_MoveDown(WARTSILA_LEVEL_DOWN_SPEED_X100)", "slow downward move")
    require(source, "MotorCtrl_LostStepInit();", "lost step init before slow downward search")
    require(source, "MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length)", "lost step check during slow downward search")
    require(source, 'if (!is_moving) {\n            ret = MotorCtrl_MoveDown(WARTSILA_LEVEL_DOWN_SPEED_X100);', "continue downward after segment stop")
    require(source, "*level_mm = cur_mm;", "current position captured as level")
    require(source, "WARTSILA_DENSITY_MAX_WAIT_MS        (5U * 60U * 1000U)", "five minute density timeout")
    require(source, "密度读取超过 5 分钟仍未形成有效液体点时，按密度 0 的空气点处理。", "timeout density zero policy")
    require(source, '0.0f,\n                                     cur_temp,\n                                     cur_mm,\n                                     true,\n                                     "density_timeout_zero"', "timeout output zero as air")
    require(source, "valid_limit_mm = level_mm - (float)min_gap_surface", "valid point boundary")
    require(source, "point_pos_mm >= valid_limit_mm", "strict valid point comparison")

    forbid(source, "motorMoveUpToPositionOrAir(target_mm", "point-to-point in-motion level detection")
    forbid(source, "determine_level_status(&st0)", "start point level-mode precheck")
    forbid(source, "cur_mm-100.0f", "estimated liquid level")
    forbid(source, "have_last_nonzero", "using last nonzero density after five minute timeout")

    require(measure_source, "if (ret != NO_ERROR)", "Wartsila failure branch")
    require(measure_source, "瓦锡兰分布测量失败", "Wartsila failure log")
    forbid(measure_source, "SET_ERROR(ret);\n\tg_measurement.density_distribution = temp;", "writeback on failed Wartsila result")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

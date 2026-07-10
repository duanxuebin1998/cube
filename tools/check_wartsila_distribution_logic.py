from __future__ import annotations

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "wartsila_density_measurement.c"
MEASURE_SOURCE = ROOT / "LTD_MAIN_CPU2" / "Application" / "Src" / "measure.c"
CPU3_PROTOCOL_SOURCE = (
    ROOT
    / "LTD_DISPLAY_CPU3"
    / "Communication"
    / "external"
    / "wartsila_modbus"
    / "wartsila_modbus_communication.c"
)
CPU3_PROTOCOL_DATA_SOURCE = (
    ROOT
    / "LTD_DISPLAY_CPU3"
    / "Communication"
    / "external"
    / "wartsila_modbus"
    / "wartsila_modbus_data_analysis.c"
)
CPU3_COMM_SOURCE = (
    ROOT
    / "LTD_DISPLAY_CPU3"
    / "Communication"
    / "internal"
    / "main_board_modbus"
    / "cpu2_communicate.c"
)


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
    cpu3_protocol_source = CPU3_PROTOCOL_SOURCE.read_text(encoding="utf-8")
    cpu3_protocol_data_source = CPU3_PROTOCOL_DATA_SOURCE.read_text(encoding="utf-8")
    cpu3_comm_source = CPU3_COMM_SOURCE.read_text(encoding="utf-8")

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

    require(cpu3_protocol_source, "wartsila_write_targets_cpu2(start, qty)", "CPU2 target range gate")
    require(
        cpu3_protocol_source,
        "wartsila_read_targets_cpu2_parameters(start, qty) && !CPU2_CommIsAvailable()",
        "unconfirmed CPU2 parameter read gate",
    )
    require(
        cpu3_protocol_source,
        "build_exception(addr, 0x03, 0x06, tx, tx_len)",
        "busy response for unconfirmed Wartsila parameter reads",
    )
    require(cpu3_protocol_source, "build_exception(addr, 0x10, 0x06, tx, tx_len)", "busy response before local write")
    require(cpu3_protocol_source, "build_exception(addr, func, 0x06, tx, tx_len)", "busy response after CPU2 failure")
    require(
        cpu3_protocol_source,
        "if (parameter_write && !ForwardParamsToLowerDevice())",
        "parameter sync before optional command",
    )
    if cpu3_protocol_source.index("if (parameter_write && !ForwardParamsToLowerDevice())") > cpu3_protocol_source.index("if (command_write"):
        raise AssertionError("Wartsila combined write must sync parameters before command")
    require(cpu3_protocol_source, "static void Wartsila_ClearCommandShadow(void)", "single-use command clear helper")
    require(cpu3_protocol_source, "bool command_sent = true;", "single-use command result")
    require(cpu3_protocol_source, "if (g_deviceParams.command != CMD_NONE)", "zero command does not reach CPU2")
    require(
        cpu3_protocol_source,
        "if (parameter_write && !ForwardParamsToLowerDevice())",
        "parameter failure branch",
    )
    parameter_failure = cpu3_protocol_source.index("if (parameter_write && !ForwardParamsToLowerDevice())")
    parameter_failure_end = cpu3_protocol_source.index("return false;", parameter_failure)
    parameter_failure_slice = cpu3_protocol_source[parameter_failure:parameter_failure_end]
    if "Wartsila_ClearCommandShadow();" not in parameter_failure_slice:
        raise AssertionError("Wartsila combined-write parameter failure must clear the pending command")
    for field in (
        "wartsila_upper_density_limit",
        "wartsila_lower_density_limit",
        "wartsila_density_interval",
        "wartsila_max_height_above_surface",
    ):
        if f"g_deviceParams.{field} = confirmed_" not in parameter_failure_slice:
            raise AssertionError(f"Wartsila parameter failure must restore the last confirmed {field}")
    command_clear = cpu3_protocol_source.index("g_deviceParams.command = CMD_NONE;")
    command_failure = cpu3_protocol_source.index("if (!command_sent)")
    if command_clear > command_failure:
        raise AssertionError("Wartsila command shadow must clear before returning CPU2 failure")
    require(cpu3_protocol_data_source, "dsm->command = CMD_NONE;", "zero command clears stale shadow")
    if cpu3_protocol_data_source.index("dsm->command = CMD_NONE;") > cpu3_protocol_data_source.index("if (wxl->down_command != 0)"):
        raise AssertionError("Wartsila register load must clear stale command before decoding")
    require(cpu3_comm_source, "void CPU2_CommRequestParameterRefresh(void)", "forced parameter refresh API")
    require(cpu3_comm_source, "s_cpu2_has_parameter_snapshot = false;", "parameter snapshot invalidation")
    require(cpu3_comm_source, "s_cpu2_parameter_refresh_requested = true;", "forced refresh request")
    require(
        cpu3_comm_source,
        "static bool CPU2_CommFinishFailedRequest(bool parameter_write_attempted)",
        "shared failed-request finalizer",
    )
    require(
        cpu3_comm_source,
        "!((startadd == HOLDREGISTER_DEVICEPARAM_COMMAND) && (registercnt == 2U))",
        "command write exclusion from parameter invalidation",
    )
    if cpu3_comm_source.count("return CPU2_CommFinishFailedRequest(parameter_write_attempted);") != 4:
        raise AssertionError("all four sent-request failure exits must use the shared parameter uncertainty finalizer")
    failed_request = cpu3_comm_source.index("static bool CPU2_CommFinishFailedRequest")
    failed_request_end = cpu3_comm_source.index("return false;", failed_request)
    failed_request_slice = cpu3_comm_source[failed_request:failed_request_end]
    if failed_request_slice.index("CPU2_CommRecordFailure();") > failed_request_slice.index("CPU2_CommRequestParameterRefresh();"):
        raise AssertionError("failed request must be counted before parameter refresh invalidation is scheduled")
    require(
        cpu3_comm_source,
        "if (s_cpu2_parameter_refresh_requested) {",
        "polling consumes forced refresh request",
    )
    forced_refresh = cpu3_comm_source.index("if (s_cpu2_parameter_refresh_requested) {")
    hold_refresh = cpu3_comm_source.index("if (hold_refresh_pending) {", forced_refresh)
    if "hold_refresh_pending = true;" not in cpu3_comm_source[forced_refresh:hold_refresh]:
        raise AssertionError("forced parameter refresh must enter the full holding-register refresh state")
    refresh_complete = cpu3_comm_source.index("if (hold_refresh_index >= REFRESH_HOLD_GROUP_COUNT)")
    refresh_complete_end = cpu3_comm_source.index("return;", refresh_complete)
    refresh_slice = cpu3_comm_source[refresh_complete:refresh_complete_end]
    if refresh_slice.index("DeviceParams_StoreToRegisters(g_holding_regs);") > refresh_slice.index("s_cpu2_has_parameter_snapshot = true;"):
        raise AssertionError("Wartsila mirror must refresh before the CPU2 parameter snapshot is reopened")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

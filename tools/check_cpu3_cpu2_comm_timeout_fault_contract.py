#!/usr/bin/env python3
"""检查 CPU3 连续请求未获得合法 CPU2 响应后的本机故障契约。"""

from __future__ import annotations

import re
import sys
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CPU3_PARAM_H = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.h"
CPU3_PARAM_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "system_param" / "system_parameter.c"
CPU3_COMM_C = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "internal" / "main_board_modbus" / "cpu2_communicate.c"
CPU3_COMM_H = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "internal" / "main_board_modbus" / "cpu2_communicate.h"
CPU3_DEVICE_SYNC_C = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "internal" / "main_board_modbus" / "device_param_sync.c"
CPU3_DEVICE_SYNC_H = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "internal" / "main_board_modbus" / "device_param_sync.h"
CPU3_DSM_DATA_C = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "external" / "DSM_modbus" / "DSM_DataAnalysis_modbus2.c"
CPU3_DSM_SLAVE_C = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "external" / "DSM_modbus" / "DSM_SlaveModbus_modbus2.c"
CPU3_SI_C = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "external" / "si_modbus" / "si_modbus_slave.c"
CPU3_WARTSILA_C = ROOT / "LTD_DISPLAY_CPU3" / "Communication" / "external" / "wartsila_modbus" / "wartsila_modbus_communication.c"
CPU3_DISPLAY_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display.c"
CPU3_MENU_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display_tankopera.c"
CPU3_APP_MAIN_C = ROOT / "LTD_DISPLAY_CPU3" / "Application" / "app_main.c"
FAULT_CODE_DOC = ROOT / "docs" / "00_构建与版本" / "故障码" / "当前程序故障代码清单.md"
FAULT_CODE_WORKBOOK = ROOT / "docs" / "00_构建与版本" / "故障码" / "LTD故障代码统一表.xlsx"
REMEDIATION_DOC = ROOT / "docs" / "03_问题分析与整改" / "已闭环" / "2026-07-09_CPU3与CPU2通信超时故障状态整改.md"
STATUS_DISPLAY_DOC = ROOT / "docs" / "04_界面与菜单" / "CPU3状态页不同状态显示信息确认表.md"


def read_text(path: Path) -> str:
    """按源码可能的编码读取文件，避免检查脚本误报编码问题。"""
    data = path.read_bytes()
    for encoding in ("utf-8-sig", "gbk"):
        try:
            return data.decode(encoding)
        except UnicodeDecodeError:
            continue
    raise UnicodeDecodeError("unknown", data, 0, 1, f"cannot decode {path}")


def strip_c_comments(text: str) -> str:
    """剥离 C 注释，避免说明性注释导致契约匹配误报。"""
    return re.sub(r"/\*.*?\*/|//[^\r\n]*", "", text, flags=re.DOTALL)


def normalize(text: str) -> str:
    """压缩空白字符，使契约检查不受格式化差异影响。"""
    return re.sub(r"\s+", "", text)


def extract_c_function(text: str, name: str) -> str:
    """提取简单 C 函数定义，供契约检查限定语句所在函数。"""
    source = strip_c_comments(text)
    match = re.search(
        rf"^[ \t]*(?:static[ \t]+)?[A-Za-z_][A-Za-z0-9_ \t*]*[ \t*]+"
        rf"{re.escape(name)}\s*\([^;{{}}]*\)\s*\{{",
        source,
        flags=re.MULTILINE,
    )
    if match is None:
        return ""

    brace_start = source.find("{", match.start())
    depth = 0
    for index in range(brace_start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace_start + 1 : index]
    return ""


def workbook_xml_contains(*needles: str) -> bool:
    """检查 xlsx 内部 XML，避免契约脚本依赖第三方 Excel 库。"""
    with zipfile.ZipFile(FAULT_CODE_WORKBOOK) as workbook:
        xml_text = "".join(
            workbook.read(name).decode("utf-8", errors="ignore")
            for name in workbook.namelist()
            if name.endswith(".xml")
        )
    return all(needle in xml_text for needle in needles)


def main() -> int:
    param_h = read_text(CPU3_PARAM_H)
    param_c = read_text(CPU3_PARAM_C)
    comm_c = read_text(CPU3_COMM_C)
    comm_h = read_text(CPU3_COMM_H)
    device_sync_c = read_text(CPU3_DEVICE_SYNC_C)
    device_sync_h = read_text(CPU3_DEVICE_SYNC_H)
    dsm_data_c = read_text(CPU3_DSM_DATA_C)
    dsm_slave_c = read_text(CPU3_DSM_SLAVE_C)
    si_c = read_text(CPU3_SI_C)
    wartsila_c = read_text(CPU3_WARTSILA_C)
    display_c = read_text(CPU3_DISPLAY_C)
    menu_c = read_text(CPU3_MENU_C)
    app_main_c = read_text(CPU3_APP_MAIN_C)
    fault_code_doc = read_text(FAULT_CODE_DOC)
    remediation_doc = read_text(REMEDIATION_DOC)
    status_display_doc = read_text(STATUS_DISPLAY_DOC)
    compact_param_h = normalize(strip_c_comments(param_h))
    compact_param_c = normalize(strip_c_comments(param_c))
    compact_comm = normalize(strip_c_comments(comm_c))
    compact_device_sync_h = normalize(strip_c_comments(device_sync_h))
    compact_display = normalize(strip_c_comments(display_c))
    compact_app_main = normalize(strip_c_comments(app_main_c))
    frame_valid_body = normalize(extract_c_function(comm_c, "CPU2_ResponseFrameIsValid"))
    status_range_body = normalize(extract_c_function(comm_c, "CPU2_ResponseContainsDeviceStatus"))
    protocol_range_body = normalize(extract_c_function(comm_c, "CPU2_ResponseContainsProtocolVersion"))
    valid_response_body = normalize(extract_c_function(comm_c, "CPU2_CommMarkValidResponse"))
    failure_body = normalize(extract_c_function(comm_c, "CPU2_CommRecordFailure"))
    parameter_refresh_body = normalize(extract_c_function(comm_c, "CPU2_CommRequestParameterRefresh"))
    failed_request_body = normalize(extract_c_function(comm_c, "CPU2_CommFinishFailedRequest"))
    uart_error_body = normalize(extract_c_function(comm_c, "CPU2_CommNotifyUartErrorFromISR"))
    send_body = normalize(extract_c_function(comm_c, "sendToCPU2"))
    host_body = normalize(extract_c_function(comm_c, "HostCommuProcess"))
    combinate_body = normalize(extract_c_function(comm_c, "CPU2_CombinatePackage_Send"))
    send_hold_body = normalize(extract_c_function(device_sync_c, "DeviceParams_SendHoldValueToCPU2"))
    sync_one_body = normalize(extract_c_function(device_sync_c, "DeviceParams_SyncOneHold"))
    sync_all_body = normalize(extract_c_function(device_sync_c, "DeviceParams_SyncAllToCPU2"))
    legacy_update_body = normalize(extract_c_function(dsm_data_c, "UpdateDeviceParamsFromLegacyRegs"))
    dsm_response05_body = normalize(extract_c_function(dsm_slave_c, "Response05"))
    dsm_response16_body = normalize(extract_c_function(dsm_slave_c, "Response16"))
    si_write_coil_body = normalize(extract_c_function(si_c, "si_handle_write_single_coil"))
    si_write_reg_body = normalize(extract_c_function(si_c, "si_handle_write_single_reg"))
    si_send_command_body = normalize(extract_c_function(si_c, "si_send_cpu2_command"))
    wartsila_write_body = normalize(extract_c_function(wartsila_c, "handle_0x10"))
    wartsila_callback_body = normalize(extract_c_function(wartsila_c, "modbus_on_holding_written"))
    wartsila_clear_body = normalize(extract_c_function(wartsila_c, "Wartsila_ClearCommandShadow"))
    polling_body = normalize(extract_c_function(comm_c, "PollingInputData"))
    response04_body = normalize(extract_c_function(comm_c, "CPU2_Response04Process"))
    response03_body = normalize(extract_c_function(comm_c, "CPU2_Response03Process"))
    startup_query_body = normalize(extract_c_function(comm_c, "CPU2_CommShouldShowStartup"))
    available_query_body = normalize(extract_c_function(comm_c, "CPU2_CommIsAvailable"))
    command_query_body = normalize(extract_c_function(comm_c, "CPU2_CommCanSendCommand"))
    bulk_skip_body = normalize(extract_c_function(device_sync_c, "DeviceParams_ShouldSkipBulkSync"))
    refresh_body = normalize(extract_c_function(display_c, "RefreshScreen"))
    menu_read_body = normalize(extract_c_function(menu_c, "get_para_data"))
    cancel_body = normalize(extract_c_function(menu_c, "Display_RequestCancelMeasurement"))
    motor_stop_body = normalize(extract_c_function(menu_c, "motor_run_monitor_request_stop"))
    confirm_cancel_body = normalize(extract_c_function(menu_c, "confirm_cancel_measurement"))
    app_loop_body = normalize(extract_c_function(app_main_c, "App_MainLoop"))
    uart_callback_body = normalize(extract_c_function(app_main_c, "HAL_UART_ErrorCallback"))
    failure_increment = "s_cpu2_consecutive_failure_count++"
    failure_guard = (
        "if(s_cpu2_consecutive_failure_count<CPU2_COMM_FAILURE_LIMIT)"
        "{s_cpu2_consecutive_failure_count++;}"
    )
    failure_increment_pos = failure_body.find(failure_increment)
    failure_threshold_pos = failure_body.find(
        "if(s_cpu2_consecutive_failure_count>=CPU2_COMM_FAILURE_LIMIT)"
    )
    host_address_check_pos = host_body.find("if(rcv[0]!=ADERSS)")
    host_frame_valid_pos = host_body.find("CPU2_ResponseFrameIsValid(rcv,len)")
    host_valid_response_pos = host_body.find("CPU2_CommMarkValidResponse();")
    menu_send_pos = menu_read_body.find("CPU2_CombinatePackage_Send(")
    menu_available_positions = [
        match.start()
        for match in re.finditer(r"!CPU2_CommIsAvailable\(\)", menu_read_body)
    ]
    sync_send_pos = sync_one_body.find("if(!DeviceParams_SendHoldValueToCPU2(h,dev_val))")
    sync_cache_commit_pos = sync_one_body.find("h->val=dev_val;")

    checks = [
        (
            "dedicated CPU2 communication timeout error code",
            "CPU2_COMM_TIMEOUT = 0x0013000A" in param_h,
        ),
        (
            "failure threshold is ten and private counter starts from zero",
            "#defineCPU2_COMM_FAILURE_LIMIT10U" in compact_comm
            and "staticuint32_ts_cpu2_consecutive_failure_count=0U;" in compact_comm
            and "staticbools_cpu2_comm_fault_active=false;" in compact_comm,
        ),
        (
            "legacy shared timeout sentinel is removed",
            "cnt_commutoCPU2" not in compact_param_c
            and "cnt_commutoCPU2" not in compact_param_h
            and "COMMU_ERROR_MAX" not in compact_param_h,
        ),
        (
            "first CPU2 status, parameter and protocol snapshots start false",
            "staticbools_cpu2_has_status_snapshot=false;" in compact_comm
            and "staticbools_cpu2_has_parameter_snapshot=false;" in compact_comm
            and "staticbools_cpu2_has_protocol_snapshot=false;" in compact_comm,
        ),
        (
            "communication state queries are public",
            "CPU2_CommShouldShowStartup" in comm_h
            and "CPU2_CommIsAvailable" in comm_h
            and "CPU2_CommCanSendCommand" in comm_h
            and "CPU2_CommNotifyUartErrorFromISR" in comm_h,
        ),
        (
            "response frame shape is validated before communication is marked valid",
            "rcv[1]!=(uint8_t)RCV_functioncode" in frame_valid_body
            and "expected_byte_count=(uint32_t)RCV_registercnt*2U;" in frame_valid_body
            and "len==(int)(expected_byte_count+5U)" in frame_valid_body
            and "caseFUNCTIONCODE_WRITE_MULREGISTER:" in frame_valid_body
            and "len==8" in frame_valid_body
            and host_frame_valid_pos > host_address_check_pos
            and host_valid_response_pos > host_frame_valid_pos,
        ),
        (
            "legal response only resets request failure count",
            "s_cpu2_consecutive_failure_count=0U;" in valid_response_body
            and "s_cpu2_has_status_snapshot" not in valid_response_body
            and "s_cpu2_comm_fault_active" not in valid_response_body
            and host_address_check_pos >= 0
            and host_valid_response_pos > host_address_check_pos,
        ),
        (
            "failure count increments once with saturation before threshold check",
            failure_guard in failure_body
            and failure_body.count(failure_increment) == 1
            and failure_increment_pos >= 0
            and failure_threshold_pos >= 0
            and failure_increment_pos < failure_threshold_pos,
        ),
        (
            "failure threshold latches STATE_ERROR and CPU2 timeout code",
            "s_cpu2_comm_fault_active=true;" in failure_body
            and "s_cpu2_has_parameter_snapshot=false;" in failure_body
            and "s_cpu2_has_protocol_snapshot=false;" in failure_body
            and "if(s_cpu2_comm_fault_active)" in failure_body
            and "g_measurement.device_status.device_state=STATE_ERROR;" in failure_body
            and "g_measurement.device_status.error_code=CPU2_COMM_TIMEOUT;" in failure_body,
        ),
        (
            "UART ISR only records a pending error",
            "if(wait_response)" in uart_error_body
            and "s_cpu2_uart_error_pending=true;" in uart_error_body
            and "wait_response=false;" in uart_error_body
            and "CPU2_CommRecordFailure" not in uart_error_body
            and "g_measurement" not in uart_error_body
            and "CPU2_CommNotifyUartErrorFromISR();" in uart_callback_body,
        ),
        (
            "all request failure exits are counted in main context",
            bool(send_body)
            and "CPU2_CommRecordFailure();" in failed_request_body
            and combinate_body.count("CPU2_CommFinishFailedRequest(parameter_write_attempted)") == 4
            and "if(!sendToCPU2(arr,len,false))" in combinate_body
            and "if(s_cpu2_uart_error_pending)" in combinate_body
            and "if(!HostCommuProcess(UART5_RX_BUF,UART5_RX_LEN))" in combinate_body
            and "returntrue;" in combinate_body,
        ),
        (
            "uncertain non-command writes immediately invalidate the parameter snapshot and force a full refresh",
            "s_cpu2_has_parameter_snapshot=false;" in parameter_refresh_body
            and "s_cpu2_parameter_refresh_requested=true;" in parameter_refresh_body
            and "if(parameter_write_attempted){CPU2_CommRequestParameterRefresh();}" in failed_request_body
            and "!((startadd==HOLDREGISTER_DEVICEPARAM_COMMAND)&&(registercnt==2U))" in combinate_body
            and "if(s_cpu2_parameter_refresh_requested)" in polling_body
            and "hold_refresh_pending=true;" in polling_body
            and "hold_refresh_index=0;" in polling_body,
        ),
        (
            "only a complete status response establishes the status snapshot and releases the latched fault",
            "RCV_functioncode==FUNCTIONCODE_READ_INPUTREGISTER" in status_range_body
            and "REG_DEVICE_STATUS_DEVICE_STATE" in status_range_body
            and "REG_DEVICE_STATUS_ERROR_CODE" in status_range_body
            and "if(response_contains_status){s_cpu2_has_status_snapshot=true;}" in response04_body
            and "if(s_cpu2_comm_fault_active)" in response04_body
            and "if(response_contains_status){s_cpu2_comm_fault_active=false;}" in response04_body
            and "g_measurement.device_status.error_code=CPU2_COMM_TIMEOUT;" in response04_body,
        ),
        (
            "only a 0x03 response covering protocolVersion validates the current CPU2 protocol snapshot",
            "RCV_functioncode==FUNCTIONCODE_READ_HOLDREGISTER" in protocol_range_body
            and "HOLDREGISTER_DEVICEPARAM_PROTOCOL_VERSION" in protocol_range_body
            and "if(response_contains_protocol){s_cpu2_has_protocol_snapshot=true;}" in response03_body
            and "s_cpu2_has_protocol_snapshot" not in response04_body,
        ),
        (
            "poll groups advance only after a legal response and rebuild the parameter snapshot after faults",
            "if(s_cpu2_comm_fault_active)" in polling_body
            and "status_group=&runtime_groups[0]" in polling_body
            and "poweron_done=false;" in polling_body
            and "poweron_index=0;" in polling_body
            and polling_body.count("if(!CPU2_CombinatePackage_Send(") >= 3
            and polling_body.find("if(!CPU2_CombinatePackage_Send(")
            < polling_body.find("poweron_index++;")
            and polling_body.count("s_cpu2_has_parameter_snapshot=true;") >= 2
            and "s_cpu2_has_parameter_snapshot=false;" in polling_body
            and "s_cpu2_has_protocol_snapshot=false;" not in polling_body
            and polling_body.count("DeviceParams_StoreToRegisters(g_holding_regs);") >= 2,
        ),
        (
            "CPU2 polling checks a 100 ms scheduling threshold under external traffic",
            "HAL_GetTick()-last_cpu2_poll_tick)>=CPU2_POLL_PERIOD_MS" in app_loop_body
            and "PollingInputData();" in app_loop_body
            and "if(!did_work){PollingInputData();" not in app_loop_body,
        ),
        (
            "startup page waits for current status and protocol snapshots unless a fault is latched",
            "!s_cpu2_has_status_snapshot" in startup_query_body
            and "!s_cpu2_has_protocol_snapshot" in startup_query_body
            and "!s_cpu2_comm_fault_active" in startup_query_body
            and '"cpu2_communicate.h"' in display_c
            and "CPU2_CommShouldShowStartup()" in refresh_body
            and "s_cpu2_consecutive_failure_count" not in refresh_body,
        ),
        (
            "CPU2 write availability requires complete snapshots, no fault and matching protocol",
            "s_cpu2_has_status_snapshot" in available_query_body
            and "s_cpu2_has_parameter_snapshot" in available_query_body
            and "s_cpu2_has_protocol_snapshot" in available_query_body
            and "!s_cpu2_comm_fault_active" in available_query_body
            and "g_deviceParams.protocolVersion==DEVICE_PROTOCOL_VERSION" in available_query_body
            and len(menu_available_positions) == 2
            and menu_available_positions[0] < menu_send_pos < menu_available_positions[1],
        ),
        (
            "device parameter cache commits only after CPU2 accepts the write",
            "DeviceParams_MetaValueToRaw(h,target_value)" in send_hold_body
            and "returnCPU2_CombinatePackage_Send(" in send_hold_body
            and sync_send_pos >= 0
            and sync_cache_commit_pos > sync_send_pos
            and "returnfalse;" in sync_one_body
            and "returntrue;" in sync_one_body
            and "operanum==COM_NUM_DEVICEPARAM_COMMAND" in bulk_skip_body,
        ),
        (
            "bulk parameter sync reports CPU2 write failure to DSM",
            "boolDeviceParams_SyncAllToCPU2(void);" in compact_device_sync_h
            and "if(!DeviceParams_SyncOneHold(&param_meta[i])){returnfalse;}" in sync_all_body
            and "returntrue;" in sync_all_body
            and "if(!DeviceParams_SyncAllToCPU2()){returnPARAMETER_WRITE_FAIL;}" in legacy_update_body,
        ),
        (
            "ordinary writes require full availability while cancel remains reachable during parameter refresh",
            "cmd!=CMD_CANCEL_MEASUREMENT" in command_query_body
            and "returnCPU2_CommIsAvailable();" in command_query_body
            and "s_cpu2_has_status_snapshot" in command_query_body
            and "s_cpu2_has_protocol_snapshot" in command_query_body
            and "s_cpu2_has_parameter_snapshot" not in command_query_body
            and "g_deviceParams.protocolVersion==DEVICE_PROTOCOL_VERSION" in command_query_body
            and "startadd==HOLDREGISTER_DEVICEPARAM_COMMAND" in combinate_body
            and "registercnt==2U" in combinate_body
            and "CPU2_CommCanSendCommand((CommandType)(*holddata))" in combinate_body
            and "if(!cancel_command_allowed){returnfalse;}" in combinate_body,
        ),
        (
            "DSM commands and parameters return busy instead of false success",
            "!CPU2_CommIsAvailable()" in legacy_update_body
            and "returnPARAMETER_WRITE_FAIL;" in legacy_update_body
            and "!CPU2_CommCanSendCommand((CommandType)cmd)||!CPU2_CombinatePackage_Send(" in dsm_response05_body
            and "EXCEPTIONCODE_ERRORDEVIVEBUSY" in dsm_response05_body
            and "!IsHoldingRegisterZeroSegment(" in dsm_response16_body
            and "!CPU2_CommIsAvailable()" in dsm_response16_body
            and dsm_response16_body.find("!CPU2_CommIsAvailable()")
            < dsm_response16_body.find("WriteHoldingRegister("),
        ),
        (
            "SI writes return device busy when CPU2 does not accept the request",
            "if(!si_apply_coil_write(offset,is_on))" in si_write_coil_body
            and "SI_EX_SLAVE_DEVICE_BUSY" in si_write_coil_body
            and "if(!CPU2_CommCanSendCommand(cmd)){returnfalse;}" in si_send_command_body
            and "if(!si_apply_holding_write(offset,value))" in si_write_reg_body
            and "SI_EX_SLAVE_DEVICE_BUSY" in si_write_reg_body,
        ),
        (
            "Wartsila writes gate CPU2 targets and limit parameter forwarding",
            "wartsila_write_targets_cpu2(start,qty)" in wartsila_write_body
            and "!CPU2_CommIsAvailable()" in wartsila_write_body
            and "0x06" in wartsila_write_body
            and "if(!command_write&&!parameter_write){returntrue;}" in wartsila_callback_body
            and "if(parameter_write&&!ForwardParamsToLowerDevice())" in wartsila_callback_body
            and "if(command_write)" in wartsila_callback_body
            and "if(g_deviceParams.command!=CMD_NONE)" in wartsila_callback_body
            and wartsila_callback_body.count("Wartsila_ClearCommandShadow();") >= 2
            and "g_deviceParams.command=CMD_NONE;" in wartsila_clear_body
            and wartsila_callback_body.find("ForwardParamsToLowerDevice()")
            < wartsila_callback_body.find("if(command_write"),
        ),
        (
            "cancel and motor-stop UI consume the actual CPU2 request result",
            "boolDisplay_RequestCancelMeasurement(void);" in normalize(strip_c_comments(comm_h + read_text(ROOT / "LTD_DISPLAY_CPU3" / "Application" / "display" / "display_tankopera.h")))
            and "if(!display_state_can_cancel_measurement(state)){returntrue;}" in cancel_body
            and "if(!CPU2_CommCanSendCommand(CMD_CANCEL_MEASUREMENT)){returnfalse;}" in cancel_body
            and "returnsend_cpu2_command(CMD_CANCEL_MEASUREMENT);" in cancel_body
            and "if(!Display_RequestCancelMeasurement())" in motor_stop_body
            and "enter_motor_run_monitor_page_waiting_stop();" in motor_stop_body
            and motor_stop_body.find("if(!Display_RequestCancelMeasurement())")
            < motor_stop_body.find("enter_motor_run_monitor_page_waiting_stop();")
            and "if(!Display_RequestCancelMeasurement())" in confirm_cancel_body
            and confirm_cancel_body.find("if(!Display_RequestCancelMeasurement())")
            < confirm_cancel_body.find("exitTankOpera();"),
        ),
        (
            "display maps CPU2 timeout reason",
            "caseCPU2_COMM_TIMEOUT:" in compact_display
            and '"CPU2通信超时"' in display_c,
        ),
        (
            "fault code Markdown records CPU3-local timeout code",
            "CPU3 本机扩展故障码" in fault_code_doc
            and "CPU2_COMM_TIMEOUT" in fault_code_doc
            and "0x0013000A" in fault_code_doc,
        ),
        (
            "fault code workbook records CPU3-local timeout code",
            workbook_xml_contains("CPU3本机故障码", "CPU2_COMM_TIMEOUT", "0x0013000A"),
        ),
        (
            "remediation document records cold-start tenth-failure boundary",
            "s_cpu2_has_status_snapshot" in remediation_doc
            and "s_cpu2_consecutive_failure_count" in remediation_doc
            and "CPU2_COMM_FAILURE_LIMIT = 10" in remediation_doc
            and "非法响应" in remediation_doc
            and "UART 错误" in remediation_doc
            and "连续第 10 次" in remediation_doc,
        ),
        (
            "status display document records independent startup state",
            "CPU2_CommShouldShowStartup()" in status_display_doc
            and "CPU2_CommIsAvailable()" in status_display_doc
            and "第 1～9 次" in status_display_doc
            and "第 10 次" in status_display_doc,
        ),
    ]

    failures = [name for name, ok in checks if not ok]
    if failures:
        print("CPU3 CPU2 communication timeout fault contract check failed:")
        for name in failures:
            print(f"- missing {name}")
        return 1

    print("CPU3 CPU2 communication timeout fault contract check passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

/*
 * 模块职责：关联DM4 V4失败结果与原始通信包，并在事务超时后检查无线滑环链路。
 * 调用边界：只在线程态打印既有诊断快照或执行超时归因，不参与协议探测和正常收发。
 * 结果约束：包证据不改变错误码；蓝牙链路正常也不代表具体传感器事务已经成功。
 */
#include "sensor_comm_diagnostics.h"

#include "error_log.h"
#include "Protocols/Dm4/V4/multiparam_v4_communication.h"
#include "sensor_runtime.h"
#include "system_parameter.h"
#include "Wireless/wireless_pairing.h"

#include <stdio.h>

/*
 * 函数用途：把当前DM4 V4失败结果与协议层保存的原始通信包一起打印。
 * 调用场景：液位频率读取返回错误或业务层拒绝频率值之后。
 * 关键约束：只在线程态调用；协议重试已打印的事务自动去重，不改变通信方式或错误码。
 */
void SensorComm_PrintFailurePackets(uint32_t result, const char *context)
{
    if ((result == NO_ERROR) || (result == STATE_SWITCH) ||
        (SensorRuntime_IsDetectionValid() == 0U) ||
        (g_deviceParams.sensorType != DM4_SENSOR) ||
        (SensorRuntime_GetDm4ProtocolMode() != SENSOR_DM4_PROTOCOL_V4)) {
        return;
    }

    MULTIPARAM_V4_PrintFailurePackets(
        (context != NULL) ? context : "未知操作",
        result);
}

/*
 * 函数用途：在传感器无响应后统一诊断无线滑环的蓝牙链路并保留原操作上下文。
 * 调用场景：SensorService完成协议调用且结果为通信超时时。
 * 关键约束：只在线程态调用，允许打印和执行AT诊断；非超时错误原样返回，命令切换不记故障。
 */
uint32_t SensorComm_DiagnoseTimeout(uint32_t ret, const char *context)
{
    uint32_t link_ret;
    uint32_t diag_ret;
    const char *op_context = (context != NULL) ? context : "未知操作";
    char detail[96];

    if (ret != SENSOR_DEVICE_COMM_TIMEOUT) {
        return ret;
    }

    snprintf(detail, sizeof(detail), "原操作：%s", op_context);
    ErrorLog_WarnDetail(ERROR_LOG_MODULE_SENSOR,
                        ERROR_LOG_OP_COMM_DIAG,
                        ErrorLog_GetReasonByCode(ret),
                        ERROR_LOG_ACTION_CONTINUE,
                        detail);

    link_ret = WirelessPairing_CheckBluetoothLink();
    if (link_ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }

    diag_ret = link_ret;
    if (diag_ret != NO_ERROR) {
        snprintf(detail, sizeof(detail), "原操作：%s,链路：蓝牙", op_context);
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_SLIPRING_COMM,
                            "蓝牙链路诊断",
                            ErrorLog_GetReasonByCode(diag_ret),
                            ERROR_LOG_ACTION_CONTINUE,
                            detail);
        return diag_ret;
    }

    return SENSOR_DEVICE_COMM_TIMEOUT;
}

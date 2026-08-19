/*
 * 模块职责：保存“本次上电是否识别成功”的运行态结果，并根据持久化身份解析当前驱动。
 * 数据边界：g_deviceParams保存跨重启身份；s_sensor_detection_result只描述本次上电探测结果。
 * DM4约束：SAFE_SENSOR和旧MULTIPARAM_V4_SENSOR都会迁移为DM4_SENSOR(14)，实际使用
 * V4还是Safe协议由当前Safe会话状态决定，不再通过两个持久化类型区分同一只传感器。
 */
#include "sensor_runtime.h"

#include "sensor_driver_registry.h"
#include "Protocols/Dm4/Safe/sensor_safe_legacy_adapter.h"
#include "system_parameter.h"

/* 本次上电探测的最终结果；与FRAM保存的历史sensorType相互独立。 */
static uint32_t s_sensor_detection_result = SENSOR_DEVICE_COMM_TIMEOUT;

/*
 * 函数用途：开始新一轮传感器识别并撤销本次启动的测量授权。
 * 调用场景：上电自动识别或维护流程重新探测之前。
 * 关键约束：只重置运行态结果，不覆盖FRAM中上次持久化的类型和编号。
 */
void SensorRuntime_BeginDetection(void)
{
    /* 探测开始即撤销本次启动的通信授权，避免沿用FRAM中的旧身份直接测量。 */
    s_sensor_detection_result = SENSOR_DEVICE_COMM_TIMEOUT;
}

/*
 * 函数用途：提交已验证的物理传感器类型和编号，并按需持久化。
 * 调用场景：V4、V3或DSM探测完整成功之后。
 * 关键约束：编号0是合法值；只有身份变化才写FRAM，提交后本次识别立即有效。
 */
void SensorRuntime_CommitIdentity(uint32_t sensor_type, uint32_t sensor_id)
{
    uint8_t identity_changed =
        ((g_deviceParams.sensorType != sensor_type) ||
         (g_deviceParams.sensorID != sensor_id)) ? 1U : 0U;

    g_deviceParams.sensorType = sensor_type;
    g_deviceParams.sensorID = sensor_id;
    /* 先发布内存身份和识别成功状态，再按需持久化；编号0是合法的未初始化编号。 */
    s_sensor_detection_result = NO_ERROR;
    if (identity_changed != 0U) {
        save_device_params();
    }
}

/*
 * 函数用途：记录本次识别未成功时的最终结果。
 * 调用场景：候选协议全部失败或收到命令切换时。
 * 关键约束：不改变持久化身份，错误码保持识别层原始语义。
 */
void SensorRuntime_SetDetectionResult(uint32_t result)
{
    s_sensor_detection_result = result;
}

/*
 * 函数用途：判断本次启动是否已有成功识别结果。
 * 调用场景：SensorService和测量入口执行快速门禁。
 * 关键约束：只读内存，不把历史FRAM身份视为本次识别成功。
 */
uint8_t SensorRuntime_IsDetectionValid(void)
{
    return (s_sensor_detection_result == NO_ERROR) ? 1U : 0U;
}

/*
 * 函数用途：读取本次传感器识别结果码。
 * 调用场景：上层需要保留具体失败原因时。
 * 关键约束：只读内存，不清除也不转换错误码。
 */
uint32_t SensorRuntime_GetDetectionResult(void)
{
    return s_sensor_detection_result;
}

/*
 * 函数用途：按持久化物理身份和当前Safe会话状态解析驱动操作表。
 * 调用场景：所有SensorService业务操作分派之前。
 * 关键约束：DM4类型固定为14；Safe只是运行会话，不能改变持久化物理类型。
 */
const SensorDriverOps *SensorRuntime_GetDriver(void)
{
    /* 同一个DM4物理身份在普通会话走V4，进入安全会话后才切到预留Safe驱动。 */
    return SensorDriverRegistry_Resolve(g_deviceParams.sensorType,
                                        SensorSafeAdapter_IsActive());
}


/*
 * 函数用途：把当前驱动类别转换为对外DM4协议模式。
 * 调用场景：串口调试和后续Safe会话状态查询。
 * 关键约束：DSM、V3和未识别驱动统一返回NOT_APPLICABLE。
 */
SensorDm4ProtocolMode SensorRuntime_GetDm4ProtocolMode(void)
{
    if (g_deviceParams.sensorType != (uint32_t)DM4_SENSOR) {
        return SENSOR_DM4_PROTOCOL_NOT_APPLICABLE;
    }

    return (SensorSafeAdapter_IsActive() != 0U)
               ? SENSOR_DM4_PROTOCOL_SAFE_RESERVED
               : SENSOR_DM4_PROTOCOL_V4;
}

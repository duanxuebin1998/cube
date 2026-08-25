/**
 * @file part_diagnostics.c
 * @brief 部件参数读取、传感器诊断和周期刷新实现。
 *
 * part_diagnostics.c
 * 读取电机、编码器、称重、传感器和无线链路维护参数。
 * 固定顺序：电机与位置 -> 重量 -> 姿态 -> 密度组合量 -> 扩展量 -> 零点霍尔 -> 水位 -> RSSI。
 * 状态边界：调度层负责MeasureStart，命令入口只负责READPARAMETERING状态；故障恢复复用读取流程，
 * 但不得重新启动命令状态机。任一核心步骤失败时立即返回原错误码。
 */
#include "part_diagnostics.h"
#include "fault_manager.h"
#include "system_parameter.h"

#include "AS5145.h"
#include "abortable_delay.h"

#include "motor_ctrl.h"
#include "sensor_service.h"
#include "Protocols/Dm4/V4/multiparam_v4_communication.h"
#include "Wireless/wireless_pairing.h"

#include <stdio.h>

#define SENSOR_DENSITY_MODE_SETTLE_MS 3000U /* V3 传感器切到密度模式后的稳定等待时间，单位 ms。 */
#define READ_PART_PARAMS_REFRESH_INTERVAL_MS 1000U /* 部件参数诊断常规刷新周期，单位 ms。 */
#define READ_PART_PARAMS_RSSI_REFRESH_INTERVAL_MS 5000U /* 无线 RSSI 低频刷新周期，单位 ms。 */
/*
 * 函数用途：部件诊断读取密度组合量前统一切换到密度模式。
 * 调用场景：所有已识别传感器的维护参数首轮读取和周期刷新。
 * 关键约束：V3保留三秒稳定等待，其他驱动只确认模式且不增加固定等待。
 */
static uint32_t PartDiagnostics_PrepareDensityMode(uint8_t wait_for_v3_settle)
{
    uint32_t result = SensorService_EnableDensityMode();

    if (result != NO_ERROR) {
        return result;
    }
    if (wait_for_v3_settle == 0U) {
        return NO_ERROR;
    }
    printf("读取部件参数\tLTD已切换密度模式，等待%lu ms稳定\r\n",
           (unsigned long)SENSOR_DENSITY_MODE_SETTLE_MS);
    return AbortableDelay_CommandSwitch(SENSOR_DENSITY_MODE_SETTLE_MS, 50U);
}

/*
 * 函数用途：按周期通过CH9141 AT事务刷新部件诊断使用的RSSI快照。
 * 调用场景：部件参数循环末尾，首次可强制刷新，后续按五秒间隔刷新。
 * 关键约束：V4主动接收期间跳过AT抢占；命令切换必须原样返回，AT退出后UART6需恢复透传。
 */
static uint32_t PartDiagnostics_UpdateWirelessRssi(uint8_t force_update)
{
    static uint32_t last_update_tick = 0U;
    uint32_t now_tick = HAL_GetTick();

    if ((SensorService_GetDm4ProtocolMode() == SENSOR_DM4_PROTOCOL_V4) &&
        (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_ACTIVE)) {
        /* 主动上报期间不进入CH9141 AT模式，避免与72字节常驻接收争用UART6。 */
        return NO_ERROR;
    }
    if ((force_update == 0U) &&
        ((now_tick - last_update_tick) < READ_PART_PARAMS_RSSI_REFRESH_INTERVAL_MS)) {
        return NO_ERROR;
    }

    if (HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    last_update_tick = now_tick;
    return WirelessPairing_UpdateConnectionStatusSnapshot();
}
/*
 * 函数用途：在V4交互部件参数读取失败时打印最后一组原始收发包。
 * 调用场景：姿态角、密度组合量或水位电容读取返回错误后、错误码上抛前。
 * 关键约束：主动上报路径没有对应交互事务，禁止打印可能属于旧事务的缓存包。
 */
static uint32_t PartDiagnostics_ReportV4Failure(const char *operation,
                                                 uint32_t result)
{
    if ((result != NO_ERROR) &&
        (SensorService_GetDm4ProtocolMode() == SENSOR_DM4_PROTOCOL_V4) &&
        (MULTIPARAM_V4_GetCommunicationMode() == MULTIPARAM_V4_COMMUNICATION_INTERACTIVE)) {
        MULTIPARAM_V4_PrintLastTransactionPackets(operation, result);
    }
    return result;
}

/* ================== CMD：读取部件参数 ================== */
/*
 * 函数用途：按固定顺序读取电机、位置、称重、传感器和无线链路维护参数。
 * 调用场景：读取部件参数命令首轮及故障恢复中的无状态复核。
 * 关键约束：命令入口不重复调用MeasureStart；核心项首错即返回，扩展项按能力读取且不覆盖核心成功结果。
 */
static uint32_t PartDiagnostics_ReadAllInternal(uint8_t update_command_state)
{
    uint32_t ret = NO_ERROR;
    uint32_t voltage_ret = NO_ERROR;
    uint32_t density_analysis_ret = NO_ERROR;
    uint8_t is_ltd_sensor = SensorService_IsMultiparamV3();

    float ax = 0.0f, ay = 0.0f;
    float freq = 0.0f, dens = 0.0f, temp = 0.0f;
    float cap = 0.0f;
    float magnetic_zero_voltage = 0.0f;
    float supply_voltage = 0.0f;
    float period_225 = 0.0f, period_45 = 0.0f;
    float dynamic_viscosity = 0.0f;

    if (update_command_state) {
        /* 仅显式读取命令经过测量启动门禁；恢复检查沿用当前命令状态，避免重复状态迁移。 */
        g_measurement.device_status.device_state = STATE_READPARAMETERING;
    } else if (HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    /* ---------- 1) 位置类：编码器/位置/尺带长度/步进/距离 ---------- */
    ret = MotorCtrl_CheckDriverGstat();
    if (ret != NO_ERROR) {
        return ret;
    }

    MotorCtrl_RefreshDebugDrumState();
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (!MotorCtrl_IsPositionSourceMotor()) {
        /* 业务停机只消费已经连续确认的锁存错误；单帧瞬态仅保留给ENC?诊断。 */
        ret = AS5145_GetLatchedError();
        if (ret != NO_ERROR) {
            return ret;
        }
    }

    /* MotorCtrl_RefreshDebugDrumState负责刷新电机步数和距离；位置、尺带、速度及状态沿用各业务模块维护的现有快照。 */
    g_measurement.debug_data.current_encoder_value = -(int32_t)g_encoder_count;

    /* ---------- 3) 扭力类 ---------- */
    ret = Weight_CheckOwnCommunicationTimeout();

    if (ret != NO_ERROR) {
        return ret;
    }

    g_measurement.debug_data.current_weight = (uint32_t)weight_parament.current_weight;

    /* ---------- 4) 姿态角（陀螺仪） ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (SensorService_SupportsGyroChannel()) {
        ret = SensorService_ReadGyroAngle(&ax, &ay);
        if ((SensorService_GetDm4ProtocolMode() == SENSOR_DM4_PROTOCOL_V4) &&
            (ret == SENSOR_DATA_STALE)) {
            /* V4姿态角尚未生成或当前无效时，仅跳过诊断项，不覆盖已有快照。 */
            (void)PartDiagnostics_ReportV4Failure("读取姿态角", ret);
            printf("读取部件参数\tV4姿态角当前无有效值，跳过本项且不置错误状态 | 结果=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else if (ret != NO_ERROR) {
            return PartDiagnostics_ReportV4Failure("读取姿态角", ret);
        } else {
            /* 统一按0.01度写入维护快照，避免不同驱动保留各自的内部单位。 */
            g_measurement.debug_data.angle_x = (int32_t)(ax * 100.0f);
            g_measurement.debug_data.angle_y = (int32_t)(ay * 100.0f);
        }
    } else {
        g_measurement.debug_data.angle_x = 0;
        g_measurement.debug_data.angle_y = 0;
        printf("读取部件参数\t当前传感器类型不支持姿态角读取，已跳过\r\n");
    }

    /* ---------- 5) 密度/温度/频率 ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    /* 所有驱动都先建立密度模式，避免V4或DSM继承上一次液位模式后误读密度组合量。 */
    ret = PartDiagnostics_PrepareDensityMode(is_ltd_sensor);
    if (ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }

    if (ret == NO_ERROR) {
        ret = SensorService_ReadDensity(&freq, &dens, &temp);
        if (ret == STATE_SWITCH) {
            return STATE_SWITCH;
        }
    }

    if (ret != NO_ERROR) {
        if (is_ltd_sensor) {
            g_measurement.debug_data.frequency = 0U;
            printf("读取部件参数\tLTD密度/频率暂未读到，按部分成功处理，其他部件参数保留有效。错误码=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else if ((SensorService_GetDm4ProtocolMode() == SENSOR_DM4_PROTOCOL_V4) &&
                   (ret == SENSOR_DATA_STALE)) {
            /* V4测量值尚未生成或当前无效时，仅跳过诊断项，避免维护读取进入全局错误态。 */
            (void)PartDiagnostics_ReportV4Failure("读取密度/温度/频率", ret);
            printf("读取部件参数\tV4密度/温度/频率当前无有效值，跳过本项且不置错误状态 | 结果=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else {
            return PartDiagnostics_ReportV4Failure("读取密度/温度/频率", ret);
        }
    } else {
        /* 服务层已经刷新温度；部件诊断只把浮点频率收敛到现有整数维护字段。 */
        g_measurement.debug_data.frequency = (uint32_t)freq;
    }

    /* ---------- 6) DSM 传感器电压、密度分析参数 ---------- */
    /* 两项扩展能力分别判断，避免未来驱动只实现其中一项时误调用空回调。 */
    voltage_ret = SensorService_ReadSupplyVoltage(&supply_voltage);
    if (voltage_ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    if ((voltage_ret != NO_ERROR) && (voltage_ret != SENSOR_CAPABILITY_UNSUPPORTED)) {
        printf("读取部件参数\tDSM传感器电压未更新，保留核心参数结果。错误码=0x%08lX\r\n",
               (unsigned long)voltage_ret);
    } else if (voltage_ret == NO_ERROR) {
        printf("DSM扩展参数 | 传感器电压=%.4fV\r\n", supply_voltage);
    }

    density_analysis_ret = SensorService_ReadDensityAnalysis(&period_225,
                                                              &period_45,
                                                              &dynamic_viscosity);
    if (density_analysis_ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    if ((density_analysis_ret != NO_ERROR) &&
        (density_analysis_ret != SENSOR_CAPABILITY_UNSUPPORTED)) {
        printf("读取部件参数\tDSM密度分析参数未更新，保留核心参数结果。错误码=0x%08lX\r\n",
               (unsigned long)density_analysis_ret);
    } else if (density_analysis_ret == NO_ERROR) {
        printf("DSM扩展参数 | 22.5度周期平方=%.0f | 45度周期平方=%.0f | 动态黏度=%.1f\r\n",
               period_225,
               period_45,
               dynamic_viscosity);
    }
    /* ---------- 7) V4零点霍尔 ---------- */
    if (SensorService_SupportsMagneticZeroChannel()) {
        ret = SensorService_ReadMagneticZeroVoltage(&magnetic_zero_voltage);
        if (ret == SENSOR_DATA_STALE) {
            (void)PartDiagnostics_ReportV4Failure("读取零点霍尔", ret);
            printf("读取部件参数\tV4零点霍尔当前无有效值，跳过本项且不置错误状态 | 结果=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else if (ret != NO_ERROR) {
            return PartDiagnostics_ReportV4Failure("读取零点霍尔", ret);
        } else {
            printf("V4零点霍尔 | 磁零点电压=%.6f V\r\n",
                   (double)magnetic_zero_voltage);
        }
    }

    /* ---------- 8) 水位电容 ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }

    if (SensorService_SupportsWaterCapChannel()) {
        ret = SensorService_ReadWaterCapacitance(&cap);
        if ((SensorService_GetDm4ProtocolMode() == SENSOR_DM4_PROTOCOL_V4) &&
            (ret == SENSOR_DATA_STALE)) {
            /* V4水位电容尚未生成或当前无效时，仅跳过诊断项，不覆盖已有快照。 */
            (void)PartDiagnostics_ReportV4Failure("读取水位电容", ret);
            printf("读取部件参数\tV4水位电容当前无有效值，跳过本项且不置错误状态 | 结果=0x%08lX\r\n",
                   (unsigned long)ret);
            ret = NO_ERROR;
        } else if (ret != NO_ERROR) {
            return PartDiagnostics_ReportV4Failure("读取水位电容", ret);
        } else {
            /* 水位电容快照按 0.1pF 保存，避免和历史“电压”语义混淆
               若传感器返回单位变化，需要同步调整字段名和显示小数位。 */
            g_measurement.debug_data.water_capacitance_x10 = (uint32_t)(cap * 10.0f); /* 0.1pF */
        }
    } else {
        g_measurement.debug_data.water_capacitance_x10 = 0;
        printf("读取部件参数\t当前传感器类型不支持水位电容读取，已跳过\r\n");
    }

    /* ---------- 9) 蓝牙连接 RSSI ---------- */
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }
    ret = PartDiagnostics_UpdateWirelessRssi(update_command_state);
    if (ret == STATE_SWITCH) {
        return STATE_SWITCH;
    }
    if (ret != NO_ERROR) {
        printf("读取部件参数\t蓝牙状态刷新失败，UART6透明传输未确认，停止后续流程。错误码=0x%08lX\r\n",
               (unsigned long)ret);
        return ret;
    }
    if ((!update_command_state) && HasEffectiveCommandSwitchRequest()) {
        return STATE_SWITCH;
    }


    /* ---------- 10) 打印汇总：正常信息集中一行 ---------- */
    printf("读取部件参数完成 | 编码值=%ld 位置=%ld 缆长=%ld 步数=%ld 距离=%ld(0.1mm) "
           "| freq=%lu temp=%lu | cap=%lu | w=%lu | ang_x=%ld ang_y=%ld(0.01deg) | mspd=%lu mstate=%lu | rssi_valid=%lu rssi=%ld\r\n",
           (long)g_measurement.debug_data.current_encoder_value,
           (long)g_measurement.debug_data.sensor_position,
           (long)g_measurement.debug_data.cable_length,
           (long)g_measurement.debug_data.motor_step,
           (long)g_measurement.debug_data.motor_distance,
           (unsigned long)g_measurement.debug_data.frequency,
           (unsigned long)g_measurement.debug_data.temperature,
           (unsigned long)g_measurement.debug_data.water_capacitance_x10,
           (unsigned long)g_measurement.debug_data.current_weight,
           (long)g_measurement.debug_data.angle_x,
           (long)g_measurement.debug_data.angle_y,
           (unsigned long)g_measurement.debug_data.motor_speed,
           (unsigned long)g_measurement.debug_data.motor_state,
           (unsigned long)g_measurement.wireless_pairing_status.rssi_valid,
           (long)g_measurement.wireless_pairing_status.rssi);

    if (update_command_state) {
        g_measurement.device_status.device_state = STATE_READPARAMETEROVER;
    }

    return NO_ERROR;
}

/*
 * 函数用途：复用部件诊断全流程检查当前全部部件参数。
 * 调用场景：故障恢复复核和读取部件参数命令的周期刷新。
 * 关键约束：不调用MeasureStart、不改变命令状态；首个核心失败项原样返回。
 */
uint32_t PartDiagnostics_CheckAll(void)
{
    return PartDiagnostics_ReadAllInternal(0U);
}

/*
 * 函数用途：执行读取部件参数命令并在完成态持续周期刷新。
 * 调用场景：ProcessMeasureCmd分发CMD_READ_PART_PARAMS时。
 * 关键约束：首次读取进入命令状态；循环等待可被新命令打断，STATE_SWITCH不误报为传感器故障。
 */
void PartDiagnostics_HandleCommand(void)
{
    uint32_t ret = PartDiagnostics_ReadAllInternal(1U);

    SET_ERROR(ret);

    /* 读取部件参数完成态是长驻刷新态：不回到主循环轮询，直接在命令内按周期刷新；
       等待过程保留命令切换检查，避免新命令被 1s 刷新周期阻塞。 */
    while (g_measurement.device_status.device_state == STATE_READPARAMETEROVER) {
        ret = AbortableDelay_CommandSwitch(READ_PART_PARAMS_REFRESH_INTERVAL_MS, 100U);
        if (ret == STATE_SWITCH) {
            /* 命令切换不是传感器故障，记录切换结果后退出本命令，让主循环执行新命令。 */
            SET_ERROR(ret);
            break;
        }

        ret = PartDiagnostics_CheckAll();
        SET_ERROR(ret);
    }
}

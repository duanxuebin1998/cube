/*
 * sensor_service.c
 * 向密度、液位和维护业务提供稳定的传感器能力接口。
 *
 * 调用方向：Application -> SensorService -> SensorDriverOps -> 具体协议驱动。
 * 本层保留公共模式切换中的电机停稳、稳定等待、数值修正和通信超时归因；
 * 协议帧格式、UART6 DMA和主动流状态由下层驱动负责。
 */
#include "sensor_service.h"

#include "abortable_delay.h"
#include "measure.h"
#include "motor_ctrl.h"
#include "sensor_comm_diagnostics.h"
#include "sensor_driver.h"
#include "sensor_runtime.h"
#include "sensor_transport_config.h"
#include "system_parameter.h"

#include <stdio.h>
/*
 * 函数用途：查询本次上电传感器识别是否成功。
 * 调用场景：测量命令和维护命令进入协议操作前的运行态门禁。
 * 关键约束：只读内存状态，不访问UART6、不阻塞、不打印。
 */
uint8_t SensorService_IsDetectionValid(void)
{
    return SensorRuntime_IsDetectionValid();
}

/*
 * 函数用途：取得本次上电传感器识别的完整结果码。
 * 调用场景：上层需要区分超时、格式异常和命令切换时调用。
 * 关键约束：结果不做转换，保持识别层原始错误语义。
 */
uint32_t SensorService_GetDetectionResult(void)
{
    return SensorRuntime_GetDetectionResult();
}

/*
 * 函数用途：查询DM4当前使用V4协议还是预留Safe会话协议。
 * 调用场景：维护诊断和协议状态展示。
 * 关键约束：非DM4返回NOT_APPLICABLE，不触发任何协议切换。
 */
SensorDm4ProtocolMode SensorService_GetDm4ProtocolMode(void)
{
    return SensorRuntime_GetDm4ProtocolMode();
}

/*
 * 函数用途：判断当前已识别驱动是否提供水位电容能力。
 * 调用场景：水位、回零和部件诊断开始读取水位电容之前。
 * 关键约束：本次识别无效时强制返回不支持，禁止使用FRAM旧身份放行业务。
 */
int SensorService_SupportsWaterCapChannel(void)
{
    if (SensorRuntime_IsDetectionValid() == 0U) {
        return 0;
    }
    return (int)SensorDriver_HasCapability(SensorRuntime_GetDriver(), SENSOR_CAP_WATER);
}

/*
 * 函数用途：判断当前已识别驱动是否提供双轴姿态能力。
 * 调用场景：罐高、回零和部件诊断开始读取姿态角之前。
 * 关键约束：只查询能力位，不隐式建立Safe会话或发送传感器命令。
 */
int SensorService_SupportsGyroChannel(void)
{
    if (SensorRuntime_IsDetectionValid() == 0U) {
        return 0;
    }
    return (int)SensorDriver_HasCapability(SensorRuntime_GetDriver(), SENSOR_CAP_GYRO);
}

/*
 * 函数用途：判断当前活动驱动是否为多参数V3传感器。
 * 调用场景：部件诊断需要保留V3读取密度前的专用稳定等待时。
 * 关键约束：只查询本次识别得到的驱动类别，不读取FRAM旧类型、不访问UART6。
 */
uint8_t SensorService_IsMultiparamV3(void)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();

    return (driver->kind == SENSOR_DRIVER_MULTIPARAM_V3) ? 1U : 0U;
}

/*
 * 函数用途：判断当前活动驱动是否提供零点霍尔通道。
 * 调用场景：部件诊断决定是否读取DM4零点霍尔电压。
 * 关键约束：只查询能力位，不触发模式切换、功能使能或UART6通信。
 */
int SensorService_SupportsMagneticZeroChannel(void)
{
    if (SensorRuntime_IsDetectionValid() == 0U) {
        return 0;
    }
    return (int)SensorDriver_HasCapability(SensorRuntime_GetDriver(), SENSOR_CAP_MAGNETIC_ZERO);
}

/*
 * 函数用途：通过当前驱动切换到密度测量模式。
 * 调用场景：密度测量、液位恢复和V3部件参数准备阶段。
 * 关键约束：不负责稳定等待；通信超时统一进入无线链路诊断，其他错误原样传播。
 */
uint32_t SensorService_EnableDensityMode(void)
{
    /* 驱动由本次识别出的物理类型和DM4当前协议会话共同决定。 */
    const SensorDriverOps *driver = SensorRuntime_GetDriver();
    uint32_t result = (driver->enable_density_mode != NULL)
                          ? driver->enable_density_mode()
                          : SENSOR_CAPABILITY_UNSUPPORTED;
    return SensorComm_DiagnoseTimeout(result, "切换密度模式");
}

/*
 * 函数用途：受控停稳电机后通过当前驱动切换到液位测量模式。
 * 调用场景：液位搜索、跟随以及异常频率恢复完成后。
 * 关键约束：成功后执行可被新命令打断的稳定等待；电机停机失败时不得发送模式命令。
 */
uint32_t SensorService_EnableLevelMode(void)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();
    /* 保持旧流程：所有协议切换液位模式前都先让电机受控停稳。 */
    uint32_t result = MotorCtrl_SlowStop();

    if (result != NO_ERROR) {
        return result;
    }
    result = (driver->enable_level_mode != NULL)
                 ? driver->enable_level_mode()
                 : SENSOR_CAPABILITY_UNSUPPORTED;
    result = SensorComm_DiagnoseTimeout(result, "切换液位模式");
    if (result == NO_ERROR) {
        printf("切换液位模式成功，等待%lu ms稳定\r\n",
               (unsigned long)SENSOR_LEVEL_MODE_SETTLE_MS);
        result = AbortableDelay_CommandSwitch(SENSOR_LEVEL_MODE_SETTLE_MS, 100U);
    }
    return result;
}

/*
 * 函数用途：通过当前驱动读取一次液位通道频率。
 * 调用场景：可靠频率读取内部及只需要单帧数据的维护入口。
 * 关键约束：不执行重试、模式恢复或电机动作，空指针和能力不支持直接返回。
 */
uint32_t SensorService_ReadLevelFrequency(uint32_t *frequency_out)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();

    if (frequency_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    /* 单次读取不做运动和模式恢复，液位业务策略由measure_oilLevel模块负责。 */
    return (driver->read_level_frequency != NULL)
               ? driver->read_level_frequency(frequency_out)
               : SENSOR_CAPABILITY_UNSUPPORTED;
}
/*
 * 函数用途：按整机固定参数原地修正密度和温度。
 * 调用场景：任一协议驱动成功返回原始密度组合量之后。
 * 关键约束：密度小于等于200时不修正；修正只在服务层执行一次，驱动层不得重复。
 */
static void Apply_Fixed_DensityTemp_Correction(float *density, float *temp)
{
    if ((density)&&(*density>200.0)) {
        *density = (*density) + ((float)g_deviceParams.densityCorrection - (float)DENSITY_CORRECTION_BASE_RAW) / 100.0f;
    }

    if (temp) {
        *temp = (*temp) + ((float)g_deviceParams.temperatureCorrection-1000.0f)/ 10.0f;
    }
}

/*
 * 函数用途：通过当前驱动读取频率、密度和温度并应用整机固定修正。
 * 调用场景：密度测量、液位辅助计算和部件诊断。
 * 关键约束：三个输出均必须有效；协议成功后才更新调试快照，超时统一诊断无线链路。
 */
uint32_t SensorService_ReadDensity(float *frequency, float *density, float *temp)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();
    uint32_t result;

    if ((frequency == NULL) || (density == NULL) || (temp == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    /* 驱动只返回协议原始业务值；整机固定修正仍在服务层统一且只执行一次。 */
    result = (driver->read_density != NULL)
                 ? driver->read_density(frequency, density, temp)
                 : SENSOR_CAPABILITY_UNSUPPORTED;
    result = SensorComm_DiagnoseTimeout(result, "读取密度");
    if (result == NO_ERROR) {
        float density_before = *density;
        float temp_before = *temp;

        Apply_Fixed_DensityTemp_Correction(density, temp);
        printf("原始密度: %.3f  修正后密度: %.3f\r\n", density_before, *density);
        printf("原始温度: %.3f ℃  修正后温度: %.3f ℃\r\n", temp_before, *temp);
        printf("频率: %.3f Hz\r\n", *frequency);
        g_measurement.debug_data.temperature = TEMP_TO_RAW(*temp);
        g_measurement.debug_data.frequency = *frequency;
    }
    return result;
}

/*
 * 函数用途：通过当前驱动读取水位电容。
 * 调用场景：水位测量、回零和部件诊断。
 * 关键约束：先核对能力；DM4由驱动保证密度模式及测水使能，读取完成后保持功能开启。
 */
uint32_t SensorService_ReadWaterCapacitance(float *cap_out)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();
    uint32_t result;

    if (cap_out == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (SensorDriver_HasCapability(driver, SENSOR_CAP_WATER) == 0U) {
        printf("当前传感器类型不支持读取水位电容\r\n");
        return PARAM_FEATURE_UNSUPPORTED;
    }
    /* DM4驱动会在读取前保证测水已开启；DSM驱动保持原直接读取流程。 */
    result = (driver->read_water_capacitance != NULL)
                 ? driver->read_water_capacitance(cap_out)
                 : PARAM_FEATURE_UNSUPPORTED;
    return SensorComm_DiagnoseTimeout(result, "读取水位电容");
}

/*
 * 函数用途：通过当前驱动读取双轴姿态角。
 * 调用场景：罐高、回零和部件诊断。
 * 关键约束：先核对动态能力；不支持时返回功能不支持，不伪造零值。
 */
uint32_t SensorService_ReadGyroAngle(float *angle_x_deg, float *angle_y_deg)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();
    uint32_t result;

    if ((angle_x_deg == NULL) || (angle_y_deg == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (SensorDriver_HasCapability(driver, SENSOR_CAP_GYRO) == 0U) {
        printf("当前传感器类型不支持读取姿态角\r\n");
        return PARAM_FEATURE_UNSUPPORTED;
    }
    result = (driver->read_gyro != NULL)
                 ? driver->read_gyro(angle_x_deg, angle_y_deg)
                 : PARAM_FEATURE_UNSUPPORTED;
    return SensorComm_DiagnoseTimeout(result, "读取陀螺仪");
}


/*
 * 函数用途：读取当前传感器可选的供电电压诊断量。
 * 调用场景：部件参数维护读取中的DSM扩展诊断阶段。
 * 关键约束：能力或回调缺失时返回不支持且不访问UART6；通信超时沿用统一无线链路诊断。
 */
uint32_t SensorService_ReadSupplyVoltage(float *voltage_v)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();
    uint32_t result;

    if (voltage_v == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if ((SensorDriver_HasCapability(driver, SENSOR_CAP_SUPPLY_VOLTAGE) == 0U) ||
        (driver->read_supply_voltage == NULL)) {
        return SENSOR_CAPABILITY_UNSUPPORTED;
    }
    result = driver->read_supply_voltage(voltage_v);
    return SensorComm_DiagnoseTimeout(result, "读取DSM传感器电压");
}

/*
 * 函数用途：读取当前传感器可选的密度分析扩展量。
 * 调用场景：部件参数维护读取中的DSM扩展诊断阶段。
 * 关键约束：三项输出必须同时有效；能力或回调缺失时不访问UART6，超时沿用统一诊断。
 */
uint32_t SensorService_ReadDensityAnalysis(float *period_22_5,
                                           float *period_45,
                                           float *dynamic_viscosity)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();
    uint32_t result;

    if ((period_22_5 == NULL) || (period_45 == NULL) || (dynamic_viscosity == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if ((SensorDriver_HasCapability(driver, SENSOR_CAP_DENSITY_ANALYSIS) == 0U) ||
        (driver->read_density_analysis == NULL)) {
        return SENSOR_CAPABILITY_UNSUPPORTED;
    }
    result = driver->read_density_analysis(period_22_5, period_45, dynamic_viscosity);
    return SensorComm_DiagnoseTimeout(result, "读取DSM密度分析参数");
}

/*
 * 函数用途：通过当前驱动读取零点霍尔电压。
 * 调用场景：零点流程和部件诊断读取DM4磁零点辅助量。
 * 关键约束：DM4驱动负责使能、互斥和等待首个新鲜值；超时统一诊断无线链路。
 */
uint32_t SensorService_ReadMagneticZeroVoltage(float *voltage_v)
{
    const SensorDriverOps *driver = SensorRuntime_GetDriver();
    uint32_t result;

    if (voltage_v == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    /* DM4驱动会处理与测水互斥、密度模式约束以及首个有效值等待。 */
    result = (driver->read_magnetic_zero_voltage != NULL)
                 ? driver->read_magnetic_zero_voltage(voltage_v)
                 : SENSOR_CAPABILITY_UNSUPPORTED;
    return SensorComm_DiagnoseTimeout(result, "读取零点霍尔");
}

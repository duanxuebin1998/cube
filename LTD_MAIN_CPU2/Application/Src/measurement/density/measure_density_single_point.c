/*
 * measure_density_single_point.c
 *
 * 文件职责：实现单点位置校验、结果原子发布、单点测量和固定点监测命令。
 */

#include "measure_density.h"
#include "measure_density_internal.h"
#include "../measure_commands_internal.h"

#include <stdint.h>
#include <stdio.h>

#include "abortable_delay.h"
#include "fault_manager.h"
#include "motor_ctrl.h"
#include "sensor_service.h"
#include "system_parameter.h"

/* 样机固定点监测开关：1=不读真实传感器，直接刷新虚拟温度/密度；0=恢复真实传感器流程。 */
#ifndef ENABLE_SINGLE_POINT_MONITORING_PROTOTYPE
#define ENABLE_SINGLE_POINT_MONITORING_PROTOTYPE 0U /* 单点监测原型功能开关。 */
#endif

#define SINGLE_POINT_MONITORING_PROTO_PERIOD_MS      500U /* 单点监测原型刷新周期，单位 ms。 */
#define SINGLE_POINT_MONITORING_PROTO_BASE_TEMP_RAW  22650U  /* 26.50℃：TEMP_TO_RAW(26.50) */
#define SINGLE_POINT_MONITORING_PROTO_BASE_DENS_RAW  99950U   /* 999.5kg/m3：水密度样机值，DENSITY_TO_RAW(999.5) */
#define SINGLE_POINT_MONITORING_PROTO_BASE_FREQ_HZ   121500U /* 单点监测原型基础频率，单位 Hz。 */
#define SINGLE_POINT_MONITORING_PROTO_BASE_VCF20     9995U /* 单点监测原型基础 VCF20 值。 */

/**
 * @brief 打印单点类目标位置超限原因。报错前一次性输出目标、边界、当前位置和错误码，便于现场判断参数还是位置异常。
 *
 * @param scene 用于现场日志标识当前单点位置校验场景的只读文字。
 * @param reason 用于诊断输出的 NUL 结尾只读原因文字；该文字补充错误发生背景，不代替函数另行记录或返回的数值错误码。
 * @param target_01mm 待校验或移动到的目标位置，单位 0.1 mm。
 * @param top_limit_01mm 目标位置允许达到的上边界，单位 0.1 mm。
 * @param bottom_limit_01mm 目标位置允许达到的下边界，单位 0.1 mm。
 * @param error_code 待记录、转换或判断的错误码。该值标识单点目标位置或量程错误，用于输出包含目标和允许范围的诊断。
 */
static void SinglePoint_PrintTargetRangeError(const char *scene,
                                              const char *reason,
                                              uint32_t target_01mm,
                                              uint32_t top_limit_01mm,
                                              uint32_t bottom_limit_01mm,
                                              uint32_t error_code)
{
    const char *safe_scene = (scene != NULL) ? scene : "单点位置";

    printf("%s\t目标位置检查失败\t原因:%s\t目标=%.1fmm\t零点=%.1fmm\t罐底盲区=%.1fmm\t当前位置=%.1fmm\t尺带=%.1fmm\t错误码=0x%08lX\r\n",
           safe_scene,
           (reason != NULL) ? reason : "未知",
           (double)target_01mm / 10.0,
           (double)top_limit_01mm / 10.0,
           (double)bottom_limit_01mm / 10.0,
           (double)g_measurement.debug_data.sensor_position / 10.0,
           (double)g_measurement.debug_data.cable_length / 10.0,
           (unsigned long)error_code);
}

/**
 * @brief 在电机动作前校验单点目标位置是否位于罐底盲区与罐高之间。
 *
 * @param scene 用于现场日志标识当前单点位置校验场景的只读文字。
 * @param target_01mm 待校验或移动到的目标位置，单位 0.1 mm。
 * @return NO_ERROR 表示目标位置有效；PARAM_CONFIG_MISSING 表示罐高未配置；PARAM_COMBINATION_CONFLICT
 *         表示盲区大于罐高；PARAM_RANGE_ERROR 表示目标超出罐高或进入盲区。
 * @note 目标、罐底盲区和罐高均使用 0.1 mm 定点单位；配置缺失、上下限冲突和目标越界分别返回对应错误码并打印现场范围。
 */

uint32_t SinglePoint_CheckTargetPosition(const char *scene, uint32_t target_01mm)
{
    uint32_t top_limit_01mm = g_deviceParams.tankHeight;
    uint32_t bottom_limit_01mm = g_deviceParams.blindZone;

    if (top_limit_01mm == 0U) {
        SinglePoint_PrintTargetRangeError(scene, "罐高未配置", target_01mm, top_limit_01mm, bottom_limit_01mm,
                                              PARAM_CONFIG_MISSING);
        return PARAM_CONFIG_MISSING;
    }

    if (bottom_limit_01mm > top_limit_01mm) {
        SinglePoint_PrintTargetRangeError(scene, "罐底盲区大于罐高", target_01mm, top_limit_01mm, bottom_limit_01mm,
                                              PARAM_COMBINATION_CONFLICT);
        return PARAM_COMBINATION_CONFLICT;
    }

    if (target_01mm > top_limit_01mm) {
        SinglePoint_PrintTargetRangeError(scene, "目标超过罐高", target_01mm, top_limit_01mm, bottom_limit_01mm,
                                              PARAM_RANGE_ERROR);
        return PARAM_RANGE_ERROR;
    }

    if (target_01mm < bottom_limit_01mm) {
        SinglePoint_PrintTargetRangeError(scene, "目标进入罐底盲区", target_01mm, top_limit_01mm, bottom_limit_01mm,
                                              PARAM_RANGE_ERROR);
        return PARAM_RANGE_ERROR;
    }

    return NO_ERROR;
}

/**
 * @brief 把一个真实稳定的固定点候选结果按六字段同代发布。
 *
 * @details 调用场景：单点测量完成、固定点监测取得新样本或样机生成完整样本后。
 * @note 关键约束：命令切换时拒绝发布；六字段全部写完并执行屏障后才递增对应代际。
 *
 * @param published 用于返回已经通过稳定窗口判定并发布的单点测量结果。
 * @param candidate 待校验或比较的候选值。该只读单点测量包含频率、密度、温度和位置，只有稳定性与有效性检查通过后才发布。
 * @param generation_counter 用于递增并发布单点结果代次的输出计数器。
 * @return 1 表示三个对象指针有效、临界区内未出现有效命令切换，六个结果字段已同代写入且 generation_counter 已递增；0 表示任一指针为空，或发布前检测到命令切换，本次候选未发布。
 */
static uint8_t SinglePoint_PublishStableResult(volatile DensityMeasurement *published,
                                               const DensityMeasurement *candidate,
                                               volatile uint32_t *generation_counter)
{
    uint32_t primask;

    if ((published == NULL) || (candidate == NULL) || (generation_counter == NULL)) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (HasEffectiveCommandSwitchRequest()) {
        if (primask == 0U) {
            __enable_irq();
        }
        return 0U;
    }

    published->temperature = candidate->temperature;
    published->density = candidate->density;
    published->temperature_position = candidate->temperature_position;
    published->standard_density = candidate->standard_density;
    published->vcf20 = candidate->vcf20;
    published->weight_density = candidate->weight_density;
    __DMB();
    (*generation_counter)++;
    __DMB();

    if (primask == 0U) {
        __enable_irq();
    }
    return 1U;
}

/**
 * @brief 把完整的单点测量候选交给固定点原子发布器，并递增测量完成代际。
 *
 * @details 调用场景：真实稳定测量和显式串口虚拟展示取得完整六字段后。
 * @note 关键约束：命令切换期间拒绝发布，调用方必须处理返回值。
 *
 * @param candidate 待校验或比较的候选值。该只读单点测量包含频率、密度、温度和位置，只有稳定性与有效性检查通过后才发布。
 * @return 1 表示单点测量候选已原子发布且测量完成代际已递增；候选非法或发布失败时返回 0。
 */
uint8_t SinglePoint_PublishMeasurementResult(const DensityMeasurement *candidate)
{
    return SinglePoint_PublishStableResult(&g_measurement.single_point_measurement,
                                           candidate,
                                           &g_measurement.measurement_complete_counter);
}

/**
 * @brief 把完整的固定点监测候选交给固定点原子发布器，并递增监测样本代际。
 *
 * @details 调用场景：真实稳定监测和显式串口虚拟展示取得完整六字段后。
 * @note 关键约束：命令切换期间拒绝发布，调用方必须处理返回值。
 *
 * @param candidate 待校验或比较的候选值。该只读单点测量包含频率、密度、温度和位置，只有稳定性与有效性检查通过后才发布。
 * @return 1 表示固定点监测候选已原子发布且监测样本代际已递增；候选非法或发布失败时返回 0。
 */
uint8_t SinglePoint_PublishMonitoringResult(const DensityMeasurement *candidate)
{
    return SinglePoint_PublishStableResult(&g_measurement.single_point_monitoring,
                                           candidate,
                                           &g_measurement.monitoring_sample_counter);
}

#if ENABLE_SINGLE_POINT_MONITORING_PROTOTYPE
/**
 * @brief 写入固定点监测样机虚拟数据。
 *
 * 只更新 CPU3/上位机读取的测量结果和调试字段，不访问传感器串口，适合无传感器样机演示。
 *
 * @param sample_index 样机虚拟样本序号；用于生成本轮测试频率、密度、温度和位置的可重复变化量，不是正式测量点索引。
 * @return 1 表示样机虚拟样本已写入固定点监测结果并递增样本代际；发布失败时返回 0。
 */
static uint8_t SinglePointMonitoringPrototype_WriteSample(uint32_t sample_index)
{
    static const int16_t temp_wave_x100[] = { 0, 6, 12, 18, 24, 18, 12, 6, 0, -4, -8, -4 };
    static const int16_t pos_wave_01mm[]  = { 0, 1, 2, 3, 2, 1, 0, -1, -2, -1, 0, 1 };
    const uint32_t wave_count = (uint32_t)(sizeof(temp_wave_x100) / sizeof(temp_wave_x100[0]));
    uint32_t idx = sample_index % wave_count;
    uint32_t base_pos_01mm = DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_SINGLE_POINT_MONITORING_POSITION);
    int32_t current_pos_01mm = (int32_t)base_pos_01mm + (int32_t)pos_wave_01mm[idx];
    uint32_t position_raw = DensityInternal_ValueToU01mmClamped(current_pos_01mm, "固定点监测样机位置");
    uint32_t temperature_raw = (uint32_t)((int32_t)SINGLE_POINT_MONITORING_PROTO_BASE_TEMP_RAW + temp_wave_x100[idx]);
    uint32_t density_raw = SINGLE_POINT_MONITORING_PROTO_BASE_DENS_RAW;
    DensityMeasurement candidate = {0};
    /* 样机要求固定展示水密度，三类密度保持一致，避免误读为真实油品修正值。 */
    uint32_t standard_density_raw = density_raw;
    uint32_t weight_density_raw = density_raw;
    uint32_t vcf20_raw = SINGLE_POINT_MONITORING_PROTO_BASE_VCF20 + (idx % 6U);

    candidate.temperature = temperature_raw;
    candidate.density = density_raw;
    candidate.temperature_position = position_raw;
    candidate.standard_density = standard_density_raw;
    candidate.vcf20 = vcf20_raw;
    candidate.weight_density = weight_density_raw;

    g_measurement.debug_data.sensor_position = (int32_t)position_raw;
    g_measurement.debug_data.motor_distance = (int32_t)position_raw;
    g_measurement.debug_data.temperature = temperature_raw;
    g_measurement.debug_data.frequency = SINGLE_POINT_MONITORING_PROTO_BASE_FREQ_HZ + idx * 15U;
    g_measurement.debug_data.current_amplitude = 96U + (idx % 5U);
    g_measurement.debug_data.motor_state = 0U;
    g_measurement.debug_data.motor_speed = 0U;

    printf("固定点监测样机\t位置=%.1fmm\t温度=%.2f℃\t密度=%.2f\t标密=%.2f\tVCF20=%lu\t计重密度=%.2f\r\n",
           (double)position_raw / 10.0,
           RAW_TO_TEMP(temperature_raw),
           RAW_TO_DENSITY(density_raw),
           RAW_TO_DENSITY(standard_density_raw),
           (unsigned long)vcf20_raw,
           RAW_TO_DENSITY(weight_density_raw));

    return SinglePoint_PublishStableResult(&g_measurement.single_point_monitoring,
                                           &candidate,
                                           &g_measurement.monitoring_sample_counter);
}

/**
 * @brief 固定点监测样机循环。
 *
 * 宏启用时替代真实传感器读取；循环期间只响应命令切换，不做 UART6 传感器通信。
 *
 * @return STATE_SWITCH 表示样机循环被新命令正常打断；其他非零值为点位运动、驻留等待或固定点样本发布失败，该持续循环没有主动 NO_ERROR 完成出口。
 */
static uint32_t SinglePointMonitoringPrototype_Run(void)
{
    uint32_t sample_index = 0U;

    printf("固定点监测样机模式已启用\t宏=ENABLE_SINGLE_POINT_MONITORING_PROTOTYPE\t周期=%lu ms\r\n",
           (unsigned long)SINGLE_POINT_MONITORING_PROTO_PERIOD_MS);
    printf("固定点监测样机模式\t不切换密度模式，不读取传感器，只刷新单点监测虚拟温度/密度\r\n");

    while (1) {
        uint32_t ret;

        if (HasEffectiveCommandSwitchRequest()) {
            return STATE_SWITCH;
        }

        g_measurement.device_status.device_state = STATE_SPTESTING;
        g_measurement.device_status.error_code = NO_ERROR;
        if (SinglePointMonitoringPrototype_WriteSample(sample_index) == 0U) {
            return STATE_SWITCH;
        }
        sample_index++;

        ret = AbortableDelay_CommandSwitch(SINGLE_POINT_MONITORING_PROTO_PERIOD_MS, 50U);
        if (ret != NO_ERROR) {
            return ret;
        }
    }
}
#endif

/**
 * @brief 单点测量命令：移动到指定高度 -> 单点稳定读取。
 */
void CMD_SinglePointMeasurement(void)
{
    uint32_t ret = 0;
    DensityMeasurement candidate = {0};
    g_measurement.device_status.device_state = STATE_SINGLEPOINTING;


    ret = SinglePoint_CheckTargetPosition("单点测量", DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_SINGLE_POINT_MEASUREMENT_POSITION));
    SET_ERROR(ret);

    ret = MotorCtrl_JogMoveToPosition((float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_SINGLE_POINT_MEASUREMENT_POSITION) / 10.0f,
                                              MotorCtrl_GetDefaultSpeedX100());
    if (ret == STATE_SWITCH) {
        return;
    }
    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_SPTESTING;

    ret = SensorService_EnableDensityMode();
    if (ret == STATE_SWITCH) {
        return;
    }
    SET_ERROR(ret);

    ret = SinglePoint_ReadSensor(&candidate);
    if (ret == STATE_SWITCH) {
        return;
    }
    SET_ERROR(ret);

    if (SinglePoint_PublishStableResult(&g_measurement.single_point_measurement,
                                        &candidate,
                                        &g_measurement.measurement_complete_counter) == 0U) {
        return;
    }
    g_measurement.device_status.device_state = STATE_SINGLEPOINTOVER;
}

/**
 * @brief 单点监测命令：移动到监测高度 -> 循环单点稳定读取（直到命令切换）。
 */
void CMD_SinglePointMonitoring(void)
{
    uint32_t ret = 0;
    g_measurement.device_status.device_state = STATE_RUNTOPOINTING;

    if (HasEffectiveCommandSwitchRequest()) {
        printf("检测到命令切换请求，停止固定点监测移动\r\n");
        return;
    }

    ret = SinglePoint_CheckTargetPosition("固定点监测", DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_SINGLE_POINT_MONITORING_POSITION));
    SET_ERROR(ret);

    ret = MotorCtrl_JogMoveToPosition((float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_SINGLE_POINT_MONITORING_POSITION) / 10.0f,
                                              MotorCtrl_GetDefaultSpeedX100());
    if (ret == STATE_SWITCH) {
        printf("固定点监测移动阶段检测到命令切换请求，退出\r\n");
        return;
    }
    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_SPTESTING;

#if ENABLE_SINGLE_POINT_MONITORING_PROTOTYPE
    ret = SinglePointMonitoringPrototype_Run();
    if (ret == STATE_SWITCH) {
        printf("固定点监测样机模式检测到命令切换请求，退出\r\n");
        return;
    }
    SET_ERROR(ret);
#else
    ret = SensorService_EnableDensityMode();
    if (ret == STATE_SWITCH) {
        printf("固定点监测切换密度模式时检测到命令切换请求，退出\r\n");
        return;
    }
    SET_ERROR(ret);

    while (1) {
        if (HasEffectiveCommandSwitchRequest()) {
            printf("检测到命令切换请求，停止当前操作\r\n");
            return;
        }

        DensityMeasurement candidate = {0};

        ret = SinglePoint_ReadSensor(&candidate);
        if (ret == STATE_SWITCH) {
            printf("固定点监测读数阶段检测到命令切换请求，退出\r\n");
            return;
        }
        SET_ERROR(ret);

        if (SinglePoint_PublishStableResult(&g_measurement.single_point_monitoring,
                                            &candidate,
                                            &g_measurement.monitoring_sample_counter) == 0U) {
            return;
        }
    }
#endif
}

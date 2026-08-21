/**
 * @file service_debug_demo.c
 * @brief CPU2 单点测量显示演示数据发布实现，不驱动电机且不读取传感器。
 */

#include "service_debug_internal.h"
#include "stm32f4xx_hal.h"
#include "measure_density.h"
#include "serial_command_parser.h"
#include "system_parameter.h"

#include <stdint.h>
#include <stdio.h>

#define DEMO_SINGLE_POINT_DISPLAY_BASE_DENS_RAW 83521U /* 单点展示基础密度，单位 kg/m3 x100。 */
#define DEMO_SINGLE_POINT_DISPLAY_STD_OFFSET_RAW 7U /* 单点展示标密偏移，单位 kg/m3 x100。 */
#define DEMO_SINGLE_POINT_DISPLAY_WEIGHT_OFFSET_RAW 3U /* 单点展示计重密度偏移，单位 kg/m3 x100。 */
#define DEMO_SINGLE_POINT_DISPLAY_APPROACH_DENS_RAW 83487U /* 单点展示到位前密度，单位 kg/m3 x100。 */

/**
 * @brief 判断单点数据显示演示是否应因命令切换而退出。
 * @return 1 表示检测到有效命令切换请求，函数已把设备状态恢复待机并清除演示电机状态，调用方应退出单点展示；0 表示当前没有命令切换请求，可继续演示。
 */
static uint8_t Demo_SinglePointDisplay_ShouldAbort(void)
{
    if (!HasEffectiveCommandSwitchRequest()) {
        return 0;
    }

    printf("单点展示检测到新命令，退出当前演示\r\n");
    g_measurement.device_status.device_state = STATE_STANDBY;
    g_measurement.debug_data.motor_state = 0U;
    return 1;
}

/**
 * @brief 构造完整虚拟六字段，并复用正式固定点发布器同步两块结果和对应代际。
 *
 * @details 调用场景：串口 X 展示在运行到点和稳定刷新阶段生成虚拟样本后。
 * @note 关键约束：任一发布被命令切换拒绝时返回 0，调用方立即退出展示。
 *
 * @param temperature_raw 温度原始。
 * @param density_raw 密度原始。
 * @param pos_01mm 待发布的单点测量位置，单位 0.1 mm。
 * @param standard_density_raw 密度原始。
 * @param vcf20_raw 放大 10000 倍保存的无量纲 VCF20 虚拟值，直接写入单点测量记录。
 * @param weight_density_raw 扭力密度原始。
 * @return 1 表示完整虚拟六字段已通过正式固定点发布器同步并递增代际；输入或发布失败时返回 0。
 */
static uint8_t Demo_SinglePointDisplay_PublishResult(uint32_t temperature_raw,
                                                     uint32_t density_raw,
                                                     uint32_t pos_01mm,
                                                     uint32_t standard_density_raw,
                                                     uint32_t vcf20_raw,
                                                     uint32_t weight_density_raw)
{
    DensityMeasurement candidate = {0};

    candidate.temperature = temperature_raw;
    candidate.density = density_raw;
    candidate.temperature_position = pos_01mm;
    candidate.standard_density = standard_density_raw;
    candidate.vcf20 = vcf20_raw;
    candidate.weight_density = weight_density_raw;

    if (SinglePoint_PublishMeasurementResult(&candidate) == 0U) {
        return 0U;
    }
    return SinglePoint_PublishMonitoringResult(&candidate);
}

/**
 * @brief 不驱动电机也不读取传感器；先模拟接近测量点，再持续发布带小幅波动的单点显示数据，直至新命令打断。
 *
 * 目标位置优先使用单点测量配置；未配置时依次回退到当前有效传感器位置、罐高一半和 1500 mm，并为展示过程构造高于目标点的有效罐高。
 * 第一阶段把全局设备状态设置为运行到测量点，在六个 500 ms 步骤中从目标上方 300 mm 逐步接近目标，同时发布固定的虚拟温度、密度和调试量。
 * 到达目标后切换为单点测量中，以十二点波形持续微调位置、温度、密度、标准密度、VCF20、重量密度、频率、幅值和扭力，并每 500 ms 发布一次测量及监测结果。
 * 任意新命令、命令队列拒绝发布或结果发布失败都会终止展示；本函数不读取真实传感器，也不启动电机。
 *
 * @note 该展示会持续覆盖 g_measurement 的设备状态、调试量和单点结果，退出后由后续真实业务流程重新建立测量快照。
 */
void Demo_SinglePointDisplayMock(void)
{
    static const int16_t temp_wave_x100[]    = { 0, 6, 12, 18, 24, 18, 12, 6, 0, -4, -8, -4 };
    static const int16_t density_wave_x100[] = { 0, 1, 2, 3, 4, 3, 2, 1, 0, -1, -2, -1 };
    static const int16_t pos_wave_01mm[]     = { 0, 2, 4, 6, 8, 6, 4, 2, 0, -2, -4, -2 };
    const uint32_t wave_count = (uint32_t)(sizeof(temp_wave_x100) / sizeof(temp_wave_x100[0]));
    uint32_t target_pos_01mm;
    uint32_t tank_height_01mm;
    uint32_t start_pos_01mm;
    uint32_t current_pos_01mm;
    uint32_t cable_01mm;
    uint32_t i;

    target_pos_01mm = g_deviceParams.singlePointMeasurementPosition;
    if (target_pos_01mm == 0U) {
        if (g_measurement.debug_data.sensor_position > 0) {
            target_pos_01mm = (uint32_t)g_measurement.debug_data.sensor_position;
        } else if (g_deviceParams.tankHeight > 0U) {
            target_pos_01mm = g_deviceParams.tankHeight / 2U;
        } else {
            target_pos_01mm = 15000U;
        }
    }

    tank_height_01mm = g_deviceParams.tankHeight;
    if ((tank_height_01mm == 0U) || (tank_height_01mm <= target_pos_01mm)) {
        tank_height_01mm = target_pos_01mm + 12000U;
    }

    start_pos_01mm = (target_pos_01mm > 3000U) ? (target_pos_01mm - 3000U) : 0U;
    current_pos_01mm = start_pos_01mm;

    printf("\r\n===== 单点测量展示模式开始 =====\r\n");
    printf("展示说明: 不读真实传感器、不驱动电机，只刷新单点测量显示字段\r\n");
    printf("停止方式: 下发任意新命令即可退出展示\r\n");

    g_measurement.device_status.zero_point_status = 0U;
    g_measurement.device_status.error_code = NO_ERROR;
    g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
    g_measurement.debug_data.motor_state = 2U;
    g_measurement.debug_data.motor_speed = 120U;

    for (i = 0U; i < 6U; i++) {
        if (Demo_SinglePointDisplay_ShouldAbort()) {
            return;
        }

        current_pos_01mm = start_pos_01mm +
                (uint32_t)(((uint64_t)(target_pos_01mm - start_pos_01mm) * (uint64_t)(i + 1U)) / 6U);
        cable_01mm = (tank_height_01mm > current_pos_01mm) ? (tank_height_01mm - current_pos_01mm) : 0U;

        g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
        g_measurement.debug_data.sensor_position = (int32_t)current_pos_01mm;
        g_measurement.debug_data.cable_length = (int32_t)cable_01mm;
        g_measurement.debug_data.motor_distance = (int32_t)cable_01mm;
        g_measurement.debug_data.temperature = 22580U;
        g_measurement.debug_data.frequency = 121300U + i * 20U;
        g_measurement.debug_data.air_frequency = 121900U;
        g_measurement.debug_data.current_amplitude = 88U + i;
        g_measurement.debug_data.current_weight = 3200U + i * 10U;

        if (Demo_SinglePointDisplay_PublishResult(
                22580U,
                DEMO_SINGLE_POINT_DISPLAY_APPROACH_DENS_RAW,
                current_pos_01mm,
                DEMO_SINGLE_POINT_DISPLAY_APPROACH_DENS_RAW - DEMO_SINGLE_POINT_DISPLAY_STD_OFFSET_RAW,
                9997U,
                DEMO_SINGLE_POINT_DISPLAY_APPROACH_DENS_RAW - DEMO_SINGLE_POINT_DISPLAY_WEIGHT_OFFSET_RAW) == 0U) {
            (void)Demo_SinglePointDisplay_ShouldAbort();
            return;
        }

        printf("单点展示\t运行到测量点 [%lu/6] 位置=%.1fmm\r\n",
               (unsigned long)(i + 1U),
               current_pos_01mm / 10.0f);
        HAL_Delay(500);
    }

    printf("单点展示\t已到达展示点，开始刷新虚拟温度/密度\r\n");

    for (i = 0U;; i++) {
        uint32_t idx = i % wave_count;
        uint32_t temperature_raw;
        uint32_t density_raw;
        uint32_t standard_density_raw;
        uint32_t weight_density_raw;
        uint32_t vcf20_raw;

        if (Demo_SinglePointDisplay_ShouldAbort()) {
            return;
        }

        current_pos_01mm = (uint32_t)((int32_t)target_pos_01mm + pos_wave_01mm[idx]);
        cable_01mm = (tank_height_01mm > current_pos_01mm) ? (tank_height_01mm - current_pos_01mm) : 0U;

        temperature_raw = (uint32_t)(20000 + 2650 + temp_wave_x100[idx]);
        density_raw = (uint32_t)((int32_t)DEMO_SINGLE_POINT_DISPLAY_BASE_DENS_RAW + (int32_t)density_wave_x100[idx]);
        standard_density_raw = density_raw - DEMO_SINGLE_POINT_DISPLAY_STD_OFFSET_RAW;
        weight_density_raw = density_raw - DEMO_SINGLE_POINT_DISPLAY_WEIGHT_OFFSET_RAW;
        vcf20_raw = 9995U + (idx % 6U);

        g_measurement.device_status.device_state = STATE_SINGLEPOINTING;
        g_measurement.debug_data.motor_state = 0U;
        g_measurement.debug_data.motor_speed = 0U;
        g_measurement.debug_data.sensor_position = (int32_t)current_pos_01mm;
        g_measurement.debug_data.cable_length = (int32_t)cable_01mm;
        g_measurement.debug_data.motor_distance = (int32_t)cable_01mm;
        g_measurement.debug_data.temperature = temperature_raw;
        g_measurement.debug_data.frequency = 121500U + idx * 15U;
        g_measurement.debug_data.air_frequency = 121980U;
        g_measurement.debug_data.current_amplitude = 96U + (idx % 5U);
        g_measurement.debug_data.current_weight = 3280U + (idx % 4U) * 8U;

        if (Demo_SinglePointDisplay_PublishResult(temperature_raw,
                                                  density_raw,
                                                  current_pos_01mm,
                                                  standard_density_raw,
                                                  vcf20_raw,
                                                  weight_density_raw) == 0U) {
            (void)Demo_SinglePointDisplay_ShouldAbort();
            return;
        }

        printf("单点展示\t状态=固定点测量中 位置=%.1fmm 温度=%.2fC 密度=%.2f 标密=%.2f VCF20=%lu 重量密度=%.2f\r\n",
               current_pos_01mm / 10.0f,
               RAW_TO_TEMP(temperature_raw),
               RAW_TO_DENSITY(density_raw),
               RAW_TO_DENSITY(standard_density_raw),
               (unsigned long)vcf20_raw,
               RAW_TO_DENSITY(weight_density_raw));

        HAL_Delay(500);
    }
}

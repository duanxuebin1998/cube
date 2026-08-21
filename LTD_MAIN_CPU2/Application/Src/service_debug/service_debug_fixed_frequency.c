/*
 * service_debug_fixed_frequency.c
 *
 * 文件职责：解析并执行固定频率液位搜索维护命令，不修改持久参数。
 */

#include "service_debug_fixed_frequency.h"

#include "measure_oil_level.h"
#include "main.h"
#include "system_parameter.h"

#include <stdio.h>
#include <stdlib.h>

#define LF_DEFAULT_BAND_HZ 15U /* LF 命令未给死区且设备参数未配置时使用的默认半宽，单位 Hz。 */

/**
 * @brief 把设备中的 x10 频率阈值换算成 LF 命令显示和运行使用的整数 Hz。
 *
 * @return oilLevelThreshold 未配置时返回 15 Hz，否则返回四舍五入后的整数 Hz。
 */
static uint32_t FixedLevel_GetConfiguredDeadbandHz(void)
{
    uint32_t raw_deadband = g_deviceParams.oilLevelThreshold;

    if (raw_deadband == 0U) {
        return LF_DEFAULT_BAND_HZ;
    }
    return (raw_deadband + (DENSITY_PARAM_MIGRATE_FACTOR / 2U)) /
           DENSITY_PARAM_MIGRATE_FACTOR;
}

/**
 * @brief 打印 LF 当前使用的生产目标频率和首次找液死区。
 *
 * @note 只读设备参数，不初始化电机、不启动运动。
 */
void ServiceDebug_FixedFrequencyPrintConfig(void)
{
    printf("LF CONFIG target=%luHz search_deadband=%luHz "
           "follow_deadband=%luHz\r\n",
           (unsigned long)g_deviceParams.oilLevelFrequency,
           (unsigned long)FixedLevel_GetConfiguredDeadbandHz(),
           (unsigned long)((g_deviceParams.oilLevelHysteresisThreshold == 0U) ?
                           LF_DEFAULT_BAND_HZ :
                           ((g_deviceParams.oilLevelHysteresisThreshold +
                             (DENSITY_PARAM_MIGRATE_FACTOR / 2U)) /
                            DENSITY_PARAM_MIGRATE_FACTOR)));
}

/**
 * @brief 解析本次 LF 覆盖值并调用生产方法5固定频率闭环。
 *
 * @param command 已通过串口严格语法校验的 LF 或 LF=<频率>[,<死区>] 命令。
 * @return 生产方法5闭环返回的真实结果码。
 * @note 覆盖值只在本次调用中使用，不修改设备参数或 FRAM。
 */
uint32_t ServiceDebug_FixedFrequencyRun(const uint8_t *command)
{
    uint32_t target_hz = g_deviceParams.oilLevelFrequency;
    uint32_t deadband_hz = FixedLevel_GetConfiguredDeadbandHz();
    uint32_t ret;
    char *end;

    if (command == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (command[2] == '=') {
        target_hz = (uint32_t)strtoul((const char *)&command[3], &end, 10);
        if (*end == ',') {
            deadband_hz = (uint32_t)strtoul(end + 1, NULL, 10);
        }
    }

    printf("LF START target=%luHz deadband=%luHz\r\n",
           (unsigned long)target_hz,
           (unsigned long)deadband_hz);
    ret = OilLevel_RunFixedFrequencySearch(target_hz, deadband_hz);
    printf("LF RESULT status=%s code=0x%08lX level=%lu frequency=%lu\r\n",
           (ret == NO_ERROR) ? "FOUND" :
           ((ret == STATE_SWITCH) ? "ABORT" : "ERROR"),
           (unsigned long)ret,
           (unsigned long)g_measurement.oil_measurement.oil_level,
           (unsigned long)g_measurement.oil_measurement.current_frequency);
    g_measurement.device_status.device_state = STATE_DEBUG_MODE;
    return ret;
}

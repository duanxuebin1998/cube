/*
 * 文件职责：液位标定入口和设备参数持久化。
 * 本文件负责标定参数校验、位置刷新和保存，不实现液位搜索算法。
 * 结果发布和 AO 处理统一由 oil_level_runtime.c 提供，本文件不重复实现。
 */
#include "oil_level_internal.h"
#include "motor_ctrl.h"
#include "system_parameter.h"
#include <stdint.h>
#include <stdio.h>

/**
 * @brief 根据当前尺带长度和校准命令更新罐高。
 *
 * @details 校验候选罐高后同步 AO、当前位置和持久化参数；测量运动由液位搜索流程负责。
 *
 * @note 命令快照无效或候选罐高越界时不写入设备参数。
 */

void CorrectOilLevelProcess(void) {
    printf("液位流程\t开始标定液位\r\n");
    int64_t tank_height = (int64_t)g_measurement.debug_data.cable_length +
                          (int64_t)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_OIL_LEVEL);

    if ((tank_height < 0) || (tank_height > (int64_t)UINT32_MAX)) {
        printf("液位流程\t修正后罐高非法：%ld(0.1mm)，取消修正，保留原罐高和修正参数\r\n",
               (long)tank_height);
        return;
    }
    g_deviceParams.tankHeight = (uint32_t)tank_height;
    /* 液位罐高变化后立即归一化 AO 量程，避免旧量程阻塞后续命令。 */
    (void)normalize_ao_params_after_write();
    DeviceCommandArguments_ClearIfUnchanged(DEVICE_COMMAND_ARG_CALIBRATE_OIL_LEVEL); /* 未被后续写入时清零 */
    printf("液位流程\t标定完成，罐高设置为：%lu(0.1mm)\r\n", (unsigned long)g_deviceParams.tankHeight);
    MotorCtrl_RefreshPositionFromActiveSource();  /* 修正罐高后按当前记步源刷新当前位置 */
    OilLevel_SyncCurrentPositionToResult("液位修正");
    save_device_params(); /* 保存参数 */
}

/*
 * device_param_sync.h
 *
 *  Created on: 2025年12月12日
 *      Author: Duan Xuebin
 */

#ifndef DEVICE_PARAM_SYNC_H
#define DEVICE_PARAM_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include "system_parameter.h"
#include "cpu2_communicate.h"
#include "display_tankopera.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 同步所有 DeviceParameters → CPU2
 * 步骤：
 *  1) 遍历 param_meta[]
 *  2) 使用 operanum 在 g_deviceParams 中找到对应字段
 *  3) 如果 h->val 与 g_deviceParams 不同：
 *       - h->val = g_deviceParams 中的值
 *       - 调用 10 功能码下发到 CPU2
 */
void DeviceParams_SyncAllToCPU2(void);

/* 只同步一个 operanum 对应的参数
 *  - 用于某个菜单参数修改后，只下发该参数
 */
void DeviceParams_SyncOneToCPU2(int operanum);

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_PARAM_SYNC_H */

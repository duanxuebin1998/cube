/*
 * device_param_sync.h
 *
 *  Created on: 2025年12月12日
 *      Author: Duan Xuebin
 */

#ifndef DEVICE_PARAM_SYNC_H
/* DEVICE_PARAM_SYNC_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define DEVICE_PARAM_SYNC_H

#include <stdint.h>
#include <stdbool.h>
#include "system_parameter.h"
#include "cpu2_communicate.h"
#include "display_tankopera.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 同步所有 DeviceParameters → CPU2。
 *
 * 1) 预检完整差异集；普通持久参数不允许在运行态开始部分同步。
 * 2) 跳过一次性命令和由 CPU2 运行期维护的参数。
 * 3) 使用 operanum 在 g_deviceParams 中找到对应字段。
 * 4) 如果 h->val 与 g_deviceParams 不同。
 * 七个命令前置参数由合法 FC10 ACK 确认，并更新局部镜像、请求后台完整刷新。
 * 普通参数保留批量 ACK 路径。
 * 仅在 CPU2 成功响应后更新 h->val。
 * 返回：全部差异参数同步成功或无需同步时为 true，否则为 false。
 *
 * @return true 表示无需同步，或全部差异参数已逐项写入 CPU2 并通过事务确认；false 表示预检失败，或任一参数的板间 Modbus 写入失败。
 */
bool DeviceParams_SyncAllToCPU2(void);

/* 只同步一个 operanum 对应的参数
 *  - 用于某个菜单参数修改后，只下发该参数
 */

#ifdef __cplusplus
}
#endif

#endif /* DEVICE_PARAM_SYNC_H */

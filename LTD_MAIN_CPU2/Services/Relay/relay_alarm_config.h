#ifndef RELAY_ALARM_CONFIG_H
#define RELAY_ALARM_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "system_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 按当前设备量程校验一路完整继电器报警配置。 */
bool RelayAlarmConfig_IsValid(const DeviceParameters *params,
                              const RelayAlarmConfig *config);

/*
 * 按 FC10 实际写区间校验候选继电器配置。
 * 禁用通道只校验本次触及字段；启用通道和相关罐高变化执行完整校验。
 */
bool RelayAlarmConfig_WriteCandidateIsValid(const DeviceParameters *params,
                                            uint16_t start,
                                            uint16_t count);

/*
 * 原位修复旧 FRAM 中的非法字段。
 * 返回值：存储内容发生变化的通道位掩码。
 * invalid_channel_mask：因发现非法配置而被禁用的通道位掩码；
 * 合法的一次性清报警命令会被清零，但不计为非法配置。
 */
uint32_t RelayAlarmConfig_Normalize(DeviceParameters *params,
                                    uint32_t *invalid_channel_mask);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_ALARM_CONFIG_H */

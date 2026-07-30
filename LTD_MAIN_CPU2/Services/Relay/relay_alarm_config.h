#ifndef RELAY_ALARM_CONFIG_H
/* RELAY_ALARM_CONFIG_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define RELAY_ALARM_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#include "system_parameter.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 校验一路继电器报警配置的枚举、阻尼、阈值和滞回。
 *
 * @details 调用场景：CPU2 权威写入校验和启动归一化调用。
 *
 * 按当前设备量程校验一路完整继电器报警配置。
 *
 * @note 关键约束：当前阻尼字段为预留项，只接受 0。
 *
 * @param params 完整设备参数只读快照；包含版本、结构长度、测量与协议配置、AO、继电器以及 CRC 等持久字段，函数不会修改该快照。
 * @param config 只读单路继电器报警配置；包含工作模式、数字源、触点与报警模式、无效值策略、报警源、四级阈值、滞回、阻尼和清除锁存命令。
 * @return true 表示上述校验全部通过；false 表示至少一项校验未通过。
 */
bool RelayAlarmConfig_IsValid(const DeviceParameters *params,
                              const RelayAlarmConfig *config);

/**
 * @brief 按 FC10 实际写区间校验继电器候选参数。
 *
 * @details 调用场景：CPU2 权威写入口解析完整候选快照后、提交运行态和 FRAM 前调用。
 *
 * 按 FC10 实际写区间校验候选继电器配置。
 * 禁用通道只校验本次触及字段；启用通道和相关罐高变化执行完整校验。
 *
 * @note 关键约束：禁用通道允许逐项形成暂时不完整组合；启用或已启用通道必须完整合法；
 *           罐高变化只复核启用且实际使用对应液位源的通道。
 *
 * @param params 完整设备参数只读快照；包含版本、结构长度、测量与协议配置、AO、继电器以及 CRC 等持久字段，函数不会修改该快照。
 * @param start 本次连续处理范围的起始索引。该值是继电器配置候选写区间的起始字段或寄存器地址，用于判断受影响字段。
 * @param count 参与本次处理的数据项数量。
 * @return true 表示所有被 FC10 区间触及的继电器字段、阈值交叉关系和相关罐高或 AO 来源约束均有效；false 表示参数为空、区间为空、任一触及字段越界，或组合配置在本次候选中不成立。
 */
bool RelayAlarmConfig_WriteCandidateIsValid(const DeviceParameters *params,
                                            uint16_t start,
                                            uint16_t count);

/**
 * @brief 按字段修复旧 FRAM 中的非法继电器配置并报告通道掩码。
 *
 * @details 调用场景：CPU2 上电加载 DeviceParameters 后调用。
 *
 * 原位修复旧 FRAM 中的非法字段。
 * 返回值：存储内容发生变化的通道位掩码。
 * invalid_channel_mask：因发现非法配置而被禁用的通道位掩码；
 * 合法的一次性清报警命令会被清零，但不计为非法配置。
 *
 * @note 关键约束：非法通道必须禁用；合法的一次性清报警命令只清零，不计为非法通道。
 *
 * @param params 可写设备参数对象；函数按职责从寄存器或 FRAM 还原字段，并在完成后更新版本、结构长度或 CRC 等完整性信息。
 * @param invalid_channel_mask 用于返回配置非法的继电器通道位掩码，低 4 位对应通道 1 至 4。
 * @return 返回本轮被修正字段的位掩码；全部配置原本合法时返回 0。
 */
uint32_t RelayAlarmConfig_Normalize(DeviceParameters *params,
                                    uint32_t *invalid_channel_mask);

#ifdef __cplusplus
}
#endif

#endif /* RELAY_ALARM_CONFIG_H */

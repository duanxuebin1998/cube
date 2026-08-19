/*
 * 模块职责：声明Safe线协议帧的长度探测、控制帧编解码和周期快报解码接口。
 * 调用顺序：流式接收先探测完整长度，再完成解码，最后交由会话层验收。
 * 结果边界：基础解码成功不等于会话、序号、能力或测量数据已经可用。
 */
#ifndef SENSOR_SAFE_FRAME_H_
/* SENSOR_SAFE_FRAME_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_FRAME_H_

#include "sensor_safe_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 从连续字节流头部识别控制帧或固定快报帧的期望长度。
 *
 * @details 调用场景：UART6 接收状态机找到 A5 5A 后决定还需接收的字节数。
 * @note 关键约束：只读取长度字段，不代表帧已通过 CRC 或语义校验。
 */
SensorSafeResult SensorSafeFrame_PeekLength(const uint8_t *data,
                                            size_t available,
                                            uint16_t *frame_len_out,
                                            uint8_t *is_fast_frame_out);

/**
 * @brief 逐字段编码一个 V1 控制帧并计算 CRC32C。
 *
 * @details 调用场景：CPU2 发送 HELLO、测量、维护和会话控制请求。
 * @note 关键约束：不对 C 结构体内存求 CRC，输出长度不得超过 128 字节。
 */
SensorSafeResult SensorSafeFrame_EncodeControl(const SensorSafeControlFields *fields,
                                               uint8_t *output,
                                               size_t output_capacity,
                                               uint16_t *output_len);

/**
 * @brief 校验并解码一个完整控制帧。
 *
 * @details 调用场景：UART6 收满 frame_len 后，在会话层使用响应前调用。
 * @note 关键约束：返回成功前已校验 SOF、长度、版本、CRC、消息类型和保留标志位。
 */
SensorSafeResult SensorSafeFrame_DecodeControl(const uint8_t *frame,
                                               size_t frame_len,
                                               SensorSafeControlFrame *decoded);

/**
 * @brief 校验并解码固定 44 字节快报帧。
 *
 * @details 调用场景：周期上报接收路径进入会话连续性检查之前。
 * @note 关键约束：只接受 PERIODIC_UPLINK 和可用 4 bit 表示的测量模式 0..5。
 */
SensorSafeResult SensorSafeFrame_DecodeFastReport(const uint8_t *frame,
                                                  size_t frame_len,
                                                  SensorSafeFastReport *decoded);

/**
 * @brief 解码所有 RSP/ERR 共有的 20 字节响应前缀。
 *
 * @details 调用场景：控制帧已通过帧层和会话层校验后解析业务负载。
 * @note 关键约束：该函数不替代状态位、诊断位和数据年龄的安全判断。
 */
SensorSafeResult SensorSafeFrame_DecodeCommonResponse(const uint8_t *payload,
                                                      size_t payload_len,
                                                      SensorSafeCommonResponse *decoded);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_FRAME_H_ */

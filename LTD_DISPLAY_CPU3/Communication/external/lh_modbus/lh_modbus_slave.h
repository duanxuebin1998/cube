#ifndef LH_MODBUS_SLAVE_H_
/* LH_MODBUS_SLAVE_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define LH_MODBUS_SLAVE_H_

#include <stdint.h>

/* LH 外部 Modbus 功能码 0x01，读取线圈；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define LH_MODBUS_FUNC_READ_COILS          0x01U
/* LH 外部 Modbus 功能码 0x03，读取保持寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define LH_MODBUS_FUNC_READ_HOLDING_REGS   0x03U
/* LH 外部 Modbus 功能码 0x04，读取输入寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define LH_MODBUS_FUNC_READ_INPUT_REGS     0x04U
/* LH 外部 Modbus 功能码 0x05，写单个线圈；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define LH_MODBUS_FUNC_WRITE_SINGLE_COIL   0x05U
/* LH 外部 Modbus 功能码 0x10，写多个保持寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define LH_MODBUS_FUNC_WRITE_MULTI_REGS    0x10U

/* LH Modbus 异常码：非法功能码；异常响应功能码需同时置最高位。 */
#define LH_MODBUS_EX_ILLEGAL_FUNCTION      0x01U
/* LH Modbus 异常码：非法数据地址；异常响应功能码需同时置最高位。 */
#define LH_MODBUS_EX_ILLEGAL_ADDRESS       0x02U
/* LH Modbus 异常码：非法数据值或长度；异常响应功能码需同时置最高位。 */
#define LH_MODBUS_EX_ILLEGAL_VALUE         0x03U
/* LH Modbus 异常码：从站内部处理失败；异常响应功能码需同时置最高位。 */
#define LH_MODBUS_EX_SLAVE_DEVICE_FAILURE  0x04U
/* LH Modbus 异常码：从站忙，当前无法完成请求；异常响应功能码需同时置最高位。 */
#define LH_MODBUS_EX_SLAVE_DEVICE_BUSY     0x06U

/* LH 单帧最多读取 2000 个线圈，遵循标准 Modbus 位读取上限。 */
#define LH_MODBUS_MAX_READ_COILS           2000U
/* LH 单帧最多读取 125 个 16 位寄存器，受标准响应 PDU 长度限制。 */
#define LH_MODBUS_MAX_READ_REGISTERS       125U
/* LH 单帧最多写入 2 个保持寄存器；该实现只接受一个完整 UInt32 参数，避免跨字段批量写入。 */
#define LH_MODBUS_MAX_WRITE_REGISTERS      2U

/* LH Modbus 请求处理结果；保留具体帧级失败原因，便于上层统计和现场通信诊断。 */
typedef enum {
    /* LH Modbus 帧处理结果。 */
    LH_MODBUS_OK = 0, /* 处理成功。 */
    LH_MODBUS_ERR_BAD_LENGTH, /* 帧长度与协议要求不符。 */
    LH_MODBUS_ERR_CRC, /* 帧 CRC 校验失败。 */
    LH_MODBUS_ERR_ADDRESS_MISMATCH, /* 帧地址不是本机或目标设备地址。 */
    LH_MODBUS_ERR_UNSUPPORTED_FUNCTION /* 请求的 Modbus 功能码不受支持。 */
} LhModbusResult;

/**
 * @brief 处理一帧完整的 LH Modbus RTU 请求。
 *
 * @details 调用场景：外部 COM 端口选择 LH 协议后，由 CPU3 主循环分发完整 RTU 帧。
 * @note 关键约束：只响应本机地址；命令和参数写入均以 CPU2 合法 ACK 为成功依据。
 */
LhModbusResult lh_modbus_process(const uint8_t *rx,
                                 uint16_t rx_len,
                                 uint8_t *tx,
                                 uint16_t *tx_len);

/**
 * @brief 已生成正常或异常响应帧时，外部 COM 分发层统一视为本帧处理完成。
 *
 * CPU3 外部 COM 分发使用的统一返回值包装。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx 转交 LH 协议核心处理器的响应输出缓冲区；tx_len 非 0 时内容可直接交给外部端口发送。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 已生成正常或异常响应时返回 0；没有响应帧时返回 LH Modbus 解析错误码。
 */
uint32_t lh_modbus_process_for_dispatch(const uint8_t *rx,
                                        uint16_t rx_len,
                                        uint8_t *tx,
                                        uint16_t *tx_len);

#endif /* LH_MODBUS_SLAVE_H_ */

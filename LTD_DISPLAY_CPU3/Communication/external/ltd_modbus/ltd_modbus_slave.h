#ifndef LTD_MODBUS_SLAVE_H_
/* LTD_MODBUS_SLAVE_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define LTD_MODBUS_SLAVE_H_

#include <stdint.h>

/* LTD 外部 Modbus 功能码 0x03，读取保持寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define LTD_MODBUS_FUNC_READ_HOLDING_REGS 0x03U
/* LTD 外部 Modbus 功能码 0x04，读取输入寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define LTD_MODBUS_FUNC_READ_INPUT_REGS   0x04U
/* LTD 外部 Modbus 功能码 0x10，写多个保持寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define LTD_MODBUS_FUNC_WRITE_MULTI_REGS  0x10U

/* LTD Modbus 异常码：非法功能码；异常响应功能码需同时置最高位。 */
#define LTD_MODBUS_EX_ILLEGAL_FUNCTION    0x01U
/* LTD Modbus 异常码：非法数据地址；异常响应功能码需同时置最高位。 */
#define LTD_MODBUS_EX_ILLEGAL_ADDRESS     0x02U
/* LTD Modbus 异常码：非法数据值或长度；异常响应功能码需同时置最高位。 */
#define LTD_MODBUS_EX_ILLEGAL_VALUE       0x03U
/* LTD Modbus 异常码：从站内部处理失败；异常响应功能码需同时置最高位。 */
#define LTD_MODBUS_EX_SLAVE_DEVICE_FAILURE 0x04U
/* LTD Modbus 异常码：从站忙，当前无法完成请求；异常响应功能码需同时置最高位。 */
#define LTD_MODBUS_EX_SLAVE_DEVICE_BUSY   0x06U

/* LTD 单帧最多读取 125 个 16 位寄存器，受标准响应 PDU 长度限制。 */
#define LTD_MODBUS_MAX_READ_REGISTERS     125U
/* LTD 单帧最多写入 122 个 16 位寄存器，遵循 FC10 PDU 上限。 */
#define LTD_MODBUS_MAX_WRITE_REGISTERS    122U

/* LTD Modbus 请求处理结果；保留具体帧级失败原因，便于上层统计和现场通信诊断。 */
typedef enum {
    /* LTD Modbus 帧处理结果。 */
    LTD_MODBUS_OK = 0, /* 处理成功。 */
    LTD_MODBUS_ERR_BAD_LENGTH, /* 帧长度与协议要求不符。 */
    LTD_MODBUS_ERR_CRC, /* 帧 CRC 校验失败。 */
    LTD_MODBUS_ERR_ADDRESS_MISMATCH, /* 帧地址不是本机或目标设备地址。 */
    LTD_MODBUS_ERR_UNSUPPORTED_FUNCTION /* 请求的 Modbus 功能码不受支持。 */
} LtdModbusResult;

/**
 * @brief 实现 CPU3 对外 LTD 共享 Modbus 从站。
 *
 * @details 调用场景：COM1/COM2/COM3 选择 COM_PROTO_LTD 后由主循环分发完整 RTU 帧。
 *
 * 处理一帧 LTD 共享 Modbus RTU 请求。
 * 读取由 CPU3 已确认快照响应，写入只有在 CPU2 返回合法 ACK 后才返回成功。
 *
 * @note 关键约束：地址不匹配和 CRC 错误不回包；共享字段写成功必须以 CPU2 合法 ACK 为前提。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx 完整 LTD Modbus RTU 正常或异常响应的输出缓冲区；调用方容量必须覆盖协议允许的最大响应。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 LTD Modbus 分发结果；LTD_MODBUS_OK 表示已完成处理，其他值区分帧长、CRC、从机地址和功能码错误；tx_len 非零时仍应发送已生成的异常响应。
 */
LtdModbusResult ltd_modbus_process(const uint8_t *rx,
                                   uint16_t rx_len,
                                   uint8_t *tx,
                                   uint16_t *tx_len);

/**
 * @brief 外部 COM 只要已有正常或异常响应帧，就视为本帧已被协议层完整处理。
 *
 * CPU3 外部 COM 分发使用的统一返回值包装。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx 转交 LTD 协议核心处理器的响应输出缓冲区；tx_len 非 0 时内容可直接交给外部端口发送。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 已生成正常或异常响应时返回 0；没有响应帧时返回 LTD Modbus 解析错误码。
 */
uint32_t ltd_modbus_process_for_dispatch(const uint8_t *rx,
                                         uint16_t rx_len,
                                         uint8_t *tx,
                                         uint16_t *tx_len);

#endif /* LTD_MODBUS_SLAVE_H_ */

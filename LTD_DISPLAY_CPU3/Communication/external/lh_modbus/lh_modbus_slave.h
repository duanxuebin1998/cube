#ifndef LH_MODBUS_SLAVE_H_
#define LH_MODBUS_SLAVE_H_

#include <stdint.h>

#define LH_MODBUS_FUNC_READ_HOLDING_REGS   0x03U
#define LH_MODBUS_FUNC_READ_INPUT_REGS     0x04U
#define LH_MODBUS_FUNC_WRITE_SINGLE_COIL   0x05U
#define LH_MODBUS_FUNC_WRITE_MULTI_REGS    0x10U

#define LH_MODBUS_EX_ILLEGAL_FUNCTION      0x01U
#define LH_MODBUS_EX_ILLEGAL_ADDRESS       0x02U
#define LH_MODBUS_EX_ILLEGAL_VALUE         0x03U
#define LH_MODBUS_EX_SLAVE_DEVICE_FAILURE  0x04U
#define LH_MODBUS_EX_SLAVE_DEVICE_BUSY     0x06U

#define LH_MODBUS_MAX_READ_REGISTERS       125U
#define LH_MODBUS_MAX_WRITE_REGISTERS      2U

typedef enum {
    LH_MODBUS_OK = 0,
    LH_MODBUS_ERR_BAD_LENGTH,
    LH_MODBUS_ERR_CRC,
    LH_MODBUS_ERR_ADDRESS_MISMATCH,
    LH_MODBUS_ERR_UNSUPPORTED_FUNCTION
} LhModbusResult;

/*
 * 函数用途：处理一帧完整的 LH Modbus RTU 请求。
 * 调用场景：外部 COM 端口选择 LH 协议后，由 CPU3 主循环分发完整 RTU 帧。
 * 关键约束：只响应本机地址；命令和参数写入均以 CPU2 合法 ACK 为成功依据。
 */
LhModbusResult lh_modbus_process(const uint8_t *rx,
                                 uint16_t rx_len,
                                 uint8_t *tx,
                                 uint16_t *tx_len);

/* CPU3 外部 COM 分发使用的统一返回值包装。 */
uint32_t lh_modbus_process_for_dispatch(const uint8_t *rx,
                                        uint16_t rx_len,
                                        uint8_t *tx,
                                        uint16_t *tx_len);

#endif /* LH_MODBUS_SLAVE_H_ */

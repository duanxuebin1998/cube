#ifndef LTD_MODBUS_SLAVE_H_
#define LTD_MODBUS_SLAVE_H_

#include <stdint.h>

#define LTD_MODBUS_FUNC_READ_HOLDING_REGS 0x03U
#define LTD_MODBUS_FUNC_READ_INPUT_REGS   0x04U
#define LTD_MODBUS_FUNC_WRITE_MULTI_REGS  0x10U

#define LTD_MODBUS_EX_ILLEGAL_FUNCTION    0x01U
#define LTD_MODBUS_EX_ILLEGAL_ADDRESS     0x02U
#define LTD_MODBUS_EX_ILLEGAL_VALUE       0x03U
#define LTD_MODBUS_EX_SLAVE_DEVICE_FAILURE 0x04U
#define LTD_MODBUS_EX_SLAVE_DEVICE_BUSY   0x06U

#define LTD_MODBUS_MAX_READ_REGISTERS     125U
#define LTD_MODBUS_MAX_WRITE_REGISTERS    122U

typedef enum {
    LTD_MODBUS_OK = 0,
    LTD_MODBUS_ERR_BAD_LENGTH,
    LTD_MODBUS_ERR_CRC,
    LTD_MODBUS_ERR_ADDRESS_MISMATCH,
    LTD_MODBUS_ERR_UNSUPPORTED_FUNCTION
} LtdModbusResult;

/*
 * 处理一帧 LTD 共享 Modbus RTU 请求。
 * 读取由 CPU3 已确认快照响应，写入只有在 CPU2 返回合法 ACK 后才返回成功。
 */
LtdModbusResult ltd_modbus_process(const uint8_t *rx,
                                   uint16_t rx_len,
                                   uint8_t *tx,
                                   uint16_t *tx_len);

/* CPU3 外部 COM 分发使用的统一返回值包装。 */
uint32_t ltd_modbus_process_for_dispatch(const uint8_t *rx,
                                         uint16_t rx_len,
                                         uint8_t *tx,
                                         uint16_t *tx_len);

#endif /* LTD_MODBUS_SLAVE_H_ */

#ifndef SI7000_MODBUS_SLAVE_H_
#define SI7000_MODBUS_SLAVE_H_

#include <stdint.h>

/* 当前兼容的 SI7000 地址窗口大小，必须与 .c 中枚举和 golden frame 脚本保持一致。 */
#define SI7000_COIL_COUNT            16U
#define SI7000_DISCRETE_INPUT_COUNT  32U
#define SI7000_HOLDING_REG_COUNT     23U
#define SI7000_INPUT_REG_COUNT       620U

/* SI7000 帧处理结果；tx_len>0 时即使返回非 OK，也表示已经生成可发送的异常响应。 */
typedef enum {
    SI7000_MODBUS_OK = 0,
    SI7000_MODBUS_ERR_BADLEN,
    SI7000_MODBUS_ERR_CRC,
    SI7000_MODBUS_ERR_ADDR_MISMATCH,
    SI7000_MODBUS_ERR_FUNC_UNSUPPORT
} Si7000ModbusResult;

/* 处理一帧完整 Modbus RTU 请求；返回值描述分发层状态，tx_len>0 表示已生成响应。 */
Si7000ModbusResult si7000_modbus_process(const uint8_t *rx_buf,
                                         uint16_t rx_len,
                                         uint8_t *tx_buf,
                                         uint16_t *tx_len);
/* 主分发使用的 uint32_t 返回值口径。 */
uint32_t si7000_modbus_process_for_dispatch(const uint8_t *rx_buf,
                                            uint16_t rx_len,
                                            uint8_t *tx_buf,
                                            uint16_t *tx_len);


/* 手动刷新四类 SI7000 影子区，主要供测试或后续联调入口调用。 */
void si7000_modbus_sync_from_system(void);

#endif /* SI7000_MODBUS_SLAVE_H_ */

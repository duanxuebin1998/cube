#ifndef SI_MODBUS_SLAVE_H_
#define SI_MODBUS_SLAVE_H_

#include <stdbool.h>
#include <stdint.h>

/* 当前兼容的 SI 地址窗口大小，必须与 .c 中枚举和 golden frame 脚本保持一致。 */
#define SI_COIL_COUNT            16U
#define SI_DISCRETE_INPUT_COUNT  32U
#define SI_HOLDING_REG_COUNT     23U
#define SI_INPUT_REG_COUNT       620U

/* SI 帧处理结果；tx_len>0 时即使返回非 OK，也表示已经生成可发送的异常响应。 */
typedef enum {
    SI_MODBUS_OK = 0,
    SI_MODBUS_ERR_BADLEN,
    SI_MODBUS_ERR_CRC,
    SI_MODBUS_ERR_ADDR_MISMATCH,
    SI_MODBUS_ERR_FUNC_UNSUPPORT
} SiModbusResult;

/* 处理一帧完整 Modbus RTU 请求；返回值描述分发层状态，tx_len>0 表示已生成响应。 */
SiModbusResult si_modbus_process(const uint8_t *rx_buf,
                                         uint16_t rx_len,
                                         uint8_t *tx_buf,
                                         uint16_t *tx_len);
/* 主分发使用的 uint32_t 返回值口径。 */
uint32_t si_modbus_process_for_dispatch(const uint8_t *rx_buf,
                                            uint16_t rx_len,
                                            uint8_t *tx_buf,
                                            uint16_t *tx_len);


/* 手动刷新四类 SI 影子区，主要供测试或后续联调入口调用。 */
void si_modbus_sync_from_system(void);

/* 自动 profile 调度入口；由主循环周期调用，内部按 RTC 分钟去重。 */
void si_modbus_periodic_task(void);

/* 请求启动 SI Profile；统一锁存开始时间并向 CPU2 下发 CMD_SI_PROFILE。 */
bool si_profile_request_start(void);

#endif /* SI_MODBUS_SLAVE_H_ */

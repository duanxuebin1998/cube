#include "ltd_modbus_slave.h"

#include <stdbool.h>

#include "address.h"
#include "cpu2_communicate.h"
#include "my_crc.h"
#include "stateformodbus.h"

/* LTD 对外从站地址复用 CPU3 拨码地址；地址非法时保持历史默认地址 1。 */
static uint8_t ltd_get_slave_address(void)
{
    if ((SlaveAddress >= 1) && (SlaveAddress <= 247)) {
        return (uint8_t)SlaveAddress;
    }
    return 1U;
}

/* 从 Modbus PDU 读取高字节在前的 16 位值。 */
static uint16_t ltd_be16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/* 把 16 位寄存器值按 Modbus 大端顺序写入响应。 */
static void ltd_write_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFFU);
}

/* 为当前响应追加低字节在前的 Modbus RTU CRC。 */
static void ltd_append_crc(uint8_t *tx, uint16_t payload_len, uint16_t *tx_len)
{
    uint16_t crc = CRC16_Calculate(tx, payload_len);
    tx[payload_len] = (uint8_t)(crc & 0xFFU);
    tx[payload_len + 1U] = (uint8_t)(crc >> 8);
    *tx_len = (uint16_t)(payload_len + 2U);
}

/* 构造标准 Modbus 异常响应，异常帧仍使用收到请求的外部从站地址。 */
static void ltd_build_exception(uint8_t address,
                                uint8_t function,
                                uint8_t exception,
                                uint8_t *tx,
                                uint16_t *tx_len)
{
    tx[0] = address;
    tx[1] = (uint8_t)(function | 0x80U);
    tx[2] = exception;
    ltd_append_crc(tx, 3U, tx_len);
}

/* 检查连续寄存器范围，使用减法避免 start+count 的 16 位溢出。 */
static bool ltd_register_range_valid(uint16_t start, uint16_t count, uint16_t total)
{
    return (count > 0U) && (start < total) && (count <= (uint16_t)(total - start));
}

/*
 * 处理 FC03/FC04 读取。
 * 参数和状态都必须来自 CPU2 已确认快照；快照不可用时返回设备忙而不是旧数据。
 */
static void ltd_handle_read(uint8_t address,
                            uint8_t function,
                            uint16_t start,
                            uint16_t count,
                            uint8_t *tx,
                            uint16_t *tx_len)
{
    uint16_t registers[LTD_MODBUS_MAX_READ_REGISTERS];
    uint16_t total;
    bool read_ok;

    if ((count == 0U) || (count > LTD_MODBUS_MAX_READ_REGISTERS)) {
        ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
        return;
    }

    total = (function == LTD_MODBUS_FUNC_READ_HOLDING_REGS) ?
            (uint16_t)HOLEREGISTER_STOP : (uint16_t)INPUTREGISTER_AMOUNT;
    if (!ltd_register_range_valid(start, count, total)) {
        ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
        return;
    }

    if (function == LTD_MODBUS_FUNC_READ_HOLDING_REGS) {
        read_ok = CPU2_CommReadHoldingSnapshot(start, count, registers);
    } else {
        read_ok = CPU2_CommReadInputSnapshot(start, count, registers);
    }
    if (!read_ok) {
        ltd_build_exception(address, function, LTD_MODBUS_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
        return;
    }

    tx[0] = address;
    tx[1] = function;
    tx[2] = (uint8_t)(count * 2U);
    for (uint16_t i = 0U; i < count; i++) {
        ltd_write_be16(&tx[3U + (i * 2U)], registers[i]);
    }
    ltd_append_crc(tx, (uint16_t)(3U + (count * 2U)), tx_len);
}

/*
 * 处理 FC10 写入。
 * LTD 共享参数均以 32 位字段维护，因此只接受偶数地址和偶数寄存器数量。
 */
static void ltd_handle_write(uint8_t address,
                             const uint8_t *rx,
                             uint16_t rx_len,
                             uint8_t *tx,
                             uint16_t *tx_len)
{
    uint16_t start = ltd_be16(&rx[2]);
    uint16_t count = ltd_be16(&rx[4]);
    uint8_t byte_count = rx[6];
    uint16_t registers[LTD_MODBUS_MAX_WRITE_REGISTERS];

    if ((count == 0U) || (count > LTD_MODBUS_MAX_WRITE_REGISTERS) ||
        ((start & 1U) != 0U) || ((count & 1U) != 0U) ||
        (byte_count != (uint8_t)(count * 2U)) ||
        (rx_len != (uint16_t)(9U + byte_count))) {
        ltd_build_exception(address,
                            LTD_MODBUS_FUNC_WRITE_MULTI_REGS,
                            LTD_MODBUS_EX_ILLEGAL_VALUE,
                            tx,
                            tx_len);
        return;
    }

    if (!ltd_register_range_valid(start, count, (uint16_t)HOLEREGISTER_STOP)) {
        ltd_build_exception(address,
                            LTD_MODBUS_FUNC_WRITE_MULTI_REGS,
                            LTD_MODBUS_EX_ILLEGAL_ADDRESS,
                            tx,
                            tx_len);
        return;
    }

    for (uint16_t i = 0U; i < count; i++) {
        registers[i] = ltd_be16(&rx[7U + (i * 2U)]);
    }

    if (!CPU2_CommWriteHoldingRegisters(start, count, registers)) {
        ltd_build_exception(address,
                            LTD_MODBUS_FUNC_WRITE_MULTI_REGS,
                            LTD_MODBUS_EX_SLAVE_DEVICE_BUSY,
                            tx,
                            tx_len);
        return;
    }

    tx[0] = address;
    tx[1] = LTD_MODBUS_FUNC_WRITE_MULTI_REGS;
    tx[2] = rx[2];
    tx[3] = rx[3];
    tx[4] = rx[4];
    tx[5] = rx[5];
    ltd_append_crc(tx, 6U, tx_len);
}

/*
 * 函数用途：实现 CPU3 对外 LTD 共享 Modbus 从站。
 * 调用场景：COM1/COM2/COM3 选择 COM_PROTO_LTD 后由主循环分发完整 RTU 帧。
 * 关键约束：地址不匹配和 CRC 错误不回包；所有成功写响应都以 CPU2 合法 ACK 为前提。
 */
LtdModbusResult ltd_modbus_process(const uint8_t *rx,
                                   uint16_t rx_len,
                                   uint8_t *tx,
                                   uint16_t *tx_len)
{
    uint8_t address;
    uint8_t function;
    uint16_t start;
    uint16_t count;

    if ((rx == NULL) || (tx == NULL) || (tx_len == NULL)) {
        return LTD_MODBUS_ERR_BAD_LENGTH;
    }
    *tx_len = 0U;

    if (rx_len < 4U) {
        return LTD_MODBUS_ERR_BAD_LENGTH;
    }

    address = ltd_get_slave_address();
    if (rx[0] != address) {
        return LTD_MODBUS_ERR_ADDRESS_MISMATCH;
    }
    if (!SlaveCheckCRC(rx, (int)rx_len)) {
        return LTD_MODBUS_ERR_CRC;
    }

    function = rx[1];
    switch (function) {
    case LTD_MODBUS_FUNC_READ_HOLDING_REGS:
    case LTD_MODBUS_FUNC_READ_INPUT_REGS:
        if (rx_len != 8U) {
            ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
            return LTD_MODBUS_OK;
        }
        start = ltd_be16(&rx[2]);
        count = ltd_be16(&rx[4]);
        ltd_handle_read(address, function, start, count, tx, tx_len);
        return LTD_MODBUS_OK;

    case LTD_MODBUS_FUNC_WRITE_MULTI_REGS:
        if (rx_len < 9U) {
            ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
            return LTD_MODBUS_OK;
        }
        ltd_handle_write(address, rx, rx_len, tx, tx_len);
        return LTD_MODBUS_OK;

    default:
        ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_FUNCTION, tx, tx_len);
        return LTD_MODBUS_ERR_UNSUPPORTED_FUNCTION;
    }
}

/* 外部 COM 只要已有正常或异常响应帧，就视为本帧已被协议层完整处理。 */
uint32_t ltd_modbus_process_for_dispatch(const uint8_t *rx,
                                         uint16_t rx_len,
                                         uint8_t *tx,
                                         uint16_t *tx_len)
{
    LtdModbusResult ret = ltd_modbus_process(rx, rx_len, tx, tx_len);
    if ((tx_len != NULL) && (*tx_len > 0U)) {
        return 0U;
    }
    return (uint32_t)ret;
}

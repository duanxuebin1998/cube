#include "ltd_modbus_slave.h"

#include <stdbool.h>

#include "address.h"
#include "cpu3_comm_display_params.h"
#include "cpu2_communicate.h"
#include "my_crc.h"
#include "stateformodbus.h"
#include "system_parameter.h"

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

/*
 * 函数用途：按现行Modbus地址查找CPU3本机参数元数据。
 * 调用场景：LTD FC03/FC10访问0x7000本机参数块。
 * 关键约束：仅返回 CPU3 本机参数；每个字段继续按两个 16 位寄存器对外发布。
 */
static const struct ParameterMetadata *ltd_find_cpu3_param(uint16_t address)
{
    int i;

    for (i = 0; i < param_metaAmount; i++) {
        const struct ParameterMetadata *meta = &param_meta[i];

        if (Cpu3Local_IsParam((OperatingNumber)meta->operanum) &&
            (meta->startadd == address) && (meta->rgstcnt == REG_STRIDE)) {
            return meta;
        }
    }
    return NULL;
}

/*
 * 函数用途：从CPU3当前有效运行态生成本机参数寄存器块。
 * 调用场景：LTD FC03读取0x7000参数段。
 * 关键约束：未分配槽读取为0；每个现有字段保持高字在前、低字在后。
 */
static bool ltd_read_cpu3_holding(uint16_t start, uint16_t count, uint16_t *registers)
{
    uint16_t i;

    for (i = 0U; i < count; i++) {
        uint16_t address = (uint16_t)(start + i);
        const struct ParameterMetadata *meta;
        uint16_t field_start;
        uint32_t value;

        if ((address < HOLDREGISTER_CPU3_BASE) || (address >= HOLDREGISTER_CPU3_END)) {
            return false;
        }
        field_start = (uint16_t)(address & 0xFFFEU);
        meta = ltd_find_cpu3_param(field_start);
        if (meta == NULL) {
            registers[i] = 0U;
            continue;
        }
        value = (uint32_t)Cpu3Local_ReadValue((OperatingNumber)meta->operanum);
        registers[i] = (address == field_start) ?
                       (uint16_t)(value >> 16) : (uint16_t)(value & 0xFFFFU);
    }
    return true;
}

/* CPU3 串口配置首版只读，避免正在承载请求的端口在响应完成前改变线制。 */
static bool ltd_cpu3_param_writable(const struct ParameterMetadata *meta)
{
    return (meta != NULL) && meta->authority_write &&
           !Cpu3Local_IsUartParam((OperatingNumber)meta->operanum);
}

/*
 * 函数用途：事务式写入一个完整的 CPU3 本机 32 位参数字段。
 * 调用场景：LTD FC10写0x7000参数段。
 * 关键约束：本机段每次只写一个字段；范围非法与 FRAM 写后读回失败必须区分。
 */
static uint8_t ltd_write_cpu3_holding(uint16_t start, uint16_t count, const uint16_t *registers)
{
    const struct ParameterMetadata *meta;
    int32_t value;

    if (count != REG_STRIDE) {
        return LTD_MODBUS_EX_ILLEGAL_VALUE;
    }
    if (((start & 1U) != 0U) || (start < HOLDREGISTER_CPU3_BASE) ||
        ((uint32_t)start + (uint32_t)count > (uint32_t)HOLDREGISTER_CPU3_END)) {
        return LTD_MODBUS_EX_ILLEGAL_ADDRESS;
    }
    meta = ltd_find_cpu3_param(start);
    if (!ltd_cpu3_param_writable(meta)) {
        return LTD_MODBUS_EX_ILLEGAL_ADDRESS;
    }

    value = (int32_t)(((uint32_t)registers[0] << 16) | (uint32_t)registers[1]);
    if (meta->flag_checkvalue && ((value < meta->valuemin) || (value > meta->valuemax))) {
        return LTD_MODBUS_EX_ILLEGAL_VALUE;
    }
    if (!Cpu3Local_WriteValueChecked((OperatingNumber)meta->operanum, value)) {
        return LTD_MODBUS_EX_SLAVE_DEVICE_FAILURE;
    }
    return 0U;
}

/*
 * 处理 FC03/FC04 读取。
 * CPU2 参数和状态来自已确认快照；CPU3 本机参数直接读取当前有效运行态。
 */
static void ltd_handle_read(uint8_t address,
                            uint8_t function,
                            uint16_t start,
                            uint16_t count,
                            uint8_t *tx,
                            uint16_t *tx_len)
{
    uint16_t registers[LTD_MODBUS_MAX_READ_REGISTERS];
    bool read_ok;

    if ((count == 0U) || (count > LTD_MODBUS_MAX_READ_REGISTERS)) {
        ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
        return;
    }

    if (function == LTD_MODBUS_FUNC_READ_HOLDING_REGS) {
        if (start >= HOLDREGISTER_CPU3_BASE) {
            if ((uint32_t)start + (uint32_t)count > (uint32_t)HOLDREGISTER_CPU3_END) {
                ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
                return;
            }
            if (!ltd_read_cpu3_holding(start, count, registers)) {
                ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
                return;
            }
            read_ok = true;
        } else {
            if (!LtdModbus_RangeWithin(start, count, HOLDREGISTER_AMOUNT)) {
                ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
                return;
            }
            read_ok = CPU2_CommReadHoldingSnapshot(start, count, registers);
        }
    } else {
        if (!LtdModbus_RangeWithin(start, count, INPUTREGISTER_AMOUNT)) {
            ltd_build_exception(address, function, LTD_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
            return;
        }
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
    uint8_t local_exception;

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

    if ((start < HOLDREGISTER_CPU3_BASE) &&
        !LtdModbus_HoldingWriteRangeIsValid(start, count)) {
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

    if ((start == HOLDREGISTER_DEVICEPARAM_COMMAND) && (count == REG_STRIDE)) {
        uint32_t command_value;

        if (count != 2U) {
            ltd_build_exception(address,
                                LTD_MODBUS_FUNC_WRITE_MULTI_REGS,
                                LTD_MODBUS_EX_ILLEGAL_VALUE,
                                tx,
                                tx_len);
            return;
        }
        command_value = ((uint32_t)registers[0] << 16) | (uint32_t)registers[1];
        if (!LtdModbus_CommandIsImplemented(command_value)) {
            ltd_build_exception(address,
                                LTD_MODBUS_FUNC_WRITE_MULTI_REGS,
                                LTD_MODBUS_EX_ILLEGAL_VALUE,
                                tx,
                                tx_len);
            return;
        }
    }

    if (start >= HOLDREGISTER_CPU3_BASE) {
        local_exception = ltd_write_cpu3_holding(start, count, registers);
        if (local_exception != 0U) {
            ltd_build_exception(address,
                                LTD_MODBUS_FUNC_WRITE_MULTI_REGS,
                                local_exception,
                                tx,
                                tx_len);
            return;
        }
    } else if (!CPU2_CommWriteHoldingRegisters(start, count, registers)) {
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
 * 关键约束：地址不匹配和 CRC 错误不回包；共享字段写成功必须以 CPU2 合法 ACK 为前提。
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

#include "lh_modbus_slave.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "address.h"
#include "cpu2_communicate.h"
#include "lh_register_map.h"
#include "my_crc.h"
#include "stateformodbus.h"
#include "system_parameter.h"

/* LH 参数写入后等待 CPU2 回读确认的总超时 1500 ms；超时返回从站忙或失败，不能假定参数已经生效。 */
#define LH_PARAMETER_CONFIRM_TIMEOUT_MS 1500U
/* LH 参数确认期间允许的最大轮询次数 128；与总超时共同限制内部通信占用。 */
#define LH_PARAMETER_CONFIRM_MAX_POLLS  128U
/* LH 电机速度写入下限 0.10 m/min，线值单位为 0.01 m/min。 */
#define LH_MOTOR_SPEED_MIN_X100          10U
/* LH 电机速度写入上限 6.00 m/min，线值单位为 0.01 m/min。 */
#define LH_MOTOR_SPEED_MAX_X100          600U

/* LH 保持寄存器可写区到 CPU2 共享寄存器的映射描述；每项给出 LH 起址、长度和内部目标起址。 */
typedef struct {
    /* LH 写寄存器区到 CPU2 共享参数区的连续区间映射。 */
    uint16_t lh_start; /* 该可写区在 LH 保持寄存器表中的起始地址。 */
    uint16_t lh_count; /* 该可写区连续占用的 LH 保持寄存器数量。 */
    uint16_t shared_start; /* 该可写区映射到 CPU2 共享保持寄存器的起始地址。 */
} LhWritableField;

/* LH 只开放现场文档中的可写字段；只读兼容字段不进入此表。 */
static const LhWritableField s_lh_writable_fields[] = {
    {LH_HR_TANK_HEIGHT,                    2U, HOLDREGISTER_DEVICEPARAM_TANKHEIGHT},
    {LH_HR_CALIBRATE_OIL_LEVEL,            2U, HOLDREGISTER_DEVICEPARAM_CALIBRATE_OIL_LEVEL},
    {LH_HR_CALIBRATE_WATER_LEVEL,          2U, HOLDREGISTER_DEVICEPARAM_CALIBRATE_WATER_LEVEL},
    {LH_HR_CALIBRATE_TANK_HEIGHT,          2U, HOLDREGISTER_DEVICEPARAM_CALIBRATE_TANK_HEIGHT},
    {LH_HR_MOTOR_COMMAND_DISTANCE,         2U, HOLDREGISTER_DEVICEPARAM_MOTOR_COMMAND_DISTANCE},
    {LH_HR_MOTOR_CURRENT,                  1U, HOLDREGISTER_DEVICEPARAM_MOTOR_CURRENT},
    {LH_HR_MAX_MOTOR_SPEED,                2U, HOLDREGISTER_DEVICEPARAM_MAX_MOTOR_SPEED},
    {LH_HR_BLIND_ZONE,                     2U, HOLDREGISTER_DEVICEPARAM_BLINDZONE},
    {LH_HR_ENCODER_WHEEL_CIRCUMFERENCE,    2U, HOLDREGISTER_DEVICEPARAM_ENCODER_WHEEL_CIRCUMFERENCE_MM},
    {LH_HR_LIQUID_TO_WATER_PROBE_DISTANCE, 1U, HOLDREGISTER_DEVICEPARAM_LIQUID_SENSOR_DISTANCE_DIFF},
    {LH_HR_MAX_DOWN_DISTANCE,              1U, HOLDREGISTER_DEVICEPARAM_MAXDOWNDISTANCE},
    {LH_HR_AO_WORK_MODE,                   1U, HOLDREGISTER_DEVICEPARAM_AO_WORK_MODE},
    {LH_HR_AO_CURRENT_MODE,                1U, HOLDREGISTER_DEVICEPARAM_AO_CURRENT_MODE},
    {LH_HR_AO_OUTPUT_SOURCE,               1U, HOLDREGISTER_DEVICEPARAM_AO_OUTPUT_SOURCE},
    {LH_HR_AO_CURRENT_CORRECTION_MA_X1000, 2U, HOLDREGISTER_DEVICEPARAM_AO_CURRENT_CORRECTION_MA_X1000},
    {LH_HR_AO_FIXED_CURRENT_MA_X100,       2U, HOLDREGISTER_DEVICEPARAM_AO_FIXED_CURRENT_MA_X100},
    {LH_HR_AO_RANGE_0_01MM,                2U, HOLDREGISTER_DEVICEPARAM_AO_RANGE_0_01MM},
    {LH_HR_AO_RANGE_100_01MM,              2U, HOLDREGISTER_DEVICEPARAM_AO_RANGE_100_01MM},
    {LH_HR_AO_DAMPING_X10_S,               2U, HOLDREGISTER_DEVICEPARAM_AO_DAMPING_X10_S},
    {LH_HR_AO_FAULT_MODE,                  1U, HOLDREGISTER_DEVICEPARAM_AO_FAULT_MODE},
    {LH_HR_AO_FAULT_CURRENT_MA_X100,       2U, HOLDREGISTER_DEVICEPARAM_AO_FAULT_CURRENT_MA_X100},
    {LH_HR_AO_POWER_ON_CURRENT_MA_X100,    2U, HOLDREGISTER_DEVICEPARAM_AO_POWER_ON_CURRENT_MA_X100},

    {LH_HR_RELAY_OPERATING_MODE(LH_HR_RELAY_K1_BASE),   1U, HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(0U)},
    {LH_HR_RELAY_DIGITAL_SOURCE(LH_HR_RELAY_K1_BASE),   1U, HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(0U)},
    {LH_HR_RELAY_CONTACT_TYPE(LH_HR_RELAY_K1_BASE),     1U, HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(0U)},
    {LH_HR_RELAY_ALARM_MODE(LH_HR_RELAY_K1_BASE),       1U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(0U)},
    {LH_HR_RELAY_ERROR_VALUE(LH_HR_RELAY_K1_BASE),      1U, HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(0U)},
    {LH_HR_RELAY_ALARM_SOURCE(LH_HR_RELAY_K1_BASE),     1U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(0U)},
    {LH_HR_RELAY_HH_ALARM_VALUE(LH_HR_RELAY_K1_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(0U)},
    {LH_HR_RELAY_H_ALARM_VALUE(LH_HR_RELAY_K1_BASE),    2U, HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(0U)},
    {LH_HR_RELAY_L_ALARM_VALUE(LH_HR_RELAY_K1_BASE),    2U, HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(0U)},
    {LH_HR_RELAY_LL_ALARM_VALUE(LH_HR_RELAY_K1_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(0U)},
    {LH_HR_RELAY_ALARM_HYSTERESIS(LH_HR_RELAY_K1_BASE), 2U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(0U)},
    {LH_HR_RELAY_DAMPING_FACTOR(LH_HR_RELAY_K1_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(0U)},

    {LH_HR_RELAY_OPERATING_MODE(LH_HR_RELAY_K2_BASE),   1U, HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(1U)},
    {LH_HR_RELAY_DIGITAL_SOURCE(LH_HR_RELAY_K2_BASE),   1U, HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(1U)},
    {LH_HR_RELAY_CONTACT_TYPE(LH_HR_RELAY_K2_BASE),     1U, HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(1U)},
    {LH_HR_RELAY_ALARM_MODE(LH_HR_RELAY_K2_BASE),       1U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(1U)},
    {LH_HR_RELAY_ERROR_VALUE(LH_HR_RELAY_K2_BASE),      1U, HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(1U)},
    {LH_HR_RELAY_ALARM_SOURCE(LH_HR_RELAY_K2_BASE),     1U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(1U)},
    {LH_HR_RELAY_HH_ALARM_VALUE(LH_HR_RELAY_K2_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(1U)},
    {LH_HR_RELAY_H_ALARM_VALUE(LH_HR_RELAY_K2_BASE),    2U, HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(1U)},
    {LH_HR_RELAY_L_ALARM_VALUE(LH_HR_RELAY_K2_BASE),    2U, HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(1U)},
    {LH_HR_RELAY_LL_ALARM_VALUE(LH_HR_RELAY_K2_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(1U)},
    {LH_HR_RELAY_ALARM_HYSTERESIS(LH_HR_RELAY_K2_BASE), 2U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(1U)},
    {LH_HR_RELAY_DAMPING_FACTOR(LH_HR_RELAY_K2_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(1U)},

    {LH_HR_RELAY_OPERATING_MODE(LH_HR_RELAY_K3_BASE),   1U, HOLDREGISTER_DEVICEPARAM_RELAY_OPERATING_MODE(2U)},
    {LH_HR_RELAY_DIGITAL_SOURCE(LH_HR_RELAY_K3_BASE),   1U, HOLDREGISTER_DEVICEPARAM_RELAY_DIGITAL_SOURCE(2U)},
    {LH_HR_RELAY_CONTACT_TYPE(LH_HR_RELAY_K3_BASE),     1U, HOLDREGISTER_DEVICEPARAM_RELAY_CONTACT_TYPE(2U)},
    {LH_HR_RELAY_ALARM_MODE(LH_HR_RELAY_K3_BASE),       1U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_MODE(2U)},
    {LH_HR_RELAY_ERROR_VALUE(LH_HR_RELAY_K3_BASE),      1U, HOLDREGISTER_DEVICEPARAM_RELAY_ERROR_VALUE(2U)},
    {LH_HR_RELAY_ALARM_SOURCE(LH_HR_RELAY_K3_BASE),     1U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_SOURCE(2U)},
    {LH_HR_RELAY_HH_ALARM_VALUE(LH_HR_RELAY_K3_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_HH_ALARM_VALUE(2U)},
    {LH_HR_RELAY_H_ALARM_VALUE(LH_HR_RELAY_K3_BASE),    2U, HOLDREGISTER_DEVICEPARAM_RELAY_H_ALARM_VALUE(2U)},
    {LH_HR_RELAY_L_ALARM_VALUE(LH_HR_RELAY_K3_BASE),    2U, HOLDREGISTER_DEVICEPARAM_RELAY_L_ALARM_VALUE(2U)},
    {LH_HR_RELAY_LL_ALARM_VALUE(LH_HR_RELAY_K3_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_LL_ALARM_VALUE(2U)},
    {LH_HR_RELAY_ALARM_HYSTERESIS(LH_HR_RELAY_K3_BASE), 2U, HOLDREGISTER_DEVICEPARAM_RELAY_ALARM_HYSTERESIS(2U)},
    {LH_HR_RELAY_DAMPING_FACTOR(LH_HR_RELAY_K3_BASE),   2U, HOLDREGISTER_DEVICEPARAM_RELAY_DAMPING_FACTOR(2U)}
};

/* 线圈 ON 写入与 CPU2 命令一一对应；OFF 写入不触发业务动作。 */
static const CommandType s_lh_coil_commands[LH_COIL_COUNT] = {
    [LH_COIL_BACK_ZERO] = CMD_BACK_ZERO,
    [LH_COIL_FIND_OIL] = CMD_FIND_OIL,
    [LH_COIL_FIND_WATER] = CMD_FIND_WATER,
    [LH_COIL_FIND_BOTTOM] = CMD_FIND_BOTTOM,
    [LH_COIL_FOLLOW_WATER] = CMD_FOLLOW_WATER,
    [LH_COIL_CANCEL_MEASUREMENT] = CMD_CANCEL_MEASUREMENT,
    [LH_COIL_MAINTENANCE_ENTER] = CMD_MAINTENANCE_MODE,
    [LH_COIL_MAINTENANCE_EXIT] = CMD_MAINTENANCE_EXIT,
    [LH_COIL_CLEAR_ALL_RELAY_LATCHED] = CMD_CLEAR_ALL_RELAY_LATCHED_ALARMS,
    [LH_COIL_CALIBRATE_OIL] = CMD_CALIBRATE_OIL,
    [LH_COIL_READ_PART_PARAMS] = CMD_READ_PART_PARAMS,
    [LH_COIL_MOVE_UP] = CMD_MOVE_UP,
    [LH_COIL_MOVE_DOWN] = CMD_MOVE_DOWN,
    [LH_COIL_CORRECT_OIL] = CMD_CORRECT_OIL,
    [LH_COIL_CALIBRATE_ZERO] = CMD_CALIBRATE_ZERO,
    [LH_COIL_CALIBRATE_WATER] = CMD_CALIBRATE_WATER,
    [LH_COIL_CALIBRATE_TANKHEIGHT] = CMD_CALIBRATE_TANKHEIGHT
};

/**
 * @brief 读取 LH 协议使用的 CPU3 拨码从站地址，并对非法值回退为地址 1。
 *
 * @return 返回按当前映射得到的 LH 协议使用的 CPU3 拨码从站地址，并对非法值回退为地址 1；非法输入使用 @brief 说明的兜底地址或无效值。
 */
static uint8_t lh_get_slave_address(void)
{
    if ((SlaveAddress >= 1) && (SlaveAddress <= 247)) {
        return (uint8_t)SlaveAddress;
    }
    return 1U;
}

/**
 * @brief 从 Modbus PDU 读取高字节在前的 16 位值。
 *
 * @param data 至少包含两个字节的 Modbus 大端字段；读取函数组合为 uint16_t，写入函数把 uint16_t 拆成高、低字节。
 * @return 返回从连续 2 字节 Modbus 大端数据还原的 16 位无符号值。
 */
static uint16_t lh_be16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

/**
 * @brief 把 16 位寄存器值按 Modbus 大端顺序写入响应。
 *
 * @param data 至少包含两个字节的 Modbus 大端字段；读取函数组合为 uint16_t，写入函数把 uint16_t 拆成高、低字节。
 * @param value 待写入协议缓冲区或寄存器的16 位数值。
 */
static void lh_write_be16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)(value & 0xFFU);
}

/**
 * @brief 把 32 位值写入连续两个 LH 寄存器，高字在前。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 * @param start 本次连续处理范围的起始索引。该值是目标寄存器数组中的零基下标，32 位值的高低字依次写入 start 和 start + 1。
 * @param value 待编码写入线格式的 32 位数值。
 */
static void lh_write_u32_to_regs(uint16_t *regs, uint16_t start, uint32_t value)
{
    regs[start] = (uint16_t)(value >> 16);
    regs[start + 1U] = (uint16_t)(value & 0xFFFFU);
}

/**
 * @brief 为当前响应追加低字节在前的 Modbus RTU CRC。
 *
 * @param tx 已包含 payload_len 字节 LH 响应载荷的输入输出缓冲区；末尾必须预留 2 字节写入 CRC16。
 * @param payload_len 协议载荷有效长度，单位字节。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 */
static void lh_append_crc(uint8_t *tx, uint16_t payload_len, uint16_t *tx_len)
{
    uint16_t crc = CRC16_Calculate(tx, payload_len);
    tx[payload_len] = (uint8_t)(crc & 0xFFU);
    tx[payload_len + 1U] = (uint8_t)(crc >> 8);
    *tx_len = (uint16_t)(payload_len + 2U);
}

/**
 * @brief 构造标准 Modbus 异常响应。
 *
 * @param address 合法 LH Modbus 请求帧中的从站地址字节；正常响应和异常响应均原样回显该地址。
 * @param function 满足当前流程条件时调用的处理函数。
 * @param exception 异常码。
 * @param tx LH 标准 Modbus 异常响应输出缓冲区，调用方至少提供 5 个可写字节。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 */
static void lh_build_exception(uint8_t address,
                               uint8_t function,
                               uint8_t exception,
                               uint8_t *tx,
                               uint16_t *tx_len)
{
    tx[0] = address;
    tx[1] = (uint8_t)(function | 0x80U);
    tx[2] = exception;
    lh_append_crc(tx, 3U, tx_len);
}

/**
 * @brief 兼容 LH 主机的 FC01 读线圈请求。
 *
 * @details 调用场景：主机读取 LH 连续命令线圈区时调用。
 * @note 关键约束：线圈仅作为命令写入口，合法读请求统一返回 0，不回显历史命令或运行状态。
 *
 * @param address 合法 LH Modbus 请求帧中的从站地址字节；正常响应和异常响应均原样回显该地址。
 * @param start 本次连续处理范围的起始索引。该值在 LH 请求处理中表示起始线圈或寄存器地址；写入本地数组时表示首个寄存器下标。
 * @param count 参与本次处理的数据项数量。
 * @param tx LH FC01 正常数据或异常响应的输出缓冲区；函数同步更新 tx_len。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 */
static void lh_handle_read_coils(uint8_t address,
                                 uint16_t start,
                                 uint16_t count,
                                 uint8_t *tx,
                                 uint16_t *tx_len)
{
    uint16_t byte_count;

    if ((count == 0U) || (count > LH_MODBUS_MAX_READ_COILS)) {
        lh_build_exception(address, LH_MODBUS_FUNC_READ_COILS,
                           LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
        return;
    }
    if (((uint32_t)start + (uint32_t)count) > LH_COIL_COUNT) {
        lh_build_exception(address, LH_MODBUS_FUNC_READ_COILS,
                           LH_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
        return;
    }

    byte_count = (uint16_t)((count + 7U) / 8U);
    tx[0] = address;
    tx[1] = LH_MODBUS_FUNC_READ_COILS;
    tx[2] = (uint8_t)byte_count;
    memset(&tx[3], 0, byte_count);
    lh_append_crc(tx, (uint16_t)(3U + byte_count), tx_len);
}

/**
 * @brief 把 CPU2 内部设备状态投影为 LH 手册定义的现场状态码。
 *
 * @details 调用场景：生成 LH 输入寄存器中的设备状态。
 * @note 关键约束：只发布 LH 手册明确支持的状态；其它协议或屏幕触发的内部状态
 *           统一投影为待机，避免在 LH 侧与既有状态码产生歧义。
 *
 * @param state CPU2 内部 DeviceState 状态码；函数将其投影为 LH 手册规定的现场状态值。
 * @return 返回 LH 手册状态码；已知密度阶段按 LH 固定码映射，其他可用状态保留内部枚举或回退待机。
 */
static uint16_t lh_translate_device_state(DeviceState state)
{
    switch (state) {
    case STATE_STANDBY:
    case STATE_INIT:
    case STATE_BACKZEROING:
    case STATE_FINDZEROING:
    case STATE_CALIBRATIONOILING:
    case STATE_READPARAMETERING:
    case STATE_RUNUPING:
    case STATE_RUNDOWNING:
    case STATE_FINDOIL:
    case STATE_FINDWATER:
    case STATE_FINDBOTTOM:
    case STATE_FOLLOW_WATER_POINT_SEARCHING:
    case STATE_FINDZEROOVER:
    case STATE_FINDOILOVER:
    case STATE_READPARAMETEROVER:
    case STATE_RUNUPOVER:
    case STATE_RUNDOWNOVER:
    case STATE_FLOWOIL:
    case STATE_FINDWATER_OVER:
    case STATE_FINDBOTTOM_OVER:
    case STATE_FOLLOW_WATERING:
    case STATE_ERROR:
        return (uint16_t)state;
    case STATE_CALIBRATE_WATERING:
        return 0x0026U;
    case STATE_CALIBRATE_WATER_OVER:
        return 0x8026U;
    case STATE_CALIBRATE_TANKHEIGHTING:
        return 0x0028U;
    case STATE_CALIBRATE_TANKHEIGHT_OVER:
        return 0x8028U;
    default:
        return (uint16_t)STATE_STANDBY;
    }
}

/**
 * @brief 从 CPU2 已确认运行快照生成完整 LH 输入寄存器表。
 *
 * @details 调用场景：LH FC04 地址和数量校验通过后调用。
 * @note 关键约束：继电器状态使用维护屏蔽后的逻辑动作位，不使用物理触点驱动电平。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 */
static void lh_build_input_registers(uint16_t *regs)
{
    uint32_t relay_mask = g_measurement.device_status.relay_alarm_action_mask;

    memset(regs, 0, LH_INPUT_REGISTER_COUNT * sizeof(regs[0]));
    regs[LH_IR_DEVICE_STATE] = lh_translate_device_state(g_measurement.device_status.device_state);
    lh_write_u32_to_regs(regs, LH_IR_ERROR_CODE, g_measurement.device_status.error_code);
    lh_write_u32_to_regs(regs, LH_IR_OIL_LEVEL, g_measurement.oil_measurement.oil_level);
    lh_write_u32_to_regs(regs, LH_IR_WATER_LEVEL, g_measurement.water_measurement.water_level);
    lh_write_u32_to_regs(regs, LH_IR_TANK_BOTTOM_HEIGHT,
                         g_measurement.height_measurement.current_real_height);
    lh_write_u32_to_regs(regs, LH_IR_REALTIME_POSITION,
                         (uint32_t)g_measurement.debug_data.sensor_position);
    regs[LH_IR_RELAY_K1_ACTION] = (uint16_t)((relay_mask >> 0U) & 1U);
    regs[LH_IR_RELAY_K2_ACTION] = (uint16_t)((relay_mask >> 1U) & 1U);
    regs[LH_IR_RELAY_K3_ACTION] = (uint16_t)((relay_mask >> 2U) & 1U);
    regs[LH_IR_MAINTENANCE_ACTIVE] =
        (g_measurement.device_status.maintenance_mode_active != 0U) ? 1U : 0U;
    regs[LH_IR_ANGLE_X] =
        (uint16_t)((uint32_t)g_measurement.debug_data.angle_x + 0x8000U);
    regs[LH_IR_ANGLE_Y] = (uint16_t)g_measurement.debug_data.angle_y;
    lh_write_u32_to_regs(regs, LH_IR_FREQUENCY, g_measurement.debug_data.frequency);
    regs[LH_IR_TEMPERATURE] = (uint16_t)g_measurement.debug_data.temperature;
    regs[LH_IR_AMPLITUDE] = (uint16_t)g_measurement.debug_data.current_amplitude;
    lh_write_u32_to_regs(regs, LH_IR_WATER_CAPACITANCE_X10,
                         g_measurement.debug_data.water_capacitance_x10);
    regs[LH_IR_MOTOR_STATE] = (uint16_t)g_measurement.debug_data.motor_state;
    lh_write_u32_to_regs(regs, LH_IR_MOTOR_SPEED,
                         g_measurement.debug_data.motor_speed);
    lh_write_u32_to_regs(regs, LH_IR_CURRENT_TORQUE,
                         g_measurement.debug_data.current_weight);
}

/**
 * @brief 把一路继电器参数快照投影到 LH 连续保持寄存器。
 *
 * @details 调用场景：LH FC03 组建 K1、K2、K3 参数块时调用。
 * @note 关键约束：枚举压缩为一个 LH 寄存器，Float32 原始位和 UInt32 均保持高字在前。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 * @param base 基址。
 * @param config 来自共享设备参数区的只读单路继电器报警配置；读取时保持 volatile 语义，字段包含模式、报警源、阈值、滞回、阻尼和锁存清除命令。
 */
static void lh_write_relay_holding_registers(uint16_t *regs,
                                             uint16_t base,
                                             const volatile RelayAlarmConfig *config)
{
    regs[LH_HR_RELAY_OPERATING_MODE(base)] = (uint16_t)config->operating_mode;
    regs[LH_HR_RELAY_DIGITAL_SOURCE(base)] = (uint16_t)config->digital_source;
    regs[LH_HR_RELAY_CONTACT_TYPE(base)] = (uint16_t)config->contact_type;
    regs[LH_HR_RELAY_ALARM_MODE(base)] = (uint16_t)config->alarm_mode;
    regs[LH_HR_RELAY_ERROR_VALUE(base)] = (uint16_t)config->error_value;
    regs[LH_HR_RELAY_ALARM_SOURCE(base)] = (uint16_t)config->alarm_source;
    lh_write_u32_to_regs(regs, LH_HR_RELAY_HH_ALARM_VALUE(base),
                         config->HH_alarm_value);
    lh_write_u32_to_regs(regs, LH_HR_RELAY_H_ALARM_VALUE(base),
                         config->H_alarm_value);
    lh_write_u32_to_regs(regs, LH_HR_RELAY_L_ALARM_VALUE(base),
                         config->L_alarm_value);
    lh_write_u32_to_regs(regs, LH_HR_RELAY_LL_ALARM_VALUE(base),
                         config->LL_alarm_value);
    lh_write_u32_to_regs(regs, LH_HR_RELAY_ALARM_HYSTERESIS(base),
                         config->alarm_hysteresis);
    lh_write_u32_to_regs(regs, LH_HR_RELAY_DAMPING_FACTOR(base),
                         config->damping_factor);
}

/**
 * @brief 从 CPU2 已确认参数快照生成完整 LH 保持寄存器表。
 *
 * @details 调用场景：LH FC03 地址和数量校验通过后调用。
 * @note 关键约束：只投影 LH 手册列出的字段，不发布 AO 隐藏位、仿真项、K4 或清报警瞬时位。
 *
 * @param regs 连续寄存器值缓冲区。函数按职责向 regs[] 写入协议镜像，或从可写镜像中读取并更新指定字段。
 */
static void lh_build_holding_registers(uint16_t *regs)
{
    memset(regs, 0, LH_HOLDING_REGISTER_COUNT * sizeof(regs[0]));
    lh_write_u32_to_regs(regs, LH_HR_TANK_HEIGHT, g_deviceParams.tankHeight);
    lh_write_u32_to_regs(regs, LH_HR_CALIBRATE_OIL_LEVEL, g_deviceParams.calibrateOilLevel);
    lh_write_u32_to_regs(regs, LH_HR_CALIBRATE_WATER_LEVEL, g_deviceParams.calibrateWaterLevel);
    lh_write_u32_to_regs(regs, LH_HR_CALIBRATE_TANK_HEIGHT, g_deviceParams.calibrateTankHeight);
    lh_write_u32_to_regs(regs, LH_HR_MOTOR_COMMAND_DISTANCE, g_deviceParams.motorCommandDistance);
    regs[LH_HR_MOTOR_CURRENT] = (uint16_t)g_deviceParams.motor_current;
    lh_write_u32_to_regs(regs, LH_HR_MAX_MOTOR_SPEED, g_deviceParams.max_motor_speed);
    lh_write_u32_to_regs(regs, LH_HR_BLIND_ZONE, g_deviceParams.blindZone);
    lh_write_u32_to_regs(regs, LH_HR_ENCODER_WHEEL_CIRCUMFERENCE,
                         g_deviceParams.encoder_wheel_circumference_mm);
    regs[LH_HR_LIQUID_TO_WATER_PROBE_DISTANCE] =
        (uint16_t)g_deviceParams.liquid_sensor_distance_diff;
    regs[LH_HR_MAX_DOWN_DISTANCE] = (uint16_t)g_deviceParams.maxDownDistance;
    lh_write_u32_to_regs(regs, LH_HR_SENSOR_ID, g_deviceParams.sensorID);

    regs[LH_HR_AO_WORK_MODE] = (uint16_t)g_deviceParams.ao_output.work_mode;
    regs[LH_HR_AO_CURRENT_MODE] = (uint16_t)g_deviceParams.ao_output.current_mode;
    regs[LH_HR_AO_OUTPUT_SOURCE] = (uint16_t)g_deviceParams.ao_output.output_source;
    lh_write_u32_to_regs(regs, LH_HR_AO_CURRENT_CORRECTION_MA_X1000,
                         (uint32_t)g_deviceParams.ao_output.current_correction_mA_x1000);
    lh_write_u32_to_regs(regs, LH_HR_AO_FIXED_CURRENT_MA_X100,
                         g_deviceParams.ao_output.fixed_current_mA_x100);
    lh_write_u32_to_regs(regs, LH_HR_AO_RANGE_0_01MM,
                         (uint32_t)g_deviceParams.ao_output.range_0_01mm);
    lh_write_u32_to_regs(regs, LH_HR_AO_RANGE_100_01MM,
                         (uint32_t)g_deviceParams.ao_output.range_100_01mm);
    lh_write_u32_to_regs(regs, LH_HR_AO_DAMPING_X10_S,
                         g_deviceParams.ao_output.damping_x10_s);
    regs[LH_HR_AO_FAULT_MODE] = (uint16_t)g_deviceParams.ao_output.fault_mode;
    lh_write_u32_to_regs(regs, LH_HR_AO_FAULT_CURRENT_MA_X100,
                         g_deviceParams.ao_output.fault_current_mA_x100);
    lh_write_u32_to_regs(regs, LH_HR_AO_POWER_ON_CURRENT_MA_X100,
                         g_deviceParams.ao_output.power_on_current_mA_x100);

    lh_write_relay_holding_registers(regs, LH_HR_RELAY_K1_BASE,
                                     &g_deviceParams.relayAlarm[0U]);
    lh_write_relay_holding_registers(regs, LH_HR_RELAY_K2_BASE,
                                     &g_deviceParams.relayAlarm[1U]);
    lh_write_relay_holding_registers(regs, LH_HR_RELAY_K3_BASE,
                                     &g_deviceParams.relayAlarm[2U]);
}

/**
 * @brief 查找一个与 FC10 起始地址和数量完全一致的 LH 可写字段。
 *
 * @param start 本次连续处理范围的起始索引。该值在 LH 请求处理中表示起始线圈或寄存器地址；写入本地数组时表示首个寄存器下标。
 * @param count 参与本次处理的数据项数量。
 * @return 成功时返回指向查找一个与 FC10 起始地址和数量完全一致的 LH 可写字段的指针；输入非法或未找到匹配项时返回 NULL。
 */
static const LhWritableField *lh_find_writable_field(uint16_t start, uint16_t count)
{
    uint32_t i;

    for (i = 0U; i < (uint32_t)(sizeof(s_lh_writable_fields) /
                                sizeof(s_lh_writable_fields[0])); i++) {
        if ((s_lh_writable_fields[i].lh_start == start) &&
            (s_lh_writable_fields[i].lh_count == count)) {
            return &s_lh_writable_fields[i];
        }
    }
    return NULL;
}

/**
 * @brief 下发一个 LH 线圈对应的 CPU2 命令并等待 ACK。
 *
 * @details 调用场景：FC05 收到标准 ON 值后调用。
 * @note 关键约束：不得先更新本地命令影子；CPU2 标准异常原样返回，断链按设备故障返回。
 *
 * @param command LH 线圈地址已映射得到的 CPU2 CommandType 命令；函数负责下发并等待板间 ACK。
 * @return 返回 CPU2 写命令结果；CPU2_MODBUS_RESULT_OK 表示已获确认，LH_MODBUS_EX_SLAVE_DEVICE_BUSY 表示链路不可用或设备忙。
 */
static uint8_t lh_send_command(CommandType command)
{
    uint32_t command_value = (uint32_t)command;
    uint16_t command_regs[REG_STRIDE];

    if (!CPU2_CommCanSendCommand(command)) {
        return LH_MODBUS_EX_SLAVE_DEVICE_BUSY;
    }
    command_regs[0] = (uint16_t)(command_value >> 16);
    command_regs[1] = (uint16_t)(command_value & 0xFFFFU);
    return CPU2_CommWriteHoldingRegistersEx(HOLDREGISTER_DEVICEPARAM_COMMAND,
                                            REG_STRIDE,
                                            command_regs);
}

/**
 * @brief 处理 FC03/FC04 读取，快照无效时整帧返回设备忙。
 *
 * @param address 合法 LH Modbus 请求帧中的从站地址字节；正常响应和异常响应均原样回显该地址。
 * @param function 满足当前流程条件时调用的处理函数。
 * @param start 本次连续处理范围的起始索引。该值在 LH 请求处理中表示起始线圈或寄存器地址；写入本地数组时表示首个寄存器下标。
 * @param count 参与本次处理的数据项数量。
 * @param tx LH FC03/FC04 正常寄存器数据或异常响应的输出缓冲区；函数同步更新 tx_len。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 */
static void lh_handle_read(uint8_t address,
                           uint8_t function,
                           uint16_t start,
                           uint16_t count,
                           uint8_t *tx,
                           uint16_t *tx_len)
{
    uint16_t registers[LH_HOLDING_REGISTER_COUNT];
    uint16_t register_count;
    uint16_t i;
    uint32_t read_end;

    if ((count == 0U) || (count > LH_MODBUS_MAX_READ_REGISTERS)) {
        lh_build_exception(address, function, LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
        return;
    }

    if (function == LH_MODBUS_FUNC_READ_HOLDING_REGS) {
        register_count = LH_HOLDING_REGISTER_COUNT;
        if (((uint32_t)start + (uint32_t)count > register_count)) {
            lh_build_exception(address, function, LH_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
            return;
        }
        if (!CPU2_CommIsAvailable()) {
            lh_build_exception(address, function, LH_MODBUS_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
            return;
        }
        read_end = (uint32_t)start + (uint32_t)count;
        if (((uint32_t)start <= (uint32_t)LH_HR_LIQUID_TO_WATER_PROBE_DISTANCE) &&
            (read_end > (uint32_t)LH_HR_LIQUID_TO_WATER_PROBE_DISTANCE) &&
            (g_deviceParams.liquid_sensor_distance_diff > (uint32_t)UINT16_MAX)) {
            /* LH 将该字段定义为 UInt16，内部负值或超范围值不得截断后发布。 */
            lh_build_exception(address, function, LH_MODBUS_EX_SLAVE_DEVICE_FAILURE, tx, tx_len);
            return;
        }
        lh_build_holding_registers(registers);
    } else {
        register_count = LH_INPUT_REGISTER_COUNT;
        if (((uint32_t)start + (uint32_t)count > register_count)) {
            lh_build_exception(address, function, LH_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
            return;
        }
        if (!CPU2_CommHasRuntimeSnapshot()) {
            lh_build_exception(address, function, LH_MODBUS_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
            return;
        }
        lh_build_input_registers(registers);
    }

    tx[0] = address;
    tx[1] = function;
    tx[2] = (uint8_t)(count * 2U);
    for (i = 0U; i < count; i++) {
        lh_write_be16(&tx[3U + (i * 2U)], registers[start + i]);
    }
    lh_append_crc(tx, (uint16_t)(3U + (count * 2U)), tx_len);
}

/**
 * @brief 处理 FC05；标准 OFF 值只回显，不向 CPU2 下发任何命令。
 *
 * @param address 合法 LH Modbus 请求帧中的从站地址字节；正常响应和异常响应均原样回显该地址。
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param tx LH FC05 标准回显或异常响应的输出缓冲区；函数同步更新 tx_len。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 */
static void lh_handle_write_single_coil(uint8_t address,
                                        const uint8_t *rx,
                                        uint8_t *tx,
                                        uint16_t *tx_len)
{
    uint16_t offset = lh_be16(&rx[2]);
    uint16_t value = lh_be16(&rx[4]);
    uint8_t command_exception;

    if (offset >= LH_COIL_COUNT) {
        lh_build_exception(address, LH_MODBUS_FUNC_WRITE_SINGLE_COIL,
                           LH_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
        return;
    }
    if ((value != 0x0000U) && (value != 0xFF00U)) {
        lh_build_exception(address, LH_MODBUS_FUNC_WRITE_SINGLE_COIL,
                           LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
        return;
    }
    if (value == 0xFF00U) {
        command_exception = lh_send_command(s_lh_coil_commands[offset]);
        if (command_exception != CPU2_MODBUS_RESULT_OK) {
            lh_build_exception(address, LH_MODBUS_FUNC_WRITE_SINGLE_COIL,
                               command_exception, tx, tx_len);
            return;
        }
    }

    memcpy(tx, rx, 6U);
    lh_append_crc(tx, 6U, tx_len);
}

/**
 * @brief 把一个完整 LH 参数字段转换为共享 32 位寄存器值。
 *
 * @details 调用场景：FC10 已通过帧长度、字段边界和运行状态预检查后调用。
 * @note 关键约束：LH 单寄存器字段只写共享字段低 16 位，高 16 位固定为 0。
 *
 * @param field 待查询或显示的字段枚举值。该指针指向已经按 LH 保持寄存器地址匹配的可写字段元数据，包含地址、宽度和写入策略。
 * @param wire_data 当前 LH 字段在 FC10 数据区中的大端字节序列，长度由 field->lh_count 决定。
 * @param shared_regs 用于接收转换后 CPU2 共享高、低 16 位寄存器值的两元素数组。
 */
static void lh_decode_holding_field(const LhWritableField *field,
                                    const uint8_t *wire_data,
                                    uint16_t *shared_regs)
{
    if (field->lh_count == 1U) {
        shared_regs[0] = 0U;
        shared_regs[1] = lh_be16(wire_data);
    } else {
        shared_regs[0] = lh_be16(wire_data);
        shared_regs[1] = lh_be16(&wire_data[2]);
    }
}

/**
 * @brief 将两个 LH 16 位寄存器按高字在前顺序合并为 32 位无符号值。
 *
 * @param regs 连续寄存器值缓冲区。函数按既定寄存器数量只读 regs[]，并按高低字、字段偏移或协议映射解析业务值。
 * @return 返回由 regs[0] 作为高 16 位、regs[1] 作为低 16 位合并得到的 uint32_t 值。
 */
static uint32_t lh_regs_to_u32(const uint16_t *regs)
{
    return ((uint32_t)regs[0] << 16) | (uint32_t)regs[1];
}

/**
 * @brief 判断 32 位原始浮点字是否表示有限数。
 *
 * @param raw LH 保持寄存器携带的 IEEE 754 Float32 原始 32 位位模式。
 * @return true 表示 IEEE 754 单精度原始字的指数位不是全 1，数值为有限数；false 表示指数位全 1，原始字表示正负无穷或 NaN。
 */
static bool lh_float_raw_is_finite(uint32_t raw)
{
    return (raw & 0x7F800000U) != 0x7F800000U;
}

/**
 * @brief 判断 32 位原始浮点字是否为非负有限数。
 *
 * @param raw LH 保持寄存器携带的 IEEE 754 Float32 原始 32 位位模式；函数同时检查有限性和符号。
 * @return true 表示 32 位原始浮点字为非负有限数；false 表示 32 位原始浮点字不是非负有限数。
 */
static bool lh_float_raw_is_nonnegative(uint32_t raw)
{
    return lh_float_raw_is_finite(raw) &&
           (((raw & 0x80000000U) == 0U) || ((raw & 0x7FFFFFFFU) == 0U));
}

/**
 * @brief 按 AO 输出源选择油罐高或水罐高作为 0.1 mm 量程上限。
 *
 * @return 返回 AO 当前过程量来源对应的满量程上限，单位 0.1 mm。
 */
static int32_t lh_ao_range_max_01mm(void)
{
    uint32_t max_01mm;

    if (g_deviceParams.ao_output.output_source == AO_PROCESS_SOURCE_WATER_LEVEL) {
        max_01mm = g_deviceParams.water_tank_height;
        if (max_01mm == 0U) {
            max_01mm = g_deviceParams.tankHeight;
        }
    } else {
        max_01mm = g_deviceParams.tankHeight;
    }

    if (max_01mm == 0U) {
        max_01mm = 1U;
    }
    if (max_01mm > 0x7FFFFFFFU) {
        max_01mm = 0x7FFFFFFFU;
    }
    return (int32_t)max_01mm;
}

/**
 * @brief 按继电器字段校验枚举范围、浮点有限性和非负约束。
 *
 * @param start 本次连续处理范围的起始索引。该值在 LH 请求处理中表示起始线圈或寄存器地址；写入本地数组时表示首个寄存器下标。
 * @param raw 从两个 LH 保持寄存器拼成的 32 位字段原始值；解释方式由 start 对应的继电器字段决定。
 * @return true 表示指定继电器字段的枚举、有限浮点、非负数或保留零值约束已经满足；false 表示字段地址未知，或 raw 超出该字段允许的协议值域。
 */
static bool lh_relay_field_value_is_valid(uint16_t start, uint32_t raw)
{
    uint16_t offset;

    if ((start < LH_HR_RELAY_K1_BASE) || (start >= LH_HOLDING_REGISTER_COUNT)) {
        return false;
    }
    offset = (uint16_t)((start - LH_HR_RELAY_K1_BASE) %
                        LH_HR_RELAY_CHANNEL_REG_COUNT);

    switch (offset) {
    case 0x0000U:
        return raw <= RELAY_ALARM_OPERATING_OUTPUT_PASSIVE;
    case 0x0001U:
        return raw <= RELAY_ALARM_DIGITAL_ANY;
    case 0x0002U:
        return raw <= RELAY_ALARM_CONTACT_NORMALLY_CLOSED;
    case 0x0003U:
        return raw <= RELAY_ALARM_MODE_LATCHING;
    case 0x0004U:
        return raw <= RELAY_ALARM_ERROR_ALL_ALARMS;
    case 0x0005U:
        return raw <= RELAY_ALARM_SOURCE_NONE;
    case 0x0006U:
    case 0x0008U:
    case 0x000AU:
    case 0x000CU:
        return lh_float_raw_is_finite(raw);
    case 0x000EU:
        return lh_float_raw_is_nonnegative(raw);
    case 0x0010U:
        return raw == 0U;
    default:
        return false;
    }
}

/**
 * @brief 按 LH 手册校验一个完整字段的类型和值域。
 *
 * @details 调用场景：FC10 已确认字段边界、CPU2可用且通过运行状态预检查后调用。
 * @note 关键约束：只做 LH 对外预校验，CPU2 仍负责最终业务校验、运行态提交和持久化。
 *
 * @param field 待查询或显示的字段枚举值。该指针指向已经按 LH 保持寄存器地址匹配的可写字段元数据，包含地址、宽度和写入策略。
 * @param shared_regs 已经按 CPU2 共享高、低 16 位顺序排列的只读两元素数组。
 * @return true 表示字段已通过继电器规则或其专属范围、符号、有限性及交叉量程约束；false 表示字段未知，或数值越界、非有限、符号非法，或 0%/100% 量程与另一端相等。
 */
static bool lh_holding_field_value_is_valid(const LhWritableField *field,
                                            const uint16_t *shared_regs)
{
    uint32_t raw = lh_regs_to_u32(shared_regs);
    int32_t signed_value;
    int32_t range_max_01mm;

    memcpy(&signed_value, &raw, sizeof(signed_value));

    if (field->lh_start >= LH_HR_RELAY_K1_BASE) {
        return lh_relay_field_value_is_valid(field->lh_start, raw);
    }

    switch (field->lh_start) {
    case LH_HR_TANK_HEIGHT:
        return raw > g_deviceParams.blindZone;
    case LH_HR_CALIBRATE_OIL_LEVEL:
    case LH_HR_CALIBRATE_WATER_LEVEL:
    case LH_HR_CALIBRATE_TANK_HEIGHT:
        return true;
    case LH_HR_MOTOR_COMMAND_DISTANCE:
        return raw > 0U;
    case LH_HR_MOTOR_CURRENT:
        return (raw >= MOTOR_CURRENT_MIN) && (raw <= MOTOR_CURRENT_MAX);
    case LH_HR_MAX_MOTOR_SPEED:
        return (raw >= LH_MOTOR_SPEED_MIN_X100) &&
               (raw <= LH_MOTOR_SPEED_MAX_X100);
    case LH_HR_BLIND_ZONE:
        return (raw > 0U) && (raw < g_deviceParams.tankHeight);
    case LH_HR_ENCODER_WHEEL_CIRCUMFERENCE:
    case LH_HR_LIQUID_TO_WATER_PROBE_DISTANCE:
    case LH_HR_MAX_DOWN_DISTANCE:
        return raw > 0U;
    case LH_HR_AO_WORK_MODE:
        return raw <= AO_WORK_MODE_HART_SLAVE_OUTPUT;
    case LH_HR_AO_CURRENT_MODE:
        return raw <= AO_CURRENT_MODE_FIXED;
    case LH_HR_AO_OUTPUT_SOURCE:
        return raw <= AO_PROCESS_SOURCE_WATER_LEVEL;
    case LH_HR_AO_CURRENT_CORRECTION_MA_X1000:
        return (signed_value >= AO_CURRENT_CORRECTION_MIN_MA_X1000) &&
               (signed_value <= AO_CURRENT_CORRECTION_MAX_MA_X1000);
    case LH_HR_AO_FIXED_CURRENT_MA_X100:
        return (raw >= AO_FIXED_CURRENT_MIN_MA_X100) &&
               (raw <= AO_FIXED_CURRENT_MAX_MA_X100);
    case LH_HR_AO_RANGE_0_01MM:
        range_max_01mm = lh_ao_range_max_01mm();
        return (signed_value >= 0) &&
               (signed_value <= range_max_01mm) &&
               (signed_value != g_deviceParams.ao_output.range_100_01mm);
    case LH_HR_AO_RANGE_100_01MM:
        range_max_01mm = lh_ao_range_max_01mm();
        return (signed_value >= 0) &&
               (signed_value <= range_max_01mm) &&
               (signed_value != g_deviceParams.ao_output.range_0_01mm);
    case LH_HR_AO_DAMPING_X10_S:
        return raw <= AO_DAMPING_MAX_X10_S;
    case LH_HR_AO_FAULT_MODE:
        return raw <= AO_FAULT_ACTION_HOLD_LAST_VALID;
    case LH_HR_AO_FAULT_CURRENT_MA_X100:
        return (raw >= AO_FAULT_CURRENT_MIN_MA_X100) &&
               (raw <= AO_FAULT_CURRENT_MAX_MA_X100);
    case LH_HR_AO_POWER_ON_CURRENT_MA_X100:
        return (raw >= AO_INITIAL_CURRENT_MIN_MA_X100) &&
               (raw <= AO_INITIAL_CURRENT_MAX_MA_X100);
    default:
        return false;
    }
}

/**
 * @brief 同步读取一个共享 32 位参数字段，并只接受当前请求的合法 CPU2 响应。
 *
 * @param field 待查询或显示的字段枚举值。该指针指向已经按 LH 保持寄存器地址匹配的可写字段元数据，包含地址、宽度和写入策略。
 * @param shared_regs 用于接收 CPU2 返回的共享高、低 16 位寄存器值的两元素数组。
 * @return true 表示字段元数据有效，CPU2 定向补读成功且两个共享寄存器已复制；false 表示字段/输出指针无效、字段宽度不符，或 CPU2 补读失败。
 */
static bool lh_read_shared_holding_field(const LhWritableField *field,
                                         uint16_t *shared_regs)
{
    if (!CPU2_CombinatePackage_Send(FUNCTIONCODE_READ_HOLDREGISTER,
                                    field->shared_start,
                                    REG_STRIDE,
                                    NULL)) {
        return false;
    }
    shared_regs[0] = HoldingRegisterArray[field->shared_start];
    shared_regs[1] = HoldingRegisterArray[field->shared_start + 1U];
    return true;
}

/**
 * @brief 同步刷新 CPU2 状态块，供状态白名单和保存完成计数共同确认。
 *
 * @return true 表示 CPU2 状态块补读事务成功；false 表示板间通信不可用、Modbus 异常、响应校验失败或状态块未能刷新。
 */
static bool lh_refresh_cpu2_status(void)
{
    return CPU2_CombinatePackage_Send(
        FUNCTIONCODE_READ_INPUTREGISTER,
        REG_DEVICE_STATUS_WORK_MODE,
        (uint16_t)(REG_DEVICE_STATUS_BLOCK_END - REG_DEVICE_STATUS_WORK_MODE),
        NULL);
}

/**
 * @brief 比较 LH 映射区与共享寄存器区的内容是否一致。
 *
 * @param left 区间左端值或左侧比较对象。
 * @param right 区间右端值或右侧比较对象。
 * @return true 表示两组共享寄存器的高、低 16 位均相等；false 表示任一寄存器字不同。
 */
static bool lh_shared_regs_equal(const uint16_t *left, const uint16_t *right)
{
    return (left[0] == right[0]) && (left[1] == right[1]);
}

/**
 * @brief 等待 CPU2 参数完成持久化并回读确认目标字段。
 *
 * @details 调用场景：LH FC10 写入任一持久参数并收到 CPU2 即时 ACK 后调用。
 * @note 关键约束：不使用 HAL_Delay；每轮依靠一次同步状态事务推进，超时或回读不一致即失败。
 *
 * @param field 待查询或显示的字段枚举值。该指针指向已经按 LH 保持寄存器地址匹配的可写字段元数据，包含地址、宽度和写入策略。
 * @param previous_update_flag 上一次更新标志。
 * @param expected_regs CPU2 应完成持久化的两个共享保持寄存器期望值，用于更新代次变化后的回读核对。
 * @return true 表示参数保存完成计数已变化，且定向补读值与 expected_regs 一致；false 表示轮询次数或总时限到期、CPU2 状态/参数补读失败，或保存后的实际值与期望值不一致。
 */
static bool lh_wait_parameter_persisted(const LhWritableField *field,
                                        uint32_t previous_update_flag,
                                        const uint16_t *expected_regs)
{
    uint32_t start_tick = HAL_GetTick();
    uint32_t poll_count = 0U;
    uint16_t confirmed_regs[REG_STRIDE];

    while ((HAL_GetTick() - start_tick) <= LH_PARAMETER_CONFIRM_TIMEOUT_MS) {
        if (poll_count >= LH_PARAMETER_CONFIRM_MAX_POLLS) {
            return false;
        }
        poll_count++;

        if (!lh_refresh_cpu2_status()) {
            return false;
        }
        if (g_measurement.device_status.parameter_update_flag != previous_update_flag) {
            if (!lh_read_shared_holding_field(field, confirmed_regs)) {
                return false;
            }
            return lh_shared_regs_equal(confirmed_regs, expected_regs);
        }
    }
    return false;
}

/**
 * @brief 写入一个完整 LH 参数，并按字段类型选择最终成功条件。
 *
 * @details 调用场景：FC10 字段和帧格式校验通过后调用。
 * @note 关键约束：命令前置参数由公共写入口完成 ACK 加定向回读，不等待阻塞业务期间无法推进的
 *           FRAM 完成代次；普通参数仍等待保存完成计数变化后补读确认。
 *
 * @param field 待查询或显示的字段枚举值。该指针指向已经按 LH 保持寄存器地址匹配的可写字段元数据，包含地址、宽度和写入策略。
 * @param expected_regs 已经转换为共享线序的两个保持寄存器目标值，用于写入 CPU2 并确认最终生效值。
 * @return 返回 CPU2 参数写入结果或 LH Modbus 异常码；成功必须同时满足写 ACK 及字段约定的最终确认条件。
 */
static uint8_t lh_write_holding_field(const LhWritableField *field,
                                      const uint16_t *expected_regs)
{
    uint32_t previous_update_flag = 0U;
    uint8_t write_exception;
    bool command_argument_only;

    if (!lh_refresh_cpu2_status()) {
        return LH_MODBUS_EX_SLAVE_DEVICE_BUSY;
    }
    command_argument_only = LtdModbus_HoldingWriteIsCommandArgumentOnly(
        field->shared_start, REG_STRIDE);
    if (!command_argument_only &&
        !DeviceState_AllowsPersistentParamWrite(
            g_measurement.device_status.device_state)) {
        return LH_MODBUS_EX_SLAVE_DEVICE_BUSY;
    }
    if (!command_argument_only) {
        previous_update_flag =
            g_measurement.device_status.parameter_update_flag;
    }

    write_exception = CPU2_CommWriteHoldingRegistersEx(field->shared_start,
                                                       REG_STRIDE,
                                                       expected_regs);
    if (write_exception != CPU2_MODBUS_RESULT_OK) {
        return write_exception;
    }
    if (command_argument_only) {
        return CPU2_MODBUS_RESULT_OK;
    }

    return lh_wait_parameter_persisted(field, previous_update_flag, expected_regs) ?
           CPU2_MODBUS_RESULT_OK : LH_MODBUS_EX_SLAVE_DEVICE_FAILURE;
}

/**
 * @brief 处理 FC10；一次写一个完整LH字段，普通持久参数仍遵循状态白名单。
 *
 * @param address 合法 LH Modbus 请求帧中的从站地址字节；正常响应和异常响应均原样回显该地址。
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx LH FC10 写入确认或异常响应的输出缓冲区；函数同步更新 tx_len。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 */
static void lh_handle_write_holding(uint8_t address,
                                    const uint8_t *rx,
                                    uint16_t rx_len,
                                    uint8_t *tx,
                                    uint16_t *tx_len)
{
    uint16_t start = lh_be16(&rx[2]);
    uint16_t count = lh_be16(&rx[4]);
    uint8_t byte_count = rx[6];
    const LhWritableField *field;
    uint8_t write_exception;
    uint16_t expected_regs[REG_STRIDE];

    if ((count == 0U) || (count > LH_MODBUS_MAX_WRITE_REGISTERS) ||
        (byte_count != (uint8_t)(count * 2U)) ||
        (rx_len != (uint16_t)(9U + byte_count))) {
        lh_build_exception(address, LH_MODBUS_FUNC_WRITE_MULTI_REGS,
                           LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
        return;
    }

    if (((uint32_t)start + (uint32_t)count) > LH_HOLDING_REGISTER_COUNT) {
        lh_build_exception(address, LH_MODBUS_FUNC_WRITE_MULTI_REGS,
                           LH_MODBUS_EX_ILLEGAL_ADDRESS, tx, tx_len);
        return;
    }

    field = lh_find_writable_field(start, count);
    if (field == NULL) {
        /* 只读字段或不完整字段属于合法地址上的非法写值。 */
        lh_build_exception(address, LH_MODBUS_FUNC_WRITE_MULTI_REGS,
                           LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
        return;
    }
    if (!CPU2_CommIsAvailable() ||
        (!LtdModbus_HoldingWriteIsCommandArgumentOnly(
             field->shared_start, REG_STRIDE) &&
         !DeviceState_AllowsPersistentParamWrite(
             g_measurement.device_status.device_state))) {
        lh_build_exception(address, LH_MODBUS_FUNC_WRITE_MULTI_REGS,
                           LH_MODBUS_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
        return;
    }
    lh_decode_holding_field(field, &rx[7], expected_regs);
    if (!lh_holding_field_value_is_valid(field, expected_regs)) {
        lh_build_exception(address, LH_MODBUS_FUNC_WRITE_MULTI_REGS,
                           LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
        return;
    }
    write_exception = lh_write_holding_field(field, expected_regs);
    if (write_exception != CPU2_MODBUS_RESULT_OK) {
        lh_build_exception(address, LH_MODBUS_FUNC_WRITE_MULTI_REGS,
                           write_exception, tx, tx_len);
        return;
    }

    tx[0] = address;
    tx[1] = LH_MODBUS_FUNC_WRITE_MULTI_REGS;
    tx[2] = rx[2];
    tx[3] = rx[3];
    tx[4] = rx[4];
    tx[5] = rx[5];
    lh_append_crc(tx, 6U, tx_len);
}

/**
 * @brief 校验 LH Modbus 地址和 CRC，并分发支持的功能码或构造异常响应。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx 完整 LH Modbus RTU 正常或异常响应的输出缓冲区；调用方容量必须覆盖协议允许的最大响应。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 LH Modbus 分发结果；LH_MODBUS_OK 表示已完成处理，其他值区分帧长、CRC、从机地址和功能码错误；tx_len 非零时仍应发送已生成的异常响应。
 */
LhModbusResult lh_modbus_process(const uint8_t *rx,
                                 uint16_t rx_len,
                                 uint8_t *tx,
                                 uint16_t *tx_len)
{
    uint8_t address;
    uint8_t function;

    if ((rx == NULL) || (tx == NULL) || (tx_len == NULL)) {
        return LH_MODBUS_ERR_BAD_LENGTH;
    }
    *tx_len = 0U;

    if (rx_len < 4U) {
        return LH_MODBUS_ERR_BAD_LENGTH;
    }
    address = lh_get_slave_address();
    if (rx[0] != address) {
        return LH_MODBUS_ERR_ADDRESS_MISMATCH;
    }
    if (!SlaveCheckCRC(rx, (int)rx_len)) {
        return LH_MODBUS_ERR_CRC;
    }

    function = rx[1];
    switch (function) {
    case LH_MODBUS_FUNC_READ_COILS:
        if (rx_len != 8U) {
            lh_build_exception(address, function, LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
            return LH_MODBUS_OK;
        }
        lh_handle_read_coils(address, lh_be16(&rx[2]), lh_be16(&rx[4]), tx, tx_len);
        return LH_MODBUS_OK;

    case LH_MODBUS_FUNC_READ_HOLDING_REGS:
    case LH_MODBUS_FUNC_READ_INPUT_REGS:
        if (rx_len != 8U) {
            lh_build_exception(address, function, LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
            return LH_MODBUS_OK;
        }
        lh_handle_read(address, function, lh_be16(&rx[2]), lh_be16(&rx[4]), tx, tx_len);
        return LH_MODBUS_OK;

    case LH_MODBUS_FUNC_WRITE_SINGLE_COIL:
        if (rx_len != 8U) {
            lh_build_exception(address, function, LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
            return LH_MODBUS_OK;
        }
        lh_handle_write_single_coil(address, rx, tx, tx_len);
        return LH_MODBUS_OK;

    case LH_MODBUS_FUNC_WRITE_MULTI_REGS:
        if (rx_len < 9U) {
            lh_build_exception(address, function, LH_MODBUS_EX_ILLEGAL_VALUE, tx, tx_len);
            return LH_MODBUS_OK;
        }
        lh_handle_write_holding(address, rx, rx_len, tx, tx_len);
        return LH_MODBUS_OK;

    default:
        lh_build_exception(address, function, LH_MODBUS_EX_ILLEGAL_FUNCTION, tx, tx_len);
        return LH_MODBUS_ERR_UNSUPPORTED_FUNCTION;
    }
}

/**
 * @brief 已生成正常或异常响应帧时，外部 COM 分发层统一视为本帧处理完成。
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
                                        uint16_t *tx_len)
{
    LhModbusResult ret = lh_modbus_process(rx, rx_len, tx, tx_len);

    if ((tx_len != NULL) && (*tx_len > 0U)) {
        return 0U;
    }
    return (uint32_t)ret;
}

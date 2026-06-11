#include "si7000_modbus_slave.h"

#include <string.h>

#include "address.h"
#include "cpu2_communicate.h"
#include "cpu3_clock.h"
#include "main.h"
#include "my_crc.h"
#include "stateformodbus.h"
#include "system_parameter.h"

/*
 * SI7000 兼容层只在 CPU3 侧做协议转换：
 * - 对 PLC 暴露 SI7000 Modbus RTU 地址、功能码和缩放；
 * - 对内复用 CPU2 已有测量变量、命令和协议辅助状态；
 * - 未确认的 SI7000 参数先保存在影子寄存器，避免提前改动 CPU2 参数存储。
 */
#define SI7000_FUNC_READ_COILS             0x01U
#define SI7000_FUNC_READ_DISCRETE_INPUTS   0x02U
#define SI7000_FUNC_READ_HOLDING_REGS      0x03U
#define SI7000_FUNC_READ_INPUT_REGS        0x04U
#define SI7000_FUNC_WRITE_SINGLE_COIL      0x05U
#define SI7000_FUNC_WRITE_SINGLE_REG       0x06U

#define SI7000_EX_ILLEGAL_FUNCTION         0x01U
#define SI7000_EX_ILLEGAL_ADDRESS          0x02U
#define SI7000_EX_ILLEGAL_VALUE            0x03U

#define SI7000_INVALID_TEMP_RAW_CPU2       9999U

/* 线圈地址保持 SI7000 手册编号，数组下标即协议 offset。 */
enum {
    SI7000_COIL_MANUAL = 0,
    SI7000_COIL_CALIBRATE,
    SI7000_COIL_AUTO,
    SI7000_COIL_PROFILE,
    SI7000_COIL_STOP = 8,
    SI7000_COIL_UP_SLOW,
    SI7000_COIL_UP_MEDIUM,
    SI7000_COIL_UP_FAST,
    SI7000_COIL_DOWN_SLOW,
    SI7000_COIL_DOWN_MEDIUM,
    SI7000_COIL_DOWN_FAST
};

/* 离散输入同样按协议 offset 排列，预留洞保留原地址，不重新压缩。 */
enum {
    SI7000_DI_BOTTOM_REFERENCE = 0,
    SI7000_DI_LOWER_LEVEL_SENSOR,
    SI7000_DI_UPPER_LEVEL_SENSOR,
    SI7000_DI_INTERLOCK,
    SI7000_DI_PROFILE_COMPLETE,
    SI7000_DI_UNIT_IS_METRIC,
    SI7000_DI_REEL_ALARM = 8,
    SI7000_DI_PROBE_UNCALIBRATED,
    SI7000_DI_INTERVAL_TIMER = 11,
    SI7000_DI_PROBE_AT_LIQUID_LEVEL = 12,
    SI7000_DI_LOW_DENSITY_ALARM = 16,
    SI7000_DI_HIGH_DENSITY_ALARM,
    SI7000_DI_LOW_TEMP_ALARM,
    SI7000_DI_HIGH_TEMP_ALARM,
    SI7000_DI_LL_LEVEL_ALARM,
    SI7000_DI_HH_LEVEL_ALARM,
    SI7000_DI_LOW_LEVEL_ALARM,
    SI7000_DI_HIGH_LEVEL_ALARM,
    SI7000_DI_PROFILE_TEMP_DEVIATION_ALARM,
    SI7000_DI_PROFILE_DENSITY_DEVIATION_ALARM,
    SI7000_DI_PROFILE_LOW_TEMP_ALARM = 28,
    SI7000_DI_PROFILE_HIGH_TEMP_ALARM,
    SI7000_DI_PROFILE_LOW_DENSITY_ALARM,
    SI7000_DI_PROFILE_HIGH_DENSITY_ALARM
};

/* 输入寄存器：实时测量值、profile 时间戳、当前时间和 profile 点阵共用一张表。 */
enum {
    SI7000_IR_CURRENT_PROBE_POSITION = 0,
    SI7000_IR_CURRENT_TEMPERATURE,
    SI7000_IR_CURRENT_DENSITY,
    SI7000_IR_LIQUID_LEVEL,
    SI7000_IR_NUMBER_OF_POINTS = 5,
    SI7000_IR_PROFILE_TIMESTAMP_MONTH,
    SI7000_IR_PROFILE_TIMESTAMP_DAY,
    SI7000_IR_PROFILE_TIMESTAMP_HOUR,
    SI7000_IR_PROFILE_TIMESTAMP_MINUTE,
    SI7000_IR_CURRENT_TIME_HOUR,
    SI7000_IR_CURRENT_TIME_MINUTE,
    SI7000_IR_CURRENT_TIME_SECOND,
    SI7000_IR_PROFILE_POINT0_POSITION = 20
};

/* 保持寄存器：已落地的 profile 参数会写 CPU2，其余先作为 PLC 可回读影子值。 */
enum {
    SI7000_HR_PROFILE_FIRST_POINT = 0,
    SI7000_HR_PROFILE_INCREMENT,
    SI7000_HR_PROFILE_DWELL_TIME,
    SI7000_HR_AUTO_PROFILE_INTERVAL = 9,
    SI7000_HR_AUTO_PROFILE_ENABLE,
    SI7000_HR_AUTO_PROFILE_HOUR,
    SI7000_HR_AUTO_PROFILE_MINUTE,
    SI7000_HR_LOW_DENSITY_SETPOINT,
    SI7000_HR_HIGH_DENSITY_SETPOINT,
    SI7000_HR_LOW_TEMPERATURE_SETPOINT,
    SI7000_HR_HIGH_TEMPERATURE_SETPOINT,
    SI7000_HR_LL_LEVEL_SETPOINT,
    SI7000_HR_HH_LEVEL_SETPOINT,
    SI7000_HR_LOW_LEVEL_SETPOINT,
    SI7000_HR_HIGH_LEVEL_SETPOINT,
    SI7000_HR_TEMP_DEVIATION_SETPOINT,
    SI7000_HR_DENSITY_DEVIATION_SETPOINT
};

static uint8_t s_slave_address = 1U; /* Modbus 协议地址配置，影响协议寻址或硬件访问。 */
/* 四类寄存器区都是 CPU3 侧快照，收到请求前由 si7000_modbus_sync_from_system 刷新。 */
static uint8_t s_coils[SI7000_COIL_COUNT]; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static uint8_t s_discrete_inputs[SI7000_DISCRETE_INPUT_COUNT]; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static uint16_t s_holding_regs[SI7000_HOLDING_REG_COUNT]; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static uint16_t s_input_regs[SI7000_INPUT_REG_COUNT]; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static Cpu3DateTime s_profile_timestamp; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */
static uint32_t s_seen_profile_counter = 0U; /* Modbus 协议计数值，用于节拍、统计或协议数量控制。 */
static uint8_t s_profile_timestamp_valid = 0U; /* Modbus 协议模块级变量，保存跨函数共享的业务状态。 */

/*
 * 从 Modbus PDU 中读取大端 16 位值。
 * 寄存器地址和值都走该入口，CRC 仍按 RTU 小端附加。
 */
static inline uint16_t si7000_be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

/*
 * 将 16 位值按 Modbus 大端格式写入响应 PDU。
 * 该函数只写数据字段，CRC 由各响应构造路径统一追加。
 */
static inline void si7000_wr_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)(value & 0xFFU);
}

/*
 * 将 32 位无符号值压缩到 16 位寄存器范围。
 * SI7000 对外寄存器宽度固定为 16 bit，超范围时按最大值饱和。
 */
static uint16_t si7000_clamp_u16(uint32_t value)
{
    return (value > 0xFFFFU) ? 0xFFFFU : (uint16_t)value;
}

/*
 * 将有符号值压缩到 Modbus 16 位有符号寄存器范围。
 * 返回类型保持 uint16_t，实际字节解释由 PLC 按 int16_t 处理。
 */
static uint16_t si7000_clamp_s16(int32_t value)
{
    if (value < -32768) {
        value = -32768;
    } else if (value > 32767) {
        value = 32767;
    }
    return (uint16_t)((int16_t)value);
}

/* 优先使用系统参数中的 Modbus 地址；地址非法时回退本模块默认地址。 */
static uint8_t si7000_get_effective_slave_address(void)
{
    if ((SlaveAddress >= 1) && (SlaveAddress <= 247)) {
        return (uint8_t)SlaveAddress;
    }

    return s_slave_address;
}

/*
 * 将 CPU2 的 0.1mm 无符号长度转换为 SI7000 的 mm 寄存器值。
 * CPU2 无效液位统一输出 0，避免 PLC 读到内部哨兵值。
 */
static uint16_t si7000_u01mm_to_mm_u16(uint32_t value_01mm)
{
    if (value_01mm == UNVALID_LEVEL) {
        return 0U;
    }

    return si7000_clamp_u16((value_01mm + 5U) / 10U);
}

/* 探头位置对 PLC 暴露为无符号 mm，负位置按 0 处理，避免 SI7000 端误读为大正数。 */
static uint16_t si7000_pos01mm_to_mm_u16(int32_t value_01mm)
{
    if (value_01mm <= 0) {
        return 0U;
    }

    return si7000_u01mm_to_mm_u16((uint32_t)value_01mm);
}

/*
 * 将 SI7000 写入的 mm 参数转换为 CPU2 使用的 0.1mm。
 * 当前只用于 profile 起点和间距这类非负参数。
 */
static uint32_t si7000_mm_to_u01mm(uint16_t value_mm)
{
    return (uint32_t)value_mm * 10U;
}

/*
 * 判断 CPU2 温度原始值是否有效。
 * 兼容 0、9999 和无线温度无效值三类历史哨兵，避免对外生成虚假温度。
 */
static uint8_t si7000_is_invalid_temp_raw(uint32_t raw_temperature)
{
    return ((raw_temperature == 0U) ||
            (raw_temperature == SI7000_INVALID_TEMP_RAW_CPU2) ||
            (raw_temperature == (uint32_t)(int32_t)UNVALID_TEMPERATURE_WIRELESS)) ? 1U : 0U;
}

/* CPU2 温度原始值以 0.01K 偏移编码，SI7000 按 0.01C 的有符号值输出。 */
static uint16_t si7000_temp_raw_to_si_s16(uint32_t raw_temperature)
{
    /* 先处理异常边界，避免Modbus 协议状态机带故障继续运行。 */
    if (si7000_is_invalid_temp_raw(raw_temperature) != 0U) {
        return 0U;
    }

    return si7000_clamp_s16((int32_t)raw_temperature - 20000);
}

/*
 * 将 CPU2 密度原始值转换为 SI7000 0.1 单位密度。
 * 无效值输出 0，超范围按 16 位最大值饱和。
 */
static uint16_t si7000_density_raw_to_si_u16(uint32_t raw_density)
{
    if (raw_density == UNVALID_DENSITY) {
        return 0U;
    }
    if (raw_density > (0xFFFFU / 10U)) {
        return 0xFFFFU;
    }

    return (uint16_t)(raw_density * 10U);
}

/*
 * 判断线圈是否允许 PLC 写入。
 * 只放开会触发明确动作的线圈，未实现或只读状态位返回非法地址。
 */
static uint8_t si7000_is_coil_writeable(uint16_t offset)
{
    if (offset <= SI7000_COIL_PROFILE) {
        return 1U;
    }
    if ((offset >= SI7000_COIL_STOP) && (offset <= SI7000_COIL_DOWN_FAST)) {
        return 1U;
    }

    return 0U;
}

/*
 * 判断保持寄存器是否允许 PLC 写入。
 * 已确认能落地或作为影子配置保存的地址才允许写入。
 */
static uint8_t si7000_is_holding_writeable(uint16_t offset)
{
    if (offset <= SI7000_HR_PROFILE_DWELL_TIME) {
        return 1U;
    }
    if ((offset >= SI7000_HR_AUTO_PROFILE_INTERVAL) &&
        (offset <= SI7000_HR_DENSITY_DEVIATION_SETPOINT)) {
        return 1U;
    }

    return 0U;
}

/*
 * 无符号低报警判断。
 * 阈值为 0 时视为未配置，避免默认零值导致误报警。
 */
static uint8_t si7000_low_alarm_u16(uint16_t value, uint16_t setpoint)
{
    return ((setpoint != 0U) && (value < setpoint)) ? 1U : 0U;
}

/*
 * 无符号高报警判断。
 * 阈值为 0 时视为未配置，保持与低报警口径一致。
 */
static uint8_t si7000_high_alarm_u16(uint16_t value, uint16_t setpoint)
{
    return ((setpoint != 0U) && (value > setpoint)) ? 1U : 0U;
}

/*
 * 有符号低温报警判断。
 * SI7000 温度按 int16_t 解释，阈值为 0 时仍视为未配置。
 */
static uint8_t si7000_low_alarm_s16(int16_t value, uint16_t setpoint)
{
    return (setpoint != 0U && value < (int16_t)setpoint) ? 1U : 0U;
}

/*
 * 有符号高温报警判断。
 * 统一在 CPU3 侧计算，CPU2 只发布原始测量值和辅助状态。
 */
static uint8_t si7000_high_alarm_s16(int16_t value, uint16_t setpoint)
{
    return (setpoint != 0U && value > (int16_t)setpoint) ? 1U : 0U;
}

/*
 * 刷新 profile 完成时间戳。
 * 由 FC04 读输入寄存器路径调用，不打印、不阻塞；如果 RTC 短暂不可读，
 * 保持未锁存状态，后续读寄存器时继续尝试，避免丢失本次完成事件。
 */
static void si7000_refresh_profile_timestamp(void)
{
    uint32_t counter = g_measurement.density_distribution.profile_complete_counter;

    if (counter == 0U) {
        s_seen_profile_counter = 0U;
        s_profile_timestamp_valid = 0U;
        return;
    }

    if (g_measurement.density_distribution.profile_complete_latched == 0U) {
        s_profile_timestamp_valid = 0U;
        return;
    }

    if (counter != s_seen_profile_counter) {
        s_seen_profile_counter = counter;
        s_profile_timestamp_valid = 0U;
    }

    if ((s_profile_timestamp_valid == 0U) && (Cpu3Clock_GetDateTime(&s_profile_timestamp) != 0U)) {
        s_profile_timestamp_valid = 1U;
    }
}

/**
 * @brief 发送Modbus 协议中的 si7000_send_cpu2_command 逻辑。
 *
 * @param cmd 命令值。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void si7000_send_cpu2_command(CommandType cmd)
{
    uint32_t cmd32 = (uint32_t)cmd;

    /* 通过 CPU2 既有命令保持寄存器下发，CPU3 不直接改 CPU2 状态机。 */
    CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                               HOLDREGISTER_DEVICEPARAM_COMMAND,
                               2U,
                               &cmd32);
    g_deviceParams.command = CMD_NONE;
}

/**
 * @brief 写入或设置Modbus 协议中的 si7000_write_device_param_u32 逻辑。
 *
 * @param hold_addr 地址参数。
 * @param shadow 业务参数。
 * @param value 待处理数值。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void si7000_write_device_param_u32(uint16_t hold_addr,
                                          volatile uint32_t *shadow,
                                          uint32_t value)
{
    uint32_t value32 = value;

    if (shadow != NULL) {
        /* 先更新 CPU3 影子值，保证 PLC 写后立即回读能看到新配置。 */
        *shadow = value;
    }

    /* 真正持久化和业务生效仍交给 CPU2 原参数通道处理。 */
    CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                               hold_addr,
                               2U,
                               &value32);
}

/**
 * @brief 执行Modbus 协议中的 si7000_build_exception 逻辑。
 *
 * @param func 业务参数。
 * @param ex_code 业务参数。
 * @param tx 业务参数。
 * @param tx_len 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint8_t si7000_build_exception(uint8_t func,
                                      uint8_t ex_code,
                                      uint8_t *tx,
                                      uint16_t *tx_len)
{
    /* Modbus 异常响应为功能码 OR 0x80 + 异常码，仍带标准 CRC。 */
    tx[0] = si7000_get_effective_slave_address();
    tx[1] = (uint8_t)(func | 0x80U);
    tx[2] = ex_code;

    {
        uint16_t crc = CRC16_Calculate(tx, 3);
        tx[3] = (uint8_t)(crc & 0xFFU);
        tx[4] = (uint8_t)(crc >> 8);
    }
    *tx_len = 5U;
    return 1U;
}

/*
 * 构造 FC05/FC06 写单点的标准回显帧。
 * 明确按功能码、地址和值重建回包，避免依赖请求 PDU 前一个字节。
 */
static void si7000_build_write_echo(uint8_t func,
                                    uint16_t offset,
                                    uint16_t value,
                                    uint8_t *tx,
                                    uint16_t *tx_len)
{
    tx[0] = si7000_get_effective_slave_address();
    tx[1] = func;
    si7000_wr_be16(&tx[2], offset);
    si7000_wr_be16(&tx[4], value);

    {
        uint16_t crc = CRC16_Calculate(tx, 6);
        tx[6] = (uint8_t)(crc & 0xFFU);
        tx[7] = (uint8_t)(crc >> 8);
    }
    *tx_len = 8U;
}

/*
 * 校验 FC06 写保持寄存器的基础值域。
 * 仅检查协议层确定无歧义的字段，其余影子寄存器保持宽松，避免误拦 PLC 预留配置。
 */
static uint8_t si7000_is_holding_value_valid(uint16_t offset, uint16_t value)
{
    switch (offset) {
    case SI7000_HR_AUTO_PROFILE_ENABLE:
        return (value <= 1U) ? 1U : 0U;
    case SI7000_HR_AUTO_PROFILE_HOUR:
        return (value <= 23U) ? 1U : 0U;
    case SI7000_HR_AUTO_PROFILE_MINUTE:
        return (value <= 59U) ? 1U : 0U;
    default:
        return 1U;
    }
}

/*
 * 根据 CPU2 当前状态刷新 SI7000 线圈快照。
 * PLC 写线圈只作为命令入口，最终读回以 CPU2 真实状态为准。
 */
static void si7000_refresh_coils_from_state(void)
{
    memset(s_coils, 0, sizeof(s_coils));

    /* 运行模式线圈由 CPU2 当前测量状态反推，PLC 写入只作为命令入口。 */
    switch (g_measurement.device_status.device_state) {
    case STATE_SPREADPOINTING:
    case STATE_GB_SPREADPOINTING:
    case STATE_METER_DENSITY:
    case STATE_INTERVAL_DENSITY:
    case STATE_WARTSILA_DENSITY_MEASURING:
        s_coils[SI7000_COIL_PROFILE] = 1U;
        break;

    case STATE_CALIBRATIONOILING:
    case STATE_CALIBRATE_WATERING:
    case STATE_CALIBRATE_TANKHEIGHTING:
        s_coils[SI7000_COIL_CALIBRATE] = 1U;
        break;

    case STATE_FLOWOIL:
        s_coils[SI7000_COIL_AUTO] = 1U;
        break;

    default:
        s_coils[SI7000_COIL_MANUAL] = 1U;
        break;
    }

    /* 手动方向/停止线圈以电机实时状态为准，避免动作结束后仍显示上/下行。 */
    switch (g_measurement.debug_data.motor_state) {
    case 1U:
        s_coils[SI7000_COIL_UP_MEDIUM] = 1U;
        break;
    case 2U:
        s_coils[SI7000_COIL_DOWN_MEDIUM] = 1U;
        break;
    default:
        s_coils[SI7000_COIL_STOP] = 1U;
        break;
    }
}

/*
 * 根据测量结果和影子阈值刷新 SI7000 离散输入。
 * 该函数不下发 CPU2 命令，只做 CPU3 外部协议状态转换。
 */
static void si7000_refresh_discrete_inputs_from_state(void)
{
    uint16_t current_density;
    uint16_t liquid_level;
    uint16_t point_count;
    uint16_t i;
    uint8_t current_density_valid;
    uint8_t liquid_level_valid;
    uint8_t current_temp_valid;
    int16_t current_temp;

    current_density = si7000_density_raw_to_si_u16(g_measurement.single_point_monitoring.density);
    liquid_level = si7000_u01mm_to_mm_u16(g_measurement.oil_measurement.oil_level);
    current_density_valid = (g_measurement.single_point_monitoring.density != UNVALID_DENSITY) ? 1U : 0U;
    liquid_level_valid = (g_measurement.oil_measurement.oil_level != UNVALID_LEVEL) ? 1U : 0U;
    current_temp_valid = (si7000_is_invalid_temp_raw(g_measurement.debug_data.temperature) == 0U) ? 1U : 0U;
    current_temp = (int16_t)si7000_temp_raw_to_si_s16(g_measurement.debug_data.temperature);
    point_count = si7000_clamp_u16(g_measurement.density_distribution.measurement_points);
    if (point_count > MAX_MEASUREMENT_POINTS) {
        point_count = MAX_MEASUREMENT_POINTS;
    }

    memset(s_discrete_inputs, 0, sizeof(s_discrete_inputs));

    /*
     * 离散输入由实时状态和阈值组合生成。
     * 尚未接入的上下液位传感器保持 0，避免给 PLC 虚假的硬件状态。
     */
    s_discrete_inputs[SI7000_DI_BOTTOM_REFERENCE] =
        (g_measurement.height_measurement.bottom_reference_valid != 0U) ? 1U : 0U;
    s_discrete_inputs[SI7000_DI_INTERLOCK] =
        ((g_measurement.device_status.error_code != NO_ERROR) ||
         (g_measurement.density_distribution.profile_blocked_by_process != 0U)) ? 1U : 0U;
    s_discrete_inputs[SI7000_DI_PROFILE_COMPLETE] =
        (g_measurement.density_distribution.profile_complete_latched != 0U) ? 1U : 0U;
    s_discrete_inputs[SI7000_DI_UNIT_IS_METRIC] = 1U;
    s_discrete_inputs[SI7000_DI_REEL_ALARM] =
        (g_measurement.device_status.error_code != NO_ERROR) ? 1U : 0U;
    s_discrete_inputs[SI7000_DI_PROBE_UNCALIBRATED] =
        (g_measurement.device_status.zero_point_status != 0U) ? 1U : 0U;
    s_discrete_inputs[SI7000_DI_INTERVAL_TIMER] =
        (s_holding_regs[SI7000_HR_AUTO_PROFILE_ENABLE] != 0U) ? 1U : 0U;
    s_discrete_inputs[SI7000_DI_PROBE_AT_LIQUID_LEVEL] =
        (g_measurement.oil_measurement.probe_at_liquid_level != 0U) ? 1U : 0U;
    if (current_density_valid != 0U) {
        s_discrete_inputs[SI7000_DI_LOW_DENSITY_ALARM] =
            si7000_low_alarm_u16(current_density, s_holding_regs[SI7000_HR_LOW_DENSITY_SETPOINT]);
        s_discrete_inputs[SI7000_DI_HIGH_DENSITY_ALARM] =
            si7000_high_alarm_u16(current_density, s_holding_regs[SI7000_HR_HIGH_DENSITY_SETPOINT]);
    }
    if (current_temp_valid != 0U) {
        s_discrete_inputs[SI7000_DI_LOW_TEMP_ALARM] =
            si7000_low_alarm_s16(current_temp, s_holding_regs[SI7000_HR_LOW_TEMPERATURE_SETPOINT]);
        s_discrete_inputs[SI7000_DI_HIGH_TEMP_ALARM] =
            si7000_high_alarm_s16(current_temp, s_holding_regs[SI7000_HR_HIGH_TEMPERATURE_SETPOINT]);
    }
    if (liquid_level_valid != 0U) {
        s_discrete_inputs[SI7000_DI_LL_LEVEL_ALARM] =
            si7000_low_alarm_u16(liquid_level, s_holding_regs[SI7000_HR_LL_LEVEL_SETPOINT]);
        s_discrete_inputs[SI7000_DI_HH_LEVEL_ALARM] =
            si7000_high_alarm_u16(liquid_level, s_holding_regs[SI7000_HR_HH_LEVEL_SETPOINT]);
        s_discrete_inputs[SI7000_DI_LOW_LEVEL_ALARM] =
            si7000_low_alarm_u16(liquid_level, s_holding_regs[SI7000_HR_LOW_LEVEL_SETPOINT]);
        s_discrete_inputs[SI7000_DI_HIGH_LEVEL_ALARM] =
            si7000_high_alarm_u16(liquid_level, s_holding_regs[SI7000_HR_HIGH_LEVEL_SETPOINT]);
    }
    s_discrete_inputs[SI7000_DI_PROFILE_TEMP_DEVIATION_ALARM] =
        (g_measurement.density_distribution.profile_temp_deviation_alarm != 0U) ? 1U : 0U;
    s_discrete_inputs[SI7000_DI_PROFILE_DENSITY_DEVIATION_ALARM] =
        (g_measurement.density_distribution.profile_density_deviation_alarm != 0U) ? 1U : 0U;

    /* profile 点位报警只在完成锁存后计算，未完成时不扫描旧点位数据。 */
    if (g_measurement.density_distribution.profile_complete_latched != 0U) {
        for (i = 0U; i < point_count; ++i) {
            const volatile DensityMeasurement *p = &g_measurement.density_distribution.single_density_data[i];
            uint16_t point_density = si7000_density_raw_to_si_u16(p->density);
            uint8_t point_density_valid = (p->density != UNVALID_DENSITY) ? 1U : 0U;

            /* 先处理异常边界，避免Modbus 协议状态机带故障继续运行。 */
            if (si7000_is_invalid_temp_raw(p->temperature) == 0U) {
                int16_t point_temp = (int16_t)si7000_temp_raw_to_si_s16(p->temperature);

                if (si7000_low_alarm_s16(point_temp, s_holding_regs[SI7000_HR_LOW_TEMPERATURE_SETPOINT]) != 0U) {
                    s_discrete_inputs[SI7000_DI_PROFILE_LOW_TEMP_ALARM] = 1U;
                }
                if (si7000_high_alarm_s16(point_temp, s_holding_regs[SI7000_HR_HIGH_TEMPERATURE_SETPOINT]) != 0U) {
                    s_discrete_inputs[SI7000_DI_PROFILE_HIGH_TEMP_ALARM] = 1U;
                }
            }

            if (point_density_valid != 0U) {
                if (si7000_low_alarm_u16(point_density, s_holding_regs[SI7000_HR_LOW_DENSITY_SETPOINT]) != 0U) {
                    s_discrete_inputs[SI7000_DI_PROFILE_LOW_DENSITY_ALARM] = 1U;
                }
                if (si7000_high_alarm_u16(point_density, s_holding_regs[SI7000_HR_HIGH_DENSITY_SETPOINT]) != 0U) {
                    s_discrete_inputs[SI7000_DI_PROFILE_HIGH_DENSITY_ALARM] = 1U;
                }
            }
        }
    }

    switch (g_measurement.device_status.device_state) {
    case STATE_SPREADPOINTOVER:
    case STATE_GB_SPREADPOINTOVER:
    case STATE_COM_METER_DENSITY_OVER:
    case STATE_INTERVAL_DENSITY_OVER:
    case STATE_WARTSILA_DENSITY_OVER:
        s_discrete_inputs[SI7000_DI_PROFILE_COMPLETE] = 1U;
        break;
    default:
        break;
    }
}

/*
 * 从 CPU2/CPU3 参数刷新 SI7000 保持寄存器快照。
 * 已落地参数来自 g_deviceParams，未确认参数保留影子值便于 PLC 写后回读。
 */
static void si7000_refresh_holding_registers_from_config(void)
{
    /* 已有 CPU2 分布测量参数作为 SI7000 profile 起点、间距和停留时间的来源。 */
    s_holding_regs[SI7000_HR_PROFILE_FIRST_POINT] =
        si7000_u01mm_to_mm_u16(g_deviceParams.spreadTopLimit);
    s_holding_regs[SI7000_HR_PROFILE_INCREMENT] =
        si7000_u01mm_to_mm_u16(g_deviceParams.spreadMeasurementDistance);
    s_holding_regs[SI7000_HR_PROFILE_DWELL_TIME] =
        si7000_clamp_u16(g_deviceParams.spreadPointHoverTime);

    /* 其余地址先保留影子值，便于 FC06 回写后立即回读。 */
}

/*
 * 从实时测量和 profile 结果刷新 SI7000 输入寄存器。
 * 未完成 profile 时点阵区域保持 0，避免 PLC 读取上一轮残留。
 */
static void si7000_refresh_input_registers_from_measurement(void)
{
    Cpu3DateTime now;
    uint16_t i;
    uint16_t point_count = 0U;

    if (g_measurement.density_distribution.profile_complete_latched != 0U) {
        point_count = si7000_clamp_u16(g_measurement.density_distribution.measurement_points);
    }

    if (point_count > MAX_MEASUREMENT_POINTS) {
        point_count = MAX_MEASUREMENT_POINTS;
    }

    memset(s_input_regs, 0, sizeof(s_input_regs));

    /* 实时值每帧刷新，无效值统一输出 0，符合当前 PLC 兼容策略。 */
    s_input_regs[SI7000_IR_CURRENT_PROBE_POSITION] =
        si7000_pos01mm_to_mm_u16(g_measurement.debug_data.sensor_position);
    s_input_regs[SI7000_IR_CURRENT_TEMPERATURE] =
        si7000_temp_raw_to_si_s16(g_measurement.debug_data.temperature);
    s_input_regs[SI7000_IR_CURRENT_DENSITY] =
        si7000_density_raw_to_si_u16(g_measurement.single_point_monitoring.density);
    s_input_regs[SI7000_IR_LIQUID_LEVEL] =
        si7000_u01mm_to_mm_u16(g_measurement.oil_measurement.oil_level);
    s_input_regs[SI7000_IR_NUMBER_OF_POINTS] = point_count;

    si7000_refresh_profile_timestamp();
    if (s_profile_timestamp_valid != 0U) {
        s_input_regs[SI7000_IR_PROFILE_TIMESTAMP_MONTH] = s_profile_timestamp.month;
        s_input_regs[SI7000_IR_PROFILE_TIMESTAMP_DAY] = s_profile_timestamp.day;
        s_input_regs[SI7000_IR_PROFILE_TIMESTAMP_HOUR] = s_profile_timestamp.hour;
        s_input_regs[SI7000_IR_PROFILE_TIMESTAMP_MINUTE] = s_profile_timestamp.minute;
    }

    /* 当前时间来自 CPU3 RTC，仅用于 SI7000 显示，不参与 CPU2 测量调度。 */
    if (Cpu3Clock_GetDateTime(&now) != 0U) {
        s_input_regs[SI7000_IR_CURRENT_TIME_HOUR] = now.hour;
        s_input_regs[SI7000_IR_CURRENT_TIME_MINUTE] = now.minute;
        s_input_regs[SI7000_IR_CURRENT_TIME_SECOND] = now.second;
    }

    /* 只有完成锁存后的 profile 数据才对 PLC 开放，范围外保持 0，避免读到旧点阵残留。 */
    for (i = 0U; i < point_count; ++i) {
        uint16_t base = (uint16_t)(SI7000_IR_PROFILE_POINT0_POSITION + (3U * i));
        const volatile DensityMeasurement *p = &g_measurement.density_distribution.single_density_data[i];

        if ((uint32_t)base + 2U >= SI7000_INPUT_REG_COUNT) {
            break;
        }

        s_input_regs[base] = si7000_u01mm_to_mm_u16(p->temperature_position);
        s_input_regs[base + 1U] = si7000_temp_raw_to_si_s16(p->temperature);
        s_input_regs[base + 2U] = si7000_density_raw_to_si_u16(p->density);
    }
}

/*
 * 应用 PLC 对线圈的写入。
 * ON 写入会转成 CPU2 命令；OFF 写入只更新影子位，不主动停止 CPU2。
 */
static void si7000_apply_coil_write(uint16_t offset, uint8_t is_on)
{
    if (!is_on) {
        return;
    }

    if (offset <= SI7000_COIL_PROFILE) {
        /* SI7000 模式线圈互斥，影子区先收口，真实状态随后由 CPU2 状态刷新。 */
        for (uint16_t i = SI7000_COIL_MANUAL; i <= SI7000_COIL_PROFILE; ++i) {
            s_coils[i] = 0U;
        }
        s_coils[offset] = 1U;
    } else if ((offset >= SI7000_COIL_STOP) && (offset <= SI7000_COIL_DOWN_FAST)) {
        /* 停止和手动方向/速度线圈互斥，避免 FC05 连续写入后读回多个动作位。 */
        for (uint16_t i = SI7000_COIL_STOP; i <= SI7000_COIL_DOWN_FAST; ++i) {
            s_coils[i] = 0U;
        }
        s_coils[offset] = 1U;
    }

    switch (offset) {
    case SI7000_COIL_MANUAL:
        /* 当前系统里最接近 SI7000 Manual 的入口是维护模式。 */
        si7000_send_cpu2_command(CMD_MAINTENANCE_MODE);
        break;
    case SI7000_COIL_CALIBRATE:
        /* 先保守映射到零点标定，后续再细分为零点/液位/水位等标定流程。 */
        si7000_send_cpu2_command(CMD_CALIBRATE_ZERO);
        break;
    case SI7000_COIL_AUTO:
        si7000_send_cpu2_command(CMD_FIND_OIL);
        break;
    case SI7000_COIL_PROFILE:
        si7000_send_cpu2_command(CMD_MEASURE_DISTRIBUTED);
        break;
    case SI7000_COIL_STOP:
        /* 现阶段用维护模式承担“停当前动作并进入手动态”的作用。 */
        si7000_send_cpu2_command(CMD_MAINTENANCE_MODE);
        break;
    case SI7000_COIL_UP_SLOW:
    case SI7000_COIL_UP_MEDIUM:
    case SI7000_COIL_UP_FAST:
        /* 速度档位后续再细分，先统一桥到可被命令切换打断的强制上行。 */
        si7000_send_cpu2_command(CMD_FORCE_MOVE_UP);
        break;
    case SI7000_COIL_DOWN_SLOW:
    case SI7000_COIL_DOWN_MEDIUM:
    case SI7000_COIL_DOWN_FAST:
        /* 速度档位后续再细分，先统一桥到可被命令切换打断的强制下行。 */
        si7000_send_cpu2_command(CMD_FORCE_MOVE_DOWN);
        break;
    default:
        break;
    }
}

/*
 * 应用 PLC 对保持寄存器的写入。
 * profile 基础参数通过 CPU2 参数通道落地，其余未确认配置先留在影子寄存器。
 */
static void si7000_apply_holding_write(uint16_t offset, uint16_t value)
{
    s_holding_regs[offset] = value;

    switch (offset) {
    case SI7000_HR_PROFILE_FIRST_POINT:
        si7000_write_device_param_u32(HOLDREGISTER_DEVICEPARAM_SPREADTOPLIMIT,
                                      &g_deviceParams.spreadTopLimit,
                                      si7000_mm_to_u01mm(value));
        break;
    case SI7000_HR_PROFILE_INCREMENT:
        si7000_write_device_param_u32(HOLDREGISTER_DEVICEPARAM_SPREADMEASUREMENTDISTANCE,
                                      &g_deviceParams.spreadMeasurementDistance,
                                      si7000_mm_to_u01mm(value));
        break;
    case SI7000_HR_PROFILE_DWELL_TIME:
        si7000_write_device_param_u32(HOLDREGISTER_DEVICEPARAM_SPREAD_POINT_HOVER_TIME,
                                      &g_deviceParams.spreadPointHoverTime,
                                      value);
        break;
    case SI7000_HR_AUTO_PROFILE_INTERVAL:
    case SI7000_HR_AUTO_PROFILE_ENABLE:
    case SI7000_HR_AUTO_PROFILE_HOUR:
    case SI7000_HR_AUTO_PROFILE_MINUTE:
    case SI7000_HR_LOW_TEMPERATURE_SETPOINT:
    case SI7000_HR_HIGH_TEMPERATURE_SETPOINT:
    case SI7000_HR_LL_LEVEL_SETPOINT:
    case SI7000_HR_HH_LEVEL_SETPOINT:
    case SI7000_HR_LOW_LEVEL_SETPOINT:
    case SI7000_HR_HIGH_LEVEL_SETPOINT:
    case SI7000_HR_LOW_DENSITY_SETPOINT:
    case SI7000_HR_HIGH_DENSITY_SETPOINT:
    case SI7000_HR_TEMP_DEVIATION_SETPOINT:
    case SI7000_HR_DENSITY_DEVIATION_SETPOINT:
        /* 当前系统还没有一一对应参数，先保留在影子寄存器中。 */
        break;
    default:
        break;
    }
}

/*
 * 处理 FC01/FC02 读位请求。
 * 请求长度、数量和地址边界都在这里统一检查，响应位按 Modbus 低位优先打包。
 */
static uint8_t si7000_handle_read_bits(uint8_t func,
                                       const uint8_t *bit_pool,
                                       uint16_t bit_count,
                                       const uint8_t *pdu,
                                       uint16_t pdu_len,
                                       uint8_t *tx,
                                       uint16_t *tx_len)
{
    uint16_t start;
    uint16_t qty;
    uint16_t byte_count;
    uint16_t i;

    /* FC01/FC02 请求固定为起始地址 2 字节 + 数量 2 字节。 */
    if (pdu_len != 5U) {
        return si7000_build_exception(func, SI7000_EX_ILLEGAL_VALUE, tx, tx_len);
    }

    start = si7000_be16(&pdu[1]);
    qty = si7000_be16(&pdu[3]);

    if ((qty == 0U) || (qty > 2000U)) {
        return si7000_build_exception(func, SI7000_EX_ILLEGAL_VALUE, tx, tx_len);
    }
    if ((uint32_t)start + qty > bit_count) {
        return si7000_build_exception(func, SI7000_EX_ILLEGAL_ADDRESS, tx, tx_len);
    }

    byte_count = (uint16_t)((qty + 7U) / 8U);
    tx[0] = si7000_get_effective_slave_address();
    tx[1] = func;
    tx[2] = (uint8_t)byte_count;
    memset(&tx[3], 0, byte_count);

    /* Modbus bit 响应按低位优先打包，bit0 对应请求的第一个 offset。 */
    for (i = 0U; i < qty; ++i) {
        if (bit_pool[start + i] != 0U) {
            tx[3U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
        }
    }

    {
        uint16_t frame_len = (uint16_t)(3U + byte_count);
        uint16_t crc = CRC16_Calculate(tx, frame_len);
        tx[frame_len] = (uint8_t)(crc & 0xFFU);
        tx[frame_len + 1U] = (uint8_t)(crc >> 8);
        *tx_len = (uint16_t)(frame_len + 2U);
    }
    return 1U;
}

/*
 * 处理 FC03/FC04 读寄存器请求。
 * 限制最大 125 个寄存器，保证响应始终落在 256 字节发送缓冲区内。
 */
static uint8_t si7000_handle_read_regs(uint8_t func,
                                       const uint16_t *reg_pool,
                                       uint16_t reg_count,
                                       const uint8_t *pdu,
                                       uint16_t pdu_len,
                                       uint8_t *tx,
                                       uint16_t *tx_len)
{
    uint16_t start;
    uint16_t qty;
    uint16_t i;

    /* FC03/FC04 请求固定为起始寄存器 2 字节 + 数量 2 字节。 */
    if (pdu_len != 5U) {
        return si7000_build_exception(func, SI7000_EX_ILLEGAL_VALUE, tx, tx_len);
    }

    start = si7000_be16(&pdu[1]);
    qty = si7000_be16(&pdu[3]);

    if ((qty == 0U) || (qty > 125U)) {
        return si7000_build_exception(func, SI7000_EX_ILLEGAL_VALUE, tx, tx_len);
    }
    if ((uint32_t)start + qty > reg_count) {
        return si7000_build_exception(func, SI7000_EX_ILLEGAL_ADDRESS, tx, tx_len);
    }

    /* 寄存器响应逐项转为大端，保持 Modbus RTU 标准字节序。 */
    tx[0] = si7000_get_effective_slave_address();
    tx[1] = func;
    tx[2] = (uint8_t)(qty * 2U);
    for (i = 0U; i < qty; ++i) {
        si7000_wr_be16(&tx[3U + (2U * i)], reg_pool[start + i]);
    }

    {
        uint16_t frame_len = (uint16_t)(3U + qty * 2U);
        uint16_t crc = CRC16_Calculate(tx, frame_len);
        tx[frame_len] = (uint8_t)(crc & 0xFFU);
        tx[frame_len + 1U] = (uint8_t)(crc >> 8);
        *tx_len = (uint16_t)(frame_len + 2U);
    }
    return 1U;
}

/*
 * 处理 FC05 写单线圈请求。
 * 只接受标准 0xFF00/0x0000 写值，合法 ON 写入再桥接到 CPU2 命令。
 */
static uint8_t si7000_handle_write_single_coil(const uint8_t *pdu,
                                               uint16_t pdu_len,
                                               uint8_t *tx,
                                               uint16_t *tx_len)
{
    uint16_t offset;
    uint16_t value;
    uint8_t is_on;

    if (pdu_len != 5U) {
        return si7000_build_exception(SI7000_FUNC_WRITE_SINGLE_COIL,
                                      SI7000_EX_ILLEGAL_VALUE,
                                      tx,
                                      tx_len);
    }

    offset = si7000_be16(&pdu[1]);
    value = si7000_be16(&pdu[3]);

    if ((offset >= SI7000_COIL_COUNT) || (si7000_is_coil_writeable(offset) == 0U)) {
        return si7000_build_exception(SI7000_FUNC_WRITE_SINGLE_COIL,
                                      SI7000_EX_ILLEGAL_ADDRESS,
                                      tx,
                                      tx_len);
    }
    /* FC05 标准只接受 0xFF00/0x0000，其他值按非法数据值处理。 */
    if ((value != 0x0000U) && (value != 0xFF00U)) {
        return si7000_build_exception(SI7000_FUNC_WRITE_SINGLE_COIL,
                                      SI7000_EX_ILLEGAL_VALUE,
                                      tx,
                                      tx_len);
    }

    is_on = (value == 0xFF00U) ? 1U : 0U;
    s_coils[offset] = is_on;
    si7000_apply_coil_write(offset, is_on);

    si7000_build_write_echo(SI7000_FUNC_WRITE_SINGLE_COIL, offset, value, tx, tx_len);
    return 1U;
}

/*
 * 处理 FC06 写单保持寄存器请求。
 * 先做地址和值域校验，再更新影子寄存器或通过 CPU2 参数通道下发。
 */
static uint8_t si7000_handle_write_single_reg(const uint8_t *pdu,
                                              uint16_t pdu_len,
                                              uint8_t *tx,
                                              uint16_t *tx_len)
{
    uint16_t offset;
    uint16_t value;

    if (pdu_len != 5U) {
        return si7000_build_exception(SI7000_FUNC_WRITE_SINGLE_REG,
                                      SI7000_EX_ILLEGAL_VALUE,
                                      tx,
                                      tx_len);
    }

    offset = si7000_be16(&pdu[1]);
    value = si7000_be16(&pdu[3]);

    if ((offset >= SI7000_HOLDING_REG_COUNT) || (si7000_is_holding_writeable(offset) == 0U)) {
        return si7000_build_exception(SI7000_FUNC_WRITE_SINGLE_REG,
                                      SI7000_EX_ILLEGAL_ADDRESS,
                                      tx,
                                      tx_len);
    }
    /* 对自动 profile 使能和时分字段做基础范围保护，其余阈值保留现场自由度。 */
    if (si7000_is_holding_value_valid(offset, value) == 0U) {
        return si7000_build_exception(SI7000_FUNC_WRITE_SINGLE_REG,
                                      SI7000_EX_ILLEGAL_VALUE,
                                      tx,
                                      tx_len);
    }

    s_holding_regs[offset] = value;
    si7000_apply_holding_write(offset, value);

    si7000_build_write_echo(SI7000_FUNC_WRITE_SINGLE_REG, offset, value, tx, tx_len);
    return 1U;
}



/*
 * 手动同步 SI7000 四类寄存器影子区。
 * 正常处理请求前会自动调用，外部入口主要用于测试或联调前刷新快照。
 */
void si7000_modbus_sync_from_system(void)
{
    /* 每次处理请求前统一刷新快照，保证读请求和写后回读看到同一套状态口径。 */
    si7000_refresh_coils_from_state();
    si7000_refresh_holding_registers_from_config();
    si7000_refresh_discrete_inputs_from_state();
    si7000_refresh_input_registers_from_measurement();
}

/*
 * 处理一帧完整 SI7000 Modbus RTU 请求。
 * 地址和 CRC 通过后才同步系统状态，避免无关帧扰动 CPU3 快照。
 */
Si7000ModbusResult si7000_modbus_process(const uint8_t *rx,
                                         uint16_t rx_len,
                                         uint8_t *tx,
                                         uint16_t *tx_len)
{
    const uint8_t *pdu;
    uint16_t pdu_len;
    uint8_t func;

    if ((rx == NULL) || (tx == NULL) || (tx_len == NULL)) {
        return SI7000_MODBUS_ERR_BADLEN;
    }

    *tx_len = 0U;

    if (rx_len < 4U) {
        return SI7000_MODBUS_ERR_BADLEN;
    }
    if (rx[0] != si7000_get_effective_slave_address()) {
        return SI7000_MODBUS_ERR_ADDR_MISMATCH;
    }
    if (!SlaveCheckCRC(rx, rx_len)) {
        return SI7000_MODBUS_ERR_CRC;
    }

    /* CRC 和地址通过后再同步系统状态，降低无关帧对 CPU3 运行状态的扰动。 */
    si7000_modbus_sync_from_system();

    func = rx[1];
    pdu = &rx[1];
    pdu_len = (uint16_t)(rx_len - 3U);

    switch (func) {
    case SI7000_FUNC_READ_COILS:
        si7000_handle_read_bits(func, s_coils, SI7000_COIL_COUNT, pdu, pdu_len, tx, tx_len);
        return SI7000_MODBUS_OK;

    case SI7000_FUNC_READ_DISCRETE_INPUTS:
        si7000_handle_read_bits(func, s_discrete_inputs, SI7000_DISCRETE_INPUT_COUNT, pdu, pdu_len, tx, tx_len);
        return SI7000_MODBUS_OK;

    case SI7000_FUNC_READ_HOLDING_REGS:
        si7000_handle_read_regs(func, s_holding_regs, SI7000_HOLDING_REG_COUNT, pdu, pdu_len, tx, tx_len);
        return SI7000_MODBUS_OK;

    case SI7000_FUNC_READ_INPUT_REGS:
        si7000_handle_read_regs(func, s_input_regs, SI7000_INPUT_REG_COUNT, pdu, pdu_len, tx, tx_len);
        return SI7000_MODBUS_OK;

    case SI7000_FUNC_WRITE_SINGLE_COIL:
        si7000_handle_write_single_coil(pdu, pdu_len, tx, tx_len);
        return SI7000_MODBUS_OK;

    case SI7000_FUNC_WRITE_SINGLE_REG:
        si7000_handle_write_single_reg(pdu, pdu_len, tx, tx_len);
        return SI7000_MODBUS_OK;

    default:
        si7000_build_exception(func, SI7000_EX_ILLEGAL_FUNCTION, tx, tx_len);
        return SI7000_MODBUS_ERR_FUNC_UNSUPPORT;
    }
}

/*
 * 适配 CPU3 协议分发表的返回值口径。
 * 只要已生成正常或异常响应帧，分发层就按成功处理并发送该响应。
 */
uint32_t si7000_modbus_process_for_dispatch(const uint8_t *rx,
                                            uint16_t rx_len,
                                            uint8_t *tx,
                                            uint16_t *tx_len)
{
    Si7000ModbusResult ret = si7000_modbus_process(rx, rx_len, tx, tx_len);

    /* 统一外部协议分发语义：只要已经生成正常/异常响应帧，分发层就视为本帧已处理。 */
    if ((tx_len != NULL) && (*tx_len > 0U)) {
        return 0U;
    }

    return (uint32_t)ret;
}

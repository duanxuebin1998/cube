#include "si_modbus_slave.h"

#include <string.h>

#include "address.h"
#include "cpu2_communicate.h"
#include "../external_read_freshness.h"
#include "cpu3_comm_display_params.h"
#include "cpu3_clock.h"
#include "main.h"
#include "my_crc.h"
#include "stateformodbus.h"
#include "system_parameter.h"

/*
 * SI协议兼容层只在 CPU3 侧做协议转换：
 * - 对 PLC 暴露 SI Modbus RTU 地址、功能码和缩放；
 * - 对内复用 CPU2 已有测量变量、命令和协议辅助状态；
 * - SI 专用配置按归属分别落到 CPU2 profile 参数或 CPU3 本机参数。
 */
/* SI 外部 Modbus 功能码 0x01，读取线圈；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define SI_FUNC_READ_COILS             0x01U
/* SI 外部 Modbus 功能码 0x02，读取离散输入；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define SI_FUNC_READ_DISCRETE_INPUTS   0x02U
/* SI 外部 Modbus 功能码 0x03，读取保持寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define SI_FUNC_READ_HOLDING_REGS      0x03U
/* SI 外部 Modbus 功能码 0x04，读取输入寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define SI_FUNC_READ_INPUT_REGS        0x04U
/* SI 外部 Modbus 功能码 0x05，写单个线圈；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define SI_FUNC_WRITE_SINGLE_COIL      0x05U
/* SI 外部 Modbus 功能码 0x06，写单个保持寄存器；处理前仍需校验完整帧长、地址范围和字段边界。 */
#define SI_FUNC_WRITE_SINGLE_REG       0x06U

/* SI Modbus 异常码：非法功能码；异常响应功能码需同时置最高位。 */
#define SI_EX_ILLEGAL_FUNCTION         0x01U
/* SI Modbus 异常码：非法数据地址；异常响应功能码需同时置最高位。 */
#define SI_EX_ILLEGAL_ADDRESS          0x02U
/* SI Modbus 异常码：非法数据值或长度；异常响应功能码需同时置最高位。 */
#define SI_EX_ILLEGAL_VALUE            0x03U
/* SI Modbus 异常码：从站忙，当前无法完成请求；异常响应功能码需同时置最高位。 */
#define SI_EX_SLAVE_DEVICE_BUSY        0x06U

/* CPU2 共享快照中的历史无效温度原始值 9999；SI 映射时必须转换为 SI_TEMP_INVALID_REGISTER，而不是按有效温度缩放。 */
#define SI_INVALID_TEMP_RAW_CPU2       9999U
/* SI 协议对外发布的无效温度寄存器位模式 0xB1E0；对应 -200.00 ℃ 的有符号 0.01 ℃ 编码。 */
#define SI_TEMP_INVALID_REGISTER       0xB1E0U
/* SI 自动剖面启动请求未被接受后的最小重试间隔 5000 ms；同一分钟内只在限流后重试。 */
#define SI_AUTO_PROFILE_RETRY_DELAY_MS 5000U
/* SI 剖面结果尚未就绪或拉取失败后的重试间隔 1000 ms。 */
#define SI_PROFILE_FETCH_RETRY_DELAY_MS 1000U
/* SI 剖面单点停留时间允许上限 3600 s；FC06/FC10 写入超出范围时返回非法数据值。 */
#define SI_PROFILE_DWELL_TIME_MAX_S    3600U
/* SI 剖面点缓存占用的寄存器数量；每个测点发布位置、密度和温度三个寄存器字段。 */
#define SI_PROFILE_POINT_REG_COUNT     (MAX_MEASUREMENT_POINTS * 3U)

/* 线圈地址保持 SI协议手册编号，数组下标即协议 offset。 */
enum {
    /* SI 线圈地址索引；数值即线圈表偏移，保留的地址间隙不得压缩。 */
    SI_COIL_MANUAL = 0, /* 切换到 SI 手动控制模式。 */
    SI_COIL_CALIBRATE, /* 触发 SI 标定流程。 */
    SI_COIL_AUTO, /* 切换到 SI 自动运行模式。 */
    SI_COIL_PROFILE, /* 触发一次 SI 剖面测量。 */
    SI_COIL_STOP = 8, /* 停止当前 SI 运动或测量流程。 */
    SI_COIL_UP_SLOW, /* 以低速向上移动探头。 */
    SI_COIL_UP_MEDIUM, /* 以中速向上移动探头。 */
    SI_COIL_UP_FAST, /* 以高速向上移动探头。 */
    SI_COIL_DOWN_SLOW, /* 以低速向下移动探头。 */
    SI_COIL_DOWN_MEDIUM, /* 以中速向下移动探头。 */
    SI_COIL_DOWN_FAST /* 以高速向下移动探头。 */
};

/* 离散输入同样按协议 offset 排列，预留洞保留原地址，不重新压缩。 */
enum {
    /* SI 离散输入地址索引；数值即离散输入表偏移，保留的地址间隙不得压缩。 */
    SI_DI_BOTTOM_REFERENCE = 0, /* 罐底位置基准已经建立。 */
    SI_DI_LOWER_LEVEL_SENSOR, /* 下液位传感条件当前成立。 */
    SI_DI_UPPER_LEVEL_SENSOR, /* 上液位传感条件当前成立。 */
    SI_DI_INTERLOCK, /* 设备互锁条件当前成立。 */
    SI_DI_PROFILE_COMPLETE, /* 最近一次 SI 剖面测量已经完整提交。 */
    SI_DI_UNIT_IS_METRIC, /* SI 寄存器中的过程量采用公制单位。 */
    SI_DI_REEL_ALARM = 8, /* 卷筒或尺带机构报警。 */
    SI_DI_PROBE_UNCALIBRATED, /* 探头尚未完成有效标定。 */
    SI_DI_INTERVAL_TIMER = 11, /* 自动剖面间隔定时条件已到。 */
    SI_DI_PROBE_AT_LIQUID_LEVEL = 12, /* 探头当前已到达液面。 */
    SI_DI_LOW_DENSITY_ALARM = 16, /* 当前密度低于低密度报警阈值。 */
    SI_DI_HIGH_DENSITY_ALARM, /* 当前密度高于高密度报警阈值。 */
    SI_DI_LOW_TEMP_ALARM, /* 当前温度低于低温报警阈值。 */
    SI_DI_HIGH_TEMP_ALARM, /* 当前温度高于高温报警阈值。 */
    SI_DI_LL_LEVEL_ALARM, /* 当前液位满足低低报警条件。 */
    SI_DI_HH_LEVEL_ALARM, /* 当前液位满足高高报警条件。 */
    SI_DI_LOW_LEVEL_ALARM, /* 当前液位满足低报警条件。 */
    SI_DI_HIGH_LEVEL_ALARM, /* 当前液位满足高报警条件。 */
    SI_DI_PROFILE_TEMP_DEVIATION_ALARM, /* 剖面内温度最大偏差超过设定阈值。 */
    SI_DI_PROFILE_DENSITY_DEVIATION_ALARM, /* 剖面内密度最大偏差超过设定阈值。 */
    SI_DI_PROFILE_LOW_TEMP_ALARM = 28, /* 剖面测点中存在低温报警。 */
    SI_DI_PROFILE_HIGH_TEMP_ALARM, /* 剖面测点中存在高温报警。 */
    SI_DI_PROFILE_LOW_DENSITY_ALARM, /* 剖面测点中存在低密度报警。 */
    SI_DI_PROFILE_HIGH_DENSITY_ALARM /* 剖面测点中存在高密度报警。 */
};

/* 输入寄存器：实时测量值、profile 时间戳、当前时间和 profile 点阵共用一张表。 */
enum {
    /* SI 输入寄存器地址索引；多字测量值按协议规定的连续寄存器布局投影。 */
    SI_IR_CURRENT_PROBE_POSITION = 0, /* 当前探头位置输入寄存器起址。 */
    SI_IR_CURRENT_TEMPERATURE, /* 当前温度输入寄存器起址。 */
    SI_IR_CURRENT_DENSITY, /* 当前密度输入寄存器起址。 */
    SI_IR_LIQUID_LEVEL, /* 当前液位输入寄存器起址。 */
    SI_IR_NUMBER_OF_POINTS = 5, /* 当前已发布剖面的有效测点数寄存器。 */
    SI_IR_PROFILE_TIMESTAMP_MONTH, /* 剖面完成时间的月份寄存器。 */
    SI_IR_PROFILE_TIMESTAMP_DAY, /* 剖面完成时间的日期寄存器。 */
    SI_IR_PROFILE_TIMESTAMP_HOUR, /* 剖面完成时间的小时寄存器。 */
    SI_IR_PROFILE_TIMESTAMP_MINUTE, /* 剖面完成时间的分钟寄存器。 */
    SI_IR_CURRENT_TIME_HOUR, /* CPU3 当前 RTC 小时寄存器。 */
    SI_IR_CURRENT_TIME_MINUTE, /* CPU3 当前 RTC 分钟寄存器。 */
    SI_IR_CURRENT_TIME_SECOND, /* CPU3 当前 RTC 秒寄存器。 */
    SI_IR_COIL_MIRROR, /* SI 线圈镜像打包寄存器。 */
    SI_IR_DISCRETE_MIRROR_LOW, /* SI 离散输入位图低 16 位镜像寄存器。 */
    SI_IR_DISCRETE_MIRROR_HIGH, /* SI 离散输入位图高 16 位镜像寄存器。 */
    SI_IR_PROFILE_POINT0_POSITION = 20 /* 剖面第 0 点位置字段的输入寄存器起址。 */
};

/* 保持寄存器：profile 执行参数在 CPU2，自动调度和报警限值在 CPU3。 */
enum {
    /* SI 保持寄存器地址索引；包含剖面配置、自动调度和报警阈值，保留兼容槽不得复用。 */
    SI_HR_PROFILE_FIRST_POINT = 0, /* SI 剖面首个测点位置配置。 */
    SI_HR_PROFILE_INCREMENT, /* SI 剖面相邻测点间距配置。 */
    SI_HR_PROFILE_DWELL_TIME, /* SI 每个剖面测点的停留时间配置。 */
    SI_HR_COMPAT_RAW_40004, /* 兼容旧主站的 40004 原始保留寄存器。 */
    SI_HR_COMPAT_RAW_40005, /* 兼容旧主站的 40005 原始保留寄存器。 */
    SI_HR_COMPAT_RAW_40006, /* 兼容旧主站的 40006 原始保留寄存器。 */
    SI_HR_COMPAT_RAW_40007, /* 兼容旧主站的 40007 原始保留寄存器。 */
    SI_HR_COMPAT_RAW_40008, /* 兼容旧主站的 40008 原始保留寄存器。 */
    SI_HR_COMPAT_RAW_40009, /* 兼容旧主站的 40009 原始保留寄存器。 */
    SI_HR_AUTO_PROFILE_INTERVAL = 9, /* SI 自动剖面重复间隔配置。 */
    SI_HR_AUTO_PROFILE_ENABLE, /* SI 自动剖面调度使能配置。 */
    SI_HR_AUTO_PROFILE_HOUR, /* SI 自动剖面首次启动小时配置。 */
    SI_HR_AUTO_PROFILE_MINUTE, /* SI 自动剖面首次启动分钟配置。 */
    SI_HR_LOW_DENSITY_SETPOINT, /* SI 低密度报警设定值。 */
    SI_HR_HIGH_DENSITY_SETPOINT, /* SI 高密度报警设定值。 */
    SI_HR_LOW_TEMPERATURE_SETPOINT, /* SI 低温报警设定值。 */
    SI_HR_HIGH_TEMPERATURE_SETPOINT, /* SI 高温报警设定值。 */
    SI_HR_LL_LEVEL_SETPOINT, /* SI 低低液位报警设定值。 */
    SI_HR_HH_LEVEL_SETPOINT, /* SI 高高液位报警设定值。 */
    SI_HR_LOW_LEVEL_SETPOINT, /* SI 低液位报警设定值。 */
    SI_HR_HIGH_LEVEL_SETPOINT, /* SI 高液位报警设定值。 */
    SI_HR_TEMP_DEVIATION_SETPOINT, /* SI 剖面温度偏差报警阈值。 */
    SI_HR_DENSITY_DEVIATION_SETPOINT /* SI 剖面密度偏差报警阈值。 */
};

/* SI 剖面测量投影上下文；跨轮询保存 CPU2 周期键、候选快照和最终发布状态，确保只发布前后头一致的完整点阵。 */
typedef struct {
    /* CPU3 对 SI 剖面测量的投影状态；有效标志和代际键共同约束候选点阵何时可以对外发布。 */
    uint8_t initialized; /* SI 投影上下文已经清零并建立初始阶段基线的标志。 */
    uint8_t connected; /* CPU2 通信当前已建立的标志；断开时投影状态会重新建立基线。 */
    uint8_t cycle_established; /* 当前 SI 剖面周期键已经建立的标志。 */
    uint8_t candidate_valid; /* candidate_key 和候选点阵已经完整抓取、可进入一致性复核的标志。 */
    uint8_t final_snapshot_ready; /* 候选点阵已通过完整性检查、等待最终发布的标志。 */
    uint8_t published_key_valid; /* published_key 已随完整点阵提交、可以参与去重比较的标志。 */
    uint8_t fetch_attempt_valid; /* 本轮点阵抓取尝试的周期和节拍基线已经建立的标志。 */
    uint8_t complete_baseline_valid; /* 剖面完成计数基线已经建立、可用于检测新完成事件的标志。 */
    uint32_t cpu2_snapshot_generation; /* 读取候选剖面时对应的 CPU2 输入快照代际。 */
    uint32_t active_cycle; /* 当前正在跟踪的 SI 剖面周期计数。 */
    uint32_t last_phase; /* 上一轮读取到的 SI 剖面阶段，用于检测阶段边沿。 */
    uint32_t cycle_complete_baseline;         /* PREPARING、初始化或重连时冻结的完成计数 */
    uint32_t fetch_cycle; /* 当前候选点阵抓取所属的 SI 周期计数。 */
    uint32_t fetch_complete_counter; /* 开始本轮抓取时锁存的剖面完成计数，用于确认结果未被新周期覆盖。 */
    uint32_t last_fetch_tick; /* 最近一次尝试抓取 SI 点阵的 HAL 毫秒节拍，用于限制重试频率。 */
    Cpu2SiProfileCandidateKey candidate_key; /* 尚未发布的 SI 剖面候选组合键。 */
    Cpu2SiProfileCandidateKey published_key; /* 最近一次完整发布的 SI 剖面组合键。 */
    uint16_t progress_points; /* 当前周期已经完成并可报告的剖面测点数。 */
    uint16_t final_points; /* 最终快照确认的有效测点数。 */
    uint16_t point_regs[SI_PROFILE_POINT_REG_COUNT]; /* 从 CPU2 分块读取的 SI 剖面原始寄存器点阵。 */
    uint8_t profile_temp_deviation_alarm; /* 本次 SI 剖面温度偏差超过设定阈值的报警标志。 */
    uint8_t profile_density_deviation_alarm; /* 本次 SI 剖面密度偏差超过设定阈值的报警标志。 */
    uint8_t profile_low_temp_alarm; /* 本次 SI 剖面低温报警标志。 */
    uint8_t profile_high_temp_alarm; /* 本次 SI 剖面高温报警标志。 */
    uint8_t profile_low_density_alarm; /* 本次 SI 剖面低密度报警标志。 */
    uint8_t profile_high_density_alarm; /* 本次 SI 剖面高密度报警标志。 */
} SiProfileProjection;

static uint8_t s_slave_address = 1U; /* Modbus 协议地址配置，影响协议寻址或硬件访问。 */
/* 四类寄存器区都是 CPU3 侧快照，收到请求前由 si_modbus_sync_from_system 刷新。 */
static uint8_t s_coils[SI_COIL_COUNT]; /* SI 线圈镜像；按 SI 地址表保存命令位，处理后同步清除瞬时动作。 */
static uint8_t s_discrete_inputs[SI_DISCRETE_INPUT_COUNT]; /* SI 离散输入镜像；每次响应前由 CPU2 运行态和报警状态重建。 */
static uint16_t s_holding_regs[SI_HOLDING_REG_COUNT]; /* SI 保持寄存器镜像；包含剖面配置、自动调度和报警设定值。 */
static uint16_t s_input_regs[SI_INPUT_REG_COUNT]; /* SI 输入寄存器镜像；包含当前测量值、状态位和剖面点阵投影。 */
static Cpu3DateTime s_profile_timestamp; /* 最近一次已发布 SI 剖面快照对应的本地日期时间。 */
static uint8_t s_profile_timestamp_valid = 0U; /* s_profile_timestamp 已随完整剖面建立的有效标志。 */
static SiProfileProjection s_profile_projection;
/* 最近一次成功触发 SI 自动剖面的绝对分钟编号，防止同一分钟重复启动。 */
static uint32_t s_si_auto_last_trigger_minute = 0xFFFFFFFFUL;
/* 最近一次尝试启动 SI 自动剖面的 HAL 毫秒节拍。 */
static uint32_t s_si_auto_last_attempt_tick = 0U;
/* SI 周期调度锚点的绝对分钟编号。 */
static uint32_t s_si_auto_schedule_anchor_minute = 0U;
/* 最近一次校验通过的 SI 自动剖面周期间隔，单位为分钟。 */
static uint16_t s_si_auto_cached_interval = 0U;
/* 最近一次缓存的 SI 自动剖面使能值。 */
static uint8_t s_si_auto_cached_enable = 0U;
/* 最近一次缓存的 SI 首次启动小时。 */
static uint8_t s_si_auto_cached_hour = 0U;
/* 最近一次缓存的 SI 首次启动分钟。 */
static uint8_t s_si_auto_cached_minute = 0U;
/* 最近尝试节拍已经建立的标志。 */
static bool s_si_auto_last_attempt_valid = false;
/* 上电保护窗口尚未结束、暂不允许自动启动剖面的标志。 */
static bool s_si_auto_boot_guard_pending = true;
/* 缓存的 SI 自动调度配置已经通过范围校验的标志。 */
static bool s_si_auto_config_valid = false;
/* SI 周期调度锚点已经建立的标志。 */
static bool s_si_auto_schedule_anchor_valid = false;
/* 调度锚点应从下一次计划启动时间建立的标志。 */
static bool s_si_auto_anchor_from_next_start = false;

/**
 * @brief 从 Modbus PDU 中读取大端 16 位值。
 *
 * 寄存器地址和值都走该入口，CRC 仍按 RTU 小端附加。
 *
 * @param p 指向至少 2 字节 Modbus PDU 数据的只读指针。
 * @return 返回从连续 2 字节 Modbus 大端数据还原的 16 位无符号值。
 */
static inline uint16_t si_be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

/**
 * @brief 将 16 位值按 Modbus 大端格式写入响应 PDU。
 *
 * 该函数只写数据字段，CRC 由各响应构造路径统一追加。
 *
 * @param p 指向至少 2 字节可写响应区的指针，用于写入大端 16 位值。
 * @param value 待写入协议缓冲区或寄存器的16 位数值。
 */
static inline void si_wr_be16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);
    p[1] = (uint8_t)(value & 0xFFU);
}

/**
 * @brief 将 32 位无符号值压缩到 16 位寄存器范围。
 *
 * SI协议对外寄存器宽度固定为 16 bit，超范围时按最大值饱和。
 *
 * @param value 待限制到合法范围的原始输入值。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static uint16_t si_clamp_u16(uint32_t value)
{
    return (value > 0xFFFFU) ? 0xFFFFU : (uint16_t)value;
}

/**
 * @brief 将有符号值压缩到 Modbus 16 位有符号寄存器范围。
 *
 * 返回类型保持 uint16_t，实际字节解释由 PLC 按 int16_t 处理。
 *
 * @param value 待限制到合法范围的原始输入值。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static uint16_t si_clamp_s16(int32_t value)
{
    if (value < -32768) {
        value = -32768;
    } else if (value > 32767) {
        value = 32767;
    }
    return (uint16_t)((int16_t)value);
}

/**
 * @brief 优先使用系统参数中的 Modbus 地址；地址非法时回退本模块默认地址。
 *
 * @return 返回当前有效 SI Modbus 从机地址；系统配置非法时回退模块默认地址。
 */
static uint8_t si_get_effective_slave_address(void)
{
    if ((SlaveAddress >= 1) && (SlaveAddress <= 247)) {
        return (uint8_t)SlaveAddress;
    }

    return s_slave_address;
}

/**
 * @brief 将 CPU2 的 0.1mm 无符号长度转换为 SI 的 mm 寄存器值。
 *
 * @param value_01mm CPU2 无符号长度原始值，单位 0.1 mm；UNVALID_LEVEL 表示内部液位无效。
 * @return UNVALID_LEVEL 返回 0；其他输入由 0.1 mm 四舍五入为整毫米，并在超过 SI 16 位寄存器范围时饱和为 UINT16_MAX。
 * @note CPU2 无效液位统一对外输出 0，禁止把内部哨兵值暴露给 PLC。
 */
static uint16_t si_u01mm_to_mm_u16(uint32_t value_01mm)
{
    if (value_01mm == UNVALID_LEVEL) {
        return 0U;
    }

    return si_clamp_u16((value_01mm + 5U) / 10U);
}

/**
 * @brief 探头位置对 PLC 暴露为无符号 mm，负位置按 0 处理，避免 SI 端误读为大正数。
 *
 * @param value_01mm 位置或距离值，单位 0.1 mm。
 * @return 返回 0.1 mm 位置换算并钳位后的无符号毫米值；负位置返回 0。
 */
static uint16_t si_pos01mm_to_mm_u16(int32_t value_01mm)
{
    if (value_01mm <= 0) {
        return 0U;
    }

    return si_u01mm_to_mm_u16((uint32_t)value_01mm);
}

/**
 * @brief 将 SI 写入的 mm 参数转换为 CPU2 使用的 0.1mm。
 *
 * 当前只用于 profile 起点和间距这类非负参数。
 *
 * @param value_mm SI 保持寄存器写入的毫米值。
 * @return 返回 value_mm 乘以 10 后的 CPU2 长度值，单位 0.1 mm；uint16_t 全范围换算均不会溢出 uint32_t。
 */
static uint32_t si_mm_to_u01mm(uint16_t value_mm)
{
    return (uint32_t)value_mm * 10U;
}

/**
 * @brief 判断 CPU2 温度原始值是否有效。
 *
 * 兼容 0、9999 和无线温度无效值三类历史哨兵，避免对外生成虚假温度。
 *
 * @param raw_temperature 原始温度。
 * @return 1 表示 raw_temperature 为 0、SI_INVALID_TEMP_RAW_CPU2，或无线无效温度哨兵，应判定为无效；0 表示未命中这些哨兵，可继续按 SI 温度口径转换。
 */
static uint8_t si_is_invalid_temp_raw(uint32_t raw_temperature)
{
    return ((raw_temperature == 0U) ||
            (raw_temperature == SI_INVALID_TEMP_RAW_CPU2) ||
            (raw_temperature == (uint32_t)(int32_t)UNVALID_TEMPERATURE_WIRELESS)) ? 1U : 0U;
}

/**
 * @brief 将 SI 有符号 16 位值按原始位模式转换为保持寄存器无符号字。
 *
 * @param value 待处理的 SI 有符号 16 位值按原始位模式。
 * @return 返回与输入有符号值完全相同的 16 位原始位模式，并以 uint16_t 承载；不执行数值缩放。
 */
static uint16_t si_s16_to_holding(int16_t value)
{
    return (uint16_t)value;
}

/**
 * @brief 将 SI 保持寄存器无符号字按原始位模式还原为有符号 16 位值。
 *
 * @param value 待处理的 SI 保持寄存器无符号字按原始位模式。
 * @return 返回与输入保持寄存器完全相同的 16 位原始位模式，并以 int16_t 解释；不执行数值缩放。
 */
static int16_t si_holding_to_s16(uint16_t value)
{
    return (int16_t)value;
}

/**
 * @brief 计算两个无符号 16 位数的绝对差值。
 *
 * @param a 算法或比较使用的第一个输入值。
 * @param b 算法或比较使用的第二个输入值。
 * @return 返回两个无符号 16 位数的绝对差值；有符号边界按函数内饱和规则处理。
 */
static uint16_t si_absdiff_u16(uint16_t a, uint16_t b)
{
    return (a >= b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

/**
 * @brief 计算 SI 温度寄存器的有符号绝对差值。
 *
 * 相邻点温差报警必须按 int16 温度值判断，避免负温度跨零时按补码无符号差值误报警。
 *
 * @param a 算法或比较使用的第一个输入值。
 * @param b 算法或比较使用的第二个输入值。
 * @return 返回 SI 温度寄存器的有符号绝对差值；有符号边界按函数内饱和规则处理。
 */
static uint16_t si_absdiff_s16(int16_t a, int16_t b)
{
    int32_t delta = (int32_t)a - (int32_t)b;

    if (delta < 0) {
        delta = -delta;
    }

    return si_clamp_u16((uint32_t)delta);
}

/**
 * @brief 仅在 CPU2 处于油位跟随、探头已到液面且液位稳定时返回真。
 *
 * @return 1 表示 CPU2 正处于油位跟随、探头命中液面且结果稳定；任一条件不满足时返回 0。
 */
static uint8_t si_is_probe_follow_stable(void)
{
    return ((g_measurement.device_status.device_state == STATE_FLOWOIL) &&
            (g_measurement.oil_measurement.probe_at_liquid_level != 0U) &&
            (g_measurement.oil_measurement.liquid_stable != 0U)) ? 1U : 0U;
}

/**
 * @brief 把 CPU2 带 200.00 ℃ 零点偏移的 0.01 ℃ 温度原始值转换为 SI 有符号 0.01 ℃ 定点寄存器。
 *
 * 函数先识别 0、9999 和无线温度无效值三类历史哨兵；有效值减去 20000 后限制到 int16_t 可表示范围。
 *
 * @param raw_temperature CPU2 温度原始编码，单位 0.01 ℃ 且包含 +20000 零点偏移；0、9999 和无线无效值均视为无效。
 * @return 无效温度返回 SI_TEMP_INVALID_REGISTER；有效值减去 20000 后饱和到 int16_t 范围，并以 uint16_t 原样返回其有符号 0.01 ℃ 位模式。
 * @note 返回类型为 uint16_t，仅用于原样承载 SI 有符号 16 位温度的寄存器位模式。
 * @note CPU2 温度原始值采用 0.01 K 偏移编码；SI 输出为 0.01 ℃ 的有符号寄存器值。
 */
static uint16_t si_temp_raw_to_si_s16(uint32_t raw_temperature)
{
    /* CPU2 温度原始值为无效哨兵时直接发布 SI 无效寄存器值，不得继续减去 200.00 ℃ 偏移后伪装成有效负温度。 */
    if (si_is_invalid_temp_raw(raw_temperature) != 0U) {
        return SI_TEMP_INVALID_REGISTER;
    }

    return si_clamp_s16((int32_t)raw_temperature - 20000);
}

/**
 * @brief 将 CPU2 密度原始值转换为 SI 0.01 单位密度。
 *
 * 无效值输出 0，超范围按 16 位最大值饱和。
 *
 * @param raw_density CPU2 密度原始值，单位 0.01 kg/m3；UNVALID_DENSITY 表示无有效密度。
 * @return UNVALID_DENSITY 返回 0；有效密度以 0.01 kg/m3 单位返回，超过 16 位范围时饱和为 UINT16_MAX。
 */
static uint16_t si_density_raw_to_si_u16(uint32_t raw_density)
{
    if (raw_density == UNVALID_DENSITY) {
        return 0U;
    }
    return si_clamp_u16(raw_density);
}

/**
 * @brief 判断线圈是否允许 PLC 写入。
 *
 * 只放开会触发明确动作的线圈，未实现或只读状态位返回非法地址。
 *
 * @param offset 相对起始位置的偏移量。
 * @return 1 表示线圈允许 PLC 写入；0 表示线圈不允许 PLC 写入。
 */
static uint8_t si_is_coil_writeable(uint16_t offset)
{
    if (offset <= SI_COIL_PROFILE) {
        return 1U;
    }
    if ((offset >= SI_COIL_STOP) && (offset <= SI_COIL_DOWN_FAST)) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 判断保持寄存器是否允许 PLC 写入。
 *
 * 已确认能落地或作为影子配置保存的地址才允许写入。
 *
 * @param offset 相对起始位置的偏移量。
 * @return 1 表示保持寄存器允许 PLC 写入；0 表示保持寄存器不允许 PLC 写入。
 */
static uint8_t si_is_holding_writeable(uint16_t offset)
{
    if (offset <= SI_HR_PROFILE_DWELL_TIME) {
        return 1U;
    }
    if ((offset >= SI_HR_COMPAT_RAW_40004) &&
        (offset <= SI_HR_COMPAT_RAW_40009)) {
        return 1U;
    }
    if ((offset >= SI_HR_AUTO_PROFILE_INTERVAL) &&
        (offset <= SI_HR_DENSITY_DEVIATION_SETPOINT)) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 按低限和滞回值更新无符号 SI 低报警状态。
 *
 * @param value 本次报警门限判断使用的实时输入值。
 * @param setpoint 控制或报警判断使用的设定值。
 * @return 返回更新后的低限报警状态；1 表示报警激活，0 表示未激活或已越过滞回释放点。
 */
static uint8_t si_low_alarm_u16(uint16_t value, uint16_t setpoint)
{
    return (value < setpoint) ? 1U : 0U;
}

/**
 * @brief 按高限和滞回值更新无符号 SI 高报警状态。
 *
 * @param value 本次报警门限判断使用的实时输入值。
 * @param setpoint 控制或报警判断使用的设定值。
 * @return 返回更新后的高限报警状态；1 表示报警激活，0 表示未激活或已越过滞回释放点。
 */
static uint8_t si_high_alarm_u16(uint16_t value, uint16_t setpoint)
{
    return (value > setpoint) ? 1U : 0U;
}

/**
 * @brief 按低限和滞回值更新有符号 SI 低报警状态。
 *
 * @param value 本次报警门限判断使用的实时输入值。
 * @param setpoint 控制或报警判断使用的设定值。
 * @return 返回更新后的有符号低限报警状态；1 表示报警激活，0 表示未激活或已释放。
 */
static uint8_t si_low_alarm_s16(int16_t value, uint16_t setpoint)
{
    return (value < si_holding_to_s16(setpoint)) ? 1U : 0U;
}

/**
 * @brief 按高限和滞回值更新有符号 SI 高报警状态。
 *
 * @param value 本次报警门限判断使用的实时输入值。
 * @param setpoint 控制或报警判断使用的设定值。
 * @return 返回更新后的有符号高限报警状态；1 表示报警激活，0 表示未激活或已释放。
 */
static uint8_t si_high_alarm_s16(int16_t value, uint16_t setpoint)
{
    return (value > si_holding_to_s16(setpoint)) ? 1U : 0U;
}

/**
 * @brief 判断SI协议剖面结果有效性。
 *
 * @return final_snapshot_ready 非 0 时返回 1，表示已有可对外发布的 SI 最终剖面快照；否则返回 0。
 */
static uint8_t si_is_si_profile_result_valid(void)
{
    return (s_profile_projection.final_snapshot_ready != 0U) ? 1U : 0U;
}

/**
 * @brief 把 SI Profile 点数限制到协议和缓存共同支持的容量。
 *
 * @param points 待限制的 SI 剖面测点数量；超过协议允许上限时钳位为最大测点数。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static uint16_t si_profile_clamp_points(uint32_t points)
{
    if (points > MAX_MEASUREMENT_POINTS) {
        return MAX_MEASUREMENT_POINTS;
    }

    return (uint16_t)points;
}

/**
 * @brief 比较 SI 候选的周期、完成计数、点数、来源和阶段是否一致。
 *
 * @param left 区间左端值或左侧比较对象。
 * @param right 区间右端值或右侧比较对象。
 * @return 1 表示两个候选键的周期计数、完成计数、点数、来源和阶段全部相等；0 表示任一字段不同。
 */
static uint8_t si_profile_key_equal(const Cpu2SiProfileCandidateKey *left,
                                    const Cpu2SiProfileCandidateKey *right)
{
    return ((left->cycle_counter == right->cycle_counter) &&
            (left->complete_counter == right->complete_counter) &&
            (left->measurement_points == right->measurement_points) &&
            (left->profile_source == right->profile_source) &&
            (left->phase == right->phase)) ? 1U : 0U;
}

/**
 * @brief 从 CPU2 共享快照组合当前 SI 候选代际键。
 *
 * @return 返回由候选完成计数、周期、阶段、点数和来源组成的 Cpu2SiProfileCandidateKey。
 */
static Cpu2SiProfileCandidateKey si_profile_current_key(void)
{
    Cpu2SiProfileCandidateKey key;

    key.cycle_counter = g_measurement.si_profile_runtime.cycle_counter;
    key.complete_counter = g_measurement.density_distribution.profile_complete_counter;
    key.measurement_points = g_measurement.density_distribution.measurement_points;
    key.profile_source = g_measurement.density_distribution.profile_source;
    key.phase = g_measurement.si_profile_runtime.phase;
    return key;
}

/**
 * @brief 设备故障或 Profile 被工艺阻断时报告互锁有效。
 *
 * @return 1 表示设备存在故障或 SI Profile 被工艺阻断；两项均未激活时返回 0。
 */
static uint8_t si_profile_interlock_active(void)
{
    return ((g_measurement.device_status.error_code != NO_ERROR) ||
            (g_measurement.density_distribution.profile_blocked_by_process != 0U)) ? 1U : 0U;
}

/**
 * @brief 将FC01或FC02连续状态位按Modbus低位优先规则打包成一个镜像寄存器。
 *
 * @param bits 至少包含 start 后 16 个离散状态元素的只读数组。
 * @param start 本次连续处理范围的起始索引。该值是 SI 位池中的起始位号，函数从该位开始按 count 打包为 Modbus 位响应。
 * @return 返回从 start_bit 开始的连续状态位按 Modbus 低位优先规则打包得到的 16 位镜像值。
 */
static uint16_t si_pack_bits_u16(const uint8_t *bits, uint16_t start)
{
    uint16_t value = 0U;
    uint16_t i;

    for (i = 0U; i < 16U; ++i) {
        if (bits[start + i] != 0U) {
            value |= (uint16_t)(1UL << i);
        }
    }

    return value;
}

/**
 * @brief 清除 SI 剖面结果时间戳及其有效标志。
 */
static void si_invalidate_profile_timestamp(void)
{
    memset(&s_profile_timestamp, 0, sizeof(s_profile_timestamp));
    s_profile_timestamp_valid = 0U;
}

/**
 * @brief 读取 CPU3 RTC 锁存 Profile 时间；失败时标记时间戳无效。
 */
static void si_lock_profile_timestamp_now(void)
{
    if (Cpu3Clock_GetDateTime(&s_profile_timestamp) != 0U) {
        s_profile_timestamp_valid = 1U;
    } else {
        s_profile_timestamp_valid = 0U;
    }
}

/**
 * @brief 清除当前 SI Profile 候选的温差、密度差和高低限报警。
 */
static void si_profile_clear_alarms(void)
{
    s_profile_projection.profile_temp_deviation_alarm = 0U;
    s_profile_projection.profile_density_deviation_alarm = 0U;
    s_profile_projection.profile_low_temp_alarm = 0U;
    s_profile_projection.profile_high_temp_alarm = 0U;
    s_profile_projection.profile_low_density_alarm = 0U;
    s_profile_projection.profile_high_density_alarm = 0U;
}

/**
 * @brief 清除CPU3本地发布结果；调用方单独决定是否保留Point0时间和活动进度。
 */
static void si_profile_clear_result(void)
{
    s_profile_projection.candidate_valid = 0U;
    s_profile_projection.final_snapshot_ready = 0U;
    s_profile_projection.published_key_valid = 0U;
    s_profile_projection.fetch_attempt_valid = 0U;
    s_profile_projection.final_points = 0U;
    memset(s_profile_projection.point_regs, 0, sizeof(s_profile_projection.point_regs));
    si_profile_clear_alarms();
}

/**
 * @brief 把已通过代际复核的CPU2候选转换为SI寄存器格式，并在本地计算六项Profile报警。
 *
 * @param key 候选 profile 的完成锁存、完成计数、测点数和来源组合键。
 */
static void si_profile_cache_candidate(const Cpu2SiProfileCandidateKey *key)
{
    uint16_t points = si_profile_clamp_points(key->measurement_points);
    uint16_t i;

    memset(s_profile_projection.point_regs, 0, sizeof(s_profile_projection.point_regs));
    si_profile_clear_alarms();

    for (i = 0U; i < points; ++i) {
        const volatile DensityMeasurement *point =
            &g_measurement.density_distribution.single_density_data[i];
        uint16_t base = (uint16_t)(i * 3U);
        uint8_t temp_valid = (si_is_invalid_temp_raw(point->temperature) == 0U) ? 1U : 0U;
        uint8_t density_valid = (point->density != UNVALID_DENSITY) ? 1U : 0U;
        uint16_t density = si_density_raw_to_si_u16(point->density);

        s_profile_projection.point_regs[base] =
            si_u01mm_to_mm_u16(point->temperature_position);
        s_profile_projection.point_regs[base + 1U] =
            si_temp_raw_to_si_s16(point->temperature);
        s_profile_projection.point_regs[base + 2U] = density;

        if (temp_valid != 0U) {
            int16_t temperature = (int16_t)s_profile_projection.point_regs[base + 1U];

            if (temperature < g_cpu3_comm_display_params.si_low_temperature_setpoint) {
                s_profile_projection.profile_low_temp_alarm = 1U;
            }
            if (temperature > g_cpu3_comm_display_params.si_high_temperature_setpoint) {
                s_profile_projection.profile_high_temp_alarm = 1U;
            }
        }
        if (density_valid != 0U) {
            if (density < g_cpu3_comm_display_params.si_low_density_setpoint) {
                s_profile_projection.profile_low_density_alarm = 1U;
            }
            if (density > g_cpu3_comm_display_params.si_high_density_setpoint) {
                s_profile_projection.profile_high_density_alarm = 1U;
            }
        }

        if (i > 0U) {
            const volatile DensityMeasurement *previous =
                &g_measurement.density_distribution.single_density_data[i - 1U];

            if ((temp_valid != 0U) &&
                (si_is_invalid_temp_raw(previous->temperature) == 0U) &&
                (si_absdiff_s16((int16_t)si_temp_raw_to_si_s16(point->temperature),
                                (int16_t)si_temp_raw_to_si_s16(previous->temperature)) >
                 g_cpu3_comm_display_params.si_temp_deviation_setpoint)) {
                s_profile_projection.profile_temp_deviation_alarm = 1U;
            }
            if ((density_valid != 0U) &&
                (previous->density != UNVALID_DENSITY) &&
                (si_absdiff_u16(density,
                                si_density_raw_to_si_u16(previous->density)) >
                 g_cpu3_comm_display_params.si_density_deviation_setpoint)) {
                s_profile_projection.profile_density_deviation_alarm = 1U;
            }
        }
    }

    s_profile_projection.candidate_key = *key;
    s_profile_projection.candidate_valid = 1U;
}

/**
 * @brief 仅在候选完成、探头稳定、电机停止且无互锁时开放最终发布门禁。
 *
 * @return 1 表示候选完成、探头稳定、电机停止且无互锁，允许最终发布；否则返回 0。
 */
static uint8_t si_profile_final_gate_open(void)
{
    return ((s_profile_projection.candidate_valid != 0U) &&
            (s_profile_projection.candidate_key.phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE) &&
            (g_measurement.oil_measurement.probe_at_liquid_level != 0U) &&
            (g_measurement.oil_measurement.liquid_stable != 0U) &&
            (g_measurement.debug_data.motor_state == 0U) &&
            (si_profile_interlock_active() == 0U)) ? 1U : 0U;
}

/**
 * @brief 判断当前 SI Profile 阶段是否仍属于活动流程。
 *
 * @param phase 当前状态机阶段。
 * @return 1 表示当前 SI Profile 阶段仍属于活动流程；0 表示当前 SI Profile 阶段已不再属于活动流程。
 */
static uint8_t si_profile_phase_is_active(uint32_t phase)
{
    if ((phase == (uint32_t)SI_PROFILE_PHASE_PREPARING) ||
        (phase == (uint32_t)SI_PROFILE_PHASE_MEASURING) ||
        (phase == (uint32_t)SI_PROFILE_PHASE_RETURNING_LEVEL)) {
        return 1U;
    }
    if ((phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE) &&
        (s_profile_projection.final_snapshot_ready == 0U)) {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 判断当前共享头是否仍表示本周期可拉取的SI完成候选。
 *
 * 普通分布测量会复用共享缓冲区，来源或代际不匹配时不得把旧SI阶段投影成Profile运行态。
 *
 * @return 1 表示当前共享头仍表示本周期可拉取的SI完成候选；0 表示当前共享头已不再表示本周期可拉取的SI完成候选。
 */
static uint8_t si_profile_current_complete_candidate_is_si(void)
{
    Cpu2SiProfileCandidateKey current = si_profile_current_key();

    return ((current.cycle_counter == s_profile_projection.active_cycle) &&
            ((s_profile_projection.complete_baseline_valid == 0U) ||
             (current.complete_counter != s_profile_projection.cycle_complete_baseline)) &&
            (current.profile_source == (uint32_t)PROFILE_SOURCE_SI) &&
            (current.phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE) &&
            (current.measurement_points > 0U) &&
            (current.measurement_points <= MAX_MEASUREMENT_POINTS) &&
            (g_measurement.density_distribution.profile_complete_latched != 0U)) ? 1U : 0U;
}

/**
 * @brief Point0前只有PREPARING及其取消/失败终态允许保留上一轮完整SI结果。
 *
 * @param phase 当前状态机阶段。
 * @return 1 表示当前处于 Point0 前的 PREPARING、ABORTED 或 FAILED 阶段，可保留上一轮结果；否则返回 0。
 */
static uint8_t si_profile_phase_keeps_previous_result(uint32_t phase)
{
    return ((phase == (uint32_t)SI_PROFILE_PHASE_PREPARING) ||
            (phase == (uint32_t)SI_PROFILE_PHASE_ABORTED) ||
            (phase == (uint32_t)SI_PROFILE_PHASE_FAILED)) ? 1U : 0U;
}

/**
 * @brief 判断FC01是否仍需输出本轮SI最终组合。
 *
 * 已发布结果可以长期保留，但只有共享代际未被其它分布测量覆盖、且设备仍处于SI收尾上下文时才覆盖实时线圈。
 *
 * @return 1 表示FC01仍需输出本轮SI最终组合；0 表示FC01已不再需输出本轮SI最终组合。
 */
static uint8_t si_profile_final_coil_projection_allowed(void)
{
    Cpu2SiProfileCandidateKey current;
    CommandType command = g_measurement.device_status.current_command;

    if ((s_profile_projection.final_snapshot_ready == 0U) ||
        (s_profile_projection.published_key_valid == 0U) ||
        (s_profile_projection.last_phase != (uint32_t)SI_PROFILE_PHASE_COMPLETE)) {
        return 0U;
    }

    current = si_profile_current_key();
    if (si_profile_key_equal(&current, &s_profile_projection.published_key) == 0U) {
        return 0U;
    }

    if ((command != CMD_SI_PROFILE) &&
        (command != CMD_FIND_OIL) &&
        ((command != CMD_NONE) ||
         (g_measurement.device_status.device_state != STATE_FLOWOIL))) {
        return 0U;
    }

    return ((g_measurement.oil_measurement.probe_at_liquid_level != 0U) &&
            (g_measurement.oil_measurement.liquid_stable != 0U) &&
            (g_measurement.debug_data.motor_state == 0U) &&
            (si_profile_interlock_active() == 0U)) ? 1U : 0U;
}

/**
 * @brief 校验并发布一组完整的 SI 剖面候选结果。
 */
static void si_profile_publish_candidate(void)
{
    s_profile_projection.final_points =
        si_profile_clamp_points(s_profile_projection.candidate_key.measurement_points);
    s_profile_projection.progress_points = s_profile_projection.final_points;
    s_profile_projection.published_key = s_profile_projection.candidate_key;
    s_profile_projection.published_key_valid = 1U;
    s_profile_projection.final_snapshot_ready = 1U;
    /* 候选已经转为不可变发布快照，后续共享分布头变化不得再走候选清空路径。 */
    s_profile_projection.candidate_valid = 0U;
    s_profile_projection.cycle_complete_baseline =
        s_profile_projection.candidate_key.complete_counter;
    s_profile_projection.complete_baseline_valid = 1U;
    s_profile_projection.cycle_established = 0U;
}

/**
 * @brief CPU3冷启动后尝试重建Point0前仍留在CPU2共享区的上一轮SI快照。
 *
 * 只恢复Complete、N、点阵和报警；本函数不锁存RTC，无法重建的Profile时间继续保持0。
 *
 * @param phase 当前状态机阶段。
 * @param allow_fetch 允许。
 */
static void si_profile_try_restore_previous_snapshot(uint32_t phase, uint8_t allow_fetch)
{
    Cpu2SiProfileCandidateKey current;
    Cpu2SiProfileCandidateKey fetched;
    uint32_t now_tick;

    if ((allow_fetch == 0U) ||
        (s_profile_projection.final_snapshot_ready != 0U) ||
        (s_profile_projection.complete_baseline_valid == 0U) ||
        (si_profile_phase_keeps_previous_result(phase) == 0U)) {
        return;
    }

    current = si_profile_current_key();
    if ((current.cycle_counter != s_profile_projection.active_cycle) ||
        (current.complete_counter != s_profile_projection.cycle_complete_baseline) ||
        (current.profile_source != (uint32_t)PROFILE_SOURCE_SI) ||
        (current.phase != phase) ||
        (current.measurement_points == 0U) ||
        (current.measurement_points > MAX_MEASUREMENT_POINTS) ||
        (g_measurement.density_distribution.profile_complete_latched == 0U)) {
        return;
    }

    now_tick = HAL_GetTick();
    if ((s_profile_projection.fetch_attempt_valid != 0U) &&
        (s_profile_projection.fetch_cycle == current.cycle_counter) &&
        (s_profile_projection.fetch_complete_counter == current.complete_counter) &&
        ((now_tick - s_profile_projection.last_fetch_tick) < SI_PROFILE_FETCH_RETRY_DELAY_MS)) {
        return;
    }

    s_profile_projection.fetch_attempt_valid = 1U;
    s_profile_projection.fetch_cycle = current.cycle_counter;
    s_profile_projection.fetch_complete_counter = current.complete_counter;
    s_profile_projection.last_fetch_tick = now_tick;

    if (CPU2_CommFetchSiPreviousSnapshot(&fetched) &&
        (fetched.cycle_counter == s_profile_projection.active_cycle) &&
        (fetched.complete_counter == s_profile_projection.cycle_complete_baseline) &&
        (fetched.profile_source == (uint32_t)PROFILE_SOURCE_SI) &&
        (fetched.phase == phase) &&
        (fetched.measurement_points > 0U) &&
        (fetched.measurement_points <= MAX_MEASUREMENT_POINTS) &&
        (g_measurement.density_distribution.profile_complete_latched != 0U)) {
        si_profile_cache_candidate(&fetched);
        si_profile_publish_candidate();
    }
}

/**
 * @brief 更新CPU3本地SI生命周期投影。
 *
 * allow_fetch仅在周期任务中置1，外部Modbus响应路径只做轻量状态更新。
 *
 * @param allow_fetch 仅在周期任务中置1，外部Modbus响应路径只做轻量状态更新。
 */
static void si_update_profile_projection(uint8_t allow_fetch)
{
    Cpu2SiProfileCandidateKey current;
    uint32_t phase;
    uint32_t cycle;
    uint32_t complete_counter;
    uint32_t snapshot_generation;

    if (!CPU2_CommHasRuntimeSnapshot()) {
        s_profile_projection.connected = 0U;
        return;
    }

    phase = g_measurement.si_profile_runtime.phase;
    cycle = g_measurement.si_profile_runtime.cycle_counter;
    complete_counter = g_measurement.density_distribution.profile_complete_counter;
    snapshot_generation = CPU2_CommGetSnapshotGeneration();

    if (s_profile_projection.initialized == 0U) {
        s_profile_projection.initialized = 1U;
        s_profile_projection.connected = 1U;
        s_profile_projection.cpu2_snapshot_generation = snapshot_generation;
        s_profile_projection.active_cycle = cycle;
        s_profile_projection.last_phase = phase;
        s_profile_projection.cycle_complete_baseline = complete_counter;
        s_profile_projection.complete_baseline_valid =
            (phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE) ? 0U : 1U;
        s_profile_projection.cycle_established =
            ((phase == (uint32_t)SI_PROFILE_PHASE_MEASURING) ||
             (phase == (uint32_t)SI_PROFILE_PHASE_RETURNING_LEVEL) ||
             (phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE)) ? 1U : 0U;
        s_profile_projection.progress_points =
            si_profile_clamp_points(g_measurement.si_profile_runtime.progress_points);
        si_profile_clear_result();
        si_invalidate_profile_timestamp();
    } else if ((s_profile_projection.connected == 0U) ||
               (snapshot_generation != s_profile_projection.cpu2_snapshot_generation)) {
        /* CPU2快照会话已切换，旧SI点阵、Complete和时间均不得跨会话复用。 */
        s_profile_projection.connected = 1U;
        s_profile_projection.cpu2_snapshot_generation = snapshot_generation;
        s_profile_projection.active_cycle = cycle;
        s_profile_projection.cycle_complete_baseline = complete_counter;
        s_profile_projection.complete_baseline_valid =
            (phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE) ? 0U : 1U;
        s_profile_projection.cycle_established =
            ((phase == (uint32_t)SI_PROFILE_PHASE_MEASURING) ||
             (phase == (uint32_t)SI_PROFILE_PHASE_RETURNING_LEVEL) ||
             (phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE)) ? 1U : 0U;
        s_profile_projection.progress_points =
            si_profile_clamp_points(g_measurement.si_profile_runtime.progress_points);
        si_profile_clear_result();
        si_invalidate_profile_timestamp();
        s_profile_projection.last_phase = phase;
    } else {
        if ((phase == (uint32_t)SI_PROFILE_PHASE_PREPARING) &&
            (s_profile_projection.last_phase != (uint32_t)SI_PROFILE_PHASE_PREPARING)) {
            s_profile_projection.cycle_complete_baseline = complete_counter;
            s_profile_projection.complete_baseline_valid = 1U;
            s_profile_projection.cycle_established = 0U;
            s_profile_projection.candidate_valid = 0U;
            s_profile_projection.fetch_attempt_valid = 0U;
        }

        if (cycle != s_profile_projection.active_cycle) {
            s_profile_projection.active_cycle = cycle;
            s_profile_projection.cycle_established = 1U;
            s_profile_projection.progress_points =
                si_profile_clamp_points(g_measurement.si_profile_runtime.progress_points);
            si_profile_clear_result();
            si_invalidate_profile_timestamp();
            si_lock_profile_timestamp_now();
            if ((s_profile_projection.complete_baseline_valid == 0U) &&
                (phase != (uint32_t)SI_PROFILE_PHASE_COMPLETE)) {
                s_profile_projection.cycle_complete_baseline = complete_counter;
                s_profile_projection.complete_baseline_valid = 1U;
            }
        }
    }

    si_profile_try_restore_previous_snapshot(phase, allow_fetch);

    if ((phase == (uint32_t)SI_PROFILE_PHASE_MEASURING) ||
        (phase == (uint32_t)SI_PROFILE_PHASE_RETURNING_LEVEL) ||
        (phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE)) {
        uint16_t progress =
            si_profile_clamp_points(g_measurement.si_profile_runtime.progress_points);

        if (progress > s_profile_projection.progress_points) {
            s_profile_projection.progress_points = progress;
        }
    }

    if (((phase == (uint32_t)SI_PROFILE_PHASE_ABORTED) ||
         (phase == (uint32_t)SI_PROFILE_PHASE_FAILED)) &&
        (s_profile_projection.cycle_established != 0U)) {
        si_profile_clear_result();
        s_profile_projection.progress_points = 0U;
        s_profile_projection.cycle_established = 0U;
    }

    if (phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE) {
        uint32_t now_tick;
        uint8_t fetch_due = 0U;

        current = si_profile_current_key();
        /*
         * 已发布SI快照只由下一轮SI Point0或本轮取消/失败清除。
         * 普通分布测量会复用共享分布头并改变完成计数/来源，不能据此撤销已确认的SI结果。
         */
        if ((s_profile_projection.final_snapshot_ready == 0U) &&
            (s_profile_projection.candidate_valid != 0U) &&
            (si_profile_key_equal(&current, &s_profile_projection.candidate_key) == 0U)) {
            s_profile_projection.candidate_valid = 0U;
            memset(s_profile_projection.point_regs, 0, sizeof(s_profile_projection.point_regs));
            si_profile_clear_alarms();
        }

        if ((current.cycle_counter == s_profile_projection.active_cycle) &&
            ((s_profile_projection.complete_baseline_valid == 0U) ||
             (current.complete_counter != s_profile_projection.cycle_complete_baseline)) &&
            (current.profile_source == (uint32_t)PROFILE_SOURCE_SI) &&
            (current.measurement_points > 0U) &&
            (current.measurement_points <= MAX_MEASUREMENT_POINTS) &&
            (g_measurement.density_distribution.profile_complete_latched != 0U) &&
            (s_profile_projection.final_snapshot_ready == 0U) &&
            ((s_profile_projection.candidate_valid == 0U) ||
             (si_profile_key_equal(&current, &s_profile_projection.candidate_key) == 0U)) &&
            (allow_fetch != 0U)) {
            now_tick = HAL_GetTick();
            if ((s_profile_projection.fetch_attempt_valid == 0U) ||
                (s_profile_projection.fetch_cycle != current.cycle_counter) ||
                (s_profile_projection.fetch_complete_counter != current.complete_counter) ||
                ((now_tick - s_profile_projection.last_fetch_tick) >= SI_PROFILE_FETCH_RETRY_DELAY_MS)) {
                fetch_due = 1U;
            }

            if (fetch_due != 0U) {
                Cpu2SiProfileCandidateKey fetched;

                s_profile_projection.fetch_attempt_valid = 1U;
                s_profile_projection.fetch_cycle = current.cycle_counter;
                s_profile_projection.fetch_complete_counter = current.complete_counter;
                s_profile_projection.last_fetch_tick = now_tick;
                if (CPU2_CommFetchSiProfileCandidate(&fetched) &&
                    (fetched.cycle_counter == s_profile_projection.active_cycle)) {
                    si_profile_cache_candidate(&fetched);
                }
            }
        }

        if ((s_profile_projection.final_snapshot_ready == 0U) &&
            (si_profile_final_gate_open() != 0U)) {
            si_profile_publish_candidate();
        }
    }

    s_profile_projection.last_phase = phase;
}

/**
 * @brief 把 SI 控制命令转交 CPU2，并返回板间确认结果。
 *
 * @param cmd 命令值。该 CommandType 将通过 CPU2 板间保持寄存器下发，并由函数把同步结果映射为 SI Modbus 异常码。
 * @return true 表示 CPU2 返回合法写响应，false 表示本次请求失败。
 */
static bool si_send_cpu2_command(CommandType cmd)
{
    uint32_t cmd32 = (uint32_t)cmd;
    bool request_ok;

    if (!CPU2_CommCanSendCommand(cmd)) {
        return false;
    }

    /* 通过 CPU2 既有命令保持寄存器下发，CPU3 不直接改 CPU2 状态机。 */
    request_ok = CPU2_CombinatePackage_Send(FUNCTIONCODE_WRITE_MULREGISTER,
                                            HOLDREGISTER_DEVICEPARAM_COMMAND,
                                            2U,
                                            &cmd32);
    g_deviceParams.command = CMD_NONE;
    return request_ok;
}

/**
 * @brief 请求启动 SI Profile 测量。
 *
 * 调用场景：外部 00004 Profile 线圈、自动 profile 调度和 CPU3 屏幕
 * SI Profile 菜单入口共用。
 * 关键约束：这里只下发命令；Profile Timestamp必须等CPU2发布Point0 cycle事件后锁存。
 *
 * @return true 表示 CMD_SI_PROFILE 已通过当前 CPU2 命令通道接受；false 表示CPU2 通信不可用、命令门禁拒绝、Modbus 异常或命令 ACK 无效。
 */
bool si_profile_request_start(void)
{
    return si_send_cpu2_command(CMD_SI_PROFILE);
}

/**
 * @brief 将一个 32 位 SI 参数写入 CPU2 共享保持寄存器；仅在 CPU2 确认成功后更新 CPU3 影子，参数快照由板间写入口失效并补读。
 *
 * @param hold_addr 地址参数。该值是 CPU2 共享保持寄存器中的目标参数起始地址，32 位数据按高字在前写入连续两个寄存器。
 * @param shadow CPU3 已确认参数影子；仅在 CPU2 接受新值后更新。
 * @param value 待处理的一个 32 位 SI 参数。
 * @return true 表示 CPU2 已接受参数，false 表示链路不可用或本次请求失败。
 */
static bool si_write_device_param_u32(uint16_t hold_addr,
                                      volatile uint32_t *shadow,
                                      uint32_t value)
{
    uint16_t wire_regs[2];

    wire_regs[0] = (uint16_t)(value >> 16);
    wire_regs[1] = (uint16_t)(value & 0xFFFFU);

    if (!CPU2_CommIsAvailable() ||
        !CPU2_CommWriteHoldingRegisters(hold_addr,
                                        2U,
                                        wire_regs)) {
        return false;
    }

    if (shadow != NULL) {
        /*
         * 只有 CPU2 确认写入后才提交 CPU3 影子；通用桥接入口同时会让参数
         * 快照失效并主动补读，补读完成前后续参数读写不会把请求影子当成实值。
         */
        *shadow = value;
    }
    return true;
}

/**
 * @brief 把 SI 保持寄存器 40010～40023 映射为 CPU3 本地参数编号。
 *
 * @param offset 相对起始位置的偏移量。
 * @return 返回 40010 至 40023 偏移对应的 CPU3 SI 参数操作号；偏移越界返回 COM_NUM_NOOPERA。
 */
static OperatingNumber si_holding_offset_to_cpu3_param(uint16_t offset)
{
    switch (offset) {
    case SI_HR_AUTO_PROFILE_INTERVAL:
        return COM_NUM_CPU3_SI_AUTO_PROFILE_INTERVAL;
    case SI_HR_AUTO_PROFILE_ENABLE:
        return COM_NUM_CPU3_SI_AUTO_PROFILE_ENABLE;
    case SI_HR_AUTO_PROFILE_HOUR:
        return COM_NUM_CPU3_SI_AUTO_PROFILE_HOUR;
    case SI_HR_AUTO_PROFILE_MINUTE:
        return COM_NUM_CPU3_SI_AUTO_PROFILE_MINUTE;
    case SI_HR_LOW_DENSITY_SETPOINT:
        return COM_NUM_CPU3_SI_LOW_DENSITY_SETPOINT;
    case SI_HR_HIGH_DENSITY_SETPOINT:
        return COM_NUM_CPU3_SI_HIGH_DENSITY_SETPOINT;
    case SI_HR_LOW_TEMPERATURE_SETPOINT:
        return COM_NUM_CPU3_SI_LOW_TEMPERATURE_SETPOINT;
    case SI_HR_HIGH_TEMPERATURE_SETPOINT:
        return COM_NUM_CPU3_SI_HIGH_TEMPERATURE_SETPOINT;
    case SI_HR_LL_LEVEL_SETPOINT:
        return COM_NUM_CPU3_SI_LL_LEVEL_SETPOINT;
    case SI_HR_HH_LEVEL_SETPOINT:
        return COM_NUM_CPU3_SI_HH_LEVEL_SETPOINT;
    case SI_HR_LOW_LEVEL_SETPOINT:
        return COM_NUM_CPU3_SI_LOW_LEVEL_SETPOINT;
    case SI_HR_HIGH_LEVEL_SETPOINT:
        return COM_NUM_CPU3_SI_HIGH_LEVEL_SETPOINT;
    case SI_HR_TEMP_DEVIATION_SETPOINT:
        return COM_NUM_CPU3_SI_TEMP_DEVIATION_SETPOINT;
    case SI_HR_DENSITY_DEVIATION_SETPOINT:
        return COM_NUM_CPU3_SI_DENSITY_DEVIATION_SETPOINT;
    default:
        return COM_NUM_NOOPERA;
    }
}

/**
 * @brief 构造 SI Modbus 异常响应帧并返回帧长。
 *
 * @param func 原请求的 SI Modbus 功能码；函数置位最高位后写入异常响应。
 * @param ex_code 待写入 Modbus 异常响应的异常码。
 * @param tx SI 标准 Modbus 异常响应输出缓冲区，调用方至少提供 5 个可写字节。
 * @param tx_len 数据长度。该输出指针在 SI Modbus 异常帧构造完成后写入包含 CRC 的总字节数。
 * @return 固定返回 1，表示从机地址、置位最高位的原功能码、异常码和 CRC16 组成的 5 字节 Modbus 异常响应已写入 tx，且 *tx_len 已更新为 5。
 */
static uint8_t si_build_exception(uint8_t func,
                                      uint8_t ex_code,
                                      uint8_t *tx,
                                      uint16_t *tx_len)
{
    /* Modbus 异常响应为功能码 OR 0x80 + 异常码，仍带标准 CRC。 */
    tx[0] = si_get_effective_slave_address();
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

/**
 * @brief 构造 FC05/FC06 写单点的标准回显帧。
 *
 * 明确按功能码、地址和值重建回包，避免依赖请求 PDU 前一个字节。
 *
 * @param func 需要回显的 SI Modbus FC05 或 FC06 功能码。
 * @param offset 相对起始位置的偏移量。
 * @param value 构建写入使用的输入数值。
 * @param tx SI FC05/FC06 标准 8 字节回显帧输出缓冲区。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 */
static void si_build_write_echo(uint8_t func,
                                    uint16_t offset,
                                    uint16_t value,
                                    uint8_t *tx,
                                    uint16_t *tx_len)
{
    tx[0] = si_get_effective_slave_address();
    tx[1] = func;
    si_wr_be16(&tx[2], offset);
    si_wr_be16(&tx[4], value);

    {
        uint16_t crc = CRC16_Calculate(tx, 6);
        tx[6] = (uint8_t)(crc & 0xFFU);
        tx[7] = (uint8_t)(crc >> 8);
    }
    *tx_len = 8U;
}

/**
 * @brief 校验 FC06 写保持寄存器的基础值域。
 *
 * 仅检查协议层确定无歧义的字段，其余影子寄存器保持宽松，避免误拦 PLC 预留配置。
 *
 * @param offset 相对起始位置的偏移量。
 * @param value 保持寄存器数值有效使用的输入数值。
 * @return 1 表示目标保持寄存器值满足对应启用、时间或范围约束；非法值返回 0；未设专用约束的寄存器返回 1。
 */
static uint8_t si_is_holding_value_valid(uint16_t offset, uint16_t value)
{
    switch (offset) {
    case SI_HR_PROFILE_FIRST_POINT:
    case SI_HR_PROFILE_INCREMENT:
    case SI_HR_AUTO_PROFILE_INTERVAL:
        return (value != 0U) ? 1U : 0U;
    case SI_HR_PROFILE_DWELL_TIME:
        return ((value >= 1U) &&
                (value <= SI_PROFILE_DWELL_TIME_MAX_S)) ? 1U : 0U;
    case SI_HR_AUTO_PROFILE_ENABLE:
        return (value <= 1U) ? 1U : 0U;
    case SI_HR_AUTO_PROFILE_HOUR:
        return (value <= 23U) ? 1U : 0U;
    case SI_HR_AUTO_PROFILE_MINUTE:
        return (value <= 59U) ? 1U : 0U;
    default:
        return 1U;
    }
}

/**
 * @brief 根据 CPU2 当前状态刷新 SI 线圈快照。
 *
 * @note PLC 写线圈只作为命令入口，后续读回值始终以 CPU2 真实状态为准。
 */
static void si_refresh_coils_from_state(void)
{
    memset(s_coils, 0, sizeof(s_coils));

    /* 运行模式线圈由 CPU2 当前测量状态反推，PLC 写入只作为命令入口。 */
    switch (g_measurement.device_status.device_state) {
    case STATE_SPREADPOINTING:
    case STATE_GB_SPREADPOINTING:
    case STATE_METER_DENSITY:
    case STATE_INTERVAL_DENSITY:
    case STATE_WARTSILA_DENSITY_MEASURING:
        if (g_measurement.device_status.current_command == CMD_SI_PROFILE) {
            s_coils[SI_COIL_PROFILE] = 1U;
        }
        break;

    case STATE_CALIBRATIONOILING:
    case STATE_CALIBRATE_WATERING:
    case STATE_CALIBRATE_TANKHEIGHTING:
        s_coils[SI_COIL_CALIBRATE] = 1U;
        break;

    case STATE_FLOWOIL:
        s_coils[SI_COIL_AUTO] = 1U;
        break;

    default:
        s_coils[SI_COIL_MANUAL] = 1U;
        break;
    }

    /* 手动方向/停止线圈以电机实时状态为准，避免动作结束后仍显示上/下行。 */
    switch (g_measurement.debug_data.motor_state) {
    case 1U:
        s_coils[SI_COIL_UP_MEDIUM] = 1U;
        break;
    case 2U:
        s_coils[SI_COIL_DOWN_MEDIUM] = 1U;
        break;
    default:
        s_coils[SI_COIL_STOP] = 1U;
        break;
    }

    /* 生命周期投影覆盖通用状态，避免CPU2候选Complete提前穿透为最终组合。 */
    if ((si_profile_phase_is_active(s_profile_projection.last_phase) != 0U) &&
        ((s_profile_projection.last_phase != (uint32_t)SI_PROFILE_PHASE_COMPLETE) ||
         (si_profile_current_complete_candidate_is_si() != 0U))) {
        s_coils[SI_COIL_MANUAL] = 0U;
        s_coils[SI_COIL_CALIBRATE] = 0U;
        s_coils[SI_COIL_AUTO] = 0U;
        s_coils[SI_COIL_PROFILE] = 1U;
    } else if (si_profile_final_coil_projection_allowed() != 0U) {
        s_coils[SI_COIL_MANUAL] = 0U;
        s_coils[SI_COIL_CALIBRATE] = 0U;
        s_coils[SI_COIL_AUTO] = 1U;
        s_coils[SI_COIL_PROFILE] = 0U;
        memset(&s_coils[SI_COIL_STOP],
               0,
               (size_t)(SI_COIL_DOWN_FAST - SI_COIL_STOP + 1U));
        s_coils[SI_COIL_STOP] = 1U;
    } else if ((s_profile_projection.last_phase == (uint32_t)SI_PROFILE_PHASE_ABORTED) ||
               (s_profile_projection.last_phase == (uint32_t)SI_PROFILE_PHASE_FAILED)) {
        s_coils[SI_COIL_PROFILE] = 0U;
        /* 终态只在电机真实停止时合成Stop，避免旧SI失败阶段遮住后续非SI上/下行动作。 */
        if (g_measurement.debug_data.motor_state == 0U) {
            memset(&s_coils[SI_COIL_STOP],
                   0,
                   (size_t)(SI_COIL_DOWN_FAST - SI_COIL_STOP + 1U));
            s_coils[SI_COIL_STOP] = 1U;
        }
    }
}

/**
 * @brief 根据测量结果和影子阈值刷新 SI 离散输入。
 *
 * 该函数不下发 CPU2 命令，只做 CPU3 外部协议状态转换。
 */
static void si_refresh_discrete_inputs_from_state(void)
{
    uint16_t current_density;
    uint16_t liquid_level;
    uint8_t current_density_valid;
    uint8_t liquid_level_valid;
    uint8_t current_temp_valid;
    int16_t current_temp;

    current_density = si_density_raw_to_si_u16(g_measurement.single_point_monitoring.density);
    liquid_level = si_u01mm_to_mm_u16(g_measurement.oil_measurement.oil_level);
    current_density_valid = (g_measurement.single_point_monitoring.density != UNVALID_DENSITY) ? 1U : 0U;
    liquid_level_valid = (g_measurement.oil_measurement.oil_level != UNVALID_LEVEL) ? 1U : 0U;
    current_temp_valid = (si_is_invalid_temp_raw(g_measurement.debug_data.temperature) == 0U) ? 1U : 0U;
    current_temp = (int16_t)si_temp_raw_to_si_s16(g_measurement.debug_data.temperature);

    memset(s_discrete_inputs, 0, sizeof(s_discrete_inputs));

    /* SI 液位传感器显示语义：液位跟随稳定后上空气下液体，其余状态均在液体。 */
    s_discrete_inputs[SI_DI_LOWER_LEVEL_SENSOR] = 1U;
    s_discrete_inputs[SI_DI_UPPER_LEVEL_SENSOR] =
        (si_is_probe_follow_stable() != 0U) ? 0U : 1U;
    s_discrete_inputs[SI_DI_BOTTOM_REFERENCE] =
        (g_measurement.height_measurement.bottom_reference_valid != 0U) ? 1U : 0U;
    s_discrete_inputs[SI_DI_INTERLOCK] = si_profile_interlock_active();
    s_discrete_inputs[SI_DI_PROFILE_COMPLETE] =
        (si_is_si_profile_result_valid() != 0U) ? 1U : 0U;
    s_discrete_inputs[SI_DI_UNIT_IS_METRIC] = 1U;
    s_discrete_inputs[SI_DI_REEL_ALARM] =
        (g_measurement.device_status.error_code != NO_ERROR) ? 1U : 0U;
    s_discrete_inputs[SI_DI_PROBE_UNCALIBRATED] = 0U;
    s_discrete_inputs[SI_DI_INTERVAL_TIMER] =
        (s_holding_regs[SI_HR_AUTO_PROFILE_ENABLE] != 0U) ? 1U : 0U;
    s_discrete_inputs[SI_DI_PROBE_AT_LIQUID_LEVEL] =
        (g_measurement.oil_measurement.probe_at_liquid_level != 0U) ? 1U : 0U;
    if (current_density_valid != 0U) {
        s_discrete_inputs[SI_DI_LOW_DENSITY_ALARM] =
            si_low_alarm_u16(current_density, s_holding_regs[SI_HR_LOW_DENSITY_SETPOINT]);
        s_discrete_inputs[SI_DI_HIGH_DENSITY_ALARM] =
            si_high_alarm_u16(current_density, s_holding_regs[SI_HR_HIGH_DENSITY_SETPOINT]);
    }
    if (current_temp_valid != 0U) {
        s_discrete_inputs[SI_DI_LOW_TEMP_ALARM] =
            si_low_alarm_s16(current_temp, s_holding_regs[SI_HR_LOW_TEMPERATURE_SETPOINT]);
        s_discrete_inputs[SI_DI_HIGH_TEMP_ALARM] =
            si_high_alarm_s16(current_temp, s_holding_regs[SI_HR_HIGH_TEMPERATURE_SETPOINT]);
    }
    if (liquid_level_valid != 0U) {
        s_discrete_inputs[SI_DI_LL_LEVEL_ALARM] =
            si_low_alarm_u16(liquid_level, s_holding_regs[SI_HR_LL_LEVEL_SETPOINT]);
        s_discrete_inputs[SI_DI_HH_LEVEL_ALARM] =
            si_high_alarm_u16(liquid_level, s_holding_regs[SI_HR_HH_LEVEL_SETPOINT]);
        s_discrete_inputs[SI_DI_LOW_LEVEL_ALARM] =
            si_low_alarm_u16(liquid_level, s_holding_regs[SI_HR_LOW_LEVEL_SETPOINT]);
        s_discrete_inputs[SI_DI_HIGH_LEVEL_ALARM] =
            si_high_alarm_u16(liquid_level, s_holding_regs[SI_HR_HIGH_LEVEL_SETPOINT]);
    }
    /* Profile报警只读取已发布候选的本地缓存，和Complete、N及点阵保持同一代。 */
    if (s_profile_projection.final_snapshot_ready != 0U) {
        s_discrete_inputs[SI_DI_PROFILE_TEMP_DEVIATION_ALARM] =
            s_profile_projection.profile_temp_deviation_alarm;
        s_discrete_inputs[SI_DI_PROFILE_DENSITY_DEVIATION_ALARM] =
            s_profile_projection.profile_density_deviation_alarm;
        s_discrete_inputs[SI_DI_PROFILE_LOW_TEMP_ALARM] =
            s_profile_projection.profile_low_temp_alarm;
        s_discrete_inputs[SI_DI_PROFILE_HIGH_TEMP_ALARM] =
            s_profile_projection.profile_high_temp_alarm;
        s_discrete_inputs[SI_DI_PROFILE_LOW_DENSITY_ALARM] =
            s_profile_projection.profile_low_density_alarm;
        s_discrete_inputs[SI_DI_PROFILE_HIGH_DENSITY_ALARM] =
            s_profile_projection.profile_high_density_alarm;
    }
}

/**
 * @brief 从 CPU2/CPU3 参数刷新 SI 保持寄存器快照。
 *
 * 40001~40003 来自 CPU2 SI profile 参数，40010~40023 来自 CPU3 本机参数。
 */
static void si_refresh_holding_registers_from_config(void)
{
    uint8_t i;

    s_holding_regs[SI_HR_PROFILE_FIRST_POINT] =
        si_u01mm_to_mm_u16(g_deviceParams.si_profile_first_point);
    s_holding_regs[SI_HR_PROFILE_INCREMENT] =
        si_u01mm_to_mm_u16(g_deviceParams.si_profile_increment);
    s_holding_regs[SI_HR_PROFILE_DWELL_TIME] =
        si_clamp_u16(g_deviceParams.si_profile_dwell_time);
    for (i = 0U; i < CPU3_SI_COMPAT_HOLDING_COUNT; ++i) {
        s_holding_regs[SI_HR_COMPAT_RAW_40004 + i] =
            Cpu3Local_ReadSiCompatHolding(i);
    }
    s_holding_regs[SI_HR_AUTO_PROFILE_INTERVAL] =
        g_cpu3_comm_display_params.si_auto_profile_interval;
    s_holding_regs[SI_HR_AUTO_PROFILE_ENABLE] =
        g_cpu3_comm_display_params.si_auto_profile_enable;
    s_holding_regs[SI_HR_AUTO_PROFILE_HOUR] =
        g_cpu3_comm_display_params.si_auto_profile_hour;
    s_holding_regs[SI_HR_AUTO_PROFILE_MINUTE] =
        g_cpu3_comm_display_params.si_auto_profile_minute;
    s_holding_regs[SI_HR_LOW_DENSITY_SETPOINT] =
        g_cpu3_comm_display_params.si_low_density_setpoint;
    s_holding_regs[SI_HR_HIGH_DENSITY_SETPOINT] =
        g_cpu3_comm_display_params.si_high_density_setpoint;
    s_holding_regs[SI_HR_LOW_TEMPERATURE_SETPOINT] =
        si_s16_to_holding(g_cpu3_comm_display_params.si_low_temperature_setpoint);
    s_holding_regs[SI_HR_HIGH_TEMPERATURE_SETPOINT] =
        si_s16_to_holding(g_cpu3_comm_display_params.si_high_temperature_setpoint);
    s_holding_regs[SI_HR_LL_LEVEL_SETPOINT] =
        g_cpu3_comm_display_params.si_ll_level_setpoint;
    s_holding_regs[SI_HR_HH_LEVEL_SETPOINT] =
        g_cpu3_comm_display_params.si_hh_level_setpoint;
    s_holding_regs[SI_HR_LOW_LEVEL_SETPOINT] =
        g_cpu3_comm_display_params.si_low_level_setpoint;
    s_holding_regs[SI_HR_HIGH_LEVEL_SETPOINT] =
        g_cpu3_comm_display_params.si_high_level_setpoint;
    s_holding_regs[SI_HR_TEMP_DEVIATION_SETPOINT] =
        g_cpu3_comm_display_params.si_temp_deviation_setpoint;
    s_holding_regs[SI_HR_DENSITY_DEVIATION_SETPOINT] =
        g_cpu3_comm_display_params.si_density_deviation_setpoint;
}

/**
 * @brief 判断指定年份是否为闰年。
 *
 * @param year 完整年份数值。
 * @return 1 表示指定年份为闰年；0 表示指定年份不是闰年。
 */
static uint8_t si_is_leap_year(uint16_t year)
{
    if ((year % 400U) == 0U) {
        return 1U;
    }
    if ((year % 100U) == 0U) {
        return 0U;
    }
    return ((year % 4U) == 0U) ? 1U : 0U;
}

/**
 * @brief 返回指定年月的实际天数。
 *
 * @param year 完整年份数值。
 * @param month 月份，合法范围为 1～12。
 * @return 返回指定月份的天数；月份非法时返回 0。
 */
static uint16_t si_days_in_month(uint16_t year, uint8_t month)
{
    static const uint8_t days_per_month[12] = {
        31U, 28U, 31U, 30U, 31U, 30U,
        31U, 31U, 30U, 31U, 30U, 31U
    };

    if ((month == 0U) || (month > 12U)) {
        return 31U;
    }
    if ((month == 2U) && (si_is_leap_year(year) != 0U)) {
        return 29U;
    }
    return days_per_month[month - 1U];
}

/**
 * @brief 计算目标年份之前自纪元起累计的天数。
 *
 * @param year 完整年份数值。
 * @return 返回目标年份之前自 2000 年纪元起累计的完整天数；年份早于纪元时返回 0。
 */
static uint32_t si_days_before_year(uint16_t year)
{
    uint32_t y = (uint32_t)year;

    if (y == 0U) {
        return 0U;
    }
    y--;
    return (y * 365U) + (y / 4U) - (y / 100U) + (y / 400U);
}

/**
 * @brief 将日期时间换算为 SI 调度使用的绝对分钟数。
 *
 * @param dt 待换算的 CPU3 日期时间；字段应已由 RTC 读取链路完成范围校验，传入 NULL 时函数返回 0。
 * @return 返回公历 0001-01-01 00:00 起累计的绝对分钟数；dt 为 NULL 时返回 0，结果包含当前日期之前的整天及当日时、分。
 */
static uint32_t si_datetime_to_schedule_minute(const Cpu3DateTime *dt)
{
    uint32_t day_index;
    uint8_t month;

    if (dt == NULL) {
        return 0U;
    }

    day_index = si_days_before_year(dt->year);
    for (month = 1U; month < dt->month; ++month) {
        day_index += (uint32_t)si_days_in_month(dt->year, month);
    }
    if (dt->day > 0U) {
        day_index += (uint32_t)dt->day - 1U;
    }
    return (day_index * 1440U) + ((uint32_t)dt->hour * 60U) + (uint32_t)dt->minute;
}

/**
 * @brief 观察自动Profile配置变化并重建运行期调度锚点。
 *
 * 冷启动且自动调度原本已启用时，从当天起始时分恢复周期；运行中启用或改参时，第一次触发重新对齐到当前或下一次起始时分，随后再按间隔连续跨天运行。
 */
static void si_auto_profile_refresh_schedule_config(void)
{
    uint16_t interval = g_cpu3_comm_display_params.si_auto_profile_interval;
    uint8_t enable = g_cpu3_comm_display_params.si_auto_profile_enable;
    uint8_t hour = g_cpu3_comm_display_params.si_auto_profile_hour;
    uint8_t minute = g_cpu3_comm_display_params.si_auto_profile_minute;
    bool had_config = s_si_auto_config_valid;
    bool was_enabled = had_config && (s_si_auto_cached_enable != 0U);

    if (had_config &&
        (s_si_auto_cached_interval == interval) &&
        (s_si_auto_cached_enable == enable) &&
        (s_si_auto_cached_hour == hour) &&
        (s_si_auto_cached_minute == minute)) {
        return;
    }

    s_si_auto_cached_interval = interval;
    s_si_auto_cached_enable = enable;
    s_si_auto_cached_hour = hour;
    s_si_auto_cached_minute = minute;
    s_si_auto_config_valid = true;
    s_si_auto_schedule_anchor_valid = false;
    s_si_auto_anchor_from_next_start = had_config;
    s_si_auto_last_trigger_minute = 0xFFFFFFFFUL;
    s_si_auto_last_attempt_valid = false;

    if ((enable != 0U) && (!had_config || !was_enabled)) {
        /* 启动或重新启用时先取得CPU2运行态，避免在同一计划分钟重复覆盖活动SI周期。 */
        s_si_auto_boot_guard_pending = true;
    }
}

/**
 * @brief 推进 SI 自动剖面调度、候选点同步和报警状态。
 *
 * 每次调用先分批同步 SI Profile 候选点阵，并检查自动 Profile 配置是否变化；关闭自动调度或周期为 0 时清除现有计划锚点后返回。
 * CPU3 启动后必须先取得 CPU2 运行态快照，才能判断计划分钟是否已有正在准备、测量或回液位的 SI 周期，避免重启后重复覆盖活动流程。
 * 首次建立锚点时使用当天配置时刻；配置在当天时刻之后生效时可顺延到次日。锚点建立后用累计分钟差对 interval 取模，因此 61 或 1000 分钟等非整日周期能够连续跨越午夜。
 * 同一计划分钟最多触发一次；启动请求未被接受时按 SI_AUTO_PROFILE_RETRY_DELAY_MS 限流重试，只有请求成功才记录该分钟已经触发。
 *
 * @note 该函数由 CPU3 主循环周期调用；候选点阵同步可能跨多轮完成，不应挪入外部 Modbus 请求响应路径。
 */
void si_modbus_periodic_task(void)
{
    Cpu3DateTime now;
    uint32_t interval;
    uint32_t now_minute;
    uint32_t day_start_minute;
    uint32_t configured_start_minute;
    uint32_t elapsed;
    uint32_t now_tick;
    uint8_t check_boot_guard = 0U;

    /* 候选点阵的多帧拉取只允许在周期任务中执行，避免阻塞外部Modbus响应。 */
    si_update_profile_projection(1U);
    si_auto_profile_refresh_schedule_config();

    if (g_cpu3_comm_display_params.si_auto_profile_enable == 0U) {
        s_si_auto_schedule_anchor_valid = false;
        return;
    }

    interval = g_cpu3_comm_display_params.si_auto_profile_interval;
    if (interval == 0U) {
        s_si_auto_schedule_anchor_valid = false;
        return;
    }

    /*
     * 自动调度在CPU3重启后先等到CPU2生命周期快照恢复；若首个可判定分钟
     * 正好是计划分钟，后续会用当前SI阶段阻止同一分钟重复启动同一轮Profile。
     */
    if (s_si_auto_boot_guard_pending && !CPU2_CommHasRuntimeSnapshot()) {
        return;
    }

    if (Cpu3Clock_GetDateTime(&now) == 0U) {
        return;
    }
    if (s_si_auto_boot_guard_pending) {
        s_si_auto_boot_guard_pending = false;
        check_boot_guard = 1U;
    }

    now_minute = si_datetime_to_schedule_minute(&now);
    if (!s_si_auto_schedule_anchor_valid) {
        day_start_minute = now_minute -
                           (((uint32_t)now.hour * 60U) + (uint32_t)now.minute);
        configured_start_minute = day_start_minute +
                                  ((uint32_t)g_cpu3_comm_display_params.si_auto_profile_hour * 60U) +
                                  (uint32_t)g_cpu3_comm_display_params.si_auto_profile_minute;
        if (s_si_auto_anchor_from_next_start &&
            (now_minute > configured_start_minute)) {
            configured_start_minute += 1440U;
        }
        s_si_auto_schedule_anchor_minute = configured_start_minute;
        s_si_auto_schedule_anchor_valid = true;
        s_si_auto_anchor_from_next_start = false;
    }
    if (now_minute < s_si_auto_schedule_anchor_minute) {
        return;
    }

    /* 锚点建立后不再按日期重算，因此61/1000min等非整日周期可连续跨午夜。 */
    elapsed = now_minute - s_si_auto_schedule_anchor_minute;
    if ((elapsed % interval) != 0U) {
        return;
    }
    if (s_si_auto_last_trigger_minute == now_minute) {
        return;
    }
    if (check_boot_guard != 0U) {
        uint32_t phase = g_measurement.si_profile_runtime.phase;

        if ((phase == (uint32_t)SI_PROFILE_PHASE_PREPARING) ||
            (phase == (uint32_t)SI_PROFILE_PHASE_MEASURING) ||
            (phase == (uint32_t)SI_PROFILE_PHASE_RETURNING_LEVEL)) {
            s_si_auto_last_trigger_minute = now_minute;
            return;
        }
    }

    now_tick = HAL_GetTick();
    if (s_si_auto_last_attempt_valid &&
        ((now_tick - s_si_auto_last_attempt_tick) < SI_AUTO_PROFILE_RETRY_DELAY_MS)) {
        return;
    }
    s_si_auto_last_attempt_tick = now_tick;
    s_si_auto_last_attempt_valid = true;

    if (si_profile_request_start()) {
        s_si_auto_last_trigger_minute = now_minute;
    }
}

/**
 * @brief 从实时测量和 profile 结果刷新 SI 输入寄存器。
 *
 * 未完成 profile 时点阵区域保持 0，避免 PLC 读取上一轮残留。
 */
static void si_refresh_input_registers_from_measurement(void)
{
    Cpu3DateTime now;
    uint16_t i;
    uint16_t point_count = 0U;
    uint32_t phase = s_profile_projection.last_phase;

    if (s_profile_projection.final_snapshot_ready != 0U) {
        point_count = s_profile_projection.final_points;
    } else if ((s_profile_projection.cycle_established != 0U) &&
               ((phase == (uint32_t)SI_PROFILE_PHASE_MEASURING) ||
                (phase == (uint32_t)SI_PROFILE_PHASE_RETURNING_LEVEL) ||
                (phase == (uint32_t)SI_PROFILE_PHASE_COMPLETE))) {
        point_count = s_profile_projection.progress_points;
    }

    memset(s_input_regs, 0, sizeof(s_input_regs));

    /* 实时值每帧刷新：温度无效输出 -200.00C，密度无效输出 0。 */
    s_input_regs[SI_IR_CURRENT_PROBE_POSITION] =
        si_pos01mm_to_mm_u16(g_measurement.debug_data.sensor_position);
    s_input_regs[SI_IR_CURRENT_TEMPERATURE] =
        si_temp_raw_to_si_s16(g_measurement.debug_data.temperature);
    s_input_regs[SI_IR_CURRENT_DENSITY] =
        si_density_raw_to_si_u16(g_measurement.single_point_monitoring.density);
    s_input_regs[SI_IR_LIQUID_LEVEL] =
        si_u01mm_to_mm_u16(g_measurement.oil_measurement.oil_level);
    s_input_regs[SI_IR_NUMBER_OF_POINTS] = point_count;

    if (s_profile_timestamp_valid != 0U) {
        s_input_regs[SI_IR_PROFILE_TIMESTAMP_MONTH] = s_profile_timestamp.month;
        s_input_regs[SI_IR_PROFILE_TIMESTAMP_DAY] = s_profile_timestamp.day;
        s_input_regs[SI_IR_PROFILE_TIMESTAMP_HOUR] = s_profile_timestamp.hour;
        s_input_regs[SI_IR_PROFILE_TIMESTAMP_MINUTE] = s_profile_timestamp.minute;
    }

    /* 当前时间来自 CPU3 RTC，仅用于 SI 显示，不参与 CPU2 测量调度。 */
    if (Cpu3Clock_GetDateTime(&now) != 0U) {
        s_input_regs[SI_IR_CURRENT_TIME_HOUR] = now.hour;
        s_input_regs[SI_IR_CURRENT_TIME_MINUTE] = now.minute;
        s_input_regs[SI_IR_CURRENT_TIME_SECOND] = now.second;
    }

    s_input_regs[SI_IR_COIL_MIRROR] =
        si_pack_bits_u16(s_coils, 0U);
    s_input_regs[SI_IR_DISCRETE_MIRROR_LOW] =
        si_pack_bits_u16(s_discrete_inputs, 0U);
    s_input_regs[SI_IR_DISCRETE_MIRROR_HIGH] =
        si_pack_bits_u16(s_discrete_inputs, 16U);

    /* 候选校验和最终门禁完成前不开放点阵，30017～30020依靠整区清零保持0。 */
    for (i = 0U; i < s_profile_projection.final_points; ++i) {
        uint16_t base = (uint16_t)(SI_IR_PROFILE_POINT0_POSITION + (3U * i));
        uint16_t source = (uint16_t)(3U * i);

        if ((s_profile_projection.final_snapshot_ready == 0U) ||
            ((uint32_t)base + 2U >= SI_INPUT_REG_COUNT)) {
            break;
        }

        s_input_regs[base] = s_profile_projection.point_regs[source];
        s_input_regs[base + 1U] = s_profile_projection.point_regs[source + 1U];
        s_input_regs[base + 2U] = s_profile_projection.point_regs[source + 2U];
    }
}

/**
 * @brief 应用 PLC 对线圈的写入。
 *
 * @param offset 相对起始位置的偏移量。
 * @param is_on true 表示线圈写值为导通，false 表示写值为断开。
 * @return true 表示 OFF 值已按只更新影子位语义接受，或 ON 值已映射并成功下发对应 CPU2 命令；false 表示线圈偏移不受支持，或映射后的命令请求失败。
 * @note 写 ON 会转换为 CPU2 命令；写 OFF 只更新影子位，不主动停止 CPU2 当前动作。
 */
static bool si_apply_coil_write(uint16_t offset, uint8_t is_on)
{
    bool request_ok = true;

    if (!is_on) {
        return true;
    }

    switch (offset) {
    case SI_COIL_MANUAL:
        /* 当前系统里最接近 SI Manual 的入口是维护模式。 */
        request_ok = si_send_cpu2_command(CMD_MAINTENANCE_MODE);
        break;
    case SI_COIL_CALIBRATE:
        /* 先保守映射到零点标定，后续再细分为零点/液位/水位等标定流程。 */
        request_ok = si_send_cpu2_command(CMD_CALIBRATE_ZERO);
        break;
    case SI_COIL_AUTO:
        request_ok = si_send_cpu2_command(CMD_FIND_OIL);
        break;
    case SI_COIL_PROFILE:
        request_ok = si_profile_request_start();
        break;
    case SI_COIL_STOP:
        /* Stop只取消当前测量/运动，不再隐式进入维护模式。 */
        request_ok = si_send_cpu2_command(CMD_CANCEL_MEASUREMENT);
        break;
    case SI_COIL_UP_SLOW:
    case SI_COIL_UP_MEDIUM:
    case SI_COIL_UP_FAST:
        /* 速度档位后续再细分，先统一桥到可被命令切换打断的强制上行。 */
        request_ok = si_send_cpu2_command(CMD_FORCE_MOVE_UP);
        break;
    case SI_COIL_DOWN_SLOW:
    case SI_COIL_DOWN_MEDIUM:
    case SI_COIL_DOWN_FAST:
        /* 速度档位后续再细分，先统一桥到可被命令切换打断的强制下行。 */
        request_ok = si_send_cpu2_command(CMD_FORCE_MOVE_DOWN);
        break;
    default:
        break;
    }

    if (!request_ok) {
        return false;
    }

    if (offset <= SI_COIL_PROFILE) {
        /* CPU2 确认后再收口模式影子，避免失败请求显示为已执行。 */
        for (uint16_t i = SI_COIL_MANUAL; i <= SI_COIL_PROFILE; ++i) {
            s_coils[i] = 0U;
        }
        s_coils[offset] = 1U;
    } else if ((offset >= SI_COIL_STOP) && (offset <= SI_COIL_DOWN_FAST)) {
        /* 停止和手动方向/速度线圈互斥，避免 FC05 连续写入后读回多个动作位。 */
        for (uint16_t i = SI_COIL_STOP; i <= SI_COIL_DOWN_FAST; ++i) {
            s_coils[i] = 0U;
        }
        s_coils[offset] = 1U;
    }
    return true;
}

/**
 * @brief 应用 PLC 对保持寄存器的写入。
 *
 * @param offset 相对起始位置的偏移量。
 * @param value 应用保持寄存器写入使用的输入数值。
 * @return true 表示 SI Profile 参数、兼容槽、本机自动 Profile/报警参数或日期时间字段已按各自策略写入；false 表示偏移未知、值域或日期时间无效，CPU2 参数写入失败，或 CPU3 FRAM 持久化失败。
 * @note profile 基础参数经 CPU2 参数通道落地；自动 profile 和报警限值保存在 CPU3。
 */
static bool si_apply_holding_write(uint16_t offset, uint16_t value)
{
    switch (offset) {
    case SI_HR_PROFILE_FIRST_POINT:
        return si_write_device_param_u32(HOLDREGISTER_DEVICEPARAM_SI_PROFILE_FIRST_POINT,
                                         &g_deviceParams.si_profile_first_point,
                                         si_mm_to_u01mm(value));
    case SI_HR_PROFILE_INCREMENT:
        return si_write_device_param_u32(HOLDREGISTER_DEVICEPARAM_SI_PROFILE_INCREMENT,
                                         &g_deviceParams.si_profile_increment,
                                         si_mm_to_u01mm(value));
    case SI_HR_PROFILE_DWELL_TIME:
        return si_write_device_param_u32(HOLDREGISTER_DEVICEPARAM_SI_PROFILE_DWELL_TIME,
                                         &g_deviceParams.si_profile_dwell_time,
                                         value);
    case SI_HR_COMPAT_RAW_40004:
    case SI_HR_COMPAT_RAW_40005:
    case SI_HR_COMPAT_RAW_40006:
    case SI_HR_COMPAT_RAW_40007:
    case SI_HR_COMPAT_RAW_40008:
    case SI_HR_COMPAT_RAW_40009:
        return Cpu3Local_WriteSiCompatHoldingChecked(
            (uint8_t)(offset - SI_HR_COMPAT_RAW_40004),
            value);
    case SI_HR_AUTO_PROFILE_INTERVAL:
    case SI_HR_AUTO_PROFILE_ENABLE:
    case SI_HR_AUTO_PROFILE_HOUR:
    case SI_HR_AUTO_PROFILE_MINUTE:
    case SI_HR_LL_LEVEL_SETPOINT:
    case SI_HR_HH_LEVEL_SETPOINT:
    case SI_HR_LOW_LEVEL_SETPOINT:
    case SI_HR_HIGH_LEVEL_SETPOINT:
    case SI_HR_LOW_DENSITY_SETPOINT:
    case SI_HR_HIGH_DENSITY_SETPOINT:
    case SI_HR_TEMP_DEVIATION_SETPOINT:
    case SI_HR_DENSITY_DEVIATION_SETPOINT:
    {
        OperatingNumber opera = si_holding_offset_to_cpu3_param(offset);
        if (opera == COM_NUM_NOOPERA) {
            return false;
        }
        return Cpu3Local_WriteValueChecked(opera, value);
    }
    case SI_HR_LOW_TEMPERATURE_SETPOINT:
    case SI_HR_HIGH_TEMPERATURE_SETPOINT:
    {
        OperatingNumber opera = si_holding_offset_to_cpu3_param(offset);
        if (opera == COM_NUM_NOOPERA) {
            return false;
        }
        return Cpu3Local_WriteValueChecked(opera,
                                           (int32_t)si_holding_to_s16(value));
    }
    default:
        break;
    }
    return true;
}

/**
 * @brief 处理 FC01/FC02 读位请求。
 *
 * 请求长度、数量和地址边界都在这里统一检查，响应位按 Modbus 低位优先打包。
 *
 * @param func 当前读位请求功能码，FC01 选择线圈池，FC02 选择离散输入池。
 * @param bit_pool 待按请求起始地址读取的线圈或离散输入快照数组。
 * @param bit_count 目标离散状态数组可访问的元素总数，用于校验请求范围。
 * @param pdu 待解析或构造的 Modbus PDU 缓冲区。
 * @param pdu_len Modbus PDU 有效长度，单位字节。
 * @param tx SI FC01/FC02 位数据响应或标准异常响应的输出缓冲区；函数同步更新 tx_len。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 1，表示已构造可发送的 FC01/FC02 正常响应或 Modbus 异常响应。
 */
static uint8_t si_handle_read_bits(uint8_t func,
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
        return si_build_exception(func, SI_EX_ILLEGAL_VALUE, tx, tx_len);
    }

    start = si_be16(&pdu[1]);
    qty = si_be16(&pdu[3]);

    if ((qty == 0U) || (qty > 2000U)) {
        return si_build_exception(func, SI_EX_ILLEGAL_VALUE, tx, tx_len);
    }
    if ((uint32_t)start + qty > bit_count) {
        return si_build_exception(func, SI_EX_ILLEGAL_ADDRESS, tx, tx_len);
    }
    if ((CPU3_ExternalSiBitRangeNeedsRuntime(func, start, qty) != 0U) &&
        !CPU2_CommHasRuntimeSnapshot()) {
        /* CPU2 派生位与本地位混读时整帧返回忙，禁止旧状态影子穿透。 */
        return si_build_exception(func, SI_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
    }
    if ((CPU3_ExternalSiBitRangeNeedsFixedPoint(func, start, qty) != 0U) &&
        !CPU2_CommHasFixedPointSnapshot()) {
        /* 固定点密度报警和混合范围等待固定点一致性握手。 */
        return si_build_exception(func, SI_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
    }

    byte_count = (uint16_t)((qty + 7U) / 8U);
    tx[0] = si_get_effective_slave_address();
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

/**
 * @brief 处理 FC03/FC04 读寄存器请求。
 *
 * 限制最大 125 个寄存器，保证响应始终落在 256 字节发送缓冲区内。
 *
 * @param func 当前读寄存器请求功能码，FC03 选择保持寄存器，FC04 选择输入寄存器。
 * @param reg_pool FC03/FC04 读取使用的只读寄存器镜像首地址。
 * @param reg_count 目标寄存器镜像可访问的 16 位寄存器总数。
 * @param pdu 待解析或构造的 Modbus PDU 缓冲区。
 * @param pdu_len Modbus PDU 有效长度，单位字节。
 * @param tx SI FC03/FC04 寄存器数据响应或标准异常响应的输出缓冲区；函数同步更新 tx_len。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 1，表示已构造可发送的 FC03/FC04 正常响应或 Modbus 异常响应。
 */
static uint8_t si_handle_read_regs(uint8_t func,
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
        return si_build_exception(func, SI_EX_ILLEGAL_VALUE, tx, tx_len);
    }

    start = si_be16(&pdu[1]);
    qty = si_be16(&pdu[3]);

    if ((qty == 0U) || (qty > 125U)) {
        return si_build_exception(func, SI_EX_ILLEGAL_VALUE, tx, tx_len);
    }
    if ((uint32_t)start + qty > reg_count) {
        return si_build_exception(func, SI_EX_ILLEGAL_ADDRESS, tx, tx_len);
    }
    if ((func == SI_FUNC_READ_HOLDING_REGS) &&
        (start <= SI_HR_PROFILE_DWELL_TIME) &&
        !CPU2_CommIsAvailable()) {
        /*
         * 40001～40003来自CPU2参数快照。只要请求与该段相交，快照补读完成前
         * 就返回设备忙；40004～40023仍是CPU3本机参数，可在CPU2离线时读取。
         */
        return si_build_exception(func, SI_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
    }
    if ((func == SI_FUNC_READ_INPUT_REGS) &&
        (CPU3_ExternalSiInputRangeNeedsRuntime(start, qty) != 0U) &&
        !CPU2_CommHasRuntimeSnapshot()) {
        /* Complete、时间、镜像和点阵都随 CPU2 运行态失效，不把旧发布快照当静态数据。 */
        return si_build_exception(func, SI_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
    }
    if ((func == SI_FUNC_READ_INPUT_REGS) &&
        (CPU3_ExternalSiInputRangeNeedsFixedPoint(start, qty) != 0U) &&
        !CPU2_CommHasFixedPointSnapshot()) {
        /* 当前密度及其报警镜像必须等待固定点一致性握手。 */
        return si_build_exception(func, SI_EX_SLAVE_DEVICE_BUSY, tx, tx_len);
    }

    /* 寄存器响应逐项转为大端，保持 Modbus RTU 标准字节序。 */
    tx[0] = si_get_effective_slave_address();
    tx[1] = func;
    tx[2] = (uint8_t)(qty * 2U);
    for (i = 0U; i < qty; ++i) {
        si_wr_be16(&tx[3U + (2U * i)], reg_pool[start + i]);
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

/**
 * @brief 处理 FC05 写单线圈请求。
 *
 * 只接受标准 0xFF00/0x0000 写值，合法 ON 写入再桥接到 CPU2 命令。
 *
 * @param pdu 待解析或构造的 Modbus PDU 缓冲区。
 * @param pdu_len Modbus PDU 有效长度，单位字节。
 * @param tx SI FC05 8 字节标准回显或异常响应的输出缓冲区。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 1，表示已构造 FC05 回显或对应的非法地址、非法数值、设备忙异常响应。
 */
static uint8_t si_handle_write_single_coil(const uint8_t *pdu,
                                               uint16_t pdu_len,
                                               uint8_t *tx,
                                               uint16_t *tx_len)
{
    uint16_t offset;
    uint16_t value;
    uint8_t is_on;

    if (pdu_len != 5U) {
        return si_build_exception(SI_FUNC_WRITE_SINGLE_COIL,
                                      SI_EX_ILLEGAL_VALUE,
                                      tx,
                                      tx_len);
    }

    offset = si_be16(&pdu[1]);
    value = si_be16(&pdu[3]);

    if ((offset >= SI_COIL_COUNT) || (si_is_coil_writeable(offset) == 0U)) {
        return si_build_exception(SI_FUNC_WRITE_SINGLE_COIL,
                                      SI_EX_ILLEGAL_ADDRESS,
                                      tx,
                                      tx_len);
    }
    /* FC05 标准只接受 0xFF00/0x0000，其他值按非法数据值处理。 */
    if ((value != 0x0000U) && (value != 0xFF00U)) {
        return si_build_exception(SI_FUNC_WRITE_SINGLE_COIL,
                                      SI_EX_ILLEGAL_VALUE,
                                      tx,
                                      tx_len);
    }

    is_on = (value == 0xFF00U) ? 1U : 0U;
    if (!si_apply_coil_write(offset, is_on)) {
        return si_build_exception(SI_FUNC_WRITE_SINGLE_COIL,
                                  SI_EX_SLAVE_DEVICE_BUSY,
                                  tx,
                                  tx_len);
    }
    s_coils[offset] = is_on;

    si_build_write_echo(SI_FUNC_WRITE_SINGLE_COIL, offset, value, tx, tx_len);
    return 1U;
}

/**
 * @brief 处理 FC06 写单保持寄存器请求。
 *
 * 先做地址和值域校验，再更新影子寄存器或通过 CPU2 参数通道下发。
 *
 * @param pdu 待解析或构造的 Modbus PDU 缓冲区。
 * @param pdu_len Modbus PDU 有效长度，单位字节。
 * @param tx SI FC06 8 字节标准回显或异常响应的输出缓冲区。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 1，表示已构造 FC06 回显或对应的非法地址、非法数值、设备忙异常响应。
 */
static uint8_t si_handle_write_single_reg(const uint8_t *pdu,
                                              uint16_t pdu_len,
                                              uint8_t *tx,
                                              uint16_t *tx_len)
{
    uint16_t offset;
    uint16_t value;

    if (pdu_len != 5U) {
        return si_build_exception(SI_FUNC_WRITE_SINGLE_REG,
                                      SI_EX_ILLEGAL_VALUE,
                                      tx,
                                      tx_len);
    }

    offset = si_be16(&pdu[1]);
    value = si_be16(&pdu[3]);

    if ((offset >= SI_HOLDING_REG_COUNT) || (si_is_holding_writeable(offset) == 0U)) {
        return si_build_exception(SI_FUNC_WRITE_SINGLE_REG,
                                      SI_EX_ILLEGAL_ADDRESS,
                                      tx,
                                      tx_len);
    }
    /* 对自动 profile 使能和时分字段做基础范围保护，其余阈值保留现场自由度。 */
    if (si_is_holding_value_valid(offset, value) == 0U) {
        return si_build_exception(SI_FUNC_WRITE_SINGLE_REG,
                                      SI_EX_ILLEGAL_VALUE,
                                      tx,
                                      tx_len);
    }

    if (!si_apply_holding_write(offset, value)) {
        return si_build_exception(SI_FUNC_WRITE_SINGLE_REG,
                                  SI_EX_SLAVE_DEVICE_BUSY,
                                  tx,
                                  tx_len);
    }
    s_holding_regs[offset] = value;

    si_build_write_echo(SI_FUNC_WRITE_SINGLE_REG, offset, value, tx, tx_len);
    return 1U;
}



/**
 * @brief 手动同步 SI 四类寄存器影子区。
 *
 * 正常处理请求前会自动调用，外部入口主要用于测试或联调前刷新快照。
 */
void si_modbus_sync_from_system(void)
{
    /* 请求路径只更新轻量投影，不在外部响应期间执行CPU2多帧候选拉取。 */
    si_update_profile_projection(0U);
    si_refresh_coils_from_state();
    si_refresh_holding_registers_from_config();
    si_refresh_discrete_inputs_from_state();
    si_refresh_input_registers_from_measurement();
}

/**
 * @brief 处理一帧完整 SI Modbus RTU 请求。
 *
 * 地址和 CRC 通过后才同步系统状态，避免无关帧扰动 CPU3 快照。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx 完整 SI Modbus RTU 正常或异常响应的输出缓冲区；调用方容量必须覆盖协议允许的最大响应。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 返回 SI Modbus 分发结果；SI_MODBUS_OK 表示已完成处理，其他值区分帧长、CRC、从机地址和功能码错误；tx_len 非零时仍应发送已生成的异常响应。
 */
SiModbusResult si_modbus_process(const uint8_t *rx,
                                         uint16_t rx_len,
                                         uint8_t *tx,
                                         uint16_t *tx_len)
{
    const uint8_t *pdu;
    uint16_t pdu_len;
    uint8_t func;

    if ((rx == NULL) || (tx == NULL) || (tx_len == NULL)) {
        return SI_MODBUS_ERR_BADLEN;
    }

    *tx_len = 0U;

    if (rx_len < 4U) {
        return SI_MODBUS_ERR_BADLEN;
    }
    if (rx[0] != si_get_effective_slave_address()) {
        return SI_MODBUS_ERR_ADDR_MISMATCH;
    }
    if (!SlaveCheckCRC(rx, rx_len)) {
        return SI_MODBUS_ERR_CRC;
    }

    /* CRC 和地址通过后再同步系统状态，降低无关帧对 CPU3 运行状态的扰动。 */
    si_modbus_sync_from_system();

    func = rx[1];
    pdu = &rx[1];
    pdu_len = (uint16_t)(rx_len - 3U);

    switch (func) {
    case SI_FUNC_READ_COILS:
        si_handle_read_bits(func, s_coils, SI_COIL_COUNT, pdu, pdu_len, tx, tx_len);
        return SI_MODBUS_OK;

    case SI_FUNC_READ_DISCRETE_INPUTS:
        si_handle_read_bits(func, s_discrete_inputs, SI_DISCRETE_INPUT_COUNT, pdu, pdu_len, tx, tx_len);
        return SI_MODBUS_OK;

    case SI_FUNC_READ_HOLDING_REGS:
        si_handle_read_regs(func, s_holding_regs, SI_HOLDING_REG_COUNT, pdu, pdu_len, tx, tx_len);
        return SI_MODBUS_OK;

    case SI_FUNC_READ_INPUT_REGS:
        si_handle_read_regs(func, s_input_regs, SI_INPUT_REG_COUNT, pdu, pdu_len, tx, tx_len);
        return SI_MODBUS_OK;

    case SI_FUNC_WRITE_SINGLE_COIL:
        si_handle_write_single_coil(pdu, pdu_len, tx, tx_len);
        return SI_MODBUS_OK;

    case SI_FUNC_WRITE_SINGLE_REG:
        si_handle_write_single_reg(pdu, pdu_len, tx, tx_len);
        return SI_MODBUS_OK;

    default:
        si_build_exception(func, SI_EX_ILLEGAL_FUNCTION, tx, tx_len);
        return SI_MODBUS_ERR_FUNC_UNSUPPORT;
    }
}

/**
 * @brief 适配 CPU3 协议分发表的返回值口径。
 *
 * 只要已生成正常或异常响应帧，分发层就按成功处理并发送该响应。
 *
 * @param rx 接收到的数据缓冲区。有效字节范围由 rx_len 或调用点固定帧长限定，函数不会修改原始请求帧。
 * @param rx_len 接收数据的有效长度，单位字节。函数只读取 rx[0..rx_len-1]，并在访问固定字段前检查协议要求的最小长度。
 * @param tx 转交 SI 协议核心处理器的响应输出缓冲区；tx_len 非 0 时内容可直接交给外部端口发送。
 * @param tx_len 待发送数据的有效长度，单位字节。该指针用于返回已经构造完成的响应帧总长度，长度包含当前协议要求的帧头、数据区及 CRC 等尾部字段。
 * @return 已生成正常或异常响应时返回 0；没有响应帧时返回 SI Modbus 解析错误码。
 */
uint32_t si_modbus_process_for_dispatch(const uint8_t *rx,
                                            uint16_t rx_len,
                                            uint8_t *tx,
                                            uint16_t *tx_len)
{
    SiModbusResult ret = si_modbus_process(rx, rx_len, tx, tx_len);

    /* 统一外部协议分发语义：只要已经生成正常/异常响应帧，分发层就视为本帧已处理。 */
    if ((tx_len != NULL) && (*tx_len > 0U)) {
        return 0U;
    }

    return (uint32_t)ret;
}

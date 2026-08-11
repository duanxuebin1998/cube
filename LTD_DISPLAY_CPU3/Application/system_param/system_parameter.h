/*
 * @FilePath     : \KEILe:\03CodeRepository\DSM_MCB -V1.198旧版本升级\A.c
 * @Description  :
 * @Author       : Aubon
 * @Date         : 2025-07-15 11:01:57
 * @LastEditors  : Duan
 * @LastEditTime : 2025-07-15 11:02:02
 * Copyright 2025 Aubon, All Rights Reserved.
 * 2025-07-15 11:01:57
 */
#ifndef _SYSTEM_PARAMETER_H
/* _SYSTEM_PARAMETER_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define _SYSTEM_PARAMETER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
/* 无效值 */
#define UNVALID_LEVEL 999999u /* 液位无效值 */
/* 无效电流哨兵值 3.5 mA；低于正常 4 mA 起点，用于表示当前没有可信电流过程量。 */
#define UNVALID_CURRENT 3.5
#define LEVEL_DOWNLIMIT 100u               /* 盲区液位值 */
#define UNVALID_DENSITY 0                  /* 密度无效值 */
#define LEVEL_DOWNLIMITWATER 0             /* 盲区水位值 */
#define UNVALID_VCF 1                      /* VCF无效值 */
#define UNVALID_TOV 0                      /* 体积无效值 */
#define UNVALID_GSW 0                      /* 质量无效值 */
/* 无效位置哨兵值 0；使用该值前必须结合有效标志，避免把合法零基准与无效状态混淆。 */
#define UNVALID_POSITION 0
/* 历史有线温度无效哨兵值 0；保留用于旧接口兼容，不能与无线温度无效值 UNVALID_TEMPERATURE_WIRELESS 混用。 */
#define UNVALID_TEMPERATURE 0
#define MAX_MEASUREMENT_POINTS 200 /* 密度分布测量最大点数 */
#define DEVICE_PROTOCOL_VERSION 33u /* CPU2/CPU3共享协议版本；协议33将AO电流修正及最终目标/实发电流倍率提升为x1000。 */
/* 故障自动恢复的默认重试次数 3；仅在参数缺省、越界或旧版本迁移时作为归一化值。 */
#define FAULT_AUTO_RECOVERY_RETRY_DEFAULT 3u
/* 故障自动恢复重试次数的配置硬上限 10；CPU3 菜单和参数校验不得允许写入超过该值的重试次数。 */
#define FAULT_AUTO_RECOVERY_RETRY_MAX 10u

/* AO 关闭状态使用的硬件目标电流 3.40 mA，线值单位为 0.01 mA；该值低于正常 4 mA 量程起点，用于表达禁用而非有效过程量。 */
#define AO_DISABLED_CURRENT_MA_X100        340U
/* AO 上电初始电流允许下限 3.40 mA，线值单位为 0.01 mA；参数校验和旧结构迁移必须使用同一边界。 */
#define AO_INITIAL_CURRENT_MIN_MA_X100     340U
/* AO 上电初始电流允许上限 22.60 mA，线值单位为 0.01 mA；超过该值的持久化参数必须判无效或归一化。 */
#define AO_INITIAL_CURRENT_MAX_MA_X100     2260U
/* AO 上电电流下限的语义别名；与初始电流下限保持一致，避免 CPU2/CPU3 参数校验使用不同范围。 */
#define AO_POWER_ON_CURRENT_MIN_MA_X100    AO_INITIAL_CURRENT_MIN_MA_X100
/* AO 上电电流上限的语义别名；与初始电流上限保持一致，修改基础边界时该别名自动同步。 */
#define AO_POWER_ON_CURRENT_MAX_MA_X100    AO_INITIAL_CURRENT_MAX_MA_X100
/* AO 固定输出模式允许的最小电流 4.00 mA，线值单位为 0.01 mA。 */
#define AO_FIXED_CURRENT_MIN_MA_X100       400U
/* AO 固定输出模式允许的最大电流 22.50 mA，线值单位为 0.01 mA。 */
#define AO_FIXED_CURRENT_MAX_MA_X100       2250U
/* AO 故障电流模式允许的最小电流 3.40 mA，线值单位为 0.01 mA。 */
#define AO_FAULT_CURRENT_MIN_MA_X100       340U
/* AO 故障电流模式允许的最大电流 22.60 mA，线值单位为 0.01 mA。 */
#define AO_FAULT_CURRENT_MAX_MA_X100       2260U
/* AO 调试模拟输出允许的最小电流 3.40 mA，线值单位为 0.01 mA。 */
#define AO_SIMULATION_CURRENT_MIN_MA_X100  340U
/* AO 调试模拟输出允许的最大电流 23.00 mA，线值单位为 0.01 mA；模拟模式仍不能越过硬件保护范围。 */
#define AO_SIMULATION_CURRENT_MAX_MA_X100  2300U
/* AO 电流校正量允许下限 -1.000 mA，线值单位为 0.001 mA；该值是有符号修正量，不是绝对输出电流。 */
#define AO_CURRENT_CORRECTION_MIN_MA_X1000  (-1000)
/* AO 电流校正量允许上限 +1.000 mA，线值单位为 0.001 mA；校正后结果仍需经过硬件电流限幅。 */
#define AO_CURRENT_CORRECTION_MAX_MA_X1000  1000
/* AO 阻尼时间允许上限 999.9 s，线值单位为 0.1 s；用于限制持久化参数和菜单输入，防止时间换算溢出。 */
#define AO_DAMPING_MAX_X10_S               9999U

#define DENSITY_RAW_SCALE 100U /* 密度内部原始值倍率，单位 0.01kg/m3。 */
#define DENSITY_EXTERNAL_SCALE_X10 10U /* 旧外部协议密度倍率，单位 0.1kg/m3。 */
#define DENSITY_PARAM_MIGRATE_FACTOR 10U /* 旧 x10 密度参数迁移到 x100 的倍率。 */
#define DENSITY_CORRECTION_OLD_BASE_RAW 10000U /* 旧密度修正零点，单位 0.1kg/m3。 */
#define DENSITY_CORRECTION_BASE_RAW 100000U /* 密度修正零点，单位 0.01kg/m3。 */

typedef enum {
    DSM_SENSOR = 12, /* 一体机传感器 */
    LTD_SENSOR = 13, /* 多参数传感器通信协议V3.0设备 */
    SAFE_SENSOR = 14, /* 安全协议传感器 */
    MULTIPARAM_V4_SENSOR = 15 /* 多参数V4传感器，不绑定具体产品型号 */
} SENSOR_TYPE;

/*
 * 函数用途：判断CPU2共享的传感器类型是否属于当前CPU3协议版本支持的枚举。
 * 调用场景：参数快照接收、批量同步预检和诊断显示。
 */
bool DeviceParam_IsSensorTypeSupported(uint32_t value);

typedef enum {
    /* 剖面测量结果来源；用于跨 CPU 判断点阵布局、完成条件和对外协议投影。 */
    PROFILE_SOURCE_NONE = 0u, /* 结果来源为没有有效剖面来源。 */
    PROFILE_SOURCE_STANDARD = 1u, /* 结果来源为标准密度分布流程。 */
    PROFILE_SOURCE_GB = 2u, /* 结果来源为国标密度分布流程。 */
    PROFILE_SOURCE_METER = 3u, /* 结果来源为仪表直接提供的剖面。 */
    PROFILE_SOURCE_INTERVAL = 4u, /* 结果来源为区间密度测量流程。 */
    PROFILE_SOURCE_WARTSILA = 5u, /* 结果来源为Wartsila 剖面流程。 */
    PROFILE_SOURCE_SI = 6u /* 结果来源为SI Profile 流程。 */
} ProfileSource;

typedef enum {
    /* SI 剖面测量阶段；CPU3 以该阶段配合周期和完成计数判定快照是否可发布。 */
    SI_PROFILE_PHASE_IDLE = 0u, /* 空闲，当前没有 SI 剖面任务。 */
    SI_PROFILE_PHASE_PREPARING = 1u, /* 准备阶段，尚未产生有效点阵。 */
    SI_PROFILE_PHASE_MEASURING = 2u, /* 逐点测量阶段。 */
    SI_PROFILE_PHASE_RETURNING_LEVEL = 3u, /* 测量结束后返回液面阶段。 */
    SI_PROFILE_PHASE_COMPLETE = 4u, /* 完整剖面已经提交。 */
    SI_PROFILE_PHASE_ABORTED = 5u, /* 流程被上层主动中止。 */
    SI_PROFILE_PHASE_FAILED = 6u /* 流程因故障终止。 */
} SiProfilePhase;

typedef struct {
    /* SI 剖面流程对外发布的最小运行态；阶段、周期计数和已完成点数必须来自同一次快照。 */
    uint32_t phase; /* 当前剖面或状态机阶段；消费者必须按对应枚举解释。 */
    uint32_t cycle_counter; /* 剖面测量周期计数；每开始一个新周期递增，用于区分不同点阵。 */
    uint32_t progress_points; /* 当前周期已经完成并可报告的剖面测点数。 */
} SiProfileRuntime;



#define RELAY_ALARM_CHANNEL_COUNT 4u /* 当前项目显示为 K1~K4，内部枚举仍使用 RELAY1~RELAY4 */
#define RELAY_ALARM_FIELD_COUNT   13u /* 每路继电器报警输出配置占用的 32 位字段数 */

typedef enum {
    RELAY_ALARM_OPERATING_DISABLED = 0u,       /* 禁用 */
    RELAY_ALARM_OPERATING_OUTPUT_PASSIVE = 1u  /* 无源输出 */
} RelayAlarmOperatingMode;

typedef enum {
    RELAY_ALARM_DIGITAL_NONE = 0u,       /* 无 */
    RELAY_ALARM_DIGITAL_H = 1u,          /* 高报 */
    RELAY_ALARM_DIGITAL_HH = 2u,         /* 高高报 */
    RELAY_ALARM_DIGITAL_H_OR_HH = 3u,    /* 高报或高高报 */
    RELAY_ALARM_DIGITAL_L = 4u,          /* 低报 */
    RELAY_ALARM_DIGITAL_LL = 5u,         /* 低低报 */
    RELAY_ALARM_DIGITAL_L_OR_LL = 6u,    /* 低报或低低报 */
    RELAY_ALARM_DIGITAL_ANY = 7u         /* 任意报警 */
} RelayAlarmDigitalSource;

typedef enum {
    RELAY_ALARM_CONTACT_NORMALLY_OPEN = 0u,    /* 常开 */
    RELAY_ALARM_CONTACT_NORMALLY_CLOSED = 1u   /* 常闭 */
} RelayAlarmContactType;

typedef enum {
    RELAY_ALARM_MODE_OFF = 0u,       /* 关 */
    RELAY_ALARM_MODE_ON = 1u,        /* 开 */
    RELAY_ALARM_MODE_LATCHING = 2u   /* 锁存 */
} RelayAlarmMode;

typedef enum {
    RELAY_ALARM_ERROR_NO_ALARM = 0u,       /* 无报警 */
    RELAY_ALARM_ERROR_HH_H = 1u,           /* 高高/高报警 */
    RELAY_ALARM_ERROR_H = 2u,              /* 高报警 */
    RELAY_ALARM_ERROR_L = 3u,              /* 低报警 */
    RELAY_ALARM_ERROR_LL_L = 4u,           /* 低低/低报警 */
    RELAY_ALARM_ERROR_ALL_ALARMS = 5u      /* 全部报警 */
} RelayAlarmErrorValue;

typedef enum {
    RELAY_ALARM_SOURCE_TANK_LEVEL = 0u,        /* 储罐液位 */
    RELAY_ALARM_SOURCE_LIQUID_TEMP = 1u,       /* 液相温度 */
    RELAY_ALARM_SOURCE_WATER_LEVEL = 2u,       /* 水位 */
    RELAY_ALARM_SOURCE_DISPLACER_POS = 3u,     /* 浮子位置 */
    RELAY_ALARM_SOURCE_NONE = 4u               /* 无 */
} RelayAlarmSource;

typedef enum {
    RELAY_ALARM_STATE_ACTIVE = 0u,     /* 激活，保持参考程序 0=激活 的语义 */
    RELAY_ALARM_STATE_INACTIVE = 1u    /* 未激活 */
} RelayAlarmState;

/* 继电器锁存报警清除命令值；该字段按瞬时命令处理，CPU2 消费后不得长期保持为“清除”。 */
typedef enum {
    /* 继电器锁存报警清除请求；YES 为一次性动作，消费后应恢复为 NO。 */
    RELAY_ALARM_CLEAR_NO = 0u, /* 不请求清除继电器锁存报警。 */
    RELAY_ALARM_CLEAR_YES = 1u /* 请求一次清除继电器锁存报警；CPU2 消费后复位。 */
} RelayAlarmClearCommand;

typedef struct {
    uint32_t operating_mode;     /* 工作模式 */
    uint32_t digital_source;     /* 输出报警位 */
    uint32_t contact_type;       /* 接点类型 */
    uint32_t alarm_mode;         /* 报警模式 */
    uint32_t error_value;        /* 报警值无效时的故障值策略 */
    uint32_t alarm_source;       /* 报警值源 */
    uint32_t HH_alarm_value;     /* 高高报警值，IEEE754 float 原始位 */
    uint32_t H_alarm_value;      /* 高报警值，IEEE754 float 原始位 */
    uint32_t L_alarm_value;      /* 低报警值，IEEE754 float 原始位 */
    uint32_t LL_alarm_value;     /* 低低报警值，IEEE754 float 原始位 */
    uint32_t alarm_hysteresis;   /* 报警滞回值，IEEE754 float 原始位 */
    uint32_t damping_factor;     /* 阻尼因子，预留给后续滤波 */
    uint32_t clear_alarm;        /* 清除锁存报警命令，CPU2 消费后清零 */
} RelayAlarmConfig;

/* 模拟量输出工作模式；区分关闭、4～20 mA 电流输出和 HART 从站输出，CPU2 与 CPU3 的枚举数值必须保持一致。 */
typedef enum {
    /* AO 工作模式，CPU2 与 CPU3 的数值约定必须一致。 */
    AO_WORK_MODE_DISABLED = 0u, /* 关闭 AO 输出服务并进入安全关闭路径。 */
    AO_WORK_MODE_CURRENT_OUTPUT = 1u, /* 启用 4～20 mA 电流输出路径。 */
    AO_WORK_MODE_HART_SLAVE_OUTPUT = 2u /* 启用 HART 从站输出路径。 */
} AoWorkMode;

typedef enum {
    /* AO 电流制式及固定输出模式选择。 */
    AO_CURRENT_MODE_NE = 0u, /* 采用 NAMUR NE43 电流制式边界。 */
    AO_CURRENT_MODE_US = 1u, /* 采用美国习惯电流制式边界。 */
    AO_CURRENT_MODE_NORMAL = 2u, /* 采用普通 4～20 mA 电流制式。 */
    AO_CURRENT_MODE_FIXED = 3u /* 忽略过程量并输出配置的固定电流。 */
} AoCurrentMode;

typedef enum {
    /* AO 过程量来源选择，决定 0%～100% 量程换算使用的输入值。 */
    AO_PROCESS_SOURCE_TANK_LEVEL = 0u, /* 以储罐油位作为 AO 过程量。 */
    AO_PROCESS_SOURCE_SENSOR_POSITION = 1u, /* 以传感器位置作为 AO 过程量。 */
    AO_PROCESS_SOURCE_WATER_LEVEL = 2u /* 以水位作为 AO 过程量。 */
} AoProcessSource;

/* AO 故障处理动作；决定故障期间输出配置的故障电流，或保持最近一次已经确认有效的输出。 */
typedef enum {
    /* AO 进入故障状态后的输出动作。 */
    AO_FAULT_ACTION_OUTPUT_CURRENT = 0u, /* 故障时输出配置的故障电流。 */
    AO_FAULT_ACTION_HOLD_LAST_VALID = 1u /* 故障时保持最近一次已经确认有效的电流。 */
} AoFaultAction;

typedef struct {
    /* AO 核心工作配置；CPU2 消费该结构决定工作模式、电流制式、过程量来源及电流零点修正。 */
    uint32_t work_mode; /* AO 工作模式，取值遵循 AoWorkMode。 */
    uint32_t current_mode; /* AO 电流制式或固定输出模式，取值遵循 AoCurrentMode。 */
    uint32_t output_source; /* AO 过程量来源，决定量程换算使用油位、位置或水位。 */
    int32_t current_correction_mA_x1000;     /* 电流修正值，单位0.001mA，复用原SIL/WHG预留槽 */
    uint32_t fixed_current_mA_x100; /* AO 固定电流模式的目标值，单位为 0.01 mA。 */
    int32_t range_0_01mm; /* AO 过程量量程的 0% 端点，单位为 0.1 mm。 */
    int32_t range_100_01mm; /* AO 过程量量程的 100% 端点，单位为 0.1 mm。 */
    uint32_t damping_x10_s; /* AO 阻尼时间的十倍定点值，单位为 0.1 s。 */
    uint32_t fault_mode;                    /* 故障动作：0输出故障电流，1保持上次有效过程电流 */
    uint32_t fault_current_mA_x100; /* 故障电流，单位为 0.01 mA；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t error_level;                   /* 隐藏预留槽位，固定为0 */
    uint32_t power_on_current_mA_x100;      /* 初始电流，保留原字段名和槽位 */
    uint32_t simulation_current_mA_x100; /* AO 仿真模式目标电流，单位为 0.01 mA。 */
} AoOutputConfig;

#define REPEATMAX 3 /* 重复性测试次数 */
#define REALTEMPMAXSPOT 16 /* 实时温度计最多测量点数 */
/* 液位盲区值 */
#define OILLEVELDOWNLIMIT 100
/* 无效值 */
#define UNVALID_LEVEL 999999u /* 液位无效值 */
#define LEVEL_DOWNLIMIT 100u /* 盲区液位值 */
#define UNVALID_TEMPERATURE_WIRELESS -2000 /* 温度无效值 */
#define UNVALID_DENSITY 0 /* 密度无效值 */
#define LEVEL_DOWNLIMITWATER 0 /* 水位无效值 */
#define UNVALID_QUALITY 0 /* 质量无效值 */
#define UNVALID_VOLUME 0 /* 体积无效值 */
#define UNVALID_X 0 /* 暂用于流速流量无效值 */

/* 报警状态 */
#define ALARMSTATE_NONE 0
/* 报警方向编码 1：表示低限或低低限侧报警。 */
#define ALARMSTATE_LOW 1
/* 报警方向编码 2：表示高限或高高限侧报警。 */
#define ALARMSTATE_HIGH 2


/**
 * @brief 系统错误码定义（2025-10修正版）
 * @note  错误码格式：0x00TT000N
 */

/**
 * @brief 系统错误码定义（2025-10 修正版）
 * @note  错误码格式: 0x00TT000N
 */

typedef enum {
    /* ==================== 正常状态 ==================== */
    NO_ERROR = 0,                    /* 正常状态 */
    STATE_SWITCH = 1,                /* 状态切换（非故障） */

    /* ==================== 11 电机驱动故障 (0x000B0000 - 0x000BFFFF) ==================== */
    MOTOR_TMC_COMM_ERROR = 0x000B0002,            /* 电机驱动寄存器通信异常 */
    MOTOR_DISABLED = 0x000B0004,                  /*电机驱动电流未建立*/
    MOTOR_UNKNOWN_FEEDBACK = 0x000B0005,          /*电机驱动故障类型未知*/
    MOTOR_STEP_ERROR = 0x000B000A,                /* 电机运动无有效位移 */
    MOTOR_CHARGE_PUMP_UNDER_VOLTAGE = 0x000B0010, /*电机驱动内部升压不足*/
    MOTOR_OVERTEMPERATURE = 0x000B0011,           /* 电机驱动过温关断 */
    MOTOR_RUN_TIMEOUT = 0x000B0012,               /* 电机整段运行超时 */
    MOTOR_TMC_CONFIG_LOST = 0x000B0003,           /* 电机驱动运行期复位或关键配置丢失 */
    MOTOR_STALL_ERROR = 0x000B000B,               /* 电机堵转 */
    MOTOR_PHASE_SHORT_ERROR = 0x000B0013,         /* 电机相线短路 */
    MOTOR_PHASE_OPEN_ERROR = 0x000B0014,          /* 电机相线断路 */
    MOTOR_DRIVER_OVERTEMP_WARNING = 0x000B0015,   /* 电机驱动过温预警 */
    MOTOR_DRIVER_NOT_INITIALIZED = 0x000B0016,    /* 电机驱动未初始化 */
    MOTOR_STOP_WAIT_TIMEOUT = 0x000B0017,         /*电机未按时停下*/
    MOTOR_ARRIVAL_WAIT_TIMEOUT = 0x000B0018,      /*电机未按时到位*/

    /* ==================== 12 编码器故障 (0x000C0000 - 0x000CFFFF) ==================== */
    ENCODER_TIMEOUT = 0x000C0001,                 /*编码器没有有效位置数据*/
    ENCODER_PARITY_ERROR = 0x000C0002,            /*编码器数据校验失败*/
    ENCODER_LOST_STEP = 0x000C0003,               /*编码器记录位移不足*/
    ENCODER_POWERON_FAIL = 0x000C0005,            /*编码器位置记录不可用*/
    ENCODER_POWERON_CHANGE = 0x000C0006,          /*编码器上电位置跳变（保留码）*/
    ENCODER_DIFF_EXCESS = 0x000C000A,             /*编码轮周长标定值过大*/
    ENCODER_CORDIC_OVERFLOW = 0x000C000C,         /*编码器角度计算超限*/
    ENCODER_LINEARITY_WARNING = 0x000C000D,       /*编码器角度线性异常（保留码）*/
    ENCODER_OCF_INCOMPLETE = 0x000C000E,          /*编码器角度计算未完成*/
    ENCODER_FIRST_SAMPLE_TIMEOUT = 0x000C000F,    /* 启动后首个有效位置等待超时 */
    ENCODER_CIRCUMFERENCE_CALIBRATION_ERROR = 0x000C0010, /* 编码轮周长标定异常 */

    /* ==================== 13 传感器与密度故障 (0x000D0000 - 0x000DFFFF) ==================== */
    SENSOR_BCC_ERROR = 0x000D0001,                /* 传感器数据校验失败 */
    SONIC_FREQ_ABNORMAL = 0x000D0002,             /*振动管频率异常*/
    SENSOR_DEVICE_COMM_TIMEOUT = 0x000D0003,      /* 传感器设备通信超时 */
    SENSOR_INTERNAL_CPU_COMM_TIMEOUT = 0x000D0005, /* 传感器内部处理单元通信超时 */
    SENSOR_GYRO_ANGLE_ERROR = 0x000D000E,         /* 传感器姿态角异常 */
    SENSOR_INTERNAL_COMM_CHECK_ERROR = 0x000D0010, /*传感器内部通信校验失败*/
    SENSOR_NO_RESONANCE = 0x000D0015,             /* 传感器无谐振 */
    DENSITY_INVALID = 0x000D001A,                 /*密度值超出有效范围*/
    SENSOR_RESP_FORMAT_ERROR = 0x000D001B,        /* 传感器响应格式异常 */
    SENSOR_POWER_SUPPLY_ERROR = 0x000D001C,       /* 传感器供电异常 */
    SENSOR_GYRO_COMM_TIMEOUT = 0x000D001D,        /* 传感器姿态模块通信超时 */
    SENSOR_SELF_TEST_FAILED = 0x000D001E,         /* 传感器自检失败 */
    SENSOR_IDENTITY_MISMATCH = 0x000D001F,        /* 传感器身份不匹配 */
    SENSOR_PROTOCOL_VERSION_INCOMPATIBLE = 0x000D0020, /* 传感器协议版本不兼容 */
    SENSOR_MODE_NOT_READY = 0x000D0021,           /* 传感器模式未就绪 */
    SENSOR_CONFIG_EPOCH_MISMATCH = 0x000D0022,    /*传感器配置批次不一致*/
    SENSOR_STREAM_STATE_ERROR = 0x000D0023,       /* 传感器周期上报状态不一致 */
    SENSOR_DATA_STALE = 0x000D0024,               /* 传感器数据已过期 */
    SENSOR_TEMPERATURE_RANGE_ERROR = 0x000D0025,  /* 传感器温度超出工作范围 */
    SENSOR_REMOTE_INTERNAL_ERROR = 0x000D0026,    /* 传感器远端内部故障 */
    SENSOR_ADDRESS_MISMATCH = 0x000D0027,         /* 传感器通信地址不匹配 */
    SENSOR_SESSION_INVALID = 0x000D0028,          /*传感器连接状态失效*/
    SENSOR_SEQUENCE_ERROR = 0x000D0029,           /*传感器应答序号不一致*/
    SENSOR_HANDSHAKE_REQUIRED = 0x000D002A,       /*传感器需要重新确认连接*/
    SENSOR_REPLAY_DETECTED = 0x000D002B,          /*收到重复的传感器数据*/
    SENSOR_CAPABILITY_UNSUPPORTED = 0x000D002C,   /*传感器不支持所需功能*/
    SENSOR_COMMAND_UNSUPPORTED = 0x000D002D,      /* 传感器命令不支持 */
    SENSOR_ARGUMENT_REJECTED = 0x000D002E,        /*传感器不接受当前参数*/
    SENSOR_MODE_MISMATCH = 0x000D002F,            /* 传感器测量模式不一致 */
    SENSOR_MODE_NOT_ALLOWED = 0x000D0030,         /*传感器当前模式不允许操作*/
    SENSOR_TRANSACTION_PENDING = 0x000D0031,      /*上一次传感器操作尚未完成*/
    SENSOR_DEVICE_BUSY = 0x000D0032,              /* 传感器设备忙 */
    SENSOR_PARAM_CRC_ERROR = 0x000D0033,          /* 传感器参数校验失败 */
    SENSOR_SAMPLE_COUNTER_ERROR = 0x000D0034,     /*传感器采样序号异常*/
    SENSOR_STREAM_STOPPED = 0x000D0035,           /* 传感器周期上报意外停止 */
    SENSOR_STREAM_NOT_ACTIVE = 0x000D0036,        /* 传感器周期上报未启动 */
    SENSOR_STREAM_ALREADY_ACTIVE = 0x000D0037,    /*传感器周期上报重复启动*/
    SENSOR_STREAM_EXIT_FAILED = 0x000D0038,       /* 传感器周期上报退出失败 */

    /* ==================== 14 零点与位置检测故障 (0x000E0000 - 0x000EFFFF) ==================== */
    MEASUREMENT_ZERO_OUT_OF_RANGE = 0x000E0009,   /*零点位置超出允许范围*/
    MEASUREMENT_ZERO_REPEAT_FAIL = 0x000E000B,    /*陀螺仪基准不稳定*/
    POSITION_DATA_INVALID = 0x000E000C,           /* 当前位置数据无效 */
    POSITION_TARGET_OVERRUN = 0x000E000D,         /* 运动越过目标位置 */
    POSITION_ARRIVAL_DEVIATION = 0x000E000E,      /*电机到位偏差过大*/
    POSITION_MOTOR_NOT_STOPPED = 0x000E000F,      /* 结果提交时电机仍未停止 */

    /* ==================== 15 测量过程故障 (0x000F0000 - 0x000FFFFF) ==================== */
    MEASUREMENT_OILLEVEL_HIGH = 0x000F0006,       /*上行找液位到达位置上限*/
    MEASUREMENT_OVERSPEED = 0x000F000F,           /* 液位变化速度异常 */
    MEASUREMENT_HEIGHT_DEVIATION = 0x000F0010,    /* 实高偏差过大，保留 */
    MEASUREMENT_OILLEVEL_LOW = 0x000F0012,        /* 下行未找到液位 */
    MEASUREMENT_OILLEVEL_NOTFOUND = 0x000F0013,   /* 上行未找到液位 */
    MEASUREMENT_WEIGHT_DOWN_FAIL = 0x000F0014,    /*下行未找到罐底*/
    MEASUREMENT_WEIGHT_UP_FAIL = 0x000F0015,      /*上行未找到零点*/
    MEASUREMENT_WATERLEVEL_LOW = 0x000F0016,      /* 下行未找到水位 */
    MEASUREMENT_DENSITY_NO_VALID_POINT = 0x000F0017, /* 密度测量无有效测点 */
    MEASUREMENT_DENSITY_SURFACE_NOTFOUND = 0x000F0018, /* 密度测量未找到油面 */
    MEASUREMENT_DENSITY_LEVEL_TIMEOUT = 0x000F0011, /* 密度闭环找液位超时 */
    MEASUREMENT_FREQUENCY_LEVEL_TIMEOUT = 0x000F0019, /* 频率闭环找液位超时 */
    MEASUREMENT_BOTTOM_RELEASE_FAIL = 0x000F001A, /*探底前未能离开罐底*/
    MEASUREMENT_TANK_HEIGHT_NOT_CONFIGURED = 0x000F001B, /* 未设置罐高标定值 */
    MEASUREMENT_WATER_CALIBRATION_NOT_CONFIGURED = 0x000F001C, /* 未设置水位标定值 */
    MEASUREMENT_DENSITY_PLAN_INVALID = 0x000F001D, /* 密度测点规划失败 */
    MEASUREMENT_TANK_HEIGHT_RESULT_INVALID = 0x000F001E, /* 罐高测量结果无效 */
    MEASUREMENT_WATER_CALC_OUT_OF_RANGE = 0x000F001F, /* 水位标定计算结果越界 */

    /* ==================== 17 参数与存储故障 (0x00110000 - 0x0011FFFF) ==================== */
    PARAM_EEPROM_FAIL = 0x00110001,               /* 参数存储读写失败 */
    PARAM_UNINITIALIZED = 0x00110002,             /* 参数存储未初始化 */
    PARAM_RANGE_ERROR = 0x00110005,               /* 单个参数值超出允许范围 */
    PARAM_CRC_ERROR = 0x00110006,                 /* 参数完整性校验失败 */
    PARAM_CONFIG_MISSING = 0x00110007,            /*关键参数未设置*/
    PARAM_COMBINATION_CONFLICT = 0x00110008,      /* 参数组合互相冲突 */
    PARAM_FEATURE_UNSUPPORTED = 0x00110009,       /* 当前配置不支持所选功能 */
    PARAM_STORAGE_SIZE_MISMATCH = 0x0011000A,     /* 参数存储结构大小不匹配 */
    PARAM_STORAGE_VERSION_MISMATCH = 0x0011000B,  /* 参数存储版本不匹配 */
    PARAM_STORAGE_WRITE_VERIFY_FAILED = 0x0011000C, /* 参数写入后校验失败 */

    /* ==================== 18 扭力检测故障 (0x00120000 - 0x0012FFFF) ==================== */
    WEIGHT_OUT_OF_RANGE = 0x00120001,             /*满载标定扭力超过上限*/
    WEIGHT_UNDER_RANGE = 0x00120002,              /*满载标定扭力低于下限*/
    WEIGHT_COLLISION_DETECTED = 0x00120003,       /*扭力变化超过保护值*/
    WEIGHT_DRIFT_ERROR = 0x00120004,              /*空载标定扭力偏离零点*/
    WEIGHT_SENSOR_SATURATION = 0x00120005,        /* 扭力传感器饱和，保留 */
    WEIGHT_COMM_TIMEOUT = 0x00120006,             /* 扭力通信超时 */

    /* ==================== 20 设备通信链路故障 (0x00140000 - 0x0014FFFF) ==================== */
    COMM_UART_TRANSFER_ERROR = 0x00140001,        /* 串口或DMA传输异常 */
    SLIPRING_COMM_FAIL = 0x00140002,              /*无线滑环通信失败*/
    SLIPRING_SIGNAL_WEAK = 0x00140003,            /*无线配对信号不达标*/
    WIRELESS_HOST_COMM_TIMEOUT = 0x00140004,      /* 无线主机通信超时 */
    WIRELESS_SLAVE_COMM_TIMEOUT = 0x00140005,     /* 无线从机未连接或无响应 */
    WIRELESS_RESP_FORMAT_ERROR = 0x00140006,      /* 无线模块响应格式异常 */
    CPU2_COMM_TIMEOUT = 0x00140007,               /* CPU3本机检测到CPU2通信连续失败 */
    WIRELESS_SCAN_NO_DEVICE = 0x00140008,         /*无线扫描未发现设备*/
    WIRELESS_NAME_NOT_UNIQUE = 0x00140009,        /* 无线名称重复 */
    WIRELESS_NAME_INVALID = 0x0014000A,           /* 无线名称参数无效 */
    WIRELESS_NOT_HOST_MODE = 0x0014000B,          /* 无线模块未处于主机模式 */
    WIRELESS_NAME_NOT_FOUND = 0x0014000C,         /* 无线名称未找到 */
    CPU2_PROFILE_SYNC_FAILED = 0x0014000D,         /* CPU3本机分布结果同步三轮仍不一致 */

    /* ==================== 21 模拟输出与自检故障 (0x00150000 - 0x0015FFFF) ==================== */
    AD5421_INIT_ERROR = 0x00150002,               /* 模拟输出芯片初始化兜底失败 */
    AD5421_READBACK_ERROR = 0x00150006,           /* 模拟输出控制寄存器回读不一致 */
    AD5421_INTERNAL_COMM_ERROR = 0x00150009,      /* 模拟输出芯片内部通信异常 */
    AD5421_LOOP_CURRENT_HIGH = 0x0015000A,        /* 模拟输出环路电流过高 */
    AD5421_LOOP_CURRENT_LOW = 0x0015000B,         /* 模拟输出环路电流过低或断环 */
    AD5421_LOOP_VOLTAGE_LOW = 0x0015000C,         /* 模拟输出环路供电电压不足 */
    AD5421_SPI_TRANSFER_ERROR = 0x0015000D,       /*主控与模拟量输出模块通信失败*/
    AD5421_ACCESS_BUSY = 0x0015000E,              /* 模拟输出访问冲突 */
    AD5421_OVERTEMP_SHUTDOWN = 0x0015000F,        /* 模拟输出芯片过温关断 */
    AD5421_OVERTEMP_WARNING = 0x00150010,         /* 模拟输出芯片过温预警 */

    /* ==================== 22 系统与软件故障 (0x00160000 - 0x0016FFFF) ==================== */
    SYSTEM_BUFFER_CAPACITY_ERROR = 0x00160001,    /*内部数据空间不足*/
    SYSTEM_CALL_CONDITION_ERROR = 0x00160002,     /* 内部调用参数或前置条件异常 */
    SYSTEM_CALCULATION_ERROR = 0x00160003,        /* 内部计算无法得到有效结果 */

    /* ==================== 23 整机供电与电源监控故障 (0x00170000 - 0x0017FFFF) ==================== */
    POWER_SUPPLY_24V_UNDERVOLTAGE = 0x00170001,   /* 整机24V供电低于安全阈值 */
    POWER_MONITOR_ADC_OVERRUN = 0x00170002,       /*电源采样数据处理不及时*/
    POWER_MONITOR_DMA_STOPPED = 0x00170003,       /*电源采样传输意外停止*/
    POWER_MONITOR_INIT_FAILED = 0x00170004,       /*电源采样启动失败*/
    POWER_MONITOR_RECOVERY_FAILED = 0x00170005,   /*电源采样连续恢复失败*/
    POWER_LOSS_POSITION_SAVE_FAILED = 0x00170006  /* 低压紧急位置保存未完成 */

} ErrorCode;

/* 测量命令 */
typedef enum {

    /* ======================= 基础 ======================= */
    CMD_NONE                       = 0,    /* 无命令 */

    /* ======================= 普通指令（测量类 1~99） ======================= */

    /* --- 基础测量动作 --- */
    CMD_BACK_ZERO                  = 1,    /* 回零点 */
    CMD_FIND_OIL                   = 2,    /* 寻找液位 */
    CMD_FIND_WATER                 = 3,    /* 寻找水位 */
    CMD_FIND_BOTTOM                = 4,    /* 寻找罐底 */

    CMD_MEASURE_SINGLE             = 5,    /* 单点测量 */
    CMD_MONITOR_SINGLE             = 6,    /* 单点监测 */
    CMD_SYNTHETIC                  = 7,    /* 综合测量 */

    /* --- 跟随 / 运动控制类（新增，建议 8~9 占位） --- */
    CMD_FOLLOW_WATER               = 8,    /* 水位跟随（新增） */
    CMD_RUN_TO_POSITION            = 9,    /* 电机运行到指定位置（新增） */

    /* --- 密度分布测量（整个系列，与普通液位分布区分） --- */
    CMD_MEASURE_DISTRIBUTED        = 10,   /* 普通分布测量 */
    CMD_GB_MEASURE_DISTRIBUTED     = 11,   /* 国标分布测量 */
    CMD_MEASURE_DENSITY_METER      = 12,   /* 密度每米测量 */
    CMD_MEASURE_DENSITY_RANGE      = 13,   /* 区间密度测量 */
    CMD_WARTSILA_DENSITY_RANGE     = 14,   /* 瓦西莱密度区间测量 */

    /* --- 参数读取类（新增，放在 15，避免占用你后续密度扩展） --- */
    CMD_READ_PART_PARAMS           = 15,   /* 读取部件参数（新增） */
    CMD_CANCEL_MEASUREMENT         = 16,   /* 取消当前测量并进入待机 */

    /* 普通指令预留 */
    CMD_SI_PROFILE             = 20,
    CMD_RESERVED_CMD2              = 21,
    CMD_RESERVED_CMD3              = 22,

    /* ======================= 调试/强制指令（100~199） ======================= */

    CMD_DEBUG_MODE                 = 100,  /* 进入调试模式 */

    CMD_CALIBRATE_ZERO             = 101,  /* 标定零点 */
    CMD_CALIBRATE_OIL              = 102,  /* 标定液位 */
    CMD_CORRECT_OIL                = 103,  /* 修正液位 */

    CMD_MOVE_UP                    = 104,  /* 上行 */
    CMD_MOVE_DOWN                  = 105,  /* 下行 */

    CMD_SET_EMPTY_WEIGHT           = 106,  /* 设置空载扭力 */
    CMD_SET_FULL_WEIGHT            = 107,  /* 设置满载扭力 */
    CMD_RESTORE_FACTORY            = 108,  /* 恢复出厂设置 */
    CMD_MAINTENANCE_MODE           = 109,  /* 维护模式 */

    /* --- 强制运动 / 强制位置类（新增） --- */
    CMD_FORCE_MOVE_UP              = 113,  /* 电机强制上行（新增） */
    CMD_FORCE_MOVE_DOWN            = 114,  /* 电机强制下行（新增） */
    CMD_RESERVED_CMD7              = 115,  /* 保留，原115指令已删除 */

    /* --- 水位标定（新增，建议归类到标定类） --- */
    CMD_CALIBRATE_WATER            = 116,  /* 水位标定（新增） */
    CMD_PAIR_NEAREST_WIRELESS_SLIPRING = 117, /* 匹配最近无线滑环 */
    CMD_MAINTENANCE_EXIT           = 118,  /* 退出维护模式 */
    CMD_CLEAR_ALL_RELAY_LATCHED_ALARMS = 119,  /* 清除全部继电器锁存报警 */

    /* 调试预留 */
    CMD_CALIBRATE_TANKHEIGHT       = 110,  /* 罐高标定 */
    CMD_RESERVED_CMD5              = 111,
    CMD_RESERVED_CMD6              = 112,

    /* ======================= 其他 ======================= */
    CMD_UNKNOWN                    = 255   /* 未知命令 */

} CommandType;

typedef enum {
    CMD_NONE_DEF                       = 0,    /* 无命令 */
    CMD_BACK_ZERO_DEF                  = 1,    /* 回零点 */
    CMD_FIND_OIL_DEF                   = 2,    /* 寻找液位 */
    CMD_MONITOR_SINGLE_DEF             = 3,    /* 单点监测 */
    CMD_FOLLOW_WATER_DEF               = 4,    /* 水位跟随（新增） */
} DefaultCommandType;

/* 设备状态 */
typedef enum {
    STATE_STANDBY = 0x0000,                   /* 待机状态 */
    STATE_INIT = 0x0001,                      /* 初始化状态 */
    STATE_BACKZEROING = 0x0002,               /* 回零点中 */
    STATE_FINDZEROING = 0x0010,               /* 标定零点中 */
    STATE_SINGLEPOINTING = 0x0011,            /* 单点测量中 */
    STATE_RUNTOPOINTING = 0x0012,             /* 运行到测量点中 */
    STATE_GB_SPREADPOINTING = 0x0013,         /* 国标分布测量中 */
    STATE_SPREADPOINTING = 0x0014,            /* 分布测量中 */
    STATE_CALIBRATIONOILING = 0x0015,         /* 标定液位 */
    STATE_READPARAMETERING = 0x0016,          /* 读取参数中 */
    STATE_RUNUPING = 0x0017,                  /* 向上运行中 */
    STATE_RUNDOWNING = 0x0018,                /* 向下运行中 */
    STATE_SETZEROCIRCLING = 0x0019,           /* 设置零点编码值 */
    STATE_SETZEROANGLING = 0x001A,            /* 设置零点编码值 */
    STATE_EFACTORYSETTING_RESTORING = 0x001B, /* 恢复出厂设置中 */
    STATE_BACKUPING = 0x001C,                 /* 备份配置文件中 */
    STATE_RESTORYING = 0x001D,                /* 恢复配置文件中 */
    STATE_FINDOIL = 0x001E,                   /* 寻找液位中 */
    STATE_FINDWATER = 0x001F,                 /* 寻找水位中 */
    STATE_FINDBOTTOM = 0x0020,                /* 寻找罐底中 */
    STATE_FORCEZERO = 0x0021,                 /* 设置电机零点中 */
    STATE_ONTANKOPRATIONING = 0x0022,         /* 罐上仪表操作中 */
    STATE_SYNTHETICING = 0x0023,              /* 综合指令中 */

    /* ===================== LTD / 新增测量中状态（顺延） ===================== */
    STATE_FOLLOW_WATER_POINT_SEARCHING = 0x0024, /* 寻找水位跟随点中 */
    STATE_METER_DENSITY = 0x0025,              /* 密度每米测量中 */
    STATE_INTERVAL_DENSITY = 0x0026,           /* 液位区间测量中 */
    STATE_GET_FULLWEIGHT = 0x0027,             /* 获取满载扭力中 */
    STATE_GET_EMPTYWEIGHT = 0x0028,            /* 获取空载扭力中 */
    STATE_MAINTENANCEMODE = 0x0029,            /* 维护模式中 */
    STATE_WARTSILA_DENSITY_START = 0x002A,     /* 瓦西莱密度梯度测量开始 */
    STATE_WARTSILA_DENSITY_MEASURING = 0x002B, /* 瓦西莱密度梯度测量中 */
    STATE_RUN_TO_POSITIONING = 0x002C,         /* 运行到指定位置中 */
    STATE_FORCE_RUNUPING = 0x002D,             /* 电机强制上行中 */
    STATE_FORCE_RUNDOWNING = 0x002E,           /* 电机强制下行中 */
    STATE_RESERVED_002F = 0x002F,              /* 保留状态 */
    STATE_CALIBRATE_WATERING = 0x0030,         /* 水位标定中 */
    STATE_CALIBRATE_TANKHEIGHTING = 0x0031,     /* 罐高标定中 */
    STATE_WIRELESS_PAIRING = 0x0032,            /* 无线滑环匹配中 */
    STATE_DEBUG_MODE = 0x0033,                  /* 调试模式中 */

    /* ===================== 完成态（0x80xx） ===================== */
    STATE_FINDZEROOVER = 0x8010,               /* 标定零点完成 */
    STATE_SINGLEPOINTOVER = 0x8011,            /* 单点测量完成 */
    STATE_SPTESTING = 0x8012,                  /* 正在单点检测 */
    STATE_GB_SPREADPOINTOVER = 0x8013,         /* 国标分布测量完成 */
    STATE_SPREADPOINTOVER = 0x8014,            /* 分布测量完成 */
    STATE_FINDOILOVER = 0x8015,                /* 标定液位完成 */
    STATE_READPARAMETEROVER = 0x8016,          /* 读取参数完成 */
    STATE_RUNUPOVER = 0x8017,                  /* 向上运行完成 */
    STATE_RUNDOWNOVER = 0x8018,                /* 向下运行完成 */
    STATE_SETZEROCIRCLOVER = 0x8019,           /* 设置零点编码值完成 */
    STATE_SETZEROANGLOVER = 0x801A,            /* 设置零点编码值完成 */
    STATE_EFACTORYSETTING_RESTOROVER = 0x801B, /* 恢复出厂设置完成 */
    STATE_BACKUPOVER = 0x801C,                 /* 备份配置文件完成 */
    STATE_RESTORYOVER = 0x801D,                /* 恢复配置文件完成 */
    STATE_FLOWOIL = 0x801E,                    /* 液位跟随中（旧逻辑） */
    STATE_FINDWATER_OVER = 0x801F,             /* 寻找水位完成 */
    STATE_FINDBOTTOM_OVER = 0x8020,            /* 寻找罐底完成 */
    STATE_FORCEZERO_OVER = 0x8021,             /* 设置电机零点完成 */
    STATE_ONTANKOPRATIONCOMPLATE = 0x8022,     /* 罐上仪表操作完成 */
    STATE_SYNTHETICING_OVER = 0x8023,          /* 综合指令完成 */

    /* ===================== LTD / 新增完成态 ===================== */
    STATE_FOLLOW_WATERING = 0x8024,            /* 水位跟随中（沿用0x80xx状态码） */
    STATE_COM_METER_DENSITY_OVER = 0x8025,      /* 密度每米测量完成 */
    STATE_INTERVAL_DENSITY_OVER = 0x8026,       /* 液位区间测量完成 */
    STATE_GET_FULLWEIGHT_OVER = 0x8027,         /* 获取满载扭力完成 */
    STATE_GET_EMPTYWEIGHT_OVER = 0x8028,        /* 获取空载扭力完成 */
    STATE_WARTSILA_DENSITY_OVER = 0x8029,       /* 瓦西莱密度梯度测量完成 */
    STATE_RUN_TO_POSITION_OVER = 0x802C,        /* 运行到指定位置完成 */
    STATE_FORCE_RUNUP_OVER = 0x802D,            /* 强制上行完成 */
    STATE_FORCE_RUNDOWN_OVER = 0x802E,          /* 强制下行完成 */
    STATE_RESERVED_802F = 0x802F,              /* 保留状态 */
    STATE_CALIBRATE_WATER_OVER = 0x8030,        /* 水位标定完成 */
    STATE_CALIBRATE_TANKHEIGHT_OVER = 0x8031,   /* 罐高标定完成 */
    STATE_WIRELESS_PAIRING_OVER = 0x8032,       /* 无线滑环匹配完成 */

    STATE_ERROR = 0xFFFF                        /* 故障 */
} DeviceState;

/**
 * @brief 判断当前设备状态是否允许尝试写入CPU2持久参数。
 *
 * @details 调用场景：CPU2最终写门禁和CPU3菜单、外部协议预检查。
 * @note 关键约束：只列出已确认空闲的完成态；持续态、预留态和未来新增状态默认拒绝。
 *
 * @param state 待检查的 CPU2 设备状态；函数按共享持久参数写白名单决定是否允许写入。
 * @return true 表示当前设备状态允许尝试写入CPU2持久参数；false 表示当前设备状态不允许尝试写入CPU2持久参数。
 */
static inline bool DeviceState_AllowsPersistentParamWrite(DeviceState state)
{
    switch (state) {
    case STATE_STANDBY:
    case STATE_FINDZEROOVER:
    case STATE_SINGLEPOINTOVER:
    case STATE_GB_SPREADPOINTOVER:
    case STATE_SPREADPOINTOVER:
    case STATE_RUNUPOVER:
    case STATE_RUNDOWNOVER:
    case STATE_FINDWATER_OVER:
    case STATE_FINDBOTTOM_OVER:
    case STATE_SYNTHETICING_OVER:
    case STATE_COM_METER_DENSITY_OVER:
    case STATE_INTERVAL_DENSITY_OVER:
    case STATE_GET_FULLWEIGHT_OVER:
    case STATE_GET_EMPTYWEIGHT_OVER:
    case STATE_RUN_TO_POSITION_OVER:
    case STATE_FORCE_RUNUP_OVER:
    case STATE_FORCE_RUNDOWN_OVER:
    case STATE_CALIBRATE_WATER_OVER:
    case STATE_CALIBRATE_TANKHEIGHT_OVER:
    case STATE_WIRELESS_PAIRING_OVER:
    case STATE_ERROR:
        return true;
    default:
        return false;
    }
}


/* 设备状态结构体 */
typedef struct {
	/* ---- 设备核心状态 ---- */
	uint32_t work_mode;          /* 工作模式（0:工作模式 1.调试模式 78.解锁模式） */
	DeviceState device_state;    /* 设备运行状态 */
	uint32_t error_code;         /* 当前错误码（0表示无错误） */
	CommandType current_command; /* 当前指令 */

	/* ---- 标志位 ---- */
	uint32_t zero_point_status; /* 零点状态（0-正常 1-需要回零） */
	uint32_t parameter_update_flag; /* 参数持久化完成代次，FRAM A/B 确认一致后递增 */
    uint32_t loading_unloading_active;        /* /< 当前是否处于装卸液过程 */
    uint32_t manual_alarm_inhibit;            /* /< 手动运行期间报警抑制 */
    uint32_t maintenance_mode_active;         /* /< 非阻塞维护模式，非持久化且不覆盖主运行状态 */
    uint32_t relay_alarm_inhibit_effective;   /* /< 维护或人工动作造成的继电器最终屏蔽状态 */
    uint32_t relay_alarm_action_mask;         /* /< bit0~bit3为K1~K4最终逻辑报警动作 */
} DeviceStatus;

/* 单点密度数据 */
typedef struct {
	uint32_t temperature;          /* /< 温度 */
	uint32_t density;              /* /< 密度 */
	uint32_t temperature_position; /* /< 温度密度位置 */
	uint32_t standard_density;     /* /< 标准密度 */
	uint32_t vcf20;                /* /< 体积修正系数 (VCF20) */
	uint32_t weight_density;       /* /< 计重密度 */
} DensityMeasurement;
/**
 * @brief 密度分布数据
 */
typedef struct {
	uint32_t average_temperature;                                   /* /< 温度 */
	uint32_t average_density;                                       /* /< 密度 */
	uint32_t average_standard_density;                              /* /< 标准密度 */
	uint32_t average_vcf20;                                         /* /< 体积修正系数 (VCF20) */
	uint32_t average_weight_density;                                /* /< 计重密度 */
	uint32_t measurement_points;                                    /* /< 实际测量点数 */
    uint32_t Density_oil_level;                                     /* 密度分布测量时的液位值 */
    uint32_t profile_complete_latched;        /* /< 分布测量完成锁存 */
    uint32_t profile_complete_counter;        /* /< 分布测量完成计数 */
    uint32_t profile_source;                  /* profile 结果来源：0=无，1=普通，2=国标，3=每米，4=间隔，5=Wartsila，6=SI */
    uint32_t profile_blocked_by_process;      /* /< 分布测量是否被工况阻止 */
    uint32_t profile_temp_deviation_alarm;    /* /< 分布温度偏差报警 */
    uint32_t profile_density_deviation_alarm; /* /< 分布密度偏差报警 */
	DensityMeasurement single_density_data[MAX_MEASUREMENT_POINTS]; /* 200个点的密度测量数据 */

} DensityDistribution;

/**
 * @brief  调试数据
 */
typedef struct {
    /* 位置相关 */
	int32_t current_encoder_value; /* /< 当前编码值 */
	int32_t sensor_position;       /* /< 传感器位置 */
	int32_t cable_length;          /* /< 尺带长度; */
	int32_t motor_step;            /* /< 电机步进值 */
	int32_t motor_distance;        /* /< 电机距离值(单位: 0.1mm) */

    /* 频率与温度、电压 */
	uint32_t frequency;            /* /< 当前频率 */
	uint32_t temperature;          /* /< 温度 (单位: 0.01°C) */
	uint32_t air_frequency;        /* /< 空气中频率 */
	uint32_t current_amplitude;    /* /< 当前幅值 */
	uint32_t water_capacitance_x10; /* /< 水位电容快照(单位: 0.1pF) */

    /* 扭力相关 */
    uint32_t current_weight;       /* /< 当前扭力值 */
    uint32_t torque_temperature_bits; /* /< 扭力模块温度IEEE754原始位 */

    /* 姿态角 */
    int32_t  angle_x;              /* /< X 轴角度 */
    int32_t  angle_y;              /* /< Y 轴角度 */

    /* 电机状态相关 */
    uint32_t motor_speed;          /* /< motor speed (0.01m/min) */
    uint32_t  motor_state;          /* /< 电机状态: 0 停止, 1 上行, 2 下行 */

    /* 多参数传感器调试测量结果 */
    float magnetic_zero_voltage;   /* /< 磁零点电压，单位 V */
    float dynamic_viscosity_cp;    /* /< 动力黏度，单位 cP */
    float kinematic_viscosity_cst; /* /< 运动黏度，单位 cSt */
    float supply_voltage_v;        /* /< 传感器供电电压，单位 V */
} DebugData;


/**
 * @brief  实高测量数据
 */
typedef struct {
	uint32_t calibrated_liquid_level; /* /< 标定液位时实高 */
	uint32_t current_real_height;     /* /< 当前实高 */
    uint32_t bottom_reference_valid;  /* /< 罐底参考位置是否有效 */
} ActualHeightMeasurement;

/**
 * @brief  液位测量数据
 */
typedef struct {
	uint32_t oil_level;                          /* /< 液位跟随液位值 */
	uint32_t air_frequency;		/* 空气中频率 */
	uint32_t oil_frequency;		/* 油中频率 */
	uint32_t follow_frequency;		/* 液位跟随频率 */
	uint32_t current_frequency;	/* 当前频率 */
    uint32_t probe_at_liquid_level;       /* /< 探头当前是否位于液位点 */
    uint32_t liquid_stable;               /* /< 液体是否稳定 */
    uint32_t manual_level_update_inhibit; /* /< 手动运行期间液位更新抑制 */
} OilMeasurement;

/**
 * @brief  液位测量数据
 */
typedef struct {
	uint32_t water_level;                        /* /< 测量水位的值 */
	float zero_capacitance;
	/* 水位测量过程中用于比较油相基准和当前位置的电容结果。 */
	float oil_capacitance; /* 进入水位判定前锁存的油相基准电容。 */
	float current_capacitance; /* 当前位置最新测得的水位探头电容。 */
} WaterMeasurement;

typedef enum {
    WIRELESS_PAIRING_RESULT_NONE = 0U,      /* 未执行或无结果 */
    WIRELESS_PAIRING_RESULT_RUNNING = 1U,   /* 正在匹配 */
    WIRELESS_PAIRING_RESULT_SUCCESS = 2U,   /* 匹配成功 */
    WIRELESS_PAIRING_RESULT_FAILED = 3U     /* 匹配失败 */
} WirelessPairingResult;

typedef struct {
    float alarm_value;        /* 当前报警值，参考程序 alarm_para_onlyread.alarm_value */
    uint32_t HH_alarm;        /* 高高报警，0=激活，1=未激活 */
    uint32_t H_alarm;         /* 高报警，0=激活，1=未激活 */
    uint32_t HH_H_alarm;      /* 高高或高报警，0=激活，1=未激活 */
    uint32_t L_alarm;         /* 低报警，0=激活，1=未激活 */
    uint32_t LL_alarm;        /* 低低报警，0=激活，1=未激活 */
    uint32_t LL_L_alarm;      /* 低低或低报警，0=激活，1=未激活 */
    uint32_t any_error;       /* 任意报警，0=激活，1=未激活 */
    uint32_t clear_alarm;     /* 清除锁存报警命令运行态，参考程序不存储类参数 */
} RelayAlarmRuntimeState;

typedef struct {
    uint32_t result;                         /* 无线滑环匹配结果 */
    uint32_t mac_valid;                      /* MAC 是否有效 */
    uint32_t mac_high;                       /* AA:BB */
    /* CPU2/CPU3 共享的无线配对状态字段。 */
    uint32_t mac_mid;                        /* CC:DD */
    uint32_t mac_low;                        /* EE:FF */
    uint32_t error_code;                     /* 失败时的 CPU2 错误码 */
    uint32_t update_counter;                 /* CPU2 每次状态变化递增 */
    uint32_t connection_valid;               /* 当前蓝牙连接是否有效 */
    uint32_t rssi_valid;                     /* 当前蓝牙 RSSI 是否有效 */
    int32_t rssi;                            /* 当前蓝牙 RSSI，单位 dB */
    uint32_t connection_error_code;          /* 当前连接/RSSI 查询错误码 */
    uint32_t rssi_update_counter;            /* CPU2 每次 RSSI 快照查询递增 */
} WirelessPairingStatus;

typedef struct {
    uint32_t target_mA_x1000;                /* AO目标电流，单位0.001mA */
    uint32_t last_sent_mA_x1000;             /* AO最近一次成功写入电流，单位0.001mA */
    uint32_t source;                         /* AO输出来源，参考CPU2 AoOutputSource */
    uint32_t driver_fault_flags;             /* AD5421驱动故障标志 */
    uint32_t driver_fault_register;          /* AD5421 READFAULT原始值 */
    uint32_t last_error_code;                /* AO最近一次错误码 */
    uint32_t update_counter;                 /* AO运行态更新计数 */
    uint32_t last_update_tick;               /* AO最近一次更新tick */
    uint32_t last_sent_tick;                 /* AO最近一次成功写入tick */
    int32_t process_value_01mm;               /* 当前选中过程量，单位0.1mm */
    int32_t percent_x100;                     /* 当前过程百分比，单位0.01% */
    uint32_t process_valid;                   /* 当前选中过程量有效标志 */
    uint32_t simulation_enabled;              /* AO仿真运行态开关，上电强制关闭 */
    uint32_t dac_readback_mA_x100;            /* DAC回读预留值，单位0.01mA */
    uint32_t dac_readback_valid;              /* DAC回读预留有效标志，当前固定为0 */
} AoOutputRuntime;

/* 测量结果结构体，输入寄存器 */
typedef struct {
	DeviceStatus device_status;                  /* /< 设备状态 */
	DebugData debug_data;                        /* /< 调试数据 */
	OilMeasurement oil_measurement;                /* /< 液位测量数据 */
	WaterMeasurement water_measurement;          /* /< 水位测量数据 */
	ActualHeightMeasurement height_measurement;  /* /< 实高测量数据 */
	DensityMeasurement single_point_measurement; /* /< 单点测量数据 */
	DensityMeasurement single_point_monitoring;  /* /< 单点监测数据 */
	DensityDistribution density_distribution;    /* /< 密度分布测量数据 */
	SiProfileRuntime si_profile_runtime;          /* /< SI Profile生命周期运行态 */
	WirelessPairingStatus wireless_pairing_status; /* /< 无线滑环匹配状态 */
	RelayAlarmRuntimeState relay_alarm_runtime[RELAY_ALARM_CHANNEL_COUNT]; /* /< 继电器报警输出每路运行态 */
	AoOutputRuntime ao_output_runtime;           /* /< AO模拟电流输出运行态 */
	uint32_t measurement_complete_counter;       /* /< 单点测量真实稳定结果代际 */
	uint32_t monitoring_sample_counter;          /* /< 固定点监测真实稳定样本代际 */

} MeasurementResult;

/* 设备参数结构体 */
/* #pragma pack(push, 1) / / 确保结构体紧凑对齐（防止编译器填充） */
typedef struct {
    /* 指令 */
    CommandType command;                  /* 当前指令 */

    /* ===================== 基础参数 ===================== */
    uint32_t sensorType;                  /* 传感器类型 */
    uint32_t sensorID;                    /* 传感器编号 */
    uint32_t sensorSoftwareVersion;       /* 传感器软件版本 */
    uint32_t softwareVersion;             /* LTD 软件版本 */
    CommandType powerOnDefaultCommand;    /* 上电默认指令 */
    uint32_t error_auto_back_zero;        /* 错误自动回零标志(0/1) */
    uint32_t error_stop_measurement;      /* 错误停止测量标志(0/1) */

    uint32_t protocolVersion;             /* CPU2/CPU3共享协议版本，旧程序该字段默认为0 */
    uint32_t fault_auto_recovery_retry_limit; /* 故障自动恢复重跑上限：0关闭，1~10为最多重跑次数 */
    uint32_t water_level_hysteresis_time_s; /* 水位滞后时间预留参数，单位s，当前不参与运行逻辑 */

    /* ===================== 电机与编码器参数 ===================== */
    uint32_t position_source_auto_switch; /* 位置源自动切换(0=不切换,1=自动切换) */
    uint32_t motor_current;               /* 电机运行电流(1~31，异常恢复为16) */
    uint32_t encoder_wheel_circumference_mm; /* 编码轮周长(0.001mm) */
    uint32_t max_motor_speed;                 /* max motor speed (0.01m/min) */
    uint32_t first_loop_circumference_mm;     /* 尺带首圈周长(0.1mm) */
    uint32_t tape_thickness_mm;               /* 尺带厚度(0.001mm) */

    uint32_t position_count_mode;          /* 当前记步模式(0=编码轮,1=电机步进) */
    uint32_t motor_count_first_loop_circumference_mm; /* 电机记步局部首圈周长(0.001mm) */

    /* ===================== 扭力参数 ===================== */
    int32_t empty_weight;                 /* 空载扭力 */
    uint32_t empty_weight_upper_limit;    /* 空载扭力上限 */
    uint32_t empty_weight_lower_limit;    /* 空载扭力下限 */
    uint32_t full_weight;                 /* 满载扭力 */
    uint32_t full_weight_upper_limit;     /* 满载扭力上限 */
    uint32_t full_weight_lower_limit;     /* 满载扭力下限 */
    uint32_t weight_upper_limit_ratio;    /* 扭力变化量检测上限比例 */
    uint32_t weight_lower_limit_ratio;    /* 扭力变化量检测下限比例 */

    uint32_t reserved8;                   /* 预留 */
    uint32_t reserved9;                   /* 预留（新增） */

    /* ===================== 零点测量 ===================== */
    uint32_t zero_weight_threshold_ratio;     /* 零点扭力阈值比例 */
    uint32_t weight_ignore_zone;              /* 零点下扭力不检测区域(建议 0.1mm) */
    uint32_t max_zero_deviation_distance;     /* 零点最大偏差距离(建议 0.1mm) */
    uint32_t findZeroDownDistance;            /* 找零点完成后下行距离(0.1mm) */

    uint32_t reserved10;                  /* 预留 */
    uint32_t reserved11;                  /* 预留（新增） */

    /* ===================== 液位测量 ===================== */
    uint32_t tankHeight;                     /* 液位罐高(0.1mm) */
    uint32_t liquid_sensor_distance_diff;    /* 液位传感器距离差(0.1mm) */
    uint32_t blindZone;                      /* 液位盲区(0.1mm) */
    uint32_t oilLevelThreshold;              /* 液位跟随阈值 */
    uint32_t oilLevelHysteresisThreshold;    /* 液位滞后阈值 */
    uint32_t liquidLevelMeasurementMethod;   /* 液位测量方式：0相对频率 1定频 2密度 3超声 4连续相对频率 5连续定频 */
    uint32_t oilLevelFrequency;                 /* 液位跟随频率 */
    uint32_t oilLevelDensity;                   /* 液位跟随密度 */
    uint32_t oilLevelHysteresisTime;            /* 液位滞后时间 */


    /* ===================== 水位测量参数 ===================== */
    uint32_t water_tank_height;                 /* 水位罐高(0.1mm) */
    uint32_t water_level_mode;                  /* 水位测量方式 */
    uint32_t waterBlindZone;                    /* 水位盲区(0.1mm) */
    uint32_t water_cap_threshold;               /* 水位电容阈值（建议明确倍率，如 x1000） */
    uint32_t water_find_cap_threshold;              /* 水位寻找电容阈值（建议明确倍率，如 x1000） */
    uint32_t maxDownDistance;                   /* 水位/罐底测量最大下行距离(0.1mm) */
    uint32_t zero_cap;                          /* 零点电容值 */
    uint32_t water_stable_threshold;            /* 水位稳定距离 */
    uint32_t waterLevelCorrection;              /* 水位修正值 */

    /* ===================== 罐高/罐底测量 ===================== */
    uint32_t bottom_detect_mode;          /* 罐底测量模式 */
    uint32_t bottom_angle_threshold;      /* 罐底角度阈值（务必明确单位/倍率） */
    uint32_t bottom_weight_threshold;     /* 罐底扭力阈值 */
    uint32_t refreshTankHeightFlag;       /* 是否更新液位罐高 */
    uint32_t maxTankHeightDeviation;      /* 罐高最大变化范围 */
    uint32_t initialTankHeight;           /* 初始实高 */
    uint32_t currentTankHeight;           /* 当前实高 */
    uint32_t bottom_encoder_correction_enable; /* 罐底测量完成后修正编码器(0=不修正,1=修正) */
    uint32_t water_lag_cap_threshold;  /* 水位滞后电容阈值(x1000)，稳定监测中偏离目标超过该值后返回跟随 */

    /* ===================== 密度和温度修正参数 ===================== */
    uint32_t densityCorrection;           /* 密度修正值、磁通量D */
    uint32_t temperatureCorrection;       /* 温度修正值、磁通量T */

    uint32_t reserved18;                 /* 预留 */
    uint32_t reserved19;                 /* 预留（新增） */

    /* ===================== 分布/区间测量参数 ===================== */
    uint32_t requireBottomMeasurement;         /* 是否需要测量罐底 */
    uint32_t requireWaterMeasurement;          /* 是否需要测量水位 */
    uint32_t requireSinglePointDensity;        /* 是否需要测量单点密度 */
    uint32_t spreadMeasurementOrder;           /* 分布测量顺序 */
    uint32_t spreadMeasurementMode;            /* 分布测量模式 */
    uint32_t spreadMeasurementCount;           /* 分布测量数量 */
    uint32_t spreadMeasurementDistance;        /* 分布测量间距 */
    uint32_t spreadTopLimit;                   /* 分布测量上限（距液面，0.1mm） */
    uint32_t spreadBottomLimit;                /* 分布测量下限（距罐底，0.1mm） */
    uint32_t spreadPointHoverTime;             /* 第一测量点悬停时间 */
    uint32_t intervalMeasurementTopLimit;      /* 区间测量上限（距液面，0.1mm） */
    uint32_t intervalMeasurementBottomLimit;   /* 区间测量下限（距罐底，0.1mm） */

    uint32_t reserved20;                 /* 预留 */
    uint32_t reserved21;                 /* 预留（新增） */

    /* ===================== Wartsila 密度区间测量参数 ===================== */
    uint32_t wartsila_upper_density_limit;        /* 上限 */
    uint32_t wartsila_lower_density_limit;        /* 下限 */
    uint32_t wartsila_density_interval;           /* 步进 */
    uint32_t wartsila_max_height_above_surface;   /* 最高测点距液面距离(0.1mm 或按定义) */

    uint32_t wartsila_bottom_detect_interval; /* 瓦锡兰测量后探底频率：0不探底，N表示每N次测量后探底一次，范围0~100 */
    uint32_t bottom_encoder_correction_tank_height; /* 探底修正罐高，仅用于罐底后编码器修正，0表示沿用液位罐高 */

    /* ===================== 4-20mA 输出 ===================== */
    AoOutputConfig ao_output;            /* 协议20 AO配置，保持原连续13字段尺寸 */

    /* ===================== 指令参数 ===================== */
    uint32_t calibrateOilLevel;              /* 标定液位值 */
    uint32_t calibrateWaterLevel;            /* 水位标定值 */
    uint32_t calibrateTankHeight;           /* 罐高标定值 */
    uint32_t singlePointMeasurementPosition; /* 单点测量位置 */
    uint32_t singlePointMonitoringPosition;  /* 单点监测位置 */
    uint32_t densityDistributionOilLevel;    /* 密度分布测量时的液位值 */
    uint32_t motorCommandDistance;           /* 电机指令的运行距离 */

    uint32_t reserved28;                 /* 预留 */
    uint32_t reserved29;                 /* 预留（新增） */

    /* ===================== Tape compensation ===================== */
    uint32_t lastOilCorrectionLevel;     /* 上次液位修正时液位值 */
    uint32_t tankGasPhaseTemperature;    /* 储罐内气相温度 */
    uint32_t tapeExpansionCoefficient;   /* 尺带膨胀系数 */
    uint32_t tapeCalibrationTemperature; /* 标定尺带时温度 */

    uint32_t si_profile_first_point;              /* SI profile首个停点，单位0.1mm */
    uint32_t si_profile_increment;                /* SI profile点间距，单位0.1mm */
    uint32_t si_profile_dwell_time;               /* SI profile每点停稳等待时间，单位s */
    uint32_t si_profile_bottom_detect_interval;   /* SI profile探底频次，1表示每次探底 */

    /* ===================== 继电器报警输出配置（四路） ===================== */
    RelayAlarmConfig relayAlarm[RELAY_ALARM_CHANNEL_COUNT];

    /* ===================== 元信息与校验 ===================== */
    uint32_t param_version;              /* 参数结构版本号 */
    uint32_t struct_size;                /* sizeof(DeviceParameters) */
    uint32_t magic;                      /* 0x4C54444D = 'LTDM' */
    uint32_t crc;                        /* CRC32 */
} DeviceParameters;
/* #pragma pack(pop) */

#define FRAM_PARAM_ADDRESS 0x0000 /* 参数存储起始地址 */
#define CRC_SEED 0xFFFFFFFF       /* CRC初始值 */


/* 保持寄存器数据结构 */
struct ParameterMetadata {
    /* CPU3 参数菜单元数据；定义显示名、寄存器地址、范围、精度、权限、数据类型及密码文本入口。 */
    uint8_t* name; /* 参数在中文界面中的显示名称。 */
    int val; /* 当前参数的协议原始值；显示时再按小数位和单位元数据进行格式化。 */
    const int operanum; /* 参数对应的菜单操作码，用于查找和回写。 */
    const uint16_t startadd; /* 参数在 CPU2 共享保持寄存器中的起始地址。 */
    const uint8_t rgstcnt; /* 该参数连续占用的 16 位寄存器数量。 */
    const bool flag_checkvalue; /* 写入前是否执行最小值和最大值范围校验。 */
    const int valuemin; /* 启用范围校验时允许写入的最小值。 */
    const int valuemax; /* 启用范围校验时允许写入的最大值。 */
    uint8_t* unit; /* 只读单位字符串；无单位时为 NULL。 */
    const uint8_t point; /* 显示和编辑时使用的小数位数。 */
    const int offset; /* 显示值与协议原始值之间使用的固定偏移。 */
    const bool authority_write; /* 当前菜单权限下是否允许写入该参数。 */
    int data_type; /* 参数底层标量类型，决定读写时的位宽和浮点解释方式。 */
    int bits; /* 参数有效位宽，用于掩码和符号扩展。 */
    uint8_t *(*pword)(); /* 密码文本回调；受保护参数写入时动态取得当前密码字符串。 */
    uint8_t* name_English; /* 参数在英文界面中的显示名称。 */
};
extern struct ParameterMetadata param_meta[];

union utof
{
    /* 32 位无符号整数与单精度浮点数共享位模式的转换联合体。 */
    float f; /* 按 IEEE 754 单精度浮点数解释的位模式视图。 */
    u32 u; /* 按无符号整数字项解释的同一位模式视图。 */
};

union utod
{
    /* 两个 32 位字与双精度浮点数共享位模式的转换联合体。 */
    double d; /* 按 IEEE 754 双精度浮点数解释的位模式视图。 */
    u32 u[2]; /* 按无符号整数字项解释的同一位模式视图。 */
};
/* CPU3 参数元数据使用的标量表示类型；决定菜单读写时按整数、单精度或双精度解释目标地址。 */
typedef enum{
    /* 参数元数据的底层标量解释方式。 */
    TYPE_INT, /* 参数目标按有符号整数读取和写入。 */
    TYPE_FLOAT, /* 参数目标按 IEEE 754 单精度浮点数读取和写入。 */
    TYPE_DOUBLE, /* 参数目标按 IEEE 754 双精度浮点数读取和写入。 */
}DATA_TYPE;
typedef enum{
    /* 参数菜单使用的语言编号。 */
    LANG_CHINESE = 0, /* 参数菜单使用中文文本。 */
    LANG_ENGLISH, /* 参数菜单使用英文文本。 */
}LANG_NUM;
typedef enum { /* 数据源取自 */
    SOURCE_FROM_MEA,
    SOURCE_FROM_INPUT,
}SOURCE;

/* 位置记步来源：0 使用编码轮，1 使用 TMC5130 XACTUAL 电机步进。 */
/* 位置记步来源编码 0：使用编码轮计数换算传感器位置。 */
#define POSITION_COUNT_MODE_ENCODER 0u
/* 位置记步来源编码 1：使用 TMC5130 XACTUAL 电机步进换算位置。 */
#define POSITION_COUNT_MODE_MOTOR   1u

/* 流程是否允许自动切换位置源：0=不切换，1=自动切换。 */
/* 位置源自动切换配置值 0：测量流程保持当前来源，不执行自动切换。 */
#define POSITION_SOURCE_AUTO_SWITCH_DISABLE 0u
/* 位置源自动切换配置值 1：流程满足条件时允许在编码器与电机步进来源间切换。 */
#define POSITION_SOURCE_AUTO_SWITCH_ENABLE  1u
/* 罐底测量完成后是否修正编码器当前值：0=不修正，1=修正。 */
/* 探底完成后的编码器修正配置值 0：保留当前编码器累计值。 */
#define BOTTOM_ENCODER_CORRECTION_DISABLE 0u
/* 探底完成后的编码器修正配置值 1：使用罐底基准修正编码器当前位置。 */
#define BOTTOM_ENCODER_CORRECTION_ENABLE  1u
/* 电机运行电流配置，范围对应 TMC5130 IRUN。 */
/* TMC5130 运行电流 IRUN 的默认配置值 12；仅在参数缺省或归一化时使用。 */
#define MOTOR_CURRENT_DEFAULT       12u
/* TMC5130 运行电流 IRUN 的最小允许值 1；菜单索引和写入校验均以此为下界。 */
#define MOTOR_CURRENT_MIN           1u
/* TMC5130 运行电流 IRUN 的最大允许值 31；对应驱动器字段的 5 位上限。 */
#define MOTOR_CURRENT_MAX           31u

/* **************** 全局变量 *************************** */
extern volatile MeasurementResult g_measurement; /* 测量结果 */
extern volatile DeviceParameters g_deviceParams; /* 设备参数 */
extern const int param_metaAmount;
/**
 * @brief 根据操作号查找对应的参数元数据索引。
 *
 * @param operanum 菜单或参数操作号，对应 `ParameterMetadata.operanum`。
 * @return 找到时返回 `param_meta` 的零基索引；未找到时返回 -1。
 */
int getHoldValueNum(int operanum);
/**
 * @brief 初始化 CPU3 本地显示参数和首包同步前的测量运行态。
 *
 * @note 本函数在 CPU3 启动初始化阶段调用。它先应用本地显示参数，再将尚未从
 * CPU2 收到的数据设置为明确的无效值或未激活态，避免界面把零初始化误当成实测结果。
 */
void InputValueInit(void);
/**
 * @brief 将当前 CPU2 设备参数快照按业务分组完整打印到调试串口。
 *
 * @note 打印使用局部快照，避免输出过程中 CPU2 同步更新全局参数造成同一次
 * 打印前后字段不一致；本函数只读取参数，不修改运行态或持久化内容。
 */
void print_device_params(void); /* 打印设备参数 */
#endif

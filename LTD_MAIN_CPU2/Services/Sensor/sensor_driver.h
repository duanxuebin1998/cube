/*
 * 模块职责：定义传感器服务层与DSM、V3、DM4驱动之间的最小操作表和能力位。
 * 业务边界：上层按能力调用统一操作，不再重复判断具体传感器类型或协议版本。
 * 兼容约束：操作表只收口分派，不改变原协议函数返回码、等待时序和错误恢复规则。
 */
#ifndef SENSOR_DRIVER_H_
/* 本头文件的包含保护标记。 */
#define SENSOR_DRIVER_H_

#include <stdint.h>

/*
 * 驱动类别表示当前唯一存在的物理传感器，不表示DM4当前使用V4还是Safe协议。
 * 上层业务只能通过SensorRuntime取得当前驱动，不能自行按持久化数值拼接协议分支。
 */
typedef enum {
    SENSOR_DRIVER_NONE = 0, /* 没有识别到可用物理传感器。 */
    SENSOR_DRIVER_DSM, /* DSM一代文本协议物理传感器。 */
    SENSOR_DRIVER_MULTIPARAM_V3, /* 多参数V3固定帧物理传感器。 */
    SENSOR_DRIVER_DM4 /* DM4物理传感器，运行时可处于V4或Safe协议状态。 */
} SensorDriverKind;

/*
 * 能力位描述业务可用功能。静态能力来自驱动表，Safe等运行时能力由get_capabilities补充。
 * 调用可选操作前必须先检查对应能力，避免把“不支持”误记为传感器通信故障。
 */
typedef enum {
    SENSOR_CAP_NONE = 0U, /* 驱动不提供任何业务能力。 */
    SENSOR_CAP_DENSITY = (1UL << 0), /* 支持密度频率、密度和温度读取。 */
    SENSOR_CAP_LEVEL = (1UL << 1), /* 支持液位模式及液位频率读取。 */
    SENSOR_CAP_WATER = (1UL << 2), /* 支持水位电容测量。 */
    SENSOR_CAP_GYRO = (1UL << 3), /* 支持双轴姿态角读取。 */
    SENSOR_CAP_MAGNETIC_ZERO = (1UL << 4), /* 支持零点霍尔功能和电压读取。 */
    SENSOR_CAP_ACTIVE_REPORT = (1UL << 5), /* 支持周期主动上报数据流。 */
    SENSOR_CAP_SAFE_SESSION = (1UL << 6), /* 支持DM4 Safe会话协议。 */
    SENSOR_CAP_SUPPLY_VOLTAGE = (1UL << 7), /* 支持读取传感器供电电压。 */
    SENSOR_CAP_DENSITY_ANALYSIS = (1UL << 8) /* 支持密度分析扩展参数读取。 */
} SensorCapability;

/*
 * 驱动操作表只完成“统一业务动作到具体协议函数”的映射。
 * 协议函数保留原始错误码；电机停稳、模式稳定等待、无线诊断和整机修正由SensorService负责。
 * 可选回调允许为NULL，但对应能力位必须同时关闭，调用方不得绕过能力检查直接解引用。
 */
typedef struct {
    SensorDriverKind kind;             /* 当前物理驱动类别，用于诊断显示和少量协议分支。 */

    uint32_t capabilities;             /* 不随会话变化的静态能力位。 */
    uint32_t (*get_capabilities)(void); /* 可选的运行时能力补充入口。 */

    /* 模式与核心测量回调：返回协议原始结果码，不在驱动层统一重试。 */
    uint32_t (*enable_density_mode)(void); /* 请求驱动进入密度测量模式。 */
    uint32_t (*enable_level_mode)(void); /* 请求驱动进入液位测量模式。 */
    /* 读取同一次密度测量的频率、密度和温度。 */
    uint32_t (*read_density)(float *frequency_hz, float *density_kg_m3, float *temperature_c);
    uint32_t (*read_level_frequency)(uint32_t *frequency_hz); /* 读取当前液位测量频率，单位Hz。 */

    /* 可选通道与功能开关：水位电容和零点霍尔在DM4密度模式下互斥。 */
    uint32_t (*read_water_capacitance)(float *capacitance_pf); /* 读取水位电容，单位pF。 */
    uint32_t (*read_gyro)(float *angle_x_deg, float *angle_y_deg); /* 读取X/Y双轴姿态角，单位度。 */
    uint32_t (*read_magnetic_zero_voltage)(float *voltage_v); /* 读取零点霍尔电压，单位V。 */

    /* 维护诊断扩展量：不属于所有传感器共有的主测量数据。 */
    uint32_t (*read_supply_voltage)(float *voltage_v); /* 读取传感器供电电压，单位V。 */
    /* 读取22.5度、45度扫频周期平方均值和动力黏度。 */
    uint32_t (*read_density_analysis)(float *period_22_5,
                                      float *period_45,
                                      float *dynamic_viscosity); /* 三项输出来自同一密度分析响应。 */
} SensorDriverOps;

/*
 * 函数用途：读取驱动的静态能力与运行时能力并集。
 * 调用场景：SensorService和维护诊断在调用可选操作前统一判断。
 * 关键约束：Safe能力只有会话激活并完成HELLO后才允许发布。
 */
uint32_t SensorDriver_GetCapabilities(const SensorDriverOps *driver);

/*
 * 函数用途：判断驱动是否完整声明指定能力位集合。
 * 调用场景：调用可选传感器操作之前。
 * 关键约束：要求全部请求位存在；空驱动返回不支持。
 */
uint8_t SensorDriver_HasCapability(const SensorDriverOps *driver, uint32_t capability);

#endif /* SENSOR_DRIVER_H_ */

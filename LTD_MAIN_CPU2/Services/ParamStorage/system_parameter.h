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
#define _SYSTEM_PARAMETER_H

#include <stdint.h>
#include <stdbool.h>
/*无效值*/
#define UNVALID_LEVEL 999999u // 液位无效值
#define UNVALID_CURRENT 3.5
#define LEVEL_DOWNLIMIT 100u               // 盲区液位值
#define UNVALID_TEMPERATURE_REALTIME 99999 // 实时温度无效值
#define UNVALID_TEMPERATURE_WIRELESS 9999  // 无线温度无效值
#define UNVALID_DENSITY 0                  // 密度无效值
#define LEVEL_DOWNLIMITWATER 0             // 盲区水位值
#define UNVALID_VCF 1                      // VCF无效值
#define UNVALID_TOV 0                      // 体积无效值
#define UNVALID_GSW 0                      // 质量无效值

#define MAX_MEASUREMENT_POINTS 100 // 密度分布测量最大点数

// 模式枚举
typedef enum {
	DSM_SENSOR = 12,   // 一体机传感器
	LTD_SENSOR = 13,   // LTD传感器
} SENSOR_TYPE;

#define TEMP_TO_RAW(t)  ((uint32_t)((t) * 100.0f + 20000.0f)) //温度存储到寄存器
#define DENSITY_TO_RAW(d) ((uint32_t)((d) * 10.0f))//密度存储到寄存器
#define RAW_TO_TEMP(raw)    (((int32_t)(raw) - 20000) / 100.0f)
#define RAW_TO_DENSITY(raw) ((raw) / 10.0f)
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
    NO_ERROR = 0,                    // 正常状态
    STATE_SWITCH = 1,                // 状态切换（非故障）

    /* ==================== 11 电机类故障 (0x000B0000 - 0x000BFFFF) ==================== */
    MOTOR_FAIL_SETTING = 0x000B0001,             // 电机设置失败
    MOTOR_UNKNOWN_FEEDBACK = 0x000B0002,         // 未知反馈
    MOTOR_RESET_FAIL = 0x000B0003,               // 复位失败
    MOTOR_DISABLED = 0x000B0004,                 // 电机被禁止
    MOTOR_ALARM_TRIGGERED = 0x000B0005,          // 电机报警
    MOTOR_STEP_ERROR = 0x000B0006,               // 步进数错误
    MOTOR_CHARGE_PUMP_UNDER_VOLTAGE = 0x000B0007,// 电荷泵欠压
    MOTOR_OVERTEMPERATURE = 0x000B0008,          // 电机过温
    MOTOR_RUN_TIMEOUT = 0x000B0009,              // 电机运行超时

    /* ==================== 12 编码器类故障 (0x000C0000 - 0x000CFFFF) ==================== */
    ENCODER_TIMEOUT = 0x000C0001,                // 编码器通信超时
    ENCODER_PARITY_ERROR = 0x000C0002,           // 校验失败
    ENCODER_LOST_STEP = 0x000C0003,              // 编码器丢步
    ENCODER_INVALID_DATA = 0x000C0004,           // 持续无效数据
    ENCODER_POWERON_FAIL = 0x000C0005,           // 上电初始化失败
    ENCODER_POWERON_CHANGE = 0x000C0006,         // 上电编码值变化
    ENCODER_CORDIC_OVERFLOW = 0x000C0007,        // CORDIC 溢出
    ENCODER_LINEARITY_WARNING = 0x000C0008,      // 线性度报警
    ENCODER_DIFF_EXCESS = 0x000C0009,            // 相邻编码差值过大
    ENCODER_OCF_INCOMPLETE = 0x000C000A,         // OCF 未完成

    /* ==================== 13 传感器类故障 (0x000D0000 - 0x000DFFFF) ==================== */
    SENSOR_BCC_ERROR = 0x000D0001,               // 数据校验错误
    SONIC_FREQ_ABNORMAL = 0x000D0002,            // 震动管频率异常
    SENSOR_COMM_TIMEOUT = 0x000D0003,            // 传感器通信超时
    DENSITY_INVALID = 0x000D0004,                // 密度值异常
    SENSOR_TEMPERATURE_ERROR = 0x000D0005,       // 温度异常
    SENSOR_VOLTAGE_ERROR = 0x000D0006,           // 电压异常
    SLIPRING_COMM_FAIL = 0x000D0007,             // 无线滑环通信失败
    SLIPRING_BCC_ERROR = 0x000D0008,             // 无线滑环校验错误
    SLIPRING_PACKET_LOSS = 0x000D0009,           // 数据包丢失
    SLIPRING_SIGNAL_WEAK = 0x000D000A,           // 信号强度不足

	DENSITY_UNSTABLE = 0x000D000C,                // 密度值不稳定
    /* ==================== 测量过程故障 (0x000F0000 - 0x000FFFFF) ==================== */
    MEASUREMENT_POSITION_ERROR = 0x000F0001,     // 位置测量错误
    MEASUREMENT_TIMEOUT = 0x000F0002,            // 测量超时
    MEASUREMENT_ZERO_OUT_OF_RANGE = 0x000F0003,  // 零点超限
    MEASUREMENT_ZERO_REPEAT_FAIL = 0x000F0005,   // 零点重复性差
    MEASUREMENT_HEIGHT_DEVIATION = 0x000F0006,   // 实高偏差过大
    MEASUREMENT_OILLEVEL_HIGH = 0x000F0007,      // 液位超过罐高
    MEASUREMENT_OILLEVEL_LOW = 0x000F0008,       // 下行未找到液位
    MEASUREMENT_OILLEVEL_NOTFOUND = 0x000F0009,  // 上行未找到液位
    MEASUREMENT_WEIGHT_DOWN_FAIL = 0x000F000A,   // 下行寻重失败
    MEASUREMENT_WEIGHT_UP_FAIL = 0x000F000B,     // 上行寻重失败
	MEASUREMENT_WATERLEVEL_LOW = 0x000F000C,       // 下行未找到水位
    MEASUREMENT_OVERSPEED = 0x000F000F,          // 液位变化过快

    /* ==================== 参数存储类故障 (0x00110000 - 0x0011FFFF) ==================== */
    PARAM_EEPROM_FAIL = 0x00110001,              // EEPROM 写入失败
    PARAM_UNINITIALIZED = 0x00110002,            // 参数未初始化
    PARAM_RANGE_ERROR = 0x00110003,              // 参数超限
    PARAM_ADDRESS_OVERFLOW = 0x00110004,         // 地址越界
    PARAM_CRC_ERROR = 0x00110005,                // 参数 CRC 错误
	PARAM_ERROR = 0x00110006,                // 程序内参数调用错误
    /* ==================== 称重类故障 (0x00120000 - 0x0012FFFF) ==================== */
    WEIGHT_OUT_OF_RANGE = 0x00120001,            // 称重超上限
    WEIGHT_UNDER_RANGE = 0x00120002,             // 称重超下限
    WEIGHT_COLLISION_DETECTED = 0x00120003,      // 检测到碰撞
    WEIGHT_DRIFT_ERROR = 0x00120004,             // 称重漂移异常
    WEIGHT_SENSOR_SATURATION = 0x00120005,       // 传感器饱和

    /* ==================== 其他错误 (0x00130000 - 0x0013FFFF) ==================== */
    OTHER_UNKNOWN_ERROR = 0x00130001,            // 未知故障
    OTHER_ADDRESS_READ_ERROR = 0x00130002,       // 地址读取错误
    OTHER_POWER_FLUCTUATION = 0x00130003,        // 电源波动异常
    OTHER_PERIPHERAL_CONFIG_ERROR = 0x00130004   // 外设配置错误

} ErrorCode;

/*测量命令*/
/*测量命令*/
typedef enum {

    /* ======================= 基础 ======================= */
    CMD_NONE                       = 0,    // 无命令

    /* ======================= 普通指令（测量类 1~99） ======================= */

    /* --- 基础测量动作 --- */
    CMD_BACK_ZERO                  = 1,    // 回零点
    CMD_FIND_OIL                   = 2,    // 寻找液位
    CMD_FIND_WATER                 = 3,    // 寻找水位
    CMD_FIND_BOTTOM                = 4,    // 寻找罐底

    CMD_MEASURE_SINGLE             = 5,    // 单点测量
    CMD_MONITOR_SINGLE             = 6,    // 单点监测
    CMD_SYNTHETIC                  = 7,    // 综合测量

    /* --- 跟随 / 运动控制类（新增，建议 8~9 占位） --- */
    CMD_FOLLOW_WATER               = 8,    // 水位跟随（新增）
    CMD_RUN_TO_POSITION            = 9,    // 电机运行到指定位置（新增）

    /* --- 密度分布测量（整个系列，与普通液位分布区分） --- */
    CMD_MEASURE_DISTRIBUTED        = 10,   // 普通分布测量
    CMD_GB_MEASURE_DISTRIBUTED     = 11,   // 国标分布测量

    CMD_MEASURE_DENSITY_METER      = 12,   // 密度每米测量
    CMD_MEASURE_DENSITY_RANGE      = 13,   // 区间密度测量
    CMD_WARTSILA_DENSITY_RANGE     = 14,   // 瓦西莱密度区间测量

    /* --- 参数读取类（新增，放在 15，避免占用你后续密度扩展） --- */
    CMD_READ_PART_PARAMS           = 15,   // 读取部件参数（新增）

    /* 普通指令预留 */
    CMD_RESERVED_CMD1              = 20,
    CMD_RESERVED_CMD2              = 21,
    CMD_RESERVED_CMD3              = 22,

    /* ======================= 调试/强制指令（100~199） ======================= */

    CMD_DEBUG_MODE                 = 100,  // 进入调试模式

    CMD_CALIBRATE_ZERO             = 101,  // 标定零点
    CMD_CALIBRATE_OIL              = 102,  // 标定液位
    CMD_CORRECT_OIL                = 103,  // 修正液位

    CMD_MOVE_UP                    = 104,  // 上行
    CMD_MOVE_DOWN                  = 105,  // 下行

    CMD_SET_EMPTY_WEIGHT           = 106,  // 设置空载称重
    CMD_SET_FULL_WEIGHT            = 107,  // 设置满载称重
    CMD_RESTORE_FACTORY            = 108,  // 恢复出厂设置
    CMD_MAINTENANCE_MODE           = 109,  // 维护模式

    /* --- 强制运动 / 强制位置类（新增） --- */
    CMD_FORCE_MOVE_UP              = 113,  // 电机强制上行（新增）
    CMD_FORCE_MOVE_DOWN            = 114,  // 电机强制下行（新增）
    CMD_FORCE_LIFT_ZERO            = 115,  // 强制提零点（新增）

    /* --- 水位标定（新增，建议归类到标定类） --- */
    CMD_CALIBRATE_WATER            = 116,  // 水位标定（新增）

    /* 调试预留 */
    CMD_RESERVED_CMD4              = 110,
    CMD_RESERVED_CMD5              = 111,
    CMD_RESERVED_CMD6              = 112,

    /* ======================= 其他 ======================= */
    CMD_UNKNOWN                    = 255   // 未知命令

} CommandType;
typedef enum {
    CMD_NONE_DEF                       = 0,    // 无命令
    CMD_BACK_ZERO_DEF                  = 1,    // 回零点
    CMD_FIND_OIL_DEF                   = 2,    // 寻找液位
    CMD_MONITOR_SINGLE_DEF             = 3,    // 单点监测
    CMD_FOLLOW_WATER_DEF               = 4,    // 水位跟随（新增）
} DefaultCommandType;

/*设备状态*/
typedef enum {
    STATE_STANDBY = 0x0000,                   // 待机状态
    STATE_INIT = 0x0001,                      // 初始化状态
    STATE_BACKZEROING = 0x0002,               // 回零点中
    STATE_FINDZEROING = 0x0010,               // 标定零点中
    STATE_SINGLEPOINTING = 0x0011,            // 单点测量中
    STATE_RUNTOPOINTING = 0x0012,             // 运行到测量点中
    STATE_GB_SPREADPOINTING = 0x0013,         // 国标分布测量中
    STATE_SPREADPOINTING = 0x0014,            // 分布测量中
    STATE_CALIBRATIONOILING = 0x0015,         // 标定液位
    STATE_READPARAMETERING = 0x0016,          // 读取参数中
    STATE_RUNUPING = 0x0017,                  // 向上运行中
    STATE_RUNDOWNING = 0x0018,                // 向下运行中
    STATE_SETZEROCIRCLING = 0x0019,           // 设置零点编码值
    STATE_SETZEROANGLING = 0x001A,            // 设置零点编码值
    STATE_EFACTORYSETTING_RESTORING = 0x001B, // 恢复出厂设置中
    STATE_BACKUPING = 0x001C,                 // 备份配置文件中
    STATE_RESTORYING = 0x001D,                // 恢复配置文件中
    STATE_FINDOIL = 0x001E,                   // 寻找液位中
    STATE_FINDWATER = 0x001F,                 // 寻找水位中
    STATE_FINDBOTTOM = 0x0020,                // 寻找罐底中
    STATE_FORCEZERO = 0x0021,                 // 设置电机零点中
    STATE_ONTANKOPRATIONING = 0x0022,         // 罐上仪表操作中
    STATE_SYNTHETICING = 0x0023,              // 综合指令中

    /* ===================== LTD / 新增测量中状态（顺延） ===================== */
    STATE_FOLLOW_WATERING = 0x0024,            // 水位跟随寻找中
    STATE_METER_DENSITY = 0x0025,              // 密度每米测量中
    STATE_INTERVAL_DENSITY = 0x0026,           // 液位区间测量中
    STATE_GET_FULLWEIGHT = 0x0027,             // 获取满载称重中
    STATE_GET_EMPTYWEIGHT = 0x0028,            // 获取空载称重中
    STATE_MAINTENANCEMODE = 0x0029,            // 维护模式中
    STATE_WARTSILA_DENSITY_START = 0x002A,     // 瓦西莱密度梯度测量开始
    STATE_WARTSILA_DENSITY_MEASURING = 0x002B, // 瓦西莱密度梯度测量中
    STATE_RUN_TO_POSITIONING = 0x002C,         // 运行到指定位置中
    STATE_FORCE_RUNUPING = 0x002D,             // 电机强制上行中
    STATE_FORCE_RUNDOWNING = 0x002E,           // 电机强制下行中
    STATE_FORCE_LIFT_ZEROING = 0x002F,         // 强制提零点中
    STATE_CALIBRATE_WATERING = 0x0030,         // 水位标定中

    /* ===================== 完成态（0x80xx） ===================== */
    STATE_FINDZEROOVER = 0x8010,               // 标定零点完成
    STATE_SINGLEPOINTOVER = 0x8011,            // 单点测量完成
    STATE_SPTESTING = 0x8012,                  // 正在单点检测
    STATE_GB_SPREADPOINTOVER = 0x8013,         // 国标分布测量完成
    STATE_SPREADPOINTOVER = 0x8014,            // 分布测量完成
    STATE_FINDOILOVER = 0x8015,                // 标定液位完成
    STATE_READPARAMETEROVER = 0x8016,          // 读取参数完成
    STATE_RUNUPOVER = 0x8017,                  // 向上运行完成
    STATE_RUNDOWNOVER = 0x8018,                // 向下运行完成
    STATE_SETZEROCIRCLOVER = 0x8019,           // 设置零点编码值完成
    STATE_SETZEROANGLOVER = 0x801A,            // 设置零点编码值完成
    STATE_EFACTORYSETTING_RESTOROVER = 0x801B, // 恢复出厂设置完成
    STATE_BACKUPOVER = 0x801C,                 // 备份配置文件完成
    STATE_RESTORYOVER = 0x801D,                // 恢复配置文件完成
    STATE_FLOWOIL = 0x801E,                    // 液位跟随中（旧逻辑）
    STATE_FINDWATER_OVER = 0x801F,             // 寻找水位完成
    STATE_FINDBOTTOM_OVER = 0x8020,            // 寻找罐底完成
    STATE_FORCEZERO_OVER = 0x8021,             // 设置电机零点完成
    STATE_ONTANKOPRATIONCOMPLATE = 0x8022,     // 罐上仪表操作完成
    STATE_SYNTHETICING_OVER = 0x8023,          // 综合指令完成

    /* ===================== LTD / 新增完成态 ===================== */
    STATE_FOLLOW_WATER_OVER = 0x8024,           // 水位跟随寻找完成
    STATE_COM_METER_DENSITY_OVER = 0x8025,      // 密度每米测量完成
    STATE_INTERVAL_DENSITY_OVER = 0x8026,       // 液位区间测量完成
    STATE_GET_FULLWEIGHT_OVER = 0x8027,         // 获取满载称重完成
    STATE_GET_EMPTYWEIGHT_OVER = 0x8028,        // 获取空载称重完成
    STATE_WARTSILA_DENSITY_OVER = 0x8029,       // 瓦西莱密度梯度测量完成
    STATE_RUN_TO_POSITION_OVER = 0x802C,        // 运行到指定位置完成
    STATE_FORCE_RUNUP_OVER = 0x802D,            // 强制上行完成
    STATE_FORCE_RUNDOWN_OVER = 0x802E,          // 强制下行完成
    STATE_FORCE_LIFT_ZERO_OVER = 0x802F,        // 强制提零点完成
    STATE_CALIBRATE_WATER_OVER = 0x8030,        // 水位标定完成

    STATE_ERROR = 0xFFFF                        // 故障
} DeviceState;


/* 设备状态结构体 */
typedef struct {
	/*---- 设备核心状态 ----*/
	uint32_t work_mode;          // 工作模式（0:工作模式 1.调试模式 78.解锁模式）
	DeviceState device_state;    // 设备运行状态
	uint32_t error_code;         // 当前错误码（0表示无错误）
	CommandType current_command; // 当前指令

	/*---- 标志位 ----*/
	uint32_t zero_point_status; // 零点状态（0-正常 1-需要回零）
} DeviceStatus;

// 单点密度数据
typedef struct {
	uint32_t temperature;          ///< 温度
	uint32_t density;              ///< 密度
	uint32_t temperature_position; ///< 温度密度位置
	uint32_t standard_density;     ///< 标准密度
	uint32_t vcf20;                ///< 体积修正系数 (VCF20)
	uint32_t weight_density;       ///< 计重密度
} DensityMeasurement;
/**
 * @brief 密度分布数据
 */
typedef struct {
	uint32_t average_temperature;                                   ///< 温度
	uint32_t average_density;                                       ///< 密度
	uint32_t average_standard_density;                              ///< 标准密度
	uint32_t average_vcf20;                                         ///< 体积修正系数 (VCF20)
	uint32_t average_weight_density;                                ///< 计重密度
	uint32_t measurement_points;                                    ///< 实际测量点数
	uint32_t Density_oil_level;                                     // 密度分布测量时的液位值
	DensityMeasurement single_density_data[MAX_MEASUREMENT_POINTS]; // 100个点的密度测量数据

} DensityDistribution;

/**
 * @brief  调试数据
 */
typedef struct {
    /* 位置相关 */
	int32_t current_encoder_value; ///< 当前编码值
	int32_t sensor_position;       ///< 传感器位置
	int32_t cable_length;          ///< 尺带长度;
	int32_t motor_step;            ///< 电机步进值
	int32_t motor_distance;        ///< 电机距离值(单位: 0.1mm)

    /* 频率与温度、电压 */
	uint32_t frequency;            ///< 当前频率
	uint32_t temperature;          ///< 温度 (单位: 0.01°C)
	uint32_t air_frequency;        ///< 空气中频率
	uint32_t current_amplitude;    ///< 当前幅值
	uint32_t water_level_voltage;  ///< 水位电压值/电容值

    /* 称重相关 */
    uint32_t current_weight;       ///< 当前称重值
    uint32_t weight_param;         ///< 称重参数

    /* 姿态角 */
    int32_t  angle_x;              ///< X 轴角度
    int32_t  angle_y;              ///< Y 轴角度

    /* 电机状态相关 */
    uint32_t motor_speed;          ///< 电机速度（0.01m/min）
    uint32_t  motor_state;          ///< 电机状态: 0 停止, 1 上行, 2 下行
} DebugData;


/**
 * @brief  实高测量数据
 */
typedef struct {
	uint32_t calibrated_liquid_level; ///< 标定液位时实高
	uint32_t current_real_height;     ///< 当前实高
} ActualHeightMeasurement;

/**
 * @brief  液位测量数据
 */
typedef struct {
	uint32_t oil_level;                          ///< 液位跟随液位值
	uint32_t air_frequency;		//空气中频率
	uint32_t oil_frequency;		//油中频率
	uint32_t follow_frequency;		//液位跟随频率
	uint32_t current_frequency;	//当前频率
} OilMeasurement;

/**
 * @brief  液位测量数据
 */
typedef struct {
	uint32_t water_level;                        ///< 测量水位的值
	float zero_capacitance;
	float oil_capacitance;
	float current_capacitance;
} WaterMeasurement;
/* 测量结果结构体，输入寄存器 */
typedef struct {
	DeviceStatus device_status;                  ///< 设备状态
	DebugData debug_data;                        ///< 调试数据
	OilMeasurement oil_measurement;                ///< 液位测量数据
	WaterMeasurement water_measurement;          ///< 水位测量数据
	ActualHeightMeasurement height_measurement;  ///< 实高测量数据
	DensityMeasurement single_point_measurement; ///< 单点测量数据
	DensityMeasurement single_point_monitoring;  ///< 单点监测数据
	DensityDistribution density_distribution;    ///< 密度分布测量数据

} MeasurementResult;

/* 设备参数结构体 */
#pragma pack(push, 1) // 确保结构体紧凑对齐（防止编译器填充）
typedef struct {
    // 指令
    CommandType command;                  // 当前指令

    // ===================== 基础参数 =====================
    uint32_t sensorType;                  // 传感器类型
    uint32_t sensorID;                    // 传感器编号
    uint32_t sensorSoftwareVersion;       // 传感器软件版本
    uint32_t softwareVersion;             // LTD 软件版本
    CommandType powerOnDefaultCommand;    // 上电默认指令
    uint32_t error_auto_back_zero;        // 错误自动回零标志(0/1)
    uint32_t error_stop_measurement;      // 错误停止测量标志(0/1)

    uint32_t reserved1;                   // 预留
    uint32_t reserved2;                   // 预留
    uint32_t reserved3;                   // 预留
    uint32_t reserved4;                   // 预留（新增）
    uint32_t reserved5;                   // 预留（新增）

    // ===================== 电机与编码器参数 =====================
    uint32_t encoder_wheel_circumference_mm; // 编码轮周长(0.001mm)
    uint32_t max_motor_speed;                 // 最大电机速度(0.01m/min)
    uint32_t first_loop_circumference_mm;     // 尺带首圈周长(0.1mm)
    uint32_t tape_thickness_mm;               // 尺带厚度(0.001mm)

    uint32_t reserved6;                   // 预留
    uint32_t reserved7;                   // 预留（新增）

    // ===================== 称重参数 =====================
    uint32_t empty_weight;                // 空载重量
    uint32_t empty_weight_upper_limit;    // 空载重量上限
    uint32_t empty_weight_lower_limit;    // 空载重量下限
    uint32_t full_weight;                 // 满载称重
    uint32_t full_weight_upper_limit;     // 满载重量上限
    uint32_t full_weight_lower_limit;     // 满载重量下限
    uint32_t weight_upper_limit_ratio;    // 称重变化量检测上限比例
    uint32_t weight_lower_limit_ratio;    // 称重变化量检测下限比例

    uint32_t reserved8;                   // 预留
    uint32_t reserved9;                   // 预留（新增）

    // ===================== 零点测量 =====================
    uint32_t zero_weight_threshold_ratio;     // 零点称重阈值比例
    uint32_t weight_ignore_zone;              // 零点下称重不检测区域(建议 0.1mm)
    uint32_t max_zero_deviation_distance;     // 零点最大偏差距离(建议 0.1mm)
    uint32_t findZeroDownDistance;            // 找零点完成后下行距离(0.1mm)

    uint32_t reserved10;                  // 预留
    uint32_t reserved11;                  // 预留（新增）

    // ===================== 液位测量 =====================
    uint32_t tankHeight;                     // 液位罐高(0.1mm)
    uint32_t liquid_sensor_distance_diff;    // 液位传感器距离差(0.1mm)
    uint32_t blindZone;                      // 液位盲区(0.1mm)
    uint32_t oilLevelThreshold;              // 液位跟随阈值
    uint32_t oilLevelHysteresisThreshold;    // 液位滞后阈值
    uint32_t liquidLevelMeasurementMethod;   // 液位测量方式 0 空气+液体频率/2 1：根据设置跟随频率跟随 2 根据设置密度跟随
    uint32_t oilLevelFrequency;              // 液位跟随频率
    uint32_t oilLevelDensity;                // 液位跟随密度
    uint32_t oilLevelHysteresisTime;         // oil level hysteresis time

    // ===================== 水位测量参数 =====================
    uint32_t water_tank_height;                 // 水位罐高(0.1mm)
    uint32_t water_level_mode;                  // 水位测量方式
    uint32_t waterBlindZone;                    // 水位盲区(0.1mm)
    uint32_t water_cap_threshold;               // 水位电容阈值（建议明确倍率，如 x1000）
    uint32_t water_cap_hysteresis;              // 水位电容滞后阈值（建议明确倍率，如 x1000）
    uint32_t maxDownDistance;                   // 水位/罐底测量最大下行距离(0.1mm)
    uint32_t zero_cap;                          //零点电容值
    uint32_t water_stable_threshold;            //水位稳定阈值
    uint32_t waterLevelCorrection;              // water level correction

    // ===================== 罐高/罐底测量 =====================
    uint32_t bottom_detect_mode;          // 罐底测量模式
    uint32_t bottom_angle_threshold;      // 罐底角度阈值（务必明确单位/倍率）
    uint32_t bottom_weight_threshold;     // 罐底称重阈值
    uint32_t refreshTankHeightFlag;       // 是否更新液位罐高
    uint32_t maxTankHeightDeviation;      // 罐高最大变化范围
    uint32_t initialTankHeight;           // 初始实高
    uint32_t currentTankHeight;           // 当前实高

    uint32_t reserved16;                 // 预留
    uint32_t reserved17;                 // 预留（新增）

    // ===================== 密度和温度修正参数 =====================
    uint32_t densityCorrection;           // 密度修正值、磁通量D
    uint32_t temperatureCorrection;       // 温度修正值、磁通量T

    uint32_t reserved18;                 // 预留
    uint32_t reserved19;                 // 预留（新增）

    // ===================== 分布/区间测量参数 =====================
    uint32_t requireBottomMeasurement;         // 是否需要测量罐底
    uint32_t requireWaterMeasurement;          // 是否需要测量水位
    uint32_t requireSinglePointDensity;        // 是否需要测量单点密度
    uint32_t spreadMeasurementOrder;           // 分布测量顺序
    uint32_t spreadMeasurementMode;            // 分布测量模式
    uint32_t spreadMeasurementCount;           // 分布测量数量
    uint32_t spreadMeasurementDistance;        // 分布测量间距
    uint32_t spreadTopLimit;                   // 分布测量上限（距液面，0.1mm）
    uint32_t spreadBottomLimit;                // 分布测量下限（距罐底，0.1mm）
    uint32_t spreadPointHoverTime;             // 第一测量点悬停时间
    uint32_t intervalMeasurementTopLimit;      // 区间测量上限（距液面，0.1mm）
    uint32_t intervalMeasurementBottomLimit;   // 区间测量下限（距罐底，0.1mm）

    uint32_t reserved20;                 // 预留
    uint32_t reserved21;                 // 预留（新增）

    // ===================== Wartsila 密度区间测量参数 =====================
    uint32_t wartsila_upper_density_limit;        // 上限
    uint32_t wartsila_lower_density_limit;        // 下限
    uint32_t wartsila_density_interval;           // 步进
    uint32_t wartsila_max_height_above_surface;   // 最高测点距液面距离(0.1mm 或按定义)

    uint32_t reserved22;                 // 预留
    uint32_t reserved23;                 // 预留（新增）

    // ===================== 继电器报警输出 =====================
    uint32_t AlarmHighDO;                // 高液位报警输出
    uint32_t AlarmLowDO;                 // 低液位报警输出
    uint32_t ThirdStateThreshold;        // 第三状态阈值

    uint32_t reserved24;                 // 预留
    uint32_t reserved25;                 // 预留（新增）

    // ===================== 4-20mA 输出 =====================
    uint32_t CurrentRangeStart_mA;       // 电流量程起始值
    uint32_t CurrentRangeEnd_mA;         // 电流量程结束值
    uint32_t AlarmHighAO;                // 高液位报警输出
    uint32_t AlarmLowAO;                 // 低液位报警输出
    uint32_t InitialCurrent_mA;          // 初始化电流值
    uint32_t AOHighCurrent_mA;           // AO高报电流值
    uint32_t AOLowCurrent_mA;            // AO低报电流值
    uint32_t FaultCurrent_mA;            // 故障模式电流值
    uint32_t DebugCurrent_mA;            // 调试模式电流值

    uint32_t reserved26;                 // 预留
    uint32_t reserved27;                 // 预留（新增）

    // ===================== 指令参数 =====================
    uint32_t calibrateOilLevel;              // 标定液位值
    uint32_t calibrateWaterLevel;            // 水位标定值
    uint32_t singlePointMeasurementPosition; // 单点测量位置
    uint32_t singlePointMonitoringPosition;  // 单点监测位置
    uint32_t densityDistributionOilLevel;    // 电机指令的运行位置
    uint32_t motorCommandDistance;           // 电机指令的运行距离

    uint32_t reserved28;                 // 预留
    uint32_t reserved29;                 // 预留（新增）

    // ===================== Tape compensation =====================
    uint32_t lastOilCorrectionLevel;     // level at last oil correction
    uint32_t tankGasPhaseTemperature;    // tank gas phase temperature
    uint32_t tapeExpansionCoefficient;   // tape expansion coefficient
    uint32_t tapeCalibrationTemperature; // tape calibration temperature

    uint32_t reserved30;                 // reserved
    uint32_t reserved31;                 // reserved
    uint32_t reserved32;                 // reserved
    uint32_t reserved33;                 // reserved

    // ===================== 元信息与校验 =====================
    uint32_t param_version;              // 参数结构版本号
    uint32_t struct_size;                // sizeof(DeviceParameters)
    uint32_t magic;                      // 0x4C54444D = 'LTDM'
    uint32_t crc;                        // CRC32
} DeviceParameters;
#pragma pack(pop)

#define FRAM_PARAM_A_ADDRESS 0x0000u
#define FRAM_PARAM_SLOT_SIZE 0x0800u
#define FRAM_PARAM_B_ADDRESS (FRAM_PARAM_A_ADDRESS + FRAM_PARAM_SLOT_SIZE)
#define FRAM_PARAM_ADDRESS FRAM_PARAM_A_ADDRESS // 兼容旧代码
#define CRC_SEED 0xFFFFFFFF       // CRC初始值

/***************** 全局变量 ****************************/

extern volatile MeasurementResult g_measurement; // 测量结果
extern volatile DeviceParameters g_deviceParams; // 设备参数

void save_device_params(void); // 存储设备参数
int load_device_params(void); // 加载设备参数
void init_device_params(void); // 初始化设备参数
void RestoreFactoryParamsConfig(void); //恢复出厂默认参数配置
void print_device_params(void); // 打印设备参数
CommandType DefaultCmd_To_MeasureCmd(DefaultCommandType def_cmd);
/* 调试打印接口 */
void PrintDensity(const char *title, const DensityMeasurement *d); // 打印单个密度测点
void PrintMeasurementResult(const MeasurementResult *m); // 打印完整测量结果
#endif

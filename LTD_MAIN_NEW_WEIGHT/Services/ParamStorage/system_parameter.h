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

#include "fault_manager.h"
#include "encoder.h"
#include <stdint.h>
#include <stdbool.h>
//typedef enum
//{
//    false,
//    true
//} bool;

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


/*测量命令*/
typedef enum
{
    COMMAND_NONE = 0,                              // 无命令
    COMMAND_BACKZERO_START = 1,                    // 开始回零点
    COMMAND_BACKZERO_STOP = 2,                     // 停止回零点
    COMMAND_FINDZERO_START = 3,                    // 开始找零点
    COMMAND_FINDZERO_STOP = 4,                     // 停止找零点
    COMMAND_SINGLEPOINT_START = 5,                 // 开始单点测量
    COMMAND_SINGLEPOINT_STOP = 6,                  // 停止单点测量
    COMMAND_SPTEST_START = 7,                      // 开始单点监测
    COMMAND_SPTEST_STOP = 8,                       // 停止单点监测
    COMMAND_SPREADPOINTS_START = 9,                // 开始分布测量（带液位高度）
    COMMAND_SPREADPOINTS_STOP = 10,                // 停止分布测量（带液位高度）
    COMMAND_SPREADPOINTSAI_START = 11,             // 开始自动分布测量
    COMMAND_SPREADPOINTSAI_STOP = 12,              // 停止自动分布测量
    COMMAND_CALOIL_START = 13,                     // 开始标定液位
    COMMAND_CALOIL_STOP = 14,                      // 停止标定液位
    COMMAND_SETWORKPATTER = 15,                    // 设置工作模式
    COMMAND_READPARAMETER = 16,                    // 读取参数
    COMMAND_RUNUP = 17,                            // 上行
    COMMAND_RUNDOWN = 18,                          // 下行
    COMMAND_SETZEROCIRCLE = 19,                    // 设置零点圈数
    COMMAND_SETZEROANGLE = 20,                     // 设置零点圈数
    COMMAND_RESTOREFACTORYSETTING = 21,            // 恢复出厂设置
    COMMAND_BACKUPFILE = 22,                       // 备份配置文件
    COMMAND_RESTORYFILE = 23,                      // 恢复配置文件
    COMMAND_FINDOIL_START = 24,                    // 开始寻找液位
    COMMAND_FINDOIL_STOP = 25,                     // 停止寻找液位
    COMMAND_FINDWATER_START = 26,                  // 开始寻找水位
    COMMAND_FINDWATER_STOP = 27,                   // 停止寻找水位
    COMMAND_FINDBOTTOM_START = 28,                 // 开始寻找罐底
    COMMAND_FINDBOTTOM_STOP = 29,                  // 停止寻找罐底
    COMMAND_FORCEZERO_START = 30,                  // 开始设置零点
    COMMAND_FORCEZERO_STOP = 31,                   // 停止设置零点
    COMMAND_ONTANKOPERATION_BACKZERO = 32,         // 罐上操作回零点中
    COMMAND_ONTANKOPERATION_RUN_BACKZEROOK = 33,   // 罐上操作完成回零点
    COMMAND_ONTANKOPERATION_RUN_BEGAINUP = 34,     // 罐上操作上行
    COMMAND_ONTANKOPERATION_RUN_UPCOMPLATE = 35,   // 罐上操作上行完成
    COMMAND_ONTANKOPERATION_RUN_BEGAINDOWN = 36,   // 罐上操作下行
    COMMAND_ONTANKOPERATION_RUN_DOWNCOMPLATE = 37, // 罐上操作下行完成
    COMMAND_SYNTHETIC_START = 38,                  // 综合指令开始
    COMMAND_SYNTHETIC_STOP = 39,                   // 综合指令停止
    COMMAND_METERDENSITY_START = 40,               // 开始密度每米测量
    COMMAND_METERDENSITY_STOP = 41,                // 停止密度每米测量
    COMMAND_INTERVALDENSITY_START = 42,            // 开始液位区间测量
    COMMAND_INTERVALDENSITY_STOP = 43,             // 停止液位区间测量
    COMMAND_UNKNOW = 44                            // 未知指令
} CommandType;

/*设备状态*/
typedef enum
{
    STATE_STANDBY = 0x0000,                   // 待机状态
    STATE_INIT = 0x0001,                      // 初始化状态
    STATE_BACKZEROING = 0x0002,               // 回零点中
    STATE_FINDZEROING = 0x0010,               // 标定零点中
    STATE_SINGLEPOINTING = 0x0011,            // 单点测量中
    STATE_RUNTOPOINTING = 0x0012,             // 运行到测量点中
    STATE_SPREADPOINTING = 0x0013,            // 分布测量中
    STATE_AI_SPREADPOINTING = 0x0014,         // 无参分布测量中
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
    STATE_SYNTHETICING = 0x0023,              // 综合指令
    STATE_METER_DENSITY = 0x0024,             // 密度每米测量中
    STATE_INTERVAL_DENSITY = 0x0025,          // 液位区间测量中

    STATE_FINDZEROOVER = 0x8010,               // 标定零点完成
    STATE_SINGLEPOINTOVER = 0x8011,            // 单点测量完成
    STATE_SPTESTING = 0x8012,                  // 正在单点检测
    STATE_SPREADPOINTOVER = 0x8013,            // 分布测量完成
    STATE_AI_SPREADPOINTOVER = 0x8014,         // 无参分布测量完成
    STATE_FINDOILOVER = 0x8015,                // 标定液位完成
    STATE_READPARAMETEROVER = 0x8016,          // 读取参数完成
    STATE_RUNUPOVER = 0x8017,                  // 向上运行完成
    STATE_RUNDOWNOVER = 0x8018,                // 向下运行完成
    STATE_SETZEROCIRCLOVER = 0x8019,           // 设置零点编码值完成
    STATE_SETZEROANGLOVER = 0x801A,            // 设置零点编码值完成
    STATE_EFACTORYSETTING_RESTOROVER = 0x801B, // 恢复出厂设置完成
    STATE_BACKUPOVER = 0x801C,                 // 备份配置文件完成
    STATE_RESTORYOVER = 0x801D,                // 恢复配置文件完成
    STATE_FLOWOIL = 0x801E,                    // 液位跟随中
    STATE_FINDWATER_OVER = 0x801F,             // 寻找水位完成
    STATE_FINDBOTTOM_OVER = 0x8020,            // 寻找罐底完成
    STATE_FORCEZERO_OVER = 0x8021,             // 设置电机零点完成
    STATE_ONTANKOPRATIONCOMPLATE = 0x8022,     // 罐上仪表操作完成
    STATE_SYNTHETICING_OVER = 0x8023,          // 综合指令完成
    STATE_COM_METER_DENSITY_OVER = 0x8024,     // 密度每米测量完成
    STATE_INTERVAL_DENSITY_OVER = 0x8025,      // 液位区间测量完成

    STATE_ERROR = 0xFFFF // 故障
} DeviceState;

/* 设备状态结构体 */
typedef struct
{
    /*---- 设备核心状态 ----*/
    uint32_t work_mode;          // 工作模式（0:工作模式 1.调试模式 78.解锁模式）
    DeviceState device_state;    // 设备运行状态
    uint32_t error_code;         // 当前错误码（0表示无错误）
    CommandType current_command; // 当前指令
    CommandType next_command; 	 // 待执行指令

    /*---- 标志位 ----*/
    uint8_t zero_point_status; // 零点状态（0-正常 1-需要回零）
} DeviceStatus;

// 单点密度数据
typedef struct
{
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
typedef struct
{
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
 * @brief  标定零点数据
 */
typedef struct
{
    uint32_t zero_encoder_turns; ///< 零点编码器圈数
    uint32_t zero_encoder_value; ///< 零点编码器编码值
} ZeroCalibration;

/**
 * @brief  标定液位数据
 */
typedef struct
{
    uint32_t tank_height; ///< 罐高
} LiquidLevelCalibration;

/**
 * @brief  调试数据
 */
typedef struct
{
    int32_t current_encoder_value; ///< 当前编码值
    int32_t sensor_position;         ///< 传感器位置
    int32_t cable_length;           ///< 尺带长度;
    uint32_t frequency;            ///< 频率
    uint32_t temperature;          ///< 温度 (单位: 0.01°C)
    uint32_t air_frequency;        ///< 空气中频率
    uint32_t current_amplitude;    ///< 当前幅值
    uint32_t water_level_voltage;  ///< 水位电压值/电容值
} DebugData;

/**
 * @brief  实高测量数据
 */
typedef struct
{
    uint32_t calibrated_liquid_level; ///< 标定液位时实高
    uint32_t current_real_height;     ///< 当前实高
} ActualHeightMeasurement;

/* 测量结果结构体，输入寄存器 */
typedef struct
{
    DeviceStatus device_status;                  ///< 设备状态
    DensityMeasurement single_point_measurement; ///< 单点测量数据
    DensityMeasurement single_point_monitoring;  ///< 单点监测数据
    DensityDistribution density_distribution;    ///< 密度分布测量数据
    uint32_t oil_level;                          ///< 液位跟随液位值
    uint32_t water_level;                        ///< 测量水位的值
    uint32_t tank_bottom_measurement;            ///< 测量实高的值
    ZeroCalibration zero_calibration;            ///< 标定零点数据
    LiquidLevelCalibration liquid_calibration;   ///< 标定液位数据
    DebugData debug_data;                        ///< 调试数据
    ActualHeightMeasurement height_measurement;  ///< 实高测量数据
} MeasurementResult;

/* 设备参数结构体 */
#pragma pack(push, 1) // 确保结构体紧凑对齐（防止编译器填充）
typedef struct
{
    // 基础参数
	uint32_t tankHeight;                     // 罐高
	uint32_t blindZone;                      // 盲区
	uint32_t encoder_wheel_circumference_mm; // 编码轮周长（毫米）
    uint32_t sensorType;                     // 传感器类型
    uint32_t softwareVersion;                // 软件版本

    //称重参数

    uint32_t empty_weight;        // 空载重量
    // 密度和温度测量参数
    uint32_t densityCorrection;     // 密度修正值
    uint32_t temperatureCorrection; // 温度修正值

    uint32_t requireBottomMeasurement;  // 是否需要测量罐底
    uint32_t requireWaterMeasurement;   // 是否需要测量水位
    uint32_t requireSinglePointDensity; // 是否需要测量单点密度
    uint32_t spreadMeasurementOrder;    // 分布测量顺序
    uint32_t spreadMeasurementMode;     // 分布测量模式
    uint32_t spreadMeasurementCount;    // 分布测量数量
    uint32_t spreadMeasurementDistance; // 分布测量间距
    uint32_t spreadTopLimit;            // 分布测量上限（距液面）
    uint32_t spreadBottomLimit;         // 分布测量下限（距罐底）

    // 水位测量参数
    uint32_t waterLevelCorrection; // 水位修正值
    uint32_t maxDownDistance;      // 水位测量最大下行距离（盲区下行距离）

    // 实高测量
    uint32_t refreshTankHeightFlag;  // 是否修正罐高
    uint32_t maxTankHeightDeviation; // 实高测量最大偏差阈值
    uint32_t initialTankHeight;      // 初始实高
    uint32_t currentTankHeight;      // 当前实高

    // 液位测量
    uint32_t oilLevelThreshold;            // 液位跟随阈值
    uint32_t liquidLevelMeasurementMethod; // 液位测量方式
    uint32_t crc;                          // 新增CRC校验字段
} DeviceParameters;
#pragma pack(pop)

#define FRAM_PARAM_ADDRESS 0x0000 // 参数存储起始地址
#define CRC_SEED 0xFFFFFFFF       // CRC初始值

/***************** 全局变量 ****************************/

extern volatile FaultInfo g_faultInfo;           // 错误信息全局变量
extern volatile MeasurementResult g_measurement; // 测量结果
extern volatile DeviceParameters g_deviceParams; // 设备参数

void save_device_params(void); // 存储设备参数
int load_device_params(void); // 加载设备参数
void init_device_params(void); // 初始化设备参数
void RestoreFactoryParamsConfig(void);//恢复出厂默认参数配置
void print_device_params(void); // 打印设备参数
#endif

/*
 * measure_water_level.c - 水位测量模块
 *
 * 功能说明：
 *   该模块控制电机运动，通过水位/界面检测逻辑（传感器/算法）确定水位位置。
 *   测量过程分为两个阶段：粗略搜索和精确搜索。
 *
 * 创建日期: Dec 19, 2025
 * 作者: 1
 */

#include "measure_water_level.h"
#include <stdio.h>
#include <stdlib.h>
#include "measure.h"
#include "motor_ctrl.h"
#include "measure_zero.h"
#include "sensor_service.h"
#include "error_log.h"
#include "AoOutput/ao_output.h"
/* TODO: 这里替换为你的水位检测头文件 */
/* #include "water.h" / / 提供 check_water_status() */

/* -------------------- 可配置参数 -------------------- */
#define WATER_INIT_UP_MM                 (100.0f)   /* 初始上行避让 */
#define WATER_ROUGH_RETRY_MAX            (3) /* 水位粗找最大重试次数。 */
#define WATER_PRECISE_RETRY_MAX          (3) /* 水位精找最大重试次数。 */
#define WATER_ROUGH_CONFIRM_DELAY_MS     (3000)     /* 粗找停下后等待稳定 */
#define WATER_FAIL_RECOVER_UP_MM         (100.0f)   /* 粗找失败上行回退 */

#define WATER_V1_SLOWDOWN_TH             (1000)     /* 接近粗定位点：第一次降速阈值 */
#define WATER_V2_SLOWDOWN_TH             (100)      /* 更接近：第二次降速阈值 */
#define WATER_OVERSHOOT_TH               (-100)     /* 走过头保护阈值 */

/* 速度设置（沿用你 bottom 的写法） */
#define WATER_VEL_MID                    (16 * 32 * 40) /* 水位测量中速运行寄存器值。 */
#define WATER_VEL_SLOW                   (16 * 32 * 2) /* 水位测量低速运行寄存器值。 */

/* #define WATER_FOLLOW_OFFSET (5.0f) / * 阈值 = air + 50 * / */
/* #define WATER_FOLLOW_HYSTERESIS (0.3f) / * 滞回，防抖：可调 3~10 * / */
/* #define WATER_FOLLOW_SAMPLE_MS (500) / * 采样周期 * / */
/* #define WATER_FOLLOW_STEP_SMALL_MM (0.2f) / * 阈值附近小步 * / */
/* #define WATER_FOLLOW_STEP_MED_MM (5.0f) / * 中等偏差 * / */
/* #define WATER_FOLLOW_STEP_BIG_MM (20.0f) / * 大偏差/饱和时快速拉回 * / */
/* #define WATER_CAP_SAT_LIMIT (9999.0f) / * 认为“饱和/无穷大”的阈值，用于保护 * / */
/* #define WATER_FOLLOW_LOST_DIFF_BIG (2.5f) / * 认为偏差很大 * / */
/* #define WATER_FOLLOW_LOST_COUNT_MAX (20) / * 连续大偏差次数阈值：20次*500ms=10s * / */


/* #define WATER_FOLLOW_OFFSET (50.0f) / * 阈值 = air + 50 * / */
/* #define WATER_FOLLOW_HYSTERESIS (5.0f) / * 滞回，防抖：可调 3~10 * / */
#define WATER_FOLLOW_SAMPLE_MS         (500)     /* 采样周期 */
#define WATER_FOLLOW_STEP_SMALL_MM     (0.2f)    /* 阈值附近小步 */
#define WATER_FOLLOW_STEP_MED_MM       (3.0f)    /* 中等偏差 */
#define WATER_FOLLOW_STEP_BIG_MM       (10.0f)    /* 大偏差/饱和时快速拉回 */
#define WATER_CAP_SAT_LIMIT            (9999.0f) /* 认为“饱和/无穷大”的阈值，用于保护 */
#define WATER_FOLLOW_LOST_DIFF_BIG       (80.0f)   /* 认为偏差很大 */
#define WATER_FOLLOW_LOST_COUNT_MAX      (20)      /* 连续大偏差次数阈值：20次*500ms=10s */
#define WATER_FOLLOW_ENTER_TIMEOUT_MS    (180000u)  /* 长时间未进入稳定区，也认为已进入跟随态 */
#define WATER_FOLLOW_STABLE_MONITOR_COUNT (5u)  /* 连续稳定采样次数，达到后进入水位变化监测循环 */

#define WATER_SENSOR_TEST_PRE_UP_MM      (50.0f)   /* 找到水位后，先上行 5cm 再开始扫描 */
#define WATER_SENSOR_TEST_SCAN_STEP_MM   (0.1f)    /* 扫描步距 0.1mm */
#define WATER_SENSOR_TEST_SCAN_MM        (100.0f)  /* 单向扫描总长度 10cm */
#define WATER_SENSOR_TEST_POINT_COUNT    (1000u)   /* 10cm / 0.1mm = 1000 个采样点 */
/* -------------------- 全局变量 -------------------- */
/* 存储最终确定的水位位置（以 sensor_position 记录） */
int32_t water_value = -100000000; /* 初始值设为无效 */

/* -------------------- 函数原型 -------------------- */
typedef struct {
    /* 水位传感器测试轨迹中的位置与电容配对样本。 */
    int32_t position_01mm; /* 位置 0.1 mm 定点值，单位为 0.1 mm；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    float capacitance; /* 该测试位置采集到的水位探头电容值。 */
} WaterSensorTestPoint;

static WaterSensorTestPoint g_water_sensor_test_up_points[WATER_SENSOR_TEST_POINT_COUNT]; /* 水位传感器上行测试采集的位置/电容点阵。 */
static WaterSensorTestPoint g_water_sensor_test_down_points[WATER_SENSOR_TEST_POINT_COUNT]; /* 水位传感器下行测试采集的位置/电容点阵。 */

static int SearchWaterRough(void);
static int SearchWaterPrecise(void);
static uint32_t WaterSensorTestScanDirection(const char *phase_name,
                                             uint32_t dir,
                                             WaterSensorTestPoint *points,
                                             uint16_t point_count);
static void WaterSensorTestPrintResults(const char *phase_name,
                                        const WaterSensorTestPoint *points,
                                        uint16_t point_count,
                                        int32_t water_pos_01mm);

/**
 * @brief 把水位电容原始位模式还原为单精度浮点值。
 *
 * @param raw 传感器水位电容的 IEEE 754 Float32 原始 32 位位模式。
 * @return 计算后的业务数值。
 */
static inline float WaterCapRawToFloat(uint32_t raw)
{
    return raw / 1000.0f;
}

/**
 * @brief 用有符号计算当前水位，避免 water_tank_height 与负尺带长度混算成无符号大数。
 *
 * @return 返回由水位罐高和当前尺带长度计算的有符号水位，单位 0.1 mm；溢出时饱和到 INT32_MIN 或 INT32_MAX。
 */
static inline int32_t WaterLevelCalcFromCable(void)
{
    int64_t lvl = (int64_t)g_deviceParams.water_tank_height -
                  (int64_t)g_measurement.debug_data.cable_length;

    if (lvl > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (lvl < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }

    return (int32_t)lvl;
}

/**
 * @brief 用有符号计算目标水位对应的尺带长度，供水位跟随移动距离使用。
 *
 * @param lvl_target_01mm 目标水位对应的位置，单位 0.1 mm。
 * @return 返回目标水位对应的有符号尺带长度，单位 0.1 mm；溢出时饱和到 INT32_MIN 或 INT32_MAX。
 */
static inline int32_t WaterCableTargetFromLevel(int32_t lvl_target_01mm)
{
    int64_t cable_target = (int64_t)g_deviceParams.water_tank_height -
                           (int64_t)lvl_target_01mm;

    if (cable_target > (int64_t)INT32_MAX) {
        return INT32_MAX;
    }
    if (cable_target < (int64_t)INT32_MIN) {
        return INT32_MIN;
    }

    return (int32_t)cable_target;
}
/**
 * @brief 把水位结果限制到协议允许上报的有效区间。
 *
 * @param lvl 待限制、保存或打印的水位值，单位 0.1 mm。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
static inline uint32_t WaterLevelClampForReport(int32_t lvl)
{
    return (lvl > 0) ? (uint32_t)lvl : 0U;
}

/**
 * @brief 用当前尺带位置更新水位结果并打印定位信息。
 *
 * @param lvl 待限制、保存或打印的水位值，单位 0.1 mm。
 */
static inline void WaterLevelSetAndLog(int32_t lvl)
{
    uint32_t old_lvl = g_measurement.water_measurement.water_level;
    uint32_t report_lvl = WaterLevelClampForReport(lvl);

    g_measurement.water_measurement.water_level = report_lvl;
    water_value = lvl;
    AoOutput_PublishProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL, (int32_t)report_lvl, 1U);

    if (lvl < 0) {
        printf("水位更新\t计算水位为负：%ld(0.1mm)，按0上报\r\n", (long)lvl);
    }

    if (old_lvl != report_lvl)
    {
        printf("水位更新\t旧值=%.1fmm 新值=%.1fmm 缆长=%.1fmm\r\n",
               old_lvl / 10.0f,
               report_lvl / 10.0f,
               g_measurement.debug_data.cable_length / 10.0f);
    }
}

/**
 * @brief 根据当前尺带长度同步水位结果和传感器位置。
 */
static inline void WaterLevelSyncFromCable(void)
{
    int32_t lvl = WaterLevelCalcFromCable();
    WaterLevelSetAndLog(lvl);
}

/**
 * @brief 按指定方向逐点移动并记录位置、电容和水区判定。
 *
 * @param phase_name 标识本轮向上或向下扫描阶段的 NUL 结尾只读名称，用于逐点和结束诊断日志。
 * @param dir 电机或扫描方向编码；使用 MOTOR_DIRECTION_UP 或 MOTOR_DIRECTION_DOWN，决定本轮步进移动的正负方向。
 * @param points 本轮水传感器扫描测点输出数组；函数按移动顺序追加位置、电容和有效性结果。
 * @param point_count points 输出数组可容纳的最大测点数；扫描达到该数量后停止追加，防止越界。
 * @return NO_ERROR 表示指定方向的全部测试点已移动、驻留和记录；命令切换、位置越界、电容读取或电机运动失败时返回对应错误码。
 */
static uint32_t WaterSensorTestScanDirection(const char *phase_name,
                                             uint32_t dir,
                                             WaterSensorTestPoint *points,
                                             uint16_t point_count)
{
    uint32_t ret;
    uint16_t i;

    printf("水位传感器测试\t%s开始，共%u点\r\n", phase_name, (unsigned int)point_count);

    for (i = 0; i < point_count; ++i)
    {
        ret = MotorCtrl_MoveAndWait(WATER_SENSOR_TEST_SCAN_STEP_MM, dir, MotorCtrl_GetDefaultSpeedX100());
        CHECK_COMMAND_SWITCH(ret);
        CHECK_ERROR(ret);

        ret = SensorService_ReadWaterCapacitance(&points[i].capacitance);
        CHECK_COMMAND_SWITCH(ret);
        CHECK_ERROR(ret);

        points[i].position_01mm = g_measurement.debug_data.sensor_position;
        g_measurement.water_measurement.current_capacitance = points[i].capacitance;

        printf("水位传感器测试\t%s[%04u/%04u] 位置=%.1fmm 电容值=%.1f\r\n",
               phase_name,
               (unsigned int)(i + 1u),
               (unsigned int)point_count,
               points[i].position_01mm / 10.0f,
               points[i].capacitance);
    }

    return NO_ERROR;
}

/**
 * @brief 按扫描方向打印水位传感器剖面测试结果。
 *
 * @param phase_name 标识当前结果属于向上或向下扫描阶段的 NUL 结尾只读名称，用于汇总日志。
 * @param points 已经采集完成的水传感器扫描测点只读数组；函数按 count 逐点打印位置、电容和判定结果。
 * @param point_count points 数组中已经采集完成并允许打印的实际测点数量。
 * @param water_pos_01mm 水位传感器当前所在位置，单位 0.1 mm。
 */
static void WaterSensorTestPrintResults(const char *phase_name,
                                        const WaterSensorTestPoint *points,
                                        uint16_t point_count,
                                        int32_t water_pos_01mm)
{
    uint16_t i;

    printf("水位传感器测试\t%s结果开始\r\n", phase_name);
    for (i = 0; i < point_count; ++i)
    {
        printf("水位传感器测试\t%s[%04u] 位置=%.1fmm 变化=%+.1fmm 电容=%.1f\r\n",
               phase_name,
               (unsigned int)(i + 1u),
               points[i].position_01mm / 10.0f,
               (points[i].position_01mm - water_pos_01mm) / 10.0f,
               points[i].capacitance);
    }
    printf("水位传感器测试\t%s结果结束\r\n", phase_name);
}
/**
 * @brief 水位测量函数 - 执行完整的慢速找水流程
 *
 * 流程梳理：
 * 1. 先确保零点状态和 zero_capacitance 可用；必要时先回零点并读取空气区电容
 * 2. 若当前探头已经在水区，则先上行避让，保证后续是从空气区往下找水
 * 3. 进入粗找：持续下探，首次检测到 WATER 后急停，等待 3s 再复判
 * 4. 粗找成功后记录一个粗略水位，再进入精找：先上提 100mm，再降速下探做精定位
 * 5. 精找成功后，统一用 water_tank_height - cable_length 回写最终水位
 *
 * 说明：
 * - 本函数负责“找到一个可靠水位点”，不负责长期闭环跟随
 * - 状态切换（STATE_SWITCH）时不做本步骤内重试，由上层状态机决定后续动作
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */
uint32_t SearchWaterLevel(void)
{
    uint32_t ret;
    uint32_t last_rough_ret = MEASUREMENT_WATERLEVEL_LOW;
    uint8_t  try_times   = 0;
    uint8_t  water_state = NORMAL;

    AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL);
    fault_info_init();
    printf("水位测量\t开始\r\n");


    /* -------------------- 零点检查 -------------------- */
    if ((g_measurement.device_status.zero_point_status == 1)&&(g_deviceParams.error_auto_back_zero==1))
    {
        printf("水位测量\t设备需要回零点\r\n");
        ret = SearchZero();
        CHECK_ERROR(ret);
        printf("水位测量\t回零点完成\r\n");
    }
    if (g_measurement.water_measurement.zero_capacitance == 0)
    {
    	if(g_deviceParams.zero_cap==0){
            printf("水位测量\t获取零点电容值\r\n");
            ret = SearchZero();
            CHECK_ERROR(ret);
            printf("水位测量\t回零点完成\r\n");
    	}
    	else {
			g_measurement.water_measurement.zero_capacitance = (float)g_deviceParams.zero_cap/10.0;
		}
    }
    /* 打印零点电容值，水位电容阈值，水位滞后阈值 */
    printf("水位测量\t零点电容值：%lu\r\n", g_deviceParams.zero_cap);
    printf("水位测量\t水位电容阈值：%lu\r\n", g_deviceParams.water_cap_threshold);
    printf("水位测量\t水位寻找电容阈值：%lu\r\n", g_deviceParams.water_find_cap_threshold);
    /* -------------------- 初始位置调整（避让） -------------------- */
    if (g_measurement.debug_data.cable_length > 2000)
    {
        ret = MotorCtrl_MoveAndWait(WATER_INIT_UP_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);
        printf("水位测量\t上行完成\r\n");
    }

    printf("水位测量\t初始位置：%.1f", g_measurement.debug_data.sensor_position / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\r\n");

    ret = check_water_status(&water_state);
    CHECK_ERROR(ret);

    if (water_state != NORMAL)
    {
        /* 约定：sensor_position 单位为 0.1mm（从你打印 /10 推断） */
        const int32_t ZERO_NEAR_TH = 1000;   /* 100mm -> 1000(0.1mm) */
        const float   UP_STEP_MM   = 100.0f; /* 每次上行 100mm */

        printf("水位测量\t当前在水区，执行上行避让\r\n");

        while (1)
        {
            /* 距离零点 <= 100mm：认为已到零点附近，仍在水里 -> 报错退出 */
            if (g_measurement.debug_data.sensor_position <= ZERO_NEAR_TH)
            {
                printf("水位测量\t零点附近仍在水区，无法避让(位置=%.1fmm)", g_measurement.debug_data.sensor_position / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\r\n");

                RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
            }

            /* 距离零点 > 100mm：上行 100mm */
            ret = MotorCtrl_MoveAndWait(UP_STEP_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
            CHECK_ERROR(ret);
            printf("水位测量\t上行%.0fmm\r\n", (double)UP_STEP_MM);

            /* 检测是否已提出水面 */
            ret = check_water_status(&water_state);
            CHECK_ERROR(ret);

            if (water_state == NORMAL)
            {
                printf("水位测量\t已提出水面，继续测量\r\n");
                break;
            }

            /* 仍在水里：继续上行，直到提出水面或到零点附近触发报错 */
            CHECK_COMMAND_SWITCH(NO_ERROR);
        }
    }

    /* ************** 粗找阶段 - 带重试机制 ************** */
    try_times = 0;
    while (try_times < WATER_ROUGH_RETRY_MAX)
    {
        try_times++;

        fault_info_init();
        ret = SearchWaterRough();

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        CHECK_COMMAND_SWITCH(ret);

        if (ret != NO_ERROR)
        {
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_WATER_ROUGH,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           (uint32_t)WATER_ROUGH_RETRY_MAX,
                           ret);
            last_rough_ret = ret;
            HAL_Delay(1000);
            continue;
        }
        else
        {
            if (try_times > 1U)
            {
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                                 ERROR_LOG_OP_SEARCH_WATER_ROUGH,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)try_times,
                                 (uint32_t)WATER_ROUGH_RETRY_MAX);
            }
            break;
        }
    }

    if (ret != NO_ERROR)
    {
        RETURN_ERROR(last_rough_ret);
    }

    printf("水位测量\t粗找完成：水位：%ld\r\n", water_value);

    /* ************** 精找阶段 - 带重试 ************** */
    try_times = 0;
    while (try_times < WATER_PRECISE_RETRY_MAX)
    {
        try_times++;

        ret = SearchWaterPrecise();

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        if (ret == NO_ERROR)
        {
            if (try_times > 1U)
            {
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                                 ERROR_LOG_OP_SEARCH_WATER_PRECISE,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)try_times,
                                 (uint32_t)WATER_PRECISE_RETRY_MAX);
            }
            break;
        }
        else
        {
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_WATER_PRECISE,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           (uint32_t)WATER_PRECISE_RETRY_MAX,
                           ret);
            HAL_Delay(1000);
        }
    }

    if (ret != NO_ERROR)
    {
        CHECK_ERROR(ret);
    }

    /* ************** 最终记录 ************** */
    WaterLevelSyncFromCable();

    printf("水位测量\t水位：%ld mm\r\n", g_measurement.water_measurement.water_level);

    return NO_ERROR;
}

/**
 * @brief 先定位水位，再分别执行下行和上行电容剖面扫描并输出结果。
 * @return NO_ERROR 表示水位定位以及下行、上行两段电容扫描均完成；命令切换、传感器读取或电机运动失败时返回具体错误码。
 */
uint32_t WaterSensorCapacitanceProfileTest(void)
{
    uint32_t ret;
    int32_t water_pos_01mm;

    printf("水位传感器测试\t开始\r\n");

    ret = SearchWaterLevel();
    CHECK_COMMAND_SWITCH(ret);
    CHECK_ERROR(ret);

    water_pos_01mm = g_measurement.debug_data.sensor_position;
    printf("水位传感器测试\t找到水位 位置=%.1fmm 水位=%.1fmm\r\n",
           water_pos_01mm / 10.0f,
           g_measurement.water_measurement.water_level / 10.0f);

    ret = MotorCtrl_MoveAndWait(WATER_SENSOR_TEST_PRE_UP_MM,
                                             MOTOR_DIRECTION_UP,
                                             MotorCtrl_GetDefaultSpeedX100());
    CHECK_COMMAND_SWITCH(ret);
    CHECK_ERROR(ret);

    printf("水位传感器测试\t上行避让%.1fmm后开始扫描，当前位置=%.1fmm\r\n",
           (double)WATER_SENSOR_TEST_PRE_UP_MM,
           g_measurement.debug_data.sensor_position / 10.0f);

    ret = WaterSensorTestScanDirection("下行扫描",
                                       MOTOR_DIRECTION_DOWN,
                                       g_water_sensor_test_down_points,
                                       WATER_SENSOR_TEST_POINT_COUNT);
    CHECK_COMMAND_SWITCH(ret);
    CHECK_ERROR(ret);

    ret = WaterSensorTestScanDirection("上行扫描",
                                       MOTOR_DIRECTION_UP,
                                       g_water_sensor_test_up_points,
                                       WATER_SENSOR_TEST_POINT_COUNT);
    CHECK_COMMAND_SWITCH(ret);
    CHECK_ERROR(ret);

    printf("水位传感器测试\t扫描完成，开始打印结果\r\n");
    WaterSensorTestPrintResults("下行扫描",
                                g_water_sensor_test_down_points,
                                WATER_SENSOR_TEST_POINT_COUNT,
                                water_pos_01mm);
    WaterSensorTestPrintResults("上行扫描",
                                g_water_sensor_test_up_points,
                                WATER_SENSOR_TEST_POINT_COUNT,
                                water_pos_01mm);

    printf("水位传感器测试\t结束 当前位置=%.1fmm\r\n",
           g_measurement.debug_data.sensor_position / 10.0f);
    return NO_ERROR;
}

/**
 * @brief 持续下行直至传感器进入水区；停稳复核成功后记录水位，否则上提 100 mm 并返回低水位错误。
 * @return NO_ERROR 表示下行进入水区且停稳复核通过；未能稳定确认水区时上提 100 mm 并返回 MEASUREMENT_WATERLEVEL_LOW，命令切换、传感器或电机错误由检查入口原样返回。
 */
static int SearchWaterRough(void)
{
    uint32_t ret;
    uint8_t  water_state = NORMAL;

    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    /* 持续下探直到检测到 WATER */
    while (1)
    {
	ret = MotorCtrl_MoveDown(MotorCtrl_GetDefaultSpeedX100());  /* 启动电机向下运动 */
	CHECK_ERROR(ret); /* 检查上行是否成功 */

        ret = check_water_status(&water_state);
        CHECK_ERROR(ret);

        if (water_state != NORMAL) {
            break;
        }

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        printf("水位测量\t长距离寻找水位\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t");

    }

    ret = MotorCtrl_SlowStop();
    CHECK_ERROR(ret);

    HAL_Delay(WATER_ROUGH_CONFIRM_DELAY_MS);

    printf("水位测量\t确认粗找水位位置\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t");


    /* 停稳后再读一次，确认状态 */
    ret = check_water_status(&water_state);
    CHECK_ERROR(ret);

    if (water_state == WATER)
    {
        water_value = WaterLevelCalcFromCable();
        return NO_ERROR;
    }
    else
    {

        ret = MotorCtrl_MoveAndWait(WATER_FAIL_RECOVER_UP_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);

        printf("水位测量\t上行%.0fmm\r\n", (double)WATER_FAIL_RECOVER_UP_MM);
        return MEASUREMENT_WATERLEVEL_LOW;
    }
}

/**
 * @brief 精确搜索水位 - 使用变速策略精确定位
 *
 * @return NO_ERROR 表示变速精找已收敛并记录水位；命令切换、传感器数据无效、位置边界或电机运动错误由检查宏返回。
 */
static int SearchWaterPrecise(void)
{
    uint32_t ret;
    uint8_t  water_state = NORMAL;
    uint32_t speed_x100;

    if (g_measurement.debug_data.sensor_position > 2000)
    {
        ret = MotorCtrl_MoveAndWait(WATER_INIT_UP_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);
        printf("水位测量\t上行完成\r\n");
    }

    /* 粗找停稳后探头可能仍在水中，细找前先上行脱离水区，避免第一次采样直接结束。 */
    ret = check_water_status(&water_state);
    CHECK_ERROR(ret);
    if (water_state == WATER)
    {
        const int32_t zero_near_th = 1000;

        MotorCtrl_LostStepInit();
        printf("水位测量\t细找前仍在水中，先上行脱离水区\r\n");
        while (water_state == WATER)
        {
            if (g_measurement.debug_data.cable_length <= zero_near_th)
            {
                ret = MotorCtrl_SlowStop();
                CHECK_ERROR(ret);
                RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
            }

            ret = MotorCtrl_MoveUp(40U);
            CHECK_ERROR(ret);

            ret = check_water_status(&water_state);
            CHECK_ERROR(ret);

            ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
            CHECK_ERROR(ret);

            CHECK_COMMAND_SWITCH(NO_ERROR);
        }

        ret = MotorCtrl_SlowStop();
        CHECK_ERROR(ret);
        printf("水位测量\t已脱离水区，开始低速下行细找\r\n");
    }

    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    while (1)
    {
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        if ((WaterLevelCalcFromCable() - water_value) < WATER_V2_SLOWDOWN_TH)
        {
            speed_x100 = 4;
        }
        else if ((WaterLevelCalcFromCable() - water_value) < WATER_V1_SLOWDOWN_TH)
        {
            speed_x100 = 40;
        }

	ret = MotorCtrl_MoveDown(speed_x100);  /* 启动电机向下运动 */
	CHECK_ERROR(ret); /* 检查上行是否成功 */

        ret = check_water_status(&water_state);
        CHECK_ERROR(ret);

        if (water_state == WATER) {
            break;
        }

        /* 走过头保护 */
/* if (g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length - water_value < WATER_OVERSHOOT_TH) */
/* { */
/* printf("水位测量\t精确寻找水位未找到水位\r\n"); */
/* RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW); */
/* } */

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        printf("水位测量\t精确寻找水位\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t速度(0.01m/min)\t%lu\t", (unsigned long)g_measurement.debug_data.motor_speed);
    }

    ret = MotorCtrl_SlowStop();
    CHECK_ERROR(ret);

    water_value = WaterLevelCalcFromCable();
    return NO_ERROR;
}

/**
 * @brief 读取水位传感器零点电容并校验结果。
 * @return NO_ERROR 表示零点电容读取成功且数值有效；其他值为水位传感器通信、数据格式或零点电容有效性错误。
 */
uint32_t read_zero_capacitance(void)
{
    uint32_t ret;
    float    cap = 0.0f;

    ret = SensorService_ReadWaterCapacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;
    }
    g_deviceParams.zero_cap = 10*cap;
    g_measurement.water_measurement.zero_capacitance = cap;

    printf("零点电容 = %.1f\r\n", cap);
    /* 参数存储 */
    save_device_params();
    return NO_ERROR;
}


/**
 * @brief 根据当前电容和零点电容判断水位探头是否接触水层。
 * @param water_state 输出水位状态。
 * @return NO_ERROR 表示判断完成，其他值表示参数或传感器异常。
 */
uint32_t check_water_status(uint8_t *water_state)
{
    uint32_t ret;
    float    cap = 0.0f;
    float    zero;
    float    th;

    if (water_state == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    ret = SensorService_ReadWaterCapacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;
    }

    zero = g_measurement.water_measurement.zero_capacitance;
    th   = zero + WaterCapRawToFloat(g_deviceParams.water_cap_threshold);

    /* 判定 */
    if (cap > th) {
        *water_state = WATER;
    } else {
        *water_state = NORMAL;
    }

    /* 单行、完整判定信息 */
    printf("[水位检查] 电容=%.1f  零点：%.1f  阈值=%.1f  电容%s阈值  -> %s\r\n",
           cap,
           zero,
           th,
           (cap > th) ? ">" : "<=",
           (*water_state == WATER) ? "WATER" : "NORMAL");

    g_measurement.water_measurement.current_capacitance = cap;

    return NO_ERROR;
}

/**
 * @brief 停机后对齐到目标水位附近（通过“目标水位 -> 目标缆长 -> 计算delta -> 精确移动”）
 * @param[in] lvl_target_01mm   目标水位（0.1mm），例如 lvl_avg
 * @return NO_ERROR / 错误码
 */
static uint32_t AlignToWaterLevel_01mm(int32_t lvl_target_01mm)
{
    uint32_t ret;

    int32_t cable_now_01mm    = g_measurement.debug_data.cable_length; /* 0.1mm */
    int32_t cable_target_01mm = WaterCableTargetFromLevel(lvl_target_01mm); /* 0.1mm */
    int32_t delta_01mm        = cable_target_01mm - cable_now_01mm; /* 0.1mm */

    int32_t abs_delta_01mm = (delta_01mm >= 0) ? delta_01mm : -delta_01mm;

    printf("快速跟随\t对齐: 目标液位=%.1fmm 当前缆长=%.1fmm 目标缆长=%.1fmm 差值：%+.1fmm\r\n",
           lvl_target_01mm / 10.0f,
           cable_now_01mm / 10.0f,
           cable_target_01mm / 10.0f,
           delta_01mm / 10.0f);

    uint32_t dir = (delta_01mm >= 0) ? MOTOR_DIRECTION_DOWN : MOTOR_DIRECTION_UP;
    float move_mm = abs_delta_01mm / 10.0f;

    printf("快速跟随\t对齐: 方向：%s 移动=%.1fmm\r\n",
           (dir == MOTOR_DIRECTION_DOWN) ? "DOWN" : "UP",
           move_mm);

    ret = MotorCtrl_MoveAndWait(move_mm, dir, MotorCtrl_GetDefaultSpeedX100());
    if (ret != NO_ERROR) return ret;

    return NO_ERROR;
}

/**
 * @brief 快速找跟随点（基于“状态翻转 + 窗口判稳”）
 *
 * 流程梳理：
 * 1. 先读取当前是 WATER 还是 NORMAL
 * 2. 若当前在水里，则连续上行直到提出水面；若当前不在水里，则连续下行直到进入水里
 * 3. 每发生一次 WATER/NORMAL 翻转，就记录一次翻转水位
 * 4. 从第二次翻转开始，用最近两次翻转位置的平均值作为当前液面估计值
 * 5. 若连续 stable_win_ms 内该估计值波动不超过阈值，则认为“快速找点完成”，停机并对齐到目标水位附近
 *
 * 说明：
 * - 这个函数的职责是“给 FollowWaterLevelCore 找一个初始跟随点”
 * - 它本身不是长期跟随主循环；退出后由上层立即转入闭环跟随
 * - 当前上层快速模式传入 stable_win_ms=0，因此只要求形成可用跟随点，不在这里长期等待
 *
 * 依赖：
 *  - MotorCtrl_MoveUp()/MotorCtrl_MoveDown(): 每次调用推动继续运动（你现有粗找就是这样用的）
 *  - check_water_status(): 返回 WATER / NORMAL
 *  - MotorCtrl_CheckLostStepAutoTiming(): 丢步检测（可选但建议保留）
 *
 * @param stable_win_ms 稳定判定窗口时长（ms）
 * 退出条件：
 *   连续 stable_win_ms 内 water_level 波动（max-min）不超过阈值 -> 认为稳定找到水位，退出
 *
 * @return NO_ERROR 表示检测到水区状态翻转并通过稳定窗口确认；命令切换、超时、传感器通信或电机错误由过程检查返回。
 */
uint32_t FindWaterLevel_FastByStateFlip_StableExit(uint32_t stable_win_ms)
{
    uint32_t ret;
    uint8_t  water_state = NORMAL;

    /* ===== 稳定判定参数 ===== */
    const uint32_t stable_window_ms = stable_win_ms;

    /* ===== 零点保护（沿用你之前逻辑，可按需删） ===== */
    const int32_t ZERO_NEAR_TH = 1000; /* 100mm -> 1000(0.1mm) */

    /* 稳定窗口统计量 */
    uint32_t win_start_tick = HAL_GetTick();
    int32_t  min_level =  2147483647;
    int32_t  max_level = -2147483647;
    /* 翻转缓存 */
    static uint8_t have_last_flip = 0;
    static int32_t last_flip_lvl  = 0;
    AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL);
    have_last_flip = 0; /* 每次调用都重置，确保独立测量 */
    /* -------------------- 零点检查 -------------------- */
    if (g_measurement.water_measurement.zero_capacitance == 0)
    {
    	if(g_deviceParams.zero_cap==0){
            printf("水位测量\t获取零点电容值\r\n");
            ret = SearchZero();
            CHECK_ERROR(ret);
            printf("水位测量\t回零点完成\r\n");
    	}
    	else {
			g_measurement.water_measurement.zero_capacitance = (float)g_deviceParams.zero_cap/10.0;
		}
    }
    /* 打印零点电容值，水位电容阈值，水位滞后阈值 */
    printf("水位测量\t零点电容值：%lu\r\n", g_deviceParams.zero_cap);
    printf("水位测量\t水位电容阈值：%lu\r\n", g_deviceParams.water_cap_threshold);
    printf("水位测量\t水位寻找电容阈值：%lu\r\n", g_deviceParams.water_find_cap_threshold);
    printf("快速跟随\t开始(稳定判定：%lu.%03lus内波动<=%.1fmm退出)\r\n",
           (unsigned long)(stable_window_ms / 1000u),
           (unsigned long)(stable_window_ms % 1000u),
           g_deviceParams.water_stable_threshold / 10.0f);

    /* 初始状态 */
    ret = check_water_status(&water_state);
    CHECK_ERROR(ret);

    while (1)
    {
        /* ===================== 段 A：在水里 -> 连续上行直到出水(NORMAL) ===================== */
        if (water_state == WATER)
        {
            printf("快速跟随\t当前：水区 -> 连续上行直到空气区\r\n");
            MotorCtrl_LostStepInit();

            while (1)
            {
                if (g_measurement.debug_data.cable_length <= ZERO_NEAR_TH)
                {
                    printf("快速跟随\t零点附近仍在水区，停止(位置=%.1fmm)\r\n",
                           g_measurement.debug_data.sensor_position / 10.0f);
                    RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
                }

                ret = MotorCtrl_MoveUp(MotorCtrl_GetDefaultSpeedX100());
                CHECK_ERROR(ret);

                ret = check_water_status(&water_state);
                CHECK_ERROR(ret);

                if (water_state == NORMAL) {
                    break;
                }

                ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
                CHECK_ERROR(ret);

                CHECK_COMMAND_SWITCH(NO_ERROR);
            }
        }
        /* ===================== 段 B：不在水里 -> 连续下行直到进水(WATER) ===================== */
        else
        {
            printf("快速跟随\t当前：空气区 -> 连续下行直到水区\r\n");
            MotorCtrl_LostStepInit();

            while (1)
            {
                ret = MotorCtrl_MoveDown(MotorCtrl_GetDefaultSpeedX100());
                CHECK_ERROR(ret);

                ret = check_water_status(&water_state);
                CHECK_ERROR(ret);

                if (water_state == WATER) {
                    break;
                }

                ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
                CHECK_ERROR(ret);

                CHECK_COMMAND_SWITCH(NO_ERROR);
            }
        }

        /* ===== 翻转结束：先计算“本次翻转水位样本” ===== */
        int32_t lvl_flip = WaterLevelCalcFromCable(); /* 0.1mm */


        /* 第一次翻转：只缓存，不更新水位/不判稳 */
        if (!have_last_flip)
        {
            have_last_flip = 1;
            last_flip_lvl  = lvl_flip;
            win_start_tick = HAL_GetTick(); /* 开始稳定判定计时 */
            printf("快速跟随\t第1次翻转 液位=%.1fmm -> 缓存(等待第二次翻转后才开始更新/判稳)\r\n",
                   lvl_flip / 10.0f);

            CHECK_COMMAND_SWITCH(NO_ERROR);
            continue; /* 回到外层 while(1)，继续下一次翻转 */
        }

        /* 第二次及以后：用最近两次翻转的平均值作为“当前水位” */
        int32_t lvl_avg;
        {
            /* 无偏平均：避免奇数和截断偏差 */
            int32_t a = last_flip_lvl;
            int32_t b = lvl_flip;
            lvl_avg = (a / 2) + (b / 2) + ((a & 1) && (b & 1));  /* 两个都是奇数时补 1 */
        }

        /* 更新“对外水位值”（只在两次翻转后才更新） */
        WaterLevelSetAndLog(lvl_avg);

        printf("快速跟随\t翻转液位1=%.1f  液位2=%.1f  平均=%.1fmm\r\n",
               last_flip_lvl / 10.0f,
               lvl_flip      / 10.0f,
               lvl_avg       / 10.0f);

        /* 更新缓存：为下一次平均做准备 */
        last_flip_lvl = lvl_flip;

        /* ===== 稳定判定窗口统计（从第二次翻转开始） ===== */
        {
            int32_t lvl = lvl_avg; /* 0.1mm */

            if (lvl < min_level) min_level = lvl;
            if (lvl > max_level) max_level = lvl;

            if ((max_level - min_level) > g_deviceParams.water_stable_threshold)
            {
                win_start_tick = HAL_GetTick();
                min_level = lvl;
                max_level = lvl;
                printf("快速跟随\t波动超限 -> 重置稳定窗口(最小=最大=%.1fmm)\r\n", lvl / 10.0f);
                /* 如果设备状态不是水位跟随状态，设置水位跟随状态 */
				if (g_measurement.device_status.device_state != STATE_FOLLOW_WATERING) {
					g_measurement.device_status.device_state = STATE_FOLLOW_WATERING;
				}
            }
            else
            {
                uint32_t elapsed = HAL_GetTick() - win_start_tick;

                printf("快速跟随\t稳定窗口：%lus  波动=%.1fmm(阈值%.1fmm)\r\n",
                       (unsigned long)(elapsed / 1000u),
                       (max_level - min_level) / 10.0f,
                       g_deviceParams.water_stable_threshold / 10.0f);

                if (elapsed >= stable_window_ms)
                {
                    printf("快速跟随\t稳定满足：连续%lu.%03lus内波动<=阈值 -> 退出\r\n",
                           (unsigned long)(stable_window_ms / 1000u),
                           (unsigned long)(stable_window_ms % 1000u));
                    ret = MotorCtrl_SlowStop();
                    CHECK_ERROR(ret);
                    printf("快速跟随\t运行到水位附近\r\n");
                    ret = AlignToWaterLevel_01mm(lvl_avg);
                    CHECK_ERROR(ret);
                    return NO_ERROR;
                }
            }
        }

        CHECK_COMMAND_SWITCH(NO_ERROR);
    }
}
/**
 * @brief 快速水位跟随（基于电容阈值 + 滞回 + 自恢复）
 *
 * 核心思想：
 * 1. 以“空气电容 zero_capacitance”为基准，构造：
 *      - 进入水区阈值 th
 *      - 离开水区滞回阈值 th_low
 * 2. 根据当前电容与阈值关系，判断“偏水 / 偏空气 / 稳定区”
 * 3. 偏水则上行，偏空气则下行；稳定区不动作
 * 4. 根据偏差大小选择不同步长（大 / 中 / 小）
 * 5. 当出现“偏差很大但电容几乎不变化”的异常情况时，
 *    累计 lost_count，超过阈值后触发重新找水位
 *
 * 特点：
 * - 步进式运动（MotorCtrl_MoveAndWait）
 * - 带滞回，避免界面抖动
 * - 带自恢复机制，避免长期卡死在错误区域
 */
typedef enum {
    /* 水位搜索失步后的恢复策略选择。 */
    WATER_RECOVER_BY_SEARCH = 0, /* 失步后重新执行水位搜索以恢复介质边界。 */
    WATER_RECOVER_BY_STATE_FLIP = 1, /* 失步后翻转介质状态并沿相反方向继续确认。 */
} WaterRecoverStrategy;

/**
 * @brief 水位信号丢失后按配置策略重新搜索或翻转搜索方向。
 *
 * @param strategy 水位丢失后的恢复动作选择。
 * @return 返回重新搜索或翻转方向搜索的原始结果码；NO_ERROR 表示恢复成功，其他值保留命令切换、传感器或运动错误。
 */
static uint32_t WaterRecoverAfterLost(WaterRecoverStrategy strategy)
{
    if (strategy == WATER_RECOVER_BY_SEARCH) {
        return SearchWaterLevel();
    }

    return FindWaterLevel_FastByStateFlip_StableExit(WATER_STABLE_WINDOW_DEFAULT_MS);
}

/**
 * @brief 水位稳定后的变化监测循环。
 *
 * 连续多次处于稳定区后进入本函数，不主动调整电机，只周期性读取水位电容。
 * 当电容偏离跟随目标值超过 water_lag_cap_threshold 时，认为水位发生变化，
 * 退出本循环并返回原闭环跟随流程，由原逻辑重新判断偏水/偏空气并调整。
 *
 * @param target_cap 目标。
 * @return NO_ERROR 表示变化监测按当前结束条件正常退出；STATE_SWITCH、传感器超时、数据失效或水位丢失恢复失败由内部检查返回。
 */
static uint32_t MonitorWaterFollowChange(float target_cap)
{
    uint32_t ret;
    float cap = 0.0f;
    float diff;
    /* 水位滞后电容阈值：稳定监测中偏离目标超过该值后返回跟随循环。 */
    float monitor_diff = WaterCapRawToFloat(g_deviceParams.water_lag_cap_threshold);

    /* 连续稳定或超时兜底进入监测循环时，统一在这里确认跟随态。 */
    g_measurement.device_status.device_state = STATE_FOLLOW_WATERING;
    printf("水位跟随\t进入稳定监测，置为水位跟随状态\r\n");
    /* 刚进入稳定监测时锁定一次当前水位，后续监测只打印该水位值。 */
    WaterLevelSyncFromCable();

    while (1)
    {
        ret = SensorService_ReadWaterCapacitance(&cap);
        CHECK_ERROR(ret);

        g_measurement.water_measurement.current_capacitance = cap;
        /* 用当前电容与跟随目标电容的差值判断水位是否已经变化。 */
        diff = fabsf(cap - target_cap);

        printf("水位跟随\t稳定监测 电容=%.1f 目标：%.1f 偏差=%.1f 阈值=%.1f\r\n",
               cap, target_cap, diff, monitor_diff);

        if (diff > monitor_diff)
        {
            printf("水位跟随\t检测到水位变化，返回闭环跟随\r\n");
            return NO_ERROR;
        }

        /* 监测期间电机不动作，只打印进入监测时锁定的水位显示值。 */
        printf("水位跟随\t稳定监测 当前水位=%.1fmm\r\n",
               g_measurement.water_measurement.water_level / 10.0f);
        HAL_Delay(WATER_FOLLOW_SAMPLE_MS);
        CHECK_COMMAND_SWITCH(NO_ERROR);
    }
}

/**
 * @brief 以空气电容和配置阈值建立跟随区间，按偏差分级步进；稳定后进入监测，信号丢失时按策略重新搜索。
 *
 * 函数以空气零点电容加 water_cap_threshold 得到目标电容，再用 water_find_cap_threshold 建立上下滞回边界；三个参数均转换为统一浮点电容单位后参与计算。
 * 当前电容高于上边界时上行、低于下边界时下行，偏差越大使用的步进距离越大；每次动作均通过 MotorCtrl_MoveAndWait 阻塞等待完成，然后重新采样形成闭环。
 * 连续位于稳定区达到 WATER_FOLLOW_STABLE_MONITOR_COUNT 后进入不驱动电机的稳定监测；连续调节超过 WATER_FOLLOW_ENTER_TIMEOUT_MS
 * 仍未稳定时也进入监测，避免长期机械往返。
 * 监测到电容偏离目标后返回闭环调节；连续大偏差达到五次则按 recover_strategy 重新搜索水位，成功后清零丢失计数并继续跟随。
 * 测量、运动、重新搜索或命令切换错误原样返回；正常跟随没有主动成功结束点，会持续运行直到收到新命令或出现错误。
 *
 * @param recover_strategy 连续大偏差判定水位丢失后的恢复策略；WATER_RECOVER_BY_SEARCH
 *                         重新执行完整找水，WATER_RECOVER_BY_STATE_FLIP 使用快速状态翻转搜索。
 * @return STATE_SWITCH 表示水位跟随被新命令正常打断；其他非零值为电容读取、电机运动、稳定监测或丢失恢复流程原样传播的错误码；正常跟随持续运行，不以 NO_ERROR 主动结束。
 * @note 本函数包含阻塞式电机等待和 HAL_Delay，只能在测量任务流程调用，不得在中断上下文执行。
 */
static uint32_t FollowWaterLevelCore(WaterRecoverStrategy recover_strategy)
{
    uint32_t ret;

    /*
     * 闭环跟随主循环说明：
     * 1. 周期性读取当前水电容，并按 air_cap/th/th_low 判断“偏水 / 偏空气 / 稳定区”
     * 2. 偏水就上行，偏空气就下行，稳定区则保持不动
     * 3. 每次动作都是“小步进 + 重新测量”，所以这是一个持续运行的闭环调节过程
     * 4. 连续稳定或调节超时后进入稳定监测循环，并在监测循环入口置 STATE_FOLLOW_WATERING
     * 5. 若长时间一直在调节但还没进入稳定区，超过 WATER_FOLLOW_ENTER_TIMEOUT_MS 后强制进入稳定监测
     * 6. 若连续多次判断为“大偏差”，说明可能跟不上液面或测量异常，此时触发重新找水位
     */

    /* -------------------- 电容相关变量 -------------------- */
    float cap = 0.0f;         /* 当前读取到的水位电容值 */
    float air_cap;            /* 空气中的基准电容（零点电容） */
    float target_cap;         /* 水位跟随目标电容值 */
    float follow_band;        /* 水位跟上判定的电容允许范围 */
    float th;                 /* 水位判定上阈值（进入水区） */
    float th_low;             /* 水位判定下阈值（离开水区，滞回） */

    /* -------------------- 控制与计算变量 -------------------- */
    float diff;               /* 当前电容偏差量（相对于阈值） */
    float step_mm;            /* 本次运动步长（mm） */
    uint32_t dir;             /* 本次运动方向（UP / DOWN） */

    /* -------------------- 状态记忆与异常检测 -------------------- */
    static float last_cap = 0.0f; /* 上一次电容值，用于判断电容是否“卡死” */
    uint16_t lost_count = 0;      /* 连续异常计数器 */
    uint16_t stable_count = 0; /* 连续处于稳定区的采样计数 */
    uint32_t follow_enter_tick = HAL_GetTick();
    const uint32_t follow_enter_timeout_ms = WATER_FOLLOW_ENTER_TIMEOUT_MS;

    /* -------------------- 阈值计算 --------------------
     * 所有参数统一使用浮点计算，避免单位混乱
     * 三个电容阈值分工：
     * water_cap_threshold：确定水位跟随目标电容；
     * water_find_cap_threshold：目标上下范围内认为已经跟上；
     * water_lag_cap_threshold：稳定监测中判断水位变化。
     */
    air_cap = g_measurement.water_measurement.zero_capacitance;
    target_cap = air_cap + WaterCapRawToFloat(g_deviceParams.water_cap_threshold);
    follow_band = WaterCapRawToFloat(g_deviceParams.water_find_cap_threshold);
    th_low = target_cap - follow_band;
    th = target_cap + follow_band;

    printf("水位跟随\t开始\r\n");
    printf("水位跟随\t空气电容=%.1f  目标阈值上限：%.1f  滞回下限：%.1f\r\n",
           air_cap, th, th_low);

    /* ============================ 主循环 ============================ */
    while (1)
    {
        /* ---------- 1. 读取当前水位电容 ---------- */
        ret = SensorService_ReadWaterCapacitance(&cap);
        if (ret != NO_ERROR)
        {
            printf("水位跟随\t读取电容失败 错误码=0x%lX\r\n", ret);
            return ret;
        }

        /* 计算电容变化量（用于“是否卡死”判断） */
        float cap_delta = fabsf(cap - last_cap);
        last_cap = cap;

        /* 保存到测量结构体，供其他模块/调试使用 */
        g_measurement.water_measurement.current_capacitance = cap;

        printf("水位跟随\t位置=%.1fmm", g_measurement.debug_data.sensor_position / 10.0f); MotorCtrl_PrintPositionRefs(); printf("  电容=%.1f\r\n", cap);



        /* ---------- 2. 基于双阈值 + 滞回的状态判断 ---------- */
        if (cap >= th)
        {
            /* 电容高于上阈值：探头偏水 -> 需要上行 */
            dir  = MOTOR_DIRECTION_UP;
            diff = cap - th;
            printf("水位跟随\t状态=偏水  差值：%.1f -> 上行\r\n", diff);
        }
        else if (cap <= th_low)
        {
            /* 电容低于滞回下限：探头偏空气 -> 需要下行 */
            dir  = MOTOR_DIRECTION_DOWN;
            diff = th_low - cap;
            printf("水位跟随\t状态=偏空气 差值：%.1f -> 下行\r\n", diff);
        }
        else
        {
            /* ---------- 稳定区 ----------
             * 电容位于 [th_low, th] 之间
             * 认为已经贴近液面，不进行任何运动
             */
            if (lost_count != 0)
            {
                printf("水位跟随\t进入稳定区，丢失计数清零(%u->0)\r\n", lost_count);
                lost_count = 0;
            }

            if (stable_count < 0xFFFFu)
            {
                stable_count++;
            }

            printf("水位跟随\t状态=稳定区(%.1f < 电容 < %.1f)，保持\r\n",
                   th_low, th);

            /* 稳定区只打印当前水位值，不在这里刷新水位。 */
            /* WaterLevelSyncFromCable(); */
            printf("水位跟随\t稳定区 当前水位=%.1fmm\r\n",
                   g_measurement.water_measurement.water_level / 10.0f);

            if (stable_count >= WATER_FOLLOW_STABLE_MONITOR_COUNT)
            {
                stable_count = 0;
                ret = MonitorWaterFollowChange(target_cap);
                CHECK_COMMAND_SWITCH(ret);
                CHECK_ERROR(ret);
                continue;
            }

            HAL_Delay(WATER_FOLLOW_SAMPLE_MS);
            CHECK_COMMAND_SWITCH(NO_ERROR);
            continue;
        }

        /* ---------- 3. 根据偏差大小选择运动步长 ----------
         * 偏差越大，说明离目标液面越远，允许使用更大的步长
         */
        stable_count = 0;

        if (diff > 0.6f * WaterCapRawToFloat(g_deviceParams.water_cap_threshold))
        {
            step_mm = WATER_FOLLOW_STEP_BIG_MM;
        }
        else if (diff > 0.1f * WaterCapRawToFloat(g_deviceParams.water_cap_threshold))
        {
            step_mm = WATER_FOLLOW_STEP_MED_MM;
        }
        else
        {
            step_mm = WATER_FOLLOW_STEP_SMALL_MM;
        }

        printf("水位跟随\t执行移动 方向：%s  步长=%.2fmm  丢失计数=%u\r\n",
               (dir == MOTOR_DIRECTION_UP) ? "UP" : "DOWN",
               step_mm,
               lost_count);

        /* ---------- 4. 执行步进运动（阻塞等待完成） ---------- */
        ret = MotorCtrl_MoveAndWait(step_mm, dir, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);

        /*
         * 长时间未进入稳定区时，也统一进入稳定监测循环。
         * 状态只在 MonitorWaterFollowChange() 入口处置位，避免多个入口直接改跟随状态。
         */
        if ((HAL_GetTick() - follow_enter_tick) >= follow_enter_timeout_ms)
        {
            printf("水位跟随\t连续调节%lu.%03lus仍未进入稳定区，进入稳定监测\r\n",
                   (unsigned long)((HAL_GetTick() - follow_enter_tick) / 1000u),
                   (unsigned long)((HAL_GetTick() - follow_enter_tick) % 1000u));
            ret = MonitorWaterFollowChange(target_cap);
            CHECK_COMMAND_SWITCH(ret);
            CHECK_ERROR(ret);
            follow_enter_tick = HAL_GetTick();
            continue;
        }

        /* 运动完成后只打印当前水位值，不在这里刷新水位。 */

        printf("水位跟随\t完成移动 位置=%.1fmm", g_measurement.debug_data.sensor_position / 10.0f); MotorCtrl_PrintPositionRefs();
        printf("  当前水位=%.1fmm\r\n", g_measurement.water_measurement.water_level / 10.0f);


        /* ---------- 5. 异常判定：大偏差 + 电容几乎不变 ----------
         *
         * 含义：
         * - diff 很大：理论上应该迅速变化
         * - cap_delta 很小：实际几乎没变
         * => 可能卡死 / 饱和 / 探头异常 / 运动无效
         */
        {
            float th_span = WaterCapRawToFloat(g_deviceParams.water_cap_threshold);

/* if ((diff > 0.8f * th_span) && (cap_delta < 1.0f)) */
            if (diff > 0.6f * th_span)
            {
                if (lost_count < 0xFFFFu)
                    lost_count++;
            }
            else
            {
                lost_count = 0;
            }

            printf("水位跟随\t丢失判定 差值：%.1f 阈值=%.1f 电容变化=%.1f -> 丢失计数=%u\r\n",
                   diff, th_span, cap_delta, lost_count);
        }

        /* ---------- 6. 连续异常过多：触发重新找水位 ---------- */
        if (lost_count >= 5)
        {
            printf("水位跟随\t长时间大偏差，重新找水位\r\n");
            ret = WaterRecoverAfterLost(recover_strategy);
            CHECK_ERROR(ret);

            lost_count = 0;
            printf("水位跟随\t重新找水位完成，恢复跟随\r\n");
        }

        /* ---------- 7. 采样周期延时 + 命令切换检测 ---------- */
        HAL_Delay(WATER_FOLLOW_SAMPLE_MS);
        CHECK_COMMAND_SWITCH(NO_ERROR);
    }
}

/**
 * @brief 以快速步进策略跟随水位并收敛到电容目标区间。
 * @return 返回快速水位跟随循环的最终状态；STATE_SWITCH 表示新命令打断，其他非零值为电容读取、目标区间收敛或电机运动错误。
 */
uint32_t FollowWaterLevel_fast(void)
{
    uint32_t ret = FollowWaterLevelCore(WATER_RECOVER_BY_STATE_FLIP);

    if (ret != NO_ERROR) {
        AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL);
    }
    return ret;
}

/**
 * @brief 执行水位跟随主流程并发布最终测量状态。
 * @return 返回所选水位跟随实现的结果；STATE_SWITCH 表示正常命令打断，其他非零值为水位定位、电容读取、运动、稳定性或丢失恢复错误。
 */
uint32_t FollowWaterLevel(void)
{
    uint32_t ret = FollowWaterLevelCore(WATER_RECOVER_BY_SEARCH);

    if (ret != NO_ERROR) {
        AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL);
    }
    return ret;
}


/**
 * @brief 用当前尺带长度和命令修正量计算水罐高度，校验后同步水位并持久化。
 *
 * @return NO_ERROR 表示修正后的水罐高度、同步水位和持久化参数均完成；输入范围、当前位置、参数写回或保存失败时由检查宏返回对应错误码。
 */
static uint32_t CorrectWaterTankHeightProcess(void)
{
    int32_t new_height;

    printf("水位标定\t开始\r\n");
    printf("水位标定\t当前缆长=%.1fmm  标定真值=%.1fmm\r\n",
           g_measurement.debug_data.cable_length / 10.0f,
           DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL) / 10.0f);

    /* 核心公式：water_tank_height = cable_length_at_water + calibrateWaterLevel */
    new_height = (int32_t)g_measurement.debug_data.cable_length
              + (int32_t)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL);

    /* 合理性保护 */
    if (new_height <= 0 || new_height > 5000000) { /* 例：500m -> 5,000,000(0.1mm) */
        printf("水位标定\t计算得到水位罐高非法：%ld(0.1mm)\r\n", new_height);
        RETURN_ERROR(MEASUREMENT_WATER_CALC_OUT_OF_RANGE);
    }

    g_deviceParams.water_tank_height = new_height;
    /* 水位罐高变化后立即归一化 AO 量程，避免旧量程阻塞后续命令。 */
    (void)normalize_ao_params_after_write();
	WaterLevelSyncFromCable();
    /* 标定完成后清零，防止重复触发 */
    DeviceCommandArguments_ClearIfUnchanged(DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL);

    save_device_params();

    printf("水位标定\t完成 水位罐高=%.1fmm\r\n",
           g_deviceParams.water_tank_height / 10.0f);

    return NO_ERROR;
}
 /**
  * @brief 按用户给出的水位真值修正水罐高度；非跟随态先重找水位，跟随态修正后恢复原跟随模式。
  *
  * 标定真值为 0 时立即报告未配置错误；非跟随态调用 SearchWaterLevel 精确定位水面，命令切换时禁止继续使用旧位置修正参数。
  * 定位或沿用跟随位置后调用 CorrectWaterTankHeightProcess 计算并保存水罐高度；原先处于跟随态时按 water_level_mode 恢复普通或快速跟随。
  */
 void CMD_CalibrateWaterLevel(void)
{
    uint32_t ret = NO_ERROR;
    uint8_t  resume_follow = 0;

    MeasureStart();

    resume_follow = (g_measurement.device_status.device_state == STATE_FOLLOW_WATERING);

    /*
     * 无论当前是否处于跟随态，水位标定都依赖用户给出的真值。
     * 若真值为 0，则无法反推 water_tank_height。
     */
    if (DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_CALIBRATE_WATER_LEVEL) == 0) {
        printf("水位标定\t未设置标定水位真值(标定水位=0)，无法标定\r\n");
        SET_ERROR(MEASUREMENT_WATER_CALIBRATION_NOT_CONFIGURED);
    }

    g_measurement.device_status.device_state = STATE_CALIBRATE_WATERING;

    if (resume_follow)
    {
		printf("水位标定\t当前处于水位跟随状态\t修正后继续跟随\r\n");
    }
    else
	{
		printf("水位标定\t当前状态需要重新寻找水位\r\n");

		/* 先找并精确定位水位界面 */
		ret = SearchWaterLevel();
        if (ret == STATE_SWITCH)
        {
            /* 新命令属于正常切换，立即退出标定，禁止继续修正和保存旧标定结果。 */
            return;
        }
		SET_ERROR(ret);
	}
    /*
     * SearchWaterLevel() 成功后：
     * - 探头在水位界面附近
     * - g_measurement.debug_data.cable_length 已更新
     */
    ret = CorrectWaterTankHeightProcess();
    SET_ERROR(ret);

    if (resume_follow)
    {
        printf("水位标定\t修正完成，恢复水位跟随\r\n");
        g_measurement.device_status.device_state = STATE_FOLLOW_WATERING;

        if (g_deviceParams.water_level_mode == 0) {
            ret = FollowWaterLevel();
        } else {
            ret = FollowWaterLevel_fast();
        }
        SET_ERROR(ret);
        return;
    }

    g_measurement.device_status.device_state = STATE_CALIBRATE_WATER_OVER;

    return;
}


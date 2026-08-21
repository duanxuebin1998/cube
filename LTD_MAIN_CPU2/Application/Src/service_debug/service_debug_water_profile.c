/**
 * @file service_debug_water_profile.c
 * @brief 水位传感器上下行电容剖面测试，用于维护诊断，不参与正常测量调度。
 */
#include "service_debug_internal.h"
#include "measure_water_level.h"
#include "measure.h"
#include "motor_ctrl.h"
#include "measure_zero.h"
#include "sensor_service.h"
#include "error_log.h"
#include "AoOutput/ao_output.h"

#include <stdio.h>
#include <stdlib.h>
/* 找到水位后，开始剖面扫描前的上行避让距离，单位 mm。 */
#define WATER_SENSOR_TEST_PRE_UP_MM   (50.0f)
/* 剖面扫描单步距离，单位 mm。 */
#define WATER_SENSOR_TEST_SCAN_STEP_MM (0.1f)
/* 单方向剖面扫描测点数量；1000 点对应 100 mm 行程。 */
#define WATER_SENSOR_TEST_POINT_COUNT (1000U)

/** 水位传感器剖面测试中的位置与电容配对样本。 */
typedef struct {
    int32_t position_01mm; /* 采样位置，单位 0.1 mm。 */
    float capacitance;     /* 对应位置读取到的水位探头电容。 */
} WaterSensorTestPoint;

/* 下行剖面样本缓存，按运动顺序保存固定数量测点。 */
static WaterSensorTestPoint s_water_sensor_test_down_points[WATER_SENSOR_TEST_POINT_COUNT];
/* 上行剖面样本缓存，按运动顺序保存固定数量测点。 */
static WaterSensorTestPoint s_water_sensor_test_up_points[WATER_SENSOR_TEST_POINT_COUNT];
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
                                       s_water_sensor_test_down_points,
                                       WATER_SENSOR_TEST_POINT_COUNT);
    CHECK_COMMAND_SWITCH(ret);
    CHECK_ERROR(ret);

    ret = WaterSensorTestScanDirection("上行扫描",
                                       MOTOR_DIRECTION_UP,
                                       s_water_sensor_test_up_points,
                                       WATER_SENSOR_TEST_POINT_COUNT);
    CHECK_COMMAND_SWITCH(ret);
    CHECK_ERROR(ret);

    printf("水位传感器测试\t扫描完成，开始打印结果\r\n");
    WaterSensorTestPrintResults("下行扫描",
                                s_water_sensor_test_down_points,
                                WATER_SENSOR_TEST_POINT_COUNT,
                                water_pos_01mm);
    WaterSensorTestPrintResults("上行扫描",
                                s_water_sensor_test_up_points,
                                WATER_SENSOR_TEST_POINT_COUNT,
                                water_pos_01mm);

    printf("水位传感器测试\t结束 当前位置=%.1fmm\r\n",
           g_measurement.debug_data.sensor_position / 10.0f);
    return NO_ERROR;
}

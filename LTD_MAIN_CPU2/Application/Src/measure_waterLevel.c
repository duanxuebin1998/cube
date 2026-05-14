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
#include "sensor.h"
#include "error_log.h"
// TODO: 这里替换为你的水位检测头文件
// #include "water.h"  // 提供 check_water_status()

/* -------------------- 可配置参数 -------------------- */
#define WATER_INIT_UP_MM                 (100.0f)   // 初始上行避让
#define WATER_ROUGH_RETRY_MAX            (3)
#define WATER_PRECISE_RETRY_MAX          (3)
#define WATER_ROUGH_CONFIRM_DELAY_MS     (3000)     // 粗找停下后等待稳定
#define WATER_FAIL_RECOVER_UP_MM         (100.0f)   // 粗找失败上行回退

#define WATER_V1_SLOWDOWN_TH             (1000)     // 接近粗定位点：第一次降速阈值
#define WATER_V2_SLOWDOWN_TH             (100)      // 更接近：第二次降速阈值
#define WATER_OVERSHOOT_TH               (-100)     // 走过头保护阈值

/* 速度设置（沿用你 bottom 的写法） */
#define WATER_VEL_MID                    (16 * 32 * 40)
#define WATER_VEL_SLOW                   (16 * 32 * 2)

//#define WATER_FOLLOW_OFFSET              (5.0f)     /* 阈值 = air + 50 */
//#define WATER_FOLLOW_HYSTERESIS          (0.3f)     /* 滞回，防抖：可调 3~10 */
//#define WATER_FOLLOW_SAMPLE_MS           (500)      /* 采样周期 */
//#define WATER_FOLLOW_STEP_SMALL_MM       (0.2f)     /* 阈值附近小步 */
//#define WATER_FOLLOW_STEP_MED_MM         (5.0f)     /* 中等偏差 */
//#define WATER_FOLLOW_STEP_BIG_MM         (20.0f)    /* 大偏差/饱和时快速拉回 */
//#define WATER_CAP_SAT_LIMIT              (9999.0f)  /* 认为“饱和/无穷大”的阈值，用于保护 */
//#define WATER_FOLLOW_LOST_DIFF_BIG       (2.5f)     /* 认为偏差很大 */
//#define WATER_FOLLOW_LOST_COUNT_MAX      (20)       /* 连续大偏差次数阈值：20次*500ms=10s */


//#define WATER_FOLLOW_OFFSET            (50.0f)   /* 阈值 = air + 50 */
//#define WATER_FOLLOW_HYSTERESIS        (5.0f)    /* 滞回，防抖：可调 3~10 */
#define WATER_FOLLOW_SAMPLE_MS         (500)     /* 采样周期 */
#define WATER_FOLLOW_STEP_SMALL_MM     (0.2f)    /* 阈值附近小步 */
#define WATER_FOLLOW_STEP_MED_MM       (3.0f)    /* 中等偏差 */
#define WATER_FOLLOW_STEP_BIG_MM       (10.0f)    /* 大偏差/饱和时快速拉回 */
#define WATER_CAP_SAT_LIMIT            (9999.0f)/* 认为“饱和/无穷大”的阈值，用于保护 */
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
int32_t water_value = -100000000; // 初始值设为无效

/* -------------------- 函数原型 -------------------- */
typedef struct {
    int32_t position_01mm;
    float capacitance;
} WaterSensorTestPoint;

static WaterSensorTestPoint g_water_sensor_test_up_points[WATER_SENSOR_TEST_POINT_COUNT];
static WaterSensorTestPoint g_water_sensor_test_down_points[WATER_SENSOR_TEST_POINT_COUNT];

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

static inline float WaterCapRawToFloat(uint32_t raw)
{
    return raw / 1000.0f;
}

static inline void WaterLevelSetAndLog(int32_t lvl)
{
    int32_t old_lvl = g_measurement.water_measurement.water_level;

    g_measurement.water_measurement.water_level = lvl;
    water_value = lvl;

    if (old_lvl != lvl)
    {
        printf("水位更新\t旧值=%.1fmm 新值=%.1fmm 缆长=%.1fmm\r\n",
               old_lvl / 10.0f,
               lvl / 10.0f,
               g_measurement.debug_data.cable_length / 10.0f);
    }
}

static inline void WaterLevelSyncFromCable(void)
{
    int32_t lvl = g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;
    WaterLevelSetAndLog(lvl);
}

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

        ret = Sensor_ReadWaterCapacitance(&points[i].capacitance);
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
    //打印零点电容值，水位电容阈值，水位滞后阈值
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

    /*************** 粗找阶段 - 带重试机制 ***************/
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
            // 错误	阶段：错误重试	模块：测量	操作：粗找水位	原因：搜索失败	尝试：try_times/WATER_ROUGH_RETRY_MAX	错误码：ret	错误名：ErrorLog_GetCodeName(ret)
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
                // 错误	阶段：重试成功	模块：测量	操作：粗找水位	原因：恢复成功	尝试：try_times/WATER_ROUGH_RETRY_MAX
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

    /*************** 精找阶段 - 带重试 ***************/
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
                // 错误	阶段：重试成功	模块：测量	操作：精找水位	原因：恢复成功	尝试：try_times/WATER_PRECISE_RETRY_MAX
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
            // 错误	阶段：错误重试	模块：测量	操作：精找水位	原因：搜索失败	尝试：try_times/WATER_PRECISE_RETRY_MAX	错误码：ret	错误名：ErrorLog_GetCodeName(ret)
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

    /*************** 最终记录 ***************/
    WaterLevelSyncFromCable();

    printf("水位测量\t水位：%ld mm\r\n", g_measurement.water_measurement.water_level);

    return NO_ERROR;
}

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

static int SearchWaterRough(void)
{
    uint32_t ret;
    uint8_t  water_state = NORMAL;

    MotorCtrl_LostStepInit();// 重置丢步检测计数器
    /* 持续下探直到检测到 WATER */
    while (1)
    {
	ret = MotorCtrl_MoveDown(MotorCtrl_GetDefaultSpeedX100());  // 启动电机向下运动
    	CHECK_ERROR(ret); // 检查上行是否成功

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
        water_value = g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;
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

    MotorCtrl_LostStepInit();// 重置丢步检测计数器
    while (1)
    {
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        if (g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length - water_value < WATER_V2_SLOWDOWN_TH)
        {
            speed_x100 = 4;
        }
        else if (g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length - water_value < WATER_V1_SLOWDOWN_TH)
        {
            speed_x100 = 40;
        }

	ret = MotorCtrl_MoveDown(speed_x100);  // 启动电机向下运动
    	CHECK_ERROR(ret); // 检查上行是否成功

        ret = check_water_status(&water_state);
        CHECK_ERROR(ret);

        if (water_state == WATER) {
            break;
        }

        /* 走过头保护 */
//        if (g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length - water_value < WATER_OVERSHOOT_TH)
//        {
//            printf("水位测量\t精确寻找水位未找到水位\r\n");
//            RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
//        }

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        printf("水位测量\t精确寻找水位\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position) / 10.0f); MotorCtrl_PrintPositionRefs(); printf("\t速度(0.01m/min)\t%lu\t", (unsigned long)g_measurement.debug_data.motor_speed);
    }

    ret = MotorCtrl_SlowStop();
    CHECK_ERROR(ret);

    water_value = g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;
    return NO_ERROR;
}

uint32_t read_zero_capacitance(void)
{
    uint32_t ret;
    float    cap = 0.0f;

    ret = Sensor_ReadWaterCapacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;
    }
    g_deviceParams.zero_cap = 10*cap;
    g_measurement.water_measurement.zero_capacitance = cap;

    printf("零点电容 = %.1f\r\n", cap);
    //参数存储
    save_device_params();
    return NO_ERROR;
}

uint32_t read_oil_capacitance(void)
{
    uint32_t ret;
    float    cap = 0.0f;

    ret = Sensor_ReadWaterCapacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;
    }

    g_measurement.water_measurement.oil_capacitance = cap;

    printf("油中电容 = %.1f\r\n", cap);
    return NO_ERROR;
}

uint32_t check_water_status(uint8_t *water_state)
{
    uint32_t ret;
    float    cap = 0.0f;
    float    zero;
    float    th;

    if (water_state == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    ret = Sensor_ReadWaterCapacitance(&cap);
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
 * @brief 根据当前设备状态更新水位值
 *
 * @note 仅在“已找到水位 / 水位跟随完成”等合法状态下才更新，
 *       防止在无效阶段误更新 water_level
 *
 * @return 1 已更新水位
 * @return 0 未更新（状态不允许）
 */
uint8_t UpdateWaterLevelIfValid(void)
{
    if (g_measurement.device_status.device_state == STATE_FINDWATER_OVER ||
                g_measurement.device_status.device_state == STATE_FOLLOW_WATERING)
    {
        /* 监测期间电机不动作，仅持续同步当前水位显示值。 */
        WaterLevelSyncFromCable();

        return 1;
    }

    return 0;
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
    int32_t cable_target_01mm = g_deviceParams.water_tank_height - lvl_target_01mm; /* 0.1mm */
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
    have_last_flip = 0;//每次调用都重置，确保独立测量
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
    //打印零点电容值，水位电容阈值，水位滞后阈值
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
        int32_t lvl_flip = g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length; /* 0.1mm */


        /* 第一次翻转：只缓存，不更新水位/不判稳 */
        if (!have_last_flip)
        {
            have_last_flip = 1;
            last_flip_lvl  = lvl_flip;
            win_start_tick = HAL_GetTick();//开始稳定判定计时
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
                //如果设备状态不是水位跟随状态，设置水位跟随状态
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
    WATER_RECOVER_BY_SEARCH = 0,
    WATER_RECOVER_BY_STATE_FLIP = 1,
} WaterRecoverStrategy;

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
        ret = Sensor_ReadWaterCapacitance(&cap);
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
        ret = Sensor_ReadWaterCapacitance(&cap);
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
            // WaterLevelSyncFromCable();
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
        // UpdateWaterLevelIfValid();

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

//            if ((diff > 0.8f * th_span) && (cap_delta < 1.0f))
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

uint32_t FollowWaterLevel_fast(void)
{
    return FollowWaterLevelCore(WATER_RECOVER_BY_STATE_FLIP);
}

uint32_t FollowWaterLevel(void)
{
    return FollowWaterLevelCore(WATER_RECOVER_BY_SEARCH);
}

#define WATER_TANK_HEIGHT_EPSILON   (1)   /* 0.1mm，避免边界抖动，可按需保留 */

static uint32_t CorrectWaterTankHeightProcess(void)
{
    int32_t new_height;

    printf("水位标定\t开始\r\n");
    printf("水位标定\t当前缆长=%.1fmm  标定真值=%.1fmm\r\n",
           g_measurement.debug_data.cable_length / 10.0f,
           g_deviceParams.calibrateWaterLevel / 10.0f);

    /* 核心公式：water_tank_height = cable_length_at_water + calibrateWaterLevel */
    new_height = (int32_t)g_measurement.debug_data.cable_length
              + (int32_t)g_deviceParams.calibrateWaterLevel
              + WATER_TANK_HEIGHT_EPSILON;

    /* 合理性保护 */
    if (new_height <= 0 || new_height > 5000000) { /* 例：500m -> 5,000,000(0.1mm) */
        printf("水位标定\t计算得到水位罐高非法：%ld(0.1mm)\r\n", new_height);
        RETURN_ERROR(PARAM_ERROR);
    }

    g_deviceParams.water_tank_height = new_height;
	WaterLevelSyncFromCable();
    /* 标定完成后清零，防止重复触发 */
    g_deviceParams.calibrateWaterLevel = 0;

    save_device_params();

    printf("水位标定\t完成 水位罐高=%.1fmm\r\n",
           g_deviceParams.water_tank_height / 10.0f);

    return NO_ERROR;
}
// 标定水位：修正 water_tank_height
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
    if (g_deviceParams.calibrateWaterLevel == 0) {
        printf("水位标定\t未设置标定水位真值(标定水位=0)，无法标定\r\n");
        SET_ERROR(PARAM_ERROR);
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


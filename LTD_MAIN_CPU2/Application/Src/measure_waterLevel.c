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

#include "motor_ctrl.h"
#include "measure_zero.h"
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

/* -------------------- 全局变量 -------------------- */
/* 存储最终确定的水位位置（以 sensor_position 记录） */
int32_t water_value = -100000000; // 初始值设为无效


/* -------------------- 函数原型 -------------------- */
static int SearchWaterRough(void);
static int SearchWaterPrecise(void);

/**
 * @brief 水位测量函数 - 执行完整的水位搜索流程
 *        包含：粗找水位（3次重试） + 精找水位（3次重试）
 *        状态切换时不重试
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */
uint32_t SearchWaterLevel(void)
{
    uint32_t ret;
    uint8_t try_times = 0;
    uint8_t water_state = NORMAL;

    fault_info_init();
    printf("水位测量\t开始\r\n");

    /* -------------------- 零点检查 -------------------- */
    if (g_measurement.device_status.zero_point_status == 1)
    {
        printf("水位测量\t设备需要回零点\r\n");
        ret = SearchZero();
        CHECK_ERROR(ret);
        printf("水位测量\t回零点完成\r\n");
    }

    /* -------------------- 初始位置调整（避让） -------------------- */
    if (g_measurement.debug_data.cable_length > 2000)
    {
        ret = motorMoveAndWaitUntilStop(WATER_INIT_UP_MM, MOTOR_DIRECTION_UP);
        CHECK_ERROR(ret);
        printf("水位测量\t上行完成\r\n");
    }

    printf("水位测量\t初始位置：%ld\r\n", g_measurement.debug_data.sensor_position);

    ret = check_water_status(&water_state);
    CHECK_ERROR(ret);

    if (water_state != NORMAL)
    {
        /* 约定：sensor_position 单位为 0.1mm（从你打印 /10 推断） */
        const int32_t ZERO_NEAR_TH   = 1000;   /* 100mm -> 1000(0.1mm) */
        const float   UP_STEP_MM     = 100.0f; /* 每次上行 100mm */

        printf("水位测量\t当前在水区，执行上行避让\r\n");

        while (1)
        {
            /* 距离零点 <= 100mm：认为已到零点附近，仍在水里 -> 报错退出 */
            if (g_measurement.debug_data.sensor_position <= ZERO_NEAR_TH)
            {
                printf("水位测量\t零点附近仍在水区，无法避让(pos=%.1fmm)\r\n",
                       g_measurement.debug_data.sensor_position / 10.0f);
                RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
            }

            /* 距离零点 > 100mm：上行 100mm */
            ret = motorMoveAndWaitUntilStop(UP_STEP_MM, MOTOR_DIRECTION_UP);
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
        printf("水位测量\t粗找水位第%d次尝试\r\n", try_times);

        fault_info_init();
        ret = SearchWaterRough();

        if (ret == STATE_SWITCH)
        {
            printf("水位测量\t检测到命令切换，中止粗找\r\n");
            break;
        }

        CHECK_COMMAND_SWITCH(ret);

        if (ret != NO_ERROR)
        {
            printf("水位测量\t粗找失败[%d]:0x%lX\r\n", try_times, ret);
            HAL_Delay(1000);
            continue;
        }
        else
        {
            printf("水位测量\t粗找水位成功\r\n");
            break;
        }
    }

    if (ret != NO_ERROR)
    {
        printf("水位测量\t粗找水位失败(尝试%d次)\r\n", try_times);
        RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
    }

    printf("水位测量\t粗找完成：水位：%ld\r\n", water_value);

    /*************** 精找阶段 - 带重试 ***************/
    try_times = 0;
    while (try_times < WATER_PRECISE_RETRY_MAX)
    {
        try_times++;
        printf("水位测量\t精找水位第%d次尝试\r\n", try_times);

        ret = SearchWaterPrecise();

        if (ret == STATE_SWITCH)
        {
            printf("水位测量\t检测到命令切换，中止精找\r\n");
            break;
        }

        if (ret == NO_ERROR)
        {
            printf("水位测量\t精找完成\r\n");
            break;
        }
        else
        {
            printf("水位测量\t精找失败:0x%lX\r\n", ret);
            HAL_Delay(1000);
        }
    }

    if (ret != NO_ERROR)
    {
        printf("水位测量\t精找失败(尝试%d次)\r\n", try_times);
        CHECK_ERROR(ret);
    }

    /*************** 最终记录 ***************/
    g_measurement.water_measurement.water_level = g_measurement.debug_data.sensor_position;
    printf("水位测量\t水位：%ld mm\r\n", g_measurement.water_measurement.water_level);

    return NO_ERROR;
}

static int SearchWaterRough(void)
{
    uint32_t ret;
    uint8_t water_state = NORMAL;

    ret = motorMove_down();
    CHECK_ERROR(ret);

    /* 持续下探直到检测到 WATER */
    while (1)
    {
        ret = check_water_status(&water_state);
        CHECK_ERROR(ret);

        if (water_state != NORMAL) {
            break;
        }

        ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        printf("水位测量\t长距离寻找水位\t{传感器位置}%.1f\t",
               (float)(g_measurement.debug_data.sensor_position) / 10.0f);
    }

    ret = motorQuickStop();
    CHECK_ERROR(ret);

    HAL_Delay(WATER_ROUGH_CONFIRM_DELAY_MS);

    printf("水位测量\t确认粗找水位位置\t{传感器位置}%.1f\t",
           (float)(g_measurement.debug_data.sensor_position) / 10.0f);

    /* 停稳后再读一次，确认状态 */
    ret = check_water_status(&water_state);
    CHECK_ERROR(ret);

    if (water_state == WATER)
    {
        water_value = g_measurement.debug_data.sensor_position;
        return NO_ERROR;
    }
    else
    {
        printf("水位测量\t粗找水位未成功，准备回退重试\r\n");

        ret = motorMoveAndWaitUntilStop(WATER_FAIL_RECOVER_UP_MM, MOTOR_DIRECTION_UP);
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
    int v1flag = 0;
    int v2flag = 0;
    uint8_t water_state = NORMAL;

    if (g_measurement.debug_data.sensor_position > 2000)
    {
        ret = motorMoveAndWaitUntilStop(WATER_INIT_UP_MM, MOTOR_DIRECTION_UP);
        CHECK_ERROR(ret);
        printf("水位测量\t上行完成\r\n");
    }

    ret = motorMove_down();
    CHECK_ERROR(ret);

    while (1)
    {
        ret = check_water_status(&water_state);
        CHECK_ERROR(ret);

        if (water_state == WATER) {
            break;
        }

        /* 接近粗定位点：第一次降速 */
        if ((g_measurement.debug_data.sensor_position- water_value   < WATER_V1_SLOWDOWN_TH) && (v1flag == 0))
        {
            stpr_setVelocity(&stepper, WATER_VEL_MID);
            printf("水位测量\t速度变化：%d\r\n", 40);
            v1flag = 1;
        }

        /* 更接近：第二次降速 */
        if ((g_measurement.debug_data.sensor_position- water_value  < WATER_V2_SLOWDOWN_TH) && (v2flag == 0))
        {
            stpr_setVelocity(&stepper, WATER_VEL_SLOW);

            printf("水位测量\t速度变化：%d\r\n", 2);

            v2flag = 1;
        }

        /* 走过头保护 */
        if (g_measurement.debug_data.sensor_position-water_value < WATER_OVERSHOOT_TH)
        {
            printf("水位测量\t精确寻找水位未找到水位\r\n");
            RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
        }

        ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        printf("水位测量\t精确寻找水位\t{传感器位置}%.1f\t",
               (float)(g_measurement.debug_data.sensor_position) / 10.0f);
    }

    ret = motorQuickStop();
    CHECK_ERROR(ret);

    water_value = g_measurement.debug_data.sensor_position;
    return NO_ERROR;
}

uint32_t read_zero_capacitance(void)
{
    uint32_t ret;
    float cap = 0.0f;

    ret = Read_Water_Capacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;
    }

    g_measurement.water_measurement.zero_capacitance = cap;

    printf(" Zero capacitance = %.1f\r\n", cap);
    return NO_ERROR;
}

uint32_t read_oil_capacitance(void)
{
    uint32_t ret;
    float cap = 0.0f;

    ret = Read_Water_Capacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;
    }

    g_measurement.water_measurement.oil_capacitance = cap;

    printf("Oil capacitance = %.1f\r\n", cap);
    return NO_ERROR;
}

uint32_t check_water_status(uint8_t *water_state)
{
    uint32_t ret;
    float cap = 0.0f;

    if (water_state == NULL) {
        return PARAM_ERROR;   /* 或你工程里的参数错误码 */
    }

    ret = Read_Water_Capacitance(&cap);
    if (ret != NO_ERROR) {
        return ret;           /* 只返回错误码 */
    }
    printf("Water capacitance = %.1f\r\n", cap);
    g_measurement.water_measurement.current_capacitance = cap;

    if (cap > (g_measurement.water_measurement.zero_capacitance + 50.0f)) {
        *water_state = WATER;
    } else {
        *water_state = NORMAL;
    }

    return NO_ERROR;
}

#define WATER_FOLLOW_OFFSET            (50.0f)   /* 阈值 = air + 50 */
#define WATER_FOLLOW_HYSTERESIS        (5.0f)    /* 滞回，防抖：可调 3~10 */
#define WATER_FOLLOW_SAMPLE_MS         (500)     /* 采样周期 */
#define WATER_FOLLOW_STEP_SMALL_MM     (0.2f)    /* 阈值附近小步 */
#define WATER_FOLLOW_STEP_MED_MM       (1.0f)    /* 中等偏差 */
#define WATER_FOLLOW_STEP_BIG_MM       (5.0f)    /* 大偏差/饱和时快速拉回 */
#define WATER_CAP_SAT_LIMIT            (9999.0f)/* 认为“饱和/无穷大”的阈值，用于保护 */
#define WATER_FOLLOW_LOST_DIFF_BIG       (80.0f)   /* 认为偏差很大 */
#define WATER_FOLLOW_LOST_COUNT_MAX      (20)      /* 连续大偏差次数阈值：20次*500ms=10s */

uint32_t FollowWaterLevel(void)
{
    uint32_t ret;
    float cap = 0.0f;
    float air_cap;
    float th;
    float th_low;
    float diff;
    float step_mm;
    uint32_t dir;

    uint16_t lost_count = 0;   /* 连续大偏差计数 */

RESTART_FOLLOW:
    air_cap = g_measurement.water_measurement.zero_capacitance;
    th     = air_cap + WATER_FOLLOW_OFFSET;
    th_low = th - WATER_FOLLOW_HYSTERESIS;

    printf("水位跟随\t开始\r\n");
    printf("水位跟随\t空气电容=%.1f  目标阈值=%.1f  滞回下限=%.1f\r\n",
           air_cap, th, th_low);

    while (1)
    {
        /* 读取电容 */
        ret = Read_Water_Capacitance(&cap);
        if (ret != NO_ERROR) {
            printf("水位跟随\t读取电容失败 ret=0x%lX\r\n", ret);
            return ret;
        }

        g_measurement.water_measurement.current_capacitance = cap;

        printf("水位跟随\tpos=%.1fmm  cap=%.1f\r\n",
               g_measurement.debug_data.sensor_position / 10.0f,
               cap);

        /* 饱和保护：饱和也算“跟不上”，累计 lost_count */
        if (cap > WATER_CAP_SAT_LIMIT) {
            lost_count++;
            printf("水位跟随\t电容饱和(cap=%.1f) lost=%u\r\n", cap, lost_count);

            /* 饱和时先强制上行拉回 */
            dir = MOTOR_DIRECTION_UP;
            step_mm = WATER_FOLLOW_STEP_BIG_MM;

            printf("水位跟随\t执行移动 dir=UP  step=%.2fmm\r\n", step_mm);

            ret = motorMoveAndWaitUntilStop(step_mm, dir);
            CHECK_ERROR(ret);

            ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.sensor_position);
            CHECK_ERROR(ret);

            g_measurement.water_measurement.water_level = g_measurement.debug_data.sensor_position;
            water_value = g_measurement.debug_data.sensor_position;

            /* 连续饱和/大偏差过久 -> 重新找水位 */
            if (lost_count >= WATER_FOLLOW_LOST_COUNT_MAX) {
                printf("水位跟随\t长时间饱和/大偏差，重新找水位\r\n");
                ret = SearchWaterLevel();
                CHECK_ERROR(ret);

                lost_count = 0;
                printf("水位跟随\t重新找水位完成，恢复跟随\r\n");
                goto RESTART_FOLLOW;
            }

            HAL_Delay(WATER_FOLLOW_SAMPLE_MS);
            CHECK_COMMAND_SWITCH(NO_ERROR);
            continue;
        }

        /* 两态 + 滞回判断 */
        if (cap >= th) {
            dir = MOTOR_DIRECTION_UP;
            diff = cap - th;
            printf("水位跟随\t状态=偏水  diff=%.1f -> 上行\r\n", diff);

        } else if (cap <= th_low) {
            dir = MOTOR_DIRECTION_DOWN;
            diff = th_low - cap;
            printf("水位跟随\t状态=偏空气 diff=%.1f -> 下行\r\n", diff);

        } else {
            /* 稳定区：清零 lost_count */
            if (lost_count != 0) {
                printf("水位跟随\t进入稳定区，lost清零(%u->0)\r\n", lost_count);
                lost_count = 0;
            }

            printf("水位跟随\t状态=稳定区(%.1f < cap < %.1f)，保持\r\n", th_low, th);

            g_measurement.water_measurement.water_level = g_measurement.debug_data.sensor_position;
            water_value = g_measurement.debug_data.sensor_position;

            HAL_Delay(WATER_FOLLOW_SAMPLE_MS);
            CHECK_COMMAND_SWITCH(NO_ERROR);
            continue;
        }

        /* 步长选择 */
        if (diff > 80.0f) {
            step_mm = WATER_FOLLOW_STEP_BIG_MM;
        } else if (diff > 30.0f) {
            step_mm = WATER_FOLLOW_STEP_MED_MM;
        } else {
            step_mm = WATER_FOLLOW_STEP_SMALL_MM;
        }

        /* 大偏差计数：只有在“确实大偏差”时计数，否则慢慢衰减/清零 */
        if (diff >= WATER_FOLLOW_LOST_DIFF_BIG) {
            lost_count++;
        } else {
            if (lost_count > 0) lost_count--;
        }

        printf("水位跟随\t执行移动 dir=%s  step=%.2fmm  lost=%u\r\n",
               (dir == MOTOR_DIRECTION_UP) ? "UP" : "DOWN",
               step_mm,
               lost_count);

        ret = motorMoveAndWaitUntilStop(step_mm, dir);
        CHECK_ERROR(ret);

        ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        g_measurement.water_measurement.water_level = g_measurement.debug_data.sensor_position;
        water_value = g_measurement.debug_data.sensor_position;

        printf("水位跟随\t完成移动 pos=%.1fmm\r\n",
               g_measurement.debug_data.sensor_position / 10.0f);

        /* 连续大偏差过久 -> 重新找水位 */
        if (lost_count >= WATER_FOLLOW_LOST_COUNT_MAX) {
            printf("水位跟随\t长时间大偏差，重新找水位\r\n");
            ret = SearchWaterLevel();
            CHECK_ERROR(ret);

            lost_count = 0;
            printf("水位跟随\t重新找水位完成，恢复跟随\r\n");
            goto RESTART_FOLLOW;
        }

        HAL_Delay(WATER_FOLLOW_SAMPLE_MS);

        /* 状态切换退出 */
        CHECK_COMMAND_SWITCH(NO_ERROR);
    }
}

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
#define WATER_FOLLOW_STEP_MED_MM       (1.0f)    /* 中等偏差 */
#define WATER_FOLLOW_STEP_BIG_MM       (5.0f)    /* 大偏差/饱和时快速拉回 */
#define WATER_CAP_SAT_LIMIT            (9999.0f)/* 认为“饱和/无穷大”的阈值，用于保护 */
#define WATER_FOLLOW_LOST_DIFF_BIG       (80.0f)   /* 认为偏差很大 */
#define WATER_FOLLOW_LOST_COUNT_MAX      (20)      /* 连续大偏差次数阈值：20次*500ms=10s */
/* -------------------- 全局变量 -------------------- */
/* 存储最终确定的水位位置（以 sensor_position 记录） */
int32_t water_value = -100000000; // 初始值设为无效

/* -------------------- 函数原型 -------------------- */
static int SearchWaterRough(void);
static int SearchWaterPrecise(void);

static inline float WaterCapRawToFloat(uint32_t raw)
{
    return raw / 1000.0f;
}

static inline void WaterLevelSyncFromCable(void)
{
    int32_t lvl = g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;
    g_measurement.water_measurement.water_level = lvl;
    water_value = lvl;
}

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
    printf("水位测量\t水位滞后阈值：%lu\r\n", g_deviceParams.water_cap_hysteresis);
    /* -------------------- 初始位置调整（避让） -------------------- */
    if (g_measurement.debug_data.cable_length > 2000)
    {
        ret = motorMoveAndWaitUntilStopWithSpeed(WATER_INIT_UP_MM, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100());
        CHECK_ERROR(ret);
        printf("水位测量\t上行完成\r\n");
    }

    printf("水位测量\t初始位置：%ld\r\n", g_measurement.debug_data.sensor_position);

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
                printf("水位测量\t零点附近仍在水区，无法避让(pos=%.1fmm)\r\n",
                       g_measurement.debug_data.sensor_position / 10.0f);
                RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
            }

            /* 距离零点 > 100mm：上行 100mm */
            ret = motorMoveAndWaitUntilStopWithSpeed(UP_STEP_MM, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100());
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
    g_measurement.water_measurement.water_level = g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;

    printf("水位测量\t水位：%ld mm\r\n", g_measurement.water_measurement.water_level);

    return NO_ERROR;
}

static int SearchWaterRough(void)
{
    uint32_t ret;
    uint8_t  water_state = NORMAL;

    MotorLostStep_Init();// 重置丢步检测计数器
    /* 持续下探直到检测到 WATER */
    while (1)
    {
    	ret = motorMove_downWithSpeed(motorGetDefaultSpeedX100());  // 启动电机向下运动
    	CHECK_ERROR(ret); // 检查上行是否成功

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
        water_value = g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;
        return NO_ERROR;
    }
    else
    {
        printf("水位测量\t粗找水位未成功，准备回退重试\r\n");

        ret = motorMoveAndWaitUntilStopWithSpeed(WATER_FAIL_RECOVER_UP_MM, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100());
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
        ret = motorMoveAndWaitUntilStopWithSpeed(WATER_INIT_UP_MM, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100());
        CHECK_ERROR(ret);
        printf("水位测量\t上行完成\r\n");
    }

    MotorLostStep_Init();// 重置丢步检测计数器
    while (1)
    {
        speed_x100 = motorGetDefaultSpeedX100();
        if (g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length - water_value < WATER_V2_SLOWDOWN_TH)
        {
            speed_x100 = 4;
        }
        else if (g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length - water_value < WATER_V1_SLOWDOWN_TH)
        {
            speed_x100 = 40;
        }

    	ret = motorMove_downWithSpeed(speed_x100);  // 启动电机向下运动
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

        ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.sensor_position);
        CHECK_ERROR(ret);

        printf("水位测量\t精确寻找水位\t{传感器位置}%.1f\t速度(0.01m/min)\t%lu\t", (float)(g_measurement.debug_data.sensor_position) / 10.0f, (unsigned long)g_measurement.debug_data.motor_speed);
    }

    ret = motorQuickStop();
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

    printf(" Zero capacitance = %.1f\r\n", cap);
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

    printf("Oil capacitance = %.1f\r\n", cap);
    return NO_ERROR;
}

uint32_t check_water_status(uint8_t *water_state)
{
    uint32_t ret;
    float    cap = 0.0f;
    float    zero;
    float    th;

    if (water_state == NULL) {
        return PARAM_ERROR;
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
    printf("[WaterChk] cap=%.1f  zero=%.1f  th=%.1f  cap%sTH  -> %s\r\n",
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
        g_measurement.device_status.device_state == STATE_FOLLOW_WATER_OVER)
    {
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

    printf("快速跟随\t对齐: lvl_tgt=%.1fmm cable_now=%.1fmm cable_tgt=%.1fmm delta=%+.1fmm\r\n",
           lvl_target_01mm / 10.0f,
           cable_now_01mm / 10.0f,
           cable_target_01mm / 10.0f,
           delta_01mm / 10.0f);

    uint32_t dir = (delta_01mm >= 0) ? MOTOR_DIRECTION_DOWN : MOTOR_DIRECTION_UP;
    float move_mm = abs_delta_01mm / 10.0f;

    printf("快速跟随\t对齐: dir=%s move=%.1fmm\r\n",
           (dir == MOTOR_DIRECTION_DOWN) ? "DOWN" : "UP",
           move_mm);

    ret = motorMoveAndWaitUntilStopWithSpeed(move_mm, dir, motorGetDefaultSpeedX100());
    if (ret != NO_ERROR) return ret;

    return NO_ERROR;
}

/**
 * @brief 液面附近快速跟随（按“连续调用 motorMove_upWithSpeed/motorMove_downWithSpeed 直到状态翻转”的结构）
 *
 * 逻辑：
 *  - 若当前在水里(WATER)：持续上行直到变为 NORMAL（提出水面/到油里）
 *  - 若当前不在水里(NORMAL)：持续下行直到变为 WATER（进入水里）
 *  - 如此循环往复，实现“贴着液面”快速跟随
 *
 * 依赖：
 *  - motorMove_upWithSpeed()/motorMove_downWithSpeed(): 每次调用推动继续运动（你现有粗找就是这样用的）
 *  - check_water_status(): 返回 WATER / NORMAL
 *  - Motor_CheckLostStep_AutoTiming(): 丢步检测（可选但建议保留）
 * 退出条件：
 *   连续 60s 内 water_level 波动（max-min）不超过 50mm -> 认为稳定找到水位，退出
 */
uint32_t FindWaterLevel_FastByStateFlip_StableExit(void)
{
    uint32_t ret;
    uint8_t  water_state = NORMAL;

    /* ===== 稳定判定参数 ===== */
    const uint32_t STABLE_WIN_MS    = 60000u; /* 60s */

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
    printf("水位测量\t水位滞后阈值：%lu\r\n", g_deviceParams.water_cap_hysteresis);
    printf("快速跟随\t开始(稳定判定：60s内波动<=50mm退出)\r\n");

    /* 初始状态 */
    ret = check_water_status(&water_state);
    CHECK_ERROR(ret);

    while (1)
    {
        /* ===================== 段 A：在水里 -> 连续上行直到出水(NORMAL) ===================== */
        if (water_state == WATER)
        {
            printf("快速跟随\t当前=WATER -> 连续上行直到 NORMAL\r\n");
            MotorLostStep_Init();

            while (1)
            {
                if (g_measurement.debug_data.sensor_position <= ZERO_NEAR_TH)
                {
                    printf("快速跟随\t零点附近仍在水区，停止(pos=%.1fmm)\r\n",
                           g_measurement.debug_data.sensor_position / 10.0f);
                    RETURN_ERROR(MEASUREMENT_WATERLEVEL_LOW);
                }

                ret = motorMove_upWithSpeed(motorGetDefaultSpeedX100());
                CHECK_ERROR(ret);

                ret = check_water_status(&water_state);
                CHECK_ERROR(ret);

                if (water_state == NORMAL) {
                    break;
                }

                ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.sensor_position);
                CHECK_ERROR(ret);

                CHECK_COMMAND_SWITCH(NO_ERROR);
            }
        }
        /* ===================== 段 B：不在水里 -> 连续下行直到进水(WATER) ===================== */
        else
        {
            printf("快速跟随\t当前=NORMAL -> 连续下行直到 WATER\r\n");
            MotorLostStep_Init();

            while (1)
            {
                ret = motorMove_downWithSpeed(motorGetDefaultSpeedX100());
                CHECK_ERROR(ret);

                ret = check_water_status(&water_state);
                CHECK_ERROR(ret);

                if (water_state == WATER) {
                    break;
                }

                ret = Motor_CheckLostStep_AutoTiming(g_measurement.debug_data.sensor_position);
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
            printf("快速跟随\tflip#1 lvl=%.1fmm -> 缓存(等待第二次翻转后才开始更新/判稳)\r\n",
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
        g_measurement.water_measurement.water_level = lvl_avg;
        water_value = lvl_avg;

        printf("快速跟随\tflip lvl1=%.1f  lvl2=%.1f  avg=%.1fmm\r\n",
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
                printf("快速跟随\t波动超限 -> 重置稳定窗口(min=max=%.1fmm)\r\n", lvl / 10.0f);
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

                if (elapsed >= STABLE_WIN_MS)
                {
                    printf("快速跟随\t稳定满足：连续60s内波动<=阈值 -> 退出\r\n");
                    ret = motorQuickStop();
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
 * - 步进式运动（motorMoveAndWaitUntilStopWithSpeed）
 * - 带滞回，避免界面抖动
 * - 带自恢复机制，避免长期卡死在错误区域
 */
uint32_t FollowWaterLevel_fast(void)
{
    uint32_t ret;

    /* -------------------- 电容相关变量 -------------------- */
    float cap = 0.0f;         /* 当前读取到的水位电容值 */
    float air_cap;            /* 空气中的基准电容（零点电容） */
    float th;                 /* 水位判定上阈值（进入水区） */
    float th_low;             /* 水位判定下阈值（离开水区，滞回） */

    /* -------------------- 控制与计算变量 -------------------- */
    float diff;               /* 当前电容偏差量（相对于阈值） */
    float step_mm;            /* 本次运动步长（mm） */
    uint32_t dir;             /* 本次运动方向（UP / DOWN） */

    /* -------------------- 状态记忆与异常检测 -------------------- */
    static float last_cap = 0.0f; /* 上一次电容值，用于判断电容是否“卡死” */
    uint16_t lost_count = 0;      /* 连续异常计数器 */

    /* -------------------- 阈值计算 --------------------
     * 所有参数统一使用浮点计算，避免单位混乱
     * water_cap_threshold / water_cap_hysteresis
     * 在参数中以 ×1000 形式保存，这里统一还原
     */
    air_cap = g_measurement.water_measurement.zero_capacitance;
    th_low      = air_cap + WaterCapRawToFloat(g_deviceParams.water_cap_threshold) - WaterCapRawToFloat(g_deviceParams.water_cap_hysteresis);
    th  = air_cap + WaterCapRawToFloat(g_deviceParams.water_cap_threshold) + WaterCapRawToFloat(g_deviceParams.water_cap_hysteresis);

    printf("水位跟随\t开始\r\n");
    printf("水位跟随\t空气电容=%.1f  目标阈值上限=%.1f  滞回下限=%.1f\r\n",
           air_cap, th, th_low);

    /* ============================ 主循环 ============================ */
    while (1)
    {
        /* ---------- 1. 读取当前水位电容 ---------- */
        ret = Sensor_ReadWaterCapacitance(&cap);
        if (ret != NO_ERROR)
        {
            printf("水位跟随\t读取电容失败 ret=0x%lX\r\n", ret);
            return ret;
        }

        /* 计算电容变化量（用于“是否卡死”判断） */
        float cap_delta = fabsf(cap - last_cap);
        last_cap = cap;

        /* 保存到测量结构体，供其他模块/调试使用 */
        g_measurement.water_measurement.current_capacitance = cap;

        printf("水位跟随\tpos=%.1fmm  cap=%.1f\r\n",
               g_measurement.debug_data.sensor_position / 10.0f,
               cap);

        /* ---------- 2. 基于双阈值 + 滞回的状态判断 ---------- */
        if (cap >= th)
        {
            /* 电容高于上阈值：探头偏水 -> 需要上行 */
            dir  = MOTOR_DIRECTION_UP;
            diff = cap - th;
            printf("水位跟随\t状态=偏水  diff=%.1f -> 上行\r\n", diff);
        }
        else if (cap <= th_low)
        {
            /* 电容低于滞回下限：探头偏空气 -> 需要下行 */
            dir  = MOTOR_DIRECTION_DOWN;
            diff = th_low - cap;
            printf("水位跟随\t状态=偏空气 diff=%.1f -> 下行\r\n", diff);
        }
        else
        {
            /* ---------- 稳定区 ----------
             * 电容位于 [th_low, th] 之间
             * 认为已经贴近液面，不进行任何运动
             */
            if (lost_count != 0)
            {
                printf("水位跟随\t进入稳定区，lost清零(%u->0)\r\n", lost_count);
                lost_count = 0;
            }

            printf("水位跟随\t状态=稳定区(%.1f < cap < %.1f)，保持\r\n",
                   th_low, th);

            /* 稳态下更新一次水位值 */
            g_measurement.water_measurement.water_level =
                g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;
            water_value =
                g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;

            HAL_Delay(WATER_FOLLOW_SAMPLE_MS);
            CHECK_COMMAND_SWITCH(NO_ERROR);
            continue;
        }

        /* ---------- 3. 根据偏差大小选择运动步长 ----------
         * 偏差越大，说明离目标液面越远，允许使用更大的步长
         */
        if (diff > 0.8f * WaterCapRawToFloat(g_deviceParams.water_cap_threshold))
        {
            step_mm = WATER_FOLLOW_STEP_BIG_MM;
        }
        else if (diff > 0.3f * WaterCapRawToFloat(g_deviceParams.water_cap_threshold))
        {
            step_mm = WATER_FOLLOW_STEP_MED_MM;
        }
        else
        {
            step_mm = WATER_FOLLOW_STEP_SMALL_MM;
        }

        printf("水位跟随\t执行移动 dir=%s  step=%.2fmm  lost=%u\r\n",
               (dir == MOTOR_DIRECTION_UP) ? "UP" : "DOWN",
               step_mm,
               lost_count);

        /* ---------- 4. 执行步进运动（阻塞等待完成） ---------- */
        ret = motorMoveAndWaitUntilStopWithSpeed(step_mm, dir, motorGetDefaultSpeedX100());
        CHECK_ERROR(ret);

        /* 运动完成后，按当前设备状态更新水位 */
        UpdateWaterLevelIfValid();

        printf("水位跟随\t完成移动 pos=%.1fmm\r\n",
               g_measurement.debug_data.sensor_position / 10.0f);

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
            if (diff > 0.8f * th_span)
            {
                if (lost_count < 0xFFFFu)
                    lost_count++;
            }
            else
            {
                lost_count = 0;
            }

            printf("水位跟随\tlost判定 diff=%.1f th=%.1f capΔ=%.1f -> lost=%u\r\n",
                   diff, th_span, cap_delta, lost_count);
        }

        /* ---------- 6. 连续异常过多：触发重新找水位 ---------- */
        if (lost_count >= 5)
        {
            printf("水位跟随\t长时间大偏差，重新找水位\r\n");
            ret = FindWaterLevel_FastByStateFlip_StableExit();
            CHECK_ERROR(ret);

            lost_count = 0;
            printf("水位跟随\t重新找水位完成，恢复跟随\r\n");
        }

        /* ---------- 7. 采样周期延时 + 命令切换检测 ---------- */
        HAL_Delay(WATER_FOLLOW_SAMPLE_MS);
        CHECK_COMMAND_SWITCH(NO_ERROR);
    }
}

uint32_t FollowWaterLevel(void)
{
    uint32_t ret;
    float    cap = 0.0f;
    float    air_cap;
    float    th;
    float    th_low;
    float    diff;
    float    step_mm;
    uint32_t dir;

    uint16_t lost_count = 0;   /* 连续大偏差计数 */

RESTART_FOLLOW:
    air_cap = g_measurement.water_measurement.zero_capacitance;
    th      = air_cap + WaterCapRawToFloat(g_deviceParams.water_cap_threshold);
    th_low  = th - WaterCapRawToFloat(g_deviceParams.water_cap_hysteresis);

    printf("水位跟随\t开始\r\n");
    printf("水位跟随\t空气电容=%.1f  目标阈值=%.1f  滞回下限=%.1f\r\n",
           air_cap, th, th_low);

    while (1)
    {
        /* 读取电容 */
        ret = Sensor_ReadWaterCapacitance(&cap);
        if (ret != NO_ERROR) {
            printf("水位跟随\t读取电容失败 ret=0x%lX\r\n", ret);
            return ret;
        }

        g_measurement.water_measurement.current_capacitance = cap;

        printf("水位跟随\tpos=%.1fmm  cap=%.1f\r\n",
               g_measurement.debug_data.sensor_position / 10.0f,
               cap);

        /* 饱和保护：饱和也算“跟不上”，累计 lost_count */
        if (cap > WATER_CAP_SAT_LIMIT)
        {
            lost_count++;
            printf("水位跟随\t电容饱和(cap=%.1f) lost=%u\r\n", cap, lost_count);

            /* 饱和时先强制上行拉回 */
            dir     = MOTOR_DIRECTION_UP;
            step_mm = WATER_FOLLOW_STEP_BIG_MM;

            printf("水位跟随\t执行移动 dir=UP  step=%.2fmm\r\n", step_mm);

            ret = motorMoveAndWaitUntilStopWithSpeed(step_mm, dir, motorGetDefaultSpeedX100());
            CHECK_ERROR(ret);

            WaterLevelSyncFromCable();

            /* 连续饱和/大偏差过久 -> 重新找水位 */
            if (lost_count >= WATER_FOLLOW_LOST_COUNT_MAX)
            {
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
        if (cap >= th)
        {
            dir  = MOTOR_DIRECTION_UP;
            diff = cap - th;
            printf("水位跟随\t状态=偏水  diff=%.1f -> 上行\r\n", diff);
        }
        else if (cap <= th_low)
        {
            dir  = MOTOR_DIRECTION_DOWN;
            diff = th_low - cap;
            printf("水位跟随\t状态=偏空气 diff=%.1f -> 下行\r\n", diff);
        }
        else
        {
            /* 稳定区：清零 lost_count */
            if (lost_count != 0) {
                printf("水位跟随\t进入稳定区，lost清零(%u->0)\r\n", lost_count);
                lost_count = 0;
            }

            printf("水位跟随\t状态=稳定区(%.1f < cap < %.1f)，保持\r\n", th_low, th);

            WaterLevelSyncFromCable();

            HAL_Delay(WATER_FOLLOW_SAMPLE_MS);
            CHECK_COMMAND_SWITCH(NO_ERROR);
            continue;
        }

        /* 步长选择 */
        if (diff > 0.8f * WaterCapRawToFloat(g_deviceParams.water_cap_threshold)) {
            step_mm = WATER_FOLLOW_STEP_BIG_MM;
        } else if (diff > 0.3f * WaterCapRawToFloat(g_deviceParams.water_cap_threshold)) {
            step_mm = WATER_FOLLOW_STEP_MED_MM;
        } else {
            step_mm = WATER_FOLLOW_STEP_SMALL_MM;
        }

        /* 大偏差计数：只有在“确实大偏差”时计数，否则慢慢衰减/清零 */
        if (diff >= WaterCapRawToFloat(g_deviceParams.water_cap_threshold)) {
            lost_count++;
        } else {
            if (lost_count > 0) lost_count--;
        }

        printf("水位跟随\t执行移动 dir=%s  step=%.2fmm  lost=%u\r\n",
               (dir == MOTOR_DIRECTION_UP) ? "UP" : "DOWN",
               step_mm,
               lost_count);

        ret = motorMoveAndWaitUntilStopWithSpeed(step_mm, dir, motorGetDefaultSpeedX100());
        CHECK_ERROR(ret);

        WaterLevelSyncFromCable();

        printf("水位跟随\t完成移动 pos=%.1fmm\r\n",
               g_measurement.debug_data.sensor_position / 10.0f);

        /* 连续大偏差过久 -> 重新找水位 */
        if (lost_count >= WATER_FOLLOW_LOST_COUNT_MAX)
        {
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

#define WATER_TANK_HEIGHT_EPSILON   (1)   /* 0.1mm，避免边界抖动，可按需保留 */

static uint32_t CorrectWaterTankHeightProcess(void)
{
    int32_t new_height;

    printf("水位标定\t开始\r\n");
    printf("水位标定\t当前位置 cable=%.1fmm  标定真值=%.1fmm\r\n",
           g_measurement.debug_data.cable_length / 10.0f,
           g_deviceParams.calibrateWaterLevel / 10.0f);

    /* 核心公式：water_tank_height = cable_length_at_water + calibrateWaterLevel */
    new_height = (int32_t)g_measurement.debug_data.cable_length
              + (int32_t)g_deviceParams.calibrateWaterLevel
              + WATER_TANK_HEIGHT_EPSILON;

    /* 合理性保护 */
    if (new_height <= 0 || new_height > 5000000) { /* 例：500m -> 5,000,000(0.1mm) */
        printf("水位标定\t计算得到 water_tank_height 非法：%ld(0.1mm)\r\n", new_height);
        RETURN_ERROR(PARAM_ERROR);
    }

    g_deviceParams.water_tank_height = new_height;
	g_measurement.water_measurement.water_level = g_deviceParams.water_tank_height - g_measurement.debug_data.cable_length;
    /* 标定完成后清零，防止重复触发 */
    g_deviceParams.calibrateWaterLevel = 0;

    save_device_params();

    printf("水位标定\t完成 water_tank_height=%.1fmm\r\n",
           g_deviceParams.water_tank_height / 10.0f);

    return NO_ERROR;
}
// 标定水位：修正 water_tank_height
 void CMD_CalibrateWaterLevel(void)
{
    uint32_t ret = NO_ERROR;

    MeasureStart();
    if(g_measurement.device_status.device_state ==STATE_FOLLOW_WATER_OVER)
    {
		printf("水位标定\t当前处于水位跟随状态\t直接修正液位\r\n");
		g_measurement.device_status.device_state = STATE_CALIBRATE_WATERING;
    }
    else
	{
    	g_measurement.device_status.device_state = STATE_CALIBRATE_WATERING;
		printf("水位标定\t当前状态需要重修寻找水位\r\n");

		/*
		 * 约定：
		 * g_deviceParams.calibrateWaterLevel != 0 代表“用户提供了真值水位高度”，可做一次点标定。
		 * 若 == 0：建议直接报参数错误（否则无法反推 water_tank_height）。
		 */
		if (g_deviceParams.calibrateWaterLevel == 0) {
			printf("水位标定\t未设置标定水位真值(calibrateWaterLevel=0)，无法标定\r\n");
			SET_ERROR(PARAM_ERROR);
		}

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

    g_measurement.device_status.device_state = STATE_CALIBRATE_WATER_OVER;

    return;
}




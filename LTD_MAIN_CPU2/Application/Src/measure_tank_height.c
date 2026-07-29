/*
 * measureTankHeight.c - 罐体高度测量模块
 *
 * 功能说明：
 *   该模块控制电机运动，通过扭力传感器检测罐底位置，实现罐体高度的精确测量。
 *   测量过程分为两个阶段：粗略搜索和精确搜索。
 *
 * 创建日期: Feb 27, 2025
 * 作者: 1
 */

#include "measure_tank_height.h"
#include "weight.h"
#include <stdio.h>
#include <stdlib.h>
#include "measure_zero.h"
#include "motor_ctrl.h"
#include "sensor.h"
#include "encoder.h"
#include "error_log.h"
/* 全局变量：存储最终确定的罐底位置（编码器计数值） */
int32_t bottom_value = -100000000; /* 初始值设为较大数值作为无效状态标识 */
/* 全局/参数区：由寄存器或本机参数配置 */

static GyroZeroRef g_gyro_zero_ref = {0};

#define BOTTOM_GYRO_REF_SAMPLE_COUNT      5U /* 罐底倾角基准采样次数。 */
#define BOTTOM_GYRO_REF_SAMPLE_DELAY_MS 300U /* 罐底倾角基准采样间隔，单位 ms。 */
#define BOTTOM_GYRO_REF_MAX_SPREAD_DEG  2.0f /* 罐底倾角基准允许的最大离散度，单位度。 */
#define BOTTOM_GYRO_REF_SAFE_LIFT_MM   100.0f /* 罐底倾角基准采样前的安全抬升距离，单位 mm。 */
#define BOTTOM_RELEASE_BEFORE_ROUGH_STEP_MM 100.0f /* 罐底粗找前每次释放距离，单位 mm。 */
#define BOTTOM_RELEASE_BEFORE_ROUGH_MAX_MM 1000.0f /* 罐底粗找前最大释放距离，单位 mm。 */
#define BOTTOM_RELEASE_BEFORE_ROUGH_DELAY_MS 500U /* 罐底释放动作后的等待时间，单位 ms。 */
#define BOTTOM_NEAR_SENSOR_POSITION_01MM 10000  /* 1m，单位0.1mm */
#define BOTTOM_NEAR_SPEED_X100           50U    /* 0.50m/min */
/* 函数原型声明 */
static int SearchBottomRough();   /* 粗略搜索罐底 */
static int SearchBottomPrecise(); /* 精确搜索罐底 */
static int32_t GetRealHeightCalibrationOffset(void);
static uint32_t ApplyRealHeightCalibration(uint32_t raw_real_height);
/**
 * @brief 采集陀螺仪零点参考平均值，用于罐底测量前的姿态基准确认。
 * @param tag 打印标签，区分调用阶段。
 * @param allow_first_sample_fallback 允许首帧有效数据作为兜底基准的标志。
 * @return NO_ERROR 表示参考值采集完成，其他值表示传感器读取失败。
 */
static uint32_t CaptureGyroZeroRefAverage(const char *tag, uint8_t allow_first_sample_fallback);
static uint32_t EnsureGyroZeroRefForBottomMeasurement(void);
static uint32_t EnsureBottomReleasedBeforeRoughSearch(void);
static uint32_t Bottom_GetMaxCableLength01mm(void);
static uint32_t Bottom_CheckMaxCableLength(void);
static uint32_t Bottom_ApplyNearSensorSpeedLimit(uint32_t speed_x100);
static uint32_t BuildTankHeightFromCableLength(int32_t cable_length_01mm);
/**
 * @brief 执行罐高测量中的 Bottom_GetMaxCableLength01mm 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t Bottom_GetMaxCableLength01mm(void)
{
    uint64_t max_cable_length = (uint64_t)g_deviceParams.tankHeight +
                                (uint64_t)g_deviceParams.maxDownDistance;

    if (max_cable_length > (uint64_t)UINT32_MAX) {
        return UINT32_MAX;
    }

    return (uint32_t)max_cable_length;
}

/**
 * @brief 检查罐高测量中的 Bottom_CheckMaxCableLength 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t Bottom_CheckMaxCableLength(void)
{
    uint32_t max_cable_length = Bottom_GetMaxCableLength01mm();
    int32_t cable_length = g_measurement.debug_data.cable_length;

    if ((cable_length > 0) && ((uint32_t)cable_length > max_cable_length)) {
        printf("罐底测量\t超过最大下行位置仍未识别到罐底 | 尺带=%.1fmm | 上限=%.1fmm | 罐高=%.1fmm | 最大下行=%.1fmm\r\n",
               (double)cable_length * 0.1,
               (double)max_cable_length * 0.1,
               (double)g_deviceParams.tankHeight * 0.1,
               (double)g_deviceParams.maxDownDistance * 0.1);
        return MEASUREMENT_WEIGHT_DOWN_FAIL;
    }

    return NO_ERROR;
}

/**
 * @brief 执行罐高测量中的 Bottom_ApplyNearSensorSpeedLimit 逻辑。
 *
 * @param speed_x100 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t Bottom_ApplyNearSensorSpeedLimit(uint32_t speed_x100)
{
    if ((g_measurement.debug_data.sensor_position < BOTTOM_NEAR_SENSOR_POSITION_01MM) &&
        (speed_x100 > BOTTOM_NEAR_SPEED_X100)) {
        printf("罐底测量\t传感器位置低于1m，降速到0.50m/min | 传感器位置=%.1fmm | 原速度=%.2fm/min\r\n",
               (double)g_measurement.debug_data.sensor_position * 0.1,
               (double)speed_x100 / 100.0);
        return BOTTOM_NEAR_SPEED_X100;
    }

    return speed_x100;
}

/**
 * @brief 执行罐高测量中的 BuildTankHeightFromCableLength 逻辑。
 *
 * @param cable_length_01mm 数据长度。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t BuildTankHeightFromCableLength(int32_t cable_length_01mm)
{
    if (cable_length_01mm < 0) {
        cable_length_01mm = 0;
    }

    int64_t tank_height_01mm = (int64_t)cable_length_01mm +
                               (int64_t)(int32_t)g_deviceParams.liquid_sensor_distance_diff;

    if (tank_height_01mm < 0) {
        tank_height_01mm = 0;
    }

    if (tank_height_01mm > (int64_t)UINT32_MAX) {
        tank_height_01mm = (int64_t)UINT32_MAX;
    }

    return (uint32_t)tank_height_01mm;
}
/**
 * @brief 执行罐高测量中的 BuildBottomCableLengthFromTankHeight 逻辑。
 *
 * @param tank_height_01mm 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t BuildBottomCableLengthFromTankHeight(uint32_t tank_height_01mm)
{
    int64_t cable_length_01mm = (int64_t)tank_height_01mm -
                                (int64_t)(int32_t)g_deviceParams.liquid_sensor_distance_diff;

    if (cable_length_01mm < 0) {
        cable_length_01mm = 0;
    }

    if (cable_length_01mm > (int64_t)UINT32_MAX) {
        cable_length_01mm = (int64_t)UINT32_MAX;
    }

    return (uint32_t)cable_length_01mm;
}
static void ApplyBottomEncoderCorrection(uint32_t fallback_tank_height);
/**
 * @brief 读取罐高测量中的 GetBottomEncoderCorrectionTankHeight 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t GetBottomEncoderCorrectionTankHeight(uint32_t fallback_tank_height)
{
    if (g_deviceParams.bottom_encoder_correction_tank_height != 0U) {
        return g_deviceParams.bottom_encoder_correction_tank_height;
    }

    return fallback_tank_height;
}

/**
 * @brief 计算罐高测量中的 CalcAbsU32Diff 逻辑。
 *
 * @param a 业务参数。
 * @param b 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t CalcAbsU32Diff(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}
/**
 * @brief 读取罐高测量中的 GetRealHeightCalibrationOffset 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int32_t GetRealHeightCalibrationOffset(void)
{
    if ((g_deviceParams.initialTankHeight == 0U) ||
        (g_deviceParams.currentTankHeight == 0U))
    {
        return 0;
    }

    return (int32_t)g_deviceParams.currentTankHeight -
           (int32_t)g_deviceParams.initialTankHeight;
}

/**
 * @brief 执行罐高测量中的 ApplyRealHeightCalibration 逻辑。
 *
 * @param raw_real_height 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t ApplyRealHeightCalibration(uint32_t raw_real_height)
{
    int32_t corrected_real_height =
            (int32_t)raw_real_height + GetRealHeightCalibrationOffset();

    if (corrected_real_height < 0)
    {
        corrected_real_height = 0;
    }

    return (uint32_t)corrected_real_height;
}

/**
 * @brief 执行罐高测量中的 ApplyBottomEncoderCorrection 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void ApplyBottomEncoderCorrection(uint32_t fallback_tank_height)
{
    int32_t old_encoder_count;
    int32_t old_cable_length;
    int32_t target_cable_length;
    uint32_t correction_tank_height;
    uint32_t measured_real_height;
    uint32_t real_height_deviation;

    printf("罐底编码器修正开关=%lu\r\n", (unsigned long)g_deviceParams.bottom_encoder_correction_enable);

    if (g_deviceParams.bottom_encoder_correction_enable != BOTTOM_ENCODER_CORRECTION_ENABLE) {
        return;
    }

    correction_tank_height = GetBottomEncoderCorrectionTankHeight(fallback_tank_height);
    if (correction_tank_height == 0U) {
        printf("罐底编码器修正跳过：目标罐高为0，不修改编码值\r\n");
        return;
    }

    measured_real_height = g_measurement.height_measurement.current_real_height;
    real_height_deviation = CalcAbsU32Diff(measured_real_height, correction_tank_height);
    if ((g_deviceParams.maxTankHeightDeviation != 0U) &&
        (real_height_deviation > g_deviceParams.maxTankHeightDeviation)) {
        printf("罐底编码器修正跳过：实高偏差超限 | 测量实高=%.1fmm | 目标罐高=%.1fmm | 偏差=%.1fmm | 最大允许=%.1fmm\r\n",
               (double)measured_real_height * 0.1,
               (double)correction_tank_height * 0.1,
               (double)real_height_deviation * 0.1,
               (double)g_deviceParams.maxTankHeightDeviation * 0.1);
        return;
    }

    old_encoder_count = g_encoder_count;
    old_cable_length = encoder_get_cable_length_01mm();
    target_cable_length = (int32_t)BuildBottomCableLengthFromTankHeight(correction_tank_height);

    /* 罐底测量完成时探头位于罐底；不修改罐高，只把编码器当前尺带长度修正到罐高扣除探头距差后的值。 */
    encoder_set_cable_length_01mm(target_cable_length);

    printf("罐底编码器修正完成 | 原编码=%ld | 新编码=%ld | 原尺带：%.1fmm | 目标尺带：%.1fmm | 液位罐高=%.1fmm | 修正罐高=%.1fmm | 采用罐高=%.1fmm | 实高偏差=%.1fmm\r\n",
           (long)old_encoder_count,
           (long)g_encoder_count,
           (double)old_cable_length * 0.1,
           (double)target_cable_length * 0.1,
           (double)g_deviceParams.tankHeight * 0.1,
           (double)g_deviceParams.bottom_encoder_correction_tank_height * 0.1,
           (double)correction_tank_height * 0.1,
           (double)real_height_deviation * 0.1);
}
/**
 * @brief 罐底测量函数 - 执行完整的罐底搜索流程
 *        包含：粗找罐底（3次重试） + 两次精找（各3次重试）
 *        状态切换时不重试
 *
 * @return uint32_t 错误代码（NO_ERROR表示成功）
 */
uint32_t SearchBottom(void)
{
    uint32_t ret;
    uint32_t pending_calibration_tank_height = 0U;
    uint8_t pending_calibration_tank_height_valid = 0U;
    uint8_t try_times = 0;

    fault_info_init();  /* 清除故障信息 */
    printf("罐底测量\t开始\r\n");

    /* -------------------- 零点检查 -------------------- */
    if ((g_measurement.device_status.zero_point_status == 1)&&(g_deviceParams.error_auto_back_zero==1))
    {
        printf("罐底测量\t设备需要回零点\r\n");
        ret = SearchZero();       /* 执行回零点测量 */
        CHECK_ERROR(ret);         /* 检查是否成功 */
        printf("罐底测量\t回零点完成\r\n");
    }

    /* -------------------- 初始位置调整 -------------------- */
    if (g_measurement.debug_data.cable_length > 2000)
    {
        ret = MotorCtrl_MoveBlockingNoDetect(100.0, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);
        printf("罐底测量\t上行完成\r\n");
    }

    ret = EnsureGyroZeroRefForBottomMeasurement();
    CHECK_ERROR(ret);

    printf("罐底测量\t初始扭力：%d\r\n", weight_parament.stable_weight);

    ret = EnsureBottomReleasedBeforeRoughSearch();
    CHECK_ERROR(ret);

    /* ************** Rough bottom search retry ************** */
    try_times = 0;
    while (try_times < 3)
    {
        try_times++;

        fault_info_init();  /* 清除故障信息 */
        ret = SearchBottomRough();

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        CHECK_COMMAND_SWITCH(ret);

        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret != NO_ERROR)
        {
            /* 错误 阶段：错误重试 模块：测量 操作：粗找罐底 原因：搜索失败 尝试：try_times/3U 错误码：ret 错误名：ErrorLog_GetCodeName(ret) */
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_BOTTOM_ROUGH,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           3U,
                           ret);
            HAL_Delay(1000);
            continue;  /* 继续尝试 */
        }
        else
        {
            if (try_times > 1U)
            {
                /* 错误 阶段：重试成功 模块：测量 操作：粗找罐底 原因：恢复成功 尝试：try_times/3U */
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                                 ERROR_LOG_OP_SEARCH_BOTTOM_ROUGH,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)try_times,
                                 3U);
            }
            break;
        }
    }

    /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
    if (ret != NO_ERROR)
    {
        RETURN_ERROR(ret);
    }

    printf("罐底测量\t粗找罐底完成：实高：%ld mm", bottom_value); MotorCtrl_PrintPositionRefs(); printf("\r\n");

    /* ************** First precise bottom search retry ************** */
    try_times = 0;
    while (try_times < 3)
    {
        try_times++;

        ret = SearchBottomPrecise();

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret == NO_ERROR)
        {
            if (try_times > 1U)
            {
                /* 错误 阶段：重试成功 模块：测量 操作：精找罐底 原因：恢复成功 尝试：try_times/3U */
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                                 ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)try_times,
                                 3U);
            }
            break;
        }
        else
        {
            /* 错误 阶段：错误重试 模块：测量 操作：精找罐底 原因：搜索失败 尝试：try_times/3U 错误码：ret 错误名：ErrorLog_GetCodeName(ret) */
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           3U,
                           ret);
            HAL_Delay(1000);
        }
    }

    /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
    if (ret != NO_ERROR)
    {
        CHECK_ERROR(ret);
    }

    /* ************** Second precise bottom search retry ************** */
    try_times = 0;
    while (try_times < 3)
    {
        try_times++;

        ret = SearchBottomPrecise();

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret == NO_ERROR)
        {
            if (try_times > 1U)
            {
                /* 错误 阶段：重试成功 模块：测量 操作：精找罐底 原因：恢复成功 尝试：try_times/3U */
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                                 ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)try_times,
                                 3U);
            }
            break;
        }
        else
        {
            /* 错误 阶段：错误重试 模块：测量 操作：精找罐底 原因：搜索失败 尝试：try_times/3U 错误码：ret 错误名：ErrorLog_GetCodeName(ret) */
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           3U,
                           ret);
            HAL_Delay(1000);
        }
    }

    /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
    if (ret != NO_ERROR)
    {
        CHECK_ERROR(ret);
    }

    /* ************** Tank height record ************** */
    {
        int32_t bottom_cable_length_s =
                (bottom_value > 0) ? bottom_value
                                   : g_measurement.debug_data.cable_length;
        if (bottom_cable_length_s < 0) {
            printf("罐底测量\t尺带长度异常：%ld(0.1mm)，按0处理\r\n",
                   (long)bottom_cable_length_s);
            bottom_cable_length_s = 0;
        }
        uint32_t bottom_cable_length = (uint32_t)bottom_cable_length_s;
        uint32_t raw_real_height = BuildTankHeightFromCableLength(bottom_cable_length_s);
        uint32_t corrected_real_height =
                ApplyRealHeightCalibration(raw_real_height);

        g_measurement.height_measurement.current_real_height = corrected_real_height;
        printf("罐底测量\t尺带长度：%lu(0.1mm)\t探头距差=%ld(0.1mm)\t原始罐高=%lu(0.1mm)\t修正罐高=%lu(0.1mm)",
               (unsigned long)bottom_cable_length,
               (long)(int32_t)g_deviceParams.liquid_sensor_distance_diff,
               (unsigned long)raw_real_height,
               (unsigned long)corrected_real_height); MotorCtrl_PrintPositionRefs(); printf("\r\n");
        if(g_measurement.device_status.device_state == STATE_CALIBRATIONOILING)
        {
            /*
             * 先保留候选罐高；安全抬升失败时不得让标定结果在 RAM 中半生效。
             */
            pending_calibration_tank_height = raw_real_height;
            pending_calibration_tank_height_valid = 1U;
        }
    }
    /* 电机上行，完成流程 */
    /* 标定候选罐高尚未提交，显式传递以保持编码器修正的原有目标语义。 */
    ApplyBottomEncoderCorrection(
            (pending_calibration_tank_height_valid != 0U) ?
            pending_calibration_tank_height : g_deviceParams.tankHeight);

    ret = MotorCtrl_MoveBlockingNoDetect(100.0, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
    CHECK_ERROR(ret);

    if (pending_calibration_tank_height_valid != 0U)
    {
        g_measurement.height_measurement.calibrated_liquid_level =
                pending_calibration_tank_height;
        g_deviceParams.tankHeight = pending_calibration_tank_height;
        /* 安全抬升成功后再提交罐高并同步归一化 AO 量程。 */
        (void)normalize_ao_params_after_write();
        printf("罐底测量\t罐高标定完成，罐高=%ld(0.1mm)\r\n",
               g_deviceParams.tankHeight);
        MotorCtrl_RefreshPositionFromActiveSource();    /* 罐高变化后按当前记步源刷新当前位置 */
    }

    printf("罐底测量\t电机上行完成，流程结束\r\n");

    return NO_ERROR;
}
/**
 * @brief 粗略搜索罐底 - 快速下探直到检测到罐底
 *
 * @return int 错误代码（NO_ERROR表示成功）
 */
static int SearchBottomRough() {
    uint32_t ret;
    Weight_StateTypeDef bottom_status = NORMAL;
    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    /* 持续监控罐底检测状态，检测失败时返回错误码，避免误判为未到罐底后继续下探。 */
    while (1) {
        ret = check_bottom_status(&bottom_status);
        CHECK_ERROR(ret);
        if (bottom_status != NORMAL) {
            break;
        }

        ret = Bottom_CheckMaxCableLength();
        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            (void)MotorCtrl_QuickStop();
            RETURN_ERROR(ret);
        }

        uint32_t speed_x100 = Bottom_ApplyNearSensorSpeedLimit(MotorCtrl_GetDefaultSpeedX100());
        ret = MotorCtrl_MoveDown(speed_x100);  /* 启动电机向下运动 */
        CHECK_ERROR(ret); /* 检查下行是否成功 */

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
        CHECK_ERROR(ret); /* 检查丢步检测是否成功 */
        printf("罐底测量\t长距离寻找罐底\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position)/10.0); MotorCtrl_PrintPositionRefs(); printf("\t{扭力值}%d\t速度(0.01m/min)%lu\r\n", weight_parament.current_weight, (unsigned long)speed_x100);
    }
    ret = MotorCtrl_QuickStop(); /* 到达罐底后快速停止电机 */
    CHECK_ERROR(ret); /* 检查快速停止是否成功 */
    HAL_Delay(3000); /* 短暂等待 */
    /* 优化：检查是否真正到达罐底，检测失败时直接进入最终报错。 */
    printf("罐底测量\t确认粗找罐底位置\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position)/10.0); MotorCtrl_PrintPositionRefs(); printf("\r\n");
    ret = check_bottom_status(&bottom_status);
    CHECK_ERROR(ret);
    if (bottom_status == BOTTOM)
    {
        /* 记录首次检测到的罐底位置 */
        bottom_value = g_measurement.debug_data.cable_length;
        return NO_ERROR;
    }
    else {
        ret = MotorCtrl_MoveBlockingNoDetect(100.0, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);  /* 检查上行是否成功 */
        printf("罐底测量\t上行100mm\r\n");
        return MEASUREMENT_WEIGHT_DOWN_FAIL;
    }
}

/**
 * @brief 粗找前确认探头已经离开罐底。
 *
 * 如果进入找底流程时已经触底，先分段上行并复查触底状态，确保粗找从 NORMAL 状态开始。
 * 上行动作只用于离底，不做扭力碰撞检测；超过最大离底距离仍触底则返回找底失败。
 */
static uint32_t EnsureBottomReleasedBeforeRoughSearch(void)
{
    uint32_t ret;
    float lifted_mm = 0.0f;
    Weight_StateTypeDef bottom_status = NORMAL;

    while (lifted_mm <= BOTTOM_RELEASE_BEFORE_ROUGH_MAX_MM) {
        ret = check_bottom_status(&bottom_status);
        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }

        if (bottom_status == NORMAL) {
            if (lifted_mm > 0.0f) {
                printf("罐底测量\t粗找前离底完成，上行%.1fmm\r\n", (double)lifted_mm);
            }
            return NO_ERROR;
        }

        if ((lifted_mm + BOTTOM_RELEASE_BEFORE_ROUGH_STEP_MM) > BOTTOM_RELEASE_BEFORE_ROUGH_MAX_MM) {
            break;
        }

        printf("罐底测量\t粗找前已触底，上行%.1fmm后复查\r\n", (double)BOTTOM_RELEASE_BEFORE_ROUGH_STEP_MM);
        ret = MotorCtrl_MoveBlockingNoDetect(BOTTOM_RELEASE_BEFORE_ROUGH_STEP_MM,
                                             MOTOR_DIRECTION_UP,
                                             MotorCtrl_GetDefaultSpeedX100());
        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        lifted_mm += BOTTOM_RELEASE_BEFORE_ROUGH_STEP_MM;
        HAL_Delay(BOTTOM_RELEASE_BEFORE_ROUGH_DELAY_MS);
    }

    printf("罐底测量\t粗找前离底失败，累计上行%.1fmm后仍触底\r\n", (double)lifted_mm);
    return MEASUREMENT_BOTTOM_RELEASE_FAIL;
}
/**
 * @brief 精确搜索罐底 - 使用变速策略精确定位罐底
 *
 * @return int 错误代码（NO_ERROR表示成功）
 */
static int SearchBottomPrecise() {
    uint32_t ret;
    Weight_StateTypeDef bottom_status = NORMAL;
    uint32_t speed_x100;
    printf("罐底测量\t稳定扭力：%d\r\n", weight_parament.stable_weight);
    if (g_measurement.debug_data.cable_length > 2000)
    {
        ret = MotorCtrl_MoveBlockingNoDetect(200.0, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        CHECK_ERROR(ret);
        printf("罐底测量\t上行完成\r\n");
    }
    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    /* 持续监控罐底检测状态，检测失败时返回错误码，避免误判为未到罐底后继续下探。 */
    while (1) {
        ret = check_bottom_status(&bottom_status);
        CHECK_ERROR(ret);
        if (bottom_status == BOTTOM) {
            break;
        }

        ret = Bottom_CheckMaxCableLength();
        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            (void)MotorCtrl_QuickStop();
            RETURN_ERROR(ret);
        }

        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        if (bottom_value-g_measurement.debug_data.cable_length < 100) {
            speed_x100 = 10;
        }
        else if (bottom_value-g_measurement.debug_data.cable_length  < 300) {
            speed_x100 = 40;
        }
        else if (bottom_value-g_measurement.debug_data.cable_length  < 3000)  {
            speed_x100 = 100;
        }

        speed_x100 = Bottom_ApplyNearSensorSpeedLimit(speed_x100);
        ret = MotorCtrl_MoveDown(speed_x100);  /* 启动电机向下运动 */
        CHECK_ERROR(ret); /* 检查下行是否成功 */

        if (bottom_value-g_measurement.debug_data.cable_length < -1000)  {
            printf("罐底测量\tt精确寻找罐底未找到罐底\r\n");
            RETURN_ERROR(MEASUREMENT_WEIGHT_DOWN_FAIL); /* 如果编码器位置异常，返回错误 */
        }

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
        CHECK_ERROR(ret); /* 检查丢步检测是否成功 */
        printf("罐底测量\t精确寻找罐底\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position)/10.0f); MotorCtrl_PrintPositionRefs(); printf("\t速度(0.01m/min)\t%lu\r\n", (unsigned long)g_measurement.debug_data.motor_speed);
    }
    ret = MotorCtrl_QuickStop();
    CHECK_ERROR(ret); /* 检查快速停止是否成功 */
    /* 更新罐底位置并停止电机 */
    bottom_value = g_measurement.debug_data.cable_length;
    return NO_ERROR;
}
/**
 * @brief 采集陀螺仪零点参考平均值，用于罐底姿态基准。
 * @param tag 打印标签，区分采样场景。
 * @param allow_first_sample_fallback 允许首帧有效值作为兜底参考。
 * @return NO_ERROR 表示采样成功，其他值表示传感器读取失败。
 */
static uint32_t CaptureGyroZeroRefAverage(const char *tag, uint8_t allow_first_sample_fallback)
{
    uint32_t ret;
    float ax = 0.0f, ay = 0.0f;
    float first_x = 0.0f, first_y = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f;
    float min_x = 0.0f, max_x = 0.0f;
    float min_y = 0.0f, max_y = 0.0f;

    for (uint32_t i = 0; i < BOTTOM_GYRO_REF_SAMPLE_COUNT; i++) {
        ret = Sensor_ReadGyroAngle(&ax, &ay);
        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            g_gyro_zero_ref.valid = 0;
            return ret;
        }

        if (i == 0U) {
            first_x = ax;
            first_y = ay;
            min_x = max_x = ax;
            min_y = max_y = ay;
        } else {
            if (ax < min_x) min_x = ax;
            if (ax > max_x) max_x = ax;
            if (ay < min_y) min_y = ay;
            if (ay > max_y) max_y = ay;
        }

        sum_x += ax;
        sum_y += ay;
        printf("%s陀螺仪基准采样[%lu/%lu] | 角度X=%.2f | 角度Y=%.2f\r\n",
               tag,
               (unsigned long)(i + 1U),
               (unsigned long)BOTTOM_GYRO_REF_SAMPLE_COUNT,
               ax,
               ay);

        if ((i + 1U) < BOTTOM_GYRO_REF_SAMPLE_COUNT) {
            HAL_Delay(BOTTOM_GYRO_REF_SAMPLE_DELAY_MS);
        }
    }

    if (((max_x - min_x) > BOTTOM_GYRO_REF_MAX_SPREAD_DEG) ||
        ((max_y - min_y) > BOTTOM_GYRO_REF_MAX_SPREAD_DEG)) {
        if (allow_first_sample_fallback) {
            g_gyro_zero_ref.x0_deg = first_x;
            g_gyro_zero_ref.y0_deg = first_y;
            g_gyro_zero_ref.valid  = 1;
            printf("%s陀螺仪基准采样不稳定 | 差值X=%.2f | 差值Y=%.2f | 阈值=%.2f | 回退第一组 | 基准X=%.2f | 基准Y=%.2f\r\n",
                   tag,
                   max_x - min_x,
                   max_y - min_y,
                   BOTTOM_GYRO_REF_MAX_SPREAD_DEG,
                   g_gyro_zero_ref.x0_deg,
                   g_gyro_zero_ref.y0_deg);
            return NO_ERROR;
        }

        g_gyro_zero_ref.valid = 0;
        printf("%s陀螺仪基准不稳定 | 差值X=%.2f | 差值Y=%.2f | 阈值=%.2f\r\n",
               tag,
               max_x - min_x,
               max_y - min_y,
               BOTTOM_GYRO_REF_MAX_SPREAD_DEG);
        return MEASUREMENT_ZERO_REPEAT_FAIL;
    }

    g_gyro_zero_ref.x0_deg = sum_x / (float)BOTTOM_GYRO_REF_SAMPLE_COUNT;
    g_gyro_zero_ref.y0_deg = sum_y / (float)BOTTOM_GYRO_REF_SAMPLE_COUNT;
    g_gyro_zero_ref.valid  = 1;

    printf("%s陀螺仪基准建立完成 | 基准X=%.2f | 基准Y=%.2f\r\n",
           tag,
           g_gyro_zero_ref.x0_deg,
           g_gyro_zero_ref.y0_deg);
    return NO_ERROR;
}

/**
 * @brief 执行罐高测量中的 EnsureGyroZeroRefForBottomMeasurement 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t EnsureGyroZeroRefForBottomMeasurement(void)
{
    uint32_t ret;

    if (g_deviceParams.bottom_detect_mode != BOTTOM_DET_BY_GYRO) {
        return NO_ERROR;
    }

    if (g_gyro_zero_ref.valid) {
        return NO_ERROR;
    }

    printf("罐底测量\t角度找底基准无效，尝试在当前位置建立基准\r\n");
    ret = MotorCtrl_SlowStop();
    /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    if (g_measurement.debug_data.cable_length > 1000) {
        ret = MotorCtrl_MoveBlockingNoDetect(BOTTOM_GYRO_REF_SAFE_LIFT_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            return ret;
        }
        printf("罐底测量\t非回零基准采集前上行%.1fmm\r\n", (double)BOTTOM_GYRO_REF_SAFE_LIFT_MM);
    }

    HAL_Delay(1000);
    ret = CaptureGyroZeroRefAverage("非回零", 1U);
    /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("罐底测量\t非回零建立角度基准失败:0x%lX\r\n", (unsigned long)ret);
    }
    return ret;
}

/**
 * @brief 保存罐高测量中的 Bottom_SaveGyroZeroRef 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t Bottom_SaveGyroZeroRef(void)
{
    return CaptureGyroZeroRefAverage("零点", 0U);
}
/**
 * @brief 罐底状态检测（不锁存）。
 * @param status 输出检测状态，检测成功时写入 NORMAL 或 BOTTOM。
 * @return NO_ERROR 表示本次检测有效；其他错误码表示检测链路异常，需要上层停止探底并进入最终报错。
 *
 * 判定方式：
 *  - g_deviceParams.bottom_detect_mode == 0 : 扭力阈值
 *  - g_deviceParams.bottom_detect_mode != 0 : 陀螺仪角度阈值
 *
 * 阈值来源：
 *  - 扭力阈值：g_deviceParams.bottom_weight_threshold
 *  - 角度阈值：g_deviceParams.bottom_angle_threshold
 */
uint32_t check_bottom_status(Weight_StateTypeDef *status)
{
    if (status == NULL) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    *status = NORMAL;

    int32_t cur_weight    = (int32_t)weight_parament.current_weight;
    int32_t stable_weight = (int32_t)weight_parament.stable_weight;
    int32_t full_weight   = (int32_t)weight_parament.full_weight;

    int32_t diff = cur_weight - stable_weight;

    uint32_t motor_dir = (uint32_t)g_measurement.debug_data.motor_state;

    /* 尺带长度/位置（debug_data: 0.1mm） */
    float cable_mm  = g_measurement.debug_data.cable_length / 10.0f;
    float sensor_mm = g_measurement.debug_data.sensor_position / 10.0f;

    /* ==========================
     *  保护：零点附近不做罐底检测
     * ========================== */
    if (cable_mm < (float)g_deviceParams.weight_ignore_zone/10.0) {
        printf("扭力跳过 | 原因:零点保护 | 方向：%lu 当前扭力=%ld 稳定扭力=%ld 差值：%+ld 满载扭力=%ld 尺带长度：%.1f",
                (unsigned long)motor_dir,
                (long)cur_weight,
                (long)stable_weight,
                (long)diff,
                (long)full_weight,
                cable_mm);
        MotorCtrl_PrintPositionRefs();
        printf(" 传感器位置=%.1f | 零点保护区=%lu\r\n",
                sensor_mm,
                (unsigned long)g_deviceParams.weight_ignore_zone);
        return NO_ERROR;
    }
    /* -------- 方式1：扭力阈值（mode=0） -------- */
    if (g_deviceParams.bottom_detect_mode == 0) {

        int lower_limit = (int)g_deviceParams.bottom_weight_threshold;
        int current = (int)weight_parament.current_weight;

        Weight_StateTypeDef state = (current < lower_limit) ? BOTTOM : NORMAL;

        printf("罐底检测(扭力) | 当前:%d | 阈值:%d | 状态:%s | 尺带长度：%.1f",
                current,
                lower_limit,
                (state == BOTTOM) ? "到达罐底" : "正常",
                cable_mm);
        MotorCtrl_PrintPositionRefs();
        printf("\r\n");

        *status = state;
        return NO_ERROR;
    }

    /* -------- 方式2：陀螺仪角度变化（mode!=0，不锁存） -------- */
    if (!g_gyro_zero_ref.valid) {
        printf("罐底检测(陀螺仪) | 角度基准无效，停止本次探底\r\n");
        return MEASUREMENT_ZERO_REPEAT_FAIL;
    }

    float ax = 0.0f, ay = 0.0f;
    uint32_t ret = Sensor_ReadGyroAngle(&ax, &ay);
    /* 先处理异常边界，避免罐高测量状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        printf("罐底检测(陀螺仪) | 读取失败 错误码：%lu\r\n", (unsigned long)ret);
        return ret;
    }

    float dx = fabsf(ax - g_gyro_zero_ref.x0_deg);
    float dy = fabsf(fabsf(ay) - fabsf(g_gyro_zero_ref.y0_deg));
    float dsum = dx + dy;

    float th = (float)g_deviceParams.bottom_angle_threshold;

    Weight_StateTypeDef state = (dsum > th) ? BOTTOM : NORMAL;

    printf("罐底检测(扭力)%d (陀螺仪) | 角度X=%.2f 角度Y=%.2f | 基准X=%.2f 基准Y=%.2f | "
           "差值X=%.2f 差值Y=%.2f 合计=%.2f | 阈值=%.2f | 状态:%s | 尺带长度：%.1f",
            (int)weight_parament.current_weight,
            ax, ay,
            g_gyro_zero_ref.x0_deg, g_gyro_zero_ref.y0_deg,
            dx, dy, dsum,
            th,
            (state == BOTTOM) ? "到达罐底" : "正常",
            cable_mm);
    MotorCtrl_PrintPositionRefs();
    printf("\r\n");

    *status = state;
    return NO_ERROR;
}

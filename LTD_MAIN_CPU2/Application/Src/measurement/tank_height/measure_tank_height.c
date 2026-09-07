/**
 * @file measure_tank_height.c
 * @brief 罐底粗找、两次精找和罐高结果生成；称重按减重量判定，角度保持姿态判定。
 *
 * 探底流程通过扭力和陀螺仪状态确认罐底，成功后保存罐底尺带参考并同步电机丢步检测基准。
 */

#include "measure_tank_height.h"
#include "weight.h"
#include <stdio.h>
#include <stdlib.h>
#include "measure_zero.h"
#include "motor_ctrl.h"
#include "sensor_service.h"
#include "encoder.h"
#include "error_log.h"

/** 探底角度判定使用的 X/Y 零位参考。 */
typedef struct {
    float x0_deg;  /* 建立探底基准时的陀螺仪 X 轴角度，单位度。 */
    float y0_deg;  /* 建立探底基准时的陀螺仪 Y 轴角度，单位度。 */
    uint8_t valid; /* 零位参考有效标志；无效时禁止执行角度探底判定。 */
} GyroZeroRef;

/* 最近一次罐底对应的尺带长度，单位 0.1 mm；极小初值表示尚未建立。 */
static int32_t s_bottom_cable_length_01mm = -100000000;
/* 罐底陀螺仪零位参考，只允许本文件内的探底流程更新。 */
static GyroZeroRef g_gyro_zero_ref = {0};

/**
 * @brief 读取最近一次可信罐底对应的尺带长度。
 * @return 罐底尺带长度，单位 0.1 mm；尚未建立时返回负的内部无效值。
 */
int32_t TankHeight_GetBottomCableLength01mm(void)
{
    return s_bottom_cable_length_01mm;
}

/**
 * @brief 更新罐底尺带参考并同步给电机丢步检测模块。
 * @param cable_length_01mm 罐底对应的尺带长度，单位 0.1 mm。
 */
void TankHeight_SetBottomCableLength01mm(int32_t cable_length_01mm)
{
    s_bottom_cable_length_01mm = cable_length_01mm;
    /* 电机服务只接收通用位置参考，不反向读取 Application 测量状态。 */
    MotorCtrl_SetBottomReference01mm(cable_length_01mm);
}

#define BOTTOM_GYRO_REF_SAMPLE_COUNT      5U /* 罐底倾角基准采样次数。 */
#define BOTTOM_GYRO_REF_SAMPLE_DELAY_MS 300U /* 罐底倾角基准采样间隔，单位 ms。 */
#define BOTTOM_GYRO_REF_MAX_SPREAD_DEG  2.0f /* 罐底倾角基准允许的最大离散度，单位度。 */
#define BOTTOM_GYRO_REF_SAFE_LIFT_MM   100.0f /* 罐底倾角基准采样前的安全抬升距离，单位 mm。 */
#define BOTTOM_RELEASE_BEFORE_ROUGH_STEP_MM 100.0f /* 罐底粗找前每次释放距离，单位 mm。 */
#define BOTTOM_RELEASE_BEFORE_ROUGH_MAX_MM 1000.0f /* 罐底粗找前最大释放距离，单位 mm。 */
#define BOTTOM_RELEASE_BEFORE_ROUGH_DELAY_MS 500U /* 罐底释放动作后的等待时间，单位 ms。 */
#define BOTTOM_NEAR_SENSOR_POSITION_01MM 10000  /* 1m，单位0.1mm */
#define BOTTOM_NEAR_SPEED_X100           50U    /* 0.50m/min */
#define BOTTOM_WEIGHT_ENABLE_DISTANCE_01MM 3000 /* 称重候选在预计罐底上方300mm开始生效。 */
#define BOTTOM_WEIGHT_APPROACH_DISTANCE_01MM 3000 /* 参考点前300mm限速，与候选检测窗口独立。 */
#define BOTTOM_WEIGHT_PROBE_DISTANCE_01MM    300 /* 参考点前最后30mm低速，制动余量仍需台架验证。 */
#define BOTTOM_WEIGHT_REPEAT_TOLERANCE_01MM 100 /* 两次称重触发位置允许相差10mm。 */
#define BOTTOM_WEIGHT_PROBE_SPEED_X100       10U /* 称重末段探底使用0.10m/min。 */
#define BOTTOM_WEIGHT_RECOVERY_TIMEOUT_MS  1000U /* 释放后最多等待1秒恢复，期间不追加运动。 */
#define BOTTOM_WEIGHT_RECOVERY_POLL_MS       20U /* 等待时保留命令切换和故障检查响应。 */
#define BOTTOM_RELEASE_POSITION_TOLERANCE_01MM 50 /* 当前业务位置允许最多5mm释放不足，不限制向上多释放。 */
#define BOTTOM_GYRO_PRECISE_RELEASE_MM     200.0f /* 角度探底保持原有精找前上提距离。 */

static int32_t s_bottom_weight_expected_01mm = 0;
static int32_t s_bottom_weight_enable_01mm = 0;
static uint8_t s_bottom_weight_window_valid = 0U;
/* 基准和参数在本次粗找及两次精找期间固定，不能随卸载而向下漂移。 */
static int32_t s_bottom_weight_baseline = 0;
static int32_t s_bottom_weight_candidate_limit = 0;
static uint32_t s_bottom_weight_drop = 0U;
static uint8_t s_bottom_weight_baseline_valid = 0U;

/* 函数原型声明 */
static int SearchBottomRough(int32_t *trigger_position_01mm, int32_t *stop_position_01mm); /* 粗略搜索罐底 */
static int SearchBottomPrecise(int32_t reference_position_01mm,
                               int32_t *trigger_position_01mm,
                               int32_t *stop_position_01mm); /* 精确搜索罐底 */
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
static uint32_t Bottom_ApplyWeightApproachSpeedLimit(uint32_t speed_x100,
                                                    int32_t reference_position_01mm);
static uint32_t Bottom_PrepareWeightSearchWindow(void);
static uint32_t Bottom_ReleaseForNextProbe(int32_t trigger_position_01mm);
static uint32_t BuildTankHeightFromCableLength(int32_t cable_length_01mm);
static uint32_t check_bottom_status(Weight_StateTypeDef *status);
/**
 * @brief 计算探底流程允许使用的最大尺带长度，单位 0.1 mm。
 * @return 最大尺带长度，单位0.1 mm；加法超出uint32_t时饱和到UINT32_MAX。
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
 * @brief 检查当前尺带长度是否超过罐高约束的最大放带范围。
 * @return NO_ERROR 表示当前尺带长度未超过罐高约束；超出最大放带范围时返回 MEASUREMENT_WEIGHT_DOWN_FAIL。
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
 * @brief 接近传感器量程端点时限制探底运动速度。
 *
 * @param speed_x100 本次调试运动速度，单位 0.01 m/min；0 表示使用当前默认速度。
 * @return 返回探底实际采用速度，单位 0.01 m/min；接近传感器量程端点时不超过 BOTTOM_NEAR_SPEED_X100。
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
 * @brief 根据触底尺带长度和标定偏移计算罐高。
 *
 * @param cable_length_01mm 尺带长度定点值，单位 0.1 mm；函数结合浮子和安装偏移换算储罐高度并执行范围检查。
 * @return 罐高，单位0.1 mm；超出范围时钳位到0或UINT32_MAX。
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
 * @brief 根据罐高和标定偏移反算触底尺带长度。
 *
 * @param tank_height_01mm 参与换算的罐体高度，单位 0.1 mm。
 * @return 触底尺带长度，单位0.1 mm；超出范围时钳位到0或UINT32_MAX。
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
/**
 * @brief 建立本次称重探底的位置窗口。
 * @return NO_ERROR 表示预计罐底和检测起点有效；参数不足时返回探底失败。
 */
static uint32_t Bottom_PrepareWeightSearchWindow(void)
{
    int32_t expected_position_01mm;

    s_bottom_weight_window_valid = 0U;
    s_bottom_weight_baseline_valid = 0U;
    if (g_deviceParams.bottom_detect_mode != BOTTOM_DET_BY_WEIGHT) {
        return NO_ERROR;
    }

    s_bottom_weight_drop = g_deviceParams.bottom_weight_threshold;
    /* 原探底阈值直接作为减重量；0或超出有符号称重范围时禁止探底。 */
    if ((s_bottom_weight_drop == 0U) || (s_bottom_weight_drop > INT32_MAX)) {
        printf("罐底测量\t称重参数无效 | 绝对下限=%lu | 减重量=%lu\r\n",
               (unsigned long)WEIGHT_ABSOLUTE_MIN,
               (unsigned long)s_bottom_weight_drop);
        return PARAM_RANGE_ERROR;
    }

    if ((g_measurement.height_measurement.bottom_reference_valid != 0U) &&
        (s_bottom_cable_length_01mm > 0)) {
        expected_position_01mm = s_bottom_cable_length_01mm;
    } else {
        if (g_deviceParams.tankHeight == 0U) {
            printf("罐底测量\t称重探底缺少预计罐底位置，拒绝全行程高灵敏检测\r\n");
            return MEASUREMENT_WEIGHT_DOWN_FAIL;
        }
        expected_position_01mm =
                (int32_t)BuildBottomCableLengthFromTankHeight(g_deviceParams.tankHeight);
    }

    if (expected_position_01mm <= 0) {
        printf("罐底测量\t称重探底预计罐底位置无效 | 位置=%ld(0.1mm)\r\n",
               (long)expected_position_01mm);
        return MEASUREMENT_WEIGHT_DOWN_FAIL;
    }

    s_bottom_weight_expected_01mm = expected_position_01mm;
    s_bottom_weight_enable_01mm = expected_position_01mm -
                                  BOTTOM_WEIGHT_ENABLE_DISTANCE_01MM;
    if (s_bottom_weight_enable_01mm < 0) {
        s_bottom_weight_enable_01mm = 0;
    }
    s_bottom_weight_window_valid = 1U;

    printf("罐底测量\t称重探底窗口建立 | 预计=%.1fmm | 检测起点=%.1fmm | 提前=%.1fmm\r\n",
           (double)s_bottom_weight_expected_01mm * 0.1,
           (double)s_bottom_weight_enable_01mm * 0.1,
           (double)BOTTOM_WEIGHT_ENABLE_DISTANCE_01MM * 0.1);
    return NO_ERROR;
}

/**
 * @brief 粗找和精找按参考点分级降速，候选检测窗口不限制整段使用最低速度。
 * @param speed_x100 调用方准备使用的速度，单位0.01m/min。
 * @param reference_position_01mm 粗找用预计罐底，精找用上次触发位置，单位0.1mm。
 * @return 位置进入减速区时返回受限速度，否则返回原速度。
 */
static uint32_t Bottom_ApplyWeightApproachSpeedLimit(uint32_t speed_x100,
                                                    int32_t reference_position_01mm)
{
    int64_t distance_01mm;

    if ((g_deviceParams.bottom_detect_mode != BOTTOM_DET_BY_WEIGHT) ||
        (s_bottom_weight_window_valid == 0U)) {
        return speed_x100;
    }

    distance_01mm = (int64_t)reference_position_01mm -
                   (int64_t)g_measurement.debug_data.cable_length;

    /* 只向下限速，不能把低于探底档位的用户速度提高。 */
    if (distance_01mm <= BOTTOM_WEIGHT_PROBE_DISTANCE_01MM) {
        return (speed_x100 > BOTTOM_WEIGHT_PROBE_SPEED_X100) ?
               BOTTOM_WEIGHT_PROBE_SPEED_X100 : speed_x100;
    }
    if ((distance_01mm <= BOTTOM_WEIGHT_APPROACH_DISTANCE_01MM) &&
        (speed_x100 > BOTTOM_NEAR_SPEED_X100)) {
        return BOTTOM_NEAR_SPEED_X100;
    }
    return speed_x100;
}

/**
 * @brief 按首次触发位置和实际刹停位置计算上提距离，确保下一次探测从触发点上方开始。
 * @param trigger_position_01mm 本次称重候选首次触发位置，单位0.1mm。
 * @return NO_ERROR 表示到达释放目标；位置或运动异常时返回对应错误。
 */
static uint32_t Bottom_ReleaseWeightCandidate(int32_t trigger_position_01mm)
{
    uint32_t ret;
    int32_t stop_position_01mm = g_measurement.debug_data.cable_length;
    int32_t release_target_01mm;
    float lift_distance_mm;

    if (trigger_position_01mm <= BOTTOM_WEIGHT_RELEASE_MARGIN_01MM) {
        printf("罐底测量\t称重候选释放目标无效 | 触发=%.1fmm | 余量=%.1fmm\r\n",
               (double)trigger_position_01mm * 0.1,
               (double)BOTTOM_WEIGHT_RELEASE_MARGIN_01MM * 0.1);
        return MEASUREMENT_BOTTOM_RELEASE_FAIL;
    }

    release_target_01mm = trigger_position_01mm -
                          BOTTOM_WEIGHT_RELEASE_MARGIN_01MM;
    if (stop_position_01mm < 0) {
        return POSITION_DATA_INVALID;
    }
    /* 已到释放目标容差内时不再用微小补走消耗精找重试次数。 */
    if (stop_position_01mm <= release_target_01mm + BOTTOM_RELEASE_POSITION_TOLERANCE_01MM) {
        return NO_ERROR;
    }

    lift_distance_mm = (float)(stop_position_01mm - release_target_01mm) / 10.0f;
    printf("罐底测量\t称重候选释放 | 触发=%.1fmm | 停止=%.1fmm | 目标=%.1fmm | 上提=%.1fmm\r\n",
           (double)trigger_position_01mm * 0.1,
           (double)stop_position_01mm * 0.1,
           (double)release_target_01mm * 0.1,
           (double)lift_distance_mm);

    ret = MotorCtrl_MoveBlockingNoDetect(lift_distance_mm,
                                         MOTOR_DIRECTION_UP,
                                         MotorCtrl_GetDefaultSpeedX100());
    if (ret != NO_ERROR) {
        return ret;
    }

    if (g_measurement.debug_data.cable_length >
        (release_target_01mm + BOTTOM_RELEASE_POSITION_TOLERANCE_01MM)) {
        printf("罐底测量\t称重候选释放未到位 | 当前=%.1fmm | 目标=%.1fmm\r\n",
               (double)g_measurement.debug_data.cable_length * 0.1,
               (double)release_target_01mm * 0.1);
        return MEASUREMENT_BOTTOM_RELEASE_FAIL;
    }
    return NO_ERROR;
}

/**
 * @brief 在粗找或第一次精找后释放探头，为下一次探测建立起点。
 * @param trigger_position_01mm 本轮首次触发位置，单位0.1mm。
 * @return NO_ERROR 表示释放完成；其他值表示运动或位置异常。
 */
static uint32_t Bottom_ReleaseForNextProbe(int32_t trigger_position_01mm)
{
    if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
        return Bottom_ReleaseWeightCandidate(trigger_position_01mm);
    }

    if (g_measurement.debug_data.cable_length > 2000) {
        return MotorCtrl_MoveBlockingNoDetect(BOTTOM_GYRO_PRECISE_RELEASE_MM,
                                              MOTOR_DIRECTION_UP,
                                              MotorCtrl_GetDefaultSpeedX100());
    }
    return NO_ERROR;
}

/**
 * @brief 在触底结果中应用编码器罐高修正值。
 *
 * @param fallback_tank_height 兜底值高度。
 */
static void ApplyBottomEncoderCorrection(uint32_t fallback_tank_height);
/**
 * @brief 读取应用编码器修正后的探底罐高。
 *
 * @param fallback_tank_height 兜底值高度。
 * @return 返回可用的编码器修正罐高，单位 0.1 mm；修正值无效时返回调用方提供的 fallback_tank_height。
 */
static uint32_t GetBottomEncoderCorrectionTankHeight(uint32_t fallback_tank_height)
{
    if (g_deviceParams.bottom_encoder_correction_tank_height != 0U) {
        return g_deviceParams.bottom_encoder_correction_tank_height;
    }

    return fallback_tank_height;
}

/**
 * @brief 计算两个无符号 32 位位置值的绝对差。
 *
 * @param a 算法或比较使用的第一个输入值。
 * @param b 算法或比较使用的第二个输入值。
 * @return 返回两个无符号 32 位位置值的绝对差；有符号边界按函数内饱和规则处理。
 */
static uint32_t CalcAbsU32Diff(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}
/**
 * @brief 计算真实罐高标定相对探底罐高的偏移量。
 * @return 返回当前真实罐高与初始罐高之差，单位 0.1 mm；标定条件不成立时返回 0。
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
 * @brief 把真实罐高标定偏移应用到测量位置。
 *
 * @param raw_real_height 原始高度。
 * @return 返回叠加真实罐高标定偏移后的测量位置，单位 0.1 mm；负值和溢出分别钳位到 0 与 UINT32_MAX。
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
 * @brief 在触底结果中应用编码器罐高修正值。
 *
 * @param fallback_tank_height 兜底值高度。
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
    int32_t rough_trigger_01mm = 0;
    int32_t rough_stop_01mm = 0;
    int32_t precise_trigger_1_01mm = 0;
    int32_t precise_stop_1_01mm = 0;
    int32_t precise_trigger_2_01mm = 0;
    int32_t precise_stop_2_01mm = 0;

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

    ret = Bottom_PrepareWeightSearchWindow();
    /* 先读取旧参考建立窗口，再撤销本轮有效标志；参数失败时也不得保留本轮成功状态。 */
    g_measurement.height_measurement.bottom_reference_valid = 0U;
    CHECK_ERROR(ret);

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
        ret = SearchBottomRough(&rough_trigger_01mm, &rough_stop_01mm);

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        CHECK_COMMAND_SWITCH(ret);

        /* 称重只重试未找到候选，入口/释放及真实保护错误立即结束本轮。 */
        if ((g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) &&
            (ret != NO_ERROR) && (ret != MEASUREMENT_WEIGHT_DOWN_FAIL)) {
            return ret;
        }

        if (ret != NO_ERROR)
        {
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
                ErrorLog_Recover(ERROR_LOG_MODULE_MEASURE,
                                 ERROR_LOG_OP_SEARCH_BOTTOM_ROUGH,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 (uint32_t)try_times,
                                 3U);
            }
            break;
        }
    }

    if (ret != NO_ERROR)
    {
        RETURN_ERROR(ret);
    }

    printf("罐底测量\t粗找罐底完成 | 触发=%.1fmm | 停止=%.1fmm",
           (double)rough_trigger_01mm * 0.1,
           (double)rough_stop_01mm * 0.1); MotorCtrl_PrintPositionRefs(); printf("\r\n");

    /* ************** First precise bottom search retry ************** */
    try_times = 0;
    while (try_times < 3)
    {
        try_times++;

        ret = SearchBottomPrecise(
                                  (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) ?
                                  rough_trigger_01mm : rough_stop_01mm,
                                  &precise_trigger_1_01mm,
                                  &precise_stop_1_01mm);

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        /* 不能通过重复上提、重新建基准绕过释放或真实保护失败。 */
        if ((g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) &&
            (ret != NO_ERROR) && (ret != MEASUREMENT_WEIGHT_DOWN_FAIL)) {
            return ret;
        }
        if (ret == NO_ERROR)
        {
            if (try_times > 1U)
            {
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
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           3U,
                           ret);
            HAL_Delay(1000);
        }
    }

    if (ret != NO_ERROR)
    {
        CHECK_ERROR(ret);
    }

    /* ************** Second precise bottom search retry ************** */
    try_times = 0;
    while (try_times < 3)
    {
        try_times++;

        ret = SearchBottomPrecise(
                                  (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) ?
                                  precise_trigger_1_01mm : precise_stop_1_01mm,
                                  &precise_trigger_2_01mm,
                                  &precise_stop_2_01mm);

        if (ret == STATE_SWITCH)
        {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }

        /* 第二次精找沿用同一错误分类，只有未找到候选可以重试。 */
        if ((g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) &&
            (ret != NO_ERROR) && (ret != MEASUREMENT_WEIGHT_DOWN_FAIL)) {
            return ret;
        }
        if (ret == NO_ERROR)
        {
            if (try_times > 1U)
            {
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
            ErrorLog_Retry(ERROR_LOG_MODULE_MEASURE,
                           ERROR_LOG_OP_SEARCH_BOTTOM_PRECISE,
                           ERROR_LOG_REASON_SEARCH_FAIL,
                           (uint32_t)try_times,
                           3U,
                           ret);
            HAL_Delay(1000);
        }
    }

    if (ret != NO_ERROR)
    {
        CHECK_ERROR(ret);
    }

    if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
        uint32_t repeat_diff_01mm;
        int32_t confirmed_bottom_01mm;

        repeat_diff_01mm = CalcAbsU32Diff((uint32_t)precise_trigger_1_01mm,
                                         (uint32_t)precise_trigger_2_01mm);
        if (repeat_diff_01mm > BOTTOM_WEIGHT_REPEAT_TOLERANCE_01MM) {
            printf("罐底测量\t两次称重触发位置不一致 | 第一次=%.1fmm | 第二次=%.1fmm | 差值=%.1fmm | 容差=%.1fmm\r\n",
                   (double)precise_trigger_1_01mm * 0.1,
                   (double)precise_trigger_2_01mm * 0.1,
                   (double)repeat_diff_01mm * 0.1,
                   (double)BOTTOM_WEIGHT_REPEAT_TOLERANCE_01MM * 0.1);
            ret = Bottom_ReleaseWeightCandidate(precise_trigger_2_01mm);
            CHECK_ERROR(ret);
            RETURN_ERROR(MEASUREMENT_WEIGHT_DOWN_FAIL);
        }

        confirmed_bottom_01mm =
                ((precise_trigger_1_01mm < precise_trigger_2_01mm) ?
                 precise_trigger_1_01mm : precise_trigger_2_01mm) +
                (int32_t)(repeat_diff_01mm / 2U);
        TankHeight_SetBottomCableLength01mm(confirmed_bottom_01mm);
        printf("罐底测量\t两次称重触发位置确认 | 第一次=%.1fmm | 第二次=%.1fmm | 罐底=%.1fmm\r\n",
               (double)precise_trigger_1_01mm * 0.1,
               (double)precise_trigger_2_01mm * 0.1,
               (double)confirmed_bottom_01mm * 0.1);
    } else {
        /* 角度探底保持原有语义，最终位置仍取第二次精找的停稳位置。 */
        TankHeight_SetBottomCableLength01mm(precise_stop_2_01mm);
    }

    /* ************** Tank height record ************** */
    {
        int32_t bottom_cable_length_s =
                (TankHeight_GetBottomCableLength01mm() > 0) ? TankHeight_GetBottomCableLength01mm()
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
    /* 编码器修正会平移当前坐标，称重触发参考必须同步平移才能计算同一释放目标。 */
    {
        int32_t before_correction_01mm = g_measurement.debug_data.cable_length;
        ApplyBottomEncoderCorrection(
            (pending_calibration_tank_height_valid != 0U) ?
            pending_calibration_tank_height : g_deviceParams.tankHeight);
        if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
            int64_t corrected_bottom_01mm;
            MotorCtrl_RefreshPositionFromActiveSource();
            corrected_bottom_01mm = (int64_t)TankHeight_GetBottomCableLength01mm() +
                    (int64_t)g_measurement.debug_data.cable_length - before_correction_01mm;
            if ((corrected_bottom_01mm <= BOTTOM_WEIGHT_RELEASE_MARGIN_01MM) ||
                (corrected_bottom_01mm > INT32_MAX)) {
                RETURN_ERROR(MEASUREMENT_BOTTOM_RELEASE_FAIL);
            }
            TankHeight_SetBottomCableLength01mm((int32_t)corrected_bottom_01mm);
        }
    }

    /* 最终释放也校验到位，统一以两次确认的罐底参考加释放余量收尾。 */
    if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
        ret = Bottom_ReleaseWeightCandidate(TankHeight_GetBottomCableLength01mm());
    } else {
        ret = MotorCtrl_MoveBlockingNoDetect((float)BOTTOM_GYRO_RELEASE_MARGIN_01MM / 10.0f,
                                             MOTOR_DIRECTION_UP,
                                             MotorCtrl_GetDefaultSpeedX100());
    }
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

    g_measurement.height_measurement.bottom_reference_valid = 1U;
    s_bottom_weight_window_valid = 0U;
    printf("罐底测量\t电机上行完成，流程结束\r\n");

    return NO_ERROR;
}
/**
 * @brief 粗略搜索罐底 - 快速下探直到检测到罐底
 *
 * @return int 错误代码（NO_ERROR表示成功）
 */
static int SearchBottomRough(int32_t *trigger_position_01mm, int32_t *stop_position_01mm) {
    uint32_t ret;
    Weight_StateTypeDef bottom_status = NORMAL;

    if ((trigger_position_01mm == NULL) || (stop_position_01mm == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    *trigger_position_01mm = 0;
    *stop_position_01mm = 0;
    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    /* 持续监控罐底检测状态，检测失败时返回错误码，避免误判为未到罐底后继续下探。 */
    while (1) {
        ret = check_bottom_status(&bottom_status);
        CHECK_ERROR(ret);
        if (bottom_status != NORMAL) {
            break;
        }

        ret = Bottom_CheckMaxCableLength();
        if (ret != NO_ERROR) {
            (void)MotorCtrl_QuickStop();
            RETURN_ERROR(ret);
        }

        /* 称重门控仅限制减重量候选，保留窗口外相对碰撞保护；角度流程保持原行为。 */
        if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
            ret = CheckWeightCollision();
            CHECK_ERROR(ret);
        }

        uint32_t speed_x100 = Bottom_ApplyNearSensorSpeedLimit(MotorCtrl_GetDefaultSpeedX100());
        speed_x100 = Bottom_ApplyWeightApproachSpeedLimit(speed_x100,
                                                          s_bottom_weight_expected_01mm);
        ret = MotorCtrl_MoveDown(speed_x100);  /* 启动电机向下运动 */
        CHECK_ERROR(ret); /* 检查下行是否成功 */

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
        CHECK_ERROR(ret); /* 检查丢步检测是否成功 */
        printf("罐底测量\t长距离寻找罐底\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position)/10.0); MotorCtrl_PrintPositionRefs(); printf("\t{扭力值}%d\t速度(0.01m/min)%lu\r\n", weight_parament.current_weight, (unsigned long)speed_x100);
    }
    /* 必须在发出快停前锁存触发位置，停稳位置只用于计算刹车过行程。 */
    *trigger_position_01mm = g_measurement.debug_data.cable_length;
    ret = MotorCtrl_QuickStop(); /* 到达罐底后快速停止电机 */
    CHECK_ERROR(ret); /* 检查快速停止是否成功 */
    if (g_deviceParams.bottom_detect_mode != BOTTOM_DET_BY_WEIGHT) {
        /* 角度模式保留停稳复判；称重候选由后续两次位置确认，不等待重量回落。 */
        HAL_Delay(3000);
        ret = check_bottom_status(&bottom_status);
        CHECK_ERROR(ret);
        if (bottom_status != BOTTOM) {
            ret = MotorCtrl_MoveBlockingNoDetect(100.0f, MOTOR_DIRECTION_UP,
                                                 MotorCtrl_GetDefaultSpeedX100());
            CHECK_ERROR(ret);
            return MEASUREMENT_WEIGHT_DOWN_FAIL;
        }
    }
    *stop_position_01mm = g_measurement.debug_data.cable_length;
    /* 停稳称重可能回弹，仅记录位置；候选是否可信由后续两次精找位置一致性确认。 */
    printf("罐底测量\t粗找候选已锁存\t{传感器位置}%.1f", (float)(g_measurement.debug_data.sensor_position)/10.0); MotorCtrl_PrintPositionRefs(); printf("\r\n");
    return NO_ERROR;
}

/**
 * @brief 粗找前确认探头已经离开罐底。
 *
 * 如果进入找底流程时已经触底，先分段上行并复查触底状态，确保粗找从 NORMAL 状态开始。
 * 上行动作只用于离底，不做扭力碰撞检测；超过最大离底距离仍触底则返回找底失败。
 *
 * @return NO_ERROR 表示探头已确认离开罐底；退让运动或扭力读取失败时返回对应错误，允许次数内仍无法释放时返回 MEASUREMENT_BOTTOM_RELEASE_FAIL。
 */
static uint32_t EnsureBottomReleasedBeforeRoughSearch(void)
{
    uint32_t ret;
    float lifted_mm = 0.0f;
    Weight_StateTypeDef bottom_status = NORMAL;

    while (lifted_mm <= BOTTOM_RELEASE_BEFORE_ROUGH_MAX_MM) {
        ret = check_bottom_status(&bottom_status);
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
        if (ret != NO_ERROR) {
            return ret;
        }
        lifted_mm += BOTTOM_RELEASE_BEFORE_ROUGH_STEP_MM;
        HAL_Delay(BOTTOM_RELEASE_BEFORE_ROUGH_DELAY_MS);
    }

    printf("罐底测量\t粗找前离底失败，累计上行%.1fmm后仍触底\r\n", (double)lifted_mm);
    return MEASUREMENT_BOTTOM_RELEASE_FAIL;
}
/*
 * 精找上提完成后等待新帧恢复；旧缓存或单帧低值不立即判释放失败。
 * 只在停机状态调用，使用固定基准；通信/取消原样退出，超时不自动追加上提。
 */
static uint32_t Bottom_WaitForWeightRecovery(void)
{
    uint32_t sequence = Weight_GetFrameSequence();
    uint32_t started = HAL_GetTick();
    uint32_t ret;

    if (s_bottom_weight_baseline_valid == 0U) {
        return MEASUREMENT_BOTTOM_RELEASE_FAIL;
    }
    while (1) {
        uint32_t latest;
        ret = Weight_CheckCommunicationTimeout();
        CHECK_ERROR(ret);
        latest = Weight_GetFrameSequence();
        if (latest != sequence) {
            int32_t current = weight_parament.current_weight;
            sequence = latest;
            if (((int64_t)current > WEIGHT_ABSOLUTE_MIN) &&
                (current > s_bottom_weight_candidate_limit)) {
                return NO_ERROR;
            }
        }
        if ((uint32_t)(HAL_GetTick() - started) >= BOTTOM_WEIGHT_RECOVERY_TIMEOUT_MS) {
            printf("罐底测量\t释放后新帧称重未恢复 | 当前=%ld | 基准=%ld | 候选下限=%ld\r\n",
                   (long)weight_parament.current_weight, (long)s_bottom_weight_baseline,
                   (long)s_bottom_weight_candidate_limit);
            return MEASUREMENT_BOTTOM_RELEASE_FAIL;
        }
        HAL_Delay(BOTTOM_WEIGHT_RECOVERY_POLL_MS);
    }
}

/**
 * @brief 精确搜索罐底，上提后确认新称重帧恢复，再变速下探。
 * @return 错误代码，NO_ERROR表示成功。
 */
static int SearchBottomPrecise(int32_t reference_position_01mm,
                               int32_t *trigger_position_01mm,
                               int32_t *stop_position_01mm) {
    uint32_t ret;
    Weight_StateTypeDef bottom_status = NORMAL;
    uint32_t speed_x100;

    if ((reference_position_01mm <= 0) ||
        (trigger_position_01mm == NULL) ||
        (stop_position_01mm == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    *trigger_position_01mm = 0;
    *stop_position_01mm = 0;

    /* 每次精找入口都重新释放，保证错误重试不会从罐底下方继续下探。 */
    ret = Bottom_ReleaseForNextProbe(reference_position_01mm);
    CHECK_ERROR(ret);
    /* 释放后只接受新帧恢复，避免旧帧或停机瞬间低值导致立即失败。 */
    if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
        ret = Bottom_WaitForWeightRecovery();
        CHECK_ERROR(ret);
    }

    printf("罐底测量\t稳定扭力：%d | 精找参考=%.1fmm\r\n",
           weight_parament.stable_weight,
           (double)reference_position_01mm * 0.1);
    MotorCtrl_LostStepInit(); /* 重置丢步检测计数器 */
    /* 持续监控罐底检测状态，检测失败时返回错误码，避免误判为未到罐底后继续下探。 */
    while (1) {
        int32_t distance_to_reference_01mm;

        ret = check_bottom_status(&bottom_status);
        CHECK_ERROR(ret);
        if (bottom_status == BOTTOM) {
            break;
        }

        ret = Bottom_CheckMaxCableLength();
        if (ret != NO_ERROR) {
            (void)MotorCtrl_QuickStop();
            RETURN_ERROR(ret);
        }

        /* 称重门控仅限制减重量候选，保留窗口外相对碰撞保护；角度流程保持原行为。 */
        if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
            ret = CheckWeightCollision();
            CHECK_ERROR(ret);
        }

        distance_to_reference_01mm = reference_position_01mm -
                                     g_measurement.debug_data.cable_length;
        /* 精找按上一触发位置分级降速，避免释放后的整段行程都使用最低速度。 */
        speed_x100 = MotorCtrl_GetDefaultSpeedX100();
        if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
            speed_x100 = Bottom_ApplyWeightApproachSpeedLimit(speed_x100,
                                                              reference_position_01mm);
        }
        else if (distance_to_reference_01mm < 100) {
            speed_x100 = 10;
        }
        else if (distance_to_reference_01mm < 300) {
            speed_x100 = 40;
        }
        else if (distance_to_reference_01mm < 3000) {
            speed_x100 = 100;
        }

        /* 到达下探上限先退出，不能再下发一段向下运动后才报失败。 */
        if (distance_to_reference_01mm < -1000) {
            printf("罐底测量\t精确寻找罐底未找到罐底\r\n");
            RETURN_ERROR(MEASUREMENT_WEIGHT_DOWN_FAIL);
        }

        speed_x100 = Bottom_ApplyNearSensorSpeedLimit(speed_x100);
        ret = MotorCtrl_MoveDown(speed_x100);  /* 启动电机向下运动 */
        CHECK_ERROR(ret); /* 检查下行是否成功 */

        ret = MotorCtrl_CheckLostStepAutoTiming(g_measurement.debug_data.cable_length);
        CHECK_ERROR(ret); /* 检查丢步检测是否成功 */
        printf("罐底测量\t精确寻找罐底\t{传感器位置}%.1f",
               (float)(g_measurement.debug_data.sensor_position) / 10.0f);
        MotorCtrl_PrintPositionRefs();
        printf("\t速度(0.01m/min)\t%lu\r\n",
               (unsigned long)g_measurement.debug_data.motor_speed);
    }

    /* 必须在快停前锁存候选位置，避免把刹车过行程当作罐底。 */
    *trigger_position_01mm = g_measurement.debug_data.cable_length;
    ret = MotorCtrl_QuickStop();
    CHECK_ERROR(ret); /* 检查快速停止是否成功 */
    *stop_position_01mm = g_measurement.debug_data.cable_length;
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
        ret = SensorService_ReadGyroAngle(&ax, &ay);
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
 * @brief 探底前确保姿态零基准有效；必要时在允许位置重新采集并保存基准。
 * @return NO_ERROR 表示已有合法姿态零基准，或已重新采集并保存；其他值为位置不允许、陀螺仪读取、角度有效性或参数持久化错误。
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
    if (ret != NO_ERROR) {
        return ret;
    }

    if (g_measurement.debug_data.cable_length > 1000) {
        ret = MotorCtrl_MoveBlockingNoDetect(BOTTOM_GYRO_REF_SAFE_LIFT_MM, MOTOR_DIRECTION_UP, MotorCtrl_GetDefaultSpeedX100());
        if (ret != NO_ERROR) {
            return ret;
        }
        printf("罐底测量\t非回零基准采集前上行%.1fmm\r\n", (double)BOTTOM_GYRO_REF_SAFE_LIFT_MM);
    }

    HAL_Delay(1000);
    ret = CaptureGyroZeroRefAverage("非回零", 1U);
    if (ret != NO_ERROR) {
        printf("罐底测量\t非回零建立角度基准失败:0x%lX\r\n", (unsigned long)ret);
    }
    return ret;
}

/**
 * @brief 保存探底流程使用的陀螺仪零位参考角。
 * @return NO_ERROR 表示多次陀螺仪采样稳定且平均零位已保存；采样离散度超限返回 MEASUREMENT_ZERO_REPEAT_FAIL，传感器读取失败时原样返回其错误码。
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
 *  - g_deviceParams.bottom_detect_mode == 0 : 固定基准减去触底减重量
 *  - g_deviceParams.bottom_detect_mode != 0 : 陀螺仪角度阈值
 *
 * 阈值来源：
 *  - 绝对下限：固定500，仅用于基准/释放校验，不增加配置参数
 *  - 减重量：g_deviceParams.bottom_weight_threshold，达到或超过即为候选
 *  - 角度阈值：g_deviceParams.bottom_angle_threshold
 */
static uint32_t check_bottom_status(Weight_StateTypeDef *status)
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
    if (g_deviceParams.bottom_detect_mode == BOTTOM_DET_BY_WEIGHT) {
        uint32_t comm_ret = Weight_CheckCommunicationTimeout();

        if (comm_ret != NO_ERROR) {
            return comm_ret;
        }
        if ((s_bottom_weight_window_valid == 0U) ||
            (g_measurement.debug_data.cable_length < s_bottom_weight_enable_01mm)) {
            *status = NORMAL;
            return NO_ERROR;
        }
        int32_t current = weight_parament.current_weight;
        if (s_bottom_weight_baseline_valid == 0U) {
            int32_t baseline = weight_parament.stable_weight;
            int64_t candidate_limit = (int64_t)baseline - s_bottom_weight_drop;
            /* 使用已有低通称重作入口基准；绝对下限拦截已触底或明显卸载的入口。 */
            if (((int64_t)current <= WEIGHT_ABSOLUTE_MIN) ||
                (candidate_limit <= WEIGHT_ABSOLUTE_MIN) ||
                ((int64_t)current <= candidate_limit)) {
                printf("罐底测量\t入口称重不能建立基准 | 当前=%ld | 基准=%ld | 绝对下限=%lu | 减重量=%lu\r\n",
                       (long)current, (long)baseline,
                       (unsigned long)WEIGHT_ABSOLUTE_MIN,
                       (unsigned long)s_bottom_weight_drop);
                return MEASUREMENT_BOTTOM_RELEASE_FAIL;
            }
            s_bottom_weight_baseline = baseline;
            s_bottom_weight_candidate_limit = (int32_t)candidate_limit;
            s_bottom_weight_baseline_valid = 1U;
            printf("罐底测量\t称重基准锁定 | 基准=%ld | 减重量=%lu | 候选下限=%ld | 绝对下限=%lu\r\n",
                   (long)baseline, (unsigned long)s_bottom_weight_drop,
                   (long)s_bottom_weight_candidate_limit,
                   (unsigned long)WEIGHT_ABSOLUTE_MIN);
            /* 建基准这一拍不作为触底，候选必须来自后续减重。 */
            return NO_ERROR;
        }

        Weight_StateTypeDef state = (current <= s_bottom_weight_candidate_limit) ? BOTTOM : NORMAL;

        printf("罐底检测(减重) | 当前:%ld | 基准:%ld | 减重量:%lu | 候选下限:%ld | 状态:%s | 尺带长度：%.1f",
                (long)current, (long)s_bottom_weight_baseline,
                (unsigned long)s_bottom_weight_drop,
                (long)s_bottom_weight_candidate_limit,
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
    uint32_t ret = SensorService_ReadGyroAngle(&ax, &ay);
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

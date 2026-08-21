/**
 * @file measure_density.c
 * @brief 普通、国标、每米和区间密度分布的点阵生成与执行内核。
 *
 * MeasureDensity.c
 *
 * 文件功能：
 *   该文件实现“四种密度分布类测量模式”的统一流程：
 *     1) 普通分布测（SpredState=1）
 *     2) 国标测（SpredState=3，测后按标密差值规则过滤点）
 *     3) 每米测（SpredState=4）
 *     4) 区间测（SpredState=5）
 *
 * 业务流程概述（共同主链路）：
 *   A. 搜索液位（SearchOilLevel）
 *   B. 切换到密度测量模式（SensorService_EnableDensityMode）
 *   C. 按模式生成取点数组（单位：0.1mm）
 *   D. 按点位依次移动并单点测量（MotorCtrl_JogMoveToPosition + SinglePoint_ReadSensor）
 *   E. 计算平均值并输出（Print_DensitySpreadResult）
 *   F. 国标模式额外执行“按标密差值阈值过滤点，并重算平均值”
 *
 * 取点单位说明：
 *   - 例程取点使用 0.1mm（即 100um）作为整数单位，本文件保持一致。
 *   - 执行电机移动时，将 0.1mm 转换为 mm(float)：pos_mm = p01 / 10.0f
 *
 * 外部依赖（工程需提供）：
 *   - SearchOilLevel()
 *   - SensorService_EnableDensityMode()
 *   - MotorCtrl_JogMoveToPosition(float pos_mm, uint32_t speed_x100)
 *   - SinglePoint_ReadSensor(volatile DensityMeasurement *result)
 *   - g_measurement / g_deviceParams
 *   - DensityDistribution / DensityMeasurement
 *   - 错误码宏/定义：NO_ERROR、PARAM_RANGE_ERROR、SYSTEM_CALL_CONDITION_ERROR 等
 *   - 状态码/宏：SET_ERROR、CHECK_ERROR、CHECK_COMMAND_SWITCH、CMD_NONE 等
 *
 * 重要注意：
 *   - g_deviceParams.spreadPointHoverTime 的菜单单位为秒；执行延时前统一换算为毫秒。
 */

#include "measure_density.h"
#include "measure_density_internal.h"
#include "sensor_service.h"
#include "measure_oil_level.h"
#include "measure_tank_height.h"
#include "motor_ctrl.h"
#include "system_parameter.h"
#include "error_log.h"
#include "abortable_delay.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>


/* ===================== 前置声明 ===================== */
/* 国标过滤（按例程逻辑：Density20 差值阈值触发删点并搬移） */

/* ===================== 工具函数 ===================== */
/**
 * @brief 返回两个 0.1mm/计数类整数中的较大值，避免分布测点排序时重复写三目表达式。
 *
 * @param a 第一个有符号比较值；与 b 必须采用相同的长度、计数或定点单位。
 * @param b 第二个有符号比较值；与 a 必须采用相同的长度、计数或定点单位。
 * @return 返回 a 与 b 中数值较大的一项；两者相等时返回 b，结果沿用输入的长度、计数或定点单位。
 */
static inline int32_t i32_max(int32_t a, int32_t b) { return (a > b) ? a : b; }
/**
 * @brief 返回有符号位置差的绝对值，用于密度测量位置偏差判断。
 *
 * @param x 算法、坐标或比较使用的 X 值。
 * @return 返回有符号位置差的绝对值，用于密度测量位置偏差判断；有符号边界按函数内饱和规则处理。
 */
static inline int32_t i32_abs(int32_t x) { return (x >= 0) ? x : -x; }

/**
 * @brief 测量结果里的位置字段是无符号，上报前负位置统一按0处理。
 *
 * @param value_01mm 位置或距离值，单位 0.1 mm。
 * @param tag 用于区分诊断来源的只读标签文字。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
uint32_t DensityInternal_ValueToU01mmClamped(int32_t value_01mm, const char *tag)
{
    if (value_01mm < 0) {
        printf("密度测量\t%s为负：%ld(0.1mm)，按0上报\r\n",
               (tag != NULL) ? tag : "位置",
               (long)value_01mm);
        return 0U;
    }

    return (uint32_t)value_01mm;
}

/**
 * @brief 把当前电机位置换算并钳位为无符号 0.1 mm 坐标。
 * @return 返回完成边界钳位后的数值；输入低于下限时返回下限，高于上限时返回上限，区间内保持原值。
 */
uint32_t DensityInternal_CurrentPositionToU01mmClamped(void)
{
    return DensityInternal_ValueToU01mmClamped(g_measurement.debug_data.sensor_position, "测点位置");
}

/**
 * @brief 开始一轮分布点阵测量并关闭上一轮CPU2完成锁存。
 *
 * @details 调用场景：普通、国标、每米、区间、瓦锡兰和综合测量进入测量态前。
 * @note 关键约束：完成计数保持不变，CPU3继续使用上一份已确认快照。
 */
void DensityProfile_Begin(void)
{
    g_measurement.density_distribution.profile_complete_latched = 0U;
    __DMB();
    g_measurement.density_distribution.profile_source = PROFILE_SOURCE_NONE;
    g_measurement.density_distribution.profile_blocked_by_process = 0U;
    g_measurement.density_distribution.profile_temp_deviation_alarm = 0U;
    g_measurement.density_distribution.profile_density_deviation_alarm = 0U;
}

/**
 * @brief 把完整分布点阵、来源和完成锁存按同一代际发布。
 *
 * @details 调用场景：六种分布类测量取得完整临时候选结果后。
 * @note 关键约束：候选结构写完并执行屏障后才递增完成计数，CPU3以计数沿启动整阵复核。
 *
 * @param candidate 待校验或比较的候选值。该可写对象承载本轮完整密度分布、测点数和关联结果，校验通过后才发布到共享测量快照。
 * @param source 本轮密度剖面结果来源枚举；用于标记标准、国标、密度计或其它测量流程，供发布快照和后续显示选择。
 */
void DensityProfile_PublishResult(DensityDistribution *candidate, ProfileSource source)
{
    uint32_t previous_complete_counter;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    previous_complete_counter =
            g_measurement.density_distribution.profile_complete_counter;
    candidate->profile_complete_counter = previous_complete_counter;
    candidate->profile_complete_latched = 1U;
    candidate->profile_source = source;
    g_measurement.density_distribution = *candidate;
    __DMB();
    g_measurement.density_distribution.profile_complete_counter =
            previous_complete_counter + 1U;
    __DMB();
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 按 0.1 mm 单位打印密度分布取点数组。
 *
 * @param tag 用于区分诊断来源的只读标签文字。
 * @param p01 包含 n 个取点位置的只读数组，每个元素单位为 0.1 mm；日志同时打印原始整数和毫米值。
 * @param n 测点数组中参与打印或测量的有效点数。
 */
void DensityInternal_PrintPoints01mm(const char *tag, const int32_t *p01, uint32_t n)
{
    printf("%s 取点 点数=%lu: ", tag, (unsigned long)n);
    for (uint32_t i = 0; i < n; i++) {
        printf("%ld(%.1fmm) ", (long)p01[i], (double)p01[i] / 10.0);
    }
    printf("\r\n");
}

/**
 * @brief 通用执行器：按点位数组执行测量。
 *
 * 输入点位单位：0.1mm。
 * 输出：dist 写入测点列表与平均值。
 *
 * @param p01 按执行顺序排列的测点位置数组，元素单位为 0.1 mm。
 * @param n 测点数组中参与打印或测量的有效点数。
 * @param oil_level_01mm 油位值，单位 0.1 mm。
 * @param dist 密度分布测量结果对象。
 * @param dwell_time_s 测点到位后的稳定停留时间，单位 s。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */
static uint32_t Density_RunPoints01mmWithDwell(const int32_t *p01,
                                      uint32_t n,
                                      int32_t oil_level_01mm,
                                      DensityDistribution *dist,
                                      uint32_t dwell_time_s)
{
    if (!p01 || !dist) return SYSTEM_CALL_CONDITION_ERROR;
    if (n == 0 || n > MAX_MEASUREMENT_POINTS) return MEASUREMENT_DENSITY_PLAN_INVALID;

    memset(dist, 0, sizeof(*dist));

    uint32_t valid = 0;
    uint64_t sum_temp_raw = 0;
    uint64_t sum_dens_raw = 0;

    /* 悬停等待使用菜单单位秒，传入延时函数前换算为毫秒。 */
    uint32_t hover_time_s = dwell_time_s;
    uint32_t hover_ms = hover_time_s * 1000U;

    for (uint32_t i = 0; i < n; i++) {

        float pos_mm = (float)p01[i] / 10.0f;
        printf("分布测量 移动到位置 %.1f mm\r\n", pos_mm);

        uint32_t ret = MotorCtrl_JogMoveToPosition(pos_mm, MotorCtrl_GetDefaultSpeedX100());
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        if (ret != NO_ERROR) {
            printf("分布测量 电机移动失败: 位置=%.1fmm 错误码=%lu\r\n", pos_mm, (unsigned long)ret);
            return ret;
        }

        if (hover_ms > 0) {
            ret = AbortableDelay_CommandSwitch(hover_ms, 100U);
            if (ret == STATE_SWITCH) {
                /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
                return STATE_SWITCH;
            }
        }

        ret = DensityInternal_ReadSensorWithStableWindow(&dist->single_density_data[valid], hover_ms, NULL);
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        if (ret != NO_ERROR) {
            printf("分布测量 单点读取失败: 位置=%.1fmm 错误码=%lu\r\n", pos_mm, (unsigned long)ret);
            return ret;
        }

        sum_temp_raw += dist->single_density_data[valid].temperature;
        sum_dens_raw += dist->single_density_data[valid].density;
        valid++;

        /* 命令切换退出：用于上位机/按键打断当前过程 */
        if (HasEffectiveCommandSwitchRequest()) {
            printf("检测到命令切换请求，停止当前分布测量\r\n");
            return STATE_SWITCH;
        }

        if (valid >= MAX_MEASUREMENT_POINTS) break;
    }

    if (valid == 0) {
        printf("分布测量 未得到任何有效测点\r\n");
        return MEASUREMENT_DENSITY_NO_VALID_POINT;
    }

    /* 平均值（RAW 平均） */
    dist->measurement_points = valid;
    dist->Density_oil_level  = DensityInternal_ValueToU01mmClamped(oil_level_01mm, "分布测量液位"); /* 0.1mm */

    dist->average_temperature = (uint32_t)(sum_temp_raw / valid);
    dist->average_density     = (uint32_t)(sum_dens_raw / valid);

    /* 若需要严格计算标准密度/VCF/计重密度，应在单点测量或后处理中补齐 */
    dist->average_standard_density = dist->average_density;
    dist->average_vcf20            = 0;
    dist->average_weight_density   = dist->average_density;

    return NO_ERROR;
}

/**
 * @brief 按 0.1 mm 点位序列移动传感器并采集密度数据。
 *
 * @param p01 按执行顺序排列的测点位置数组，元素单位为 0.1 mm。
 * @param n 测点数组中参与打印或测量的有效点数。
 * @param oil_level_01mm 油位值，单位 0.1 mm。
 * @param dist 密度分布测量结果对象。
 * @return 返回整机错误码；NO_ERROR 表示全部点位采集完成，其他值透传点位、运动、传感器读取或命令切换失败。
 */
static uint32_t Density_RunPoints01mm(const int32_t *p01,
                                      uint32_t n,
                                      int32_t oil_level_01mm,
                                      DensityDistribution *dist)
{
    return Density_RunPoints01mmWithDwell(p01, n, oil_level_01mm, dist, g_deviceParams.spreadPointHoverTime);
}

/* =======================================================================
 * 取点逻辑：严格复刻例程
 * ======================================================================= */

/**
 * @brief 按液位、顶部限制、罐底盲区和最小点距生成普通密度分布测点。
 *
 * 关键点如下。
 * 可测下边界采用 floor = max(bottomLimit, blindZone)，即取配置下限和传感器盲区中的较大值。
 * distmin 最小 100mm（0.1mm=1000）；配置值更小时按 1000 个 0.1 mm 单位参与点数计算。
 * 有效高度 high_min = oil_level - topLimit - floor；液位、点数或有效高度不足时退化为一个液位中点，并保证该点不低于盲区。
 * 实际步距 dis 使用 high_min/(N-1) 等分（不是固定 distance）；最终点位按配置方向输出并裁剪到有效区间。
 *
 * @param oil_level_01mm 油位值，单位 0.1 mm。
 * @param out_p01 用于接收生成后测点位置的数组，元素单位为 0.1 mm。
 * @param out_n 用于返回本次生成的有效测点数量。
 * @return NO_ERROR 表示已生成合法普通分布测点；SYSTEM_CALL_CONDITION_ERROR
 *         表示输出数组或数量指针无效；MEASUREMENT_DENSITY_PLAN_INVALID 表示可测高度、点数或最小间距无法形成合法方案。
 */
static uint32_t BuildPoints_Spread_Exact(int32_t oil_level_01mm,
                                        int32_t *out_p01,
                                        uint32_t *out_n)
{
    if (!out_p01 || !out_n) return SYSTEM_CALL_CONDITION_ERROR;

    uint32_t NumOfPoints = 0;

    int32_t high     = oil_level_01mm;                               /* 0.1mm */
    int32_t top      = (int32_t)g_deviceParams.spreadTopLimit;       /* 0.1mm */
    int32_t floor    = (int32_t)g_deviceParams.spreadBottomLimit;    /* 0.1mm */
    int32_t fade0    = (int32_t)g_deviceParams.blindZone;            /* 0.1mm */

    uint32_t N_req   = g_deviceParams.spreadMeasurementCount;        /* 期望点数 */
    int32_t  distmin = (int32_t)g_deviceParams.spreadMeasurementDistance; /* 0.1mm */

    floor = i32_max(floor, fade0);

    /* 例程：distance>=1000(0.1mm)=100mm */
    if (distmin < 1000) distmin = 1000;

    if (high <= 0) return MEASUREMENT_DENSITY_PLAN_INVALID;
    if (N_req == 0) return MEASUREMENT_DENSITY_PLAN_INVALID;
    if (N_req > MAX_MEASUREMENT_POINTS) N_req = MAX_MEASUREMENT_POINTS;

    int32_t high_min = high - top - floor;

    /* 退化：液位不足、点数=1、有效高度不足 -> 只取 1 点（液位中点，且不低于盲区） */
    if (high <= distmin || N_req == 1 || high_min <= 0 || high_min < distmin) {
        NumOfPoints = 1;
        int32_t p = high / 2;
        if (p < fade0) p = fade0;
        out_p01[0] = p;
        *out_n = NumOfPoints;
        return NO_ERROR;
    }

    /* 例程：依据 high_min 和 distmin 计算可允许最大点数 */
    uint32_t num_measuring;
    if ((high_min % distmin) == 0) num_measuring = (uint32_t)(high_min / distmin) + 1;
    else                            num_measuring = (uint32_t)(high_min / distmin) + 2;
    if (num_measuring > 100) num_measuring = 100;

    NumOfPoints = N_req;
    if (NumOfPoints > num_measuring) NumOfPoints = num_measuring;
    if (NumOfPoints < 2) NumOfPoints = 2;
    if (NumOfPoints > MAX_MEASUREMENT_POINTS) NumOfPoints = MAX_MEASUREMENT_POINTS;

    /* 等分有效高度 */
    float dis = (float)high_min / (float)(NumOfPoints - 1);

    /* 0=上->下，1=下->上 */
    uint32_t order = g_deviceParams.spreadMeasurementOrder;

    for (uint32_t i = 0; i < NumOfPoints; i++) {
        float v;
        if (order == 0) v = (float)(high - top) - dis * (float)i;
        else            v = (float)(high - top) - dis * (float)(NumOfPoints - 1 - i);

        int32_t p = (int32_t)lroundf(v);

        /* 裁剪到 [floor, high-top] */
        if (p < floor) p = floor;
        if (p > (high - top)) p = (high - top);

        out_p01[i] = p;
    }

    *out_n = NumOfPoints;
    return NO_ERROR;
}

/**
 * @brief 按 GB/T 4575 的液位分段规则生成 1、3 或 5 个密度测点，并按配置确定测量顺序。
 *
 * 关键点如下。
 * 3m 以下：1 点（液位中点，且不低于盲区）。
 * 3m~4.5m：最多 3 点（5/6、1/2、1/6），若点位低于盲区则退化。
 * >4.5m：最多 5 点（1/6..5/6），若点位低于盲区则退化。
 * 任何理论点位低于 blindZone 时，从低端开始用盲区边界替代并减少有效点数，保证输出点阵不会进入传感器盲区。
 * 顺序按 spreadMeasurementOrder 决定（上->下 / 下->上）；调用方提供的数组至少需要容纳五个测点。
 *
 * @param high_01mm 当前测量区间的高端位置，单位 0.1 mm。
 * @param out_p01 用于接收生成后测点位置的数组，元素单位为 0.1 mm。
 * @param out_n 用于返回本次生成的有效测点数量。
 */
static void BuildPoints_GB4575_Exact(uint32_t high_01mm,
                                    int32_t *out_p01,
                                    uint32_t *out_n)
{
    uint32_t NumOfPoints = 0;
    int32_t  FadeZero = (int32_t)g_deviceParams.blindZone;

    if (high_01mm <= Spread_Gb_Onepiont_Posion) {
        NumOfPoints = 1;
        out_p01[0] = (int32_t)(high_01mm / 2);
        if (out_p01[0] < FadeZero) out_p01[0] = FadeZero;
    }
    else if (high_01mm <= Spread_Gb_Twopiont_Posion) {
        if (((int32_t)(high_01mm * 5 / 6)) < FadeZero) {
            NumOfPoints = 1;
            out_p01[0] = FadeZero;
        }
        else if (((int32_t)(high_01mm / 2)) < FadeZero) {
            NumOfPoints = 2;
            if (g_deviceParams.spreadMeasurementOrder == 0) {
                out_p01[0] = (int32_t)(high_01mm * 5 / 6);
                out_p01[1] = FadeZero;
            } else {
                out_p01[0] = FadeZero;
                out_p01[1] = (int32_t)(high_01mm * 5 / 6);
            }
        }
        else if (((int32_t)(high_01mm * 1 / 6)) < FadeZero) {
            NumOfPoints = 3;
            if (g_deviceParams.spreadMeasurementOrder == 0) {
                out_p01[0] = (int32_t)(high_01mm * 5 / 6);
                out_p01[1] = (int32_t)(high_01mm * 1 / 2);
                out_p01[2] = FadeZero;
            } else {
                out_p01[0] = FadeZero;
                out_p01[1] = (int32_t)(high_01mm * 1 / 2);
                out_p01[2] = (int32_t)(high_01mm * 5 / 6);
            }
        }
        else {
            NumOfPoints = 3;
            if (g_deviceParams.spreadMeasurementOrder == 0) {
                out_p01[0] = (int32_t)(high_01mm * 5 / 6);
                out_p01[1] = (int32_t)(high_01mm * 1 / 2);
                out_p01[2] = (int32_t)(high_01mm * 1 / 6);
            } else {
                out_p01[0] = (int32_t)(high_01mm * 1 / 6);
                out_p01[1] = (int32_t)(high_01mm * 1 / 2);
                out_p01[2] = (int32_t)(high_01mm * 5 / 6);
            }
        }
    }
    else {
        int32_t p1 = (int32_t)(high_01mm * 1 / 6);
        int32_t p2 = (int32_t)(high_01mm * 2 / 6);
        int32_t p3 = (int32_t)(high_01mm * 3 / 6);
        int32_t p4 = (int32_t)(high_01mm * 4 / 6);
        int32_t p5 = (int32_t)(high_01mm * 5 / 6);

        if (p5 < FadeZero) {
            NumOfPoints = 1;
            out_p01[0] = FadeZero;
        }
        else if (p4 < FadeZero) {
            NumOfPoints = 2;
            if (g_deviceParams.spreadMeasurementOrder == 0) {
                out_p01[0] = p5;
                out_p01[1] = FadeZero;
            } else {
                out_p01[0] = FadeZero;
                out_p01[1] = p5;
            }
        }
        else if (p3 < FadeZero) {
            NumOfPoints = 3;
            if (g_deviceParams.spreadMeasurementOrder == 0) {
                out_p01[0] = p5;
                out_p01[1] = p4;
                out_p01[2] = FadeZero;
            } else {
                out_p01[0] = FadeZero;
                out_p01[1] = p4;
                out_p01[2] = p5;
            }
        }
        else if (p2 < FadeZero) {
            NumOfPoints = 4;
            if (g_deviceParams.spreadMeasurementOrder == 0) {
                out_p01[0] = p5;
                out_p01[1] = p4;
                out_p01[2] = p3;
                out_p01[3] = FadeZero;
            } else {
                out_p01[0] = FadeZero;
                out_p01[1] = p3;
                out_p01[2] = p4;
                out_p01[3] = p5;
            }
        }
        else if (p1 < FadeZero) {
            NumOfPoints = 5;
            if (g_deviceParams.spreadMeasurementOrder == 0) {
                out_p01[0] = p5;
                out_p01[1] = p4;
                out_p01[2] = p3;
                out_p01[3] = p2;
                out_p01[4] = FadeZero;
            } else {
                out_p01[0] = FadeZero;
                out_p01[1] = p2;
                out_p01[2] = p3;
                out_p01[3] = p4;
                out_p01[4] = p5;
            }
        }
        else {
            NumOfPoints = 5;
            if (g_deviceParams.spreadMeasurementOrder == 0) {
                out_p01[0] = p5;
                out_p01[1] = p4;
                out_p01[2] = p3;
                out_p01[3] = p2;
                out_p01[4] = p1;
            } else {
                out_p01[0] = p1;
                out_p01[1] = p2;
                out_p01[2] = p3;
                out_p01[3] = p4;
                out_p01[4] = p5;
            }
        }
    }

    *out_n = NumOfPoints;
}

/**
 * @brief 在扣除顶部限制和罐底盲区后的有效区间内，按一米间距生成密度测点。
 *
 * 关键点如下。
 * floor = max(bottomLimit, blindZone)，即下边界取配置下限和传感器盲区中的较大值。
 * dis01 = oil_level - topLimit - floor，要求 >= 1m；按 0.1 mm 单位表示时至少为 10000。
 * 上->下：从 (high - (c_num+1)*1m) 逐米往下，必要时砍掉最后一个越界点。
 * 下->上：从 (c_num+1)*1m 逐米往上，直到超过 high-top。
 * 生成过程最多写入 MAX_MEASUREMENT_POINTS 个点；最终没有合法点位时返回方案无效，不向调用方报告部分结果。
 *
 * @param oil_level_01mm 油位值，单位 0.1 mm。
 * @param out_p01 用于接收生成后测点位置的数组，元素单位为 0.1 mm。
 * @param out_n 用于返回本次生成的有效测点数量。
 * @return NO_ERROR 表示已生成合法逐米测点；SYSTEM_CALL_CONDITION_ERROR
 *         表示输出数组或数量指针无效；MEASUREMENT_DENSITY_PLAN_INVALID 表示有效高度不足一米或生成点数无效。
 */
static uint32_t BuildPoints_Meter_Exact(int32_t oil_level_01mm,
                                       int32_t *out_p01,
                                       uint32_t *out_n)
{
    if (!out_p01 || !out_n) return SYSTEM_CALL_CONDITION_ERROR;

    int32_t high  = oil_level_01mm;
    int32_t top   = (int32_t)g_deviceParams.spreadTopLimit;
    int32_t floor = (int32_t)g_deviceParams.spreadBottomLimit;
    int32_t fade0 = (int32_t)g_deviceParams.blindZone;

    floor = i32_max(floor, fade0);

    int32_t meter = METER_STEP_01MM;
    int32_t dis01 = high - top - floor;
    if (dis01 < meter) {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }

    uint32_t NumMeter = 0;
    uint32_t mode = g_deviceParams.spreadMeasurementOrder; /* 0上->下 1下->上 */

    if (mode == 0) {
        int32_t c_num = top / meter;
        if (top == meter) c_num = 0;

        uint32_t dis = (uint32_t)(dis01 / meter);

        for (uint32_t i = 0; i <= dis && NumMeter < MAX_MEASUREMENT_POINTS; i++) {
            int32_t p = (high - (c_num + 1) * meter) - (int32_t)i * meter;
            out_p01[NumMeter++] = p;
        }

        if (NumMeter > 0) {
            int32_t last = out_p01[NumMeter - 1];
            if (last < floor || last < 0) {
                NumMeter--;
            }
        }
    } else {
        int32_t c_num = floor / meter;
        if (floor == meter) c_num = 0;

        int32_t high_edge = high - top;
        for (uint32_t i = 0; i < MAX_MEASUREMENT_POINTS; i++) {
            int32_t p = (c_num + (int32_t)i + 1) * meter;
            if (p > high_edge) break;
            out_p01[NumMeter++] = p;
        }
    }

    if (NumMeter == 0) return MEASUREMENT_DENSITY_PLAN_INVALID;

    *out_n = NumMeter;
    return NO_ERROR;
}

/**
 * @brief 校验用户配置的区间端点和点数，并在合法区间内按等差方式生成密度测点。
 *
 * 关键点如下。
 * 使用 upper/lower 两端点（0.1mm）：区间高端 high_a 由 oil_level 减去 intervalMeasurementTopLimit 得到，低端 high_b 直接使用
 * intervalMeasurementBottomLimit。
 * 点数使用 spreadMeasurementCount，并且不得超过 MAX_MEASUREMENT_POINTS 和调用方提供的 out_capacity；非法配置不得静默钳位或部分写入。
 * 端点合法性：high_b < high_a，high_a < tankHeight，high_b >= blindZone；同时要求 high_a 大于零。
 * 一个点时按测量方向选择高端或低端，两个点时直接输出两端；c_num>=3 时：等差取点，最后一点评端点，以消除整数除法累计误差。
 *
 * @param oil_level_01mm 油位值，单位 0.1 mm。
 * @param out_p01 用于接收生成后测点位置的数组，元素单位为 0.1 mm。
 * @param out_capacity out_p01 数组可写入的元素容量；配置点数超过该容量时函数返回方案无效且不写入部分点阵。
 * @param out_n 用于返回本次生成的有效测点数量。
 * @return NO_ERROR 表示已生成合法区间测点；SYSTEM_CALL_CONDITION_ERROR
 *         表示输出数组、容量或数量指针无效；MEASUREMENT_DENSITY_PLAN_INVALID 表示端点、罐高、盲区、点数或输出容量不满足方案约束。
 */
static uint32_t BuildPoints_Interval_Exact(int32_t oil_level_01mm,
                                           int32_t *out_p01,
                                           uint32_t out_capacity,
                                           uint32_t *out_n)
{
    if (!out_n) return SYSTEM_CALL_CONDITION_ERROR;
    *out_n = 0U;
    if (!out_p01) return SYSTEM_CALL_CONDITION_ERROR;

    uint32_t high_min;
    int32_t  high_a, high_b;
    uint32_t dis;
    uint32_t c_num;
    uint32_t i;

    high_min = (uint32_t)g_deviceParams.blindZone;

    /* 区间测参数：
     *   - 上限：距液面的距离（0.1mm） => 绝对点位 = oil_level - topLimit
     *   - 下限：距罐底的距离（0.1mm） => 绝对点位 = bottomLimit
     */
    high_a = oil_level_01mm - (int32_t)g_deviceParams.intervalMeasurementTopLimit;
    high_b = (int32_t)g_deviceParams.intervalMeasurementBottomLimit;

    if (high_b >= high_a) return MEASUREMENT_DENSITY_PLAN_INVALID;

    c_num = g_deviceParams.spreadMeasurementCount;
    /* 点数必须在调用方容量内，非法配置不得静默钳位或部分写入。 */
    if ((c_num == 0U) ||
        (c_num > MAX_MEASUREMENT_POINTS) ||
        (c_num > out_capacity)) {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }

    if (high_a >= (int32_t)g_deviceParams.tankHeight) {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    } else if (high_b < (int32_t)high_min) {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    } else if (high_a <= 0) {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }

    if (c_num == 1U) {
        out_p01[0] = (g_deviceParams.spreadMeasurementOrder == 0) ? high_a : high_b;
        *out_n = 1U;
        return NO_ERROR;
    }

    if (c_num == 2U) {
        if (g_deviceParams.spreadMeasurementOrder == 0) {
            out_p01[0] = high_a;
            out_p01[1] = high_b;
        } else {
            out_p01[0] = high_b;
            out_p01[1] = high_a;
        }
        *out_n = 2U;
        return NO_ERROR;
    }

    dis = (uint32_t)i32_abs(high_a - high_b) / (c_num - 1U);

    if (g_deviceParams.spreadMeasurementOrder == 0) {
        for (i = 0; i < c_num; i++) {
            out_p01[i] = high_a - (int32_t)(dis * i);
        }
        out_p01[c_num - 1] = high_b;
    } else {
        for (i = 0; i < c_num; i++) {
            out_p01[i] = high_b + (int32_t)(dis * i);
        }
        out_p01[c_num - 1] = high_a;
    }

    *out_n = c_num;
    return NO_ERROR;
}

/**
 * @brief 模式路由：取点 + 执行 + 国标后处理。
 *
 * @param mode 本次密度分布使用的 DensitySpreadModeId，决定取点算法、执行器和国标后处理路径。
 * @param out_dist 用于接收本次模式测量生成的完整密度分布结果。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
 */

uint32_t Density_MeasureByMode_Exact(DensitySpreadModeId mode, DensityDistribution *out_dist)
{
    if (!out_dist) return SYSTEM_CALL_CONDITION_ERROR;

    uint32_t ret;

    /* 1) 先液位搜索 */
    ret = SearchOilLevel();
    if (ret != NO_ERROR) {
        printf("密度测量\t液位搜索失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    /* 2) 切密度模式；模式切换失败时不能继续规划测点或读取旧模式数据。 */
    ret = SensorService_EnableDensityMode();
    if (ret != NO_ERROR) {
        printf("密度测量\t切换密度模式失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    int32_t oil_level_01mm = (int32_t)g_measurement.oil_measurement.oil_level; /* 0.1mm */

    int32_t points01[MAX_MEASUREMENT_POINTS];
    uint32_t n = 0;
    memset(points01, 0, sizeof(points01));

    /* 3) 按模式取点 */
    if (mode == DENS_MODE_SPREAD) {
        ret = BuildPoints_Spread_Exact(oil_level_01mm, points01, &n);
        if (ret != NO_ERROR) return ret;
        DensityInternal_PrintPoints01mm("普通分布测", points01, n);
    }
    else if (mode == DENS_MODE_GB) {
        BuildPoints_GB4575_Exact((uint32_t)oil_level_01mm, points01, &n);
        if (n == 0) return MEASUREMENT_DENSITY_PLAN_INVALID;
        DensityInternal_PrintPoints01mm("国标测", points01, n);
    }
    else if (mode == DENS_MODE_METER) {
        ret = BuildPoints_Meter_Exact(oil_level_01mm, points01, &n);
        if (ret != NO_ERROR) return ret;
        DensityInternal_PrintPoints01mm("每米测", points01, n);
    }
    else if (mode == DENS_MODE_INTERVAL) {
        ret = BuildPoints_Interval_Exact(
            oil_level_01mm,
            points01,
            (uint32_t)(sizeof(points01) / sizeof(points01[0])),
            &n);
        if (ret != NO_ERROR) return ret;
        DensityInternal_PrintPoints01mm("区间测", points01, n);
    }
    else {
        return MEASUREMENT_DENSITY_PLAN_INVALID;
    }

    /* 4) 统一执行测量 */
    ret = Density_RunPoints01mm(points01, n, oil_level_01mm, out_dist);
    if (ret != NO_ERROR) return ret;

    /* 5) 国标后处理：按标密差值阈值过滤点，并重算平均值
     *
     * 阈值 oil_standard_th 的量纲需与“Density20 RAW”一致：
     *   - 例程中比较的是 Result.Density20[k] 的差值
     *   - 本文件默认取 DensityMeasurement.standard_density 作为 Density20 RAW
     *
     * oil_standard_th 建议接入 systemunion.systemparameter.Oil_Standard 或对应参数。
     */
    if (mode == DENS_MODE_GB) {
        int32_t oil_standard_th = 100; /* 兼容既有国标过滤阈值，量纲与 Density20 RAW 差值一致。 */
        DensityInternal_FilterGbPointsByDensity20(out_dist, oil_level_01mm, g_deviceParams.spreadMeasurementOrder, oil_standard_th);
    }

    return NO_ERROR;
}

/* SI 剖面单测点临时结果；同时保存原始测量、换算后的频率/密度/温度以及气相判定依据。 */

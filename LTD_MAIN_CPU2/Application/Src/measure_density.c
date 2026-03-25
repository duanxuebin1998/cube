/*
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
 *   B. 切换到密度测量模式（EnableDensityMode）
 *   C. 按模式生成取点数组（单位：0.1mm）
 *   D. 按点位依次移动并单点测量（motorMoveToPositionOneShotWithSpeed + SinglePoint_ReadSensor）
 *   E. 计算平均值并输出（Print_DensitySpreadResult）
 *   F. 国标模式额外执行“按标密差值阈值过滤点，并重算平均值”
 *
 * 取点单位说明：
 *   - 例程取点使用 0.1mm（即 100um）作为整数单位，本文件保持一致。
 *   - 执行电机移动时，将 0.1mm 转换为 mm(float)：pos_mm = p01 / 10.0f
 *
 * 外部依赖（工程需提供）：
 *   - SearchOilLevel()
 *   - EnableDensityMode()
 *   - motorMoveToPositionOneShotWithSpeed(float pos_mm, uint32_t speed_x100)
 *   - SinglePoint_ReadSensor(volatile DensityMeasurement *result)
 *   - g_measurement / g_deviceParams
 *   - DensityDistribution / DensityMeasurement
 *   - 错误码宏/定义：NO_ERROR、PARAM_RANGE_ERROR、PARAM_ADDRESS_OVERFLOW、OTHER_UNKNOWN_ERROR、DENSITY_UNSTABLE 等
 *   - 状态码/宏：SET_ERROR、CHECK_ERROR、CHECK_COMMAND_SWITCH、CMD_NONE 等
 *
 * 重要注意：
 *   - g_deviceParams.spreadPointHoverTime 的“单位”在工程中存在两套使用方式：
 *       * Density_RunPoints01mm() 中按 ms 使用（HAL_Delay(hover_ms)）
 *       * SinglePoint_ReadSensor() 中按 s 使用（stable_win_ms = hover_time_s * 1000）
 *     若需要统一单位，应在参数定义侧统一；本文件保持现有逻辑不改动，仅在注释中明确差异。
 */

#include "measure_density.h"
#include "sensor.h"
#include "measure_oilLevel.h"
#include "motor_ctrl.h"
#include "system_parameter.h"
#include "measure.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* ===================== 前置声明 ===================== */
void Print_DensitySpreadResult(const DensityDistribution *dist);

/* 国标过滤（按例程逻辑：Density20 差值阈值触发删点并搬移） */
static void GB_FilterPoints_ByDensity20(DensityDistribution *dist,
                                        int32_t oil_level_01mm,
                                        uint32_t order,
                                        int32_t oil_standard_th);

/* ===================== 工具函数 ===================== */
static inline int32_t i32_max(int32_t a, int32_t b) { return (a > b) ? a : b; }
static inline int32_t i32_abs(int32_t x) { return (x >= 0) ? x : -x; }

static void PrintPoints01mm(const char *tag, const int32_t *p01, uint32_t n)
{
    printf("%s 取点 n=%lu: ", tag, (unsigned long)n);
    for (uint32_t i = 0; i < n; i++) {
        printf("%ld(%.1fmm) ", (long)p01[i], (double)p01[i] / 10.0);
    }
    printf("\r\n");
}

/* =======================================================================
 * 通用执行器：按点位数组执行测量
 *  - 输入点位单位：0.1mm
 *  - 输出：dist 写入测点列表与平均值
 * ======================================================================= */
static uint32_t Density_RunPoints01mm(const int32_t *p01,
                                      uint32_t n,
                                      int32_t oil_level_01mm,
                                      DensityDistribution *dist)
{
    if (!p01 || !dist) return PARAM_ADDRESS_OVERFLOW;
    if (n == 0 || n > MAX_MEASUREMENT_POINTS) return PARAM_RANGE_ERROR;

    memset(dist, 0, sizeof(*dist));

    uint32_t valid = 0;
    uint64_t sum_temp_raw = 0;
    uint64_t sum_dens_raw = 0;

    /* 悬停等待（此处按 ms 使用；与 SinglePoint_ReadSensor 的稳定窗口参数单位不同） */
    uint32_t hover_ms = g_deviceParams.spreadPointHoverTime;

    for (uint32_t i = 0; i < n; i++) {

        float pos_mm = (float)p01[i] / 10.0f;
        printf("分布测量 移动到位置 %.1f mm\r\n", pos_mm);

        uint32_t ret = motorMoveToPositionOneShotWithSpeed(pos_mm, motorGetDefaultSpeedX100());
        if (ret != NO_ERROR) {
            printf("分布测量 电机移动失败: pos=%.1fmm err=%lu\r\n", pos_mm, (unsigned long)ret);
            return ret;
        }

        if (hover_ms > 0) {
            HAL_Delay(hover_ms);
        }

        ret = SinglePoint_ReadSensor(&dist->single_density_data[valid]);
        if (ret != NO_ERROR) {
            printf("分布测量 单点读取失败: pos=%.1fmm err=%lu\r\n", pos_mm, (unsigned long)ret);
            return ret;
        }

        sum_temp_raw += dist->single_density_data[valid].temperature;
        sum_dens_raw += dist->single_density_data[valid].density;
        valid++;

        /* 命令切换退出：用于上位机/按键打断当前过程 */
        if (g_deviceParams.command != CMD_NONE) {
            printf("检测到命令切换请求，停止当前分布测量\r\n");
            break;
        }

        if (valid >= MAX_MEASUREMENT_POINTS) break;
    }

    if (valid == 0) {
        printf("分布测量 未得到任何有效测点\r\n");
        return OTHER_UNKNOWN_ERROR;
    }

    /* 平均值（RAW 平均） */
    dist->measurement_points = valid;
    dist->Density_oil_level  = (uint32_t)((oil_level_01mm + 5) / 10); /* mm 四舍五入 */

    dist->average_temperature = (uint32_t)(sum_temp_raw / valid);
    dist->average_density     = (uint32_t)(sum_dens_raw / valid);

    /* 若需要严格计算标准密度/VCF/计重密度，应在单点测量或后处理中补齐 */
    dist->average_standard_density = dist->average_density;
    dist->average_vcf20            = 0;
    dist->average_weight_density   = dist->average_density;

    return NO_ERROR;
}

/* =======================================================================
 * 取点逻辑：严格复刻例程
 * ======================================================================= */

/* ---------- 1) 普通分布测（SpredState=1） ----------
 * 关键点：
 *   - floor = max(bottomLimit, blindZone)
 *   - distmin 最小 100mm（0.1mm=1000）
 *   - high_min = oil_level - topLimit - floor
 *   - 实际步距 dis 使用 high_min/(N-1) 等分（不是固定 distance）
 */
static uint32_t BuildPoints_Spread_Exact(int32_t oil_level_01mm,
                                        int32_t *out_p01,
                                        uint32_t *out_n)
{
    if (!out_p01 || !out_n) return PARAM_ADDRESS_OVERFLOW;

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

    if (high <= 0) return PARAM_RANGE_ERROR;
    if (N_req == 0) return PARAM_RANGE_ERROR;
    if (N_req > MAX_MEASUREMENT_POINTS) N_req = MAX_MEASUREMENT_POINTS;

    int32_t high_min = high - top - floor;

    /* 退化：油位不足、点数=1、有效高度不足 -> 只取 1 点（油位中点，且不低于盲区） */
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

/* ---------- 2) 国标测（SpredState=3） ----------
 * 关键点：
 *   - 3m 以下：1 点（油位中点，且不低于盲区）
 *   - 3m~4.5m：最多 3 点（5/6、1/2、1/6），若点位低于盲区则退化
 *   - >4.5m：最多 5 点（1/6..5/6），若点位低于盲区则退化
 *   - 顺序按 spreadMeasurementOrder 决定（上->下 / 下->上）
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

/* ---------- 3) 每米测（SpredState=4） ----------
 * 关键点：
 *   - floor = max(bottomLimit, blindZone)
 *   - dis01 = oil_level - topLimit - floor，要求 >= 1m
 *   - 上->下：从 (high - (c_num+1)*1m) 逐米往下，必要时砍掉最后一个越界点
 *   - 下->上：从 (c_num+1)*1m 逐米往上，直到超过 high-top
 */
static uint32_t BuildPoints_Meter_Exact(int32_t oil_level_01mm,
                                       int32_t *out_p01,
                                       uint32_t *out_n)
{
    if (!out_p01 || !out_n) return PARAM_ADDRESS_OVERFLOW;

    int32_t high  = oil_level_01mm;
    int32_t top   = (int32_t)g_deviceParams.spreadTopLimit;
    int32_t floor = (int32_t)g_deviceParams.spreadBottomLimit;
    int32_t fade0 = (int32_t)g_deviceParams.blindZone;

    floor = i32_max(floor, fade0);

    int32_t meter = METER_STEP_01MM;
    int32_t dis01 = high - top - floor;
    if (dis01 < meter) {
        return PARAM_RANGE_ERROR;
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

    if (NumMeter == 0) return PARAM_RANGE_ERROR;

    *out_n = NumMeter;
    return NO_ERROR;
}

/* ---------- 4) 区间测（SpredState=5） ----------
 * 关键点：
 *   - 使用 upper/lower 两端点（0.1mm）
 *   - 点数使用 spreadMeasurementCount
 *   - 端点合法性：high_b < high_a，high_a < tankHeight，high_b >= blindZone
 *   - c_num>=3 时：等差取点，最后一点评端点
 */
static uint32_t BuildPoints_Interval_Exact(int32_t *out_p01, uint32_t *out_n)
{
    if (!out_p01 || !out_n) return PARAM_ADDRESS_OVERFLOW;

    uint32_t high_min, high_a, high_b;
    uint32_t dis;
    int c_num;
    int8_t i;

    high_min = (uint32_t)g_deviceParams.blindZone;

    high_a = g_deviceParams.wartsila_upper_density_limit;
    high_b = g_deviceParams.wartsila_lower_density_limit;

    if (high_b >= high_a) return PARAM_RANGE_ERROR;

    c_num = (int)g_deviceParams.spreadMeasurementCount;
    if (c_num <= 0) return PARAM_RANGE_ERROR;
    if (c_num > (int)MAX_MEASUREMENT_POINTS) c_num = MAX_MEASUREMENT_POINTS;

    if (high_a >= g_deviceParams.tankHeight) {
        return PARAM_RANGE_ERROR;
    } else if (high_b < high_min) {
        return PARAM_RANGE_ERROR;
    }

    if (c_num == 1) {
        out_p01[0] = (g_deviceParams.spreadMeasurementOrder == 0) ? (int32_t)high_a : (int32_t)high_b;
        *out_n = 1;
        return NO_ERROR;
    }

    if (c_num == 2) {
        if (g_deviceParams.spreadMeasurementOrder == 0) {
            out_p01[0] = (int32_t)high_a;
            out_p01[1] = (int32_t)high_b;
        } else {
            out_p01[0] = (int32_t)high_b;
            out_p01[1] = (int32_t)high_a;
        }
        *out_n = 2;
        return NO_ERROR;
    }

    dis = (uint32_t)(i32_abs((int32_t)high_a - (int32_t)high_b) / (uint32_t)(c_num - 1));

    if (g_deviceParams.spreadMeasurementOrder == 0) {
        for (i = 0; i < c_num; i++) {
            out_p01[i] = (int32_t)(high_a - dis * (uint32_t)i);
        }
        out_p01[c_num - 1] = (int32_t)high_b;
    } else {
        for (i = 0; i < c_num; i++) {
            out_p01[i] = (int32_t)(high_b + dis * (uint32_t)i);
        }
        out_p01[c_num - 1] = (int32_t)high_a;
    }

    *out_n = (uint32_t)c_num;
    return NO_ERROR;
}

/* =======================================================================
 * 模式路由：取点 + 执行 + 国标后处理
 * ======================================================================= */

uint32_t Density_MeasureByMode_Exact(DensitySpreadModeId mode, DensityDistribution *out_dist)
{
    if (!out_dist) return PARAM_ADDRESS_OVERFLOW;

    uint32_t ret;

    /* 1) 先液位搜索 */
    ret = SearchOilLevel();
    if (ret != NO_ERROR) {
        printf("密度测量\t液位搜索失败, err=0x%08lX\r\n", (unsigned long)ret);
        return ret;
    }

    /* 2) 切密度模式 */
    EnableDensityMode();

    int32_t oil_level_01mm = (int32_t)g_measurement.oil_measurement.oil_level; /* 0.1mm */

    int32_t points01[MAX_MEASUREMENT_POINTS];
    uint32_t n = 0;
    memset(points01, 0, sizeof(points01));

    /* 3) 按模式取点 */
    if (mode == DENS_MODE_SPREAD) {
        ret = BuildPoints_Spread_Exact(oil_level_01mm, points01, &n);
        if (ret != NO_ERROR) return ret;
        PrintPoints01mm("普通分布测", points01, n);
    }
    else if (mode == DENS_MODE_GB) {
        BuildPoints_GB4575_Exact((uint32_t)oil_level_01mm, points01, &n);
        if (n == 0) return PARAM_RANGE_ERROR;
        PrintPoints01mm("国标测", points01, n);
    }
    else if (mode == DENS_MODE_METER) {
        ret = BuildPoints_Meter_Exact(oil_level_01mm, points01, &n);
        if (ret != NO_ERROR) return ret;
        PrintPoints01mm("每米测", points01, n);
    }
    else if (mode == DENS_MODE_INTERVAL) {
        ret = BuildPoints_Interval_Exact(points01, &n);
        if (ret != NO_ERROR) return ret;
        PrintPoints01mm("区间测", points01, n);
    }
    else {
        return PARAM_RANGE_ERROR;
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
        int32_t oil_standard_th = 10; /* TODO：替换为实际参数，例如 systemunion.systemparameter.Oil_Standard */
        GB_FilterPoints_ByDensity20(out_dist, oil_level_01mm, g_deviceParams.spreadMeasurementOrder, oil_standard_th);
    }

    return NO_ERROR;
}

/* =======================================================================
 * 四种模式的对外入口
 * ======================================================================= */

void CMD_MeasureDensitySpread_Spread(void)
{
    uint32_t ret = 0;
    DensityDistribution temp = {0};

    g_measurement.device_status.device_state = STATE_SPREADPOINTING;

    ret = Density_MeasureByMode_Exact(DENS_MODE_SPREAD, &temp);
    if (ret != NO_ERROR) {
        printf("普通分布测\t失败, err=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }

    g_measurement.density_distribution = temp;
    Print_DensitySpreadResult(&temp);

    g_measurement.device_status.device_state = STATE_SPREADPOINTOVER;
}

void CMD_MeasureDensitySpread_GB(void)
{
    uint32_t ret = 0;
    DensityDistribution temp = {0};

    g_measurement.device_status.device_state = STATE_GB_SPREADPOINTING;

    ret = Density_MeasureByMode_Exact(DENS_MODE_GB, &temp);
    if (ret != NO_ERROR) {
        printf("国标测\t失败, err=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }

    g_measurement.density_distribution = temp;
    Print_DensitySpreadResult(&temp);

    g_measurement.device_status.device_state = STATE_GB_SPREADPOINTOVER;
}

void CMD_MeasureDensitySpread_Meter(void)
{
    uint32_t ret = 0;
    DensityDistribution temp = {0};

    g_measurement.device_status.device_state = STATE_METER_DENSITY;

    ret = Density_MeasureByMode_Exact(DENS_MODE_METER, &temp);
    if (ret != NO_ERROR) {
        printf("每米测\t失败, err=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }

    g_measurement.density_distribution = temp;
    Print_DensitySpreadResult(&temp);

    g_measurement.device_status.device_state = STATE_COM_METER_DENSITY_OVER;
}

void CMD_MeasureDensitySpread_Interval(void)
{
    uint32_t ret = 0;
    DensityDistribution temp = {0};

    g_measurement.device_status.device_state = STATE_INTERVAL_DENSITY;

    ret = Density_MeasureByMode_Exact(DENS_MODE_INTERVAL, &temp);
    if (ret != NO_ERROR) {
        printf("区间测\t失败, err=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }

    g_measurement.density_distribution = temp;
    Print_DensitySpreadResult(&temp);

    g_measurement.device_status.device_state = STATE_INTERVAL_DENSITY_OVER;
}

/* =======================================================================
 * 国标(GB)测量后处理：按“标密差值”过滤点（复刻例程的删点/搬移规则）
 * ======================================================================= */

/* 选择用于比较的“Density20 RAW”字段：
 *   - 例程比较的是 Result.Density20[k]
 *   - 本文件默认用 DensityMeasurement.standard_density 作为对应量（推荐）
 *   - 若工程尚未计算 standard_density，可临时改为 density
 */
#ifndef GB_COMPARE_USE_STANDARD_DENSITY
#define GB_COMPARE_USE_STANDARD_DENSITY  1
#endif

static inline int32_t gb_get_density20_raw(const DensityMeasurement *m)
{
#if GB_COMPARE_USE_STANDARD_DENSITY
    return (int32_t)m->standard_density;
#else
    return (int32_t)m->density;
#endif
}

static inline int32_t gb_abs_i32(int32_t x) { return (x >= 0) ? x : -x; }

static void gb_copy_point(DensityMeasurement *dst, const DensityMeasurement *src)
{
    *dst = *src;
}

static void gb_recalc_average(DensityDistribution *dist)
{
    if (!dist || dist->measurement_points == 0) return;

    uint32_t n = dist->measurement_points;
    uint64_t sum_t = 0, sum_d = 0, sum_sd = 0, sum_v = 0, sum_wd = 0;

    for (uint32_t i = 0; i < n; i++) {
        const DensityMeasurement *m = &dist->single_density_data[i];
        sum_t  += (uint64_t)m->temperature;
        sum_d  += (uint64_t)m->density;
        sum_sd += (uint64_t)m->standard_density;
        sum_v  += (uint64_t)m->vcf20;
        sum_wd += (uint64_t)m->weight_density;
    }

    dist->average_temperature      = (uint32_t)(sum_t  / n);
    dist->average_density          = (uint32_t)(sum_d  / n);
    dist->average_standard_density = (uint32_t)(sum_sd / n);
    dist->average_vcf20            = (uint32_t)(sum_v  / n);
    dist->average_weight_density   = (uint32_t)(sum_wd / n);
}

/*
 * 过滤规则（与例程一致）：
 *   1) oil_level < 3m：不做任何删点
 *   2) 3m <= oil_level <= 4.5m：
 *        - 当点数 n==3 且 |d1-d3|<=th：删为2点，保留 [1,3]
 *   3) oil_level > 4.5m：
 *        - n==3：同上，满足则 [1,3] -> 2点
 *        - n==4：
 *            * order==0（上->下，第二点为加测点）：
 *                 若 (1,3,4) 三者两两差值都<=th：删为3点，保留 [1,3,4]
 *            * order==1（下->上）：
 *                 若 (1,2,4) 三者两两差值都<=th：删为3点，保留 [1,2,4]
 *        - n==5：
 *            * 若 (1,3,5) 三者两两差值都<=th：删为3点，保留 [1,3,5]
 *
 * 数组下标说明：
 *   - dist->single_density_data[] 为 0-based
 *   - 例程 Result.xxx[] 为 1-based
 */
static void GB_FilterPoints_ByDensity20(DensityDistribution *dist,
                                       int32_t oil_level_01mm,
                                       uint32_t order,
                                       int32_t oil_standard_th)
{
    if (!dist) return;

    uint32_t n = dist->measurement_points;
    if (n < 2) return;

    /* 3m 以下：不处理 */
    if (oil_level_01mm < (int32_t)Spread_Gb_Onepiont_Posion) {
        return;
    }

    /* 3m~4.5m：仅处理 n==3 -> 2 */
    if (oil_level_01mm <= (int32_t)Spread_Gb_Twopiont_Posion) {
        if (n == 3) {
            int32_t d1 = gb_get_density20_raw(&dist->single_density_data[0]);
            int32_t d3 = gb_get_density20_raw(&dist->single_density_data[2]);
            if (gb_abs_i32(d1 - d3) <= oil_standard_th) {
                gb_copy_point(&dist->single_density_data[1], &dist->single_density_data[2]); /* 3 -> 2 */
                dist->measurement_points = 2;
                gb_recalc_average(dist);
            }
        }
        return;
    }

    /* >4.5m：处理 n==3/4/5 */
    if (n == 3) {
        int32_t d1 = gb_get_density20_raw(&dist->single_density_data[0]);
        int32_t d3 = gb_get_density20_raw(&dist->single_density_data[2]);
        if (gb_abs_i32(d1 - d3) <= oil_standard_th) {
            gb_copy_point(&dist->single_density_data[1], &dist->single_density_data[2]);
            dist->measurement_points = 2;
            gb_recalc_average(dist);
        }
        return;
    }

    if (n == 4) {
        if (order == 0) {
            int32_t d1 = gb_get_density20_raw(&dist->single_density_data[0]);
            int32_t d3 = gb_get_density20_raw(&dist->single_density_data[2]);
            int32_t d4 = gb_get_density20_raw(&dist->single_density_data[3]);

            if (gb_abs_i32(d1 - d3) <= oil_standard_th &&
                gb_abs_i32(d1 - d4) <= oil_standard_th &&
                gb_abs_i32(d4 - d3) <= oil_standard_th) {

                /* [1,3,4] -> 压缩为 3 点 */
                gb_copy_point(&dist->single_density_data[1], &dist->single_density_data[2]); /* 3 -> 2 */
                gb_copy_point(&dist->single_density_data[2], &dist->single_density_data[3]); /* 4 -> 3 */
                dist->measurement_points = 3;
                gb_recalc_average(dist);
            }
        } else {
            int32_t d1 = gb_get_density20_raw(&dist->single_density_data[0]);
            int32_t d2 = gb_get_density20_raw(&dist->single_density_data[1]);
            int32_t d4 = gb_get_density20_raw(&dist->single_density_data[3]);

            if (gb_abs_i32(d1 - d2) <= oil_standard_th &&
                gb_abs_i32(d1 - d4) <= oil_standard_th &&
                gb_abs_i32(d4 - d2) <= oil_standard_th) {

                /* [1,2,4] -> 压缩为 3 点：4 搬到 3 */
                gb_copy_point(&dist->single_density_data[2], &dist->single_density_data[3]); /* 4 -> 3 */
                dist->measurement_points = 3;
                gb_recalc_average(dist);
            }
        }
        return;
    }

    if (n == 5) {
        int32_t d1 = gb_get_density20_raw(&dist->single_density_data[0]);
        int32_t d3 = gb_get_density20_raw(&dist->single_density_data[2]);
        int32_t d5 = gb_get_density20_raw(&dist->single_density_data[4]);

        if (gb_abs_i32(d1 - d3) <= oil_standard_th &&
            gb_abs_i32(d1 - d5) <= oil_standard_th &&
            gb_abs_i32(d5 - d3) <= oil_standard_th) {

            /* [1,3,5] -> 压缩为 3 点 */
            gb_copy_point(&dist->single_density_data[1], &dist->single_density_data[2]); /* 3 -> 2 */
            gb_copy_point(&dist->single_density_data[2], &dist->single_density_data[4]); /* 5 -> 3 */
            dist->measurement_points = 3;
            gb_recalc_average(dist);
        }
        return;
    }
}

/* =======================================================================
 * 打印与单点测量函数（保持原实现，仅补充注释）
 * ======================================================================= */

void Print_DensitySpreadResult(const DensityDistribution *dist)
{
    if (dist == NULL) {
        printf("分布测量结果为空！\r\n");
        return;
    }

    printf("\r\n========== 密度分布测量结果 ==========\r\n");

    printf("测量点数量         : %lu\r\n", (unsigned long) dist->measurement_points);
    printf("测量油位/罐高 (mm) : %lu\r\n", (unsigned long) dist->Density_oil_level);

    /* average_* 为 RAW（编码值） */
    printf("平均温度 RAW       : %lu  =>  实际: %.2f ℃\r\n",
           (unsigned long) dist->average_temperature,
           RAW_TO_TEMP(dist->average_temperature));

    printf("平均密度 RAW       : %lu  =>  实际: %.1f\r\n",
           (unsigned long) dist->average_density,
           RAW_TO_DENSITY(dist->average_density));

    printf("标准密度 RAW       : %lu  =>  实际: %.1f\r\n",
           (unsigned long) dist->average_standard_density,
           RAW_TO_DENSITY(dist->average_standard_density));

    printf("VCF20 RAW          : %lu\r\n", (unsigned long) dist->average_vcf20);

    printf("计重密度 RAW       : %lu  =>  实际: %.1f\r\n",
           (unsigned long) dist->average_weight_density,
           RAW_TO_DENSITY(dist->average_weight_density));

    printf("\r\n------ 单点数据列表 ------\r\n");
    printf("序号  位置(mm)  密度RAW  密度(实测)  温度RAW   温度(℃)\r\n");

    uint32_t n = dist->measurement_points;
    if (n > MAX_MEASUREMENT_POINTS) n = MAX_MEASUREMENT_POINTS;

    for (uint32_t i = 0; i < n; i++) {
        const DensityMeasurement *p = &dist->single_density_data[i];

        float dens_f = RAW_TO_DENSITY(p->density);
        float temp_f = RAW_TO_TEMP(p->temperature);
        float temp_position = (float)(p->temperature_position) / 10.0f;

        printf("%3lu   %.1f   %7lu   %8.1f   %7lu   %7.2f\r\n",
               (unsigned long) i,
               temp_position,
               (unsigned long) p->density,
               dens_f,
               (unsigned long) p->temperature,
               temp_f);
    }

    printf("=====================================\r\n\r\n");
}

/**
 * @brief 单点密度测量（带稳定判定与超时兜底）
 *
 * 【总体逻辑】
 *  1) 周期性读取 频率 / 密度 / 温度
 *  2) 仅当“密度为非零”时，才参与稳定判定
 *  3) 若在稳定窗口内，三项数据变化均不超过阈值，则判定“数据稳定”
 *  4) 若 5 分钟内始终未稳定：
 *      - 若曾读到非零密度：取最后一次非零密度作为结果
 *      - 若 5 分钟内从未读到非零密度：输出 0 作为结果
 *
 * 【关键口径】
 *  - 密度 == 0：
 *      - 不参与稳定判定
 *      - 不能作为“稳定值”
 *      - 但在“完全无有效密度”的异常场景下，可作为最终兜底输出
 *
 * @param[out] result  单点测量结果结构体（RAW 编码）
 *
 * @return NO_ERROR           成功（稳定或兜底）
 *         其他错误码        模式切换/通信等异常
 */
uint32_t SinglePoint_ReadSensor(volatile DensityMeasurement *result)
{
    uint32_t ret = 0;

    /* ---------- 切换到密度测量模式（防御性调用） ---------- */
    ret = EnableDensityMode();
    CHECK_ERROR(ret);

    /* ---------- 稳定窗口时间配置 ----------
     * spreadPointHoverTime：
     *   - 单位：秒
     *   - 表示需要“连续稳定”多久，才认为单点数据可靠
     */
    uint32_t hover_time_s  = g_deviceParams.spreadPointHoverTime;
    uint32_t stable_win_ms = hover_time_s * 1000U;

    /* 若参数未配置，使用默认 5 秒 */
    if (stable_win_ms == 0) {
        stable_win_ms = 5000U;
    }

    /* ---------- 最大等待时间：5 分钟 ----------
     * 防止传感器异常或工况不稳定导致死等
     */
    const uint32_t MAX_WAIT_MS = 5U * 60U * 1000U;

    /* ---------- 稳定判定阈值 ----------
     * 任一项超出阈值，均认为“不稳定”，需要重新计时
     */
    const float FREQ_EPS    = 1.0f;   // 频率变化阈值（Hz）
    const float DENSITY_EPS = 0.1f;   // 密度变化阈值
    const float TEMP_EPS    = 0.2f;   // 温度变化阈值（℃）

    /* 采样周期 */
    const uint32_t SAMPLE_INTERVAL_MS = 200U;

    /* ---------- 时间与状态变量 ---------- */
    uint32_t t_start      = HAL_GetTick();  // 整个流程起始时间
    uint32_t stable_start = 0;              // 当前稳定窗口起始时间
    uint8_t  first_sample = 1;              // 是否为首次有效样本

    /* ---------- 参考值（用于稳定判定） ----------
     * 仅在“非零密度样本”下才会更新
     */
    float ref_freq = 0.0f;
    float ref_density = 0.0f;
    float ref_temp = 0.0f;

    /* 当前读取值 */
    float cur_freq = 0.0f;
    float cur_density = 0.0f;
    float cur_temp = 0.0f;

    /* ---------- 超时兜底用变量 ----------
     * 用于记录“最后一次非零密度样本”
     */
    uint8_t have_last_nonzero = 0;
    float last_density_nz = 0.0f;
    float last_temp_nz = 0.0f;

    while (1) {

        uint32_t now = HAL_GetTick();

        /* ======================================================
         * 1) 超时兜底处理（5 分钟）
         * ====================================================== */
        if (now - t_start >= MAX_WAIT_MS) {

            /* --- 情况 A：曾经读到过非零密度 --- */
            if (have_last_nonzero) {

                printf("单点测量 超时未稳定，取最后一次非零密度作为结果。\r\n");

                result->density          = DENSITY_TO_RAW(last_density_nz);
                result->temperature      = TEMP_TO_RAW(last_temp_nz);
                result->standard_density = DENSITY_TO_RAW(last_density_nz);
                result->weight_density   = DENSITY_TO_RAW(last_density_nz);

            }
            /* --- 情况 B：5 分钟内从未读到非零密度 --- */
            else {

                printf("单点测量 5分钟内未读到非零密度，输出0作为结果。\r\n");

                /* 注意：
                 * 这里的 0 仅用于流程兜底，不代表有效密度
                 */
                result->density          = 0;
                result->standard_density = 0;
                result->weight_density   = 0;

                /* 温度可取最近一次值（即使为 0） */
                result->temperature = TEMP_TO_RAW(cur_temp);
            }

            result->vcf20 = 1;
            result->temperature_position = g_measurement.debug_data.sensor_position;

            return NO_ERROR;
        }

        /* ======================================================
         * 2) 读取传感器
         * ====================================================== */
        ret = Read_Density(&cur_freq, &cur_density, &cur_temp);
        if (ret != NO_ERROR) {
            printf("读取密度/温度/频率失败：err=%lu\r\n",
                   (unsigned long)ret);
            HAL_Delay(SAMPLE_INTERVAL_MS);
            continue;
        }

        CHECK_COMMAND_SWITCH(ret);

        printf("单点读数: f=%.3f Hz  dens=%.4f  temp=%.3f ℃\r\n",
               cur_freq, cur_density, cur_temp);

        /* ======================================================
         * 3) 密度为 0 的处理策略
         * ======================================================
         *  - 认为是“无效密度”
         *  - 不参与稳定判定
         *  - 不更新参考值
         */
        if (fabsf(cur_density) < 1e-6f) {
            HAL_Delay(SAMPLE_INTERVAL_MS);
            continue;
        }

        /* 记录最后一次“非零密度”样本（用于超时兜底） */
        have_last_nonzero = 1;
        last_density_nz   = cur_density;
        last_temp_nz      = cur_temp;

        /* ======================================================
         * 4) 稳定判定逻辑（仅对非零密度生效）
         * ====================================================== */
        if (first_sample) {

            /* 首次有效样本：直接作为参考值 */
            ref_freq    = cur_freq;
            ref_density = cur_density;
            ref_temp    = cur_temp;
            stable_start = now;
            first_sample = 0;

        } else {

            float df = fabsf(cur_freq    - ref_freq);
            float dd = fabsf(cur_density - ref_density);
            float dt = fabsf(cur_temp    - ref_temp);

            /* 任一项超阈值，认为不稳定，重置参考值与计时 */
            if (df > FREQ_EPS || dd > DENSITY_EPS || dt > TEMP_EPS) {
                ref_freq    = cur_freq;
                ref_density = cur_density;
                ref_temp    = cur_temp;
                stable_start = now;
            }
        }

        /* ======================================================
         * 5) 稳定窗口满足：判定稳定
         * ====================================================== */
        if (!first_sample && (now - stable_start >= stable_win_ms)) {

            printf("单点测量 数据稳定，稳定窗口=%lu ms\r\n",
                   (unsigned long)stable_win_ms);

            result->temperature_position = g_measurement.debug_data.sensor_position;
            result->density          = DENSITY_TO_RAW(ref_density);
            result->temperature      = TEMP_TO_RAW(ref_temp);
            result->standard_density = DENSITY_TO_RAW(ref_density);
            result->weight_density   = DENSITY_TO_RAW(ref_density);
            result->vcf20            = 1;

            return NO_ERROR;
        }

        HAL_Delay(SAMPLE_INTERVAL_MS);
    }
}



/* 单点测量命令：移动到指定高度 -> 单点稳定读取 */
void CMD_SinglePointMeasurement(void)
{
    uint32_t ret = 0;
    g_measurement.device_status.device_state = STATE_SINGLEPOINTING;

    MeasureStart();

    ret = motorMoveToPositionOneShotWithSpeed((float)g_deviceParams.singlePointMeasurementPosition / 10.0f,
                                              motorGetDefaultSpeedX100());
    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_SPTESTING;

    EnableDensityMode();

    ret = SinglePoint_ReadSensor(&g_measurement.single_point_measurement);
    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_SINGLEPOINTOVER;
}

/* 单点监测命令：移动到监测高度 -> 循环单点稳定读取（直到命令切换） */
void CMD_SinglePointMonitoring(void)
{
    uint32_t ret = 0;
    g_measurement.device_status.device_state = STATE_RUNTOPOINTING;

    ret = motorMoveToPositionOneShotWithSpeed((float)g_deviceParams.singlePointMonitoringPosition / 10.0f,
                                              motorGetDefaultSpeedX100());
    SET_ERROR(ret);

    g_measurement.device_status.device_state = STATE_SPTESTING;

    EnableDensityMode();

    while (1) {
        ret = SinglePoint_ReadSensor(&g_measurement.single_point_monitoring);
        SET_ERROR(ret);

        if (g_deviceParams.command != CMD_NONE) {
            printf("检测到命令切换请求，停止当前操作\r\n");
            return;
        }
    }
}

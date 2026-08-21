/*
 * measure_density_commands.c
 *
 * 文件职责：封装四种密度分布与瓦锡兰命令、国标测点过滤和结果打印；普通分布内核只负责生成与采集点阵。
 */

#include "measure_density.h"
#include "measure_density_internal.h"
#include "../measure_commands_internal.h"

#include <stdint.h>
#include <stdio.h>

#include "fault_manager.h"
#include "system_parameter.h"
#include "wartsila_density_measurement.h"
#include "motor_ctrl.h"
#include "measure_tank_height.h"
#include "abortable_delay.h"

/**
 * @brief 执行普通密度分布测量，并仅在完整成功后打印和发布新的标准点阵。
 *
 * 开始时清除上一轮完成锁存并将设备状态置为普通分布测量中，再调用 Density_MeasureByMode_Exact 生成临时结果。
 * 测量完整成功后打印结果，通过 PROFILE_SOURCE_STANDARD 发布整份点阵及新代际，最后发布普通分布测量完成状态。
 *
 * @note STATE_SWITCH 是新命令触发的正常退出，不发布候选点阵也不进入错误报警；其他失败通过 SET_ERROR 进入统一处理。
 */

void CMD_MeasureDensitySpread_Spread(void)
{
    uint32_t ret = 0;
    DensityDistribution temp = {0};

    /* 分布测量开始前关闭上一轮完成锁存，CPU3继续保留已确认快照。 */
    DensityProfile_Begin();

    g_measurement.device_status.device_state = STATE_SPREADPOINTING;

    ret = Density_MeasureByMode_Exact(DENS_MODE_SPREAD, &temp);
    /* STATE_SWITCH 是命令切换的正常退出，不进入错误重试或报警。 */
    if (ret == STATE_SWITCH) {
        return;
    }
    if (ret != NO_ERROR) {
        printf("普通分布测\t失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }

    Print_DensitySpreadResult(&temp);

    /* 完整点阵写入后最后递增完成计数，CPU3只会同步完整新代际。 */
    DensityProfile_PublishResult(&temp, PROFILE_SOURCE_STANDARD);

    g_measurement.device_status.device_state = STATE_SPREADPOINTOVER;
}

/**
 * @brief 执行国标密度分布测量；仅在完整成功后发布点阵和完成态。
 */
void CMD_MeasureDensitySpread_GB(void)
{
    uint32_t ret = 0;
    DensityDistribution temp = {0};

    /* 分布测量开始前关闭上一轮完成锁存，CPU3继续保留已确认快照。 */
    DensityProfile_Begin();

    g_measurement.device_status.device_state = STATE_GB_SPREADPOINTING;

    ret = Density_MeasureByMode_Exact(DENS_MODE_GB, &temp);
    /* STATE_SWITCH 是命令切换的正常退出，不进入错误重试或报警。 */
    if (ret == STATE_SWITCH) {
        return;
    }
    if (ret != NO_ERROR) {
        printf("国标测\t失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }

    Print_DensitySpreadResult(&temp);

    /* 完整点阵写入后最后递增完成计数，CPU3只会同步完整新代际。 */
    DensityProfile_PublishResult(&temp, PROFILE_SOURCE_GB);

    g_measurement.device_status.device_state = STATE_GB_SPREADPOINTOVER;
}

/**
 * @brief 执行每米密度分布测量；仅在完整成功后发布点阵和完成态。
 */
void CMD_MeasureDensitySpread_Meter(void)
{
    uint32_t ret = 0;
    DensityDistribution temp = {0};

    /* 分布测量开始前关闭上一轮完成锁存，CPU3继续保留已确认快照。 */
    DensityProfile_Begin();

    g_measurement.device_status.device_state = STATE_METER_DENSITY;

    ret = Density_MeasureByMode_Exact(DENS_MODE_METER, &temp);
    /* STATE_SWITCH 是命令切换的正常退出，不进入错误重试或报警。 */
    if (ret == STATE_SWITCH) {
        return;
    }
    if (ret != NO_ERROR) {
        printf("每米测\t失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
    }

    Print_DensitySpreadResult(&temp);

    /* 完整点阵写入后最后递增完成计数，CPU3只会同步完整新代际。 */
    DensityProfile_PublishResult(&temp, PROFILE_SOURCE_METER);

    g_measurement.device_status.device_state = STATE_COM_METER_DENSITY_OVER;
}

/**
 * @brief 执行区间密度分布测量；仅在完整成功后发布点阵和完成态。
 */
void CMD_MeasureDensitySpread_Interval(void)
{
    uint32_t ret = 0;
    DensityDistribution temp = {0};

    /* 分布测量开始前关闭上一轮完成锁存，CPU3继续保留已确认快照。 */
    DensityProfile_Begin();

    g_measurement.device_status.device_state = STATE_INTERVAL_DENSITY;

    ret = Density_MeasureByMode_Exact(DENS_MODE_INTERVAL, &temp);
    /* STATE_SWITCH 是命令切换的正常退出，不进入错误重试或报警。 */
    if (ret == STATE_SWITCH) {
        return;
    }
    /* 区间测失败时禁止发布空结果或部分结果，也不得进入完成态。 */
    if (ret != NO_ERROR) {
        printf("区间测\t失败，错误码=0x%08lX\r\n", (unsigned long)ret);
        SET_ERROR(ret);
        return;
    }

    Print_DensitySpreadResult(&temp);

    /* 完整点阵写入后最后递增完成计数，CPU3只会同步完整新代际。 */
    DensityProfile_PublishResult(&temp, PROFILE_SOURCE_INTERVAL);

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
#define GB_COMPARE_USE_STANDARD_DENSITY  1 /* 国标比较时使用标准密度的开关。 */
#endif

/**
 * @brief 从国标密度点读取 20 ℃标准密度原始值。
 *
 * @param m 待读取的单个密度测点记录；函数提取其中 20 ℃标准密度的原始定点值。
 * @return GB_COMPARE_USE_STANDARD_DENSITY 启用时返回测点 standard_density，否则返回 density；两者均沿用内部 0.01 kg/m3
 *         原始定点格式。
 */
static inline int32_t gb_get_density20_raw(const DensityMeasurement *m)
{
#if GB_COMPARE_USE_STANDARD_DENSITY
    return (int32_t)m->standard_density;
#else
    return (int32_t)m->density;
#endif
}

/**
 * @brief 计算有符号 32 位整数的安全绝对值。
 *
 * @param x 算法、坐标或比较使用的 X 值。
 * @return 返回有符号 32 位整数的安全绝对值；有符号边界按函数内饱和规则处理。
 */
static inline int32_t gb_abs_i32(int32_t x) { return (x >= 0) ? x : -x; }

/**
 * @brief 复制一个国标密度测点的全部测量字段。
 *
 * @param dst 用于接收完整测点数据副本的目标结构。
 * @param src 待复制的只读测点数据结构。
 */
static void gb_copy_point(DensityMeasurement *dst, const DensityMeasurement *src)
{
    *dst = *src;
}

/**
 * @brief 按当前有效点数重新计算温度、密度、标准密度、VCF 和计重密度平均值。
 *
 * @param dist 密度分布测量结果对象。
 */
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

/**
 * @brief 过滤规则（与例程一致）：1) oil_level < 3m：不做任何删点。
 *
 * 2) 3m <= oil_level <= 4.5m：- 当点数 n==3 且 |d1-d3|<=th：删为2点，保留 [1,3]。
 * 3) oil_level > 4.5m：- n==3：同上，满足则 [1,3] -> 2点。
 * n==4：* order==0（上->下，第二点为加测点）：若 (1,3,4) 三者两两差值都<=th：删为3点，保留 [1,3,4]。
 * * order==1（下->上）：若 (1,2,4) 三者两两差值都<=th：删为3点，保留 [1,2,4]。
 * n==5：* 若 (1,3,5) 三者两两差值都<=th：删为3点，保留 [1,3,5]。
 * 数组下标说明：- dist->single_density_data[] 为 0-based。
 * 例程 Result.xxx[] 为 1-based。
 *
 * @param dist 密度分布测量结果对象。
 * @param oil_level_01mm 油位值，单位 0.1 mm。
 * @param order 按密度筛选或排序时使用的测点索引顺序数组。
 * @param oil_standard_th 油位。
 */
void DensityInternal_FilterGbPointsByDensity20(DensityDistribution *dist,
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

/**
 * @brief 打印密度分布结果的测点数、汇总值以及逐点位置、密度和温度。
 *
 * @param dist 待打印的密度分布结果；包含测点数、液位或罐高、汇总密度温度和最多 MAX_MEASUREMENT_POINTS 个单点数据，传入 NULL 时只打印空结果提示。
 */

void Print_DensitySpreadResult(const DensityDistribution *dist)
{
    if (dist == NULL) {
        printf("分布测量结果为空！\r\n");
        return;
    }

    printf("\r\n========== 密度分布测量结果 ==========\r\n");

    printf("测量点数量             : %lu\r\n", (unsigned long) dist->measurement_points);
    printf("测量液位/罐高(0.1mm)   : %lu  =>  实际: %.1f mm\r\n",
           (unsigned long) dist->Density_oil_level,
           (double) dist->Density_oil_level / 10.0);

    /* average_* 为 RAW（编码值） */
    printf("平均温度 原始值    : %lu  =>  实际: %.2f ℃\r\n",
           (unsigned long) dist->average_temperature,
           RAW_TO_TEMP(dist->average_temperature));

    printf("平均密度 原始值    : %lu  =>  实际: %.2f\r\n",
           (unsigned long) dist->average_density,
           RAW_TO_DENSITY(dist->average_density));

    printf("标准密度 原始值    : %lu  =>  实际: %.2f\r\n",
           (unsigned long) dist->average_standard_density,
           RAW_TO_DENSITY(dist->average_standard_density));

    printf("VCF20 原始值       : %lu\r\n", (unsigned long) dist->average_vcf20);

    printf("计重密度 原始值    : %lu  =>  实际: %.2f\r\n",
           (unsigned long) dist->average_weight_density,
           RAW_TO_DENSITY(dist->average_weight_density));

    printf("\r\n------ 单点数据列表 ------\r\n");
    printf("序号  位置(mm)  密度原始值  密度(实测)  温度原始值   温度(℃)\r\n");

    uint32_t n = dist->measurement_points;
    if (n > MAX_MEASUREMENT_POINTS) n = MAX_MEASUREMENT_POINTS;

    for (uint32_t i = 0; i < n; i++) {
        const DensityMeasurement *p = &dist->single_density_data[i];

        float dens_f = RAW_TO_DENSITY(p->density);
        float temp_f = RAW_TO_TEMP(p->temperature);
        float temp_position = (float)(p->temperature_position) / 10.0f;

        printf("%3lu   %.1f   %7lu   %8.2f   %7lu   %7.2f\r\n",
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
 * @brief 校验并移动到固定点监测位置，最多重试三次，不读取密度。
 * @return 返回整机错误码；NO_ERROR 表示已到达固定监测位置，其他值表示位置校验、运动或停止确认失败。
 */
static uint32_t Wartsila_MoveToMonitorPositionOnly(void)
{
    uint32_t ret = NO_ERROR;
    const uint32_t max_attempts = 3U;
    float target_mm = (float)DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_SINGLE_POINT_MONITORING_POSITION) / 10.0f;

    g_measurement.device_status.device_state = STATE_RUNTOPOINTING;
    printf("瓦锡兰测后探底\t先回固定点监测位置：%.1fmm，仅移动不读密度\r\n", (double)target_mm);

    ret = SinglePoint_CheckTargetPosition("瓦锡兰测后回固定点", DeviceCommandArguments_Get(DEVICE_COMMAND_ARG_SINGLE_POINT_MONITORING_POSITION));
    if (ret != NO_ERROR) {
        return ret;
    }

    for (uint32_t attempt = 1U; attempt <= max_attempts; attempt++) {
        ret = MotorCtrl_JogMoveToPosition(target_mm, MotorCtrl_GetDefaultSpeedX100());
        /* 回固定监测位置成功或收到命令切换时立即结束；只有真实运动失败才继续消耗剩余尝试次数。 */
        if ((ret == NO_ERROR) || (ret == STATE_SWITCH)) {
            return ret;
        }

        printf("瓦锡兰测后探底\t回固定点监测位置失败：0x%08lX，尝试：%lu/%lu\r\n",
               (unsigned long)ret,
               (unsigned long)attempt,
               (unsigned long)max_attempts);
    }

    printf("瓦锡兰测后探底\t回固定点监测位置重试失败，跳过探底且不置错误状态\r\n");
    return ret;
}

/**
 * @brief 执行并发布瓦锡兰点阵，保留结果供 CPU3 读取，并按配置周期探底后恢复固定点监测。
 *
 * 开始新一轮测量时先关闭旧完成锁存并发布瓦锡兰测量中状态；点阵先写入局部临时结构，只有 Wartsila_Density_SpreadMeasurement 完整成功后才作为新的瓦锡兰代际整体发布。
 * 发布后先保留一秒再打印结果，切换到测量完成状态，并以可被新命令打断的方式继续保留八秒，给 CPU3 足够时间读取完整快照。
 * 每轮完整测量后更新运行期总次数和探底周期计数；探底频次 0 表示禁用，1 至 100 表示每 N 次执行一次，超过 100 的异常配置按每次探底处理并输出诊断。
 * 到达探底周期时先仅移动到固定点监测位置，再执行罐底搜索；参数配置错误进入统一故障状态，普通移动或探底失败则跳过本次探底并排队恢复单点监测，不覆盖已经发布的密度结果。
 *
 * @note 任一可中断等待、移动或测量返回 STATE_SWITCH 时立即退出，把最新命令交还主循环处理。
 */
void CMD_WartsilaDensitySpread(void) {
	static uint32_t bottom_detect_count = 0; /* 瓦锡兰测量后探底计数，仅运行期累计 */
	static uint32_t wartsila_measure_total_count = 0; /* 瓦锡兰分布测量完成次数，仅运行期累计 */
	uint32_t ret = 0;
	uint32_t bottom_detect_interval = g_deviceParams.wartsila_bottom_detect_interval; /* 本次瓦锡兰测量后的探底频率参数快照 */
	DensityDistribution temp = {0};

	/* 新一轮瓦锡兰测量先关闭旧完成锁存，CPU3仍保留已确认快照。 */
	DensityProfile_Begin();
	/* 设置设备状态：分布测量中 */
	g_measurement.device_status.device_state = STATE_WARTSILA_DENSITY_MEASURING;

	ret = Wartsila_Density_SpreadMeasurement(&temp);
	if (ret == STATE_SWITCH) {
		return;
	}
	if (ret != NO_ERROR) {
		printf("瓦锡兰分布测量失败，错误码：0x%08lX，不更新新的有效结果\r\n", (unsigned long)ret);
		SET_ERROR(ret);
		return;
	}

	/* 完整点阵写入后最后递增完成计数，CPU3只会同步完整新代际。 */
	DensityProfile_PublishResult(&temp, PROFILE_SOURCE_WARTSILA);

	if (AbortableDelay_CommandSwitch(1000U, 100U) == STATE_SWITCH) {
		return;
	}
	Print_DensitySpreadResult(&temp);
/* 测量结束，状态切换为分布测量完成 */
	g_measurement.device_status.device_state = STATE_WARTSILA_DENSITY_OVER;
	printf("瓦锡兰分布测量结果保留8秒，原因：等待CPU3读取结果，期间可切换命令退出\r\n");
	for (uint32_t remain_s = 8U; remain_s > 0U; remain_s--) {
		printf("瓦锡兰分布测量结果保留倒计时：%lu秒\r\n", (unsigned long)remain_s);
		if (AbortableDelay_CommandSwitch(1000U, 100U) == STATE_SWITCH) {
			return;
		}
	}
    wartsila_measure_total_count++;
    /* 按参数控制瓦锡兰测量后的探底频率：0不探底，N表示每N次测量后探底一次，最大100。 */
    if (bottom_detect_interval > 100U) {
        printf("瓦锡兰测后探底\t探底频次参数=%lu 超出上限100，按每1次执行\r\n",
               (unsigned long)bottom_detect_interval);
        bottom_detect_interval = 1U;
    }

    if (bottom_detect_interval > 0U) {
        uint32_t current_cycle_count = bottom_detect_count + 1U;
        bool will_search_bottom = (current_cycle_count >= bottom_detect_interval);

        printf("瓦锡兰测后探底\t测量总次数=%lu | 探底频次=每%lu次 | 当前周期第%lu/%lu次 | 本次%s探底\r\n",
               (unsigned long)wartsila_measure_total_count,
               (unsigned long)bottom_detect_interval,
               (unsigned long)current_cycle_count,
               (unsigned long)bottom_detect_interval,
               will_search_bottom ? "执行" : "不执行");

        bottom_detect_count = current_cycle_count;
        if (will_search_bottom) {
            bottom_detect_count = 0U;
            ret = Wartsila_MoveToMonitorPositionOnly();
            if (ret == STATE_SWITCH) {
                return;
            }
            /* 缺失、冲突或越界的瓦锡兰配置属于确定性参数错误，先发布故障码，不能把它当作普通测量抖动静默重试。 */
            if ((ret == PARAM_CONFIG_MISSING) ||
                (ret == PARAM_COMBINATION_CONFLICT) ||
                (ret == PARAM_RANGE_ERROR)) {
                SET_ERROR(ret);
            }
            /* 瓦锡兰测量未成功时排队恢复固定点监测；参数错误已在上一步发布，其他错误保留原返回语义。 */
            if (ret != NO_ERROR) {
                DeviceCommand_Queue(CMD_MONITOR_SINGLE);
                return;
            }

            ret = SearchBottom();
            if (ret == STATE_SWITCH) {
                return;
            }
            if (ret != NO_ERROR) {
                printf("瓦锡兰测后探底\t罐底测量失败：0x%08lX，退出且不置错误状态\r\n", (unsigned long)ret);
                DeviceCommand_Queue(CMD_MONITOR_SINGLE);
                return;
            }
        }
    } else {
        bottom_detect_count = 0U;
        printf("瓦锡兰测后探底\t测量总次数=%lu | 探底频次=0，不探底\r\n",
               (unsigned long)wartsila_measure_total_count);
    }
    DeviceCommand_Queue(CMD_MONITOR_SINGLE); /* 切回单点监测状态，继续监测当前液位/密度 */
	return;
}

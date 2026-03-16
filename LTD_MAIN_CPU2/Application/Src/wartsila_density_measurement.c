/*
 * wartsila_density_measurement.c
 * 瓦西莱密度梯度测量
 *  Created on: 2025年12月1日
 *      Author: Duan Xuebin
 */


#include "wartsila_density_measurement.h"
#include "measure_oilLevel.h"
#include "ltd_sensor_communication.h"
#include "measure_density.h"

uint32_t motorMoveUpToPositionOrAir(float target_mm, Level_StateTypeDef *final_state);

/**
 * @brief  Wartsila 密度分布测量（从起始点向上，途中遇到空气或到达最高点停止）
 *
 * 流程说明：
 * 1. 使用以下参数控制分布测量：
 *    - wartsila_lower_density_limit        分布测量起始点高度（mm）
 *    - wartsila_upper_density_limit        分布测量最高点高度（mm）
 *    - wartsila_density_interval           分布测量点间距（mm）
 *    - wartsila_max_height_above_surface   最高测点距油面最小允许距离（mm）
 *
 * 2. 先把传感器移动到起始点高度；
 * 3. 从起始点开始，按间距向上依次运行到各个测点目标高度：
 *    - 上行过程中调用 motorMoveUpToPositionOrAir() 检测液位状态；
 *    - 若途中检测到传感器进入空气（AIR），立即停止，不再向上；
 *    - 若顺利到达目标高度（OIL），则在该位置采集一个密度测点；
 * 4. 若始终未进入空气，则最多运行到分布测量最高点；
 * 5. 测量结束后如果最高测点距离油面小于 wartsila_max_height_above_surface，则舍弃最高点数据。
 *
 * 坐标与单位约定：
 * - 罐底为 0 mm，向上为正方向；
 * - g_measurement.oil_measurement.oil_level 单位为 0.1 mm；
 * - wartsila_* 相关位置参数单位均为 mm；
 * - 单点密度、温度字段为原始值（与 RAW_TO_DENSITY、RAW_TO_TEMP 对应）。
 *
 * @param  dist  输出测量结果的结构体指针（一般为 &g_measurement.density_distribution）
 * @return 错误码，NO_ERROR 表示成功
 */
uint32_t Wartsila_Density_SpreadMeasurement(DensityDistribution *dist)
{
    if (dist == NULL) {
        return PARAM_ADDRESS_OVERFLOW;
    }

    memset(dist, 0, sizeof(DensityDistribution));
	EnableLevelMode();
    /* 位置相关参数 */
    uint32_t start_pos_mm    = g_deviceParams.wartsila_lower_density_limit;      /* 起始点高度 */
    uint32_t end_pos_mm      = g_deviceParams.wartsila_upper_density_limit;      /* 最高点高度 */
    uint32_t step_mm         = g_deviceParams.wartsila_density_interval;         /* 点间距 */
    uint32_t min_gap_surface = g_deviceParams.wartsila_max_height_above_surface; /* 最高点距油面最小距离 */

    if (step_mm == 0) {
        printf("分布测量参数错误：step_mm=0\n");
        return PARAM_RANGE_ERROR;
    }
    if (end_pos_mm <= start_pos_mm) {
        printf("分布测量参数错误：end_pos_mm<=start_pos_mm (%lu <= %lu)\n",
               (unsigned long)end_pos_mm, (unsigned long)start_pos_mm);
        return PARAM_RANGE_ERROR;
    }

    /* 分布测量前不知道油位，这里先置为未找到 */
    float oil_level_mm    = -1.0f;
    bool  oil_level_found = false;

    /* 根据高度范围算出理论最大点数，并限制在 MAX_MEASUREMENT_POINTS 内 */
    float range_mm = (float)(end_pos_mm - start_pos_mm);
    uint32_t max_points_by_range = (uint32_t)(range_mm / (float)step_mm) + 1U;
    if (max_points_by_range == 0) {
        printf("分布测量错误：高度范围过小，无法布点\n");
        return PARAM_RANGE_ERROR;
    }
    if (max_points_by_range > MAX_MEASUREMENT_POINTS) {
        printf("分布测量提示：理论点数 %lu 超过上限 %u，裁剪为上限\n",
               (unsigned long)max_points_by_range, (unsigned int)MAX_MEASUREMENT_POINTS);
        max_points_by_range = MAX_MEASUREMENT_POINTS;
    }

    printf("分布测量开始：起始=%lumm, 最高=%lumm, 间距=%lumm, 理论点数=%lu\n",
           (unsigned long)start_pos_mm,
           (unsigned long)end_pos_mm,
           (unsigned long)step_mm,
           (unsigned long)max_points_by_range);

    /* 先移动到起始点高度 */
    float cur_mm = 0.0f;
    snapshot_sensor_pos_mm(&cur_mm);

    uint32_t ret = NO_ERROR;

    if (cur_mm < (float)start_pos_mm - 0.05f) {
        /* 当前在起始点下方：用“上行到指定位置或空气”函数 */
        Level_StateTypeDef st = OIL;
        printf("先从 %.3fmm 上行到起始点 %lumm\n", cur_mm, (unsigned long)start_pos_mm);
        ret = motorMoveUpToPositionOrAir((float)start_pos_mm, &st);
        CHECK_ERROR(ret);
        if (st == AIR) {
            /* 理论上起始点以下不应该是空气，这里认为异常 */
            snapshot_sensor_pos_mm(&cur_mm);
            printf("到起始点之前已经进入空气，位置=%.3fmm，本次分布测量取消\n", cur_mm);
            return OTHER_UNKNOWN_ERROR;
        }
    } else if (cur_mm > (float)start_pos_mm + 0.05f) {
        /* 当前在起始点上方：直接用绝对位置函数下行，不需要检测空气 */
        printf("当前位置高于起始点，从 %.3fmm 下行到 %lumm\n", cur_mm, (unsigned long)start_pos_mm);
        ret = motorMoveToPositionOneShotWithSpeed((float)start_pos_mm, motorGetDefaultSpeedX100());
        CHECK_ERROR(ret);
    } else {
        printf("当前位置已经在起始点附近，无需调整位置。\n");
    }

    /* 再读一次当前位置，作为正式起点 */
    snapshot_sensor_pos_mm(&cur_mm);
    printf("分布测量起点位置确认：%.3fmm\n", cur_mm);
    g_measurement.device_status.device_state = STATE_WARTSILA_DENSITY_MEASURING;
    /* 起始点如果一开始就在空气中，可以直接结束（说明下面都是空气或空罐） */
    Level_StateTypeDef st0 = determine_level_status();
    if (st0 == AIR) {
        printf("起始点位置传感器在空气中，本次分布测量取消\n");
        return OTHER_UNKNOWN_ERROR;
    }

    /* 循环向上采点 */
    uint32_t valid_points = 0;
    uint32_t sum_temp     = 0;
    uint32_t sum_density  = 0;
    float    last_pos_mm  = cur_mm;

    float target_mm = (float)start_pos_mm;

    for (uint32_t i = 0; i < max_points_by_range; i++) {

        /* 第一个点：已经在起始点，不再移动；后续点：上行到新目标 */
        if (i == 0) {
            printf("分布测量 第1个点：位置=%.3fmm\n", cur_mm);
        } else {
            target_mm += (float)step_mm;
            if (target_mm > (float)end_pos_mm + 0.01f) {
                printf("目标高度 %.3fmm 超出最高限制 %lumm，停止布点\n",
                       target_mm, (unsigned long)end_pos_mm);
                break;
            }

            printf("分布测量 上行到第%lu个点目标位置 %.3fmm\n",
                   (unsigned long)(i + 1), target_mm);

            Level_StateTypeDef st = OIL;
            ret = motorMoveUpToPositionOrAir(target_mm, &st);
            CHECK_ERROR(ret);

            /* motorMoveUpToPositionOrAir 结束后，再读一次实际位置 */
            snapshot_sensor_pos_mm(&cur_mm);

            if (st == AIR) {
                /* 在从上一个点到 target_mm 的过程中，已经提出油面 */
                oil_level_mm    = cur_mm;
                oil_level_found = true;
                printf("在上行过程中检测到空气，认为液位位置=%.3fmm，停止分布测量\n", oil_level_mm);
                break;  /* 不再继续向上，也不采当前点密度 */
            }

            printf("精确寻找密度点位...\r\n");
            ret = motorMoveToPositionOneShotWithSpeed((float)target_mm, motorGetDefaultSpeedX100());
            CHECK_ERROR(ret);
            snapshot_sensor_pos_mm(&cur_mm);

            printf("分布测量 到达第%lu个点实际位置 %.3fmm\n",
                   (unsigned long)(i + 1), cur_mm);
        }
        /* 在密度点采集前再次判断当前是否在油中
                * 如果此时已经在空气中，则把当前位置-100mm当作液位值，结束分布测量
                */
               {
                   Level_StateTypeDef st_cur = determine_level_status();
                   if (st_cur == AIR) {
                       if (!oil_level_found) {
                           oil_level_mm    = cur_mm-100.0f;
                           oil_level_found = true;
                       }
                       printf("分布测量：在采点位置检测到传感器在空气中，液位=%.3fmm，停止测量\n", oil_level_mm);
                       break;  /* 不再采当前点，也不再继续向上 */
                   }
               }
        /* 当前位置在油中，采集一个密度点 */
        if (g_deviceParams.spreadPointHoverTime > 0) {
            HAL_Delay(g_deviceParams.spreadPointHoverTime);
        }

        if (valid_points >= MAX_MEASUREMENT_POINTS) {
            printf("有效点数达到上限 %u，停止采集\n", (unsigned int)MAX_MEASUREMENT_POINTS);
            break;
        }

        DensityMeasurement *pt = &dist->single_density_data[valid_points];
        ret = SinglePoint_ReadSensor(pt);
        if (ret != NO_ERROR) {
            printf("读取单点密度失败 pos=%.3fmm err=%lu\n", cur_mm, (unsigned long)ret);
            return ret;
        }

        /* 在单点结构里记录位置，单位：0.1mm */
        pt->temperature_position = (uint32_t)(cur_mm * 10.0f + 0.5f);

        sum_temp    += pt->temperature;
        sum_density += pt->density;
        valid_points++;
        last_pos_mm = cur_mm;

        printf("分布测量 点%lu：pos=%.3fmm dens=%lu temp=%lu\n",
               (unsigned long)valid_points,
               cur_mm,
               (unsigned long)pt->density,
               (unsigned long)pt->temperature);
    }

    /* 如果整个扫描过程中都没有检测到空气，则认为没有找到油面，报错 */
    if (!oil_level_found) {
        printf("分布测量执行到最高点仍未检测到空气，未找到液位，测量失败\n");
        return OTHER_UNKNOWN_ERROR;
    }

    if (valid_points == 0) {
        printf("本次分布测量没有得到任何有效测点\n");
        return OTHER_UNKNOWN_ERROR;
    }

    /* 最高测点距离油面的判断：过近则舍弃最高点
     * gap_to_surface = (液位高度 - 最高测点高度)
     */
    float gap_to_surface = oil_level_mm - last_pos_mm;

    printf("最高测点高度=%.3fmm, 液位=%.3fmm, 间距=%.3fmm, 限制=%.3fmm\n",
           last_pos_mm, oil_level_mm, gap_to_surface, (float)min_gap_surface);

    if (gap_to_surface < (float)min_gap_surface && valid_points > 0) {
        /* 丢掉最后一个点 */
        DensityMeasurement *last_pt = &dist->single_density_data[valid_points - 1];
        sum_temp    -= last_pt->temperature;
        sum_density -= last_pt->density;
        valid_points--;

        printf("最高测点距离液位小于限制，舍弃该点数据，剩余有效点数=%lu\n",
               (unsigned long)valid_points);
    }

    if (valid_points == 0) {
        printf("舍弃最高点后无有效测点\n");
        return OTHER_UNKNOWN_ERROR;
    }

    /* 统计平均值（原始单位，做简单四舍五入） */
    dist->measurement_points       = valid_points;
    dist->Density_oil_level        = (uint32_t)(oil_level_mm * 10.0f + 0.5f);  /* 0.1mm 单位 */
    dist->average_temperature      = (sum_temp    + valid_points / 2) / valid_points;
    dist->average_density          = (sum_density + valid_points / 2) / valid_points;
    dist->average_standard_density = dist->average_density;
    dist->average_weight_density   = dist->average_density;
    dist->average_vcf20            = 0;

    printf("分布测量完成：有效点数=%lu, 液位=%.1fmm, AvgDensity=%lu(%.3f), AvgTemp=%lu(%.2f)\n",
           (unsigned long)valid_points,
           oil_level_mm,
           (unsigned long)dist->average_density,
           RAW_TO_DENSITY(dist->average_density),
           (unsigned long)dist->average_temperature,
           RAW_TO_TEMP(dist->average_temperature));

    return NO_ERROR;
}

/**
 * @brief 电机向上运行到指定目标位置，途中若检测到传感器进入空气立即停止
 *
 * @param target_mm      目标绝对位置（单位：mm）
 * @param final_state    [可选] 最终状态输出（AIR / OIL），可为 NULL
 *
 * @return NO_ERROR 表示正常结束（到达目标或遇到空气）
 *         其他错误码表示电机或硬件异常
 */
uint32_t motorMoveUpToPositionOrAir(float target_mm, Level_StateTypeDef *final_state)
{
	uint32_t hz = 0;
	uint32_t ret = NO_ERROR;
    if (final_state) {
        *final_state = OIL;
    }

    /* 读取当前高度（mm） */
    float cur_mm;
    snapshot_sensor_pos_mm(&cur_mm);

    printf("motorMoveUpToPositionOrAir: 当前=%.3fmm, 目标=%.3fmm\r\n",
           cur_mm, target_mm);

    /* 如果当前就超过目标，不需要移动 */
    if (cur_mm >= target_mm) {
        printf("当前位置已高于目标点，无需上行。\r\n");
        return NO_ERROR;
    }
    //切换频率模式
    ret = EnableLevelMode();
    CHECK_ERROR(ret);
    printf("motorMoveUpToPositionOrAir: 切换到液位测量模式，等待5秒...\r\n");
    HAL_Delay(5000);
    /* 下发上行运动指令（长度设为足够大） */
    float max_move = target_mm - cur_mm;   // 理论需要跑的距离

    ret = motorMoveNoWaitWithSpeed(3*max_move, MOTOR_DIRECTION_UP, motorGetDefaultSpeedX100());//走三倍距离保证一定会跑到
    CHECK_ERROR(ret);

    /* 进入循环检测：空气 + 到位 + 安全检查 */
    uint32_t start_tick = HAL_GetTick();
    const uint32_t MAX_WAIT_MS = 60*60000;    // 最长等待 60s*60 =1小时，防止死循环

    while (stpr_isMoving(&stepper)) {

        /* 1) 检测空气状态 */
    	//如果传感器是LTD传感器
		if (g_deviceParams.sensorType == LTD_SENSOR) {
	    	ret = DSM_V2_Read_LevelFrequency(&hz);
	    	if (ret != NO_ERROR) {
	    		return ret;  // 读取失败直接返回错误码
	    	}
	    	if (hz == 0 || hz > g_deviceParams.oilLevelFrequency) {
	    		 if (final_state) *final_state = AIR;//读到0或者异常频率认为是空气
	            printf("motorMoveUpToPositionOrAir: 频率检测到到达液面，立即停止电机！\r\n");
	            stpr_stop(&stepper);
	    		break;  // 读到0也返回
	    	}
	    	 HAL_Delay(80);
		}
        Level_StateTypeDef st = determine_level_status();
        if (final_state) *final_state = st;

        if (st == AIR) {
            printf("motorMoveUpToPositionOrAir: 检测到进入空气，立即停止电机！\r\n");
            stpr_stop(&stepper);
            break;
        }

        /* 2) 检测当前位置是否已经到达目标点 */
        snapshot_sensor_pos_mm(&cur_mm);

        if (cur_mm >= target_mm - 0.05f) {   // 加一点浮动允许
            printf("motorMoveUpToPositionOrAir: 已到达目标位置 %.3fmm\r\n", cur_mm);
            stpr_stop(&stepper);
            break;
        }

        /* 3) 其他安全检测 */
        ret = CheckWeightCollision();
        CHECK_ERROR(ret);

        ret = stpr_checkGstat(&stepper);//电机状态检测
        CHECK_ERROR(ret);

        /* 4) 超时保护 */
        if (HAL_GetTick() - start_tick > MAX_WAIT_MS) {
            printf("motorMoveUpToPositionOrAir: 运行超时！\r\n");
            RETURN_ERROR(MOTOR_RUN_TIMEOUT);
        }

        HAL_Delay(80);
    }

    /* 结束后，再读一次最终位置 */
    snapshot_sensor_pos_mm(&cur_mm);
    printf("motorMoveUpToPositionOrAir 结束：最终位置 %.3fmm\r\n", cur_mm);

    return NO_ERROR;
}

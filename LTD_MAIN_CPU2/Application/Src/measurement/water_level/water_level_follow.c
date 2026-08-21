/**
 * @file water_level_follow.c
 * @brief 水位普通/快速闭环跟随、稳定监测和丢失恢复实现。
 */
#include "water_level_internal.h"
#include "fault_manager.h"
#include "system_parameter.h"
#include "motor_ctrl.h"
#include "measure_zero.h"
#include "sensor_service.h"
#include "error_log.h"
#include "AoOutput/ao_output.h"

#include <stdio.h>
#include <stdlib.h>
/* 水位跟随电容采样周期，单位 ms。 */
#define WATER_FOLLOW_SAMPLE_MS          (500U)
/* 接近目标电容时的最小调整步长，单位 mm。 */
#define WATER_FOLLOW_STEP_SMALL_MM      (0.2f)
/* 中等电容偏差时的调整步长，单位 mm。 */
#define WATER_FOLLOW_STEP_MED_MM        (3.0f)
/* 大偏差或饱和保护时的调整步长，单位 mm。 */
#define WATER_FOLLOW_STEP_BIG_MM        (10.0f)
/* 水位电容饱和保护上限，超过该值按异常大电容处理。 */
#define WATER_CAP_SAT_LIMIT             (9999.0f)
/* 持续调整超过该时长仍未稳定时进入稳定监测，单位 ms。 */
#define WATER_FOLLOW_ENTER_TIMEOUT_MS   (180000U)
/* 连续稳定采样达到该次数后进入水位变化监测。 */
#define WATER_FOLLOW_STABLE_MONITOR_COUNT (5U)
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
    /* 水位搜索失步后的恢复策略选择。 */
    WATER_RECOVER_BY_SEARCH = 0, /* 失步后重新执行水位搜索以恢复介质边界。 */
    WATER_RECOVER_BY_STATE_FLIP = 1, /* 失步后翻转介质状态并沿相反方向继续确认。 */
} WaterRecoverStrategy;

/**
 * @brief 水位信号丢失后按配置策略重新搜索或翻转搜索方向。
 *
 * @param strategy 水位丢失后的恢复动作选择。
 * @return 返回重新搜索或翻转方向搜索的原始结果码；NO_ERROR 表示恢复成功，其他值保留命令切换、传感器或运动错误。
 */
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
 *
 * @param target_cap 目标。
 * @return NO_ERROR 表示变化监测按当前结束条件正常退出；STATE_SWITCH、传感器超时、数据失效或水位丢失恢复失败由内部检查返回。
 */
static uint32_t MonitorWaterFollowChange(float target_cap)
{
    uint32_t ret;
    float cap = 0.0f;
    float diff;
    /* 水位滞后电容阈值：稳定监测中偏离目标超过该值后返回跟随循环。 */
    float monitor_diff = WaterLevel_CapRawToFloat(g_deviceParams.water_lag_cap_threshold);

    /* 连续稳定或超时兜底进入监测循环时，统一在这里确认跟随态。 */
    g_measurement.device_status.device_state = STATE_FOLLOW_WATERING;
    printf("水位跟随\t进入稳定监测，置为水位跟随状态\r\n");
    /* 刚进入稳定监测时锁定一次当前水位，后续监测只打印该水位值。 */
    WaterLevel_SyncFromCable();

    while (1)
    {
        ret = SensorService_ReadWaterCapacitance(&cap);
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

/**
 * @brief 以空气电容和配置阈值建立跟随区间，按偏差分级步进；稳定后进入监测，信号丢失时按策略重新搜索。
 *
 * 函数以空气零点电容加 water_cap_threshold 得到目标电容，再用 water_find_cap_threshold 建立上下滞回边界；三个参数均转换为统一浮点电容单位后参与计算。
 * 当前电容高于上边界时上行、低于下边界时下行，偏差越大使用的步进距离越大；每次动作均通过 MotorCtrl_MoveAndWait 阻塞等待完成，然后重新采样形成闭环。
 * 连续位于稳定区达到 WATER_FOLLOW_STABLE_MONITOR_COUNT 后进入不驱动电机的稳定监测；连续调节超过 WATER_FOLLOW_ENTER_TIMEOUT_MS
 * 仍未稳定时也进入监测，避免长期机械往返。
 * 监测到电容偏离目标后返回闭环调节；连续大偏差达到五次则按 recover_strategy 重新搜索水位，成功后清零丢失计数并继续跟随。
 * 测量、运动、重新搜索或命令切换错误原样返回；正常跟随没有主动成功结束点，会持续运行直到收到新命令或出现错误。
 *
 * @param recover_strategy 连续大偏差判定水位丢失后的恢复策略；WATER_RECOVER_BY_SEARCH
 *                         重新执行完整找水，WATER_RECOVER_BY_STATE_FLIP 使用快速状态翻转搜索。
 * @return STATE_SWITCH 表示水位跟随被新命令正常打断；其他非零值为电容读取、电机运动、稳定监测或丢失恢复流程原样传播的错误码；正常跟随持续运行，不以 NO_ERROR 主动结束。
 * @note 本函数包含阻塞式电机等待和 HAL_Delay，只能在测量任务流程调用，不得在中断上下文执行。
 */
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
    target_cap = air_cap + WaterLevel_CapRawToFloat(g_deviceParams.water_cap_threshold);
    follow_band = WaterLevel_CapRawToFloat(g_deviceParams.water_find_cap_threshold);
    th_low = target_cap - follow_band;
    th = target_cap + follow_band;

    printf("水位跟随\t开始\r\n");
    printf("水位跟随\t空气电容=%.1f  目标阈值上限：%.1f  滞回下限：%.1f\r\n",
           air_cap, th, th_low);

    /* ============================ 主循环 ============================ */
    while (1)
    {
        /* ---------- 1. 读取当前水位电容 ---------- */
        ret = SensorService_ReadWaterCapacitance(&cap);
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
            /* WaterLevel_SyncFromCable(); */
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

        if (diff > 0.6f * WaterLevel_CapRawToFloat(g_deviceParams.water_cap_threshold))
        {
            step_mm = WATER_FOLLOW_STEP_BIG_MM;
        }
        else if (diff > 0.1f * WaterLevel_CapRawToFloat(g_deviceParams.water_cap_threshold))
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
            float th_span = WaterLevel_CapRawToFloat(g_deviceParams.water_cap_threshold);

/* if ((diff > 0.8f * th_span) && (cap_delta < 1.0f)) */
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

/**
 * @brief 以快速步进策略跟随水位并收敛到电容目标区间。
 * @return 返回快速水位跟随循环的最终状态；STATE_SWITCH 表示新命令打断，其他非零值为电容读取、目标区间收敛或电机运动错误。
 */
uint32_t FollowWaterLevel_fast(void)
{
    uint32_t ret = FollowWaterLevelCore(WATER_RECOVER_BY_STATE_FLIP);

    if (ret != NO_ERROR) {
        AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL);
    }
    return ret;
}

/**
 * @brief 执行水位跟随主流程并发布最终测量状态。
 * @return 返回所选水位跟随实现的结果；STATE_SWITCH 表示正常命令打断，其他非零值为水位定位、电容读取、运动、稳定性或丢失恢复错误。
 */
uint32_t FollowWaterLevel(void)
{
    uint32_t ret = FollowWaterLevelCore(WATER_RECOVER_BY_SEARCH);

    if (ret != NO_ERROR) {
        AoOutput_InvalidateProcessSample(AO_PROCESS_SOURCE_WATER_LEVEL);
    }
    return ret;
}

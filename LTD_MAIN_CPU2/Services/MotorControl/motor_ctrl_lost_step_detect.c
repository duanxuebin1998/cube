#include "motor_ctrl_internal.h"

/**
 * @file motor_ctrl_lost_step_detect.c
 * @brief 运动过程丢步检测、报警抑制和无检测运动日志。
 *
 * 丢步检测通过固定周期采样位置，比较最近窗口内的平均速度与设定速度，判断
 * 电机是否存在疑似空转或未跟随。无检测运动日志也放在这里，便于维护调试口径。
 */

/* ===================== 私有类型/状态 ===================== */

/* 常规丢步检测状态保持原有结构和语义，供现有测量流程继续使用。 */
static int32_t pos_buf[LOST_STEP_WINDOW]; /* 丢步检测窗口内各采样时刻的业务位置，单位为 0.1 mm；索引与另外三组环形缓冲严格对齐。 */
static int32_t motor_pos_buf[LOST_STEP_WINDOW]; /* 丢步检测窗口内由电机步数模型换算的移动距离，单位为 0.1 mm；用于与编码轮实际位移比较跟随比例。 */
static int32_t encoder_pos_buf[LOST_STEP_WINDOW]; /* 丢步检测窗口内编码轮换算的尺带长度，单位为 0.1 mm；与 motor_pos_buf 的同索引样本配对判断未跟随。 */
static uint32_t tick_buf[LOST_STEP_WINDOW]; /* 丢步检测窗口内每个位置样本对应的 HAL 毫秒节拍；与三组位置缓冲同索引，用于计算窗口时间跨度和速度。 */
static int write_idx = 0; /* 常规丢步检测采样环形缓冲区的下一写入索引。 */
static int samples = 0; /* 常规丢步检测缓冲区当前已有的有效样本数。 */
static uint32_t last_check_tick = 0; /* 常规丢步检测最近一次执行窗口统计的 HAL 毫秒节拍。 */
static uint32_t last_alarm_tick = 0; /* 常规丢步告警最近一次上报的 HAL 毫秒节拍。 */

/* 方式5按指令方向累计有效前进量和期望位移，避免换向抵消及往返抖动虚增。 */
typedef struct {
    uint64_t pos_progress_buf[LOST_STEP_WINDOW]; /* 各窗口快照的业务位置累计有效前进量，单位0.1mm。 */
    uint64_t motor_progress_buf[LOST_STEP_WINDOW]; /* 各窗口快照的电机模型累计有效前进量，单位0.1mm。 */
    uint64_t encoder_progress_buf[LOST_STEP_WINDOW]; /* 各窗口快照的编码轮累计有效前进量，单位0.1mm。 */
    double expected_distance_buf[LOST_STEP_WINDOW]; /* 各窗口快照的累计期望位移，单位mm。 */
    uint32_t tick_buf[LOST_STEP_WINDOW]; /* 各累计量快照对应的HAL毫秒节拍。 */
    uint64_t pos_progress_01mm; /* 业务位置跨方向段累计的有效前进量，单位0.1mm。 */
    uint64_t motor_progress_01mm; /* 电机模型跨方向段累计的有效前进量，单位0.1mm。 */
    uint64_t encoder_progress_01mm; /* 编码轮跨方向段累计的有效前进量，单位0.1mm。 */
    double expected_distance_mm; /* 各有效指令时段累计的期望位移，单位mm。 */
    int32_t pos_frontier_01mm; /* 当前指令方向段内业务位置已到达的最远前沿，单位0.1mm。 */
    int32_t motor_frontier_01mm; /* 当前指令方向段内电机模型已到达的最远前沿，单位0.1mm。 */
    int32_t encoder_frontier_01mm; /* 当前指令方向段内编码轮已到达的最远前沿，单位0.1mm。 */
    uint32_t last_update_tick; /* 上一次控制周期结算的HAL毫秒节拍。 */
    uint32_t last_sample_tick; /* 上一次保存滚动窗口快照的HAL毫秒节拍。 */
    uint32_t last_alarm_tick; /* 最近一次方式5丢步告警的HAL毫秒节拍。 */
    float last_command_speed_m_min; /* 上一调用后生效的指令速度，单位m/min。 */
    int write_idx; /* 下一次滚动窗口快照的写入索引。 */
    int samples; /* 滚动窗口中已有的有效快照数。 */
    int8_t last_command_dir; /* 上一调用后生效的MOTOR_DIRECTION_UP/DOWN。 */
    uint8_t tracking_active; /* 非0表示方式5累计基线已经建立。 */
} Method5LostStepState;

static Method5LostStepState s_method5_lost_step;
static int32_t s_bottom_reference_01mm = -100000000; /* Application 最近同步的罐底尺带参考，单位 0.1 mm；负初值表示尚未建立。 */
#ifndef NODETECT_LOG_PERIOD_MS
#define NODETECT_LOG_PERIOD_MS        100u   /* 打印周期：100ms */
#endif
#ifndef NODETECT_GYRO_PERIOD_MS
#define NODETECT_GYRO_PERIOD_MS       300u   /* 陀螺仪读取周期：300ms */
#endif
#ifndef NODETECT_DENS_PERIOD_MS
#define NODETECT_DENS_PERIOD_MS       500u   /* 密度读取周期：500ms */
#endif
#ifndef LOST_STEP_MIN_MOTOR_DELTA_01MM
#define LOST_STEP_MIN_MOTOR_DELTA_01MM 50    /* 电机窗口内至少移动5mm，才判定编码轮未跟随 */
#endif
#ifndef LOST_STEP_ENCODER_FOLLOW_RATIO_PERCENT
#define LOST_STEP_ENCODER_FOLLOW_RATIO_PERCENT 20U /* 编码轮位移低于电机位移20%，认为未跟随 */
#endif

/* ===================== 对外接口 ===================== */

/**
 * @brief 更新丢步检测使用的罐底尺带参考。
 * @param cable_length_01mm 罐底对应的尺带长度，单位 0.1 mm。
 * @note 该接口只复制数值，不读取 Application 状态，也不访问硬件。
 */
void MotorCtrl_SetBottomReference01mm(int32_t cable_length_01mm)
{
    s_bottom_reference_01mm = cable_length_01mm;
}

/**
 * @brief 把当前位置设为方式5当前方向段的进度前沿。
 * @note 用于初始化、停机间隔和方向切换，只更新内存状态，不访问硬件。
 */
static void MotorCtrl_SetMethod5ProgressFrontiers(int32_t current_pos_01mm,
                                                  int32_t motor_pos_01mm,
                                                  int32_t encoder_pos_01mm)
{
    s_method5_lost_step.pos_frontier_01mm = current_pos_01mm;
    s_method5_lost_step.motor_frontier_01mm = motor_pos_01mm;
    s_method5_lost_step.encoder_frontier_01mm = encoder_pos_01mm;
}

/**
 * @brief 累计一个信号沿已下发方向刷新最远前沿得到的有效前进量。
 * @param current_01mm 当前信号值，单位0.1mm。
 * @param frontier_01mm 当前方向段已到达的最远前沿，单位0.1mm。
 * @param command_dir 覆盖本时段的MOTOR_DIRECTION_UP/DOWN。
 * @param increases_when_down 非0表示下行时该信号增大，0表示下行时该信号减小。
 * @return 本次新增的有效前进量，单位0.1mm；反向回退或未越过前沿时返回0。
 * @note 前沿只沿指令方向推进，因此原地往返抖动不会被重复累计。
 */
static uint64_t MotorCtrl_UpdateMethod5ForwardProgress01mm(int32_t current_01mm,
                                                           int32_t *frontier_01mm,
                                                           int command_dir,
                                                           uint8_t increases_when_down)
{
    int64_t directed_delta = (int64_t)current_01mm - (int64_t)(*frontier_01mm);

    if (command_dir == MOTOR_DIRECTION_UP) {
        directed_delta = -directed_delta;
    }
    if (increases_when_down == 0U) {
        directed_delta = -directed_delta;
    }
    if (directed_delta <= 0) {
        return 0U;
    }

    *frontier_01mm = current_01mm;
    return (uint64_t)directed_delta;
}

/**
 * @brief 按上一条方式5指令方向累计业务位置、电机模型和编码轮有效前进量。
 * @note sensor_position下行减小，motor_distance和编码轮尺带长度下行增大。
 */
static void MotorCtrl_AccumulateMethod5ForwardProgress(int32_t current_pos_01mm,
                                                       int32_t motor_pos_01mm,
                                                       int32_t encoder_pos_01mm,
                                                       int command_dir)
{
    s_method5_lost_step.pos_progress_01mm +=
        MotorCtrl_UpdateMethod5ForwardProgress01mm(
                current_pos_01mm,
                &s_method5_lost_step.pos_frontier_01mm,
                command_dir,
                0U);
    s_method5_lost_step.motor_progress_01mm +=
        MotorCtrl_UpdateMethod5ForwardProgress01mm(
                motor_pos_01mm,
                &s_method5_lost_step.motor_frontier_01mm,
                command_dir,
                1U);
    s_method5_lost_step.encoder_progress_01mm +=
        MotorCtrl_UpdateMethod5ForwardProgress01mm(
                encoder_pos_01mm,
                &s_method5_lost_step.encoder_frontier_01mm,
                command_dir,
                1U);
}

/**
 * @brief 清空方式5专用丢步检测状态。
 * @note 只重置内存状态，不访问硬件、不打印，也不改变常规检测窗口。
 */
static void MotorCtrl_ResetMethod5LostStepState(void)
{
    s_method5_lost_step = (Method5LostStepState){0};
}

/**
 * @brief 保存方式5累计有效前进量和期望位移快照，形成约5秒滚动窗口。
 * @note 由任务上下文中的方式5检测入口调用，不访问硬件。
 */
static void MotorCtrl_StoreMethod5LostStepSnapshot(uint32_t now)
{
    int index = s_method5_lost_step.write_idx;

    s_method5_lost_step.pos_progress_buf[index] = s_method5_lost_step.pos_progress_01mm;
    s_method5_lost_step.motor_progress_buf[index] = s_method5_lost_step.motor_progress_01mm;
    s_method5_lost_step.encoder_progress_buf[index] = s_method5_lost_step.encoder_progress_01mm;
    s_method5_lost_step.expected_distance_buf[index] = s_method5_lost_step.expected_distance_mm;
    s_method5_lost_step.tick_buf[index] = now;
    s_method5_lost_step.write_idx = (index + 1) % LOST_STEP_WINDOW;
    if (s_method5_lost_step.samples < LOST_STEP_WINDOW) {
        s_method5_lost_step.samples++;
    }
}

/**
 * @brief 初始化常规与方式5丢步检测窗口。
 * @note 在新的独立运动阶段开始前由任务上下文调用；只清空检测状态。
 */
void MotorCtrl_LostStepInit(void)
{
    write_idx = 0;
    samples = 0;
    last_check_tick = 0;
    last_alarm_tick = 0;

    for (int i = 0; i < LOST_STEP_WINDOW; i++) {
        pos_buf[i]  = 0;
        motor_pos_buf[i] = 0;
        encoder_pos_buf[i] = 0;
        tick_buf[i] = 0;
    }
    MotorCtrl_ResetMethod5LostStepState();
}

/**
 * @brief 按固定周期执行丢步检测。
 *
 * 函数内部自带采样节流，比较最近窗口平均速度与设定速度。
 * @param currentPos 当前业务位置，单位 0.1mm。
 * @return NO_ERROR 或疑似丢步错误码。
 */
uint32_t MotorCtrl_CheckLostStepAutoTiming(int32_t currentPos)
{
    uint32_t now = HAL_GetTick();

    /* 每1秒检查一次 */
    if ((now - last_check_tick) < LOST_STEP_INTERVAL_MS) {
        return NO_ERROR;
    }

    last_check_tick = now;

    MotorCtrl_RefreshDebugDrumState();

    /* 写入当前位置和时间戳 */
    pos_buf[write_idx]  = currentPos;   /* 单位：0.1mm */
    motor_pos_buf[write_idx] = g_measurement.debug_data.motor_distance;
    encoder_pos_buf[write_idx] = encoder_get_cable_length_01mm();
    tick_buf[write_idx] = now;          /* 单位：ms */
    write_idx = (write_idx + 1) % LOST_STEP_WINDOW;

    if (samples < LOST_STEP_WINDOW) {
        samples++;
    }

    /* 样本不足，先不判定 */
    if (samples < LOST_STEP_WINDOW) {
        return NO_ERROR;
    }

    /* oldest_idx 指向最旧样本 */
    int oldest_idx = write_idx;
    int newest_idx = (write_idx + LOST_STEP_WINDOW - 1) % LOST_STEP_WINDOW;

    int32_t pos_old = pos_buf[oldest_idx];
    int32_t pos_new = pos_buf[newest_idx];
    int32_t motor_old = motor_pos_buf[oldest_idx];
    int32_t motor_new = motor_pos_buf[newest_idx];
    int32_t encoder_old = encoder_pos_buf[oldest_idx];
    int32_t encoder_new = encoder_pos_buf[newest_idx];
    uint32_t tick_old = tick_buf[oldest_idx];
    uint32_t tick_new = tick_buf[newest_idx];

    if (tick_new <= tick_old) {
        return NO_ERROR;
    }

    /* 最近5秒总位移（绝对值），单位：0.1mm */
    int32_t delta_pos_01mm = pos_new - pos_old;
    if (delta_pos_01mm < 0) {
        delta_pos_01mm = -delta_pos_01mm;
    }
    int32_t motor_delta_01mm = motor_new - motor_old;
    if (motor_delta_01mm < 0) {
        motor_delta_01mm = -motor_delta_01mm;
    }
    int32_t encoder_delta_01mm = encoder_new - encoder_old;
    if (encoder_delta_01mm < 0) {
        encoder_delta_01mm = -encoder_delta_01mm;
    }

    /* 时间差，单位：s */
    float dt_s = (float)(tick_new - tick_old) / 1000.0f;
    if (dt_s < 0.001f) {
        return NO_ERROR;
    }

    /* 平均速度，单位：mm/s */
    float avg_speed_mm_s = ((float)delta_pos_01mm / 10.0f) / dt_s;

    /* 设定速度，单位：mm/s
       motor_speed单位=0.01m/min
       => mm/s = x100 / 6
     */
    float set_speed_m_min = MotorDriver_GetSpeedSetpointMMin();
    float set_speed_mm_s  = set_speed_m_min * 1000.0f / 60.0f;
    /* 速度阈值 = max(设定速度的20%, 绝对下限0.5mm/s) */
    float speed_threshold_mm_s =
        set_speed_mm_s * ((float)LOST_STEP_SPEED_RATIO_PERCENT / 100.0f);

    if (speed_threshold_mm_s < LOST_STEP_MIN_AVG_SPEED_MM_S) {
        speed_threshold_mm_s = LOST_STEP_MIN_AVG_SPEED_MM_S;
    }


    /* 编码轮跟随性检查：电机明显运动但编码轮基本不动时，直接判定异常。
     * 该检查不依赖罐底位置，避免粗找阶段 s_bottom_reference_01mm 尚未建立导致漏判。 */
    if (motor_delta_01mm >= LOST_STEP_MIN_MOTOR_DELTA_01MM) {
        int32_t encoder_min_delta_01mm =
            (motor_delta_01mm * (int32_t)LOST_STEP_ENCODER_FOLLOW_RATIO_PERCENT) / 100;
        if (encoder_min_delta_01mm < 10) {
            encoder_min_delta_01mm = 10;
        }

        if (encoder_delta_01mm < encoder_min_delta_01mm) {
            if ((now - last_alarm_tick) >= LOST_STEP_SUPPRESS_MS) {
                last_alarm_tick = now;
                printf("编码轮未跟随报警 | 最近%.1f s电机尺带变化=%.1fmm | 编码轮尺带变化=%.1fmm | 最小跟随=%.1fmm\r\n",
                       dt_s,
                       (double)motor_delta_01mm * 0.1,
                       (double)encoder_delta_01mm * 0.1,
                       (double)encoder_min_delta_01mm * 0.1);
                return ENCODER_LOST_STEP;
            }
        }
    }
    /* 中间区域才启用丢步判定，避免靠近零点/罐底误判 */
    if ((g_measurement.debug_data.cable_length > 1000) &&
        (g_measurement.debug_data.cable_length < s_bottom_reference_01mm - 1000)) {

        if (avg_speed_mm_s < speed_threshold_mm_s) {
            if ((now - last_alarm_tick) >= LOST_STEP_SUPPRESS_MS) {
                last_alarm_tick = now;

                printf("电机丢步报警 | 当前位置=%.1f mm | 最近%.1f s平均速度=%.3f mm/s < 阈值=%.3f mm/s | 设定=%.3f mm/s\r\n",
                       (float)currentPos / 10.0f,
                       dt_s,
                       avg_speed_mm_s,
                       speed_threshold_mm_s,
                       set_speed_mm_s);

                if (MotorCtrl_IsPositionSourceMotor()) {
                    printf("当前为电机步进位置源，忽略编码器丢步异常\r\n");
                    return NO_ERROR;
                }
                return ENCODER_LOST_STEP;
            }
        } else {
            printf("丢步检测正常 | 当前位置=%.1f mm | 最近%.1f s平均速度=%.3f mm/s | 阈值=%.3f mm/s | 设定=%.3f mm/s\r\n",
                   (float)currentPos / 10.0f,
                   dt_s,
                   avg_speed_mm_s,
                   speed_threshold_mm_s,
                   set_speed_mm_s);
        }
    }

    return NO_ERROR;
}

/**
 * @brief 按方式5已下发的有效指令方向和速度执行丢步检测。
 *
 * @param currentPos 当前业务位置，单位0.1mm。
 * @param command_dir 当前运动阶段已下发的有效方向，使用MOTOR_DIRECTION_UP/DOWN。
 * @param effective_command_speed_m_min 当前运动阶段已下发的有效指令速度，单位m/min。
 * @return NO_ERROR或疑似丢步错误码；方向无效，或速度为负数、非数、超上限时返回PARAM_RANGE_ERROR。
 * @note 传入0.0表示当前没有运动指令，command_dir仍传停止前的有效方向，用于换向慢停前结束上一速度区间。
 * @note 仅由任务上下文在位置轮询成功后调用；指令变化前先结算旧指令，变化后立即传入新指令。
 *       函数按上一条有效指令覆盖的真实时段累计期望位移，并按方向段最远前沿累计有效进度，
 *       不重复计算往返抖动，也不套用常规速度0.5mm/s绝对下限。
 */
uint32_t MotorCtrl_CheckLostStepAutoTimingWithSpeed(int32_t currentPos,
                                                   int command_dir,
                                                   float effective_command_speed_m_min)
{
    uint32_t now;
    uint32_t elapsed_ms;
    int32_t motor_pos_01mm;
    int32_t encoder_pos_01mm;

    if (((command_dir != MOTOR_DIRECTION_UP) &&
         (command_dir != MOTOR_DIRECTION_DOWN)) ||
        (!(effective_command_speed_m_min >= 0.0f)) ||
        (effective_command_speed_m_min > ((float)MOTOR_LINEAR_SPEED_MAX_X100 / 100.0f))) {
        return PARAM_RANGE_ERROR;
    }

    now = HAL_GetTick();
    motor_pos_01mm = g_measurement.debug_data.motor_distance;
    encoder_pos_01mm = encoder_get_cable_length_01mm();

    if (s_method5_lost_step.tracking_active == 0U) {
        s_method5_lost_step.tracking_active = 1U;
        MotorCtrl_SetMethod5ProgressFrontiers(currentPos,
                                              motor_pos_01mm,
                                              encoder_pos_01mm);
        s_method5_lost_step.last_update_tick = now;
        s_method5_lost_step.last_sample_tick = now;
        s_method5_lost_step.last_command_speed_m_min = effective_command_speed_m_min;
        s_method5_lost_step.last_command_dir = (int8_t)command_dir;
        MotorCtrl_StoreMethod5LostStepSnapshot(now);
        return NO_ERROR;
    }

    elapsed_ms = now - s_method5_lost_step.last_update_tick;
    s_method5_lost_step.expected_distance_mm +=
        (double)s_method5_lost_step.last_command_speed_m_min * (double)elapsed_ms / 60.0;

    if (s_method5_lost_step.last_command_speed_m_min > 0.0f) {
        MotorCtrl_AccumulateMethod5ForwardProgress(
                currentPos,
                motor_pos_01mm,
                encoder_pos_01mm,
                (int)s_method5_lost_step.last_command_dir);
    } else {
        /* 停机时段不计入进度，并用最新位置吸收减速余量和静止抖动。 */
        MotorCtrl_SetMethod5ProgressFrontiers(currentPos,
                                              motor_pos_01mm,
                                              encoder_pos_01mm);
    }

    if (command_dir != (int)s_method5_lost_step.last_command_dir) {
        /* 新方向从当前点建立独立前沿，保留此前累计量但不跨方向比较位置。 */
        MotorCtrl_SetMethod5ProgressFrontiers(currentPos,
                                              motor_pos_01mm,
                                              encoder_pos_01mm);
    }

    s_method5_lost_step.last_update_tick = now;
    s_method5_lost_step.last_command_speed_m_min = effective_command_speed_m_min;
    s_method5_lost_step.last_command_dir = (int8_t)command_dir;

    if ((now - s_method5_lost_step.last_sample_tick) < LOST_STEP_INTERVAL_MS) {
        return NO_ERROR;
    }
    s_method5_lost_step.last_sample_tick = now;
    MotorCtrl_StoreMethod5LostStepSnapshot(now);
    if (s_method5_lost_step.samples < LOST_STEP_WINDOW) {
        return NO_ERROR;
    }

    {
        int oldest_idx = s_method5_lost_step.write_idx;
        int newest_idx = (s_method5_lost_step.write_idx + LOST_STEP_WINDOW - 1) % LOST_STEP_WINDOW;
        uint32_t window_ms = s_method5_lost_step.tick_buf[newest_idx] -
                             s_method5_lost_step.tick_buf[oldest_idx];
        uint64_t pos_progress_01mm = s_method5_lost_step.pos_progress_buf[newest_idx] -
                                     s_method5_lost_step.pos_progress_buf[oldest_idx];
        uint64_t motor_progress_01mm = s_method5_lost_step.motor_progress_buf[newest_idx] -
                                       s_method5_lost_step.motor_progress_buf[oldest_idx];
        uint64_t encoder_progress_01mm = s_method5_lost_step.encoder_progress_buf[newest_idx] -
                                         s_method5_lost_step.encoder_progress_buf[oldest_idx];
        double expected_distance_mm = s_method5_lost_step.expected_distance_buf[newest_idx] -
                                      s_method5_lost_step.expected_distance_buf[oldest_idx];
        double dt_s;
        double avg_speed_mm_s;
        double effective_set_speed_mm_s;
        double speed_threshold_mm_s;

        if ((window_ms == 0U) || (!(expected_distance_mm > 0.0))) {
            return NO_ERROR;
        }
        dt_s = (double)window_ms / 1000.0;
        avg_speed_mm_s = ((double)pos_progress_01mm / 10.0) / dt_s;
        effective_set_speed_mm_s = expected_distance_mm / dt_s;
        speed_threshold_mm_s = effective_set_speed_mm_s *
                               ((double)LOST_STEP_SPEED_RATIO_PERCENT / 100.0);

        /* 编码轮跟随判定使用各方向段有效前进量，换向保留证据且抖动不重复累计。 */
        if (motor_progress_01mm >= (uint64_t)LOST_STEP_MIN_MOTOR_DELTA_01MM) {
            uint64_t encoder_min_progress_01mm =
                (motor_progress_01mm * (uint64_t)LOST_STEP_ENCODER_FOLLOW_RATIO_PERCENT) / 100U;
            if (encoder_min_progress_01mm < 10U) {
                encoder_min_progress_01mm = 10U;
            }
            if ((encoder_progress_01mm < encoder_min_progress_01mm) &&
                ((now - s_method5_lost_step.last_alarm_tick) >= LOST_STEP_SUPPRESS_MS)) {
                s_method5_lost_step.last_alarm_tick = now;
                printf("编码轮未跟随报警 | 最近%.1f s电机有效前进=%.1fmm | 编码轮有效前进=%.1fmm | 最小跟随=%.1fmm | 速度源=方式5有效指令\r\n",
                       dt_s,
                       (double)motor_progress_01mm * 0.1,
                       (double)encoder_progress_01mm * 0.1,
                       (double)encoder_min_progress_01mm * 0.1);
                return ENCODER_LOST_STEP;
            }
        }

        /* 中间区域才启用进度判定，方式5不叠加常规速度绝对下限。 */
        if ((g_measurement.debug_data.cable_length > 1000) &&
            (g_measurement.debug_data.cable_length < s_bottom_reference_01mm - 1000)) {
            if (avg_speed_mm_s < speed_threshold_mm_s) {
                if ((now - s_method5_lost_step.last_alarm_tick) >= LOST_STEP_SUPPRESS_MS) {
                    s_method5_lost_step.last_alarm_tick = now;
                    printf("电机丢步报警 | 当前位置=%.1f mm | 最近%.1f s有效前进=%.1f mm | 平均速度=%.3f mm/s < 阈值=%.3f mm/s | 有效指令=%.3f mm/s | 期望位移=%.3f mm\r\n",
                           (double)currentPos / 10.0,
                           dt_s,
                           (double)pos_progress_01mm * 0.1,
                           avg_speed_mm_s,
                           speed_threshold_mm_s,
                           effective_set_speed_mm_s,
                           expected_distance_mm);
                    if (MotorCtrl_IsPositionSourceMotor()) {
                        printf("当前为电机步进位置源，忽略编码器丢步异常\r\n");
                        return NO_ERROR;
                    }
                    return ENCODER_LOST_STEP;
                }
            } else {
                printf("丢步检测正常 | 当前位置=%.1f mm | 最近%.1f s有效前进=%.1f mm | 平均速度=%.3f mm/s | 阈值=%.3f mm/s | 有效指令=%.3f mm/s | 期望位移=%.3f mm\r\n",
                       (double)currentPos / 10.0,
                       dt_s,
                       (double)pos_progress_01mm * 0.1,
                       avg_speed_mm_s,
                       speed_threshold_mm_s,
                       effective_set_speed_mm_s,
                       expected_distance_mm);
            }
        }
    }

    return NO_ERROR;
}

/* ===================== 内部跨文件接口 ===================== */

/**
 * @brief 无检测运动过程中的周期性调试日志。
 *
 * 用于维护模式下观察位置、陀螺仪和密度数据，不参与安全停机判断。
 */
void MotorLostStep_NoDetectRuntimeLogUpdate(void)
{
    static uint32_t last_log_tick  = 0;
    static uint32_t last_den_tick  = 0;

    uint32_t now = HAL_GetTick();

    /* 1) 位置/尺带/扭力 */
    g_measurement.debug_data.current_weight = weight_parament.current_weight;
    g_measurement.debug_data.current_encoder_value = -g_encoder_count;

    /* 2) 陀螺仪 */

/* if ((now - last_gyro_tick) >= NODETECT_GYRO_PERIOD_MS) { */
/* last_gyro_tick = now; */
/* float ax = 0.0f, ay = 0.0f; */
/* uint32_t ret = Read_Gyro_Angle(&ax, &ay); */
/* if (ret == NO_ERROR) { */
/* g_measurement.debug_data.angle_x = (int32_t)(ax * 100.0f); */
/* g_measurement.debug_data.angle_y = (int32_t)(ay * 100.0f); */
/* } */
/* } */

    /* 3) 密度/温度/频率 */
    if ((now - last_den_tick) >= NODETECT_DENS_PERIOD_MS) {
        float f = 0.0f, d = 0.0f, t = 0.0f;
        uint32_t ret = SensorService_ReadDensity(&f, &d, &t);
        /* 下一周期从本次阻塞读取结束后计时，避免慢事务结束后立即再次占用 UART6。 */
        last_den_tick = HAL_GetTick();
        (void)f; (void)d; (void)t;
        if (ret == NO_ERROR) {
            /* SensorService_ReadDensity 内部写 debug_data.temperature/frequency 等 */
        }
    }

    /* 4) 节流打印 */
    if ((now - last_log_tick) < NODETECT_LOG_PERIOD_MS) {
        return;
    }
    last_log_tick = now;

    printf("无检测运行 | 方向：%lu 扭力=%lu 距离零点：%ld(0.1mm) 罐底距离=%ld "
           "| X轴（X100）=%ld Y轴（X100）=%ld | 密度=%lu 温度=%lu 频率=%lu\r\n",
           (unsigned long)g_measurement.debug_data.motor_state,
           (unsigned long)g_measurement.debug_data.current_weight,
           (long)g_measurement.debug_data.cable_length,
           (long)g_measurement.debug_data.sensor_position,
           (long)g_measurement.debug_data.angle_x,
           (long)g_measurement.debug_data.angle_y,
           (unsigned long)g_measurement.single_point_monitoring.density,
           (unsigned long)g_measurement.single_point_monitoring.temperature,
           (unsigned long)g_measurement.debug_data.frequency);
}

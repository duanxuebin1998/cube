#include "motor_ctrl_internal.h"

/**
 * @file motor_ctrl_lost_step_detect.c
 * @brief 运动过程丢步检测、报警抑制和无检测运动日志。
 *
 * 丢步检测通过固定周期采样位置，比较最近窗口内的平均速度与设定速度，判断
 * 电机是否存在疑似空转或未跟随。无检测运动日志也放在这里，便于维护调试口径。
 */

/* ===================== 私有类型/状态 ===================== */

/* 丢步检测环形缓冲区：记录最近位置采样，单位 0.1mm。 */
static int32_t pos_buf[LOST_STEP_WINDOW]; /* 丢步检测窗口内各采样时刻的业务位置，单位为 0.1 mm；索引与另外三组环形缓冲严格对齐。 */
static int32_t motor_pos_buf[LOST_STEP_WINDOW]; /* 丢步检测窗口内由电机步数模型换算的移动距离，单位为 0.1 mm；用于与编码轮实际位移比较跟随比例。 */
static int32_t encoder_pos_buf[LOST_STEP_WINDOW]; /* 丢步检测窗口内编码轮换算的尺带长度，单位为 0.1 mm；与 motor_pos_buf 的同索引样本配对判断未跟随。 */
static uint32_t tick_buf[LOST_STEP_WINDOW]; /* 丢步检测窗口内每个位置样本对应的 HAL 毫秒节拍；与三组位置缓冲同索引，用于计算窗口时间跨度和速度。 */
static int write_idx = 0; /* 失步检测采样环形缓冲区的下一写入索引。 */
static int samples = 0; /* 失步检测缓冲区当前已有的有效样本数。 */
static uint32_t last_check_tick = 0; /* 失步检测最近一次执行窗口统计的 HAL 毫秒节拍。 */
static uint32_t last_alarm_tick = 0; /* 失步告警最近一次上报的 HAL 毫秒节拍，用于限制重复告警频率。 */
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
 * @brief 初始化丢步检测窗口。
 *
 * 每次新的运动阶段开始前调用，清空位置采样、时间戳和报警抑制计时。
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

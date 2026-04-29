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
static int32_t pos_buf[LOST_STEP_WINDOW];
static uint32_t tick_buf[LOST_STEP_WINDOW];
static int write_idx = 0;
static int samples = 0;
static uint32_t last_check_tick = 0;
static uint32_t last_alarm_tick = 0;
#ifndef NODETECT_LOG_PERIOD_MS
#define NODETECT_LOG_PERIOD_MS        100u   /* 打印周期：100ms */
#endif
#ifndef NODETECT_GYRO_PERIOD_MS
#define NODETECT_GYRO_PERIOD_MS       300u   /* 陀螺仪读取周期：300ms */
#endif
#ifndef NODETECT_DENS_PERIOD_MS
#define NODETECT_DENS_PERIOD_MS       500u   /* 密度读取周期：500ms */
#endif

/* ===================== 对外接口 ===================== */

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

    /* 写入当前位置和时间戳 */
    pos_buf[write_idx]  = currentPos;   /* 单位：0.1mm */
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

    /* 中间区域才启用丢步判定，避免靠近零点/罐底误判 */
    if ((g_measurement.debug_data.cable_length > 1000) &&
        (g_measurement.debug_data.cable_length < bottom_value - 1000)) {

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
                    printf("当前为电机步进位置源，忽略编码器丢步错误\r\n");
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

    /* 1) 位置/尺带/称重 */
    g_measurement.debug_data.current_weight = weight_parament.current_weight;
    g_measurement.debug_data.current_encoder_value = -g_encoder_count;

    /* 2) 陀螺仪 */

//    if ((now - last_gyro_tick) >= NODETECT_GYRO_PERIOD_MS) {
//        last_gyro_tick = now;
//        float ax = 0.0f, ay = 0.0f;
//        uint32_t ret = Read_Gyro_Angle(&ax, &ay);
//        if (ret == NO_ERROR) {
//            g_measurement.debug_data.angle_x = (int32_t)(ax * 100.0f);
//            g_measurement.debug_data.angle_y = (int32_t)(ay * 100.0f);
//        }
//    }

    /* 3) 密度/温度/频率 */
    if ((now - last_den_tick) >= NODETECT_DENS_PERIOD_MS) {
        last_den_tick = now;
        float f = 0.0f, d = 0.0f, t = 0.0f;
        uint32_t ret = Read_Density(&f, &d, &t);
        (void)f; (void)d; (void)t;
        if (ret == NO_ERROR) {
            /* Read_Density 内部写 debug_data.temperature/frequency 等 */
        }
    }

    /* 4) 节流打印 */
    if ((now - last_log_tick) < NODETECT_LOG_PERIOD_MS) {
        return;
    }
    last_log_tick = now;

    printf("无检测运行 | 方向=%lu 称重=%lu 距离零点=%ld(0.1mm) 罐底距离=%ld "
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

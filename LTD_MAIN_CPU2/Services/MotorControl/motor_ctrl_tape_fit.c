#include "motor_ctrl_internal.h"

/**
 * @file motor_ctrl_tape_fit.c
 * @brief 电机侧尺带拟合 TFIT 的采样、求解、应用和零点基准清理。
 *
 * TFIT 使用电机 XACTUAL 与编码轮长度样本拟合首圈周长和尺带厚度。
 * 本文件只维护拟合过程的 RAM 状态；参数写入仍通过设备参数保存接口完成。
 */

/* ===================== 私有类型/状态 ===================== */

/* TFIT 采样和拟合结果缓存，仅在本文件内部使用，断电后丢失。 */
static MotorTapeFitSample s_motor_tape_fit_samples[MOTOR_TAPE_FIT_MAX_SAMPLES];
static uint16_t s_motor_tape_fit_count = 0;
static bool s_motor_tape_fit_enabled = false;
static int32_t s_motor_tape_fit_last_step = INT32_MIN;
static MotorTapeFitResult s_motor_tape_fit_result = {0};
static bool s_motor_tape_fit_result_is_local = false;
static bool s_motor_tape_fit_local_origin_valid = false;
static int32_t s_motor_tape_fit_origin_step = 0;
static int32_t s_motor_tape_fit_origin_length_01mm = 0;
static void MotorTapeFit_ResetState(bool clear_samples);
static int MotorTapeFit_StoreSample(int32_t motor_step,
                                    int32_t encoder_length_01mm,
                                    bool force);
static int MotorTapeFit_CaptureCurrentSample(bool force);
static int MotorTapeFit_SolveLinear3x3(double m[3][4], double out[3]);
static void MotorTapeFit_Range(double *n_min, double *n_max);
static void MotorTapeFit_LocalRange(double *q_min, double *q_max);

/* ===================== 私有函数声明 ===================== */

static void MotorTapeFit_ResetState(bool clear_samples);
static int MotorTapeFit_StoreSample(int32_t motor_step,
                                    int32_t encoder_length_01mm,
                                    bool force);
static int MotorTapeFit_CaptureCurrentSample(bool force);
static int MotorTapeFit_SolveLinear3x3(double m[3][4], double out[3]);
static void MotorTapeFit_Range(double *n_min, double *n_max);
static void MotorTapeFit_LocalRange(double *q_min, double *q_max);

/* ===================== 对外接口 ===================== */

/**
 * @brief 开始全局 TFIT 自动采样。
 *
 * 清空旧样本和旧结果，后续运动过程中会自动记录样本。
 */
void MotorCtrl_TapeFitStart(void)
{
    MotorTapeFit_ResetState(true);
    s_motor_tape_fit_enabled = true;
    printf("TFIT开始: 已启用自动采样, C0=%.3f mm, 尺带厚度=%.4f mm\r\n",
           MotorPosition_TapeC0Mm(),
           MotorPosition_TapeThicknessMm());
    (void)MotorTapeFit_CaptureCurrentSample(true);
}

/**
 * @brief 以当前位置作为原点开始局部 TFIT 采样。
 *
 * 局部拟合用于求当前位置附近的局部周长和厚度，不直接改变全局原点。
 */
void MotorCtrl_TapeFitStartLocalOrigin(void)
{
    MotorDrumState drum;
    int32_t encoder_length_01mm;

    if (!s_motor_driver.initialized) {
        printf("TFIT局部不可用: 电机未初始化\r\n");
        return;
    }

    MotorCtrl_UpdateDrumStateFromXActual(&stepper, &drum);
    encoder_length_01mm = MotorPosition_ReadEncoderLengthForFit();
    MotorTapeFit_ResetState(true);
    s_motor_tape_fit_origin_step = drum.motor_step;
    s_motor_tape_fit_origin_length_01mm = encoder_length_01mm;
    s_motor_tape_fit_local_origin_valid = true;
    s_motor_tape_fit_enabled = true;
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;

    printf("TFIT局部开始: 原点步数=%ld, 原点长度=%.1f mm\r\n",
           (long)s_motor_tape_fit_origin_step,
           (double)s_motor_tape_fit_origin_length_01mm * 0.1);
    (void)MotorTapeFit_CaptureCurrentSample(true);
}

/**
 * @brief 停止 TFIT 自动采样并保留已有样本。
 */
void MotorCtrl_TapeFitStop(void)
{
    s_motor_tape_fit_enabled = false;
    printf("TFIT停止: 已关闭自动采样, 采样数=%u\r\n",
           (unsigned)s_motor_tape_fit_count);
}

/**
 * @brief 手动加入当前电机/编码轮状态作为 TFIT 样本。
 */
void MotorCtrl_TapeFitAddCurrentSample(void)
{
    (void)MotorTapeFit_CaptureCurrentSample(true);
}

/**
 * @brief 打印 TFIT 当前采样状态和最近求解结果。
 */
void MotorCtrl_TapeFitPrintStatus(void)
{
    double n_min = 0.0;
    double n_max = 0.0;
    MotorTapeFit_Range(&n_min, &n_max);
    printf("TFIT状态: 使能=%u, 采样数=%u, 圈数范围=[%.3f, %.3f], 当前C0=%.3f mm, 当前尺带厚度=%.4f mm\r\n",
           s_motor_tape_fit_enabled ? 1U : 0U,
           (unsigned)s_motor_tape_fit_count,
           n_min,
           n_max,
           MotorPosition_TapeC0Mm(),
           MotorPosition_TapeThicknessMm());
    if (s_motor_tape_fit_result.valid) {
        printf("TFIT结果: 偏移=%.3f mm, C0=%.3f mm, 尺带厚度=%.4f mm, 均方根误差=%.3f mm, 最大误差=%.3f mm\r\n",
               s_motor_tape_fit_result.offset_mm,
               s_motor_tape_fit_result.first_loop_circ_mm,
               s_motor_tape_fit_result.tape_thickness_mm,
               s_motor_tape_fit_result.rmse_mm,
               s_motor_tape_fit_result.max_abs_err_mm);
    }
}

/**
 * @brief 使用全局 TFIT 样本求解 C0、厚度和零点偏移。
 *
 * @return 成功返回 NO_ERROR，否则返回样本不足或求解失败错误码。
 */
uint32_t MotorCtrl_TapeFitSolve(void)
{
    double normal[3][4] = { 0 };
    double coeff[3] = { 0 };
    double sum_sq = 0.0;
    double max_abs_err = 0.0;
    double n_min = 0.0;
    double n_max = 0.0;
    uint16_t i;
    (void)MotorTapeFit_CaptureCurrentSample(true);
    if (s_motor_tape_fit_count < 6U) {
        printf("TFIT求解失败: 至少需要6个采样点, 当前=%u\r\n",
               (unsigned)s_motor_tape_fit_count);
        return PARAM_ERROR;
    }
    MotorTapeFit_Range(&n_min, &n_max);
    if (fabs(n_max - n_min) < 1.0) {
        printf("TFIT求解失败: 电机步数跨度过小, 圈数范围=[%.3f, %.3f]\r\n",
               n_min,
               n_max);
        return PARAM_ERROR;
    }
    /* 拟合形式：
     *   y = a0 + a1*n + a2*n^2
     * 参数换算：
     *   offset=b=a0, C0=a1, t=-a2/pi
     */
    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double n = (double)s_motor_tape_fit_samples[i].motor_step / (double)MotorPosition_TapeTicksPerRev();
        const double x1 = n;
        const double x2 = n * n;
        const double y = (double)s_motor_tape_fit_samples[i].encoder_length_01mm * 0.1;
        normal[0][0] += 1.0;
        normal[0][1] += x1;
        normal[0][2] += x2;
        normal[0][3] += y;
        normal[1][0] += x1;
        normal[1][1] += x1 * x1;
        normal[1][2] += x1 * x2;
        normal[1][3] += x1 * y;
        normal[2][0] += x2;
        normal[2][1] += x1 * x2;
        normal[2][2] += x2 * x2;
        normal[2][3] += x2 * y;
    }
    if (!MotorTapeFit_SolveLinear3x3(normal, coeff)) {
        printf("TFIT求解失败: 矩阵奇异\r\n");
        return PARAM_ERROR;
    }
    s_motor_tape_fit_result.offset_mm = coeff[0];
    s_motor_tape_fit_result.first_loop_circ_mm = coeff[1];
    s_motor_tape_fit_result.tape_thickness_mm = -coeff[2] / M_PI;
    if ((s_motor_tape_fit_result.first_loop_circ_mm <= 0.0) ||
        (s_motor_tape_fit_result.tape_thickness_mm <= 0.0)) {
        printf("TFIT求解失败: 结果无效 偏移=%.3f, C0=%.3f, 尺带厚度=%.4f\r\n",
               s_motor_tape_fit_result.offset_mm,
               s_motor_tape_fit_result.first_loop_circ_mm,
               s_motor_tape_fit_result.tape_thickness_mm);
        s_motor_tape_fit_result.valid = false;
        return PARAM_ERROR;
    }
    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double n = (double)s_motor_tape_fit_samples[i].motor_step / (double)MotorPosition_TapeTicksPerRev();
        const double y = (double)s_motor_tape_fit_samples[i].encoder_length_01mm * 0.1;
        const double pred = coeff[0] + coeff[1] * n - M_PI * s_motor_tape_fit_result.tape_thickness_mm * n * n;
        const double err = pred - y;
        const double abs_err = fabs(err);
        sum_sq += err * err;
        if (abs_err > max_abs_err) {
            max_abs_err = abs_err;
        }
    }
    s_motor_tape_fit_result.rmse_mm = sqrt(sum_sq / (double)s_motor_tape_fit_count);
    s_motor_tape_fit_result.max_abs_err_mm = max_abs_err;
    s_motor_tape_fit_result.valid = true;
    s_motor_tape_fit_result_is_local = false;
    printf("TFIT求解完成: 偏移=%.3f mm, C0=%.3f mm, 尺带厚度=%.4f mm, 均方根误差=%.3f mm, 最大误差=%.3f mm, 采样数=%u\r\n",
           s_motor_tape_fit_result.offset_mm,
           s_motor_tape_fit_result.first_loop_circ_mm,
           s_motor_tape_fit_result.tape_thickness_mm,
           s_motor_tape_fit_result.rmse_mm,
           s_motor_tape_fit_result.max_abs_err_mm,
           (unsigned)s_motor_tape_fit_count);
    return NO_ERROR;
}

/**
 * @brief 使用局部原点样本求解当前位置附近的周长和厚度。
 *
 * @return 成功返回 NO_ERROR，否则返回样本不足或求解失败错误码。
 */
uint32_t MotorCtrl_TapeFitSolveLocalOrigin(void)
{
    double m00 = 0.0;
    double m01 = 0.0;
    double m11 = 0.0;
    double b0 = 0.0;
    double b1 = 0.0;
    double det;
    double a1;
    double a2;
    double sum_sq = 0.0;
    double max_abs_err = 0.0;
    double q_min = 0.0;
    double q_max = 0.0;
    uint16_t i;

    if (!s_motor_tape_fit_local_origin_valid) {
        printf("TFIT局部求解失败: 未设置局部原点\r\n");
        return PARAM_ERROR;
    }

    (void)MotorTapeFit_CaptureCurrentSample(true);
    if (s_motor_tape_fit_count < 6U) {
        printf("TFIT局部求解失败: 至少需要6个采样点, 当前=%u\r\n",
               (unsigned)s_motor_tape_fit_count);
        return PARAM_ERROR;
    }

    MotorTapeFit_LocalRange(&q_min, &q_max);
    if (fabs(q_max - q_min) < 1.0) {
        printf("TFIT局部求解失败: 相对圈数跨度过小, q范围=[%.3f, %.3f]\r\n",
               q_min,
               q_max);
        return PARAM_ERROR;
    }

    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double q = ((double)s_motor_tape_fit_samples[i].motor_step -
                          (double)s_motor_tape_fit_origin_step) /
                         (double)MotorPosition_TapeTicksPerRev();
        const double x1 = q;
        const double x2 = q * q;
        const double y = ((double)s_motor_tape_fit_samples[i].encoder_length_01mm -
                          (double)s_motor_tape_fit_origin_length_01mm) * 0.1;
        m00 += x1 * x1;
        m01 += x1 * x2;
        m11 += x2 * x2;
        b0 += x1 * y;
        b1 += x2 * y;
    }

    det = (m00 * m11) - (m01 * m01);
    if (fabs(det) < 1e-12) {
        printf("TFIT局部求解失败: 矩阵奇异\r\n");
        return PARAM_ERROR;
    }

    a1 = ((b0 * m11) - (b1 * m01)) / det;
    a2 = ((m00 * b1) - (m01 * b0)) / det;

    s_motor_tape_fit_result.offset_mm = 0.0;
    s_motor_tape_fit_result.first_loop_circ_mm = a1;
    s_motor_tape_fit_result.tape_thickness_mm = -a2 / M_PI;
    if ((s_motor_tape_fit_result.first_loop_circ_mm <= 0.0) ||
        (s_motor_tape_fit_result.tape_thickness_mm <= 0.0)) {
        printf("TFIT局部求解失败: 结果无效 局部周长=%.3f, 尺带厚度=%.4f\r\n",
               s_motor_tape_fit_result.first_loop_circ_mm,
               s_motor_tape_fit_result.tape_thickness_mm);
        s_motor_tape_fit_result.valid = false;
        return PARAM_ERROR;
    }

    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double q = ((double)s_motor_tape_fit_samples[i].motor_step -
                          (double)s_motor_tape_fit_origin_step) /
                         (double)MotorPosition_TapeTicksPerRev();
        const double y = ((double)s_motor_tape_fit_samples[i].encoder_length_01mm -
                          (double)s_motor_tape_fit_origin_length_01mm) * 0.1;
        const double pred = a1 * q + a2 * q * q;
        const double err = pred - y;
        const double abs_err = fabs(err);
        sum_sq += err * err;
        if (abs_err > max_abs_err) {
            max_abs_err = abs_err;
        }
    }

    s_motor_tape_fit_result.rmse_mm = sqrt(sum_sq / (double)s_motor_tape_fit_count);
    s_motor_tape_fit_result.max_abs_err_mm = max_abs_err;
    s_motor_tape_fit_result.valid = true;
    s_motor_tape_fit_result_is_local = true;

    printf("TFIT局部求解完成: 局部周长=%.3f mm, 尺带厚度=%.4f mm, 均方根误差=%.3f mm, 最大误差=%.3f mm, 采样数=%u\r\n",
           s_motor_tape_fit_result.first_loop_circ_mm,
           s_motor_tape_fit_result.tape_thickness_mm,
           s_motor_tape_fit_result.rmse_mm,
           s_motor_tape_fit_result.max_abs_err_mm,
           (unsigned)s_motor_tape_fit_count);
    return NO_ERROR;
}

/**
 * @brief 将最近一次 TFIT 求解结果写入设备参数。
 *
 * @param apply_c0 为 true 时应用首圈周长或局部周长。
 * @param apply_t 为 true 时应用尺带厚度。
 * @return 成功返回 NO_ERROR，否则返回参数或状态错误码。
 */
uint32_t MotorCtrl_TapeFitApply(bool apply_c0, bool apply_t)
{
    uint32_t old_c0 = g_deviceParams.first_loop_circumference_mm;
    uint32_t old_t = g_deviceParams.tape_thickness_mm;
    bool changed = false;
    if (!s_motor_tape_fit_result.valid) {
        uint32_t ret;
        if (s_motor_tape_fit_local_origin_valid) {
            if (apply_c0) {
                printf("TFIT应用失败: 局部原点采样不能更新全局C0\r\n");
                return PARAM_ERROR;
            }
            ret = MotorCtrl_TapeFitSolveLocalOrigin();
        } else {
            ret = MotorCtrl_TapeFitSolve();
        }
        if (ret != NO_ERROR) {
            return ret;
        }
    }
    if (apply_c0 && s_motor_tape_fit_result_is_local) {
        printf("TFIT应用失败: 局部结果不能更新全局C0\r\n");
        return PARAM_ERROR;
    }
    if (apply_c0) {
        const int32_t new_c0 = (int32_t)llround(s_motor_tape_fit_result.first_loop_circ_mm * 10.0);
        if (new_c0 <= 0) {
            return PARAM_ERROR;
        }
        g_deviceParams.first_loop_circumference_mm = (uint32_t)new_c0;
        changed = true;
    }
    if (apply_t) {
        const int32_t new_t = (int32_t)llround(s_motor_tape_fit_result.tape_thickness_mm * 1000.0);
        if (new_t <= 0) {
            return PARAM_ERROR;
        }
        g_deviceParams.tape_thickness_mm = (uint32_t)new_t;
        changed = true;
    }
    if (!changed) {
        return PARAM_ERROR;
    }
    save_device_params();
    printf("TFIT已应用: C0 %lu -> %lu (0.1mm), t %lu -> %lu (0.001mm)\r\n",
           (unsigned long)old_c0,
           (unsigned long)g_deviceParams.first_loop_circumference_mm,
           (unsigned long)old_t,
           (unsigned long)g_deviceParams.tape_thickness_mm);
    return NO_ERROR;
}

/* ===================== 内部跨文件接口 ===================== */

/**
 * @brief 运动过程中按电机步数间隔自动采样 TFIT。
 *
 * 只有 TFIT 采样使能时才会记录，采样间隔约为 1/4 卷筒圈。
 */
void MotorTapeFit_AutoSample(void)
{
    if (!s_motor_tape_fit_enabled) {
        return;
    }
    (void)MotorTapeFit_CaptureCurrentSample(false);
}

/* ===================== 私有函数实现 ===================== */

/**
 * @brief 重置 TFIT 采样和求解状态。
 *
 * @param clear_samples 为 true 时同时清空已采集样本。
 */
static void MotorTapeFit_ResetState(bool clear_samples)
{
    s_motor_tape_fit_enabled = false;
    s_motor_tape_fit_last_step = INT32_MIN;
    s_motor_tape_fit_result.valid = false;
    s_motor_tape_fit_result.offset_mm = 0.0;
    s_motor_tape_fit_result.first_loop_circ_mm = 0.0;
    s_motor_tape_fit_result.tape_thickness_mm = 0.0;
    s_motor_tape_fit_result.rmse_mm = 0.0;
    s_motor_tape_fit_result.max_abs_err_mm = 0.0;
    s_motor_tape_fit_result_is_local = false;
    if (clear_samples) {
        s_motor_tape_fit_count = 0;
        s_motor_tape_fit_local_origin_valid = false;
        s_motor_tape_fit_origin_step = 0;
        s_motor_tape_fit_origin_length_01mm = 0;
    }
}

/**
 * @brief 保存一条 TFIT 样本。
 *
 * 函数会处理重复点过滤和样本容量限制。
 * @param motor_step 电机 XACTUAL。
 * @param encoder_length_01mm 编码轮长度，单位 0.1mm。
 * @param force 为 true 时允许强制记录当前点。
 * @return 成功返回 1，跳过或失败返回 0。
 */
static int MotorTapeFit_StoreSample(int32_t motor_step,
                                    int32_t encoder_length_01mm,
                                    bool force)
{
    int64_t delta_ticks;
    if (s_motor_tape_fit_count > 0U) {
        delta_ticks = (int64_t)motor_step - (int64_t)s_motor_tape_fit_last_step;
        if ((!force) && (llabs(delta_ticks) < (int64_t)MOTOR_TAPE_FIT_AUTO_DELTA_TICKS)) {
            return 0;
        }
    }
    if (s_motor_tape_fit_count >= MOTOR_TAPE_FIT_MAX_SAMPLES) {
        if (s_motor_tape_fit_enabled) {
            s_motor_tape_fit_enabled = false;
            printf("TFIT采样缓冲已满，自动停止采样\r\n");
        }
        return -1;
    }
    s_motor_tape_fit_samples[s_motor_tape_fit_count].motor_step = motor_step;
    s_motor_tape_fit_samples[s_motor_tape_fit_count].encoder_length_01mm = encoder_length_01mm;
    s_motor_tape_fit_count++;
    s_motor_tape_fit_last_step = motor_step;
    s_motor_tape_fit_result.valid = false;
    s_motor_tape_fit_result_is_local = false;
    return 1;
}

/**
 * @brief 从当前电机和编码轮状态采集一条 TFIT 样本。
 *
 * @param force 为 true 时强制记录。
 * @return 成功返回 1，失败或跳过返回 0。
 */
static int MotorTapeFit_CaptureCurrentSample(bool force)
{
    MotorDrumState drum;
    int32_t encoder_length_01mm;
    int ret;
    if (!s_motor_driver.initialized) {
        printf("TFIT不可用: 电机未初始化\r\n");
        return -1;
    }
    MotorCtrl_UpdateDrumStateFromXActual(&stepper, &drum);
    encoder_length_01mm = MotorPosition_ReadEncoderLengthForFit();
    g_measurement.debug_data.motor_step = drum.motor_step;
    g_measurement.debug_data.motor_distance = drum.motor_distance_01mm;
    ret = MotorTapeFit_StoreSample(drum.motor_step,
                                   encoder_length_01mm,
                                   force);
    if (ret > 0) {
        printf("TFIT采样[%u] 步数=%ld, 编码轮=%.1f mm\r\n",
               (unsigned)s_motor_tape_fit_count,
               (long)drum.motor_step,
               encoder_length_01mm / 10.0);
    }
    return ret;
}

/**
 * @brief 求解 TFIT 使用的 3x3 线性方程组。
 *
 * @param m 增广矩阵，函数会原地消元。
 * @param out 输出求解结果。
 * @return 成功返回 1，矩阵奇异返回 0。
 */
static int MotorTapeFit_SolveLinear3x3(double m[3][4], double out[3])
{
    int row;
    int col;
    for (col = 0; col < 3; col++) {
        int pivot = col;
        double max_abs = fabs(m[col][col]);
        for (row = col + 1; row < 3; row++) {
            const double cur_abs = fabs(m[row][col]);
            if (cur_abs > max_abs) {
                max_abs = cur_abs;
                pivot = row;
            }
        }
        if (max_abs < 1e-12) {
            return 0;
        }
        if (pivot != col) {
            int k;
            for (k = col; k < 4; k++) {
                const double tmp = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = tmp;
            }
        }
        for (row = col + 1; row < 3; row++) {
            int k;
            const double factor = m[row][col] / m[col][col];
            for (k = col; k < 4; k++) {
                m[row][k] -= factor * m[col][k];
            }
        }
    }
    for (row = 2; row >= 0; row--) {
        double sum = m[row][3];
        for (col = row + 1; col < 3; col++) {
            sum -= m[row][col] * out[col];
        }
        if (fabs(m[row][row]) < 1e-12) {
            return 0;
        }
        out[row] = sum / m[row][row];
    }
    return 1;
}

/**
 * @brief 统计全局 TFIT 样本覆盖的圈数范围。
 *
 * @param n_min 输出最小圈数。
 * @param n_max 输出最大圈数。
 */
static void MotorTapeFit_Range(double *n_min, double *n_max)
{
    uint16_t i;
    if (n_min) {
        *n_min = 0.0;
    }
    if (n_max) {
        *n_max = 0.0;
    }
    if (s_motor_tape_fit_count == 0U) {
        return;
    }
    if (n_min) {
        *n_min = (double)s_motor_tape_fit_samples[0].motor_step / (double)MotorPosition_TapeTicksPerRev();
    }
    if (n_max) {
        *n_max = (double)s_motor_tape_fit_samples[0].motor_step / (double)MotorPosition_TapeTicksPerRev();
    }
    for (i = 1; i < s_motor_tape_fit_count; i++) {
        const double n = (double)s_motor_tape_fit_samples[i].motor_step / (double)MotorPosition_TapeTicksPerRev();
        if ((n_min != NULL) && (n < *n_min)) {
            *n_min = n;
        }
        if ((n_max != NULL) && (n > *n_max)) {
            *n_max = n;
        }
    }
}

/**
 * @brief 统计局部 TFIT 样本覆盖的相对圈数范围。
 *
 * @param q_min 输出最小相对圈数。
 * @param q_max 输出最大相对圈数。
 */
static void MotorTapeFit_LocalRange(double *q_min, double *q_max)
{
    uint16_t i;

    if (q_min) {
        *q_min = 0.0;
    }
    if (q_max) {
        *q_max = 0.0;
    }
    if ((!s_motor_tape_fit_local_origin_valid) ||
        (s_motor_tape_fit_count == 0U)) {
        return;
    }

    for (i = 0; i < s_motor_tape_fit_count; i++) {
        const double q = ((double)s_motor_tape_fit_samples[i].motor_step -
                          (double)s_motor_tape_fit_origin_step) /
                         (double)MotorPosition_TapeTicksPerRev();
        if (i == 0U) {
            if (q_min) {
                *q_min = q;
            }
            if (q_max) {
                *q_max = q;
            }
        } else {
            if ((q_min != NULL) && (q < *q_min)) {
                *q_min = q;
            }
            if ((q_max != NULL) && (q > *q_max)) {
                *q_max = q;
            }
        }
    }
}

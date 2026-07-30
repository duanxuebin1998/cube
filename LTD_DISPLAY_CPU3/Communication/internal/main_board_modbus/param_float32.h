#ifndef PARAM_FLOAT32_H
/* PARAM_FLOAT32_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define PARAM_FLOAT32_H

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/**
 * @brief 把 IEEE754 Float32 原始位转换为按小数位缩放的显示整数。
 *
 * @details 调用场景：CPU3 保持寄存器解析和参数快照同步共用。
 * @note 关键约束：正负对称四舍五入；NaN、无穷值和 int32 越界值在转换前拒绝。
 *
 * @param raw 待解释的 IEEE 754 Float32 原始 32 位位模式。
 * @param decimal_places 小数位。
 * @param result 缩放整数输出指针；仅当 raw 为有限 Float32 且缩放结果可由 int32_t 表示时写入。
 * @return true 表示原始位是有限 Float32，乘以指定十进制倍率后仍在 int32_t 范围内，并已按四舍五入写入输出；false 表示输出指针为空、小数位不受支持、原始值为 NaN/无穷或缩放后溢出。
 */
static inline bool ParamFloat32_TryRawToScaledInt(uint32_t raw,
                                                  uint8_t decimal_places,
                                                  int32_t *result)
{
    float value;
    double scale = 1.0;
    double scaled;
    double rounded;
    uint8_t i;

    if (result == NULL) {
        return false;
    }
    memcpy(&value, &raw, sizeof(value));
    if (!isfinite(value)) {
        return false;
    }

    for (i = 0U; i < decimal_places; i++) {
        scale *= 10.0;
        if (!isfinite(scale)) {
            return false;
        }
    }
    scaled = ((double)value) * scale;
    if (!isfinite(scaled)) {
        return false;
    }

    rounded = (scaled >= 0.0) ? floor(scaled + 0.5) : ceil(scaled - 0.5);
    if ((rounded > (double)INT32_MAX) || (rounded < (double)INT32_MIN)) {
        return false;
    }

    *result = (int32_t)rounded;
    return true;
}

#endif /* PARAM_FLOAT32_H */

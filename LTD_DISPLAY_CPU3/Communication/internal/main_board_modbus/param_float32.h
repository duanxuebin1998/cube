#ifndef PARAM_FLOAT32_H
#define PARAM_FLOAT32_H

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * 函数用途：把 IEEE754 Float32 原始位转换为按小数位缩放的显示整数。
 * 调用场景：CPU3 保持寄存器解析和参数快照同步共用。
 * 关键约束：正负对称四舍五入；NaN、无穷值和 int32 越界值在转换前拒绝。
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

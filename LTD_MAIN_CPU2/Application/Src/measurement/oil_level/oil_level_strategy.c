/*
 * 文件职责：将配置的液位测量方式映射为相互隔离的算法族。
 * 本选择器不产生副作用，硬件动作由选中的算法模块执行。
 */
#include "oil_level_strategy.h"

#include <stddef.h>

/**
 * @brief 将一个配置的测量方式转换为算法族选择结果。
 * @param raw_method 参数中保存的液位测量方式编号。
 * @param selection 输出选择结果；传入 NULL 时不产生副作用。
 * @note 参数校验和硬件动作仍由选中的算法族负责。
 */
void OilLevelStrategy_Select(uint32_t raw_method,
                             OilLevelStrategySelection *selection)
{
    if (selection == NULL) {
        return;
    }

    selection->raw_method = raw_method;
    selection->family = OIL_LEVEL_ALGORITHM_LEGACY_STEP;
    selection->direct_search = 0U;
    selection->continuous_follow = 0U;
    selection->fixed_frequency_target = 0U;

    switch (raw_method) {
    case OIL_LEVEL_METHOD_DENSITY:
        selection->family = OIL_LEVEL_ALGORITHM_DENSITY_CLOSED_LOOP;
        selection->direct_search = 1U;
        selection->continuous_follow = 1U;
        break;
    case OIL_LEVEL_METHOD_ULTRASONIC_RESERVED:
        selection->family = OIL_LEVEL_ALGORITHM_UNSUPPORTED;
        selection->direct_search = 1U;
        break;
    case OIL_LEVEL_METHOD_CONTINUOUS_RELATIVE_FREQ:
        selection->family = OIL_LEVEL_ALGORITHM_FREQUENCY_CLOSED_LOOP;
        selection->continuous_follow = 1U;
        break;
    case OIL_LEVEL_METHOD_CONTINUOUS_FIXED_FREQ:
        selection->family = OIL_LEVEL_ALGORITHM_FREQUENCY_CLOSED_LOOP;
        selection->direct_search = 1U;
        selection->continuous_follow = 1U;
        selection->fixed_frequency_target = 1U;
        break;
    case OIL_LEVEL_METHOD_RELATIVE_FREQ:
    case OIL_LEVEL_METHOD_FIXED_FREQ:
    default:
        break;
    }
}

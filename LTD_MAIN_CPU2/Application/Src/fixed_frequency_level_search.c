#include "fixed_frequency_level_search.h"

#include "abortable_delay.h"
#include "main.h"
#include "motor_ctrl.h"
#include "sensor.h"
#include "system_parameter.h"
#include "weight.h"

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

/* 定频找液位允许的罐顶安全余量 10.00 mm，单位为 0.01 mm；目标位置和当前位置都不得越过罐高减去该余量。 */
#define LF_TOP_MARGIN_01MM   1000U
/* 未配置死区时采用的默认频率死区 15 Hz。 */
#define LF_DEFAULT_BAND_HZ   15U
/* 定频搜索的最小电机速度 0.10 m/min，线值单位为 0.01 m/min。 */
#define LF_MIN_SPEED_X100    10U
/* 频率误差换算速度增量的比例因子：每增加 10 Hz 误差，速度线值增加 1，即 0.01 m/min。 */
#define LF_HZ_PER_SPEED_X100 10U
/* 定频搜索循环的传感器采样周期 200 ms。 */
#define LF_SAMPLE_MS         200U
/* 频率进入目标死区后单次稳定等待时间 1000 ms；等待期间仍允许命令切换中止当前流程。 */
#define LF_STABLE_MS         1000U
/* 确认到达目标频率所需的连续稳定采样次数 3；任一采样离开死区即清零重新累计。 */
#define LF_STABLE_COUNT      3U
/* 定频找液位流程总超时 600000 ms，即 10 min。 */
#define LF_TIMEOUT_MS        600000U
/* 尚未记录有效电机方向时使用的哨兵值 -1；与正常上、下方向枚举均不相同，用于判断是否需要方向切换。 */
#define LF_DIR_NONE          (-1)

/* 定频找液位运行配置快照；集中保存目标频率、死区、限速以及位置和扭力安全边界。 */
typedef struct {
    /* 定频找液位的目标、死区、限速及位置/扭力保护边界。 */
    uint32_t target_hz; /* 定频找液位的目标传感器频率，单位为 Hz。 */
    uint32_t deadband_hz; /* 目标频率两侧不再移动的死区宽度，单位为 Hz。 */
    uint32_t max_speed_x100; /* 定频找液位允许的最大电机速度，单位为 0.01 m/min。 */
    int32_t position_min; /* 找液位允许到达的最小位置边界。 */
    int32_t position_max; /* 找液位允许到达的最大位置边界。 */
    int32_t weight_min; /* 找液位允许的最小扭力/重量安全边界。 */
    int32_t weight_max; /* 找液位允许的最大扭力/重量安全边界。 */
} FixedLevelConfig;

/**
 * @brief 快照本次找液位使用的频率、位置、扭力和速度限制。
 *
 * @param target_override 目标。
 * @param deadband_override 调试命令显式给出的频率死区；未给出时使用系统参数默认值。
 * @param config 定频找液位配置输出对象；函数写入目标频率、死区、最大速度、位置上下限和扭力保护上下限的本轮快照。
 * @return PARAM_RANGE_ERROR 表示参数超出允许范围；NO_ERROR 表示操作成功。
 * @note 只读设备参数；零覆盖值表示使用设备当前配置。
 */
static uint32_t FixedLevel_BuildConfig(uint32_t target_override,
                                       uint32_t deadband_override,
                                       FixedLevelConfig *config)
{
    uint32_t raw_deadband = g_deviceParams.oilLevelThreshold;
    int64_t maximum_weight;

    config->target_hz = (target_override != 0U) ?
                        target_override : g_deviceParams.oilLevelFrequency;
    config->deadband_hz = (deadband_override != 0U) ?
                          deadband_override :
                          ((raw_deadband != 0U) ?
                           ((raw_deadband + (DENSITY_PARAM_MIGRATE_FACTOR / 2U)) /
                            DENSITY_PARAM_MIGRATE_FACTOR) :
                           LF_DEFAULT_BAND_HZ);

    if ((config->target_hz == 0U) || (g_deviceParams.full_weight == 0U)) {
        return PARAM_CONFIG_MISSING;
    }
    if ((config->target_hz > FIXED_FREQUENCY_LEVEL_MAX_HZ) ||
        (config->deadband_hz == 0U) ||
        (config->deadband_hz > FIXED_FREQUENCY_LEVEL_MAX_HZ) ||
        (g_deviceParams.tankHeight <= LF_TOP_MARGIN_01MM) ||
        (g_deviceParams.tankHeight > (uint32_t)INT32_MAX) ||
        (g_deviceParams.blindZone > (uint32_t)INT32_MAX) ||
        (g_deviceParams.blindZone >=
         (g_deviceParams.tankHeight - LF_TOP_MARGIN_01MM)) ||
        (g_deviceParams.full_weight > (uint32_t)INT32_MAX) ||
        (g_deviceParams.bottom_weight_threshold > (uint32_t)INT32_MAX) ||
        (g_deviceParams.zero_weight_threshold_ratio > 100U)) {
        return PARAM_RANGE_ERROR;
    }

    maximum_weight =
            ((int64_t)g_deviceParams.full_weight *
             (100LL + (int64_t)g_deviceParams.zero_weight_threshold_ratio)) /
            100LL;
    if ((maximum_weight > INT32_MAX) ||
        ((int64_t)g_deviceParams.bottom_weight_threshold >= maximum_weight)) {
        return PARAM_RANGE_ERROR;
    }

    config->position_min = (int32_t)g_deviceParams.blindZone;
    config->position_max =
            (int32_t)(g_deviceParams.tankHeight - LF_TOP_MARGIN_01MM);
    config->weight_min = (int32_t)g_deviceParams.bottom_weight_threshold;
    config->weight_max = (int32_t)maximum_weight;
    config->max_speed_x100 = MotorCtrl_GetDefaultSpeedX100();
    if (config->max_speed_x100 < LF_MIN_SPEED_X100) {
        config->max_speed_x100 = LF_MIN_SPEED_X100;
    }
    return NO_ERROR;
}

/**
 * @brief 统一慢停并返回原始失败码。
 *
 * @param error_code 待记录、转换或判断的错误码。该值是定频找液位退出原因，停止函数在保留原错误码的同时完成电机收尾。
 * @return 原 error_code 非零时始终保留并返回该根因；原结果为 NO_ERROR 但慢停失败时返回慢停错误，否则返回 NO_ERROR。
 * @note 用户停止属于状态切换，不作为故障覆盖。
 */
static uint32_t FixedLevel_Stop(uint32_t error_code)
{
    uint32_t stop_ret;

    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;
    stop_ret = MotorCtrl_SlowStop();
    printf("LF RESULT status=%s code=0x%08lX stop=0x%08lX\r\n",
           (error_code == STATE_SWITCH) ? "ABORT" : "ERROR",
           (unsigned long)error_code,
           (unsigned long)stop_ret);
    return ((error_code == NO_ERROR) && (stop_ret != NO_ERROR)) ?
           stop_ret : error_code;
}

/**
 * @brief 检查位置、绝对扭力、相对碰撞、通信和丢步保护。
 *
 * @param config 只读定频找液位配置快照；包含目标频率、死区、最大速度以及位置和扭力保护范围，用于运行前后持续执行边界检查。
 * @param moving 运动状态。
 * @return NO_ERROR 表示位置、扭力、碰撞、通信和丢步门禁均通过；越过位置边界返回 MEASUREMENT_OILLEVEL_HIGH 或 MEASUREMENT_OILLEVEL_LOW，碰撞返回 WEIGHT_COLLISION_DETECTED，其他值为扭力通信、位置读取或丢步检查的具体错误码。
 * @note 调用方收到非零返回值后必须统一慢停。
 */
static uint32_t FixedLevel_CheckGuards(const FixedLevelConfig *config,
                                       uint8_t moving)
{
    int32_t position;
    int32_t weight;
    uint32_t ret = MotorCtrl_PollRuntimePosition();

    if (ret != NO_ERROR) {
        return ret;
    }

    position = g_measurement.debug_data.sensor_position;
    if ((position >= config->position_max) ||
        (position <= config->position_min)) {
        printf("LF LIMIT position=%ld min=%ld max=%ld\r\n",
               (long)position,
               (long)config->position_min,
               (long)config->position_max);
        return (position >= config->position_max) ?
               MEASUREMENT_OILLEVEL_HIGH : MEASUREMENT_OILLEVEL_LOW;
    }

    ret = Weight_CheckCommunicationTimeout();
    if (ret != NO_ERROR) {
        return ret;
    }

    weight = (int32_t)weight_parament.current_weight;
    if ((weight >= config->weight_max) || (weight <= config->weight_min)) {
        printf("LF LIMIT weight=%ld min=%ld max=%ld\r\n",
               (long)weight,
               (long)config->weight_min,
               (long)config->weight_max);
        return WEIGHT_COLLISION_DETECTED;
    }

    if (moving == 0U) {
        return NO_ERROR;
    }
    ret = CheckWeightCollision();
    return (ret != NO_ERROR) ?
           ret : MotorCtrl_CheckLostStepAutoTiming(position);
}

/**
 * @brief 校验并打印固定频率找液位的目标、死区、位置、扭力、速度和稳定采样配置。
 */
void FixedFrequencyLevelSearch_PrintCurrentConfig(void)
{
    FixedLevelConfig config;
    uint32_t ret = FixedLevel_BuildConfig(0U, 0U, &config);

    if (ret != NO_ERROR) {
        printf("LF CONFIG status=INVALID code=0x%08lX\r\n",
               (unsigned long)ret);
        return;
    }
    printf("LF CONFIG target=%luHz deadband=%luHz pos_min=%ld pos_max=%ld "
           "weight_min=%ld weight_max=%ld speed_min=0.10 speed_max=%.2f "
           "sample=%lums stable=%lu timeout=%lums\r\n",
           (unsigned long)config.target_hz,
           (unsigned long)config.deadband_hz,
           (long)config.position_min,
           (long)config.position_max,
           (long)config.weight_min,
           (long)config.weight_max,
           (double)config.max_speed_x100 / 100.0,
           (unsigned long)LF_SAMPLE_MS,
           (unsigned long)LF_STABLE_COUNT,
           (unsigned long)LF_TIMEOUT_MS);
}

/**
 * @brief 按固定目标频率和死区闭环搜索液位。
 *
 * 函数接受已经通过串口语法分类的 LF 命令；LF=<目标频率>[,<死区>] 提供本次覆盖值，未提供的字段由 FixedLevel_BuildConfig
 * 使用设备参数或默认值，并同时建立位置、扭力和最大速度保护边界。
 * 进入液位频率模式后清除旧液位稳定标志并初始化丢步检测；循环内在读取频率前后均检查位置、扭力通信、碰撞和丢步保护，避免一次传感器事务期间越过机械边界。
 * 当前频率落入目标死区时先慢停电机，并要求连续 LF_STABLE_COUNT 次稳定采样；确认后把当前位置同时写入油位结果和密度分布油位，置位探头到液面及液体稳定标志。
 * 频率偏差超出死区时按偏差符号选择上行或下行，按偏差量计算并限制 0.01 m/min 速度；方向反转前先慢停，再启动新的速度方向。
 * 命令切换、总超时、传感器读取、位置、扭力、碰撞、丢步或电机控制失败均进入统一 stop 出口，清除稳定标志并尝试慢停，禁止异常返回后继续保持速度模式。
 *
 * @param command 以 NUL 结尾且已通过严格语法校验的 LF 命令；LF=<Hz>[,<死区Hz>] 可覆盖本次目标和死区，LF 或 LF? 不携带覆盖值时使用已配置或默认值。
 * @return NO_ERROR 表示频率连续稳定达到门限、当前位置已发布且电机已停止；STATE_SWITCH
 *         表示被新命令中断，其他值区分配置或范围、总超时、传感器、位置、扭力、丢步和电机慢停错误。
 * @note 除稳定成功直接返回外，所有退出都通过 FixedLevel_Stop；若原错误为 NO_ERROR 而慢停失败，则返回慢停错误，否则保留最先发生的业务错误。
 */
uint32_t FixedFrequencyLevelSearch_Run(const uint8_t *command)
{
    FixedLevelConfig config;
    uint32_t target_override = 0U;
    uint32_t deadband_override = 0U;
    uint32_t current_frequency = 0U;
    uint32_t stable_count = 0U;
    uint32_t start_tick;
    uint32_t ret;
    int last_direction = LF_DIR_NONE;
    uint8_t moving = 0U;
    char *end;

    if (command == NULL) {
        ret = SYSTEM_CALL_CONDITION_ERROR;
        goto stop;
    }
    if (command[2] == '=') {
        target_override = (uint32_t)strtoul(
                (const char *)&command[3], &end, 10);
        if (*end == ',') {
            deadband_override = (uint32_t)strtoul(end + 1, NULL, 10);
        }
    }

    ret = FixedLevel_BuildConfig(target_override,
                                 deadband_override,
                                 &config);
    if (ret != NO_ERROR) {
        goto stop;
    }

    g_measurement.oil_measurement.follow_frequency = config.target_hz;
    g_measurement.oil_measurement.probe_at_liquid_level = 0U;
    g_measurement.oil_measurement.liquid_stable = 0U;
    ret = EnableLevelMode();
    if (ret != NO_ERROR) {
        goto stop;
    }

    MotorCtrl_LostStepInit();
    start_tick = HAL_GetTick();
    printf("LF START target=%lu deadband=%lu pos_min=%ld pos_max=%ld "
           "weight_min=%ld weight_max=%ld speed_max=%.2f\r\n",
           (unsigned long)config.target_hz,
           (unsigned long)config.deadband_hz,
           (long)config.position_min,
           (long)config.position_max,
           (long)config.weight_min,
           (long)config.weight_max,
           (double)config.max_speed_x100 / 100.0);

    while (1) {
        int32_t error_hz;
        uint32_t absolute_error;
        uint32_t speed;
        int direction;

        if (HasEffectiveCommandSwitchRequest()) {
            ret = STATE_SWITCH;
            goto stop;
        }
        if ((HAL_GetTick() - start_tick) > LF_TIMEOUT_MS) {
            ret = MEASUREMENT_FREQUENCY_LEVEL_TIMEOUT;
            goto stop;
        }

        ret = FixedLevel_CheckGuards(
                &config,
                moving);
        if (ret != NO_ERROR) {
            goto stop;
        }
        ret = DSM_Get_LevelMode_Frequence(&current_frequency);
        if (ret != NO_ERROR) {
            goto stop;
        }
        g_measurement.oil_measurement.current_frequency = current_frequency;

        ret = FixedLevel_CheckGuards(
                &config,
                moving);
        if (ret != NO_ERROR) {
            goto stop;
        }
        if ((moving != 0U) &&
            (g_measurement.debug_data.motor_state !=
             (uint32_t)((last_direction == MOTOR_DIRECTION_UP) ? 1U : 2U))) {
            moving = 0U;
            last_direction = LF_DIR_NONE;
        }

        error_hz = (int32_t)current_frequency - (int32_t)config.target_hz;
        absolute_error = (error_hz >= 0) ?
                         (uint32_t)error_hz : (uint32_t)(-error_hz);

        if (absolute_error <= config.deadband_hz) {
            if (moving != 0U) {
                ret = MotorCtrl_SlowStop();
                if (ret != NO_ERROR) {
                    goto stop;
                }
                moving = 0U;
                last_direction = LF_DIR_NONE;
            }
            stable_count++;
            printf("LF LOOP freq=%lu error=%ld pos=%ld weight=%d "
                   "dir=STOP speed=0.00 stable=%lu/%u\r\n",
                   (unsigned long)current_frequency,
                   (long)error_hz,
                   (long)g_measurement.debug_data.sensor_position,
                   weight_parament.current_weight,
                   (unsigned long)stable_count,
                   LF_STABLE_COUNT);

            if (stable_count >= LF_STABLE_COUNT) {
                ret = FixedLevel_CheckGuards(&config, 0U);
                if (ret != NO_ERROR) {
                    goto stop;
                }
                g_measurement.oil_measurement.oil_level =
                        (uint32_t)g_measurement.debug_data.sensor_position;
                g_measurement.density_distribution.Density_oil_level =
                        g_measurement.oil_measurement.oil_level;
                g_measurement.oil_measurement.probe_at_liquid_level = 1U;
                g_measurement.oil_measurement.liquid_stable = 1U;
                printf("LF RESULT status=FOUND level=%lu frequency=%lu "
                       "target=%lu\r\n",
                       (unsigned long)g_measurement.oil_measurement.oil_level,
                       (unsigned long)current_frequency,
                       (unsigned long)config.target_hz);
                return NO_ERROR;
            }

            ret = AbortableDelay_CommandSwitch(LF_STABLE_MS, 100U);
            if (ret != NO_ERROR) {
                goto stop;
            }
            continue;
        }

        stable_count = 0U;
        g_measurement.oil_measurement.probe_at_liquid_level = 0U;
        g_measurement.oil_measurement.liquid_stable = 0U;
        direction = (error_hz > 0) ?
                    MOTOR_DIRECTION_DOWN : MOTOR_DIRECTION_UP;
        speed = LF_MIN_SPEED_X100 +
                ((absolute_error - config.deadband_hz +
                  (LF_HZ_PER_SPEED_X100 / 2U)) /
                 LF_HZ_PER_SPEED_X100);
        if (speed > config.max_speed_x100) {
            speed = config.max_speed_x100;
        }
        printf("LF LOOP freq=%lu error=%ld pos=%ld weight=%d dir=%s "
               "speed=%.2f stable=0/%u\r\n",
               (unsigned long)current_frequency,
               (long)error_hz,
               (long)g_measurement.debug_data.sensor_position,
               weight_parament.current_weight,
               MotorCtrl_DirectionText(direction),
               (double)speed / 100.0,
               LF_STABLE_COUNT);

        if ((moving != 0U) && (last_direction != direction)) {
            ret = MotorCtrl_SlowStop();
            if (ret != NO_ERROR) {
                goto stop;
            }
        }
        ret = MotorCtrl_StartVelocity(direction, speed);
        if (ret != NO_ERROR) {
            goto stop;
        }
        moving = 1U;
        last_direction = direction;
        ret = AbortableDelay_CommandSwitch(LF_SAMPLE_MS, 50U);
        if (ret != NO_ERROR) {
            goto stop;
        }
    }

stop:
    return FixedLevel_Stop(ret);
}

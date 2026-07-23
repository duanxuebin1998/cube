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

#define LF_TOP_MARGIN_01MM   1000U
#define LF_DEFAULT_BAND_HZ   15U
#define LF_MIN_SPEED_X100    10U
#define LF_HZ_PER_SPEED_X100 10U
#define LF_SAMPLE_MS         200U
#define LF_STABLE_MS         1000U
#define LF_STABLE_COUNT      3U
#define LF_TIMEOUT_MS        600000U
#define LF_DIR_NONE          (-1)

typedef struct {
    uint32_t target_hz;
    uint32_t deadband_hz;
    uint32_t max_speed_x100;
    int32_t position_min;
    int32_t position_max;
    int32_t weight_min;
    int32_t weight_max;
} FixedLevelConfig;

/*
 * 函数用途：快照本次找液位使用的频率、位置、扭力和速度限制。
 * 关键约束：只读设备参数；零覆盖值表示使用设备当前配置。
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

/*
 * 函数用途：统一慢停并返回原始失败码。
 * 关键约束：用户停止属于状态切换，不作为故障覆盖。
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

/*
 * 函数用途：检查位置、绝对扭力、相对碰撞、通信和丢步保护。
 * 关键约束：调用方收到非零返回值后必须统一慢停。
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

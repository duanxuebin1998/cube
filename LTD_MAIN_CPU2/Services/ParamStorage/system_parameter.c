/*
 * system_parameter.c
 *
 *  Created on: Feb 27, 2025
 *      Author: Duan Xuebin
 *
 * 说明:
 *  - DeviceParameters 中 command 仅用于command，不参与掉电参数校验
 *  - 从 sensorType 开始到 crc 之前的区域为持久化参数区
 */

#include "system_parameter.h"
#include "error_log.h"
#include "app_version.h"
#include "mb85rs2m.h"
#include "my_crc.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h> /* offsetof */

volatile MeasurementResult g_measurement = {0};   /* 测量结果 */
volatile DeviceParameters  g_deviceParams = {0};  /* 设备参数 */
static volatile uint8_t g_device_params_save_pending = 0; /* Deferred save request flag */
static volatile uint32_t g_device_params_save_request_tick = 0; /* Last deferred save request tick */

#ifndef DEVICE_PARAMS_SAVE_DEBOUNCE_MS
#define DEVICE_PARAMS_SAVE_DEBOUNCE_MS 100u
#endif

/* param_version和 magic 常量 */
#define DEVICE_PARAM_VERSION   (2u)
#define DEVICE_PARAM_MAGIC     (0x4C54444Du)  /* 'LTDM' */

/*
 * 持久化区域说明:
 *  - command 不参与掉电参数校验
 *  - 从 sensorType 起到 crc 前为持久化参数区
 */
#define DEVICE_PARAM_PERSIST_OFFSET   (offsetof(DeviceParameters, sensorType))
#define DEVICE_PARAM_PERSIST_LEN      (offsetof(DeviceParameters, crc) - DEVICE_PARAM_PERSIST_OFFSET)
#define DEVICE_PARAM_PERSIST_START(p) ((uint8_t *)(p) + DEVICE_PARAM_PERSIST_OFFSET)

/*========================= 参数存储逻辑 =========================*/

/* 根据持久化区计算crc（不含command和crc字段） */
static uint32_t device_param_crc(const DeviceParameters *params)
{
    const uint8_t *crc_base = DEVICE_PARAM_PERSIST_START(params);
    const uint32_t crc_size = (uint32_t)DEVICE_PARAM_PERSIST_LEN;
    return CRC32_HAL((const uint8_t *)crc_base, crc_size);
}
#define MOTOR_LOCAL_CIRC_MIN_001MM  (50000u)    /* 50.000mm，和motor_count_first_loop_circumference_mm模型下限保持一致 */
#define MOTOR_LOCAL_CIRC_MAX_001MM  (5000000u)  /* 5000.000mm，防止旧 reserved 脏值被当成有效周长 */
#define MOTOR_LOCAL_CIRC_FALLBACK_001MM (600000u) /* 默认 600.000mm，用于first_loop_circumference_mm本身也异常时兜底 */

static uint32_t device_params_default_motor_local_circ_001mm(void)
{
    uint32_t value = g_deviceParams.first_loop_circumference_mm * 100U;

    if ((value < MOTOR_LOCAL_CIRC_MIN_001MM) ||
        (value > MOTOR_LOCAL_CIRC_MAX_001MM)) {
        value = MOTOR_LOCAL_CIRC_FALLBACK_001MM;
    }
    return value;
}

/* 软件版本跟随当前固件，避免被 FRAM 里的旧参数覆盖。 */
static int apply_firmware_version_runtime(void)
{
    if (g_deviceParams.softwareVersion == CPU2_APP_VERSION_U32) {
        return 0;
    }

    g_deviceParams.softwareVersion = CPU2_APP_VERSION_U32;
    return 1;
}
/* 修正新增参数的非法值。
 * reserved6/reserved7 复用为position_count_mode和motor_count_first_loop_circumference_mm后，旧 FRAM 里可能残留任意非 0 值。
 * 这里既修正 RAM，又把是否修正返回给 load_device_params()，由加载流程决定是否写回 FRAM。 */
static int normalize_device_params_runtime(void)
{
    int changed = 0;

    if ((g_deviceParams.position_count_mode != POSITION_COUNT_MODE_ENCODER) &&
        (g_deviceParams.position_count_mode != POSITION_COUNT_MODE_MOTOR)) {
        g_deviceParams.position_count_mode = POSITION_COUNT_MODE_ENCODER;
        changed = 1;
    }

    if ((g_deviceParams.position_source_auto_switch != POSITION_SOURCE_AUTO_SWITCH_DISABLE) &&
        (g_deviceParams.position_source_auto_switch != POSITION_SOURCE_AUTO_SWITCH_ENABLE)) {
        g_deviceParams.position_source_auto_switch = POSITION_SOURCE_AUTO_SWITCH_ENABLE;
        changed = 1;
    }

    if ((g_deviceParams.bottom_encoder_correction_enable != BOTTOM_ENCODER_CORRECTION_DISABLE) &&
        (g_deviceParams.bottom_encoder_correction_enable != BOTTOM_ENCODER_CORRECTION_ENABLE)) {
        g_deviceParams.bottom_encoder_correction_enable = BOTTOM_ENCODER_CORRECTION_DISABLE;
        changed = 1;
    }

    if ((g_deviceParams.motor_current < MOTOR_CURRENT_MIN) ||
        (g_deviceParams.motor_current > MOTOR_CURRENT_MAX)) {
        g_deviceParams.motor_current = MOTOR_CURRENT_DEFAULT;
        changed = 1;
    }

    if ((g_deviceParams.motor_count_first_loop_circumference_mm < MOTOR_LOCAL_CIRC_MIN_001MM) ||
        (g_deviceParams.motor_count_first_loop_circumference_mm > MOTOR_LOCAL_CIRC_MAX_001MM)) {
        g_deviceParams.motor_count_first_loop_circumference_mm =
            device_params_default_motor_local_circ_001mm();
        changed = 1;
    }

    /* 旧版本该位置为 reserved17，可能读到 0；这里补默认值，避免监测阈值过小。 */
    if (g_deviceParams.water_lag_cap_threshold == 0U) {
        g_deviceParams.water_lag_cap_threshold = 80000U;
        changed = 1;
    }

    /* 瓦锡兰探底频率范围为0~100，旧 reserved22 脏值按默认每次探底处理。 */
    if (g_deviceParams.wartsila_bottom_detect_interval > 100U) {
        g_deviceParams.wartsila_bottom_detect_interval = 1U;
        changed = 1;
    }

    return changed;
}

/* 内部通用读取接口：
 * 1. 统一对 FRAM 槽位做 magic/version/CRC 校验；
 * 2. verbose=0 时用于静默判重，避免因为每次保存前判重而打大量日志；
 * 3. verbose=1 时用于正常加载诊断，保留详细失败原因。 */
static int load_device_params_from_slot_impl(uint32_t base_addr,
                                             DeviceParameters *out,
                                             const char *slot_name,
                                             int verbose)
{
    DeviceParameters temp;
    char detail[128];

    ReadMultiData((uint8_t *)&temp, (int)base_addr, sizeof(DeviceParameters));

    if (temp.magic != DEVICE_PARAM_MAGIC)
    {
        if (verbose) {
            printf("参数[%s]魔术字不匹配: 0x%08lX\r\n", slot_name, (unsigned long)temp.magic);
        }
        if (verbose) {
            snprintf(detail, sizeof(detail),
                     "分区：%s,魔术字：0x%08lX,期望：0x%08lX",
                     slot_name,
                     (unsigned long)temp.magic,
                     (unsigned long)DEVICE_PARAM_MAGIC);
            // 错误	阶段：错误报警	模块：参数	操作：参数校验	原因：魔术字不匹配	处理：继续尝试	详情：detail
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                                ERROR_LOG_OP_PARAM_VALIDATE,
                                ERROR_LOG_REASON_PARAM_MAGIC,
                                ERROR_LOG_ACTION_CONTINUE,
                                detail);
        }
        return 0;
    }

    if (temp.struct_size != sizeof(DeviceParameters))
    {
        if (verbose) {
            printf("参数[%s]结构体大小不匹配: FRAM=%lu, 当前：%lu\r\n",
                   slot_name,
                   (unsigned long)temp.struct_size,
                   (unsigned long)sizeof(DeviceParameters));
        }
        if (verbose) {
            snprintf(detail, sizeof(detail),
                     "分区：%s,FRAM大小：%lu,当前大小：%lu",
                     slot_name,
                     (unsigned long)temp.struct_size,
                     (unsigned long)sizeof(DeviceParameters));
            // 错误	阶段：错误报警	模块：参数	操作：参数校验	原因：结构体大小不匹配	处理：继续尝试	详情：detail
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                                ERROR_LOG_OP_PARAM_VALIDATE,
                                ERROR_LOG_REASON_PARAM_SIZE,
                                ERROR_LOG_ACTION_CONTINUE,
                                detail);
        }
        return 0;
    }

    if (temp.param_version != DEVICE_PARAM_VERSION)
    {
        if (verbose) {
            printf("参数[%s]版本不匹配: FRAM=%lu, 当前：%lu\r\n",
                   slot_name,
                   (unsigned long)temp.param_version,
                   (unsigned long)DEVICE_PARAM_VERSION);
        }
        if (verbose) {
            snprintf(detail, sizeof(detail),
                     "分区：%s,FRAM版本：%lu,当前版本：%lu",
                     slot_name,
                     (unsigned long)temp.param_version,
                     (unsigned long)DEVICE_PARAM_VERSION);
            // 错误	阶段：错误报警	模块：参数	操作：参数校验	原因：版本不匹配	处理：继续尝试	详情：detail
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                                ERROR_LOG_OP_PARAM_VALIDATE,
                                ERROR_LOG_REASON_PARAM_VERSION,
                                ERROR_LOG_ACTION_CONTINUE,
                                detail);
        }
        return 0;
    }

    {
        const uint32_t calc_crc = device_param_crc(&temp);
        if (calc_crc != temp.crc)
        {
            if (verbose) {
                printf("参数[%s] CRC不匹配: 计算=0x%08lX, FRAM=0x%08lX\r\n",
                       slot_name,
                       (unsigned long)calc_crc,
                       (unsigned long)temp.crc);
            }
            if (verbose) {
                snprintf(detail, sizeof(detail),
                         "分区：%s,计算CRC：0x%08lX,FRAMCRC：0x%08lX",
                         slot_name,
                         (unsigned long)calc_crc,
                         (unsigned long)temp.crc);
                // 错误	阶段：错误报警	模块：参数	操作：参数校验	原因：CRC不匹配	处理：继续尝试	详情：detail
                ErrorLog_WarnDetail(ERROR_LOG_MODULE_PARAM,
                                    ERROR_LOG_OP_PARAM_VALIDATE,
                                    ERROR_LOG_REASON_PARAM_CRC,
                                    ERROR_LOG_ACTION_CONTINUE,
                                    detail);
            }
            return 0;
        }
    }

    *out = temp;
    return 1;
}

/* 把当前内存里的 g_deviceParams 整理成“准备写入 FRAM 的完整镜像”。
 * 这一步会顺手补齐 version/size/magic/crc，
 * 保证后面的“判断是否需要保存”与“真正写入的内容”完全一致。 */
static void build_saved_device_params(DeviceParameters *out)
{
    apply_firmware_version_runtime();
    *out = g_deviceParams;
    out->param_version = DEVICE_PARAM_VERSION;
    out->struct_size   = (uint32_t)sizeof(DeviceParameters);
    out->magic         = DEVICE_PARAM_MAGIC;
    out->crc           = device_param_crc(out);
}

/* 只比较持久化参数区（sensorType ~ crc 之前）。
 * command 属于运行态指令，不应该因为它的变化就触发整个参数区重写。 */
static int device_param_persist_equal(const DeviceParameters *lhs, const DeviceParameters *rhs)
{
    return memcmp(DEVICE_PARAM_PERSIST_START(lhs),
                  DEVICE_PARAM_PERSIST_START(rhs),
                  DEVICE_PARAM_PERSIST_LEN) == 0;
}

/* 统一的参数保存入口：
 * mark_updated=1：说明这是一次“真正的参数变更”，需要递增 parameter_update_flag，
 *                 让 CPU3 在后续轮询中检测到并补读保持寄存器。
 * force_write=1：忽略判重，强制回写 FRAM，主要用于 A/B 分区自修复这种场景。 */
static void save_device_params_internal(int mark_updated, int force_write)
{
    DeviceParameters params;
    DeviceParameters slot_a;
    DeviceParameters slot_b;
    int slot_a_valid;
    int slot_b_valid;

    if (sizeof(DeviceParameters) > FRAM_PARAM_SLOT_SIZE)
    {
        printf("参数大小超出分区容量: 大小=%lu, 分区：%lu\r\n",
               (unsigned long)sizeof(DeviceParameters),
               (unsigned long)FRAM_PARAM_SLOT_SIZE);
        g_measurement.device_status.error_code = PARAM_ADDRESS_OVERFLOW;
        return;
    }

    build_saved_device_params(&params);
    /* 保存镜像里的元信息也回填到全局参数，避免后续参数打印显示旧 CRC。 */
    g_deviceParams.param_version = params.param_version;
    g_deviceParams.struct_size = params.struct_size;
    g_deviceParams.magic = params.magic;
    g_deviceParams.crc = params.crc;

    slot_a_valid = load_device_params_from_slot_impl(FRAM_PARAM_A_ADDRESS, &slot_a, "A", 0);
    slot_b_valid = load_device_params_from_slot_impl(FRAM_PARAM_B_ADDRESS, &slot_b, "B", 0);

    /* 仅当 A/B 两份 FRAM 都有效，且持久化区和本次待保存内容完全一致时，
     * 才真正跳过写入。
     * 只要 A/B 有一份异常或内容不一致，就仍然要重写，避免错过自修复机会。 */
    if (!force_write
        && slot_a_valid
        && slot_b_valid
        && device_param_persist_equal(&slot_a, &params)
        && device_param_persist_equal(&slot_b, &params))
    {
        printf("设备参数未变化，跳过保存\r\n");
        return;
    }

    /* 只有“参数真的发生改变”时才递增更新标志。
     * 如果因为判重被跳过或仅仅是分区自修复，都不应该触发 CPU3 再次补读。 */
    if (mark_updated) {
        g_measurement.device_status.parameter_update_flag++;
    }

    printf("保存设备参数: 版本：%lu, 大小=%lu, CRC=0x%08lX\r\n",
           (unsigned long)params.param_version,
           (unsigned long)params.struct_size,
           (unsigned long)params.crc);

    WriteMultiData((uint8_t *)&params, (int)FRAM_PARAM_A_ADDRESS, sizeof(DeviceParameters));
    WriteMultiData((uint8_t *)&params, (int)FRAM_PARAM_B_ADDRESS, sizeof(DeviceParameters));

    printf("设备参数已保存到 FRAM A/B\r\n");

    print_device_params();
}

/* 从指定分区读取并校验设备参数：返回1成功，0失败 */
static int load_device_params_from_slot(uint32_t base_addr, DeviceParameters *out, const char *slot_name)
{
    return load_device_params_from_slot_impl(base_addr, out, slot_name, 1);
}

/* 对外的默认保存入口：
 * 表示“用户或业务逻辑确实修改了系统参数”，
 * 因此需要同时进行判重 + 必要时写 FRAM + 递增参数更新标志。 */
void save_device_params(void)
{
    save_device_params_internal(1, 0);
}

/* Called from the Modbus write path after a 0x10 parameter update.
 * This function only sets a pending flag. The actual FRAM write is moved
 * to the main loop so the UART interrupt path stays short. */
void request_device_params_save(void)
{
    g_device_params_save_pending = 1;
    g_device_params_save_request_tick = HAL_GetTick();
}

/* Handle deferred tasks in the main loop.
 * For now this only flushes pending parameter saves, but more deferred
 * work can be merged here later if needed. */
void process_device_params_deferred_tasks(void)
{
    uint32_t now;

    if (!g_device_params_save_pending)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - g_device_params_save_request_tick) < DEVICE_PARAMS_SAVE_DEBOUNCE_MS)
    {
        return;
    }

    g_device_params_save_pending = 0;
    save_device_params_internal(1, 0);
}
int load_device_params(void)
{
    DeviceParameters temp;
    int loaded_from_a = 0;
    int params_normalized = 0;

    if (sizeof(DeviceParameters) > FRAM_PARAM_SLOT_SIZE)
    {
        printf("设备参数超出单分区容量: 大小=%lu, 分区：%lu\r\n", (unsigned long)sizeof(DeviceParameters), (unsigned long)FRAM_PARAM_SLOT_SIZE);
        g_measurement.device_status.error_code = PARAM_ADDRESS_OVERFLOW;
        return 0;
    }

    if (load_device_params_from_slot(FRAM_PARAM_A_ADDRESS, &temp, "A"))
    {
        loaded_from_a = 1;
    }
    else if (load_device_params_from_slot(FRAM_PARAM_B_ADDRESS, &temp, "B"))
    {
        // 错误	阶段：重试成功	模块：参数	操作：FRAM参数分区回退	原因：A分区异常，使用B分区	尝试：1U/1U
        ErrorLog_Recover(ERROR_LOG_MODULE_PARAM,
                         ERROR_LOG_OP_FRAM_FALLBACK,
                         ERROR_LOG_REASON_FRAM_FALLBACK,
                         1U,
                         1U);
    }
    else
    {
        g_measurement.device_status.error_code = PARAM_EEPROM_FAIL;
        return 0;
    }

    memcpy((void * volatile)&g_deviceParams, &temp, sizeof(DeviceParameters));

    g_deviceParams.command = g_deviceParams.powerOnDefaultCommand;
    params_normalized = normalize_device_params_runtime();
    params_normalized |= apply_firmware_version_runtime();

    /* 上电时如果 A 分区损坏、但 B 分区有效，
     * 这里只做“存储介质自修复”，不视为用户修改参数，
     * 所以不递增 parameter_update_flag，避免 CPU3 在上电后被平白触发一次“参数变更”。 */
    /* repair A from B without bumping update flag */
    if ((!loaded_from_a) || params_normalized) {
        save_device_params_internal(0, 1);
    }

    printf("设备参数加载成功 (来源=%s)\r\n", loaded_from_a ? "A" : "B");
    return 1;
}

/*========================= 参数初始化 =========================*/

/* 初始化设备参数模块：连续 3 次读取失败才恢复出厂 */
void init_device_params(void)
{
    const int MAX_RETRY = 3;
    int ok = 0;

    for (int attempt = 1; attempt <= MAX_RETRY; attempt++)
    {
        if (load_device_params())
        {
            ok = 1;
            /* 上电成功读取参数后固定打印一次完整参数表，
             * 不再依赖保存、A/B 修复或恢复出厂等附带路径。 */
            print_device_params();
            break;
        }
        // 错误	阶段：错误重试	模块：参数	操作：FRAM参数分区回退	原因：FRAM参数分区异常	尝试：attempt/MAX_RETRY	错误码：PARAM_EEPROM_FAIL	错误名：ErrorLog_GetCodeName(PARAM_EEPROM_FAIL)
        ErrorLog_Retry(ERROR_LOG_MODULE_PARAM,
                       ERROR_LOG_OP_FRAM_FALLBACK,
                       ERROR_LOG_REASON_FRAM_ERROR,
                       (uint32_t)attempt,
                       (uint32_t)MAX_RETRY,
                       PARAM_EEPROM_FAIL);
        HAL_Delay(100);
    }

    if (!ok)
    {
        /* 连续失败 3 次 -> 恢复出厂参数并保存 */
        memset((void * volatile)&g_deviceParams, 0, sizeof(DeviceParameters));
        RestoreFactoryParamsConfig(); /* 内部会调用 save_device_params() */

        g_measurement.device_status.error_code = PARAM_EEPROM_FAIL;
        // 错误	阶段：错误报警	模块：参数	操作：FRAM参数分区回退	原因：FRAM参数分区异常	处理：使用默认参数
        ErrorLog_Warn(ERROR_LOG_MODULE_PARAM,
                      ERROR_LOG_OP_FRAM_FALLBACK,
                      ERROR_LOG_REASON_FRAM_ERROR,
                      ERROR_LOG_ACTION_USE_DEFAULT_PARAM);
    }
}
/*========================= 恢复出厂参数 =========================*/

/*
 * 恢复出厂参数配置
 * 注意: 这里设置的是默认值, 可根据实际项目需要调整
 */
void RestoreFactoryParamsConfig(void)
{
    /* 整体清零, 保证保留字段等为 0 */
    memset((void * volatile)&g_deviceParams, 0, sizeof(DeviceParameters));

    /* ---------------- 指令类字段 ---------------- */
    g_deviceParams.command               = CMD_NONE;
    g_deviceParams.powerOnDefaultCommand = CMD_NONE;

    /* ---------------- 基础参数 ---------------- */
    g_deviceParams.sensorType            = DSM_SENSOR;
    g_deviceParams.sensorID              = 1234567;
    g_deviceParams.sensorSoftwareVersion = 0x00010001;
    g_deviceParams.softwareVersion       = CPU2_APP_VERSION_U32;
    g_deviceParams.error_auto_back_zero  = 1;   /* 默认: 报错回零 */
    g_deviceParams.error_stop_measurement= 1;   /* 默认: 报错停止测量 */
    g_deviceParams.position_source_auto_switch = POSITION_SOURCE_AUTO_SWITCH_ENABLE; /* 默认: 允许流程自动切换位置源 */

    /* ---------------- 电机与编码器参数 ---------------- */
    g_deviceParams.encoder_wheel_circumference_mm = 95000;  /* 0.001mm */
    g_deviceParams.max_motor_speed                = 400;    /* 0.01m/min */
    g_deviceParams.first_loop_circumference_mm    = 6000; /* 0.1mm */
    g_deviceParams.tape_thickness_mm              = 200;    /* 0.001mm */
    g_deviceParams.motor_current                 = MOTOR_CURRENT_DEFAULT; /* TMC5130 IRUN */
    g_deviceParams.position_count_mode                        = POSITION_COUNT_MODE_ENCODER; /* 默认编码轮记步 */
    g_deviceParams.motor_count_first_loop_circumference_mm =
        g_deviceParams.first_loop_circumference_mm * 100U; /* 0.001mm */

    /* ---------------- 称重参数 ---------------- */
    g_deviceParams.empty_weight             = 0;
    g_deviceParams.empty_weight_upper_limit = 2000;
    g_deviceParams.empty_weight_lower_limit = 0;

    g_deviceParams.full_weight              = 5000;
    g_deviceParams.full_weight_upper_limit  = 30000;
    g_deviceParams.full_weight_lower_limit  = 2000;

    g_deviceParams.weight_upper_limit_ratio = 50;
    g_deviceParams.weight_lower_limit_ratio = 20;

    /* ---------------- 零点测量 ---------------- */
    g_deviceParams.zero_weight_threshold_ratio                 = 50;   /* 按算法需要调整 */
    g_deviceParams.weight_ignore_zone          = 1000; /* 0.1mm => 100mm */
    g_deviceParams.max_zero_deviation_distance = 1000;  /* 零点区域最大偏差值 0.1mm => 20mm */
    g_deviceParams.findZeroDownDistance        = 1000; /* 0.1mm => 100mm */

    /* ---------------- 液位测量 ---------------- */
    g_deviceParams.tankHeight                  = 200000; /* 0.1mm => 20000mm */
    g_deviceParams.liquid_sensor_distance_diff = 0;      /* 0.1mm */
    g_deviceParams.blindZone                   = 3000;   /* 0.1mm => 300mm */

    g_deviceParams.oilLevelThreshold                     = 15;     /* 项目自定义倍率/单位 */
    g_deviceParams.oilLevelHysteresisThreshold = 20;     /* 项目自定义倍率/单位 */
    g_deviceParams.liquidLevelMeasurementMethod= 0;		/* 0 空气+液体频率/2 1：根据设置跟随频率跟随 2 根据设置密度跟随 3.根据振动管跟随 */
    g_deviceParams.oilLevelFrequency                = 5500;      /* oilLevelFrequency */
    g_deviceParams.oilLevelDensity                = 0;      /* oilLevelDensity */

    /* ---------------- 水位测量参数 ---------------- */
    g_deviceParams.water_tank_height                = 200000; /* 0.1mm */
    g_deviceParams.water_level_mode                      = 0;      /* 0:慢速 */
    g_deviceParams.waterBlindZone                   = 100;    /* 0.1mm */
    g_deviceParams.water_cap_threshold                      = 50000;      /* 建议明确倍率后再设默认 */
    g_deviceParams.water_find_cap_threshold                      = 5000;      /* 水位寻找电容阈值，x1000，5.0pF */
    g_deviceParams.maxDownDistance                  = 3000;   /* 0.1mm => 300mm */
    g_deviceParams.zero_cap                         = 0;      /* 0.1pf */
    g_deviceParams.water_stable_threshold                      = 500;      /* 0.1mm */
    g_deviceParams.water_lag_cap_threshold  = 80000;  /* 水位滞后电容阈值，x1000，80.0pF */

    /* ---------------- 罐高/罐底测量 ---------------- */
    g_deviceParams.bottom_detect_mode      = 0;    /* 0=按项目定义 */
    g_deviceParams.bottom_angle_threshold  = 12;    /* 单位(度）/倍率*1 */
    g_deviceParams.bottom_weight_threshold = 2000;    /* 按现场经验再设默认 */

    g_deviceParams.refreshTankHeightFlag   = 0;  /* 不自动刷新 */
    g_deviceParams.maxTankHeightDeviation  = 100;  /* 0.1mm => 10mm */
    g_deviceParams.initialTankHeight       = 0;
    g_deviceParams.currentTankHeight       = 0;
    g_deviceParams.bottom_encoder_correction_enable = BOTTOM_ENCODER_CORRECTION_DISABLE; /* 默认: 罐底测量后不修正编码器 */

    /* ---------------- 密度/温度修正 ---------------- */
    g_deviceParams.densityCorrection       = 10000;
    g_deviceParams.temperatureCorrection   = 1000;

    /* ---------------- 分布/区间测量参数 ---------------- */
    g_deviceParams.requireBottomMeasurement                   = 0;
    g_deviceParams.requireWaterMeasurement                   = 0;
    g_deviceParams.requireSinglePointDensity               = 0;

    g_deviceParams.spreadMeasurementOrder                   = 0;
    g_deviceParams.spreadMeasurementMode                   = 0;
    g_deviceParams.spreadMeasurementCount                   = 5;
    g_deviceParams.spreadMeasurementDistance                   = 10000; /* 0.1mm */
    g_deviceParams.spreadTopLimit              = 300;  /* 0.1mm */
    g_deviceParams.spreadBottomLimit           = 300;  /* 0.1mm */
    g_deviceParams.spreadPointHoverTime               = 10;

    g_deviceParams.intervalMeasurementTopLimit    = 300; /* 0.1mm */
    g_deviceParams.intervalMeasurementBottomLimit = 300; /* 0.1mm */

    /* ---------------- Wartsila 密度区间 ---------------- */
    g_deviceParams.wartsila_upper_density_limit      = 38000;
    g_deviceParams.wartsila_lower_density_limit      = 500;
    g_deviceParams.wartsila_density_interval         = 1000;
    g_deviceParams.wartsila_max_height_above_surface = 200; /* 0.1mm 或按定义 */
    g_deviceParams.wartsila_bottom_detect_interval  = 1;   /* 瓦锡兰测量后探底频率：0不探底，N表示每N次测量后探底一次 */

    /* ---------------- 继电器报警输出 ---------------- */
    g_deviceParams.AlarmHighDO         = 0;
    g_deviceParams.AlarmLowDO          = 0;
    g_deviceParams.ThirdStateThreshold = 0;

    /* ---------------- 4~20mA 输出 ---------------- */
    g_deviceParams.CurrentRangeStart_mA = 400;   /* 4.00mA (×0.01) */
    g_deviceParams.CurrentRangeEnd_mA   = 2000;  /* 20.00mA (×0.01) */
    g_deviceParams.AlarmHighAO          = 2000;
    g_deviceParams.AlarmLowAO           = 400;
    g_deviceParams.InitialCurrent_mA    = 400;
    g_deviceParams.AOHighCurrent_mA     = 2000;
    g_deviceParams.AOLowCurrent_mA      = 400;
    g_deviceParams.FaultCurrent_mA      = 2200;  /* 22.00mA */
    g_deviceParams.DebugCurrent_mA      = 1200;  /* 12.00mA */

    /* ---------------- 指令参数 ---------------- */
    g_deviceParams.calibrateOilLevel                      = 0;
    g_deviceParams.calibrateWaterLevel                      = 0;
    g_deviceParams.calibrateTankHeight           = 0;
    g_deviceParams.singlePointMeasurementPosition = 0;
    g_deviceParams.singlePointMonitoringPosition  = 0;
    g_deviceParams.densityDistributionOilLevel                = 0;
    g_deviceParams.motorCommandDistance                    = 0;

    g_deviceParams.oilLevelHysteresisTime                    = 0;
    g_deviceParams.waterLevelCorrection                      = 0;
    g_deviceParams.lastOilCorrectionLevel                = 0;
    g_deviceParams.tankGasPhaseTemperature                        = 0;
    g_deviceParams.tapeExpansionCoefficient                      = 0;
    g_deviceParams.tapeCalibrationTemperature                    = 0;

    /* ---------------- 元信息与校验字段 ---------------- */
    g_deviceParams.param_version = DEVICE_PARAM_VERSION;
    g_deviceParams.struct_size   = (uint32_t)sizeof(DeviceParameters);
    g_deviceParams.magic         = DEVICE_PARAM_MAGIC;
    g_deviceParams.crc           = 0; /* save_device_params 内更新 */

    save_device_params();
}

/*========================= 参数打印 =========================*/

/* 打印所有设备参数, 便于调试 */
void print_device_params(void)
{
    DeviceParameters params;
    memcpy(&params, (void *)&g_deviceParams, sizeof(DeviceParameters));

    printf("\r\n========================================\r\n");
    printf("              设备参数\r\n");
    printf("========================================\r\n");

    /* 指令 */
    printf("\r\n-- 指令 --\r\n");
    printf("  %-32s : %u\r\n", "当前指令", (unsigned)params.command);
    printf("  %-32s : %u\r\n", "上电默认指令", (unsigned)params.powerOnDefaultCommand);

    /* 基础参数 */
    printf("\r\n-- 基础参数 --\r\n");
    printf("  %-32s : %lu\r\n", "传感器类型", (unsigned long)params.sensorType);
    printf("  %-32s : %lu\r\n", "传感器编号", (unsigned long)params.sensorID);
    printf("  %-32s : 0x%08lX\r\n", "传感器软件版本", (unsigned long)params.sensorSoftwareVersion);
    printf("  %-32s : 0x%08lX\r\n", "软件版本", (unsigned long)params.softwareVersion);
    printf("  %-32s : V%lu.%lu.%lu.%lu\r\n", "软件版本文本",
           (unsigned long)((params.softwareVersion >> 24) & 0xFFU),
           (unsigned long)((params.softwareVersion >> 16) & 0xFFU),
           (unsigned long)((params.softwareVersion >> 8) & 0xFFU),
           (unsigned long)(params.softwareVersion & 0xFFU));
    printf("  %-32s : %lu\r\n", "故障自动回零", (unsigned long)params.error_auto_back_zero);
    printf("  %-32s : %lu\r\n", "故障停止测量", (unsigned long)params.error_stop_measurement);
    printf("  %-32s : %lu\r\n", "位置源自动切换", (unsigned long)params.position_source_auto_switch);

    /* 电机与编码器 */
    printf("\r\n-- 电机与编码器 --\r\n");
    printf("  %-32s : %lu\r\n", "编码轮周长(0.001mm)", (unsigned long)params.encoder_wheel_circumference_mm);
    printf("  %-32s : %lu\r\n", "电机最大速度(0.01m/min)", (unsigned long)params.max_motor_speed);
    printf("  %-32s : %lu\r\n", "电机运行电流(IRUN 1-31)", (unsigned long)params.motor_current);
    printf("  %-32s : %lu\r\n", "首圈周长(0.1mm)", (unsigned long)params.first_loop_circumference_mm);
    printf("  %-32s : %lu\r\n", "尺带厚度(0.001mm)", (unsigned long)params.tape_thickness_mm);
    printf("  %-32s : %lu\r\n", "记步模式", (unsigned long)params.position_count_mode);
    printf("  %-32s : %lu\r\n", "电机局部周长(0.001mm)", (unsigned long)params.motor_count_first_loop_circumference_mm);

    /* 称重 */
    printf("\r\n-- 称重参数 --\r\n");
    printf("  %-32s : %lu\r\n", "空载重量", (unsigned long)params.empty_weight);
    printf("  %-32s : %lu\r\n", "空载重量上限", (unsigned long)params.empty_weight_upper_limit);
    printf("  %-32s : %lu\r\n", "空载重量下限", (unsigned long)params.empty_weight_lower_limit);
    printf("  %-32s : %lu\r\n", "满载重量", (unsigned long)params.full_weight);
    printf("  %-32s : %lu\r\n", "满载重量上限", (unsigned long)params.full_weight_upper_limit);
    printf("  %-32s : %lu\r\n", "满载重量下限", (unsigned long)params.full_weight_lower_limit);
    printf("  %-32s : %lu\r\n", "碰撞上限比率", (unsigned long)params.weight_upper_limit_ratio);
    printf("  %-32s : %lu\r\n", "碰撞下限比率", (unsigned long)params.weight_lower_limit_ratio);

    /* 零点 */
    printf("\r\n-- 零点参数 --\r\n");
    printf("  %-32s : %lu\r\n", "零点阈值比例", (unsigned long)params.zero_weight_threshold_ratio);
    printf("  %-32s : %lu\r\n", "称重忽略区(0.1mm)", (unsigned long)params.weight_ignore_zone);
    printf("  %-32s : %lu\r\n", "零点最大偏差(0.1mm)", (unsigned long)params.max_zero_deviation_distance);
    printf("  %-32s : %lu\r\n", "找零下行距离(0.1mm)", (unsigned long)params.findZeroDownDistance);

    /* 液位 */
    printf("\r\n-- 液位参数 --\r\n");
    printf("  %-32s : %lu\r\n", "液位罐高(0.1mm)", (unsigned long)params.tankHeight);
    printf("  %-32s : %lu\r\n", "液位探头距差(0.1mm)", (unsigned long)params.liquid_sensor_distance_diff);
    printf("  %-32s : %lu\r\n", "液位盲区(0.1mm)", (unsigned long)params.blindZone);
    printf("  %-32s : %lu\r\n", "找油阈值", (unsigned long)params.oilLevelThreshold);
    printf("  %-32s : %lu\r\n", "液位滞后阈值", (unsigned long)params.oilLevelHysteresisThreshold);
    printf("  %-32s : %lu\r\n", "液位测量方式", (unsigned long)params.liquidLevelMeasurementMethod);
    printf("  %-32s : %lu\r\n", "液位跟随频率", (unsigned long)params.oilLevelFrequency);
    printf("  %-32s : %lu\r\n", "液位跟随密度", (unsigned long)params.oilLevelDensity);
    printf("  %-32s : %lu\r\n", "液位滞后时间", (unsigned long)params.oilLevelHysteresisTime);

    /* 水位 */
    printf("\r\n-- 水位参数 --\r\n");
    printf("  %-32s : %lu\r\n", "水位罐高(0.1mm)", (unsigned long)params.water_tank_height);
    printf("  %-32s : %lu\r\n", "水位测量方式", (unsigned long)params.water_level_mode);
    printf("  %-32s : %lu\r\n", "水位盲区(0.1mm)", (unsigned long)params.waterBlindZone);
    printf("  %-32s : %lu\r\n", "水位电容阈值", (unsigned long)params.water_cap_threshold);
    printf("  %-32s : %lu\r\n", "水位寻找电容阈值", (unsigned long)params.water_find_cap_threshold);
    printf("  %-32s : %lu\r\n", "最大下行距离(0.1mm)", (unsigned long)params.maxDownDistance);
    printf("  %-32s : %lu\r\n", "零点电容", (unsigned long)params.zero_cap);
    printf("  %-32s : %lu\r\n", "水位零点电容", (unsigned long)params.zero_cap);
    printf("  %-32s : %lu\r\n", "水位稳定阈值", (unsigned long)params.water_stable_threshold);
    printf("  %-32s : %lu\r\n", "水位滞后电容阈值", (unsigned long)params.water_lag_cap_threshold);
    printf("  %-32s : %lu\r\n", "水位修正值", (unsigned long)params.waterLevelCorrection);

    /* 罐底/罐高 */
    printf("\r\n-- 罐底/罐高参数 --\r\n");
    printf("  %-32s : %lu\r\n", "罐底检测模式", (unsigned long)params.bottom_detect_mode);
    printf("  %-32s : %lu\r\n", "罐底角度阈值", (unsigned long)params.bottom_angle_threshold);
    printf("  %-32s : %lu\r\n", "罐底称重阈值", (unsigned long)params.bottom_weight_threshold);
    printf("  %-32s : %lu\r\n", "更新罐高标志", (unsigned long)params.refreshTankHeightFlag);
    printf("  %-32s : %lu\r\n", "实高最大偏差", (unsigned long)params.maxTankHeightDeviation);
    printf("  %-32s : %lu\r\n", "初始罐高", (unsigned long)params.initialTankHeight);
    printf("  %-32s : %lu\r\n", "当前罐高", (unsigned long)params.currentTankHeight);
    printf("  %-32s : %lu\r\n", "罐底后编码器修正", (unsigned long)params.bottom_encoder_correction_enable);

    /* 修正 */
    printf("\r\n-- 修正参数 --\r\n");
    printf("  %-32s : %lu\r\n", "密度修正值", (unsigned long)params.densityCorrection);
    printf("  %-32s : %lu\r\n", "温度修正值", (unsigned long)params.temperatureCorrection);

    /* 分布/区间 */
    printf("\r\n-- 分布/区间参数 --\r\n");
    printf("  %-32s : %lu\r\n", "是否测罐底", (unsigned long)params.requireBottomMeasurement);
    printf("  %-32s : %lu\r\n", "是否测水位", (unsigned long)params.requireWaterMeasurement);
    printf("  %-32s : %lu\r\n", "是否测单点密度", (unsigned long)params.requireSinglePointDensity);
    printf("  %-32s : %lu\r\n", "分布测顺序", (unsigned long)params.spreadMeasurementOrder);
    printf("  %-32s : %lu\r\n", "分布测模式", (unsigned long)params.spreadMeasurementMode);
    printf("  %-32s : %lu\r\n", "分布测点数", (unsigned long)params.spreadMeasurementCount);
    printf("  %-32s : %lu\r\n", "分布点间距", (unsigned long)params.spreadMeasurementDistance);
    printf("  %-32s : %lu\r\n", "顶点距液面(0.1mm)", (unsigned long)params.spreadTopLimit);
    printf("  %-32s : %lu\r\n", "底点距罐底(0.1mm)", (unsigned long)params.spreadBottomLimit);
    printf("  %-32s : %lu\r\n", "分布点悬停时间", (unsigned long)params.spreadPointHoverTime);
    printf("  %-32s : %lu\r\n", "区间测量上限(0.1mm)", (unsigned long)params.intervalMeasurementTopLimit);
    printf("  %-32s : %lu\r\n", "区间测量下限(0.1mm)", (unsigned long)params.intervalMeasurementBottomLimit);

    /* Wartsila */
    printf("\r\n-- 瓦锡兰参数 --\r\n");
    printf("  %-32s : %lu\r\n", "密度点上限", (unsigned long)params.wartsila_upper_density_limit);
    printf("  %-32s : %lu\r\n", "密度点下限", (unsigned long)params.wartsila_lower_density_limit);
    printf("  %-32s : %lu\r\n", "密度点间距", (unsigned long)params.wartsila_density_interval);
    printf("  %-32s : %lu\r\n", "最高点距液面", (unsigned long)params.wartsila_max_height_above_surface);
    printf("  %-32s : %lu\r\n", "瓦锡兰探底频率", (unsigned long)params.wartsila_bottom_detect_interval);

    /* DO */
    printf("\r\n-- 报警DO参数 --\r\n");
    printf("  %-32s : %lu\r\n", "高液位报警DO", (unsigned long)params.AlarmHighDO);
    printf("  %-32s : %lu\r\n", "低液位报警DO", (unsigned long)params.AlarmLowDO);
    printf("  %-32s : %lu\r\n", "第三状态阈值", (unsigned long)params.ThirdStateThreshold);

    /* AO */
    printf("\r\n-- 4-20mA/AO参数 --\r\n");
    printf("  %-32s : %lu\r\n", "输出范围起点mA", (unsigned long)params.CurrentRangeStart_mA);
    printf("  %-32s : %lu\r\n", "输出范围终点mA", (unsigned long)params.CurrentRangeEnd_mA);
    printf("  %-32s : %lu\r\n", "高限报警AO", (unsigned long)params.AlarmHighAO);
    printf("  %-32s : %lu\r\n", "低限报警AO", (unsigned long)params.AlarmLowAO);
    printf("  %-32s : %lu\r\n", "初始电流mA", (unsigned long)params.InitialCurrent_mA);
    printf("  %-32s : %lu\r\n", "高位电流mA", (unsigned long)params.AOHighCurrent_mA);
    printf("  %-32s : %lu\r\n", "低位电流mA", (unsigned long)params.AOLowCurrent_mA);
    printf("  %-32s : %lu\r\n", "故障电流mA", (unsigned long)params.FaultCurrent_mA);
    printf("  %-32s : %lu\r\n", "调试电流mA", (unsigned long)params.DebugCurrent_mA);

    /* 指令参数 */
    printf("\r\n-- 指令参数 --\r\n");
    printf("  %-32s : %lu\r\n", "标定液位值", (unsigned long)params.calibrateOilLevel);
    printf("  %-32s : %lu\r\n", "标定水位值", (unsigned long)params.calibrateWaterLevel);
    printf("  %-32s : %lu\r\n", "单点测量位置", (unsigned long)params.singlePointMeasurementPosition);
    printf("  %-32s : %lu\r\n", "单点监测位置", (unsigned long)params.singlePointMonitoringPosition);
    printf("  %-32s : %lu\r\n", "密度分布测量液位", (unsigned long)params.densityDistributionOilLevel);
    printf("  %-32s : %lu\r\n", "电机指令距离", (unsigned long)params.motorCommandDistance);
    printf("\r\n-- 尺带补偿参数 --\r\n");
    printf("  %-32s : %lu\r\n", "上次液位修正液位", (unsigned long)params.lastOilCorrectionLevel);
    printf("  %-32s : %lu\r\n", "气相温度", (unsigned long)params.tankGasPhaseTemperature);
    printf("  %-32s : %lu\r\n", "尺带伸缩率", (unsigned long)params.tapeExpansionCoefficient);
    printf("  %-32s : %lu\r\n", "尺带标定温度", (unsigned long)params.tapeCalibrationTemperature);

    /* 元信息/CRC */
    printf("\r\n-- 元信息/CRC --\r\n");
    printf("  %-32s : %lu\r\n", "参数版本号", (unsigned long)params.param_version);
    printf("  %-32s : %lu\r\n", "结构体大小", (unsigned long)params.struct_size);
    printf("  %-32s : 0x%08lX\r\n", "魔术字", (unsigned long)params.magic);
    printf("  %-32s : 0x%08lX\r\n", "参数CRC32", (unsigned long)params.crc);

    printf("========================================\r\n");
}

/*========================= 测量结果打印（可选） =========================*/
/* 注: 该部分与参数结构无强耦合，仅保留你现有打印习惯；如果不需要可移除 */

/**
 * @brief 打印单个密度测点信息
 * @param title 输出标题
 * @param d     密度测点数据指针
 */
void PrintDensity(const char *title, const DensityMeasurement *d)
{
    if (!d) return;

    printf("  [%s]\r\n", title);
    printf("    温度: %lu\r\n", (unsigned long)d->temperature);
    printf("    密度: %lu\r\n", (unsigned long)d->density);
    printf("    标准密度: %lu\r\n", (unsigned long)d->standard_density);
    printf("    VCF20: %lu\r\n", (unsigned long)d->vcf20);
    printf("    计重密度: %lu\r\n", (unsigned long)d->weight_density);
    printf("    温度位置: %lu\r\n", (unsigned long)d->temperature_position);
}

/**
 * @brief 打印完整测量结果
 * @param m 测量结果指针
 */
void PrintMeasurementResult(const MeasurementResult *m)
{
    if (!m) return;

    printf("\r\n=====================【设备实时测量结果】=====================\r\n");

    /* 1. 设备状态 */
    printf("【设备状态】\r\n");
    printf("  工作模式: %lu\r\n", (unsigned long)m->device_status.work_mode);
    printf("  设备状态: %d\r\n",  (int)m->device_status.device_state);
    printf("  错误代码: %lu\r\n", (unsigned long)m->device_status.error_code);
    printf("  当前指令: %d\r\n",  (int)m->device_status.current_command);
    printf("  零点状态: %lu\r\n", (unsigned long)m->device_status.zero_point_status);

    printf("--------------------------------------------------------------\r\n");

    /* 2. 调试数据 */
    printf("【调试数据】\r\n");
    printf("  编码值: %ld\r\n",        (long)m->debug_data.current_encoder_value);
    printf("  传感器位置: %ld mm\r\n", (long)m->debug_data.sensor_position);
    printf("  尺带长度: %ld mm\r\n",   (long)m->debug_data.cable_length);
    printf("  电机步进: %ld\r\n",      (long)m->debug_data.motor_step);
    printf("  电机距离: %ld (0.1mm)\r\n",(long)m->debug_data.motor_distance);

    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->debug_data.frequency);
    printf("  温度: %.2f ℃\r\n",       m->debug_data.temperature / 100.0f);
    printf("  空气中频率: %lu Hz\r\n", (unsigned long)m->debug_data.air_frequency);
    printf("  幅值: %lu\r\n",          (unsigned long)m->debug_data.current_amplitude);
    printf("  水位电压/电容值: %lu\r\n",(unsigned long)m->debug_data.water_level_voltage);

    printf("  当前称重值: %lu\r\n",    (unsigned long)m->debug_data.current_weight);
    printf("  称重参数: %lu\r\n",      (unsigned long)m->debug_data.weight_param);

    printf("  X角度: %ld\r\n", (long)m->debug_data.angle_x);
    printf("  Y角度: %ld\r\n", (long)m->debug_data.angle_y);

    printf("  电机速度(0.01m/min): %lu\r\n", (unsigned long)m->debug_data.motor_speed);
    printf("  电机状态: %lu (%s)\r\n",
           (unsigned long)m->debug_data.motor_state,
           (m->debug_data.motor_state == 0) ? "停止" :
           (m->debug_data.motor_state == 1) ? "上行" :
           (m->debug_data.motor_state == 2) ? "下行" : "未知");

    printf("--------------------------------------------------------------\r\n");

    /* 3. 液位测量 */
    printf("【液位测量】\r\n");
    printf("  跟随液位: %lu mm\r\n",   (unsigned long)m->oil_measurement.oil_level);
    printf("  空气频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.air_frequency);
    printf("  油中频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.oil_frequency);
    printf("  跟随频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.follow_frequency);
    printf("  当前频率: %lu Hz\r\n",   (unsigned long)m->oil_measurement.current_frequency);

    printf("--------------------------------------------------------------\r\n");

    /* 4. 水位测量 */
    printf("【水位测量】\r\n");
    printf("  水位值: %lu mm\r\n", (unsigned long)m->water_measurement.water_level);
    printf("  零点电容: %.3f\r\n", m->water_measurement.zero_capacitance);
    printf("  油区电容: %.3f\r\n", m->water_measurement.oil_capacitance);
    printf("  当前电容: %.3f\r\n", m->water_measurement.current_capacitance);

    printf("--------------------------------------------------------------\r\n");

    /* 5. 实高测量 */
    printf("【实高测量】\r\n");
    printf("  标定液位实高: %lu mm\r\n", (unsigned long)m->height_measurement.calibrated_liquid_level);
    printf("  当前实高: %lu mm\r\n",     (unsigned long)m->height_measurement.current_real_height);

    printf("--------------------------------------------------------------\r\n");

    /* 6. 单点密度测量 */
    printf("【单点密度测量】\r\n");
    PrintDensity("单点测量", &m->single_point_measurement);

    /* 7. 单点监测 */
    printf("【单点监测】\r\n");
    PrintDensity("单点监测", &m->single_point_monitoring);

    printf("--------------------------------------------------------------\r\n");

    /* 8. 密度分布（概要） */
    printf("【密度分布】\r\n");
    printf("  平均温度: %lu\r\n",     (unsigned long)m->density_distribution.average_temperature);
    printf("  平均密度: %lu\r\n",     (unsigned long)m->density_distribution.average_density);
    printf("  平均计重密度: %lu\r\n", (unsigned long)m->density_distribution.average_weight_density);
    printf("  测量点数: %lu\r\n",         (unsigned long)m->density_distribution.measurement_points);
    printf("  测量时液位(0.1mm): %lu\r\n",(unsigned long)m->density_distribution.Density_oil_level);
    printf("  测量时液位(实际): %.1f mm\r\n",
           (double)m->density_distribution.Density_oil_level / 10.0);

    printf("  --- 单点数据（仅打印前10个）---\r\n");
    for (uint32_t i = 0; i < 10 && i < m->density_distribution.measurement_points; i++)
    {
        const DensityMeasurement *d = &m->density_distribution.single_density_data[i];
        printf("    [%02lu] 温度=%lu 密度=%lu 标密=%lu VCF=%lu 重量密度=%lu 位置=%lu\r\n",
               (unsigned long)i,
               (unsigned long)d->temperature,
               (unsigned long)d->density,
               (unsigned long)d->standard_density,
               (unsigned long)d->vcf20,
               (unsigned long)d->weight_density,
               (unsigned long)d->temperature_position);
    }

    printf("========================【打印结束】========================\r\n");
}
/**
 * @brief 默认指令 -> 测量命令 映射
 * @param def_cmd  DefaultCommandType
 * @return         CommandType（测量命令）；无匹配返回 CMD_UNKNOWN
 */
CommandType DefaultCmd_To_MeasureCmd(DefaultCommandType def_cmd)
{
    switch (def_cmd) {

    case CMD_NONE_DEF:
        return CMD_NONE;

    case CMD_BACK_ZERO_DEF:
        return CMD_BACK_ZERO;

    case CMD_FIND_OIL_DEF:
        return CMD_FIND_OIL;

    case CMD_MONITOR_SINGLE_DEF:
        return CMD_MONITOR_SINGLE;

    case CMD_FOLLOW_WATER_DEF:
        return CMD_FOLLOW_WATER;

    default:
        return CMD_NONE;
    }
}

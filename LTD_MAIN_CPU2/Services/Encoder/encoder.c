/*
 * encoder.c
 *
 *  Created on: Mar 21, 2025
 *      Author: Duan Xuebin
 */

#include "AS5145.h"
#include "encoder.h"
#include "mb85rs2m.h"
#include "my_crc.h"
#include "motor_ctrl.h"
#include "system_parameter.h"
#include <limits.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* 当前累计计数与最近成功持久化计数，单位均为AS5145的1/4096圈。 */
volatile int32_t g_encoder_count = 0;
volatile int32_t g_encoder_saved = 0;

/* prev_angle用于跨零点展开单圈角；MAX_ANGLE是AS5145每圈离散计数。 */
static uint16_t prev_angle = 0U;
static const uint16_t MAX_ANGLE = 4096U;

/*
 * V1旧格式：单槽记录，没有结构长度、代次和提交标记。
 * 仅用于上电兼容迁移；读取成功后立即转换为V2，后续不再写V1。
 */
typedef struct {
    uint32_t magic;          /* 固定魔术字，排除未初始化区域。 */
    uint32_t version;        /* 固定为ENCODER_STORE_VERSION_V1。 */
    int32_t encoder_count;   /* 掉电前累计计数。 */
    uint32_t prev_angle;     /* 掉电前单圈角，必须小于4096。 */
    uint32_t crc;            /* 从version到prev_angle的硬件CRC32。 */
} EncoderPersistRecordV1;

/*
 * V2当前格式：A/B原子提交记录中包含保存原因。
 * 写入顺序为未提交主体、完整读回、最后提交标记、再次完整校验。
 */
typedef struct {
    uint32_t magic;          /* 编码器记录魔术字。 */
    uint32_t version;        /* 固定为V2。 */
    uint32_t struct_size;    /* 当前结构实际字节数。 */
    uint32_t generation;     /* 每次成功提交递增，用于A/B选新。 */
    uint32_t persist_reason; /* 普通阈值保存或紧急掉电保存。 */
    int32_t encoder_count;   /* 本次快照累计计数。 */
    uint32_t prev_angle;     /* 本次快照单圈角。 */
    uint32_t sample_sequence;/* 本次快照对应的采样序号。 */
    uint32_t crc;            /* 从version到sample_sequence的CRC32。 */
    uint32_t commit_marker;  /* 最后提交标记，防止接受半写记录。 */
} EncoderPersistRecordV2;

/*
 * 独立掉电回执：只有V2 A/B记录成功后才提交。
 * 下次启动必须同时满足BOR/POR、ADC来源、CRC/标记有效及代次/计数一致，才确认成功。
 */
typedef struct {
    uint32_t magic;              /* 掉电回执魔术字。 */
    uint32_t source;             /* ADC真实低压或PWRTEST软件测试。 */
    uint32_t encoder_generation; /* 与已提交V2记录绑定的代次。 */
    int32_t encoder_count;       /* 与已提交V2记录绑定的累计计数。 */
    uint32_t crc;                /* 从source到encoder_count的CRC32。 */
    uint32_t commit_marker;      /* 最后提交标记。 */
} EncoderPowerSaveReceipt;

/* “ENCD”，用于识别编码器持久化记录。 */
#define ENCODER_STORE_MAGIC                0x454E4344u
/* 支持读取的历史格式版本号；当前只写V2。 */
#define ENCODER_STORE_VERSION_V1           1U
#define ENCODER_STORE_VERSION_V2           2U
/* “COMT”，记录主体读回成功后最后单独写入。 */
#define ENCODER_STORE_COMMIT_MARKER        0x434F4D54u
/* “NORM”和“EMER”，区分普通保存与紧急保存证据。 */
#define ENCODER_PERSIST_REASON_NORMAL       0x4E4F524Du
#define ENCODER_PERSIST_REASON_EMERGENCY    0x454D4552u
/* 编码器A槽沿用历史FRAM角度地址，避免破坏现场数据布局。 */
#define FRAM_ENCODER_A_ADDRESS             FRAM_ANGLE_ADDRESS
/* 每槽预留64字节；V2实际结构必须由编译期断言确认可容纳。 */
#define FRAM_ENCODER_SLOT_SIZE             0x40U
#define FRAM_ENCODER_B_ADDRESS             (FRAM_ENCODER_A_ADDRESS + FRAM_ENCODER_SLOT_SIZE)
/* 独立掉电回执地址，不与A/B记录共享提交标记。 */
#define FRAM_POWER_SAVE_RECEIPT_ADDRESS    0x1180U
/* “PWRS”和“COMT”，分别标识回执类型及完整提交。 */
#define POWER_SAVE_RECEIPT_MAGIC           0x50575253U
#define POWER_SAVE_RECEIPT_COMMIT_MARKER   0x434F4D54U
/* 上电等待AS5145首个有效帧的最大时间，单位ms。 */
#define ENCODER_BOOT_READY_TIMEOUT_MS      1500U
/* 编译期容量检查表达式，防止结构扩展越过A/B槽边界。 */
#define ENCODER_RECORD_V2_FITS_SLOT        (sizeof(EncoderPersistRecordV2) <= FRAM_ENCODER_SLOT_SIZE)
/* 普通运行每累计变化256计数请求保存，约为1/16圈。 */
#define ENCODER_PERSIST_THRESHOLD_COUNTS   256
/* 线程态同步保存单次调用的最大快照提交次数。 */
#define ENCODER_PERSIST_RETRY_LIMIT        3U
/* 紧急PendSV跨多次调度累计允许的总尝试次数。 */
#define ENCODER_EMERGENCY_TOTAL_ATTEMPTS   3U

/* 数组长度为负会使编译失败，从而把FRAM布局约束固化为编译期门禁。 */
typedef char EncoderPersistRecordV2FitsSlot[ENCODER_RECORD_V2_FITS_SLOT ? 1 : -1];
typedef char EncoderPowerSaveReceiptIs24Bytes[(sizeof(EncoderPowerSaveReceipt) == 24U) ? 1 : -1];

/*
 * 下列状态横跨PendSV、SysTick和主循环。volatile字段用于可见性；快照、请求完成和
 * 报告发布等多字段一致性由短临界区及请求序号共同保证。
 */
static volatile uint8_t s_encoder_persist_pending = 0U; /* 普通或紧急快照待提交。 */
static volatile uint8_t s_encoder_emergency_persist_pending = 0U; /* 紧急A/B与回执流程未结束。 */
static volatile uint32_t s_encoder_emergency_request_sequence = 0U; /* 区分保存期间到达的新请求。 */
static volatile uint8_t s_encoder_emergency_attempt_count = 0U; /* 当前紧急请求累计失败次数。 */
static volatile EncoderEmergencyPersistSource s_encoder_emergency_source =
    ENCODER_EMERGENCY_SOURCE_NONE; /* 当前紧急请求来源，ADC优先于测试。 */
static volatile uint8_t s_encoder_position_valid = 0U; /* 累计位置是否可被保存和用于运动。 */
static volatile uint32_t s_encoder_sample_sequence = 0U; /* 每个已处理有效帧递增一次。 */
static volatile EncoderPersistResult s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_NONE; /* 最近提交结果。 */
static volatile EncoderPowerLossPersistResult s_encoder_boot_power_loss_result =
    ENCODER_POWER_LOSS_RESULT_NO_RECORD; /* 本次启动对上次掉电的判定。 */
static volatile uint8_t s_encoder_boot_power_loss_save_failed = 0U; /* 是否需升级为23-6故障。 */
static volatile uint8_t s_encoder_emergency_report_ready = 0U; /* 一次性结果快照可供主循环消费。 */
static EncoderEmergencyPersistenceReport s_encoder_emergency_report; /* PendSV到主循环的固定结果邮箱。 */
static uint16_t s_encoder_saved_angle = 0U; /* 最近成功提交时的单圈角。 */
static uint32_t s_encoder_active_slot = FRAM_ENCODER_A_ADDRESS; /* 当前有效A/B槽地址。 */
static uint32_t s_encoder_generation = 0U; /* 当前活动记录代次。 */

static void update_sensor_height_from_encoder_impl(bool force_position_update);

/* 计算V1记录受保护字段CRC；magic用于快速识别，不纳入历史CRC兼容范围。 */
static uint32_t EncoderRecordV1CRC(const EncoderPersistRecordV1 *record)
{
    const uint8_t *base = (const uint8_t *)&record->version;
    const uint32_t length =
        (uint32_t)(offsetof(EncoderPersistRecordV1, crc) - offsetof(EncoderPersistRecordV1, version));
    return CRC32_HAL(base, length);
}

/* 计算V2记录CRC，覆盖保存原因、累计位置、单圈角和样本序号。 */
static uint32_t EncoderRecordV2CRC(const EncoderPersistRecordV2 *record)
{
    const uint8_t *base = (const uint8_t *)&record->version;
    const uint32_t length =
        (uint32_t)(offsetof(EncoderPersistRecordV2, crc) - offsetof(EncoderPersistRecordV2, version));
    return CRC32_HAL(base, length);
}

/* 计算掉电回执CRC；魔术字和最后提交标记不进入CRC范围。 */
static uint32_t EncoderPowerSaveReceiptCRC(const EncoderPowerSaveReceipt *receipt)
{
    const uint8_t *base = (const uint8_t *)&receipt->source;
    const uint32_t length =
        (uint32_t)(offsetof(EncoderPowerSaveReceipt, crc) -
                   offsetof(EncoderPowerSaveReceipt, source));
    return CRC32_HAL(base, length);
}

/* 完整验证当前V2记录，拒绝未知保存原因和未最终提交的半写记录。 */
static bool EncoderRecordV2IsValid(const EncoderPersistRecordV2 *record)
{
    if (record->magic != ENCODER_STORE_MAGIC) {
        return false;
    }
    if ((record->version != ENCODER_STORE_VERSION_V2) ||
        (record->struct_size != sizeof(EncoderPersistRecordV2))) {
        return false;
    }
    if ((record->persist_reason != ENCODER_PERSIST_REASON_NORMAL) &&
        (record->persist_reason != ENCODER_PERSIST_REASON_EMERGENCY)) {
        return false;
    }
    if (record->prev_angle >= MAX_ANGLE) {
        return false;
    }
    if (record->commit_marker != ENCODER_STORE_COMMIT_MARKER) {
        return false;
    }
    return EncoderRecordV2CRC(record) == record->crc;
}

static bool EncoderReadV2Raw(uint32_t address, EncoderPersistRecordV2 *record)
{
    return FRAM_Read((uint8_t *)record, address, sizeof(*record)) == FRAM_STATUS_OK;
}

static bool EncoderReadV1(uint32_t address, EncoderPersistRecordV1 *record)
{
    if (FRAM_Read((uint8_t *)record, address, sizeof(*record)) != FRAM_STATUS_OK) {
        return false;
    }
    if ((record->magic != ENCODER_STORE_MAGIC) ||
        (record->version != ENCODER_STORE_VERSION_V1) ||
        (record->prev_angle >= MAX_ANGLE)) {
        return false;
    }
    return EncoderRecordV1CRC(record) == record->crc;
}

/* 以有符号差比较32bit代次，使正常回绕后仍能在半个计数空间内判断新旧。 */
static bool EncoderGenerationIsNewer(uint32_t candidate, uint32_t reference)
{
    return ((int32_t)(candidate - reference)) > 0;
}

static bool EncoderPowerSaveReceiptIsValid(const EncoderPowerSaveReceipt *receipt)
{
    if ((receipt->magic != POWER_SAVE_RECEIPT_MAGIC) ||
        (receipt->commit_marker != POWER_SAVE_RECEIPT_COMMIT_MARKER)) {
        return false;
    }
    if ((receipt->source != (uint32_t)ENCODER_EMERGENCY_SOURCE_ADC) &&
        (receipt->source != (uint32_t)ENCODER_EMERGENCY_SOURCE_TEST)) {
        return false;
    }
    return EncoderPowerSaveReceiptCRC(receipt) == receipt->crc;
}

/*
 * 紧急保存开始前先清旧回执提交标记。
 * 若后续掉电中断，新启动只能看到不完整回执，不能误用上一次成功证据。
 */
static bool EncoderInvalidatePowerSaveReceipt(void)
{
    uint32_t cleared_marker = 0U;
    const uint32_t marker_address =
        FRAM_POWER_SAVE_RECEIPT_ADDRESS +
        (uint32_t)offsetof(EncoderPowerSaveReceipt, commit_marker);

    return FRAM_Write((const uint8_t *)&cleared_marker,
                      marker_address,
                      sizeof(cleared_marker)) == FRAM_STATUS_OK;
}

/*
 * 函数用途：在编码器A/B提交完成后写入独立掉电回执。
 * 调用场景：真实低压和PWRTEST紧急保存。
 * 关键约束：先清提交标记，主体写后读回，最后提交并再次校验。
 */
static bool EncoderWritePowerSaveReceipt(EncoderEmergencyPersistSource source,
                                         uint32_t generation,
                                         int32_t encoder_count)
{
    EncoderPowerSaveReceipt receipt;
    EncoderPowerSaveReceipt verify;
    uint32_t marker = POWER_SAVE_RECEIPT_COMMIT_MARKER;
    uint32_t marker_verify = 0U;
    const uint32_t marker_address =
        FRAM_POWER_SAVE_RECEIPT_ADDRESS +
        (uint32_t)offsetof(EncoderPowerSaveReceipt, commit_marker);

    memset(&receipt, 0, sizeof(receipt));
    receipt.magic = POWER_SAVE_RECEIPT_MAGIC;
    receipt.source = (uint32_t)source;
    receipt.encoder_generation = generation;
    receipt.encoder_count = encoder_count;
    receipt.crc = EncoderPowerSaveReceiptCRC(&receipt);

    if (FRAM_Write((const uint8_t *)&receipt,
                   FRAM_POWER_SAVE_RECEIPT_ADDRESS,
                   (uint32_t)offsetof(EncoderPowerSaveReceipt, commit_marker)) !=
        FRAM_STATUS_OK) {
        return false;
    }
    if (FRAM_Read((uint8_t *)&verify,
                  FRAM_POWER_SAVE_RECEIPT_ADDRESS,
                  sizeof(verify)) != FRAM_STATUS_OK) {
        return false;
    }
    if ((memcmp(&receipt,
                &verify,
                offsetof(EncoderPowerSaveReceipt, commit_marker)) != 0) ||
        (verify.commit_marker != 0U)) {
        return false;
    }
    if (FRAM_Write((const uint8_t *)&marker,
                   marker_address,
                   sizeof(marker)) != FRAM_STATUS_OK) {
        return false;
    }
    if ((FRAM_Read((uint8_t *)&marker_verify,
                   marker_address,
                   sizeof(marker_verify)) != FRAM_STATUS_OK) ||
        (marker_verify != marker)) {
        return false;
    }
    return true;
}

/*
 * 函数用途：向非活动槽提交一份带保存原因的编码器快照。
 * 调用场景：运行期阈值保存、停稳强制保存、旧格式迁移和掉电紧急保存。
 * 关键约束：先写未提交记录，完整读回后最后写提交标记；旧活动槽不被本次覆盖。
 */
static bool EncoderPersistSnapshot(int32_t encoder_count,
                                   uint16_t angle,
                                   uint32_t sample_sequence,
                                   uint32_t persist_reason)
{
    EncoderPersistRecordV2 record;
    EncoderPersistRecordV2 verify;
    uint32_t target_slot;
    uint32_t marker = ENCODER_STORE_COMMIT_MARKER;
    uint32_t primask;

    target_slot = (s_encoder_active_slot == FRAM_ENCODER_A_ADDRESS) ?
                  FRAM_ENCODER_B_ADDRESS :
                  FRAM_ENCODER_A_ADDRESS;

    memset(&record, 0, sizeof(record));
    record.magic = ENCODER_STORE_MAGIC;
    record.version = ENCODER_STORE_VERSION_V2;
    record.struct_size = sizeof(EncoderPersistRecordV2);
    record.generation = s_encoder_generation + 1U;
    record.persist_reason = persist_reason;
    record.encoder_count = encoder_count;
    record.prev_angle = angle;
    record.sample_sequence = sample_sequence;
    record.crc = EncoderRecordV2CRC(&record);
    record.commit_marker = 0U;

    if (FRAM_Write((const uint8_t *)&record, target_slot, sizeof(record)) != FRAM_STATUS_OK) {
        s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        return false;
    }
    if (FRAM_Read((uint8_t *)&verify, target_slot, sizeof(verify)) != FRAM_STATUS_OK) {
        s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        return false;
    }
    if (memcmp(&record, &verify, sizeof(record)) != 0) {
        s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        return false;
    }
    if (FRAM_Write((const uint8_t *)&marker,
                   target_slot + (uint32_t)offsetof(EncoderPersistRecordV2, commit_marker),
                   sizeof(marker)) != FRAM_STATUS_OK) {
        s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        return false;
    }
    if (!EncoderReadV2Raw(target_slot, &verify) ||
        !EncoderRecordV2IsValid(&verify)) {
        s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        return false;
    }
    if ((verify.generation != record.generation) ||
        (verify.persist_reason != record.persist_reason) ||
        (verify.encoder_count != record.encoder_count) ||
        (verify.prev_angle != record.prev_angle) ||
        (verify.sample_sequence != record.sample_sequence)) {
        s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    s_encoder_active_slot = target_slot;
    s_encoder_generation = record.generation;
    g_encoder_saved = encoder_count;
    s_encoder_saved_angle = angle;
    s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_OK;
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

/* 以64bit计算当前值与已保存值的绝对差，避免int32减法在边界溢出。 */
static int64_t EncoderUnsavedDistance(void)
{
    int64_t distance = (int64_t)g_encoder_count - (int64_t)g_encoder_saved;
    return (distance < 0) ? -distance : distance;
}

/*
 * 在短临界区一次性捕获位置与紧急请求序号。
 * 序号用于提交完成时确认本次快照没有误清保存期间新到达的请求。
 */
static void EncoderCaptureSnapshot(int32_t *encoder_count,
                                   uint16_t *angle,
                                   uint32_t *sample_sequence,
                                   uint32_t *emergency_request_sequence,
                                   uint8_t *emergency_request_active,
                                   EncoderEmergencyPersistSource *emergency_source)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *encoder_count = g_encoder_count;
    *angle = prev_angle;
    *sample_sequence = s_encoder_sample_sequence;
    *emergency_request_sequence = s_encoder_emergency_request_sequence;
    *emergency_request_active = s_encoder_emergency_persist_pending;
    *emergency_source = s_encoder_emergency_source;
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 成功提交后只清除序号仍匹配的紧急请求；随后依据新请求或256计数差重算普通脏标志。
 */
static void EncoderRefreshPendingAfterCommit(uint32_t satisfied_emergency_sequence)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if ((s_encoder_emergency_persist_pending != 0U) &&
        (s_encoder_emergency_request_sequence == satisfied_emergency_sequence)) {
        s_encoder_emergency_persist_pending = 0U;
        s_encoder_emergency_source = ENCODER_EMERGENCY_SOURCE_NONE;
    }
    s_encoder_persist_pending =
        ((s_encoder_emergency_persist_pending != 0U) ||
         (EncoderUnsavedDistance() >= ENCODER_PERSIST_THRESHOLD_COUNTS)) ? 1U : 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：记录一次紧急FRAM提交成功快照。
 * 调用场景：PendSV完成紧急编码器持久化后。
 * 关键约束：只写固定RAM结构并置就绪标志，不打印、不阻塞。
 */
static void Encoder_RecordEmergencyPersistenceResult(
    EncoderEmergencyPersistSource source,
    bool receipt_committed)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    s_encoder_emergency_report.encoder_count = g_encoder_saved;
    s_encoder_emergency_report.angle = s_encoder_saved_angle;
    s_encoder_emergency_report.generation = s_encoder_generation;
    s_encoder_emergency_report.active_slot =
        (s_encoder_active_slot == FRAM_ENCODER_A_ADDRESS) ? (uint8_t)'A' : (uint8_t)'B';
    s_encoder_emergency_report.source = source;
    s_encoder_emergency_report.receipt_committed = receipt_committed ? 1U : 0U;
    __DMB();
    s_encoder_emergency_report_ready = 1U;
    if (primask == 0U) {
        __enable_irq();
    }
}

/*
 * 函数用途：在紧急保存达到上限后结束请求并发布失败结果。
 * 调用场景：PendSV 中 A/B 记录或掉电回执连续提交失败。
 * 关键约束：真实低压仍保持电机禁止；测试请求必须释放 FRAM 预约。
 */
static void EncoderFinishEmergencyFailure(
    uint32_t satisfied_emergency_sequence,
    EncoderEmergencyPersistSource emergency_source)
{
    uint32_t primask = __get_PRIMASK();
    bool finished = false;

    __disable_irq();
    if ((s_encoder_emergency_persist_pending != 0U) &&
        (s_encoder_emergency_request_sequence == satisfied_emergency_sequence)) {
        s_encoder_emergency_persist_pending = 0U;
        s_encoder_emergency_source = ENCODER_EMERGENCY_SOURCE_NONE;
        s_encoder_emergency_attempt_count = 0U;
        s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        s_encoder_persist_pending =
            (EncoderUnsavedDistance() >= ENCODER_PERSIST_THRESHOLD_COUNTS) ? 1U : 0U;
        finished = true;
    }
    if (primask == 0U) {
        __enable_irq();
    }

    if (finished) {
        Encoder_RecordEmergencyPersistenceResult(emergency_source, false);
        if (emergency_source == ENCODER_EMERGENCY_SOURCE_TEST) {
            FRAM_ReleaseEmergencyReservation();
        }
    }
}

/*
 * 函数用途：在PendSV中最多处理一份普通或紧急编码器快照。
 * 调用场景：每次编码器延后事件处理之后。
 * 关键约束：紧急请求先取得FRAM紧急所有权；失败最多跨三次PendSV重试。
 */
void Encoder_ProcessDeferredPersistence(void)
{
    int32_t encoder_count;
    uint16_t angle;
    uint32_t sample_sequence;
    uint32_t emergency_request_sequence;
    uint8_t emergency_request_active;
    EncoderEmergencyPersistSource emergency_source;
    bool commit_success;
    bool receipt_committed = false;
    bool emergency_receipt_invalidated = true;

    if ((s_encoder_persist_pending == 0U) ||
        (s_encoder_position_valid == 0U)) {
        return;
    }

    EncoderCaptureSnapshot(&encoder_count,
                           &angle,
                           &sample_sequence,
                           &emergency_request_sequence,
                           &emergency_request_active,
                           &emergency_source);
    if (emergency_request_active != 0U) {
        /*
         * 紧急所有者可以越过普通写禁止和预约门禁；先作废旧回执，再写本次V2快照。
         */
        FRAM_EnterEmergencyOwner();
        emergency_receipt_invalidated = EncoderInvalidatePowerSaveReceipt();
        if (!emergency_receipt_invalidated) {
            s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        }
    }
    commit_success =
        emergency_receipt_invalidated &&
        EncoderPersistSnapshot(
            encoder_count,
            angle,
            sample_sequence,
            (emergency_request_active != 0U) ?
                ENCODER_PERSIST_REASON_EMERGENCY :
                ENCODER_PERSIST_REASON_NORMAL);
    if (commit_success && (emergency_request_active != 0U)) {
        receipt_committed =
            EncoderWritePowerSaveReceipt(emergency_source,
                                         s_encoder_generation,
                                         g_encoder_saved);
    }
    if (emergency_request_active != 0U) {
        FRAM_ExitEmergencyOwner();
    }

    /*
     * 普通保存只要求V2记录成功；紧急保存必须再有独立回执成功，才允许结束请求。
     */
    if (commit_success &&
        ((emergency_request_active == 0U) || receipt_committed)) {
        s_encoder_position_valid = 1U;
        EncoderRefreshPendingAfterCommit(emergency_request_sequence);
        if (emergency_request_active != 0U) {
            s_encoder_emergency_attempt_count = 0U;
            Encoder_RecordEmergencyPersistenceResult(emergency_source, true);
            if ((emergency_source == ENCODER_EMERGENCY_SOURCE_TEST) &&
                (!Encoder_HasEmergencyPersistencePending())) {
                FRAM_ReleaseEmergencyReservation();
            }
        }
    } else if (emergency_request_active != 0U) {
        if (s_encoder_emergency_attempt_count < UINT8_MAX) {
            s_encoder_emergency_attempt_count++;
        }
        s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        if (s_encoder_emergency_attempt_count >=
            ENCODER_EMERGENCY_TOTAL_ATTEMPTS) {
            EncoderFinishEmergencyFailure(emergency_request_sequence,
                                          emergency_source);
        }
    }
}

/*
 * 函数用途：锁存一次掉电紧急保存请求。
 * 调用场景：24V模拟看门狗中断。
 * 关键约束：只置RAM标志，不访问FRAM、不打印、不阻塞。
 */
void Encoder_RequestEmergencyPersistenceFromISR(EncoderEmergencyPersistSource source)
{
    if ((source != ENCODER_EMERGENCY_SOURCE_ADC) &&
        (source != ENCODER_EMERGENCY_SOURCE_TEST)) {
        return;
    }
    s_encoder_emergency_request_sequence++;
    if ((s_encoder_emergency_persist_pending == 0U) ||
        (source == ENCODER_EMERGENCY_SOURCE_ADC)) {
        s_encoder_emergency_attempt_count = 0U;
    }
    if ((s_encoder_emergency_source != ENCODER_EMERGENCY_SOURCE_ADC) ||
        (source == ENCODER_EMERGENCY_SOURCE_ADC)) {
        s_encoder_emergency_source = source;
    }
    s_encoder_emergency_persist_pending = 1U;
    s_encoder_persist_pending = 1U;
}

bool Encoder_HasEmergencyPersistencePending(void)
{
    return s_encoder_emergency_persist_pending != 0U;
}

/*
 * 函数用途：取得编码器运行值、持久化值和活动槽的一致快照。
 * 调用场景：线程态处理ENC?命令。
 * 关键约束：只短暂关中断复制RAM状态，不读取FRAM。
 */
bool Encoder_GetDebugSnapshot(EncoderDebugSnapshot *snapshot)
{
    int64_t unsaved;
    int32_t raw_delta;
    uint32_t primask;

    if (snapshot == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    snapshot->encoder_count = g_encoder_count;
    snapshot->saved_count = g_encoder_saved;
    snapshot->current_angle = prev_angle;
    snapshot->saved_angle = s_encoder_saved_angle;
    snapshot->generation = s_encoder_generation;
    snapshot->active_slot =
        (s_encoder_active_slot == FRAM_ENCODER_A_ADDRESS) ? (uint8_t)'A' : (uint8_t)'B';
    snapshot->position_valid = s_encoder_position_valid;
    snapshot->persistence_pending = s_encoder_persist_pending;
    snapshot->emergency_persistence_pending = s_encoder_emergency_persist_pending;
    snapshot->last_commit_result = s_encoder_last_commit_result;
    snapshot->boot_power_loss_result = s_encoder_boot_power_loss_result;
    if (primask == 0U) {
        __enable_irq();
    }

    unsaved = (int64_t)snapshot->encoder_count - (int64_t)snapshot->saved_count;
    if (unsaved > INT32_MAX) {
        snapshot->unsaved_count = INT32_MAX;
    } else if (unsaved < INT32_MIN) {
        snapshot->unsaved_count = INT32_MIN;
    } else {
        snapshot->unsaved_count = (int32_t)unsaved;
    }

    raw_delta = (int32_t)snapshot->current_angle - (int32_t)snapshot->saved_angle;
    if (raw_delta > (int32_t)(MAX_ANGLE / 2U)) {
        raw_delta -= (int32_t)MAX_ANGLE;
    } else if (raw_delta < -(int32_t)(MAX_ANGLE / 2U)) {
        raw_delta += (int32_t)MAX_ANGLE;
    } else {
        /* 差值已在单圈最短路径范围内。 */
    }
    snapshot->raw_delta = raw_delta;
    snapshot->fault_latched = AS5145_IsFaultLatched() ? 1U : 0U;
    return true;
}

/*
 * 函数用途：消费一份紧急持久化成功快照。
 * 调用场景：主循环输出POWER_SAVE结果。
 * 关键约束：PendSV只发布固定快照，消费方清除一次性就绪标志。
 */
bool Encoder_TakeEmergencyPersistenceReport(EncoderEmergencyPersistenceReport *report)
{
    uint32_t primask;

    if (report == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (s_encoder_emergency_report_ready == 0U) {
        if (primask == 0U) {
            __enable_irq();
        }
        return false;
    }
    *report = s_encoder_emergency_report;
    s_encoder_emergency_report_ready = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return true;
}

/*
 * 函数用途：在线程态同步提交当前编码器快照，单次最多尝试三次。
 * 调用场景：停稳、回零、人工修正及受控断电前。
 * 关键约束：紧急请求存在时必须同时提交独立回执；失败不推进已保存基线。
 */
uint32_t Encoder_SaveCurrentPosition(void)
{
    int32_t encoder_count;
    uint16_t angle;
    uint32_t sample_sequence;
    uint32_t emergency_request_sequence;
    uint8_t emergency_request_active;
    EncoderEmergencyPersistSource emergency_source;
    uint32_t attempt;
    bool commit_success = false;
    bool receipt_committed = false;
    bool emergency_receipt_invalidated = true;

    if (s_encoder_position_valid == 0U) {
        return ENCODER_POWERON_FAIL;
    }

    EncoderCaptureSnapshot(&encoder_count,
                           &angle,
                           &sample_sequence,
                           &emergency_request_sequence,
                           &emergency_request_active,
                           &emergency_source);
    if (emergency_request_active != 0U) {
        /*
         * 紧急所有者可以越过普通写禁止和预约门禁；先作废旧回执，再写本次V2快照。
         */
        FRAM_EnterEmergencyOwner();
        emergency_receipt_invalidated = EncoderInvalidatePowerSaveReceipt();
        if (!emergency_receipt_invalidated) {
            s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_FAILED;
        }
    }
    for (attempt = 0U;
         emergency_receipt_invalidated &&
         (attempt < ENCODER_PERSIST_RETRY_LIMIT);
         attempt++) {
        if (EncoderPersistSnapshot(
                encoder_count,
                angle,
                sample_sequence,
                (emergency_request_active != 0U) ?
                    ENCODER_PERSIST_REASON_EMERGENCY :
                    ENCODER_PERSIST_REASON_NORMAL)) {
            commit_success = true;
            break;
        }
    }
    if (commit_success && (emergency_request_active != 0U)) {
        receipt_committed =
            EncoderWritePowerSaveReceipt(emergency_source,
                                         s_encoder_generation,
                                         g_encoder_saved);
    }
    if (emergency_request_active != 0U) {
        FRAM_ExitEmergencyOwner();
    }

    /*
     * 普通保存只要求V2记录成功；紧急保存必须再有独立回执成功，才允许结束请求。
     */
    if (commit_success &&
        ((emergency_request_active == 0U) || receipt_committed)) {
        s_encoder_position_valid = 1U;
        EncoderRefreshPendingAfterCommit(emergency_request_sequence);
        if (emergency_request_active != 0U) {
            s_encoder_emergency_attempt_count = 0U;
            Encoder_RecordEmergencyPersistenceResult(emergency_source, true);
            if ((emergency_source == ENCODER_EMERGENCY_SOURCE_TEST) &&
                (!Encoder_HasEmergencyPersistencePending())) {
                FRAM_ReleaseEmergencyReservation();
            }
        }
        return NO_ERROR;
    }

    s_encoder_persist_pending = 1U;
    return ENCODER_POWERON_FAIL;
}

bool Encoder_IsReady(void)
{
    return (s_encoder_position_valid != 0U) && AS5145_HasValidSample();
}

uint32_t Encoder_WaitReady(uint32_t timeout_ms)
{
    if (s_encoder_position_valid == 0U) {
        return ENCODER_POWERON_FAIL;
    }
    return AS5145_WaitFirstValidSample(timeout_ms);
}

bool Encoder_CanStartHoming(void)
{
    return AS5145_HasValidSample();
}

bool Encoder_HasTrustedPosition(void)
{
    return s_encoder_position_valid != 0U;
}

bool Encoder_DidBootDetectPowerLossSaveFailure(void)
{
    return s_encoder_boot_power_loss_save_failed != 0U;
}

void Encoder_BeginNewProcess(void)
{
    AS5145_ClearLatchedFaultForNewProcess();
}

bool Encoder_HasLatchedFault(void)
{
    return AS5145_IsFaultLatched();
}

/*
 * 函数用途：上电时把独立回执、当前V2活动记录和MCU复位原因交叉校验。
 * 调用场景：A/B位置恢复完成后、启动编码器定时采集之前。
 * 关键约束：只有BOR/POR下真实ADC回执异常才置23-6；软件测试或软件复位只记录诊断。
 */
static void EncoderConsumePowerSaveReceipt(void)
{
    EncoderPowerSaveReceipt receipt;
    uint32_t reset_flags = RCC->CSR;
    bool receipt_read;
    bool receipt_valid;
    bool position_matches;
    bool power_reset;
    bool adc_receipt;
    bool test_receipt;
    bool incomplete_receipt;
    const char active_slot =
        (s_encoder_active_slot == FRAM_ENCODER_A_ADDRESS) ? 'A' : 'B';

    receipt_read =
        FRAM_Read((uint8_t *)&receipt,
                  FRAM_POWER_SAVE_RECEIPT_ADDRESS,
                  sizeof(receipt)) == FRAM_STATUS_OK;
    receipt_valid = receipt_read && EncoderPowerSaveReceiptIsValid(&receipt);
    position_matches =
        receipt_valid &&
        (s_encoder_position_valid != 0U) &&
        (receipt.encoder_generation == s_encoder_generation) &&
        (receipt.encoder_count == g_encoder_saved);
    power_reset =
        (reset_flags & (RCC_CSR_BORRSTF | RCC_CSR_PORRSTF)) != 0U;
    adc_receipt =
        receipt_valid &&
        (receipt.source == (uint32_t)ENCODER_EMERGENCY_SOURCE_ADC);
    test_receipt =
        receipt_valid &&
        (receipt.source == (uint32_t)ENCODER_EMERGENCY_SOURCE_TEST);
    incomplete_receipt =
        receipt_read &&
        (receipt.magic == POWER_SAVE_RECEIPT_MAGIC) &&
        (!receipt_valid);

    s_encoder_boot_power_loss_result = ENCODER_POWER_LOSS_RESULT_NO_RECORD;
    s_encoder_boot_power_loss_save_failed = 0U;
    /*
     * 真实低压成功必须同时具备四项证据：BOR/POR、ADC来源、有效提交回执、
     * 回执代次/计数与当前选中的A/B记录一致，缺一项都不能宣布成功。
     */
    if (power_reset && adc_receipt && position_matches) {
        s_encoder_boot_power_loss_result = ENCODER_POWER_LOSS_RESULT_SUCCESS;
        printf("[编码器][上电][掉电存储] 结果=成功，依据=真实低压回执和掉电复位一致，累计编码=%ld，存储代次=%lu，活动槽=%c，复位标志=0x%08lX\r\n",
               (long)g_encoder_saved,
               (unsigned long)s_encoder_generation,
               active_slot,
               (unsigned long)reset_flags);
    } else if (power_reset && adc_receipt) {
        s_encoder_boot_power_loss_result = ENCODER_POWER_LOSS_RESULT_INCOMPLETE;
        s_encoder_boot_power_loss_save_failed = 1U;
        printf("[编码器][上电][掉电存储] 结果=失败，原因=真实低压回执与当前A/B记录不一致，回执代次=%lu，当前代次=%lu，复位标志=0x%08lX\r\n",
               (unsigned long)receipt.encoder_generation,
               (unsigned long)s_encoder_generation,
               (unsigned long)reset_flags);
    } else if (test_receipt) {
        /* PWRTEST只验证存储链，不能伪造成现场24V低压，也不能触发23-6。 */
        printf("[编码器][上电][掉电存储] 结果=未确认，原因=检测到软件测试回执，不作为真实掉电结果，累计编码=%ld，存储代次=%lu，活动槽=%c\r\n",
               (long)g_encoder_saved,
               (unsigned long)s_encoder_generation,
               active_slot);
    } else if (power_reset && incomplete_receipt) {
        s_encoder_boot_power_loss_result = ENCODER_POWER_LOSS_RESULT_INCOMPLETE;
        s_encoder_boot_power_loss_save_failed = 1U;
        printf("[编码器][上电][掉电存储] 结果=失败，原因=掉电复位且回执未完整提交或校验失败，当前代次=%lu，复位标志=0x%08lX\r\n",
               (unsigned long)s_encoder_generation,
               (unsigned long)reset_flags);
    } else if (adc_receipt && !power_reset) {
        /*
         * 看门狗、软件或调试复位不证明供电曾中断；保留诊断但不报告掉电保存失败。
         */
        printf("[编码器][上电][掉电存储] 结果=未确认，原因=真实低压回执存在但复位原因不是掉电或上电，存储代次=%lu，复位标志=0x%08lX\r\n",
               (unsigned long)s_encoder_generation,
               (unsigned long)reset_flags);
    } else if (receipt_read && (receipt.magic == POWER_SAVE_RECEIPT_MAGIC)) {
        printf("[编码器][上电][掉电存储] 结果=未确认，原因=存在不完整回执但本次不是掉电复位，当前代次=%lu，复位标志=0x%08lX\r\n",
               (unsigned long)s_encoder_generation,
               (unsigned long)reset_flags);
    } else {
        printf("[编码器][上电][掉电存储] 结果=无可判定回执，当前代次=%lu，复位标志=0x%08lX\r\n",
               (unsigned long)s_encoder_generation,
               (unsigned long)reset_flags);
    }

    if (receipt_read && (receipt.magic == POWER_SAVE_RECEIPT_MAGIC) &&
        (!EncoderInvalidatePowerSaveReceipt())) {
        printf("[编码器][上电][掉电存储] 回执消费标记清除失败，下次上电仍会重新校验\r\n");
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

/*
 * 函数用途：按V2、V1优先级恢复编码器位置并启动采集。
 * 调用场景：CPU2业务初始化。
 * 关键约束：V2单槽有效允许降级运行并修复；所有格式都无效时禁止把零值当可信位置。
 */
uint32_t Initialize_Encoder(void)
{
    EncoderPersistRecordV2 record_a_v2;
    EncoderPersistRecordV2 record_b_v2;
    EncoderPersistRecordV1 record_a_v1;
    EncoderPersistRecordV1 record_b_v1;
    bool raw_a_v2;
    bool raw_b_v2;
    bool valid_a_v2;
    bool valid_b_v2;
    bool valid_a_v1;
    bool valid_b_v1;
    bool repair_required = false;
    bool migration_required = false;
    uint32_t init_error = NO_ERROR;
    HAL_StatusTypeDef start_status;
    uint32_t ready_result;

    s_encoder_last_commit_result = ENCODER_PERSIST_RESULT_NONE;
    s_encoder_boot_power_loss_result = ENCODER_POWER_LOSS_RESULT_NO_RECORD;
    s_encoder_boot_power_loss_save_failed = 0U;
    s_encoder_emergency_report_ready = 0U;
    s_encoder_emergency_source = ENCODER_EMERGENCY_SOURCE_NONE;

    raw_a_v2 = EncoderReadV2Raw(FRAM_ENCODER_A_ADDRESS, &record_a_v2);
    raw_b_v2 = EncoderReadV2Raw(FRAM_ENCODER_B_ADDRESS, &record_b_v2);
    valid_a_v2 = raw_a_v2 && EncoderRecordV2IsValid(&record_a_v2);
    valid_b_v2 = raw_b_v2 && EncoderRecordV2IsValid(&record_b_v2);

    if (valid_a_v2 || valid_b_v2) {
        /*
         * 当前格式优先：双槽有效选较新代次，单槽有效继续运行并安排冗余修复。
         */
        const EncoderPersistRecordV2 *selected;

        if (valid_a_v2 && valid_b_v2) {
            if (EncoderGenerationIsNewer(record_b_v2.generation, record_a_v2.generation)) {
                selected = &record_b_v2;
                s_encoder_active_slot = FRAM_ENCODER_B_ADDRESS;
            } else {
                selected = &record_a_v2;
                s_encoder_active_slot = FRAM_ENCODER_A_ADDRESS;
            }
        } else if (valid_a_v2) {
            selected = &record_a_v2;
            s_encoder_active_slot = FRAM_ENCODER_A_ADDRESS;
            repair_required = true;
        } else {
            selected = &record_b_v2;
            s_encoder_active_slot = FRAM_ENCODER_B_ADDRESS;
            repair_required = true;
        }

        g_encoder_count = selected->encoder_count;
        prev_angle = (uint16_t)selected->prev_angle;
        s_encoder_sample_sequence = selected->sample_sequence;
        s_encoder_generation = selected->generation;
        s_encoder_position_valid = 1U;
        g_encoder_saved = g_encoder_count;
        s_encoder_saved_angle = prev_angle;
    } else {
        valid_a_v1 = EncoderReadV1(FRAM_ENCODER_A_ADDRESS, &record_a_v1);
        valid_b_v1 = EncoderReadV1(FRAM_ENCODER_B_ADDRESS, &record_b_v1);

        if (valid_a_v1 || valid_b_v1) {
            /* V1没有代次，只作兼容来源；恢复后立即迁移到V2。 */
            const EncoderPersistRecordV1 *selected_v1;

            if (valid_a_v1) {
                selected_v1 = &record_a_v1;
                s_encoder_active_slot = FRAM_ENCODER_A_ADDRESS;
            } else {
                selected_v1 = &record_b_v1;
                s_encoder_active_slot = FRAM_ENCODER_B_ADDRESS;
            }

            g_encoder_count = selected_v1->encoder_count;
            prev_angle = (uint16_t)selected_v1->prev_angle;
            s_encoder_sample_sequence = 0U;
            s_encoder_generation = 0U;
            s_encoder_position_valid = 1U;
            g_encoder_saved = g_encoder_count;
            s_encoder_saved_angle = prev_angle;
            migration_required = true;
        } else {
            /*
             * 所有持久化格式都无效：RAM归零仅用于确定性初始化，position_valid保持0，
             * 只有明确回零或人工修正才能重新建立可信位置。
             */
            g_encoder_count = 0;
            g_encoder_saved = 0;
            prev_angle = 0U;
            s_encoder_saved_angle = 0U;
            s_encoder_sample_sequence = 0U;
            s_encoder_generation = 0U;
            s_encoder_active_slot = FRAM_ENCODER_A_ADDRESS;
            s_encoder_position_valid = 0U;
            s_encoder_persist_pending = 0U;
            init_error = ENCODER_POWERON_FAIL;
            g_measurement.device_status.error_code = init_error;
            printf("编码器持久化 A/B 分区均无有效记录，必须重新回零\r\n");
        }
    }

    if (s_encoder_position_valid != 0U) {
        update_sensor_height_from_encoder();
    }

    EncoderConsumePowerSaveReceipt();

    if ((repair_required || migration_required) &&
        (Encoder_SaveCurrentPosition() != NO_ERROR)) {
        s_encoder_persist_pending = 1U;
        printf("编码器持久化冗余修复暂未完成，将在后续采样继续尝试\r\n");
    }

    start_status = Start_Encoder_Collection_TIM();
    if (start_status != HAL_OK) {
        if (!MotorCtrl_IsPositionSourceMotor()) {
            g_measurement.device_status.error_code = ENCODER_TIMEOUT;
            init_error = ENCODER_TIMEOUT;
        }
        return init_error;
    }

    ready_result = Encoder_WaitReady(ENCODER_BOOT_READY_TIMEOUT_MS);
    if ((ready_result != NO_ERROR) && (!MotorCtrl_IsPositionSourceMotor())) {
        g_measurement.device_status.error_code = ready_result;
        if (init_error == NO_ERROR) {
            init_error = ready_result;
        }
        printf("编码器就绪等待失败：0x%08lX\r\n", (unsigned long)ready_result);
    }

    return init_error;
}
void Update_Encoder_Count(uint16_t current_angle)
{
    int16_t delta = (int16_t)(current_angle - prev_angle);

    if (delta > (int16_t)(MAX_ANGLE / 2U)) {
        delta = (int16_t)(delta - (int16_t)MAX_ANGLE);
    } else if (delta < -(int16_t)(MAX_ANGLE / 2U)) {
        delta = (int16_t)(delta + (int16_t)MAX_ANGLE);
    }

    g_encoder_count += delta;
    prev_angle = current_angle;
    s_encoder_sample_sequence++;
    update_sensor_height_from_encoder();

    if ((s_encoder_position_valid != 0U) &&
        (EncoderUnsavedDistance() >= ENCODER_PERSIST_THRESHOLD_COUNTS)) {
        s_encoder_persist_pending = 1U;
    }
}

/*
 * 根据编码器脉冲值更新传感器高度测量。
 */
static void update_sensor_height_from_encoder_impl(bool force_position_update)
{
    float revolutions;
    float cable_length;
    float current_height;

    g_measurement.debug_data.current_encoder_value = -g_encoder_count;
    if ((!force_position_update) && MotorCtrl_IsPositionSourceMotor()) {
        return;
    }

    revolutions = (float)g_measurement.debug_data.current_encoder_value / (float)MAX_ANGLE;
    cable_length = g_deviceParams.encoder_wheel_circumference_mm * revolutions / 100;
    current_height = g_deviceParams.tankHeight - cable_length;
    g_measurement.debug_data.cable_length = (int)cable_length;
    g_measurement.debug_data.sensor_position = (int)current_height;
}

void update_sensor_height_from_encoder(void)
{
    update_sensor_height_from_encoder_impl(false);
}

void update_sensor_height_from_encoder_force(void)
{
    update_sensor_height_from_encoder_impl(true);
}

int32_t encoder_get_cable_length_01mm(void)
{
    const float encoder_value = (float)(-g_encoder_count);
    const float revolutions = encoder_value / (float)MAX_ANGLE;
    const float cable_length =
        g_deviceParams.encoder_wheel_circumference_mm * revolutions / 100.0f;

    return (int32_t)cable_length;
}

int32_t encoder_get_sensor_position_01mm(void)
{
    return (int32_t)g_deviceParams.tankHeight - encoder_get_cable_length_01mm();
}

/*
 * 函数用途：建立编码器零点并同步保存可信位置。
 * 调用场景：回零、零点标定或人工调试完成后。
 * 关键约束：持久化失败必须把错误返回调用方，不能继续宣告标零成功。
 */
uint32_t set_encoder_zero(void)
{
    uint32_t save_result;

    printf("设置编码器零点，置零前编码值 %ld\r\n", (long)g_encoder_count);
    g_encoder_count = 0;
    s_encoder_sample_sequence++;
    s_encoder_position_valid = 1U;
    s_encoder_persist_pending = 1U;
    update_sensor_height_from_encoder();

    save_result = Encoder_SaveCurrentPosition();
    if (save_result != NO_ERROR) {
        g_measurement.device_status.error_code = save_result;
        printf("编码器零点持久化失败：0x%08lX\r\n", (unsigned long)save_result);
        return save_result;
    }
    if (g_measurement.device_status.error_code == ENCODER_POWERON_FAIL) {
        g_measurement.device_status.error_code = NO_ERROR;
    }
    printf("编码器零点设置为 %ld\r\n", (long)g_encoder_count);
    return NO_ERROR;
}

void encoder_set_cable_length_01mm(int32_t cable_length_01mm)
{
    double encoder_value;
    int32_t new_count;
    uint32_t save_result;

    if (cable_length_01mm < 0) {
        cable_length_01mm = 0;
    }
    if (g_deviceParams.encoder_wheel_circumference_mm == 0U) {
        printf("编码器修正失败：编码轮周长为0\r\n");
        return;
    }

    encoder_value = ((double)cable_length_01mm * 100.0 * (double)MAX_ANGLE) /
                    (double)g_deviceParams.encoder_wheel_circumference_mm;
    new_count = -(int32_t)(encoder_value + 0.5);

    printf("编码器修正 | 目标尺带长度：%ld(0.1mm) | 原计数=%ld | 新计数=%ld\r\n",
           (long)cable_length_01mm,
           (long)g_encoder_count,
           (long)new_count);

    g_encoder_count = new_count;
    s_encoder_sample_sequence++;
    s_encoder_position_valid = 1U;
    s_encoder_persist_pending = 1U;
    update_sensor_height_from_encoder();

    save_result = Encoder_SaveCurrentPosition();
    if (save_result != NO_ERROR) {
        g_measurement.device_status.error_code = save_result;
        printf("编码器修正持久化失败：0x%08lX\r\n", (unsigned long)save_result);
    }
}

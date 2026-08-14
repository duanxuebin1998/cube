/*
 * encoder.h
 *
 *  Created on: Mar 21, 2025
 *      Author: Duan Xuebin
 */

#ifndef ENCODER_ENCODER_H_
/* ENCODER_ENCODER_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define ENCODER_ENCODER_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * 编码器跨模块累计计数，单位为AS5145的1/4096圈。
 * 由PendSV消费有效帧时更新，读取方若需与其它字段一致必须使用调试快照接口。
 */
extern volatile int32_t g_encoder_count;

/* 最近一次FRAM编码器快照提交结果；NONE表示本次上电尚未尝试。 */
typedef enum {
    ENCODER_PERSIST_RESULT_NONE = 0, /* 未执行提交。 */
    ENCODER_PERSIST_RESULT_OK,       /* A/B新槽和完整提交标记均已读回验证。 */
    ENCODER_PERSIST_RESULT_FAILED    /* 记录主体、提交标记或读回验证失败。 */
} EncoderPersistResult;

/* 紧急保存来源会写入V2记录和独立回执，用于区分真实低压与软件测试。 */
typedef enum {
    ENCODER_EMERGENCY_SOURCE_NONE = 0, /* 当前没有紧急请求。 */
    ENCODER_EMERGENCY_SOURCE_ADC,      /* ADC看门狗确认的真实24V低压。 */
    ENCODER_EMERGENCY_SOURCE_TEST      /* PWRTEST软件请求，不作为真实掉电证据。 */
} EncoderEmergencyPersistSource;

/* 本次启动对“上一次掉电保存”的只读判定结果。 */
typedef enum {
    ENCODER_POWER_LOSS_RESULT_NO_RECORD = 0, /* 没有可与BOR/POR共同判定的回执。 */
    ENCODER_POWER_LOSS_RESULT_SUCCESS,       /* 真实低压回执与当前A/B记录完全一致。 */
    ENCODER_POWER_LOSS_RESULT_INCOMPLETE     /* 掉电复位下回执不完整或与A/B不一致。 */
} EncoderPowerLossPersistResult;

/* ENC? 使用的一致RAM快照，不在查询路径读取FRAM。 */
typedef struct {
    int32_t encoder_count;             /* 当前累计计数，单位1/4096圈。 */
    int32_t saved_count;               /* 最近成功提交到V2记录的累计计数。 */
    int32_t unsaved_count;             /* 当前值减已保存值，保留运动方向。 */
    int32_t raw_delta;                 /* 当前单圈角与已保存单圈角的最短路径差。 */
    uint32_t generation;               /* 当前活动A/B记录的单调递增代次。 */
    uint16_t current_angle;            /* 最近有效AS5145单圈角，范围0至4095。 */
    uint16_t saved_angle;              /* 最近成功提交时的单圈角。 */
    uint8_t active_slot;               /* 当前活动槽字符：'A'或'B'。 */
    uint8_t position_valid;            /* 1表示累计位置来自有效记录或明确重建。 */
    uint8_t persistence_pending;       /* 普通或紧急快照仍需提交。 */
    uint8_t emergency_persistence_pending; /* 紧急快照和回执流程尚未结束。 */
    uint8_t fault_latched;             /* AS5145连续异常故障是否锁存。 */
    EncoderPersistResult last_commit_result; /* 最近一次快照提交结果。 */
    EncoderPowerLossPersistResult boot_power_loss_result; /* 上次掉电保存判定。 */
    int16_t last_candidate_delta;       /* 最近一次候选单帧增量。 */
    uint16_t last_candidate_limit;      /* 最近一次候选增量物理上限。 */
    uint32_t last_candidate_dt_ms;      /* 最近一次候选与可信样本间隔。 */
    uint32_t rejected_sample_count;     /* 本次上电累计拒绝的候选样本数。 */
    uint32_t direction_mismatch_count;  /* 方向状态与编码增量不一致的诊断次数，不触发停机。 */
    uint8_t last_reject_reason;         /* 最近拒绝原因，0表示未拒绝。 */
    uint8_t boot_stable_count;          /* 上电角度稳定确认帧数。 */
    uint8_t angle_synchronized;         /* 单圈角是否已与累计位置同步。 */
    uint8_t runtime_resynchronizing;    /* 1表示运行故障后仅重建角度基准，不执行上电位置比较。 */
} EncoderDebugSnapshot;

typedef enum {
    ENCODER_REJECT_NONE = 0,
    ENCODER_REJECT_BOOT_UNSTABLE,
    ENCODER_REJECT_BOOT_CHANGE,
    ENCODER_REJECT_SAMPLE_GAP,
    ENCODER_REJECT_PHYSICAL_LIMIT
} EncoderRejectReason;

/* PendSV向主循环发布的一次性紧急保存结果；主循环只负责报告和故障升级。 */
typedef struct {
    int32_t encoder_count;             /* 本次实际提交的累计计数。 */
    uint32_t generation;               /* 本次提交后的V2代次。 */
    uint16_t angle;                    /* 本次提交的单圈角。 */
    uint8_t active_slot;               /* 本次成功提交后的活动槽。 */
    uint8_t receipt_committed;         /* 1表示A/B记录与独立回执均验证成功。 */
    EncoderEmergencyPersistSource source; /* 真实ADC低压或软件测试来源。 */
} EncoderEmergencyPersistenceReport;

/**
 * @brief 恢复编码器持久化记录并启动 AS5145 采集。
 *
 * @details 调用场景：CPU2 上电初始化。
 * @note 关键约束：A/B 都无效时返回 ENCODER_POWERON_FAIL，不把 0 写成可信位置。
 */
uint32_t Initialize_Encoder(void);

/**
 * @brief 仅在位置恢复有效且 AS5145 已取得有效样本时报告编码器就绪。
 *
 * IsReady同时要求累计位置可信和AS5145已有有效帧；WaitReady用于启动期有限等待首帧。
 * 二者都不会把A/B无效时的零值自动提升为可信位置。
 *
 * @note 关键约束：在中断或回调上下文中只更新必要状态，避免阻塞和高耗时操作。
 *
 * @return true 表示编码器位置恢复有效且 AS5145 已取得至少一帧有效样本；false 表示持久位置尚未建立，或角度传感器仍无有效样本。
 */
bool Encoder_IsReady(void);
/**
 * @brief 位置恢复有效时等待 AS5145 首个有效样本；位置无效立即返回 ENCODER_POWERON_FAIL。
 *
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @return NO_ERROR 表示已取得首个有效 AS5145 样本；位置恢复无效返回 ENCODER_POWERON_FAIL，等待失败透传 AS5145 错误。
 */
uint32_t Encoder_WaitReady(uint32_t timeout_ms);

/**
 * @brief 在线程态有限等待角度重新同步，优先返回等待期间锁存的具体编码器故障。
 *
 * @param timeout_ms 允许等待的最长时间，单位 ms。
 * @return NO_ERROR 表示完成同步；具体编码器故障优先原样返回，否则返回 ENCODER_FIRST_SAMPLE_TIMEOUT。
 */
uint32_t Encoder_WaitAngleSynchronized(uint32_t timeout_ms);

/**
 * @brief 判断编码器传感器是否可用于重新回零。
 *
 * @details 调用场景：A/B 持久化记录都损坏后的 CMD_BACK_ZERO 运动门控。
 * @note 关键约束：只代表 AS5145 已完成单圈角稳定同步，不代表累计位置已经可信。
 */
bool Encoder_CanStartHoming(void);
/**
 * @brief 判断当前累计位置是否来自有效的持久化恢复。
 *
 * @return true 表示编码器累计位置有效标志已置位，当前位置来自可信的持久化恢复；false 表示尚未恢复出可信累计位置，调用方不得把当前位置作为已建立基准。
 */
bool Encoder_HasTrustedPosition(void);

/**
 * @brief 查询本次上电是否确认上一次真实掉电位置保存失败。
 *
 * @details 调用场景：App_Init在编码器恢复和回执消费完成后调用。
 * @note 关键约束：仅BOR/POR复位下的真实低压回执不一致或不完整回执置位。
 */
bool Encoder_DidBootDetectPowerLossSaveFailure(void);

/**
 * @brief 根据 AS5145 当前角度更新 RAM 累计值。
 *
 * @details 调用场景：PendSV 延后处理有效编码器帧。
 * @note 关键约束：不访问 FRAM、不打印、不阻塞。
 */
uint32_t Encoder_ProcessAngleSample(uint16_t current_angle,
                                    uint32_t sample_tick,
                                    bool *sample_accepted);

/**
 * @brief 判断错误码是否属于编码器采集、位置恢复或运行跳变故障。
 *
 * @note 周长标定确定性错误不在本集合内，不能在电机记步模式下被后台清除。
 */
bool Encoder_IsRuntimeFaultCode(uint32_t error_code);

/**
 * @brief 尝试提交一份待保存编码器快照。
 *
 * @details 调用场景：PendSV 每轮编码器事件处理结束。
 * @note 关键约束：每次最多提交一份快照，失败后保留脏标志等待下次采样重试。
 */
void Encoder_ProcessDeferredPersistence(void);

/**
 * @brief 请求掉电紧急保存并查询请求是否仍待提交。
 *
 * @details 调用场景：ADC看门狗和SysTick中断。
 * @note 关键约束：接口只访问RAM标志，不访问FRAM、不打印、不阻塞。
 */
void Encoder_RequestEmergencyPersistenceFromISR(EncoderEmergencyPersistSource source);
/**
 * @brief 判断是否仍有待处理的编码器紧急保存请求。
 *
 * @return true 表示仍有待处理的编码器紧急保存请求；false 表示已不再有待处理的编码器紧急保存请求。
 */
bool Encoder_HasEmergencyPersistencePending(void);

/**
 * @brief 取得编码器运行态、已保存值和FRAM槽位的一致调试快照。
 *
 * @details 调用场景：线程态处理ENC?查询。
 * @note 关键约束：只短暂关中断复制RAM状态，不访问FRAM、不打印。
 */
bool Encoder_GetDebugSnapshot(EncoderDebugSnapshot *snapshot);

/**
 * @brief 消费一次紧急持久化成功快照。
 *
 * @details 调用场景：主循环延后打印POWER_SAVE结果。
 * @note 关键约束：PendSV只置快照；打印方消费后清除就绪标志。
 */
bool Encoder_TakeEmergencyPersistenceReport(EncoderEmergencyPersistenceReport *report);

/**
 * @brief 同步保存当前编码器快照。
 *
 * @details 调用场景：电机停稳、回零、人工修正和受控断电前。
 * @note 关键约束：只在线程态调用，成功后才推进已保存计数。
 */
uint32_t Encoder_SaveCurrentPosition(void);

/**
 * @brief 开始新的顶层正式业务过程。
 *
 * @details 调用场景：ProcessMeasureCmd 通过即时控制命令过滤后调用。
 * @note 关键约束：只解除编码器故障锁存；业务内部粗找、精找重试不得调用。
 */
void Encoder_BeginNewProcess(void);

/**
 * @brief 判断 AS5145 是否仍有锁存故障。
 *
 * 函数用途：查询编码器异步故障是否仍处于锁存状态。
 * 调用场景：故障初始化和错误清除权限判断。
 *
 * @return true 表示 AS5145 仍有锁存故障；false 表示 AS5145 已不再有锁存故障。
 */
bool Encoder_HasLatchedFault(void);

/**
 * @brief 位置源为编码器时，按累计计数刷新尺带长度和传感器高度。
 */
void update_sensor_height_from_encoder(void);
/**
 * @brief 忽略当前位置源选择，强制按编码器累计计数刷新尺带长度和传感器高度。
 */
void update_sensor_height_from_encoder_force(void);
/**
 * @brief 把当前编码器位置置零并同步提交可信位置记录。
 *
 * @details 调用场景：正式回零、零点标定和人工调试置零。
 * @note 关键约束：只有FRAM写入和回读验证成功才返回NO_ERROR。
 */
uint32_t set_encoder_zero(void);
/**
 * @brief 根据目标尺带长度反算编码器计数并持久化，单位 0.1 mm。
 *
 * @param cable_length_01mm 待写入编码器位置模型的尺带长度，单位 0.1 mm。
 */
void encoder_set_cable_length_01mm(int32_t cable_length_01mm);
/**
 * @brief 将编码器累计计数换算为尺带长度，单位 0.1 mm。
 *
 * @return 返回尺带长度，单位 0.1 mm的有效长度，单位字节；0 表示没有可供消费的数据。
 */
int32_t encoder_get_cable_length_01mm(void);
/**
 * @brief 根据罐高和尺带长度计算传感器位置，单位 0.1 mm。
 *
 * @return 返回 tankHeight 减去当前尺带长度得到的有符号传感器位置，单位 0.1 mm；函数不对负值或罐高上限执行钳位。
 */
int32_t encoder_get_sensor_position_01mm(void);

#endif

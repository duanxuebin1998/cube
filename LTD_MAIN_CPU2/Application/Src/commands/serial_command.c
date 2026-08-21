/*
 * serial_command.c
 *
 * 文件职责：在线程态分发查询、正式业务、维护测试和停止类串口命令。
 */

#include "serial_command.h"

#include "app_version.h"
#include "AS5145.h"
#include "encoder.h"
#include "error_log.h"
#include "power_monitor.h"
#include "serial_command_parser.h"
#include "system_parameter.h"
#include "service_debug.h"

#include <stdio.h>
#include <string.h>

/* 调试串口逐字节命令接收状态。 */
static SerialCommandRxState s_serial_command_rx;
/* 超长命令事件已经就绪、等待前台输出错误提示的标志。 */
static volatile uint8_t s_serial_command_too_long_ready = 0U;
/* 完整调试命令已经组帧、等待前台解析执行的标志。 */
volatile uint8_t new_command_ready = 0U;

/**
 * @brief 将已严格校验的正式单字母命令映射为统一业务命令枚举。
 *
 * @param command 已经通过正式命令语法校验的单字节 ASCII 命令字母。
 * @return I、G、K、R、W、O、P、Q 分别映射回零、探底、找油、分布测量、找水、空载标定、满载标定和恢复出厂命令；其他字符返回 CMD_UNKNOWN。
 */
static CommandType SerialCommand_MapFormalCommand(uint8_t command)
{
    switch (command) {
    case 'I': return CMD_BACK_ZERO;
    case 'G': return CMD_FIND_BOTTOM;
    case 'K': return CMD_FIND_OIL;
    case 'R': return CMD_MEASURE_DISTRIBUTED;
    case 'W': return CMD_FIND_WATER;
    case 'O': return CMD_SET_EMPTY_WEIGHT;
    case 'P': return CMD_SET_FULL_WEIGHT;
    case 'Q': return CMD_RESTORE_FACTORY;
    default:  return CMD_UNKNOWN;
    }
}

/**
 * @brief 打印现场可用命令族摘要，详细参数由协议卷维护。
 */
static void SerialCommand_PrintHelp(void)
{
    printf("HELP 正式命令：I/G/K/R/W/O/P/Q，Q=恢复出厂参数\r\n");
    printf("HELP 安全与查询：STOP/HELP/VER?/STAT?/ERR?/PWR?/ENC?\r\n");
    printf("HELP 掉电保存测试：PWRTEST（不伪造ADC低压）\r\n");
    printf("HELP 运动：A0/A+mm/A-mm/B/BE/BJ/BJP\r\n");
    printf("HELP 通信与诊断：SC/SPC/SPS/SPR/SPN=<name>/V3?/V3P/V3D/V3L/V3A/V3R=xx/V3W=20..114,value;V4?/V4W=66..159,raw32/YS/YC/YT\r\n");
    printf("HELP 测试：C/D/E/F/H/J/L/M/N/T0-TU/AO/X\r\n");
    printf("HELP 找液位：LF/LF=<频率Hz>[,<死区Hz>]/LF?，STOP=停止\r\n");
    printf("HELP 详细参数见《CPU2串口调试命令协议卷》\r\n");
}

/**
 * @brief 打印 CPU2 固件、共享协议和当前参数存储版本。
 */
static void SerialCommand_PrintVersion(void)
{
    printf("VERSION cpu2=%s protocol=%lu param=%lu\r\n",
           CPU2_APP_VERSION_STRING,
           (unsigned long)DEVICE_PROTOCOL_VERSION,
           (unsigned long)g_deviceParams.param_version);
}

/**
 * @brief 打印设备状态、当前命令和待执行命令的只读快照。
 */
static void SerialCommand_PrintStatus(void)
{
    printf("STATUS state=0x%04lX current=%lu pending=%lu zero=%lu work=%lu\r\n",
           (unsigned long)g_measurement.device_status.device_state,
           (unsigned long)g_measurement.device_status.current_command,
           (unsigned long)g_deviceParams.command,
           (unsigned long)g_measurement.device_status.zero_point_status,
           (unsigned long)g_measurement.device_status.work_mode);
}

/**
 * @brief 打印当前错误码及统一错误名称、模块和原因。
 */
static void SerialCommand_PrintError(void)
{
    uint32_t error_code = g_measurement.device_status.error_code;

    printf("ERROR code=0x%08lX name=%s module=%s reason=%s\r\n",
           (unsigned long)error_code,
           ErrorLog_GetCodeName(error_code),
           ErrorLog_GetModuleByCode(error_code),
           ErrorLog_GetReasonByCode(error_code));
}

/**
 * @brief 把编码器最近持久化结果转换为稳定的串口文本。
 *
 * @details 调用场景：ENC?状态输出。
 * @note 关键约束：只返回静态字符串，不访问硬件、不修改状态。
 *
 * @param result 编码器最近一次持久化结果枚举，用于映射为稳定串口文字。
 * @return 返回稳定的串口文本对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *SerialCommand_GetPersistResultName(EncoderPersistResult result)
{
    switch (result) {
    case ENCODER_PERSIST_RESULT_OK:
        return "成功";
    case ENCODER_PERSIST_RESULT_FAILED:
        return "失败";
    case ENCODER_PERSIST_RESULT_NONE:
    default:
        return "未发生";
    }
}

/**
 * @brief 把本次上电判定的上一次掉电存储结果转换为稳定串口文本。
 *
 * @details 调用场景：ENC?状态输出。
 * @note 关键约束：结果在初始化时固定，不被本次启动后的普通保存覆盖。
 *
 * @param result 本次启动对上次掉电保存的判定结果枚举，用于映射为稳定串口文字。
 * @return 返回稳定串口文本对应的只读文本首地址；内容由当前输入或语言配置选择，调用方不得修改或释放。
 */
static const char *SerialCommand_GetPowerLossResultName(EncoderPowerLossPersistResult result)
{
    switch (result) {
    case ENCODER_POWER_LOSS_RESULT_SUCCESS:
        return "成功";
    case ENCODER_POWER_LOSS_RESULT_INCOMPLETE:
        return "失败或未完成";
    case ENCODER_POWER_LOSS_RESULT_NO_RECORD:
    default:
        return "无可判定记录";
    }
}

/**
 * @brief 输出24V监测和紧急保存请求状态。
 *
 * @details 调用场景：线程态处理PWR?命令。
 * @note 关键约束：只读取一致RAM快照，不直接操作ADC看门狗或FRAM。
 */
static void SerialCommand_PrintPowerStatus(void)
{
    PowerMonitorDebugSnapshot snapshot;

    if (!PowerMonitor_GetDebugSnapshot(&snapshot)) {
        printf("PWR 电源监测：状态=不可用\r\n");
        return;
    }
    printf("PWR 电源监测：ADC=%lu，24V电压=%lu毫伏，监控状态=%u，首次故障=0x%08lX，当前故障=0x%08lX，恢复原因=0x%08lX，恢复尝试=%u/%u，低压活动=%u，电机锁存=%u，恢复计时=%u毫秒，本次上电触发=%lu，最近触发ADC=%lu，紧急布防=%u，紧急保存中=%u，DMA运行=%u，ADC过载=%u\r\n",
           (unsigned long)snapshot.adc_sample,
           (unsigned long)snapshot.millivolts_24v,
           (unsigned int)snapshot.monitor_state,
           (unsigned long)snapshot.first_fault_code,
           (unsigned long)snapshot.latched_fault_code,
           (unsigned long)snapshot.recovery_cause_code,
           (unsigned int)snapshot.recovery_attempts,
           (unsigned int)POWER_MONITOR_RECOVERY_RETRY_LIMIT,
           (unsigned int)snapshot.power_fail_active,
           (unsigned int)snapshot.motor_inhibited,
           (unsigned int)snapshot.recover_stable_ms,
           (unsigned long)snapshot.trip_count,
           (unsigned long)snapshot.last_trip_adc_sample,
           (unsigned int)snapshot.emergency_persistence_armed,
           (unsigned int)snapshot.emergency_persistence_pending,
           (unsigned int)snapshot.dma_running,
           (unsigned int)snapshot.adc_overrun);
}

/**
 * @brief 输出编码器当前值、已保存值和FRAM提交状态。
 *
 * @details 调用场景：线程态处理ENC?命令。
 * @note 关键约束：只读取一致RAM快照，不在查询路径访问FRAM。
 */
static void SerialCommand_PrintEncoderStatus(void)
{
    EncoderDebugSnapshot snapshot;
    AS5145DiagnosticSnapshot ssi_snapshot;
    uint64_t link_total;
    uint64_t process_total;
    uint32_t link_quality_x1000;
    uint32_t parity_quality_x1000;
    uint32_t process_quality_x1000;
    uint32_t link_bad_frames;
    uint32_t parity_bad_frames;

    if (!Encoder_GetDebugSnapshot(&snapshot) ||
        !AS5145_GetDiagnosticSnapshot(&ssi_snapshot)) {
        printf("ENC 编码器：状态=不可用\r\n");
        return;
    }

    link_bad_frames = (ssi_snapshot.disconnected_pattern_count > ssi_snapshot.frame_count) ?
                      ssi_snapshot.frame_count : ssi_snapshot.disconnected_pattern_count;
    parity_bad_frames = (ssi_snapshot.parity_count > ssi_snapshot.frame_count) ?
                        ssi_snapshot.frame_count : ssi_snapshot.parity_count;
    link_total = (uint64_t)ssi_snapshot.frame_count +
                 (uint64_t)ssi_snapshot.transfer_error_count;
    process_total = link_total;
    link_quality_x1000 = (link_total == 0ULL) ? 0U :
        (uint32_t)((((uint64_t)ssi_snapshot.frame_count - link_bad_frames) * 100000ULL) /
                   link_total);
    parity_quality_x1000 = (ssi_snapshot.frame_count == 0U) ? 0U :
        (uint32_t)((((uint64_t)ssi_snapshot.frame_count - parity_bad_frames) * 100000ULL) /
                   (uint64_t)ssi_snapshot.frame_count);
    process_quality_x1000 = (process_total == 0ULL) ? 0U :
        (uint32_t)((((process_total > ssi_snapshot.queue_overrun_count) ?
                     (process_total - ssi_snapshot.queue_overrun_count) : 0ULL) * 100000ULL) /
                   process_total);

    printf("ENC 编码器：当前累计=%ld，已保存=%ld，未保存差值=%ld，当前单圈=%u，已保存单圈=%u，单圈差值=%ld，位置有效=%u，普通保存中=%u，紧急保存中=%u，存储代次=%lu，活动槽=%c，最近提交=%s，上次掉电保存=%s，故障锁存=%u，锁存码=0x%08lX，最近错误=0x%08lX，同步=%u，重同步=%u，启动稳定=%u，候选增量=%d，上限=%u，间隔=%lums，拒绝原因=%u，拒绝总数=%lu，方向不一致=%lu\r\n",
           (long)snapshot.encoder_count,
           (long)snapshot.saved_count,
           (long)snapshot.unsaved_count,
           (unsigned int)snapshot.current_angle,
           (unsigned int)snapshot.saved_angle,
           (long)snapshot.raw_delta,
           (unsigned int)snapshot.position_valid,
           (unsigned int)snapshot.persistence_pending,
           (unsigned int)snapshot.emergency_persistence_pending,
           (unsigned long)snapshot.generation,
           (char)snapshot.active_slot,
           SerialCommand_GetPersistResultName(snapshot.last_commit_result),
           SerialCommand_GetPowerLossResultName(snapshot.boot_power_loss_result),
           (unsigned int)snapshot.fault_latched,
           (unsigned long)ssi_snapshot.latched_error_code,
           (unsigned long)ssi_snapshot.last_error_code,
           (unsigned int)snapshot.angle_synchronized,
           (unsigned int)snapshot.runtime_resynchronizing,
           (unsigned int)snapshot.boot_stable_count,
           (int)snapshot.last_candidate_delta,
           (unsigned int)snapshot.last_candidate_limit,
           (unsigned long)snapshot.last_candidate_dt_ms,
           (unsigned int)snapshot.last_reject_reason,
           (unsigned long)snapshot.rejected_sample_count,
           (unsigned long)snapshot.direction_mismatch_count);
    printf("ENC SSI：原始=%02X %02X %02X %02X，解析有效=%u，角度=%u，OCF=%u，COF=%u，LIN=%u，MagINCn=%u，MagDECn=%u，校验=%u，连续错误=0x%08lX，连续数=%u，恢复正常数=%u，帧=%lu，异常证据=%lu，溢出=%lu，超时=%lu，校验错=%lu，OCF=%lu，COF=%lu，LIN=%lu，跳变=%lu，其他=%lu，最后正常=%lu，最后异常=%lu\r\n",
           (unsigned int)ssi_snapshot.raw[0],
           (unsigned int)ssi_snapshot.raw[1],
           (unsigned int)ssi_snapshot.raw[2],
           (unsigned int)ssi_snapshot.raw[3],
           (unsigned int)ssi_snapshot.parsed_valid,
           (unsigned int)ssi_snapshot.parsed.angle,
           (unsigned int)ssi_snapshot.parsed.OCF,
           (unsigned int)ssi_snapshot.parsed.COF,
           (unsigned int)ssi_snapshot.parsed.LIN,
           (unsigned int)ssi_snapshot.parsed.MagINCn,
           (unsigned int)ssi_snapshot.parsed.MagDECn,
           (unsigned int)ssi_snapshot.parsed.parity_ok,
           (unsigned long)ssi_snapshot.consecutive_error_code,
           (unsigned int)ssi_snapshot.consecutive_bad_count,
           (unsigned int)ssi_snapshot.recovery_good_count,
           (unsigned long)ssi_snapshot.frame_count,
           (unsigned long)ssi_snapshot.error_count,
           (unsigned long)ssi_snapshot.queue_overrun_count,
           (unsigned long)ssi_snapshot.timeout_count,
           (unsigned long)ssi_snapshot.parity_count,
           (unsigned long)ssi_snapshot.ocf_count,
           (unsigned long)ssi_snapshot.cof_count,
           (unsigned long)ssi_snapshot.lin_count,
           (unsigned long)ssi_snapshot.position_jump_count,
           (unsigned long)ssi_snapshot.other_error_count,
           (unsigned long)ssi_snapshot.last_ok_tick,
           (unsigned long)ssi_snapshot.last_bad_tick);
    printf("ENC 质量：链路=%lu.%03lu%%，校验=%lu.%03lu%%，处理=%lu.%03lu%%，传输失败=%lu，断线帧=%lu，处理溢出=%lu，统计范围=本次上电\r\n",
           (unsigned long)(link_quality_x1000 / 1000U),
           (unsigned long)(link_quality_x1000 % 1000U),
           (unsigned long)(parity_quality_x1000 / 1000U),
           (unsigned long)(parity_quality_x1000 % 1000U),
           (unsigned long)(process_quality_x1000 / 1000U),
           (unsigned long)(process_quality_x1000 % 1000U),
           (unsigned long)ssi_snapshot.transfer_error_count,
           (unsigned long)ssi_snapshot.disconnected_pattern_count,
           (unsigned long)ssi_snapshot.queue_overrun_count);
}

/**
 * @brief 请求一次不改变ADC状态的软件紧急保存测试。
 *
 * @details 调用场景：线程态处理PWRTEST命令。
 * @note 关键约束：命令只投递请求，最终提交结果由POWER_TEST_SAVE和ENC?确认。
 */
static void SerialCommand_RunPowerTest(void)
{
    PowerMonitorTestRequestResult result =
        PowerMonitor_RequestEmergencyPersistenceForTest();

    if (result == POWER_MONITOR_TEST_QUEUED) {
        printf("PWRTEST 掉电保存测试：已排队=1\r\n");
    } else if (result == POWER_MONITOR_TEST_BUSY) {
        printf("PWRTEST 掉电保存测试：已排队=0，原因=已有保存任务\r\n");
    } else {
        printf("PWRTEST 掉电保存测试：已排队=0，原因=编码器位置无效\r\n");
    }
}

/**
 * @brief 在 UART 接收中断中把单字节写入串口命令环形缓冲区。
 *
 * @param byte UART 接收中断取得的单个原始字节，随后交给定长收帧状态机处理。
 * @note USART1 中断入口只记录字节和收帧事件，不在中断内打印或执行命令。
 */
void SerialCommand_RxByteFromIsr(uint8_t byte)
{
    SerialCommandRxEvent event;

    if (new_command_ready != 0U) {
        return;
    }

    event = SerialCommandRx_PushByte(&s_serial_command_rx, byte);
    if (event == SERIAL_COMMAND_RX_READY) {
        new_command_ready = 1U;
    } else if (event == SERIAL_COMMAND_RX_TOO_LONG) {
        s_serial_command_too_long_ready = 1U;
        new_command_ready = 1U;
    } else {
        /* 中断内只累计接收状态，未形成完整帧时不做额外处理。 */
    }
}

/**
 * @brief 主循环先复制完整帧再清就绪标志，避免下一帧覆盖正在处理的文本。
 */
void SerialCommand_ProcessReady(void)
{
    uint8_t command[SERIAL_COMMAND_RX_CAPACITY];

    if (s_serial_command_too_long_ready != 0U) {
        s_serial_command_too_long_ready = 0U;
        new_command_ready = 0U;
        SerialCommand_ReportTooLong();
        return;
    }

    memcpy(command, s_serial_command_rx.buffer, sizeof(command));
    command[sizeof(command) - 1U] = '\0';
    new_command_ready = 0U;
    SerialCommand_Process(command);
}

/**
 * @brief 严格解析完整命令，并按查询、正式业务、测试和停止四类分发。
 *
 * @param command 以 NUL 结尾的完整串口命令字节串；函数先严格分类，再分发查询、正式业务、测试或停止命令。
 */
void SerialCommand_Process(const uint8_t *command)
{
    SerialCommandParseResult parsed = SerialCommandParser_Parse(command);
    CommandType formal_command;

    if (parsed.kind == SERIAL_COMMAND_KIND_INVALID) {
        printf("ERR cmd=%s reason=INVALID_FORMAT\r\n",
               (command != NULL) ? (const char *)command : "<null>");
        return;
    }
    if (parsed.kind == SERIAL_COMMAND_KIND_UNSUPPORTED) {
        printf("ERR cmd=%s reason=UNSUPPORTED\r\n", (const char *)command);
        return;
    }

    printf("ACK cmd=%s\r\n", (const char *)command);
    switch (parsed.kind) {
    case SERIAL_COMMAND_KIND_FORMAL:
        formal_command = SerialCommand_MapFormalCommand(parsed.formal_command);
        if (formal_command == CMD_UNKNOWN) {
            printf("ERR cmd=%s reason=UNSUPPORTED\r\n", (const char *)command);
            return;
        }
        DeviceCommand_Queue(formal_command);
        return;
    case SERIAL_COMMAND_KIND_TEST:
        (void)ServiceDebug_ProcessSerialCommand((uint8_t *)command);
        return;
    case SERIAL_COMMAND_KIND_STOP:
        DeviceCommand_Queue(CMD_CANCEL_MEASUREMENT);
        return;
    case SERIAL_COMMAND_KIND_HELP:
        SerialCommand_PrintHelp();
        return;
    case SERIAL_COMMAND_KIND_VERSION:
        SerialCommand_PrintVersion();
        return;
    case SERIAL_COMMAND_KIND_STATUS:
        SerialCommand_PrintStatus();
        return;
    case SERIAL_COMMAND_KIND_ERROR:
        SerialCommand_PrintError();
        return;
    case SERIAL_COMMAND_KIND_POWER_STATUS:
        SerialCommand_PrintPowerStatus();
        return;
    case SERIAL_COMMAND_KIND_ENCODER_STATUS:
        SerialCommand_PrintEncoderStatus();
        return;
    case SERIAL_COMMAND_KIND_POWER_TEST:
        SerialCommand_RunPowerTest();
        return;
    default:
        printf("ERR cmd=%s reason=UNSUPPORTED\r\n", (const char *)command);
        return;
    }
}

/**
 * @brief 输出PendSV已完成的紧急持久化成功快照。
 *
 * @details 调用场景：App_MainLoop每轮后台服务入口。
 * @note 关键约束：仅线程态打印；完全掉电时字符可能来不及发送。
 */
void SerialCommand_ProcessDeferredReports(void)
{
    EncoderEmergencyPersistenceReport report;

    if (Encoder_TakeEmergencyPersistenceReport(&report)) {
        const char *report_name =
            (report.source == ENCODER_EMERGENCY_SOURCE_ADC) ?
                "POWER_SAVE 真实低压掉电保存" :
                "POWER_TEST_SAVE 软件测试保存";
        const char *result_name =
            (report.receipt_committed != 0U) ? "成功" : "失败";

        if ((report.source == ENCODER_EMERGENCY_SOURCE_ADC) &&
            (report.receipt_committed == 0U)) {
            PowerMonitor_ReportEmergencyPersistenceFailure();
        }
        printf("%s：结果=%s，编码器A/B与回执=%s，累计编码=%ld，单圈角度=%u，存储代次=%lu，活动槽=%c\r\n",
               report_name,
               result_name,
               (report.receipt_committed != 0U) ? "已确认" : "未确认",
               (long)report.encoder_count,
               (unsigned int)report.angle,
               (unsigned long)report.generation,
               (char)report.active_slot);
    }
}

/**
 * @brief 在主循环统一报告超长帧，保持 USART1 中断无打印。
 */
void SerialCommand_ReportTooLong(void)
{
    printf("ERR reason=CMD_TOO_LONG max=%u\r\n", (unsigned int)(SERIAL_COMMAND_RX_CAPACITY - 1U));
}

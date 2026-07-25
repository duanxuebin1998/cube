#include "serial_command.h"

#include "app_version.h"
#include "error_log.h"
#include "serial_command_parser.h"
#include "system_parameter.h"
#include "test.h"

#include <stdio.h>
#include <string.h>

static SerialCommandRxState s_serial_command_rx;
static volatile uint8_t s_serial_command_too_long_ready = 0U;
volatile uint8_t new_command_ready = 0U;

/* 将已严格校验的正式单字母命令映射为统一业务命令枚举。 */
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

/* 打印现场可用命令族摘要，详细参数由协议卷维护。 */
static void SerialCommand_PrintHelp(void)
{
    printf("HELP 正式命令：I/G/K/R/W/O/P/Q，Q=恢复出厂参数\r\n");
    printf("HELP 安全与查询：STOP/HELP/VER?/STAT?/ERR?\r\n");
    printf("HELP 运动：A0/A+mm/A-mm/B/BE/BJ/BJP\r\n");
    printf("HELP 通信与诊断：SC/SPC/SPS/SPR/SPN=<name>/YS/YC/YT\r\n");
    printf("HELP 测试：C/D/E/F/H/J/L/M/N/T0-TU/AO/X\r\n");
    printf("HELP 找液位：LF/LF=<频率Hz>[,<死区Hz>]/LF?，STOP=停止\r\n");
    printf("HELP 详细参数见《CPU2串口调试命令协议卷》\r\n");
}

/* 打印 CPU2 固件、共享协议和当前参数存储版本。 */
static void SerialCommand_PrintVersion(void)
{
    printf("VERSION cpu2=%s protocol=%lu param=%lu\r\n",
           CPU2_APP_VERSION_STRING,
           (unsigned long)DEVICE_PROTOCOL_VERSION,
           (unsigned long)g_deviceParams.param_version);
}

/* 打印设备状态、当前命令和待执行命令的只读快照。 */
static void SerialCommand_PrintStatus(void)
{
    printf("STATUS state=0x%04lX current=%lu pending=%lu zero=%lu work=%lu\r\n",
           (unsigned long)g_measurement.device_status.device_state,
           (unsigned long)g_measurement.device_status.current_command,
           (unsigned long)g_deviceParams.command,
           (unsigned long)g_measurement.device_status.zero_point_status,
           (unsigned long)g_measurement.device_status.work_mode);
}

/* 打印当前错误码及统一错误名称、模块和原因。 */
static void SerialCommand_PrintError(void)
{
    uint32_t error_code = g_measurement.device_status.error_code;

    printf("ERROR code=0x%08lX name=%s module=%s reason=%s\r\n",
           (unsigned long)error_code,
           ErrorLog_GetCodeName(error_code),
           ErrorLog_GetModuleByCode(error_code),
           ErrorLog_GetReasonByCode(error_code));
}

/* USART1 中断入口只记录字节和事件，不在中断内打印或执行命令。 */
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

/* 主循环先复制完整帧再清就绪标志，避免下一帧覆盖正在处理的文本。 */
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

/* 严格解析完整命令，并按查询、正式业务、测试和停止四类分发。 */
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
        (void)Test_ProcessSerialCommand((uint8_t *)command);
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
    default:
        printf("ERR cmd=%s reason=UNSUPPORTED\r\n", (const char *)command);
        return;
    }
}

/* 在主循环统一报告超长帧，保持 USART1 中断无打印。 */
void SerialCommand_ReportTooLong(void)
{
    printf("ERR reason=CMD_TOO_LONG max=%u\r\n", (unsigned int)(SERIAL_COMMAND_RX_CAPACITY - 1U));
}

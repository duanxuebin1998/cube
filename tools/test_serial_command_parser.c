#include "serial_command_parser.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

/* 校验单条命令的分类结果，并在失败时打印可定位信息。 */
static void expect_kind(const char *command, SerialCommandKind expected)
{
    SerialCommandParseResult result = SerialCommandParser_Parse((const uint8_t *)command);
    if (result.kind != expected) {
        fprintf(stderr, "command=%s expected=%d actual=%d\n", command, (int)expected, (int)result.kind);
    }
    assert(result.kind == expected);
}

/* 覆盖合法旧命令、新增查询命令和严格拒绝场景。 */
static void test_parser(void)
{
    static const char *const valid_formal[] = {"I", "G", "K", "R", "W", "O", "P", "Q"};
    static const char *const valid_test[] = {
        "SC", "SPC", "SPS", "SPR", "SPN=DEVICE_NAME",
        "A0", "A+10", "A-1000", "B200", "B200S", "B200,1",
        "BE200", "BE200S", "BE200,1", "BE200,2", "BE200,2,5", "BE200,2,5,S",
        "BJ+100,1.5", "BJ-100,1.5", "BJP1420,1.5",
        "C", "D", "E", "F", "H", "J", "L", "M", "N",
        "T0", "T1", "T2", "TA", "TS", "TR", "TV", "TP", "TU",
        "YM", "YE", "YS", "YC", "YT", "AO400", "AO2200", "AOS", "X"
    };
    size_t i;

    for (i = 0U; i < (sizeof(valid_formal) / sizeof(valid_formal[0])); i++) {
        expect_kind(valid_formal[i], SERIAL_COMMAND_KIND_FORMAL);
    }
    for (i = 0U; i < (sizeof(valid_test) / sizeof(valid_test[0])); i++) {
        expect_kind(valid_test[i], SERIAL_COMMAND_KIND_TEST);
    }

    expect_kind("STOP", SERIAL_COMMAND_KIND_STOP);
    expect_kind("HELP", SERIAL_COMMAND_KIND_HELP);
    expect_kind("VER?", SERIAL_COMMAND_KIND_VERSION);
    expect_kind("STAT?", SERIAL_COMMAND_KIND_STATUS);
    expect_kind("ERR?", SERIAL_COMMAND_KIND_ERROR);

    expect_kind("Qxxxx", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("Ixxxx", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("T1abc", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("YMabc", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("Cabc", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("A+10abc", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("BE200,2,5,garbage", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("BJ+100,1.5abc", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("AO400abc", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("A+0", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("BE0", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("BE200,6.1", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("BE200,2,21", SERIAL_COMMAND_KIND_INVALID);
    expect_kind("UNKNOWN", SERIAL_COMMAND_KIND_UNSUPPORTED);
}

/* 覆盖 CRLF/LF 收帧、超长整帧丢弃和丢弃后的恢复。 */
static void test_receiver(void)
{
    SerialCommandRxState state;
    SerialCommandRxEvent event = SERIAL_COMMAND_RX_NONE;
    const char *command = "STAT?\r\n";
    size_t i;

    SerialCommandRx_Init(&state);
    for (i = 0U; command[i] != '\0'; i++) {
        event = SerialCommandRx_PushByte(&state, (uint8_t)command[i]);
    }
    assert(event == SERIAL_COMMAND_RX_READY);
    assert(strcmp((const char *)state.buffer, "STAT?") == 0);

    SerialCommandRx_Init(&state);
    for (i = 0U; i < SERIAL_COMMAND_RX_CAPACITY; i++) {
        event = SerialCommandRx_PushByte(&state, (uint8_t)'A');
        assert(event == SERIAL_COMMAND_RX_NONE);
    }
    event = SerialCommandRx_PushByte(&state, (uint8_t)'\n');
    assert(event == SERIAL_COMMAND_RX_TOO_LONG);

    command = "HELP\n";
    for (i = 0U; command[i] != '\0'; i++) {
        event = SerialCommandRx_PushByte(&state, (uint8_t)command[i]);
    }
    assert(event == SERIAL_COMMAND_RX_READY);
    assert(strcmp((const char *)state.buffer, "HELP") == 0);
}

/* 运行固定回归用例，并可额外校验命令面板传入的全部命令。 */
int main(int argc, char **argv)
{
    int i;

    test_parser();
    test_receiver();
    for (i = 1; i < argc; i++) {
        SerialCommandParseResult result = SerialCommandParser_Parse((const uint8_t *)argv[i]);
        if ((result.kind == SERIAL_COMMAND_KIND_INVALID) ||
            (result.kind == SERIAL_COMMAND_KIND_UNSUPPORTED)) {
            fprintf(stderr, "panel command rejected: %s kind=%d\n", argv[i], (int)result.kind);
            return 1;
        }
    }
    puts("serial command parser tests passed");
    return 0;
}

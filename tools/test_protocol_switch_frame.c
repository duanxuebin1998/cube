#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "protocol_switch_frame.h"

static int g_failures = 0;

#define CHECK_TRUE(condition)                                                     \
    do {                                                                          \
        if (!(condition)) {                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);          \
            g_failures++;                                                         \
        }                                                                         \
    } while (0)

/*
 * 函数用途：为主机单元测试提供与固件一致的 Modbus RTU CRC16。
 * 调用场景：链接 protocol_switch_frame.c 时替代依赖 HAL 的固件 CRC 模块。
 * 关键约束：返回值低字节先写入帧、高字节后写入帧。
 */
uint16_t CRC16_Calculate(const uint8_t *data, uint32_t length)
{
    uint16_t crc = 0xFFFFU;
    uint32_t i;

    for (i = 0U; i < length; i++) {
        uint8_t bit;
        crc ^= data[i];
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

/*
 * 函数用途：验证一个合法目标协议切换帧会被接受并返回独立 ACK。
 * 调用场景：覆盖 DSM、Wartsila、LTD 和 SI 四个远程切换目标。
 * 关键约束：测试帧使用站号 1 和固定 golden CRC。
 */
static void test_valid_target(const uint8_t frame[PROTOCOL_SWITCH_REQUEST_LENGTH],
                              const uint8_t expected_response[PROTOCOL_SWITCH_ACK_LENGTH],
                              ComProtocolType expected_target)
{
    uint8_t response[PROTOCOL_SWITCH_REQUEST_LENGTH] = {0U};
    uint16_t response_len = 0U;
    ComProtocolType target = COM_PROTO_DSM;
    ProtocolSwitchFrameResult result;

    result = ProtocolSwitchFrame_Process(1U,
                                         frame,
                                         PROTOCOL_SWITCH_REQUEST_LENGTH,
                                         response,
                                         &response_len,
                                         &target);

    CHECK_TRUE(result == PROTOCOL_SWITCH_FRAME_ACCEPTED);
    CHECK_TRUE(response_len == PROTOCOL_SWITCH_ACK_LENGTH);
    CHECK_TRUE(memcmp(response, expected_response, PROTOCOL_SWITCH_ACK_LENGTH) == 0);
    CHECK_TRUE(target == expected_target);
}

static void test_bad_crc_is_silent(void)
{
    static const uint8_t frame[PROTOCOL_SWITCH_REQUEST_LENGTH] = {
        0x01U, 0x46U, 0x4CU, 0x54U, 0x44U, 0x02U, 0x6DU, 0x85U
    };
    uint8_t response[PROTOCOL_SWITCH_REQUEST_LENGTH] = {0U};
    uint16_t response_len = 99U;
    ComProtocolType target = COM_PROTO_DSM;

    CHECK_TRUE(ProtocolSwitchFrame_Process(1U,
                                           frame,
                                           PROTOCOL_SWITCH_REQUEST_LENGTH,
                                           response,
                                           &response_len,
                                           &target) == PROTOCOL_SWITCH_FRAME_HANDLED);
    CHECK_TRUE(response_len == 0U);
    CHECK_TRUE(target == COM_PROTO_DSM);
}

static void test_invalid_target_returns_exception(void)
{
    static const uint8_t frame[PROTOCOL_SWITCH_REQUEST_LENGTH] = {
        0x01U, 0x46U, 0x4CU, 0x54U, 0x44U, 0x03U, 0xACU, 0x44U
    };
    static const uint8_t expected_response[5] = {
        0x01U, 0xC6U, 0x03U, 0x33U, 0xA1U
    };
    uint8_t response[PROTOCOL_SWITCH_REQUEST_LENGTH] = {0U};
    uint16_t response_len = 0U;
    ComProtocolType target = COM_PROTO_DSM;

    CHECK_TRUE(ProtocolSwitchFrame_Process(1U,
                                           frame,
                                           PROTOCOL_SWITCH_REQUEST_LENGTH,
                                           response,
                                           &response_len,
                                           &target) == PROTOCOL_SWITCH_FRAME_HANDLED);
    CHECK_TRUE(response_len == sizeof(expected_response));
    CHECK_TRUE(memcmp(response, expected_response, sizeof(expected_response)) == 0);
}

static void test_non_management_frame_falls_through(void)
{
    static const uint8_t wrong_address[PROTOCOL_SWITCH_REQUEST_LENGTH] = {
        0x02U, 0x46U, 0x4CU, 0x54U, 0x44U, 0x00U, 0xECU, 0x45U
    };
    static const uint8_t wrong_magic[PROTOCOL_SWITCH_REQUEST_LENGTH] = {
        0x01U, 0x46U, 0x4CU, 0x54U, 0x58U, 0x00U, 0xECU, 0x45U
    };
    uint8_t response[PROTOCOL_SWITCH_REQUEST_LENGTH] = {0U};
    uint16_t response_len = 0U;
    ComProtocolType target = COM_PROTO_DSM;

    CHECK_TRUE(ProtocolSwitchFrame_Process(1U,
                                           wrong_address,
                                           PROTOCOL_SWITCH_REQUEST_LENGTH,
                                           response,
                                           &response_len,
                                           &target) == PROTOCOL_SWITCH_FRAME_NOT_MATCHED);
    CHECK_TRUE(ProtocolSwitchFrame_Process(1U,
                                           wrong_magic,
                                           PROTOCOL_SWITCH_REQUEST_LENGTH,
                                           response,
                                           &response_len,
                                           &target) == PROTOCOL_SWITCH_FRAME_NOT_MATCHED);
}

int main(void)
{
    static const uint8_t dsm_frame[PROTOCOL_SWITCH_REQUEST_LENGTH] = {
        0x01U, 0x46U, 0x4CU, 0x54U, 0x44U, 0x00U, 0xECU, 0x45U
    };
    static const uint8_t wartsila_frame[PROTOCOL_SWITCH_REQUEST_LENGTH] = {
        0x01U, 0x46U, 0x4CU, 0x54U, 0x44U, 0x01U, 0x2DU, 0x85U
    };
    static const uint8_t ltd_frame[PROTOCOL_SWITCH_REQUEST_LENGTH] = {
        0x01U, 0x46U, 0x4CU, 0x54U, 0x44U, 0x02U, 0x6DU, 0x84U
    };
    static const uint8_t si_frame[PROTOCOL_SWITCH_REQUEST_LENGTH] = {
        0x01U, 0x46U, 0x4CU, 0x54U, 0x44U, 0x05U, 0x2CU, 0x46U
    };
    static const uint8_t dsm_ack[PROTOCOL_SWITCH_ACK_LENGTH] = {
        0x01U, 0x46U, 0x00U, 0x00U, 0xE0U, 0x0DU
    };
    static const uint8_t wartsila_ack[PROTOCOL_SWITCH_ACK_LENGTH] = {
        0x01U, 0x46U, 0x00U, 0x01U, 0x21U, 0xCDU
    };
    static const uint8_t ltd_ack[PROTOCOL_SWITCH_ACK_LENGTH] = {
        0x01U, 0x46U, 0x00U, 0x02U, 0x61U, 0xCCU
    };
    static const uint8_t si_ack[PROTOCOL_SWITCH_ACK_LENGTH] = {
        0x01U, 0x46U, 0x00U, 0x05U, 0x20U, 0x0EU
    };

    test_valid_target(dsm_frame, dsm_ack, COM_PROTO_DSM);
    test_valid_target(wartsila_frame, wartsila_ack, COM_PROTO_WARTSILA);
    test_valid_target(ltd_frame, ltd_ack, COM_PROTO_LTD);
    test_valid_target(si_frame, si_ack, COM_PROTO_SI);
    test_bad_crc_is_silent();
    test_invalid_target_returns_exception();
    test_non_management_frame_falls_through();

    if (g_failures != 0) {
        printf("protocol switch frame tests failed: %d\n", g_failures);
        return 1;
    }

    printf("protocol switch frame tests passed\n");
    return 0;
}

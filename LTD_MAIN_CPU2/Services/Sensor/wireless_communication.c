/*
 * wireless_communication.c
 *
 * 作用：
 * 1. 读取无线主机/无线从机自身参数，例如软件版本、电压
 * 2. 与传感器透传数据链路共用 huart6，因此每次收发前都要主动清理残留数据
 *
 * 说明：
 * - 无线主机/从机与 CPU2 之间复用了 LTD 传感器的 8 字节协议格式
 * - 本文件只负责“无线设备自身应答”的通信，不负责传感器测量数据透传
 */

#include "ltd_sensor_communication.h"
#include "sensor.h"

#ifndef WIRELESS_MAX_RETRY
#define WIRELESS_MAX_RETRY   3
#endif

#ifndef WIRELESS_RX_TIMEOUT
#define WIRELESS_RX_TIMEOUT  DSM_CMD_TIMEOUT
#endif

/* 主站发给无线设备时使用的功能码。当前只用到读取功能。 */
typedef enum {
    WIRELESS_FUNC_R = 'R',
    WIRELESS_FUNC_W = 'W',
} wireless_func_t;

/*
 * 计算 8 字节协议的校验和。
 * 协议规则：前 7 字节求和，低 8 位作为第 8 字节校验值。
 */
static inline uint8_t WIRELESS_CalcSum(const uint8_t f[8]) {
    uint32_t s = 0;
    for (int i = 0; i < 7; ++i) {
        s += f[i];
    }
    return (uint8_t)(s & 0xFF);
}

/*
 * 组装一帧 8 字节无线命令。
 *
 * 帧结构：
 * [0] 地址
 * [1] 功能码
 * [2..5] 数据区
 * [6] 参数码
 * [7] 校验和
 *
 * 对读取命令来说，data_be 一般传 0 即可。
 */
static inline void WIRELESS_MakeFrame(uint8_t out[8],
                                      uint8_t addr,
                                      uint8_t func,
                                      uint32_t data_be,
                                      uint8_t param)
{
    out[0] = addr;
    out[1] = func;
    out[2] = (uint8_t)((data_be >> 24) & 0xFF);
    out[3] = (uint8_t)((data_be >> 16) & 0xFF);
    out[4] = (uint8_t)((data_be >> 8)  & 0xFF);
    out[5] = (uint8_t)( data_be        & 0xFF);
    out[6] = param;
    out[7] = WIRELESS_CalcSum(out);
}

/*
 * 发命令前清空 UART6 接收缓存中的残留数据。
 *
 * 由于 huart6 会被无线节点参数访问和传感器透传链路复用，
 * 如果上一次通信残留半包/错包，下一次固定 8 字节回复很容易错位。
 * 这里通过“持续读取直到空闲 idle_ms”为止”的方式做一个轻量 flush。
 */
static void UART6_DrainRX_UntilIdle(uint32_t idle_ms) {
    uint8_t dump;
    uint32_t last = HAL_GetTick();

    for (;;) {
        if (HAL_UART_Receive(&huart6, &dump, 1, 1) == HAL_OK) {
            last = HAL_GetTick();
        } else if ((HAL_GetTick() - last) >= idle_ms) {
            break;
        }
    }

    /* 清除 UART 溢出等异常标志，避免历史异常挂住后续接收。 */
    __HAL_UART_CLEAR_OREFLAG(&huart6);
}

/*
 * 完成一次完整的 8->8 收发。
 *
 * 该函数只负责：
 * 1. 发出完整 8 字节命令
 * 2. 收到完整 8 字节应答
 * 3. 检查应答的求和校验
 *
 * 更高层的“地址/功能码/参数码是否匹配”由 WIRELESS_CheckReply() 负责。
 */
static uint32_t WIRELESS_Transceive(const uint8_t tx[8], uint8_t rx[8]) {
#ifdef DEBUG_WIRELESS
    printf("WIRELESS TX: ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", tx[i]);
    }
    printf("\r\n");
#endif

    UART6_DrainRX_UntilIdle(5);

    if (HAL_UART_Transmit(&huart6, (uint8_t*)tx, 8, DSM_CMD_TIMEOUT) != HAL_OK) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS TX failed\r\n");
#endif
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    uint32_t start = HAL_GetTick();
    int got = 0;
    while ((HAL_GetTick() - start) < WIRELESS_RX_TIMEOUT && got < 8) {
        if (HAL_UART_Receive(&huart6, &rx[got], 1, 1) == HAL_OK) {
            got++;
        }
    }

    if (got < 8) {
#ifdef DEBUG_WIRELESS
        if (got == 0) {
            printf("WIRELESS RX timeout, got 0 bytes\r\n");
        } else {
            printf("WIRELESS RX length error, got %d bytes\r\n", got);
        }
#endif
        return (got == 0) ? SENSOR_DEVICE_COMM_TIMEOUT : SENSOR_RESP_FORMAT_ERROR;
    }

#ifdef DEBUG_WIRELESS
    printf("WIRELESS RX: ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", rx[i]);
    }
    printf("\r\n");
#endif

    if (WIRELESS_CalcSum(rx) != rx[7]) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX checksum error: calc=%02X, rx=%02X\r\n",
               WIRELESS_CalcSum(rx), rx[7]);
#endif
        return SENSOR_BCC_ERROR;
    }

    return NO_ERROR;
}

/*
 * 检查回复帧的协议语义。
 *
 * 校验项：
 * 1. 地址必须匹配
 * 2. 功能码必须是请求功能码的应答形式
 * 3. 参数码不能是 0xFF
 * 4. 参数码必须和请求值一致
 */
static uint32_t WIRELESS_CheckReply(const uint8_t tx[8], const uint8_t rx[8]) {
    uint8_t expect_addr  = tx[0];
    uint8_t expect_func  = tx[1] | 0x80;
    uint8_t expect_param = tx[6];

    if (rx[0] != expect_addr) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX addr mismatch: expect %02X, got %02X\r\n",
               expect_addr, rx[0]);
#endif
        return SENSOR_RESP_FORMAT_ERROR;
    }

    if (rx[1] != expect_func) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX func mismatch: expect %02X, got %02X\r\n",
               expect_func, rx[1]);
#endif
        return SENSOR_RESP_FORMAT_ERROR;
    }

    if (rx[6] == 0xFF) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX indicates FAIL (param=FF)\r\n");
#endif
        return SENSOR_DEVICE_REPORTED_ERROR;
    }

    if (rx[6] != expect_param) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX param mismatch: expect %02X, got %02X\r\n",
               expect_param, rx[6]);
#endif
        return SENSOR_RESP_FORMAT_ERROR;
    }

    return NO_ERROR;
}

/* 把回复数据区 [2..5] 按小端解释为 int32。 */
static inline uint32_t WIRELESS_ParseInt32_LE(const uint8_t *d) {
    return ((uint32_t)d[3] << 24 |
            (uint32_t)d[2] << 16 |
            (uint32_t)d[1] << 8  |
            (uint32_t)d[0]);
}

/* 把回复数据区 [2..5] 按小端解释为 float。 */
static inline float WIRELESS_ParseFloat_LE(const uint8_t *d) {
    union {
        uint32_t u;
        float f;
    } cvt;

    cvt.u = ((uint32_t)d[3] << 24 |
             (uint32_t)d[2] << 16 |
             (uint32_t)d[1] << 8  |
             (uint32_t)d[0]);
    return cvt.f;
}

/*
 * 读取无线节点的浮点参数。
 *
 * 流程：
 * 1. 组读取命令
 * 2. 最多重试 WIRELESS_MAX_RETRY 次
 * 3. 每次执行收发和回复校验
 * 4. 成功后把数据区转成 float 返回
 */
uint32_t WIRELESS_Read_FloatParam(uint8_t addr, uint8_t param, float *out_value)
{
    if (!out_value) {
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    uint8_t tx[8], rx[8];
    int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

    WIRELESS_MakeFrame(tx, addr, (uint8_t)WIRELESS_FUNC_R, 0x00000000u, param);

    for (int attempt = 0; attempt < WIRELESS_MAX_RETRY; ++attempt) {
        HAL_Delay(DSM_PRE_SEND_DELAY);

        int ret = WIRELESS_Transceive(tx, rx);
        if (ret != NO_ERROR) {
            last_err = ret;
            continue;
        }

        ret = WIRELESS_CheckReply(tx, rx);
        if (ret == NO_ERROR) {
            float v = WIRELESS_ParseFloat_LE(rx + 2);
            *out_value = v;
#ifdef DEBUG_WIRELESS
            printf("[WIRELESS addr=%02X] Read Float R %u: %f\r\n",
                   (unsigned)addr, (unsigned)param, (double)v);
#endif
            return NO_ERROR;
        }

        last_err = ret;
        HAL_Delay(DSM_BCC_DELAY);
    }

    return last_err;
}

/*
 * 读取无线节点的整型参数。
 * 与浮点读取的流程一致，只是最终解析类型不同。
 */
uint32_t WIRELESS_Read_IntParam(uint8_t addr, uint8_t param, int32_t *out_value)
{
    if (!out_value) {
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    uint8_t tx[8], rx[8];
    int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

    WIRELESS_MakeFrame(tx, addr, (uint8_t)WIRELESS_FUNC_R, 0x00000000u, param);

    for (int attempt = 0; attempt < WIRELESS_MAX_RETRY; ++attempt) {
        HAL_Delay(DSM_PRE_SEND_DELAY);

        int ret = WIRELESS_Transceive(tx, rx);
        if (ret != NO_ERROR) {
            last_err = ret;
            continue;
        }

        ret = WIRELESS_CheckReply(tx, rx);
        if (ret == NO_ERROR) {
            int32_t v = WIRELESS_ParseInt32_LE(rx + 2);
            *out_value = v;
#ifdef DEBUG_WIRELESS
            printf("[WIRELESS addr=%02X] Read Int R %u: %ld (0x%08lX)\r\n",
                   (unsigned)addr, (unsigned)param, (long)v, (unsigned long)v);
#endif
            return NO_ERROR;
        }

        last_err = ret;
        HAL_Delay(DSM_BCC_DELAY);
    }

    return last_err;
}

/* 读取软件版本，参数码固定为 0x00。 */
uint32_t WIRELESS_Read_SoftwareVersion(uint8_t addr, float *v)
{
    return WIRELESS_Read_FloatParam(addr, 0x00, v);
}

/* 读取节点电压，参数码固定为 0x01。 */
uint32_t WIRELESS_Read_Voltage(uint8_t addr, float *v)
{
    return WIRELESS_Read_FloatParam(addr, 0x01, v);
}

/*
 * 打印无线节点摘要信息。
 *
 * 输出策略采用单行摘要，便于在识别阶段快速阅读：
 *   无线主机 OK | Ver=1.230 | Volt=24.500 V
 *
 * 失败时直接打印失败原因并返回错误码。
 */
uint32_t WIRELESS_PrintInfo(uint8_t addr)
{
    float ver = 0.0f;
    float volt = 0.0f;
    const char *role = NULL;

    if (addr == 1) {
        role = "无线主机";
    } else if (addr == 2) {
        role = "无线从机";
    } else {
        role = "无线节点";
    }

    uint32_t ret_ver = WIRELESS_Read_SoftwareVersion(addr, &ver);
    if (ret_ver != NO_ERROR) {
        printf("%s: 软件版本读取失败(err=%ld)\r\n", role, ret_ver);
        return ret_ver;
    }

    uint32_t ret_volt = WIRELESS_Read_Voltage(addr, &volt);
    if (ret_volt != NO_ERROR) {
        printf("%s: 电压读取失败(err=%ld)\r\n", role, ret_volt);
        return ret_volt;
    }

    printf("%s OK | Ver=%.3f | Volt=%.3f V\r\n",
           role,
           (double)ver,
           (double)volt);
    return NO_ERROR;
}

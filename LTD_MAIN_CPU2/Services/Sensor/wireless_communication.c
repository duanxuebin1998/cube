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
#include "system_parameter.h"
#include "sensor.h"
#include "error_log.h"

#ifndef WIRELESS_MAX_RETRY
#define WIRELESS_MAX_RETRY   UART6_COMM_MAX_RETRY
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
 * 如果上一次通信残留半包/误包，下一次固定 8 字节回复很容易误位。
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

static uint8_t s_wireless_dma_rx_buf[8]; /* 无线通信数据缓冲区，注意与中断或 DMA 访问边界保持一致。 */

/**
 * @brief 停止 UART6 DMA 接收并清理无线 8 字节协议的硬件错误状态。
 *
 * 无线设备参数读取与传感器共用 UART6，提前退出时必须释放 DMA，防止影响后续透传链路。
 */
static void WIRELESS_StopDmaReceive(void)
{
    (void)HAL_UART_DMAStop(&huart6);
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    huart6.ErrorCode = HAL_UART_ERROR_NONE;
}

/**
 * @brief 获取 UART6 DMA 当前已收到的无线应答长度。
 *
 * DMA 计数器为剩余字节数，这里换算成已收长度；句柄异常时返回 0，由等待逻辑走超时分支。
 */
static uint16_t WIRELESS_GetDmaReceivedLength(uint16_t rx_len)
{
    uint16_t remain;

    if (huart6.hdmarx == NULL) {
        return 0U;
    }
    remain = (uint16_t)__HAL_DMA_GET_COUNTER(huart6.hdmarx);
    if (remain > rx_len) {
        return 0U;
    }
    return (uint16_t)(rx_len - remain);
}

/**
 * @brief 等待无线链路 UART6 DMA 发送完成。
 *
 * 接收 DMA 已提前启动，发送阶段只等待 TX 状态回到 READY；异常退出时释放 DMA，避免影响传感器透传链路。
 */
static uint32_t WIRELESS_WaitTransmitDmaDone(uint32_t timeout)
{
    uint32_t startTick = HAL_GetTick();

    while ((HAL_GetTick() - startTick) < timeout) {
        if (HasEffectiveCommandSwitchRequest()) {
            WIRELESS_StopDmaReceive();
            return STATE_SWITCH;
        }
        if (huart6.gState == HAL_UART_STATE_READY) {
            return NO_ERROR;
        }
        /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            WIRELESS_StopDmaReceive();
            return OTHER_PERIPHERAL_CONFIG_ERROR;
        }
        /* 无线通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
        HAL_Delay(1);
    }

    WIRELESS_StopDmaReceive();
    return SENSOR_DEVICE_COMM_TIMEOUT;
}

/**
 * @brief 启动无线设备固定 8 字节应答的 UART6 DMA 接收。
 *
 * 先打开接收窗口再发送命令，避免无线设备快速回包时丢掉应答帧头。
 */
static uint32_t WIRELESS_StartFixedReceiveDma(uint8_t rx[8])
{
    const uint16_t expect_len = 8U;

    if (rx == NULL) {
        return SENSOR_RESP_FORMAT_ERROR;
    }
    for (uint16_t i = 0U; i < expect_len; i++) {
        s_wireless_dma_rx_buf[i] = 0U;
        rx[i] = 0U;
    }

    WIRELESS_StopDmaReceive();
    if (HAL_UART_Receive_DMA(&huart6, s_wireless_dma_rx_buf, expect_len) != HAL_OK) {
        WIRELESS_StopDmaReceive();
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }
    return NO_ERROR;
}

/**
 * @brief 等待无线设备固定 8 字节应答接收完成。
 *
 * 本函数只负责帧长度和 UART 硬件错误判断，校验和、地址、功能码和参数码仍由原有逻辑检查。
 */
static uint32_t WIRELESS_WaitFixedReceiveDma(uint8_t rx[8], uint32_t timeout)
{
    const uint16_t expect_len = 8U;
    uint32_t startTick;

    if (rx == NULL) {
        return SENSOR_RESP_FORMAT_ERROR;
    }

    startTick = HAL_GetTick();
    while ((HAL_GetTick() - startTick) < timeout) {
        uint16_t got = WIRELESS_GetDmaReceivedLength(expect_len);

        if (HasEffectiveCommandSwitchRequest()) {
            WIRELESS_StopDmaReceive();
            return STATE_SWITCH;
        }
        /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
        if (huart6.ErrorCode != HAL_UART_ERROR_NONE) {
            WIRELESS_StopDmaReceive();
            return SENSOR_RESP_FORMAT_ERROR;
        }
        if (got >= expect_len) {
            for (uint16_t i = 0U; i < expect_len; i++) {
                rx[i] = s_wireless_dma_rx_buf[i];
            }
            WIRELESS_StopDmaReceive();
            return NO_ERROR;
        }
        /* 无线通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
        HAL_Delay(1);
    }

    {
        uint16_t got = WIRELESS_GetDmaReceivedLength(expect_len);
        for (uint16_t i = 0U; (i < got) && (i < expect_len); i++) {
            rx[i] = s_wireless_dma_rx_buf[i];
        }
        WIRELESS_StopDmaReceive();
        return (got == 0U) ? SENSOR_DEVICE_COMM_TIMEOUT : SENSOR_RESP_FORMAT_ERROR;
    }
}
/*
 * 完成一次完整的无线 8->8 收发。
 *
 * DMA 接收只负责凑满 8 字节，后续校验和、地址、功能码和参数码检查仍保持原有职责分层。
 */
static uint32_t WIRELESS_Transceive(const uint8_t tx[8], uint8_t rx[8]) {
#ifdef DEBUG_WIRELESS
    printf("无线发送: ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", tx[i]);
    }
    printf("\r\n");
#endif

    UART6_DrainRX_UntilIdle(5);

    uint32_t rx_ret = WIRELESS_StartFixedReceiveDma(rx);
    /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
    if (rx_ret != NO_ERROR) {
        return rx_ret;
    }

    if (HAL_UART_Transmit_DMA(&huart6, (uint8_t*)tx, 8) != HAL_OK) {
#ifdef DEBUG_WIRELESS
#endif
        WIRELESS_StopDmaReceive();
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    rx_ret = WIRELESS_WaitTransmitDmaDone(DSM_CMD_TIMEOUT);
    /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
    if (rx_ret != NO_ERROR) {
        return rx_ret;
    }

    rx_ret = WIRELESS_WaitFixedReceiveDma(rx, WIRELESS_RX_TIMEOUT);
    /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
    if (rx_ret != NO_ERROR) {
        return rx_ret;
    }
#ifdef DEBUG_WIRELESS
    printf("无线接收: ");
    for (int i = 0; i < 8; i++) {
        printf("%02X ", rx[i]);
    }
    printf("\r\n");
#endif

    if (WIRELESS_CalcSum(rx) != rx[7]) {
#ifdef DEBUG_WIRELESS
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
        printf("无线接收地址不匹配: 期望：%02X, 实际=%02X\r\n",
               expect_addr, rx[0]);
#endif
        return SENSOR_RESP_FORMAT_ERROR;
    }

    if (rx[1] != expect_func) {
#ifdef DEBUG_WIRELESS
        printf("无线接收功能码不匹配: 期望：%02X, 实际=%02X\r\n",
               expect_func, rx[1]);
#endif
        return SENSOR_RESP_FORMAT_ERROR;
    }

    if (rx[6] == 0xFF) {
#ifdef DEBUG_WIRELESS
#endif
        return SENSOR_DEVICE_REPORTED_ERROR;
    }

    if (rx[6] != expect_param) {
#ifdef DEBUG_WIRELESS
        printf("无线接收参数不匹配: 期望：%02X, 实际=%02X\r\n",
               expect_param, rx[6]);
#endif
        return SENSOR_RESP_FORMAT_ERROR;
    }

    return NO_ERROR;
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
        return PARAM_ADDRESS_OVERFLOW;
    }

    uint8_t tx[8], rx[8];
    int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;
    char detail[64];

    WIRELESS_MakeFrame(tx, addr, (uint8_t)WIRELESS_FUNC_R, 0x00000000u, param);
    snprintf(detail, sizeof(detail), "地址：%u,功能：R,参数：0x%02X", (unsigned)addr, (unsigned)param);

    for (int attempt = 0; attempt < WIRELESS_MAX_RETRY; ++attempt) {
        if (HasEffectiveCommandSwitchRequest()) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        /* 无线通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
        HAL_Delay(DSM_PRE_SEND_DELAY);

        int ret = WIRELESS_Transceive(tx, rx);
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            last_err = ret;
            /* 错误 阶段：错误重试 模块：滑环通信 操作：读取浮点参数 原因：ErrorLog_GetReasonByCode((uint32_t)ret) 尝试：(attempt + 1)/WIRELESS_MAX_RETRY 错误码：ret 错误名：ErrorLog_GetCodeName(ret) 详情：detail */
            ErrorLog_RetryDetail(ERROR_LOG_MODULE_SLIPRING_COMM,
                                 ERROR_LOG_OP_READ_FLOAT_PARAM,
                                 ErrorLog_GetReasonByCode((uint32_t)ret),
                                 (uint32_t)(attempt + 1),
                                 WIRELESS_MAX_RETRY,
                                 (uint32_t)ret,
                                 detail);
            continue;
        }

        ret = WIRELESS_CheckReply(tx, rx);
        if (ret == STATE_SWITCH) {
            /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
            return STATE_SWITCH;
        }
        /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
        if (ret == NO_ERROR) {
            float v = WIRELESS_ParseFloat_LE(rx + 2);
            *out_value = v;
            if (attempt > 0) {
                /* 错误 阶段：重试成功 模块：滑环通信 操作：读取浮点参数 原因：ErrorLog_GetReasonByCode((uint32_t)last_err) 尝试：(attempt + 1)/WIRELESS_MAX_RETRY 详情：detail */
                ErrorLog_RecoverDetail(ERROR_LOG_MODULE_SLIPRING_COMM,
                                       ERROR_LOG_OP_READ_FLOAT_PARAM,
                                       ErrorLog_GetReasonByCode((uint32_t)last_err),
                                       (uint32_t)(attempt + 1),
                                       WIRELESS_MAX_RETRY,
                                       detail);
            }
#ifdef DEBUG_WIRELESS
            printf("[无线 地址：%02X] 读取浮点寄存器 R%u: %f\r\n",
                   (unsigned)addr, (unsigned)param, (double)v);
#endif
            return NO_ERROR;
        }

        last_err = ret;
        /* 错误 阶段：错误重试 模块：滑环通信 操作：读取浮点参数 原因：ErrorLog_GetReasonByCode((uint32_t)ret) 尝试：(attempt + 1)/WIRELESS_MAX_RETRY 错误码：ret 错误名：ErrorLog_GetCodeName(ret) 详情：detail */
        ErrorLog_RetryDetail(ERROR_LOG_MODULE_SLIPRING_COMM,
                             ERROR_LOG_OP_READ_FLOAT_PARAM,
                             ErrorLog_GetReasonByCode((uint32_t)ret),
                             (uint32_t)(attempt + 1),
                             WIRELESS_MAX_RETRY,
                             (uint32_t)ret,
                             detail);
        /* 无线通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
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
 * 轻量探测无线节点是否能应答。
 * 该函数用于传感器通信超时后的链路归因，只做一次最小读请求，
 * 不打印节点信息，也不走读取参数接口的重试日志。
 */
uint32_t WIRELESS_ProbeNode(uint8_t addr)
{
    uint8_t tx[8], rx[8];
    uint32_t ret;

    if (HasEffectiveCommandSwitchRequest()) {
        /* 命令切换是正常打断，直接向上透传，不参与故障重试。 */
        return STATE_SWITCH;
    }

    WIRELESS_MakeFrame(tx, addr, (uint8_t)WIRELESS_FUNC_R, 0x00000000u, 0x01U);
    /* 无线通信与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
    HAL_Delay(DSM_PRE_SEND_DELAY);

    ret = WIRELESS_Transceive(tx, rx);
    /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }

    return WIRELESS_CheckReply(tx, rx);
}

/*
 * 打印无线节点摘要信息。
 *
 * 输出策略采用单行摘要，便于在识别阶段快速阅读：
 *   无线主机 OK | Ver=1.230 | Volt=24.500 V
 *
 * 失败时直接打印失败原因并错误码。
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
    /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
    if (ret_ver != NO_ERROR) {
        printf("%s: 软件版本读取失败 | 错误码：0x%08lX\r\n", role, (unsigned long)ret_ver);
        return ret_ver;
    }

    uint32_t ret_volt = WIRELESS_Read_Voltage(addr, &volt);
    /* 先处理异常边界，避免无线通信状态机带故障继续运行。 */
    if (ret_volt != NO_ERROR) {
        printf("%s: 电压读取失败 | 错误码：0x%08lX\r\n", role, (unsigned long)ret_volt);
        return ret_volt;
    }

    printf("%s 正常 | 版本：%.3f | 电压：%.3f V\r\n",
           role,
           (double)ver,
           (double)volt);
    return NO_ERROR;
}

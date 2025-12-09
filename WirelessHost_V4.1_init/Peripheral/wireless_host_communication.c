/*
 * wireless_host_communication.c
 *
 *  Created on: 2025年12月8日
 *      Author: admin
 */

#include "wireless_host_communication.h"
#include <string.h>
#include <stdio.h>

#define WIRELESS_SLAVE_ADDR   0x01   // ★ 本机地址，根据实际修改

// 示例：本机维护两个参数（可换成你的系统参数）
static float g_sw_version = 4.10f;   // 参数码 0x00
static float g_voltage    = 24.50f;  // 参数码 0x01

// 求前7字节的和
static uint8_t wireless_sum7(const uint8_t *b)
{
    uint32_t s = 0;
    for (int i = 0; i < 7; ++i) s += b[i];
    return (uint8_t)(s & 0xFF);
}

// 小端写 float
static void wireless_write_float_le(uint8_t *d, float f)
{
    union { float f; uint32_t u; } cvt;
    cvt.f  = f;
    d[0] = (uint8_t)(cvt.u & 0xFF);
    d[1] = (uint8_t)((cvt.u >> 8)  & 0xFF);
    d[2] = (uint8_t)((cvt.u >> 16) & 0xFF);
    d[3] = (uint8_t)((cvt.u >> 24) & 0xFF);
}

// 小端读 float
static float wireless_read_float_le(const uint8_t *d)
{
    union { float f; uint32_t u; } cvt;
    cvt.u =  (uint32_t)d[0]
           | ((uint32_t)d[1] << 8)
           | ((uint32_t)d[2] << 16)
           | ((uint32_t)d[3] << 24);
    return cvt.f;
}

// 处理主机→本机的一帧（8字节）
// 前提：这帧已经转发给了下游（USART3），这里仅决定“本机要不要额外回主机”
void Wireless_Handle_MasterFrame(const uint8_t req[8])
{
    // 1) 校验和检查
    uint8_t calc = wireless_sum7(req);
    if (calc != req[7]) {
        printf("[WLS SLAVE] checksum error: calc=%02X, recv=%02X\r\n", calc, req[7]);
        return; // 校验错不响应
    }

    uint8_t addr  = req[0];
    uint8_t func  = req[1];   // 'R' / 'W'
    uint8_t param = req[6];

    // 2) 不是本机地址 → 不响应（但已经透传过了）
    if (addr != WIRELESS_SLAVE_ADDR) {
        return;
    }

    // 3) 按功能码+参数码处理，构造 8 字节应答
    uint8_t resp[8] = {0};

    resp[0] = WIRELESS_SLAVE_ADDR;
    resp[1] = func | 0x80;          // ★ 如果你想回复功能码|0x80，这里改成：func | 0x80

    if (func == 'R') {
        // 读参数：主机 data 无效，从机返回 data 为当前值
        float value = 0.0f;
        int   ok    = 1;

        switch (param)
        {
        case 0x00:  // 软件版本
            value = g_sw_version;
            break;

        case 0x01:  // 电压
            value = g_voltage;
            break;

        default:
            ok = 0;
            break;
        }

        if (ok) {
            wireless_write_float_le(&resp[2], value);
            resp[6] = param;      // 成功：返回原参数码
        } else {
            memset(&resp[2], 0, 4);
            resp[6] = 0xFF;       // 失败：0xFF
        }
    }
    else if (func == 'W') {
        // 写参数：主机 data 带要写入的值，从机写入后返回该值
        float w = wireless_read_float_le(&req[2]);
        int   ok = 1;

        switch (param)
        {
        case 0x00:
            g_sw_version = w;
            break;
        case 0x01:
            g_voltage    = w;
            break;
        default:
            ok = 0;
            break;
        }

        if (ok) {
            wireless_write_float_le(&resp[2], w);
            resp[6] = param;
        } else {
            memset(&resp[2], 0, 4);
            resp[6] = 0xFF;
        }
    }
    else {
        // 未知功能码
        memset(&resp[2], 0, 4);
        resp[6] = 0xFF;
    }

    resp[7] = wireless_sum7(resp);

    // 4) 把应答通过 USART2 回主机
    HAL_UART_Transmit(&huart2, resp, 8, 0xFFFF);

    printf("[WLS SLAVE] resp -> host: addr=%02X func=%c param=%02X\r\n",
           addr, func, param);
}

/*
 * wireless_communication.c
 *
 *  Created on: 2025年12月8日
 *      Author: admin
 */

#include "ltd_sensor_communication.h"
#include "sensor.h"

#ifndef WIRELESS_MAX_RETRY
#define WIRELESS_MAX_RETRY   3
#endif
#ifndef WIRELESS_RX_TIMEOUT
#define WIRELESS_RX_TIMEOUT  DSM_CMD_TIMEOUT
#endif

// 主机方向功能码
typedef enum {
	WIRELESS_FUNC_R = 'R', WIRELESS_FUNC_W = 'W',
} wireless_func_t;

// === 内部：求和校验（前7字节） ===
static inline uint8_t WIRELESS_CalcSum(const uint8_t f[8]) {
	uint32_t s = 0;
	for (int i = 0; i < 7; ++i)
		s += f[i];
	return (uint8_t) (s & 0xFF);
}

// === 内部：打帧（addr 由上层传入，data 大端入参；若给 0 则无所谓端序） ===
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


// === 内部：传输 8→8 ===

// === 1) 发前清空可能的残留：非正式“flush” ===
static void UART6_DrainRX_UntilIdle(uint32_t idle_ms) {
    uint8_t dump;
    uint32_t last = HAL_GetTick();
    for (;;) {
        // 1字节超时设为1ms：有字节就读走并更新时间；没字节就看是否空闲超时
        if (HAL_UART_Receive(&huart6, &dump, 1, 1) == HAL_OK) {
            last = HAL_GetTick(); // 读到了字节，刷新“最近一次活动时间”
        } else {
            if ((HAL_GetTick() - last) >= idle_ms) break; // 空闲>=idle_ms 认为干净
        }
    }

    // 清理溢出等异常标志（可选，避免ORE悬挂）
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    // 读SR/DR清RXNE的老派做法（F4上清ORE通常需要读SR后读DR，HAL宏已封装）
}

static uint32_t WIRELESS_Transceive(const uint8_t tx[8], uint8_t rx[8]) {
#ifdef DEBUG_WIRELESS
	printf("WIRELESS TX: ");
	for (int i = 0; i < 8; i++)
		printf("%02X ", tx[i]);
	printf("\r\n");
#endif
	UART6_DrainRX_UntilIdle(5); // 发前清空残留数据
	if (HAL_UART_Transmit(&huart6, (uint8_t*) tx, 8, DSM_CMD_TIMEOUT) != HAL_OK) {
#ifdef DEBUG_WIRELESS
		printf("WIRELESS TX failed\r\n");
#endif
		return OTHER_PERIPHERAL_CONFIG_ERROR;
	}

	uint32_t start = HAL_GetTick();
	int got = 0;
	while ((HAL_GetTick() - start) < WIRELESS_RX_TIMEOUT && got < 8) {
		if (HAL_UART_Receive(&huart6, &rx[got], 1, 1) == HAL_OK)
			got++;
	}
	if (got < 8) {
#ifdef DEBUG_WIRELESS
		printf("WIRELESS RX timeout, got %d bytes\r\n", got);
#endif
		return SENSOR_COMM_TIMEOUT;
	}

#ifdef DEBUG_WIRELESS
	printf("WIRELESS RX: ");
	for (int i = 0; i < 8; i++)
		printf("%02X ", rx[i]);
	printf("\r\n");
#endif

	if (WIRELESS_CalcSum(rx) != rx[7]) {
#ifdef DEBUG_WIRELESS
		printf("WIRELESS RX checksum error: calc=%02X, rx=%02X\r\n", WIRELESS_CalcSum(rx), rx[7]);
#endif
		return SENSOR_BCC_ERROR;
	}
	return NO_ERROR;
}

// === 内部：校验功能码/参数码 ===
static uint32_t WIRELESS_CheckReply(const uint8_t tx[8], const uint8_t rx[8]) {
    uint8_t expect_addr  = tx[0];
    uint8_t expect_func  = tx[1] | 0x80; // 如果实际协议没有高位+1，这里改成 tx[1]
    uint8_t expect_param = tx[6];

    if (rx[0] != expect_addr) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX addr mismatch: expect %02X, got %02X\r\n",
               expect_addr, rx[0]);
#endif
        return SENSOR_COMM_TIMEOUT;
    }

    if (rx[1] != expect_func) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX func mismatch: expect %02X, got %02X\r\n",
               expect_func, rx[1]);
#endif
        return SENSOR_COMM_TIMEOUT;
    }

    if (rx[6] == 0xFF) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX indicates FAIL (param=FF)\r\n");
#endif
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    if (rx[6] != expect_param) {
#ifdef DEBUG_WIRELESS
        printf("WIRELESS RX param mismatch: expect %02X, got %02X\r\n",
               expect_param, rx[6]);
#endif
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }
    return NO_ERROR;
}


// === 内部：端序解析 ===
static inline uint32_t WIRELESS_ParseInt32_LE(const uint8_t *d) {
	return  ((uint32_t) d[3] << 24 | (uint32_t) d[2] << 16 | (uint32_t) d[1] << 8 | (uint32_t) d[0]);
}
static inline float WIRELESS_ParseFloat_LE(const uint8_t *d) {
	union {
		uint32_t u;
		float f;
	} cvt;
	cvt.u = (uint32_t) d[3] << 24 | (uint32_t) d[2] << 16 | (uint32_t) d[1] << 8 | (uint32_t) d[0];
	return cvt.f;
}



uint32_t WIRELESS_Read_FloatParam(uint8_t addr, uint8_t param, float *out_value)
{
    if (!out_value)
        return OTHER_PERIPHERAL_CONFIG_ERROR;

    uint8_t tx[8], rx[8];
    int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

    // 这里传入 addr
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


uint32_t WIRELESS_Read_IntParam(uint8_t addr, uint8_t param, int32_t *out_value)
{
    if (!out_value)
        return OTHER_PERIPHERAL_CONFIG_ERROR;

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


// === 便捷读取 ===
// param 0x00: 软件版本
uint32_t WIRELESS_Read_SoftwareVersion(uint8_t addr, float *v)
{
    return WIRELESS_Read_FloatParam(addr, 0x00, v);
}

// param 0x01: 电压
uint32_t WIRELESS_Read_Voltage(uint8_t addr, float *v)
{
    return WIRELESS_Read_FloatParam(addr, 0x01, v);
}

uint32_t WIRELESS_PrintInfo(uint8_t addr)
{
    float ver = 0.0f;
    float volt = 0.0f;
    if (addr == 1)
        {
    		printf("==== 无线主机信息读取 ====\r\n");
        }
        else if (addr == 2)
        {
        	 printf("==== 无线从机信息读取 ====\r\n");
        }

    // 打印版本号
    uint32_t ret_ver  = WIRELESS_Read_SoftwareVersion(addr, &ver);
    if (ret_ver == NO_ERROR) {
        printf("  软件版本 : %.3f\r\n", (double)ver);
    } else {
        printf("  软件版本 : READ FAIL (err=%ld)\r\n", ret_ver);
        return ret_ver;
    }
    uint32_t ret_volt = WIRELESS_Read_Voltage(addr, &volt);
    // 打印电压
    if (ret_volt == NO_ERROR) {
        printf("  电压    : %.3f V\r\n", (double)volt);
    } else {
        printf("  电压    : READ FAIL (err=%ld)\r\n", ret_volt);
        return ret_volt;
    }

    printf("===========================================\r\n");
    return NO_ERROR;  // 其他未定义地址
}



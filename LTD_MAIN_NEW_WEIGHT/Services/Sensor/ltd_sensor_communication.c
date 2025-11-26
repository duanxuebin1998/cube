/*
 * dsm_v2.c
 *
 *  Created on: Nov 10, 2025
 *      Author: admin
 */

#include <ltd_sensor_communication.h>
#include "sensor.h"

#ifndef DSM_V2_MAX_RETRY
#define DSM_V2_MAX_RETRY   3
#endif
#ifndef DSM_V2_RX_TIMEOUT
#define DSM_V2_RX_TIMEOUT  DSM_CMD_TIMEOUT
#endif

// 主机方向功能码
typedef enum {
	DSM_V2_FUNC_R = 'R', DSM_V2_FUNC_W = 'W', DSM_V2_FUNC_L = 'T', DSM_V2_FUNC_D = 'D', DSM_V2_FUNC_B = 'B',
} dsm_v2_func_t;

// === 内部：求和校验（前7字节） ===
static inline uint8_t DSM_V2_CalcSum(const uint8_t f[8]) {
	uint32_t s = 0;
	for (int i = 0; i < 7; ++i)
		s += f[i];
	return (uint8_t) (s & 0xFF);
}

// === 内部：打帧（addr=0x00，data 大端入参；若给 0 则无所谓端序） ===
static inline void DSM_V2_MakeFrame(uint8_t out[8], uint8_t func, uint32_t data_be, uint8_t param) {
	out[0] = 0x00;
	out[1] = func;
	out[2] = (uint8_t) ((data_be >> 24) & 0xFF);
	out[3] = (uint8_t) ((data_be >> 16) & 0xFF);
	out[4] = (uint8_t) ((data_be >> 8) & 0xFF);
	out[5] = (uint8_t) (data_be & 0xFF);
	out[6] = param;
	out[7] = DSM_V2_CalcSum(out);
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

static int DSM_V2_Transceive(const uint8_t tx[8], uint8_t rx[8]) {
#ifdef DEBUG_DSM
	printf("V2 TX: ");
	for (int i = 0; i < 8; i++)
		printf("%02X ", tx[i]);
	printf("\r\n");
#endif
	UART6_DrainRX_UntilIdle(5); // 发前清空残留数据
	if (HAL_UART_Transmit(&huart6, (uint8_t*) tx, 8, DSM_CMD_TIMEOUT) != HAL_OK) {
#ifdef DEBUG_DSM
		printf("V2 TX failed\r\n");
#endif
		return OTHER_PERIPHERAL_CONFIG_ERROR;
	}

	uint32_t start = HAL_GetTick();
	int got = 0;
	while ((HAL_GetTick() - start) < DSM_V2_RX_TIMEOUT && got < 8) {
		if (HAL_UART_Receive(&huart6, &rx[got], 1, 1) == HAL_OK)
			got++;
	}
	if (got < 8) {
#ifdef DEBUG_DSM
		printf("V2 RX timeout, got %d bytes\r\n", got);
#endif
		return SENSOR_COMM_TIMEOUT;
	}

#ifdef DEBUG_DSM
	printf("V2 RX: ");
	for (int i = 0; i < 8; i++)
		printf("%02X ", rx[i]);
	printf("\r\n");
#endif

	if (DSM_V2_CalcSum(rx) != rx[7]) {
#ifdef DEBUG_DSM
		printf("V2 RX checksum error: calc=%02X, rx=%02X\r\n", DSM_V2_CalcSum(rx), rx[7]);
#endif
		return SENSOR_BCC_ERROR;
	}
	return NO_ERROR;
}

// === 内部：校验功能码/参数码 ===
static int DSM_V2_CheckReply(const uint8_t tx[8], const uint8_t rx[8]) {
	uint8_t expect_func = tx[1] | 0x80; // 从机高位置1
	uint8_t expect_param = tx[6];        // 设置模式时固定 0x00

	if (rx[1] != expect_func) {
#ifdef DEBUG_DSM
		printf("V2 RX func mismatch: expect %02X, got %02X\r\n", expect_func, rx[1]);
#endif
		return SENSOR_COMM_TIMEOUT;
	}
	if (rx[6] == 0xFF) {
#ifdef DEBUG_DSM
		printf("V2 RX indicates FAIL (param=FF)\r\n");
#endif
		return OTHER_PERIPHERAL_CONFIG_ERROR;
	}
	if (rx[6] != expect_param) {
#ifdef DEBUG_DSM
		printf("V2 RX param mismatch: expect %02X, got %02X\r\n", expect_param, rx[6]);
#endif
		return OTHER_PERIPHERAL_CONFIG_ERROR;
	}
	return NO_ERROR;
}

// === 内部：端序解析 ===
static inline int32_t DSM_V2_ParseInt32_LE(const uint8_t *d) {
	return (int32_t) ((uint32_t) d[3] << 24 | (uint32_t) d[2] << 16 | (uint32_t) d[1] << 8 | (uint32_t) d[0]);
}
static inline float DSM_V2_ParseFloat_LE(const uint8_t *d) {
	union {
		uint32_t u;
		float f;
	} cvt;
	cvt.u = (uint32_t) d[3] << 24 | (uint32_t) d[2] << 16 | (uint32_t) d[1] << 8 | (uint32_t) d[0];
	return cvt.f;
}

// === 对外：切换模式（param=0x00） ===
int DSM_V2_SwitchMode(dsm_v2_mode_t mode) {
	uint8_t tx[8], rx[8];
	int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

	DSM_V2_MakeFrame(tx, (uint8_t) mode, 0x00000000u, (uint8_t) mode); // param 必须为 0x00

	for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = DSM_V2_Transceive(tx, rx);
		if (ret != NO_ERROR) {
			last_err = ret;
			continue;
		}

		ret = DSM_V2_CheckReply(tx, rx);
		if (ret == NO_ERROR) {
#ifdef DEBUG_DSM
			printf("[V2] switch mode '%c' OK\r\n", (char) mode);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		HAL_Delay(DSM_BCC_DELAY);
	}
#ifdef DEBUG_DSM
	printf("[V2] switch mode '%c' FAIL, err=%d\r\n", (char) mode, last_err);
#endif
	return last_err;
}
int DSM_V2_SwitchToLevelMode(void) {
	return DSM_V2_SwitchMode(DSM_V2_MODE_LEVEL);
}
int DSM_V2_SwitchToDensityMode(void) {
	return DSM_V2_SwitchMode(DSM_V2_MODE_DENSITY);
}

// === 对外：通用读取 ===
int DSM_V2_Read_FloatParam(uint8_t param, float *out_value) {
	if (!out_value)
		return OTHER_PERIPHERAL_CONFIG_ERROR;

	uint8_t tx[8], rx[8];
	int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

	DSM_V2_MakeFrame(tx, (uint8_t) DSM_V2_FUNC_R, 0x00000000u, param);

	for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = DSM_V2_Transceive(tx, rx);
		if (ret != NO_ERROR) {
			last_err = ret;
			continue;
		}

		ret = DSM_V2_CheckReply(tx, rx);
		if (ret == NO_ERROR) {
			float v = DSM_V2_ParseFloat_LE(rx + 2);
			*out_value = v;
#ifdef DEBUG_DSM
			printf("[V2] Read Float R %u: %f\r\n", (unsigned) param, (double) v);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		HAL_Delay(DSM_BCC_DELAY);
	}
	return last_err;
}

int DSM_V2_Read_IntParam(uint8_t param, int32_t *out_value) {
	if (!out_value)
		return OTHER_PERIPHERAL_CONFIG_ERROR;

	uint8_t tx[8], rx[8];
	int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

	DSM_V2_MakeFrame(tx, (uint8_t) DSM_V2_FUNC_R, 0x00000000u, param);

	for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
		HAL_Delay(DSM_PRE_SEND_DELAY);
		int ret = DSM_V2_Transceive(tx, rx);
		if (ret != NO_ERROR) {
			last_err = ret;
			continue;
		}

		ret = DSM_V2_CheckReply(tx, rx);
		if (ret == NO_ERROR) {
			int32_t v = DSM_V2_ParseInt32_LE(rx + 2);
			*out_value = v;
#ifdef DEBUG_DSM
			printf("[V2] Read Int R %u: %ld (0x%08lX)\r\n", (unsigned) param, (long) v, (unsigned long) v);
#endif
			return NO_ERROR;
		}
		last_err = ret;
		HAL_Delay(DSM_BCC_DELAY);
	}
	return last_err;
}

// === 便捷读取 ===
int DSM_V2_Read_SoftwareVersion(float *v) {
	return DSM_V2_Read_FloatParam(0x00, v);
}
int DSM_V2_Read_Temperature(float *t) {
	return DSM_V2_Read_FloatParam(0x06, t);
}
int DSM_V2_Read_Density(float *rho) {
	return DSM_V2_Read_FloatParam(0x07, rho);
}
int DSM_V2_Read_DynamicViscosity(float *mu) {
	return DSM_V2_Read_FloatParam(0x08, mu);
}
int DSM_V2_Read_KinematicViscosity(float *nu) {
	return DSM_V2_Read_FloatParam(0x09, nu);
}
int DSM_V2_Read_MeanSquare45(float *msq45) {
	return DSM_V2_Read_FloatParam(0x11, msq45);
} // 17
int DSM_V2_Read_MeanSquare22p5(float *msq22p5) {
	return DSM_V2_Read_FloatParam(0x12, msq22p5);
} // 18
// R04 液位频率（整型）
int DSM_V2_Read_LevelFrequency(uint32_t *freq_hz) {
	if (!freq_hz)
		return OTHER_PERIPHERAL_CONFIG_ERROR;
	int32_t v = 0;
	int ret = DSM_V2_Read_IntParam(0x04, &v);   // 参数码 0x04 = R04
	if (ret == NO_ERROR) {
		if (v < 0) {
			v = -v;
		}
		if (v > 10000) {
			v = v - 10000;
		}
		*freq_hz = (uint32_t) v;
	}
	return ret;
}

int DSM_V2_Read_SensorID(uint32_t *sensor_id) {
	if (!sensor_id)
		return OTHER_PERIPHERAL_CONFIG_ERROR;
	int32_t v = 0;
	int ret = DSM_V2_Read_IntParam(0x16, &v); // 22
	if (ret == NO_ERROR)
		*sensor_id = (uint32_t) v;
	return ret;
}

// R16 频率寄存器（整型）
int DSM_V2_Read_FrequencyRegister(uint32_t *freq_reg_value) {
	if (!freq_reg_value)
		return OTHER_PERIPHERAL_CONFIG_ERROR;
	int32_t v = 0;
	int ret = DSM_V2_Read_IntParam(0x10, &v);   // 16
	if (ret == NO_ERROR)
		*freq_reg_value = (uint32_t) v;
	return ret;
}

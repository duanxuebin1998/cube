/*
 * dsm_v2.c
 *
 *  Created on: Nov 10, 2025
 *      Author: admin
 */

/*****************新通讯协议说明V2.0，2022.8.3***********
一、协议包说明：
通讯协议包长为8个字节，
1.首字节为地址，保留，目前使用00
2.第二字节为功能码，主机到从机为W,R,L,D,B，从机到主机则在功能码高位置一，
	W为写入参数
	R为读取参数
	T为测量液位，设置测量模式，交互通讯，液位、水位，无罐底功能，液位扫频方式
	D为测量密度，设置测量模式
	B为测量罐底，设置测量模式，主动发送，实时发送液位、水位和罐底状态，液位固定频率方式
3.第3-6个字节为数据位，
	1）主机到从机：读取和设置测量模式时，无效，写入参数时为参数值
	2）从机到主机：读取时返回数据，写入时返回写入值，设置测量模式时，无效，返回00
4.第七个字节为参数码，
	1）主机到从机：读取和写入参数时，需要操作的参数，设置测量模式时，使用00
	2）从机到主机：用于指示是否成功
	成功时，返回参数码；失败时返回FF
5.第8个字节为校验码，为以上7个字节的和

二、具体参数
功能码	参数码	参数		读写类型	数据类型
		L		00			无			无				无
		D		00			无			无				无
		B		00			无			无				无
	=======================测量值（只读参数）===================================
	 --------------------------基本信息-------------------------------------
		R		00			软件版本	R				单浮点
		R		01			存储版本	R				单浮点
		R		02			传感器状态	R				整型
	 --------------------------测量输出值------------------------------------
		R		03			零点电压	R				单浮点
		R		04			液位频率	R				整型
		R		05			水位电压	R				单浮点
		R		06			温度值		R				单浮点
		R		07			密度值		R				单浮点
		R		08			动力粘度	R				单浮点
		R		09			运动粘度	R				单浮点
	 --------------------------测量原始数据----------------------------------
		R		10			零点AD均值			R				整型
		R		11			液位AD值			R				整型
		R		12			液位空气频率均值	R				整型
		R		13			水位AD均值			R				整型
		R		14			温度AD均值			R				整型
		R		15			当前频率AD均值		R				整型
		R		16			频率寄存器值		R				整型
		R		17			45周期平方均值		R				单浮点
		R		18			22.5周期平方均值	R				单浮点
		R		19			周期平方均值差值	R				单浮点
	========================参数（可读可写）=====================================
	 --------------------------基本参数--------------------------------------
		W		20			通讯方式				W				整型
		W		21			地址					W				整型
		W		22			传感器号				W				整型
		W		23			扫频存储数				W				整型
		W		24			密度计算循环次数		W				整型
	 ---------------------------温度相关-------------------------------------
		W		25			基准温度				W				单浮点
		W		26			温度修正值				W				单浮点
	 ---------------------------基础密度点------------------------------------
		W		27			密度点1					W				单浮点
		W		28			45周期平方				W				单浮点
		W		29			密度点2					W				单浮点
		W		30			45周期平方				W				单浮点
		W		31			密度点3					W				单浮点
		W		32			45周期平方				W				单浮点
		W		33			密度点4					W				单浮点
		W		34			45周期平方				W				单浮点
	 ---------------------------粘度计算-----------------------------------
		W		35			1.动粘*密度^2	W			单浮点
		W		36			1.周期平方差	W			单浮点
		W		37			2.动粘*密度^2	W			单浮点
		W		38			2.周期平方差	W			单浮点
		W		39			3.动粘*密度^2	W			单浮点
		W		40			3.周期平方差	W			单浮点
		W		41			4.动粘*密度^2	W			单浮点
		W		42			4.周期平方差	W			单浮点
		W		43			5.动粘*密度^2	W			单浮点
		W		44			5.周期平方差	W			单浮点
		W		45			6.动粘*密度^2	W			单浮点
		W		46			6.周期平方差	W			单浮点
		W		47			7.动粘*密度^2	W			单浮点
		W		48			7.周期平方差	W			单浮点
		W		49			8.动粘*密度^2	W			单浮点
		W		50			8.周期平方差	W			单浮点
		W		51			9.动粘*密度^2	W			单浮点
		W		52			9.周期平方差	W			单浮点
	 ---------------------------粘度修正-----------------------------------
		W		53			1.动力粘度		W			单浮点
		W		54			1.密度修正		W			单浮点
		W		55			2.动力粘度		W			单浮点
		W		56			2.密度修正		W			单浮点
		W		57			3.动力粘度		W			单浮点
		W		58			3.密度修正		W			单浮点
		W		59			4.动力粘度		W			单浮点
		W		60			4.密度修正		W			单浮点
		W		61			5.动力粘度		W			单浮点
		W		62			5.密度修正		W			单浮点
		W		63			6.动力粘度		W			单浮点
		W		64			6.密度修正		W			单浮点
	 ---------------------------密度的温度修正（30度）-------------------------------
		W		65			1.密度温度修正	W			单浮点
		W		66			2.密度温度修正	W			单浮点
		W		67			3.密度温度修正	W			单浮点
		W		68			4.密度温度修正	W			单浮点
	 ---------------------------修正值-------------------------------
		W		69			粘度的修正	W			单浮点
		W		70			密度的修正	W			单浮点
		W		71			液位的修正	W			单浮点
		W		72			水位的修正	W			单浮点
	 ---------------------------密度的温度修正（10度）-------------------------------
		W		73			1.密度副温度修正	W			单浮点
		W		74			2.密度副温度修正	W			单浮点
		W		75			3.密度副温度修正	W			单浮点
		W		76			4.密度副温度修正	W			单浮点
	 ---------------------------密度扫频-------------------------------
		W		77			起始频率	W			单浮点
		W		78			截止频率	W			单浮点
	 ---------------------------液位扫频-------------------------------
		W		79			起始频率	W			单浮点
		W		80			截止频率	W			单浮点
	 ---------------------------测量阈值-------------------------------
		W		81			测水AD阈值	W			单浮点
		W		82			液位AD阈值	W			单浮点
		W		83			空气频率阈值	W			单浮点
		W		84			液位频率差值	W			单浮点
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
    DSM_V2_FUNC_R = 'R',
    DSM_V2_FUNC_W = 'W',
    DSM_V2_FUNC_L = 'T',
    DSM_V2_FUNC_D = 'D',
    DSM_V2_FUNC_B = 'B',
} dsm_v2_func_t;

// === 内部：求和校验（前7字节） ===
static inline uint8_t DSM_V2_CalcSum(const uint8_t f[8]) {
    uint32_t s = 0;
    for (int i = 0; i < 7; ++i) s += f[i];
    return (uint8_t)(s & 0xFF);
}

// === 内部：打帧（addr=0x00，data 大端入参；若给 0 则无所谓端序） ===
static inline void DSM_V2_MakeFrame(uint8_t out[8], uint8_t func, uint32_t data_be, uint8_t param) {
    out[0] = 0x00;
    out[1] = func;
    out[2] = (uint8_t)((data_be >> 24) & 0xFF);
    out[3] = (uint8_t)((data_be >> 16) & 0xFF);
    out[4] = (uint8_t)((data_be >>  8) & 0xFF);
    out[5] = (uint8_t)( data_be        & 0xFF);
    out[6] = param;
    out[7] = DSM_V2_CalcSum(out);
}

// === 内部：传输 8→8 ===
static int DSM_V2_Transceive(const uint8_t tx[8], uint8_t rx[8]) {
#ifdef DEBUG_DSM
    printf("V2 TX: "); for (int i=0;i<8;i++) printf("%02X ", tx[i]); printf("\r\n");
#endif
    if (HAL_UART_Transmit(&huart6, (uint8_t*)tx, 8, DSM_CMD_TIMEOUT) != HAL_OK) {
#ifdef DEBUG_DSM
        printf("V2 TX failed\r\n");
#endif
        return OTHER_PERIPHERAL_CONFIG_ERROR;
    }

    uint32_t start = HAL_GetTick();
    int got = 0;
    while ((HAL_GetTick() - start) < DSM_V2_RX_TIMEOUT && got < 8) {
        if (HAL_UART_Receive(&huart6, &rx[got], 1, 1) == HAL_OK) got++;
    }
    if (got < 8) {
#ifdef DEBUG_DSM
        printf("V2 RX timeout, got %d bytes\r\n", got);
#endif
        return SENSOR_COMM_TIMEOUT;
    }

#ifdef DEBUG_DSM
    printf("V2 RX: "); for (int i=0;i<8;i++) printf("%02X ", rx[i]); printf("\r\n");
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
    uint8_t expect_func  = tx[1] | 0x80; // 从机高位置1
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
    return (int32_t)((uint32_t)d[3]<<24 | (uint32_t)d[2]<<16 | (uint32_t)d[1]<<8 | (uint32_t)d[0]);
}
static inline float DSM_V2_ParseFloat_LE(const uint8_t *d) {
    union { uint32_t u; float f; } cvt;
    cvt.u = (uint32_t)d[3]<<24 | (uint32_t)d[2]<<16 | (uint32_t)d[1]<<8 | (uint32_t)d[0];
    return cvt.f;
}

// === 对外：切换模式（param=0x00） ===
int DSM_V2_SwitchMode(dsm_v2_mode_t mode) {
    uint8_t tx[8], rx[8];
    int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

    DSM_V2_MakeFrame(tx, (uint8_t)mode, 0x00000000u, 0x00); // param 必须为 0x00

    for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
        HAL_Delay(DSM_PRE_SEND_DELAY);
        int ret = DSM_V2_Transceive(tx, rx);
        if (ret != NO_ERROR) { last_err = ret; continue; }

        ret = DSM_V2_CheckReply(tx, rx);
        if (ret == NO_ERROR) {
#ifdef DEBUG_DSM
            printf("[V2] switch mode '%c' OK\r\n", (char)mode);
#endif
            return NO_ERROR;
        }
        last_err = ret;
        HAL_Delay(DSM_BCC_DELAY);
    }
#ifdef DEBUG_DSM
    printf("[V2] switch mode '%c' FAIL, err=%d\r\n", (char)mode, last_err);
#endif
    return last_err;
}
int DSM_V2_SwitchToLevelMode(void)   { return DSM_V2_SwitchMode(DSM_V2_MODE_LEVEL); }
int DSM_V2_SwitchToDensityMode(void) { return DSM_V2_SwitchMode(DSM_V2_MODE_DENSITY); }

// === 对外：通用读取 ===
int DSM_V2_Read_FloatParam(uint8_t param, float *out_value) {
    if (!out_value) return OTHER_PERIPHERAL_CONFIG_ERROR;

    uint8_t tx[8], rx[8];
    int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

    DSM_V2_MakeFrame(tx, (uint8_t)DSM_V2_FUNC_R, 0x00000000u, param);

    for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
        HAL_Delay(DSM_PRE_SEND_DELAY);
        int ret = DSM_V2_Transceive(tx, rx);
        if (ret != NO_ERROR) { last_err = ret; continue; }

        ret = DSM_V2_CheckReply(tx, rx);
        if (ret == NO_ERROR) {
            float v = DSM_V2_ParseFloat_LE(rx + 2);
            *out_value = v;
#ifdef DEBUG_DSM
            printf("[V2] Read Float R %u: %f\r\n", (unsigned)param, (double)v);
#endif
            return NO_ERROR;
        }
        last_err = ret;
        HAL_Delay(DSM_BCC_DELAY);
    }
    return last_err;
}

int DSM_V2_Read_IntParam(uint8_t param, int32_t *out_value) {
    if (!out_value) return OTHER_PERIPHERAL_CONFIG_ERROR;

    uint8_t tx[8], rx[8];
    int last_err = OTHER_PERIPHERAL_CONFIG_ERROR;

    DSM_V2_MakeFrame(tx, (uint8_t)DSM_V2_FUNC_R, 0x00000000u, param);

    for (int attempt = 0; attempt < DSM_V2_MAX_RETRY; ++attempt) {
        HAL_Delay(DSM_PRE_SEND_DELAY);
        int ret = DSM_V2_Transceive(tx, rx);
        if (ret != NO_ERROR) { last_err = ret; continue; }

        ret = DSM_V2_CheckReply(tx, rx);
        if (ret == NO_ERROR) {
            int32_t v = DSM_V2_ParseInt32_LE(rx + 2);
            *out_value = v;
#ifdef DEBUG_DSM
            printf("[V2] Read Int R %u: %ld (0x%08lX)\r\n",
                   (unsigned)param, (long)v, (unsigned long)v);
#endif
            return NO_ERROR;
        }
        last_err = ret;
        HAL_Delay(DSM_BCC_DELAY);
    }
    return last_err;
}

// === 便捷读取 ===
int DSM_V2_Read_SoftwareVersion(float *v)        { return DSM_V2_Read_FloatParam(0x00, v); }
int DSM_V2_Read_Temperature    (float *t)        { return DSM_V2_Read_FloatParam(0x06, t); }
int DSM_V2_Read_Density        (float *rho)      { return DSM_V2_Read_FloatParam(0x07, rho); }
int DSM_V2_Read_DynamicViscosity(float *mu)      { return DSM_V2_Read_FloatParam(0x08, mu); }
int DSM_V2_Read_KinematicViscosity(float *nu)    { return DSM_V2_Read_FloatParam(0x09, nu); }
int DSM_V2_Read_MeanSquare45   (float *msq45)    { return DSM_V2_Read_FloatParam(0x11, msq45); } // 17
int DSM_V2_Read_MeanSquare22p5 (float *msq22p5)  { return DSM_V2_Read_FloatParam(0x12, msq22p5);} // 18
// R04 液位频率（整型）
int DSM_V2_Read_LevelFrequency(uint32_t *freq_hz) {
    if (!freq_hz) return OTHER_PERIPHERAL_CONFIG_ERROR;
    int32_t v = 0;
    int ret = DSM_V2_Read_IntParam(0x04, &v);   // 参数码 0x04 = R04
    if (ret == NO_ERROR) *freq_hz = (uint32_t)v;
    return ret;
}

int DSM_V2_Read_SensorID(uint32_t *sensor_id) {
    if (!sensor_id) return OTHER_PERIPHERAL_CONFIG_ERROR;
    int32_t v = 0;
    int ret = DSM_V2_Read_IntParam(0x16, &v); // 22
    if (ret == NO_ERROR) *sensor_id = (uint32_t)v;
    return ret;
}

// R16 频率寄存器（整型）
int DSM_V2_Read_FrequencyRegister(uint32_t *freq_reg_value) {
    if (!freq_reg_value) return OTHER_PERIPHERAL_CONFIG_ERROR;
    int32_t v = 0;
    int ret = DSM_V2_Read_IntParam(0x10, &v);   // 16
    if (ret == NO_ERROR) *freq_reg_value = (uint32_t)v;
    return ret;
}

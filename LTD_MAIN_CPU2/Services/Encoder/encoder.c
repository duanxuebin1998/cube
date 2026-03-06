/*
 * encoder.c
 *
 *  Created on: Mar 21, 2025
 *      Author: Duan Xuebin
 */

#include "AS5145.h"
#include "encoder.h"
#include "mb85rs2m.h"
#include "my_crc.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

// 全局变量
volatile int32_t g_encoder_count = 0; // 编码计数值
volatile int32_t g_encoder_saved = 0;
static uint16_t prev_angle = 0;        // 上一次的角度值
static const uint16_t MAX_ANGLE = 4096; // 最大角度值 (12位分辨率)

typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t encoder_count;
    uint32_t prev_angle;
    uint32_t crc;
} EncoderPersistRecord;

#define ENCODER_STORE_MAGIC     (0x454E4344u) /* 'ENCD' */
#define ENCODER_STORE_VERSION   (1u)
#define FRAM_ENCODER_A_ADDRESS  FRAM_ANGLE_ADDRESS
#define FRAM_ENCODER_SLOT_SIZE  (0x40u)
#define FRAM_ENCODER_B_ADDRESS  (FRAM_ENCODER_A_ADDRESS + FRAM_ENCODER_SLOT_SIZE)

static uint32_t EncoderRecordCRC(const EncoderPersistRecord *record)
{
    const uint8_t *base = (const uint8_t *)&record->version;
    const uint32_t len = (uint32_t)(offsetof(EncoderPersistRecord, crc) - offsetof(EncoderPersistRecord, version));
    return CRC32_HAL(base, len);
}

static int ReadEncoderDataFromSlot(uint32_t base_addr, int32_t *encoder_count, uint16_t *angle, const char *slot_name)
{
    EncoderPersistRecord rec;
    ReadMultiData((uint8_t *)&rec, (int)base_addr, sizeof(rec));

    if (rec.magic != ENCODER_STORE_MAGIC) {
        printf("编码值[%s] magic 错误: 0x%08lX\r\n", slot_name, (unsigned long)rec.magic);
        return 0;
    }

    if (rec.version != ENCODER_STORE_VERSION) {
        printf("编码值[%s] version 错误: %lu\r\n", slot_name, (unsigned long)rec.version);
        return 0;
    }

    if (rec.prev_angle >= MAX_ANGLE) {
        printf("编码值[%s] angle 越界: %lu\r\n", slot_name, (unsigned long)rec.prev_angle);
        return 0;
    }

    if (EncoderRecordCRC(&rec) != rec.crc) {
        printf("编码值[%s] CRC 校验失败\r\n", slot_name);
        return 0;
    }

    *encoder_count = rec.encoder_count;
    *angle = (uint16_t)rec.prev_angle;
    return 1;
}

static void WriteEncoderDataAB(int32_t encoder_count, uint16_t angle)
{
    EncoderPersistRecord rec;
    rec.magic = ENCODER_STORE_MAGIC;
    rec.version = ENCODER_STORE_VERSION;
    rec.encoder_count = encoder_count;
    rec.prev_angle = (uint32_t)angle;
    rec.crc = EncoderRecordCRC(&rec);

    WriteMultiData((const uint8_t *)&rec, (int)FRAM_ENCODER_A_ADDRESS, sizeof(rec));
    WriteMultiData((const uint8_t *)&rec, (int)FRAM_ENCODER_B_ADDRESS, sizeof(rec));
}

static int ReadEncoderDataAB(int32_t *encoder_count, uint16_t *angle)
{
    if (ReadEncoderDataFromSlot(FRAM_ENCODER_A_ADDRESS, encoder_count, angle, "A")) {
        return 1;
    }

    if (ReadEncoderDataFromSlot(FRAM_ENCODER_B_ADDRESS, encoder_count, angle, "B")) {
        printf("编码值 A 分区异常, 已回退到 B 分区\r\n");
        WriteEncoderDataAB(*encoder_count, *angle);
        return 1;
    }

    return 0;
}

// 在系统启动时初始化编码计数器
void Initialize_Encoder(void)
{
    int32_t loaded_encoder = 0;
    uint16_t loaded_angle = 0;

    if (ReadEncoderDataAB(&loaded_encoder, &loaded_angle)) {
        g_encoder_count = loaded_encoder;
        prev_angle = loaded_angle;
    } else {
        g_encoder_count = 0;
        prev_angle = 0;
        g_measurement.device_status.error_code = ENCODER_POWERON_FAIL;
        printf("编码值 A/B 分区均异常，已回退默认值\r\n");
        WriteEncoderDataAB(g_encoder_count, prev_angle);
    }

    update_sensor_height_from_encoder();
    g_encoder_saved = g_encoder_count; // 保存初始编码计数值

    Start_Encoder_Collection_TIM(); // 启动编码器定时采集
}

// 更新编码计数
void Update_Encoder_Count(uint16_t current_angle)
{
    int16_t delta = current_angle - prev_angle;

    // 处理跨零点情况
    if (delta > (MAX_ANGLE / 2)) {
        delta -= MAX_ANGLE;
    } else if (delta < -(MAX_ANGLE / 2)) {
        delta += MAX_ANGLE;
    }

    g_encoder_count += delta;
    prev_angle = current_angle; // 更新上一角度值

    if ((abs((int)g_encoder_count - (int)g_encoder_saved) > 3) || (g_measurement.debug_data.sensor_position == 0)) {
        WriteEncoderDataAB(g_encoder_count, prev_angle);
        g_encoder_saved = g_encoder_count;
        update_sensor_height_from_encoder();
    }
}

/**
 * 根据编码器脉冲值更新传感器高度测量
 * @param encoder_value 编码器原始计数值
 *
 * 功能说明：
 * 1. 将编码器值取反
 * 2. 计算当前位置高度
 * 3. 更新测量系统中的高度和编码值
 */
void update_sensor_height_from_encoder(void)
{
    // 更新调试信息：当前编码值取负（编码器方向取反）
    g_measurement.debug_data.current_encoder_value = -g_encoder_count;
    // 1. 计算转动的总圈数（含小数）
    float revolutions = (float)g_measurement.debug_data.current_encoder_value / (float)MAX_ANGLE;
    // 2. 计算尺带的收放长度
    float cable_length = g_deviceParams.encoder_wheel_circumference_mm * revolutions / 100;
    // 3. 计算当前传感器高度（油罐高度减去悬吊长度）
    float current_height = g_deviceParams.tankHeight - cable_length;
    // 更新调试数据（高度值）
    g_measurement.debug_data.cable_length = (int)cable_length;
    g_measurement.debug_data.sensor_position = (int)current_height;
}

// 设置编码器零点
void set_encoder_zero(void)
{
    printf("设置编码器零点，当前置零点前零点编码值 %ld\r\n", (long)g_encoder_count);
    g_encoder_count = 0;
    g_encoder_saved = 0;
    WriteEncoderDataAB(g_encoder_count, prev_angle);
    update_sensor_height_from_encoder();
    printf("编码器零点设置为 %ld\r\n", (long)g_encoder_count);
}
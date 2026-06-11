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
#include "motor_ctrl.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>

/* 全局变量 */
volatile int32_t g_encoder_count = 0; /* 编码计数值 */
volatile int32_t g_encoder_saved = 0; /* 本模块模块级变量，保存跨函数共享的业务状态。 */
static uint16_t prev_angle = 0;        /* 上一次的角度值 */
static const uint16_t MAX_ANGLE = 4096; /* 最大角度值 (12位分辨率) */

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
#define ENCODER_BOOT_READY_TIMEOUT_MS 300U

/**
 * @brief 计算校验本模块中的 EncoderRecordCRC 逻辑。
 *
 * @param record 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t EncoderRecordCRC(const EncoderPersistRecord *record)
{
    const uint8_t *base = (const uint8_t *)&record->version;
    const uint32_t len = (uint32_t)(offsetof(EncoderPersistRecord, crc) - offsetof(EncoderPersistRecord, version));
    return CRC32_HAL(base, len);
}

/**
 * @brief 读取本模块中的 ReadEncoderDataFromSlot 逻辑。
 *
 * @param base_addr 地址参数。
 * @param encoder_count 业务参数。
 * @param angle 业务参数。
 * @param slot_name 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static int ReadEncoderDataFromSlot(uint32_t base_addr, int32_t *encoder_count, uint16_t *angle, const char *slot_name)
{
    EncoderPersistRecord rec;
    ReadMultiData((uint8_t *)&rec, (int)base_addr, sizeof(rec));

    if (rec.magic != ENCODER_STORE_MAGIC) {
        printf("编码值[%s] magic 异常: 0x%08lX\r\n", slot_name, (unsigned long)rec.magic);
        return 0;
    }

    if (rec.version != ENCODER_STORE_VERSION) {
        printf("编码值[%s] version 异常: %lu\r\n", slot_name, (unsigned long)rec.version);
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

/**
 * @brief 写入或设置本模块中的 WriteEncoderDataAB 逻辑。
 *
 * @param encoder_count 业务参数。
 * @param angle 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
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

/**
 * @brief 读取本模块中的 ReadEncoderDataAB 逻辑。
 *
 * @param encoder_count 业务参数。
 * @param angle 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
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

/**
 * @brief 判断编码器是否已有可信首帧位置。
 *
 * 该函数不访问 SPI，只读取 AS5145 采集链路的有效帧锁存状态。
 */
bool Encoder_IsReady(void)
{
    return AS5145_HasValidSample();
}

/**
 * @brief 等待编码器首帧有效位置。
 *
 * 启动流程使用该函数缩短位置盲区；函数会阻塞等待，不能在中断上下文调用。
 */
uint32_t Encoder_WaitReady(uint32_t timeout_ms)
{
    return AS5145_WaitFirstValidSample(timeout_ms);
}
/* 在系统启动时初始化编码计数器 */
void Initialize_Encoder(void)
{
    int32_t loaded_encoder = 0;
    uint16_t loaded_angle = 0;
    HAL_StatusTypeDef start_status;
    uint32_t ready_ret;

    if (ReadEncoderDataAB(&loaded_encoder, &loaded_angle)) {
        g_encoder_count = loaded_encoder;
        prev_angle = loaded_angle;
    } else {
        g_encoder_count = 0;
        prev_angle = 0;
        g_measurement.device_status.error_code = ENCODER_POWERON_FAIL;
        printf("编码器持久化 A/B 分区均异常，已回退默认值\r\n");
        WriteEncoderDataAB(g_encoder_count, prev_angle);
    }

    update_sensor_height_from_encoder();
    g_encoder_saved = g_encoder_count;

    start_status = Start_Encoder_Collection_TIM();
    if (start_status != HAL_OK) {
        if (!MotorCtrl_IsPositionSourceMotor()) {
            g_measurement.device_status.error_code = ENCODER_TIMEOUT;
        }
        return;
    }

    /* 编码轮记步模式必须等首帧有效位置，否则重启后立即运动会丢失盲区位移。 */
    ready_ret = Encoder_WaitReady(ENCODER_BOOT_READY_TIMEOUT_MS);
    /* 先处理异常边界，避免本模块状态机带故障继续运行。 */
    if ((ready_ret != NO_ERROR) && (!MotorCtrl_IsPositionSourceMotor())) {
        g_measurement.device_status.error_code = ready_ret;
        printf("编码器首帧有效数据等待失败：0x%08lX\r\n", (unsigned long)ready_ret);
    }
}
/* 更新编码计数 */
void Update_Encoder_Count(uint16_t current_angle)
{
    int16_t delta = current_angle - prev_angle;

    /* 处理跨零点情况 */
    if (delta > (MAX_ANGLE / 2)) {
        delta -= MAX_ANGLE;
    } else if (delta < -(MAX_ANGLE / 2)) {
        delta += MAX_ANGLE;
    }

    g_encoder_count += delta;
    prev_angle = current_angle; /* 更新上一角度值 */

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
static void update_sensor_height_from_encoder_impl(bool force_position_update)
{
    float revolutions;
    float cable_length;
    float current_height;

    /* 更新调试信息：当前编码值取负（编码器方向取反） */
    g_measurement.debug_data.current_encoder_value = -g_encoder_count;
    if ((!force_position_update) && MotorCtrl_IsPositionSourceMotor()) {
        return;
    }

    /* 1. 计算转动的总圈数（含小数） */
    revolutions = (float)g_measurement.debug_data.current_encoder_value / (float)MAX_ANGLE;
    /* 2. 计算尺带的收放长度 */
    cable_length = g_deviceParams.encoder_wheel_circumference_mm * revolutions / 100;
    /* 3. 计算当前传感器高度（油罐高度减去悬吊长度） */
    current_height = g_deviceParams.tankHeight - cable_length;
    /* 更新调试数据（高度值） */
    g_measurement.debug_data.cable_length = (int)cable_length;
    g_measurement.debug_data.sensor_position = (int)current_height;
}

/**
 * @brief 按编码轮当前计数更新尺带长度和传感器高度调试数据。
 * @note 使用普通更新路径，不强制覆盖已有高度来源。
 */
void update_sensor_height_from_encoder(void)
{
    update_sensor_height_from_encoder_impl(false);
}

/**
 * @brief 更新本模块中的 update_sensor_height_from_encoder_force 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void update_sensor_height_from_encoder_force(void)
{
    update_sensor_height_from_encoder_impl(true);
}

/* 设置编码器零点 */

int32_t encoder_get_cable_length_01mm(void)
{
    const float encoder_value = (float)(-g_encoder_count);
    const float revolutions = encoder_value / (float)MAX_ANGLE;
    const float cable_length = g_deviceParams.encoder_wheel_circumference_mm * revolutions / 100.0f;

    return (int32_t)cable_length;
}

/**
 * @brief 执行本模块中的 encoder_get_sensor_position_01mm 逻辑。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
int32_t encoder_get_sensor_position_01mm(void)
{
    return (int32_t)g_deviceParams.tankHeight - encoder_get_cable_length_01mm();
}

/**
 * @brief 写入或设置本模块中的 set_encoder_zero 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void set_encoder_zero(void)
{
    printf("设置编码器零点，当前置零点前零点编码值 %ld\r\n", (long)g_encoder_count);
    g_encoder_count = 0;
    g_encoder_saved = 0;
    WriteEncoderDataAB(g_encoder_count, prev_angle);
    update_sensor_height_from_encoder();
    printf("编码器零点设置为 %ld\r\n", (long)g_encoder_count);
}
/**
 * @brief 执行本模块中的 encoder_set_cable_length_01mm 逻辑。
 *
 * @param cable_length_01mm 数据长度。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void encoder_set_cable_length_01mm(int32_t cable_length_01mm)
{
    double encoder_value;
    int32_t new_count;

    if (cable_length_01mm < 0) {
        cable_length_01mm = 0;
    }

    if (g_deviceParams.encoder_wheel_circumference_mm == 0U) {
        printf("编码器修正失败：编码轮周长为0\r\n");
        return;
    }

    /* cable_length 单位为0.1mm，encoder_wheel_circumference_mm 单位为0.001mm。 */
    encoder_value = ((double)cable_length_01mm * 100.0 * (double)MAX_ANGLE) /
                    (double)g_deviceParams.encoder_wheel_circumference_mm;
    new_count = -(int32_t)(encoder_value + 0.5);

    printf("编码器修正 | 目标尺带长度：%ld(0.1mm) | 原计数=%ld | 新计数=%ld\r\n",
           (long)cable_length_01mm,
           (long)g_encoder_count,
           (long)new_count);

    g_encoder_count = new_count;
    g_encoder_saved = new_count;
    WriteEncoderDataAB(g_encoder_count, prev_angle);
    update_sensor_height_from_encoder();
}

#ifndef sil__MB85RS16_H
#define sil__MB85RS16_H
#include "main.h"

#define FRAM_CS_Pin GPIO_PIN_11 /* FRAM 片选 GPIO 引脚。 */
#define FRAM_CS_GPIO_Port GPIOE /* FRAM 片选 GPIO 端口。 */
#define FRAM_SPI hspi4 /* FRAM 使用的 SPI 句柄。 */

/* MB85RS2M SPI操作码：写使能、写数据和读数据。 */
#define MB_WRITEENABLE 0x06
#define MB_WRITEDATA  0x02
#define MB_READDATA   0x03

/* MB85RS2M总容量256KiB，用于统一地址与长度越界检查。 */
#define FRAM_CAPACITY_BYTES 0x40000u

extern SPI_HandleTypeDef FRAM_SPI;

/* 统一FRAM事务结果；调用方必须区分参数错误、仲裁忙、超时和硬件错误。 */
typedef enum {
    FRAM_STATUS_OK = 0,          /* 完整SPI事务成功。 */
    FRAM_STATUS_INVALID_ARGUMENT,/* 空指针、零长度或超过HAL长度上限。 */
    FRAM_STATUS_OUT_OF_RANGE,    /* 地址加长度越过256KiB器件边界。 */
    FRAM_STATUS_BUSY,            /* 已有事务、紧急预约或普通写禁止。 */
    FRAM_STATUS_TIMEOUT,         /* 某个HAL SPI阶段超过5ms。 */
    FRAM_STATUS_HAL_ERROR        /* HAL返回其它SPI错误。 */
} FRAM_Status;

/* 历史编码器数据起始地址；V3 A槽继续沿用该地址保证迁移兼容。 */
#define FRAM_ANGLE_ADDRESS 0x1000
#define FRAM_ENCODER_ADDRESS 0x1004

/*
 * 函数用途：通过统一事务入口写入 FRAM。
 * 调用场景：参数、编码器和电机位置等所有生产调用路径。
 * 关键约束：有限超时、地址校验、互斥访问，所有出口都会释放片选和所有权。
 */
FRAM_Status FRAM_Write(const uint8_t *data, uint32_t address, uint32_t length);

/*
 * 函数用途：通过统一事务入口读取 FRAM。
 * 调用场景：参数、编码器和电机位置等所有生产调用路径。
 * 关键约束：有限超时、地址校验、互斥访问，所有出口都会释放片选和所有权。
 */
FRAM_Status FRAM_Read(uint8_t *data, uint32_t address, uint32_t length);

/*
 * 函数用途：在掉电窗口预留FRAM并标记紧急提交所有者。
 * 调用场景：ADC看门狗ISR、编码器PendSV提交和电压稳定恢复。
 * 关键约束：不抢断已开始事务，普通调用者在预留期间立即返回BUSY。
 */
void FRAM_RequestEmergencyReservationFromISR(void); /* 阻止新的普通事务进入。 */
void FRAM_EnterEmergencyOwner(void);                /* 允许紧急保存越过预约和写禁止。 */
void FRAM_ExitEmergencyOwner(void);                 /* 紧急事务结束，仍保留预约状态。 */
void FRAM_ReleaseEmergencyReservation(void);        /* 电压稳定且紧急请求结束后释放预约。 */
bool FRAM_IsEmergencyReserved(void);                /* 参数延后保存据此选择继续等待。 */
void FRAM_SetNormalWriteInhibitedFromISR(bool inhibited); /* 电源不可信时禁止普通写。 */

/*
 * 兼容旧调用接口。新代码必须使用带返回状态的 FRAM_Read/FRAM_Write。
 */
void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length);
void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length);
uint32_t ReadSingleData(uint32_t address);
void WriteSingleData(uint32_t data, uint32_t address);
void WriteTwoData(int steps, int circle, int address);
void Test_FRAM_ReadWrite(void);

#endif
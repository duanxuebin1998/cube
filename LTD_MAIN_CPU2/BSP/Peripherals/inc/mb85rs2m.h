#ifndef sil__MB85RS16_H
/* sil__MB85RS16_H 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define sil__MB85RS16_H
#include "main.h"

#define FRAM_CS_Pin GPIO_PIN_11 /* FRAM 片选 GPIO 引脚。 */
#define FRAM_CS_GPIO_Port GPIOE /* FRAM 片选 GPIO 端口。 */
#define FRAM_SPI hspi4 /* FRAM 使用的 SPI 句柄。 */

/* MB85RS2M SPI操作码：写使能、写数据和读数据。 */
#define MB_WRITEENABLE 0x06
/* MB85RS2M FRAM 写数据指令 0x02；发送地址后连续写入数据字节。 */
#define MB_WRITEDATA  0x02
/* MB85RS2M FRAM 读数据指令 0x03；发送地址后连续读取数据字节。 */
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
/* 历史编码器持久化区的 FRAM 起始地址 0x1004；新 A/B 记录布局仍需兼容从该旧地址迁移。 */
#define FRAM_ENCODER_ADDRESS 0x1004

/**
 * @brief 通过统一事务入口写入 FRAM。
 *
 * @details 调用场景：参数、编码器和电机位置等所有生产调用路径。
 * @note 关键约束：有限超时、地址校验、互斥访问，所有出口都会释放片选和所有权。
 */
FRAM_Status FRAM_Write(const uint8_t *data, uint32_t address, uint32_t length);

/**
 * @brief 通过统一事务入口读取 FRAM。
 *
 * @details 调用场景：参数、编码器和电机位置等所有生产调用路径。
 * @note 关键约束：有限超时、地址校验、互斥访问，所有出口都会释放片选和所有权。
 */
FRAM_Status FRAM_Read(uint8_t *data, uint32_t address, uint32_t length);

/**
 * @brief 在掉电窗口预留FRAM并标记紧急提交所有者。
 *
 * @details 调用场景：ADC看门狗ISR、编码器PendSV提交和电压稳定恢复。
 * @note 关键约束：不抢断已开始事务，普通调用者在预留期间立即返回BUSY。
 */
void FRAM_RequestEmergencyReservationFromISR(void); /* 阻止新的普通事务进入。 */
/**
 * @brief PendSV在执行紧急A/B与回执事务前声明所有者，使其可越过普通写门禁。
 */
void FRAM_EnterEmergencyOwner(void);                /* 允许紧急保存越过预约和写禁止。 */
/**
 * @brief 结束紧急所有者身份；预约仍保留到电压稳定且紧急请求完全结束。
 */
void FRAM_ExitEmergencyOwner(void);                 /* 紧急事务结束，仍保留预约状态。 */
/**
 * @brief 释放掉电预约，允许普通读写重新参与无等待仲裁。
 */
void FRAM_ReleaseEmergencyReservation(void);        /* 电压稳定且紧急请求结束后释放预约。 */
/**
 * @brief 查询紧急掉电流程是否已经预约 FRAM。
 *
 * @details 调用场景：主循环参数延后保存决定是否继续等待。
 * @note 关键约束：只读取单字节状态，不等待、不打印。
 *
 * @return true 表示紧急掉电流程已预约 FRAM，普通写入应停止占用；false 表示当前没有紧急预约。
 */
bool FRAM_IsEmergencyReserved(void);                /* 参数延后保存据此选择继续等待。 */
/**
 * @brief 在电源不可靠时禁止普通 FRAM 写入。
 *
 * @details 调用场景：24V 监控启动、低压、DMA 故障和恢复边界。
 * @note 关键约束：紧急所有者仍可写入掉电记录；FRAM 读取不受影响。
 *
 * @param inhibited true 表示禁止普通 FRAM 写事务，false 表示解除该禁止。
 */
void FRAM_SetNormalWriteInhibitedFromISR(bool inhibited); /* 电源不可信时禁止普通写。 */

/**
 * @brief 旧兼容接口：从绝对 FRAM 字节地址写入数据，负地址直接返回且不传播底层状态。
 *
 * 兼容旧调用接口。新代码必须使用带返回状态的 FRAM_Read/FRAM_Write。
 *
 * @param p_array 待连续写入 FRAM 的源字节数组。
 * @param startcnt FRAM 连续读写的起始字节地址。
 * @param length 输入数据的有效长度，单位字节。
 */
void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length);
/**
 * @brief 旧兼容接口：从绝对 FRAM 字节地址读取数据，负地址直接返回且不传播底层状态。
 *
 * @param p_array 用于接收 FRAM 连续读取结果的目标字节数组。
 * @param startcnt FRAM 连续读写的起始字节地址。
 * @param length 输入数据的有效长度，单位字节。
 */
void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length);
/**
 * @brief 使用 24 位绝对字节地址，按大端字节序从 FRAM 读取 32 位数；读取失败与真实零值当前无法区分。
 *
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 * @return 返回从指定 FRAM 绝对地址按大端顺序组合的 32 位原始值。
 */
uint32_t ReadSingleData(uint32_t address);
/**
 * @brief 使用 24 位绝对字节地址，按大端字节序向 FRAM 写入一个 32 位数；底层失败不向调用方返回。
 *
 * @param data 准备写入 CPU2 FRAM 的 32 位原始值；函数按固定字节顺序从 address 开始写入四个连续字节。
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 */
void WriteSingleData(uint32_t data, uint32_t address);
/**
 * @brief 从 address×4 字节地址起，按大端序连续写入 steps 和 circle。
 *
 * @param steps 与卷绕圈数一同保存的编码器累计步数。
 * @param circle 与编码器位置一同保存的卷绕圈数。
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 */
void WriteTwoData(int steps, int circle, int address);
/**
 * @brief 破坏性 FRAM 维护测试：会覆盖固定测试地址并打印回读结果，禁止在生产流程调用。
 */
void Test_FRAM_ReadWrite(void);

#endif
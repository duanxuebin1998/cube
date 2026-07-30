#include <mb85rs2m.h>
#include <stdio.h>
#include "usart.h"

/* 单个SPI命令、地址或数据阶段的最大阻塞时间，单位ms，禁止使用HAL_MAX_DELAY。 */
#define FRAM_SPI_TIMEOUT_MS 5U

/* 仲裁状态会被ADC/SysTick、PendSV和主循环共同访问，复合变更使用短临界区。 */
static volatile uint8_t s_fram_transaction_active = 0U; /* 1表示已有读写事务持有SPI4。 */
static volatile uint8_t s_fram_emergency_reserved = 0U; /* 1表示掉电保存已预约后续事务。 */
static volatile uint8_t s_fram_emergency_owner = 0U; /* 1表示当前调用属于编码器紧急保存。 */
static volatile uint8_t s_fram_normal_write_inhibited = 0U; /* 1表示电源不可信，拒绝普通写。 */

/**
 * @brief 把 HAL SPI 状态转换为 FRAM 统一状态。
 *
 * @details 调用场景：FRAM 事务的每一个命令、地址和数据阶段。
 * @note 关键约束：不打印、不递归访问 FRAM。
 *
 * @param status 底层 HAL 外设访问状态；函数把 HAL_OK、HAL_ERROR、HAL_BUSY 和 HAL_TIMEOUT映射为模块自己的状态或诊断结果。
 * @return 返回 FRAM 事务状态；FRAM_STATUS_OK 表示操作完成，其他值区分参数非法、地址越界、资源忙、阶段超时和 HAL SPI 错误。
 */
static FRAM_Status FRAM_MapHalStatus(HAL_StatusTypeDef status)
{
    if (status == HAL_OK) {
        return FRAM_STATUS_OK;
    }
    if (status == HAL_BUSY) {
        return FRAM_STATUS_BUSY;
    }
    if (status == HAL_TIMEOUT) {
        return FRAM_STATUS_TIMEOUT;
    }
    return FRAM_STATUS_HAL_ERROR;
}

/**
 * @brief 在短临界区内取得唯一 FRAM 事务所有权。
 *
 * @details 调用场景：FRAM_Read 和 FRAM_Write 进入硬件事务之前。
 * @note 关键约束：不等待；已有事务时立即返回 BUSY，避免中断上下文死锁。
 *
 * @return 返回 FRAM 事务状态；FRAM_STATUS_OK 表示操作完成，其他值区分参数非法、地址越界、资源忙、阶段超时和 HAL SPI 错误。
 */
static FRAM_Status FRAM_TryLock(void)
{
    uint32_t primask = __get_PRIMASK();
    FRAM_Status result = FRAM_STATUS_BUSY;

    __disable_irq();
    if (((s_fram_emergency_reserved == 0U) ||
         (s_fram_emergency_owner != 0U)) &&
        (s_fram_transaction_active == 0U)) {
        s_fram_transaction_active = 1U;
        __DMB();
        result = FRAM_STATUS_OK;
    }
    if (primask == 0U) {
        __enable_irq();
    }
    return result;
}

/**
 * @brief 释放 FRAM 事务所有权。
 *
 * @details 调用场景：FRAM 事务统一清理出口。
 * @note 关键约束：先恢复片选为高，再允许其它调用者进入。
 */
static void FRAM_Unlock(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    __DMB();
    s_fram_transaction_active = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief 为掉电编码器提交预留后续FRAM事务。
 *
 * @details 调用场景：24V模拟看门狗中断。
 * @note 关键约束：不打断已经开始的事务；预留后普通新事务立即返回BUSY。
 */
void FRAM_RequestEmergencyReservationFromISR(void)
{
    s_fram_emergency_reserved = 1U;
    __DMB();
}

/**
 * @brief PendSV在执行紧急A/B与回执事务前声明所有者，使其可越过普通写门禁。
 */
void FRAM_EnterEmergencyOwner(void)
{
    s_fram_emergency_owner = 1U;
    __DMB();
}

/**
 * @brief 结束紧急所有者身份；预约仍保留到电压稳定且紧急请求完全结束。
 */
void FRAM_ExitEmergencyOwner(void)
{
    __DMB();
    s_fram_emergency_owner = 0U;
}

/**
 * @brief 释放掉电预约，允许普通读写重新参与无等待仲裁。
 */
void FRAM_ReleaseEmergencyReservation(void)
{
    __DMB();
    s_fram_emergency_reserved = 0U;
}

/**
 * @brief 查询紧急掉电流程是否已经预约 FRAM。
 *
 * @details 调用场景：主循环参数延后保存决定是否继续等待。
 * @note 关键约束：只读取单字节状态，不等待、不打印。
 *
 * @return true 表示紧急掉电流程已预约 FRAM，普通写入应停止占用；false 表示当前没有紧急预约。
 */
bool FRAM_IsEmergencyReserved(void)
{
    return s_fram_emergency_reserved != 0U;
}

/**
 * @brief 在电源不可靠时禁止普通 FRAM 写入。
 *
 * @details 调用场景：24V 监控启动、低压、DMA 故障和恢复边界。
 * @note 关键约束：紧急所有者仍可写入掉电记录；FRAM 读取不受影响。
 *
 * @param inhibited true 表示禁止普通 FRAM 写事务，false 表示解除该禁止。
 */
void FRAM_SetNormalWriteInhibitedFromISR(bool inhibited)
{
    s_fram_normal_write_inhibited = inhibited ? 1U : 0U;
    __DMB();
}

/**
 * @brief 在触碰片选前校验指针、HAL 16bit长度限制及256KiB地址边界。
 *
 * @param data 待校验的读写缓冲区首地址；length 大于 0 时必须非 NULL，本函数不访问缓冲区内容。
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 * @param length 输入数据的有效长度，单位字节。
 * @return 返回 FRAM 事务状态；FRAM_STATUS_OK 表示操作完成，其他值区分参数非法、地址越界、资源忙、阶段超时和 HAL SPI 错误。
 */
static FRAM_Status FRAM_ValidateRange(const void *data, uint32_t address, uint32_t length)
{
    if ((data == NULL) || (length == 0U) || (length > UINT16_MAX)) {
        return FRAM_STATUS_INVALID_ARGUMENT;
    }
    if ((address >= FRAM_CAPACITY_BYTES) || (length > (FRAM_CAPACITY_BYTES - address))) {
        return FRAM_STATUS_OUT_OF_RANGE;
    }
    return FRAM_STATUS_OK;
}

/**
 * @brief 已持有事务锁时发送WREN；无论HAL结果如何都在返回前释放片选。
 *
 * @return 返回 FRAM 事务状态；FRAM_STATUS_OK 表示操作完成，其他值区分参数非法、地址越界、资源忙、阶段超时和 HAL SPI 错误。
 */
static FRAM_Status FRAM_WriteEnableLocked(void)
{
    uint8_t command = MB_WRITEENABLE;
    FRAM_Status result;

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
    result = FRAM_MapHalStatus(HAL_SPI_Transmit(&FRAM_SPI,
                                               &command,
                                               1U,
                                               FRAM_SPI_TIMEOUT_MS));
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    return result;
}

/**
 * @brief 统一写事务：参数校验、双重普通写门禁、无等待加锁、WREN、24bit地址和数据发送。
 *
 * 所有硬件阶段共用cleanup出口恢复片选并释放事务锁。
 *
 * @param data 准备写入 SPI4 FRAM 的连续只读字节序列；有效范围为 data[0..length-1]。
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 * @param length 输入数据的有效长度，单位字节。
 * @return 返回 FRAM 事务状态；FRAM_STATUS_OK 表示操作完成，其他值区分参数非法、地址越界、资源忙、阶段超时和 HAL SPI 错误。
 */
FRAM_Status FRAM_Write(const uint8_t *data, uint32_t address, uint32_t length)
{
    uint8_t command = MB_WRITEDATA;
    uint8_t address_bytes[3];
    FRAM_Status result;

    result = FRAM_ValidateRange(data, address, length);
    if (result != FRAM_STATUS_OK) {
        return result;
    }
    if ((s_fram_normal_write_inhibited != 0U) &&
        (s_fram_emergency_owner == 0U)) {
        return FRAM_STATUS_BUSY;
    }

    result = FRAM_TryLock();
    if (result != FRAM_STATUS_OK) {
        return result;
    }
    /*
     * 取得事务所有权后再次检查，关闭 ADC/DMA 故障在首次检查与加锁之间到达的竞态。
     */
    if ((s_fram_normal_write_inhibited != 0U) &&
        (s_fram_emergency_owner == 0U)) {
        FRAM_Unlock();
        return FRAM_STATUS_BUSY;
    }

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    result = FRAM_WriteEnableLocked();
    if (result != FRAM_STATUS_OK) {
        goto cleanup;
    }

    address_bytes[0] = (uint8_t)((address >> 16) & 0xFFU);
    address_bytes[1] = (uint8_t)((address >> 8) & 0xFFU);
    address_bytes[2] = (uint8_t)(address & 0xFFU);

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
    result = FRAM_MapHalStatus(HAL_SPI_Transmit(&FRAM_SPI,
                                               &command,
                                               1U,
                                               FRAM_SPI_TIMEOUT_MS));
    if (result != FRAM_STATUS_OK) {
        goto cleanup;
    }
    result = FRAM_MapHalStatus(HAL_SPI_Transmit(&FRAM_SPI,
                                               address_bytes,
                                               3U,
                                               FRAM_SPI_TIMEOUT_MS));
    if (result != FRAM_STATUS_OK) {
        goto cleanup;
    }
    result = FRAM_MapHalStatus(HAL_SPI_Transmit(&FRAM_SPI,
                                               (uint8_t *)data,
                                               (uint16_t)length,
                                               FRAM_SPI_TIMEOUT_MS));

cleanup:
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    FRAM_Unlock();
    return result;
}

/**
 * @brief 统一读事务：读取不受普通写禁止影响，但仍必须遵守紧急预约和唯一事务锁。
 *
 * 任一SPI阶段失败均从cleanup恢复片选并释放所有权。
 *
 * @param data FRAM 连续读取的输出缓冲区；成功时写入 address 起始的 length 个字节。
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 * @param length 输入数据的有效长度，单位字节。
 * @return 返回 FRAM 事务状态；FRAM_STATUS_OK 表示操作完成，其他值区分参数非法、地址越界、资源忙、阶段超时和 HAL SPI 错误。
 */
FRAM_Status FRAM_Read(uint8_t *data, uint32_t address, uint32_t length)
{
    uint8_t command = MB_READDATA;
    uint8_t address_bytes[3];
    FRAM_Status result;

    result = FRAM_ValidateRange(data, address, length);
    if (result != FRAM_STATUS_OK) {
        return result;
    }

    result = FRAM_TryLock();
    if (result != FRAM_STATUS_OK) {
        return result;
    }

    address_bytes[0] = (uint8_t)((address >> 16) & 0xFFU);
    address_bytes[1] = (uint8_t)((address >> 8) & 0xFFU);
    address_bytes[2] = (uint8_t)(address & 0xFFU);

    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
    result = FRAM_MapHalStatus(HAL_SPI_Transmit(&FRAM_SPI,
                                               &command,
                                               1U,
                                               FRAM_SPI_TIMEOUT_MS));
    if (result != FRAM_STATUS_OK) {
        goto cleanup;
    }
    result = FRAM_MapHalStatus(HAL_SPI_Transmit(&FRAM_SPI,
                                               address_bytes,
                                               3U,
                                               FRAM_SPI_TIMEOUT_MS));
    if (result != FRAM_STATUS_OK) {
        goto cleanup;
    }
    result = FRAM_MapHalStatus(HAL_SPI_Receive(&FRAM_SPI,
                                              data,
                                              (uint16_t)length,
                                              FRAM_SPI_TIMEOUT_MS));

cleanup:
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
    FRAM_Unlock();
    return result;
}

/**
 * @brief 旧兼容接口：从绝对 FRAM 字节地址写入数据，负地址直接返回且不传播底层状态。
 *
 * @param p_array 待连续写入 FRAM 的源字节数组。
 * @param startcnt FRAM 连续读写的起始字节地址。
 * @param length 输入数据的有效长度，单位字节。
 */
void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length)
{
    if (startcnt < 0) {
        return;
    }
    (void)FRAM_Write(p_array, (uint32_t)startcnt, length);
}

/**
 * @brief 旧兼容接口：从绝对 FRAM 字节地址读取数据，负地址直接返回且不传播底层状态。
 *
 * @param p_array 用于接收 FRAM 连续读取结果的目标字节数组。
 * @param startcnt FRAM 连续读写的起始字节地址。
 * @param length 输入数据的有效长度，单位字节。
 */
void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length)
{
    if (startcnt < 0) {
        return;
    }
    (void)FRAM_Read(p_array, (uint32_t)startcnt, length);
}

/**
 * @brief 使用 24 位绝对字节地址，按大端字节序向 FRAM 写入一个 32 位数；底层失败不向调用方返回。
 *
 * @param data 准备写入 CPU2 FRAM 的 32 位原始值；函数按固定字节顺序从 address 开始写入四个连续字节。
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 */
void WriteSingleData(uint32_t data, uint32_t address)
{
    uint8_t bytes[4];

    bytes[0] = (uint8_t)((data >> 24) & 0xFFU);
    bytes[1] = (uint8_t)((data >> 16) & 0xFFU);
    bytes[2] = (uint8_t)((data >> 8) & 0xFFU);
    bytes[3] = (uint8_t)(data & 0xFFU);
    (void)FRAM_Write(bytes, address, sizeof(bytes));
}

/**
 * @brief 使用 24 位绝对字节地址，按大端字节序从 FRAM 读取 32 位数；读取失败与真实零值当前无法区分。
 *
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 * @return 返回从指定 FRAM 绝对地址按大端顺序组合的 32 位原始值。
 */
uint32_t ReadSingleData(uint32_t address)
{
    uint8_t bytes[4] = {0U};

    if (FRAM_Read(bytes, address, sizeof(bytes)) != FRAM_STATUS_OK) {
        return 0U;
    }
    return ((uint32_t)bytes[0] << 24) |
           ((uint32_t)bytes[1] << 16) |
           ((uint32_t)bytes[2] << 8) |
           (uint32_t)bytes[3];
}

/**
 * @brief 从 address×4 字节地址起，按大端序连续写入 steps 和 circle。
 *
 * @param steps 与卷绕圈数一同保存的编码器累计步数。
 * @param circle 与编码器位置一同保存的卷绕圈数。
 * @param address SPI4 FRAM 的绝对字节地址；函数先结合 length 检查容量边界，再从该地址连续读写。
 */
void WriteTwoData(int steps, int circle, int address)
{
    uint8_t bytes[8];
    uint32_t steps_value = (uint32_t)steps;
    uint32_t circle_value = (uint32_t)circle;

    if (address < 0) {
        return;
    }
    bytes[0] = (uint8_t)((steps_value >> 24) & 0xFFU);
    bytes[1] = (uint8_t)((steps_value >> 16) & 0xFFU);
    bytes[2] = (uint8_t)((steps_value >> 8) & 0xFFU);
    bytes[3] = (uint8_t)(steps_value & 0xFFU);
    bytes[4] = (uint8_t)((circle_value >> 24) & 0xFFU);
    bytes[5] = (uint8_t)((circle_value >> 16) & 0xFFU);
    bytes[6] = (uint8_t)((circle_value >> 8) & 0xFFU);
    bytes[7] = (uint8_t)(circle_value & 0xFFU);
    (void)FRAM_Write(bytes, (uint32_t)address * 4U, sizeof(bytes));
}

/**
 * @brief 破坏性 FRAM 维护测试：会覆盖固定测试地址并打印回读结果，禁止在生产流程调用。
 */
void Test_FRAM_ReadWrite(void)
{
    uint32_t test_cases[][2] = {
        {50U, 0x12345678U},
        {511U, 0xA5A5A5A5U},
        {100U, 0x00000000U},
        {200U, 0xFFFFFFFFU}
    };
    uint8_t write_data[8] = {0x11U, 0x22U, 0x33U, 0x44U, 0x55U, 0x66U, 0x77U, 0x88U};
    uint8_t read_data[8] = {0U};
    uint32_t start_address = 0x0100U;
    uint32_t index;

    for (index = 0U; index < (uint32_t)(sizeof(test_cases) / sizeof(test_cases[0])); index++) {
        uint32_t read_value;

        WriteSingleData(test_cases[index][1], test_cases[index][0]);
        read_value = ReadSingleData(test_cases[index][0]);
        printf("FRAM single test %lu | address=0x%04lX | write=0x%08lX | read=0x%08lX\r\n",
               (unsigned long)index,
               (unsigned long)test_cases[index][0],
               (unsigned long)test_cases[index][1],
               (unsigned long)read_value);
    }

    if ((FRAM_Write(write_data, start_address, sizeof(write_data)) == FRAM_STATUS_OK) &&
        (FRAM_Read(read_data, start_address, sizeof(read_data)) == FRAM_STATUS_OK)) {
        for (index = 0U; index < (uint32_t)sizeof(write_data); index++) {
            printf("FRAM multi test | address=0x%04lX | write=0x%02X | read=0x%02X\r\n",
                   (unsigned long)(start_address + index),
                   write_data[index],
                   read_data[index]);
        }
    }
}

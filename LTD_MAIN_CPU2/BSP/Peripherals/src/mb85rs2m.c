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

/*
 * 函数用途：把 HAL SPI 状态转换为 FRAM 统一状态。
 * 调用场景：FRAM 事务的每一个命令、地址和数据阶段。
 * 关键约束：不打印、不递归访问 FRAM。
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

/*
 * 函数用途：在短临界区内取得唯一 FRAM 事务所有权。
 * 调用场景：FRAM_Read 和 FRAM_Write 进入硬件事务之前。
 * 关键约束：不等待；已有事务时立即返回 BUSY，避免中断上下文死锁。
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

/*
 * 函数用途：释放 FRAM 事务所有权。
 * 调用场景：FRAM 事务统一清理出口。
 * 关键约束：先恢复片选为高，再允许其它调用者进入。
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

/*
 * 函数用途：为掉电编码器提交预留后续FRAM事务。
 * 调用场景：24V模拟看门狗中断。
 * 关键约束：不打断已经开始的事务；预留后普通新事务立即返回BUSY。
 */
void FRAM_RequestEmergencyReservationFromISR(void)
{
    s_fram_emergency_reserved = 1U;
    __DMB();
}

/* PendSV在执行紧急A/B与回执事务前声明所有者，使其可越过普通写门禁。 */
void FRAM_EnterEmergencyOwner(void)
{
    s_fram_emergency_owner = 1U;
    __DMB();
}

/* 结束紧急所有者身份；预约仍保留到电压稳定且紧急请求完全结束。 */
void FRAM_ExitEmergencyOwner(void)
{
    __DMB();
    s_fram_emergency_owner = 0U;
}

/* 释放掉电预约，允许普通读写重新参与无等待仲裁。 */
void FRAM_ReleaseEmergencyReservation(void)
{
    __DMB();
    s_fram_emergency_reserved = 0U;
}

/*
 * 函数用途：查询紧急掉电流程是否已经预约 FRAM。
 * 调用场景：主循环参数延后保存决定是否继续等待。
 * 关键约束：只读取单字节状态，不等待、不打印。
 */
bool FRAM_IsEmergencyReserved(void)
{
    return s_fram_emergency_reserved != 0U;
}

/*
 * 函数用途：在电源不可靠时禁止普通 FRAM 写入。
 * 调用场景：24V 监控启动、低压、DMA 故障和恢复边界。
 * 关键约束：紧急所有者仍可写入掉电记录；FRAM 读取不受影响。
 */
void FRAM_SetNormalWriteInhibitedFromISR(bool inhibited)
{
    s_fram_normal_write_inhibited = inhibited ? 1U : 0U;
    __DMB();
}

/* 在触碰片选前校验指针、HAL 16bit长度限制及256KiB地址边界。 */
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

/* 已持有事务锁时发送WREN；无论HAL结果如何都在返回前释放片选。 */
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

/*
 * 统一写事务：参数校验、双重普通写门禁、无等待加锁、WREN、24bit地址和数据发送。
 * 所有硬件阶段共用cleanup出口恢复片选并释放事务锁。
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

/*
 * 统一读事务：读取不受普通写禁止影响，但仍必须遵守紧急预约和唯一事务锁。
 * 任一SPI阶段失败均从cleanup恢复片选并释放所有权。
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

void WriteMultiData(uint8_t const *p_array, int startcnt, uint32_t length)
{
    if (startcnt < 0) {
        return;
    }
    (void)FRAM_Write(p_array, (uint32_t)startcnt, length);
}

void ReadMultiData(uint8_t *p_array, int startcnt, uint32_t length)
{
    if (startcnt < 0) {
        return;
    }
    (void)FRAM_Read(p_array, (uint32_t)startcnt, length);
}

void WriteSingleData(uint32_t data, uint32_t address)
{
    uint8_t bytes[4];

    bytes[0] = (uint8_t)((data >> 24) & 0xFFU);
    bytes[1] = (uint8_t)((data >> 16) & 0xFFU);
    bytes[2] = (uint8_t)((data >> 8) & 0xFFU);
    bytes[3] = (uint8_t)(data & 0xFFU);
    (void)FRAM_Write(bytes, address, sizeof(bytes));
}

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

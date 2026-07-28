#include "TMC5130.h"

#include "assert.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "spi.h"
#include "fault_manager.h"
#include "error_log.h"
#include "encoder.h"
#include "power_monitor.h"

#include "sensor.h"    /* 传感器相关接口（如扭力、防撞检测等） */
#include "motor_ctrl.h" /* 电机控制上层接口 */

/* 保持电流固定为同一口径，运行时修改 motor_current 只改变 IRUN。 */
#define TMC5130_MOTOR_IHOLD_VALUE       (4U) /* TMC5130 电机保持电流配置值。 */
#define TMC5130_MOTOR_IHOLDDELAY_VALUE  (7U) /* TMC5130 保持电流延时配置值。 */

#define TMC5130_BYTE(value, n)           (((value) >> ((n) << 3)) & 0xFF) /* 从 32 位寄存器值中取第 n 个字节。 */
#define TMC5130_SET_IHOLD(a)             (((a) & 0x1F) << 0) /* 生成 IHOLD_IRUN 寄存器的保持电流字段。 */
#define TMC5130_SET_IRUN(a)              (((a) & 0x1F) << 8) /* 生成 IHOLD_IRUN 寄存器的运行电流字段。 */
#define TMC5130_SET_IHOLDDELAY(a)        (((a) & 0x0F) << 16) /* 生成 IHOLD_IRUN 寄存器的保持电流延时字段。 */
#define TMC5130_IHOLD_IRUN_FIELD_MASK     (0x000F1F1FU) /* IHOLD_IRUN 寄存器中电流相关字段掩码。 */
#ifndef TMC5130_SPI_CS_DELAY_CYCLES
/* CS 保护延时：手册的最小时序是 ns 级，但现场在电机运动时出现过读帧错位/非法 GSTAT。
 * 这里保留更大的高/低电平间隔，用来提高片选重新同步和抗干扰裕量。 */
#define TMC5130_SPI_CS_DELAY_CYCLES      (512U) /* SPI 片选翻转后的短延时循环数。 */
#endif
#ifndef TMC5130_XACTUAL_READ_TOLERANCE_TICKS
#define TMC5130_XACTUAL_READ_TOLERANCE_TICKS  (8192L) /* XACTUAL 连续读数一致性允许偏差，单位 tick。 */
#endif
#ifndef TMC5130_INIT_WRITE_RETRY_MAX
#define TMC5130_INIT_WRITE_RETRY_MAX          (3U) /* TMC5130 初始化寄存器写入最大重试次数。 */
#endif
#ifndef TMC5130_WAIT_POS_TOLERANCE_TICKS
/* waitMove 退出前会比较 XACTUAL 与 XTARGET。该容差用于吸收细分步误差、
 * 停止瞬间寄存器刷新延迟，以及位置读数的正常抖动，避免等不到完全相等。 */
#define TMC5130_WAIT_POS_TOLERANCE_TICKS     (1024L) /* 等待到位时允许的位置误差，单位 tick。 */
#endif
/* GSTAT 按手册只有 bit0(reset)、bit1(drv_err)、bit2(uv_cp) 有效。
 * 高位非 0 不是新增故障位，而是 SPI 读数不可信，应按通信异常处理。 */
#define TMC5130_GSTAT_VALID_MASK              (0x07UL) /* GSTAT 寄存器合法状态位掩码。 */

/* TMC5130 初始化写寄存器失败即返回的检查宏。 */
#define TMC5130_REQUIRE_WRITE(handle, address, value)            \
    do {                                                         \
        if (!stpr_writeInt((handle), (address), (value))) {      \
            return MOTOR_TMC_COMM_ERROR;                         \
        }                                                        \
    } while (0)

#define TMC5130_DRVSTATUS_SG_RESULT_MASK   (0x000003FFUL) /* TMC5130 DRV_STATUS 位掩码：SG RESULT 掩码。 */
#define TMC5130_DRVSTATUS_FSACTIVE         (1UL << 15) /* TMC5130 DRV_STATUS 位掩码：fullstep 主动标志。 */
#define TMC5130_DRVSTATUS_CS_ACTUAL_MASK   (0x001F0000UL) /* TMC5130 DRV_STATUS 位掩码：实际线圈电流档位 CS_ACTUAL。 */
#define TMC5130_DRVSTATUS_STALLGUARD       (1UL << 24) /* TMC5130 DRV_STATUS 位掩码：stallGuard 报警标志。 */
#define TMC5130_DRVSTATUS_OT               (1UL << 25) /* TMC5130 DRV_STATUS 位掩码：OT。 */
#define TMC5130_DRVSTATUS_OTPW             (1UL << 26) /* TMC5130 DRV_STATUS 位掩码：OTPW。 */
#define TMC5130_DRVSTATUS_S2GA             (1UL << 27) /* TMC5130 DRV_STATUS 位掩码：S2GA。 */
#define TMC5130_DRVSTATUS_S2GB             (1UL << 28) /* TMC5130 DRV_STATUS 位掩码：S2GB。 */
#define TMC5130_DRVSTATUS_OLA              (1UL << 29) /* TMC5130 DRV_STATUS 位掩码：OLA。 */
#define TMC5130_DRVSTATUS_OLB              (1UL << 30) /* TMC5130 DRV_STATUS 位掩码：OLB。 */
#define TMC5130_DRVSTATUS_STST             (1UL << 31) /* TMC5130 DRV_STATUS 位掩码：STST。 */

/**
 * @brief 从 DRV_STATUS 中提取实际电流档 CS_ACTUAL。
 *
 * 该值用于判断驱动功率级是否真正建立电流；只做位解析，不访问硬件、不打印日志。
 */
static uint32_t tmc5130_drvStatusCurrent(uint32_t drvstatus)
{
    return (drvstatus & TMC5130_DRVSTATUS_CS_ACTUAL_MASK) >> 16;
}

/**
 * @brief 执行TMC5130 驱动中的 tmc5130_decodeDrvStatus 逻辑。
 *
 * @param drvstatus 状态值。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
static uint32_t tmc5130_decodeDrvStatus(uint32_t drvstatus)
{
    uint32_t ret = NO_ERROR;
    uint32_t cs_actual = tmc5130_drvStatusCurrent(drvstatus);
    uint32_t sg_result = drvstatus & TMC5130_DRVSTATUS_SG_RESULT_MASK;

    printf("TMC5130 DRV_STATUS 解析 | SG_RESULT=%lu | CS_ACTUAL=%lu",
           (unsigned long)sg_result, (unsigned long)cs_actual);
    if (drvstatus & TMC5130_DRVSTATUS_FSACTIVE) {
        printf(" | fsactive");
    }
    if (drvstatus & TMC5130_DRVSTATUS_STST) {
        printf(" | standstill");
    }
    printf("\r\n");

    if (drvstatus & TMC5130_DRVSTATUS_STALLGUARD) {
        printf("TMC5130 StallGuard 触发（DRV_STATUS[24]）\r\n");
        ret = MOTOR_STALL_ERROR;
    }
    if (drvstatus & TMC5130_DRVSTATUS_OT) {
        printf("TMC5130 过温关断（DRV_STATUS[25]=ot）\r\n");
        ret = MOTOR_OVERTEMPERATURE;
    }
    if (drvstatus & TMC5130_DRVSTATUS_OTPW) {
        printf("TMC5130 过温预警（DRV_STATUS[26]=otpw）\r\n");
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret == NO_ERROR) {
            ret = MOTOR_DRIVER_OVERTEMP_WARNING;
        }
    }
    if (drvstatus & TMC5130_DRVSTATUS_S2GA) {
        printf("TMC5130 A 相对地短路（DRV_STATUS[27]=s2ga）\r\n");
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret == NO_ERROR) {
            ret = MOTOR_PHASE_SHORT_ERROR;
        }
    }
    if (drvstatus & TMC5130_DRVSTATUS_S2GB) {
        printf("TMC5130 B 相对地短路（DRV_STATUS[28]=s2gb）\r\n");
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret == NO_ERROR) {
            ret = MOTOR_PHASE_SHORT_ERROR;
        }
    }
    if (drvstatus & TMC5130_DRVSTATUS_OLA) {
        printf("TMC5130 A 相开路/断线（DRV_STATUS[29]=ola）\r\n");
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret == NO_ERROR) {
            ret = MOTOR_PHASE_OPEN_ERROR;
        }
    }
    if (drvstatus & TMC5130_DRVSTATUS_OLB) {
        printf("TMC5130 B 相开路/断线（DRV_STATUS[30]=olb）\r\n");
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret == NO_ERROR) {
            ret = MOTOR_PHASE_OPEN_ERROR;
        }
    }

    /* 多个故障位同时出现时，优先返回会直接损坏驱动或电机的故障，
     * 再返回断线、堵转和预警，避免低优先级状态覆盖真实保护原因。 */
    if ((drvstatus & TMC5130_DRVSTATUS_OT) != 0U) {
        ret = MOTOR_OVERTEMPERATURE;
    } else if ((drvstatus & (TMC5130_DRVSTATUS_S2GA | TMC5130_DRVSTATUS_S2GB)) != 0U) {
        ret = MOTOR_PHASE_SHORT_ERROR;
    } else if ((drvstatus & (TMC5130_DRVSTATUS_OLA | TMC5130_DRVSTATUS_OLB)) != 0U) {
        ret = MOTOR_PHASE_OPEN_ERROR;
    } else if ((drvstatus & TMC5130_DRVSTATUS_STALLGUARD) != 0U) {
        ret = MOTOR_STALL_ERROR;
    } else if ((drvstatus & TMC5130_DRVSTATUS_OTPW) != 0U) {
        ret = MOTOR_DRIVER_OVERTEMP_WARNING;
    }

    return ret;
}
/* 全局步进电机驱动句柄（默认配置） */
/* 注意：这里只是给出一个全局默认实例，实际项目中也可以在别处重新初始化 */
TMC5130TypeDef stepper = {
    .spi     = &hspi2,        /* 使用的 SPI 句柄 */
    .cs_port = GPIOB,         /* SPI 片选 GPIO 端口 */
    .cs_pin  = GPIO_PIN_12,   /* SPI 片选引脚 */
    .en_port = GPIOD,         /* 使能引脚 GPIO 端口 */
    .en_pin  = GPIO_PIN_10    /* 使能引脚 */
};

/* => SPI 底层封装（仅本文件内部使用） */
static bool tmc5130_tryReadArray(TMC5130TypeDef *tmc5130,
                                 uint8_t *data,
                                 size_t length,
                                 uint32_t *value,
                                 uint8_t stage);
static bool tmc5130_writeArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length);
static bool tmc5130_readRegisterOnce(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value);
static bool tmc5130_readXactualStable(TMC5130TypeDef *tmc5130, int32_t *value);
static bool tmc5130_isCloseInt32(int32_t a, int32_t b, int32_t tolerance);
static uint32_t tmc5130_buildCurrentSetting(uint8_t current);
static void tmc5130_logCurrentSetting(uint32_t setting);
static void tmc5130_delayCsGuard(void);
static bool tmc5130_enterSpiAccess(void);
static void tmc5130_leaveSpiAccess(void);
static void tmc5130_initWrite(TMC5130TypeDef *tmc5130,
                              uint8_t address,
                              int32_t value,
                              uint32_t *failed_count);
/* < = SPI 底层封装 */

static volatile uint8_t s_tmc5130_spi_busy = 0U; /* TMC5130 驱动模块级变量，保存跨函数共享的业务状态。 */
static TMC5130DiagnosticSnapshot s_tmc5130_diagnostic = {0U};

/*
 * 函数用途：保存最后一次有效 TMC5130 故障现场。
 * 调用场景：SPI 访问失败、XACTUAL 连续读数不稳定或配置丢失时调用。
 * 关键约束：只保存数值，不打印日志，不改变原有错误码和停机行为。
 */
static void tmc5130_saveDiagnostic(uint8_t stage,
                                   uint8_t direction,
                                   uint8_t address,
                                   uint8_t response_status,
                                   uint32_t hal_status,
                                   uint32_t error_code,
                                   uint32_t expected_value,
                                   uint32_t actual_value,
                                   int32_t sample_first,
                                   int32_t sample_second,
                                   int32_t sample_third)
{
    uint32_t primask;
    uint32_t sequence;

    primask = __get_PRIMASK();
    __disable_irq();
    sequence = s_tmc5130_diagnostic.sequence + 1U;
    if (sequence == 0U) {
        sequence = 1U;
    }
    s_tmc5130_diagnostic.sequence = sequence;
    s_tmc5130_diagnostic.error_code = error_code;
    s_tmc5130_diagnostic.hal_status = hal_status;
    s_tmc5130_diagnostic.expected_value = expected_value;
    s_tmc5130_diagnostic.actual_value = actual_value;
    s_tmc5130_diagnostic.sample_first = sample_first;
    s_tmc5130_diagnostic.sample_second = sample_second;
    s_tmc5130_diagnostic.sample_third = sample_third;
    s_tmc5130_diagnostic.stage = stage;
    s_tmc5130_diagnostic.direction = direction;
    s_tmc5130_diagnostic.address = address;
    s_tmc5130_diagnostic.response_status = response_status;
    __set_PRIMASK(primask);
}

/*
 * 函数用途：复制最后一次有效 TMC5130 故障现场。
 * 调用场景：故障管理最终出口和故障注入测试读取。
 * 关键约束：使用短临界区保证所有字段来自同一次故障，不访问 SPI。
 */
void TMC5130_GetDiagnosticSnapshot(TMC5130DiagnosticSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = s_tmc5130_diagnostic;
    __set_PRIMASK(primask);
}

static void tmc5130_delayCsGuard(void)
/* TMC5130 驱动与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
/* 这里使用 NOP 而不是 HAL_Delay，避免把每次 5 字节 SPI 访问扩大到 ms 级。
 * 延时会同时用于 CS 拉低前、拉低后和拉高后，确保每帧边界留出恢复时间。 */
{
    for (volatile uint32_t i = 0; i < TMC5130_SPI_CS_DELAY_CYCLES; ++i) {
        __NOP();
    }
}

/**
 * @brief 执行TMC5130 驱动中的 tmc5130_enterSpiAccess 逻辑。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
static bool tmc5130_enterSpiAccess(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 进入临界区，保护TMC5130 驱动共享状态，避免中断同时修改。 */
    __disable_irq();
    if (s_tmc5130_spi_busy != 0U) {
        __set_PRIMASK(primask);
        return false;
    }

    s_tmc5130_spi_busy = 1U;
    __set_PRIMASK(primask);
    return true;
}

/**
 * @brief 执行TMC5130 驱动中的 tmc5130_leaveSpiAccess 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void tmc5130_leaveSpiAccess(void)
{
    uint32_t primask = __get_PRIMASK();

    /* 进入临界区，保护TMC5130 驱动共享状态，避免中断同时修改。 */
    __disable_irq();
    s_tmc5130_spi_busy = 0U;
    __set_PRIMASK(primask);
}

/* *********************** SPI 读写封装 *********************** */

/**
 * @brief  通过 SPI 读一个寄存器（发送 length 字节，接收 5 字节）
 *         TMC5130 的寄存器读取机制：
 *         - 第一次发送地址，只是“触发”内部准备数据，不使用返回值
 *         - 第二次再发送同一地址，才能读到上一帧准备好的 32bit 数据
 *         此函数只负责发一帧并返回 RX 数据解析出来的 32bit 整数
 *
 * @param  tmc5130  驱动句柄
 * @param  data     TX 缓冲区指针（data[0] 一般为寄存器地址）
 * @param  length   发送/接收长度，一般为 5
 * @retval 组合后的 32bit 数据（rxBuff[1..4]）
 */
static bool tmc5130_tryReadArray(TMC5130TypeDef *tmc5130,
                                 uint8_t *data,
                                 size_t length,
                                 uint32_t *value,
                                 uint8_t stage)
{
    uint8_t rxBuff[5] = { 0, 0, 0, 0, 0 };
    HAL_StatusTypeDef status;
    uint8_t address = 0U;

    if (data != NULL) {
        address = data[0] & 0x7FU;
    }

    if ((tmc5130 == NULL) || (data == NULL) || (value == NULL) ||
        (tmc5130->spi == NULL) || (tmc5130->cs_port == NULL) ||
        (length == 0U) || (length > sizeof(rxBuff))) {
        tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_PARAMETER,
                               TMC5130_DIAG_DIRECTION_READ,
                               address,
                               0U,
                               (uint32_t)HAL_ERROR,
                               SYSTEM_CALL_CONDITION_ERROR,
                               0U,
                               0U,
                               0,
                               0,
                               0);
        return false;
    }
    if (!tmc5130_enterSpiAccess()) {
        tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_ACCESS_BUSY,
                               TMC5130_DIAG_DIRECTION_READ,
                               address,
                               0U,
                               (uint32_t)HAL_BUSY,
                               MOTOR_TMC_COMM_ERROR,
                               0U,
                               0U,
                               0,
                               0,
                               0);
        return false;
    }

    /* TMC5130 的 SPI 帧以 CS 上升沿结束、下一次下降沿重新对齐。
     * 连续读 XACTUAL 时如果 CS 高电平保持太短，现场会出现状态字 0x21 混入数据区，
     * 例如 0x21000000、0x77210000 这类错位值。这里给 CS 建立/保持留出少量 CPU 周期。 */
    tmc5130_delayCsGuard();
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_RESET);
    tmc5130_delayCsGuard();
    status = HAL_SPI_TransmitReceive(tmc5130->spi, data, rxBuff, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_SET);
    tmc5130_delayCsGuard();
    tmc5130_leaveSpiAccess();

    if (status != HAL_OK) {
        tmc5130_saveDiagnostic(stage,
                               TMC5130_DIAG_DIRECTION_READ,
                               address,
                               rxBuff[0],
                               (uint32_t)status,
                               MOTOR_TMC_COMM_ERROR,
                               0U,
                               0U,
                               0,
                               0,
                               0);
        return false;
    }

    /* TMC5130 数据在 rxBuff[1..4]，按大端组合为 32 位 */
    *value = (((uint32_t)rxBuff[1] << 24) |
              ((uint32_t)rxBuff[2] << 16) |
              ((uint32_t)rxBuff[3] << 8) |
              ((uint32_t)rxBuff[4]));
    return true;
}

/**
 * @brief  通过 SPI 写一帧数据，并返回 HAL 发送是否成功
 *
 * @param  tmc5130 驱动句柄
 * @param  data    待发送数据（data[0] 为寄存器地址 | 写标志位）
 * @param  length  数据长度，一般为 5
 */
static bool tmc5130_writeArray(TMC5130TypeDef *tmc5130, uint8_t *data, size_t length)
{
    HAL_StatusTypeDef status;
    uint8_t address = 0U;
    uint32_t expected_value = 0U;

    if (data != NULL) {
        address = data[0] & 0x7FU;
        if (length >= 5U) {
            expected_value = (((uint32_t)data[1] << 24) |
                              ((uint32_t)data[2] << 16) |
                              ((uint32_t)data[3] << 8) |
                              ((uint32_t)data[4]));
        }
    }

    if ((tmc5130 == NULL) || (data == NULL) ||
        (tmc5130->spi == NULL) || (tmc5130->cs_port == NULL) ||
        (length == 0U)) {
        tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_PARAMETER,
                               TMC5130_DIAG_DIRECTION_WRITE,
                               address,
                               0U,
                               (uint32_t)HAL_ERROR,
                               SYSTEM_CALL_CONDITION_ERROR,
                               expected_value,
                               0U,
                               0,
                               0,
                               0);
        return false;
    }
    if (!tmc5130_enterSpiAccess()) {
        tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_ACCESS_BUSY,
                               TMC5130_DIAG_DIRECTION_WRITE,
                               address,
                               0U,
                               (uint32_t)HAL_BUSY,
                               MOTOR_TMC_COMM_ERROR,
                               expected_value,
                               0U,
                               0,
                               0,
                               0);
        return false;
    }

    tmc5130_delayCsGuard();
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_RESET);
    tmc5130_delayCsGuard();
    status = HAL_SPI_Transmit(tmc5130->spi, data, length, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(tmc5130->cs_port, tmc5130->cs_pin, GPIO_PIN_SET);
    tmc5130_delayCsGuard();
    tmc5130_leaveSpiAccess();

    if (status != HAL_OK) {
        tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_SPI_WRITE,
                               TMC5130_DIAG_DIRECTION_WRITE,
                               address,
                               0U,
                               (uint32_t)status,
                               MOTOR_TMC_COMM_ERROR,
                               expected_value,
                               0U,
                               0,
                               0,
                               0);
        return false;
    }

    return true;
}

/* *********************** 寄存器级读写 *********************** */

/**
 * @brief  写 4 字节（x1~x4）到指定寄存器地址
 *
 * @param  address 寄存器地址（不带写标志位）
 * @param  x1~x4   四个字节，高字节在前
 */
static bool tmc5130_writeDatagram(TMC5130TypeDef *tmc5130,
                                  uint8_t address,
                                  uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4)
{
    uint8_t data[5] = { address | TMC5130_WRITE_BIT, x1, x2, x3, x4 };
    return tmc5130_writeArray(tmc5130, &data[0], 5);
}

/**
 * @brief  向 TMC5130 指定寄存器写入 32 位整数
 *
 * @param  address 寄存器地址
 * @param  value   要写入的 32 位数
 */
bool stpr_writeInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t value)
{
    return tmc5130_writeDatagram(
        tmc5130,
        address,
        TMC5130_BYTE(value, 3),
        TMC5130_BYTE(value, 2),
        TMC5130_BYTE(value, 1),
        TMC5130_BYTE(value, 0)
    );
}

/**
 * @brief 初始化TMC5130 驱动中的 tmc5130_initWrite 逻辑。
 *
 * @param tmc5130 业务参数。
 * @param address 地址参数。
 * @param value 待处理数值。
 * @param failed_count 业务参数。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static void tmc5130_initWrite(TMC5130TypeDef *tmc5130,
                              uint8_t address,
                              int32_t value,
                              uint32_t *failed_count)
{
    for (uint32_t attempt = 1U; attempt <= TMC5130_INIT_WRITE_RETRY_MAX; ++attempt) {
        if (stpr_writeInt(tmc5130, address, value)) {
            if (attempt > 1U) {
                /* 错误 阶段：重试成功 模块：电机 操作：驱动初始化 原因：恢复成功 尝试：attempt/TMC5130_INIT_WRITE_RETRY_MAX */
                ErrorLog_Recover(ERROR_LOG_MODULE_MOTOR,
                                 ERROR_LOG_OP_DRIVER_INIT,
                                 ERROR_LOG_REASON_RECOVER_OK,
                                 attempt,
                                 TMC5130_INIT_WRITE_RETRY_MAX);
                printf("TMC5130初始化写寄存器恢复 | 地址：0x%02X | 写入值=0x%08lX | 尝试：%lu/%lu\r\n",
                       address,
                       (unsigned long)((uint32_t)value),
                       (unsigned long)attempt,
                       (unsigned long)TMC5130_INIT_WRITE_RETRY_MAX);
            }
            return;
        }

        /* 错误 阶段：错误重试 模块：电机 操作：驱动初始化 原因：通信失败 尝试：attempt/TMC5130_INIT_WRITE_RETRY_MAX 错误码：MOTOR_TMC_COMM_ERROR 错误名：ErrorLog_GetCodeName(MOTOR_TMC_COMM_ERROR) */
        ErrorLog_Retry(ERROR_LOG_MODULE_MOTOR,
                       ERROR_LOG_OP_DRIVER_INIT,
                       ERROR_LOG_REASON_COMM_FAIL,
                       attempt,
                       TMC5130_INIT_WRITE_RETRY_MAX,
                       MOTOR_TMC_COMM_ERROR);
        printf("TMC5130初始化写寄存器失败 | 地址：0x%02X | 写入值=0x%08lX | 尝试：%lu/%lu\r\n",
               address,
               (unsigned long)((uint32_t)value),
               (unsigned long)attempt,
               (unsigned long)TMC5130_INIT_WRITE_RETRY_MAX);

        /* 初始化阶段出现 HAL_BUSY/瞬态干扰时，短延时后重试，避免一次失败直接误判。 */
        HAL_Delay(1U);
    }

    if (failed_count != NULL) {
        (*failed_count)++;
    }
}
/**
 * @brief  执行一次完整的 TMC5130 寄存器读取流程。
 *         TMC5130 的读操作带流水线：第一帧只提交地址，第二帧才返回上一帧准备好的数据。
 */
static bool tmc5130_readRegisterOnce(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value)
{
    uint8_t  data[5] = { 0, 0, 0, 0, 0 };
    uint32_t raw = 0U;

    if (value == NULL) {
        return false;
    }

    data[0] = address;
    if (!tmc5130_tryReadArray(tmc5130,
                              data,
                              5,
                              &raw,
                              TMC5130_DIAG_STAGE_SPI_READ_TRIGGER)) { /* 第一次触发读取（数据无效） */
        return false;
    }

    data[0] = address;
    if (!tmc5130_tryReadArray(tmc5130,
                              data,
                              5,
                              &raw,
                              TMC5130_DIAG_STAGE_SPI_READ_DATA)) { /* 第二次才是真正数据 */
        return false;
    }

    *value = (int32_t)raw;
    return true;
}

/**
 * @brief 执行TMC5130 驱动中的 tmc5130_isCloseInt32 逻辑。
 *
 * @param a 业务参数。
 * @param b 业务参数。
 * @param tolerance 业务参数。
 * @return true 表示条件满足或处理成功，false 表示条件不满足或处理失败。
 */
static bool tmc5130_isCloseInt32(int32_t a, int32_t b, int32_t tolerance)
{
    int64_t delta = (int64_t)a - (int64_t)b;

    if (delta < 0) {
        delta = -delta;
    }
    return (delta <= (int64_t)tolerance);
}

/**
 * @brief  稳定读取 XACTUAL。
 *
 * 现场日志中出现 0x21 状态字混入数据区的错位值，例如 0x4DF12100、0xF1210000。
 * 这类值会让上层看到瞬时位置跳变。XACTUAL 是位置核心基准，因此这里要求连续
 * 两次完整读取足够接近；若第一组不稳定，再补读第三组，用后两组稳定值作为结果。
 */
static bool tmc5130_readXactualStable(TMC5130TypeDef *tmc5130, int32_t *value)
{
    int32_t first = 0;
    int32_t second = 0;
    int32_t third = 0;
    char detail[128];

    if (!tmc5130_readRegisterOnce(tmc5130, TMC5130_XACTUAL, &first)) {
        return false;
    }
    if (!tmc5130_readRegisterOnce(tmc5130, TMC5130_XACTUAL, &second)) {
        return false;
    }

    if (tmc5130_isCloseInt32(first, second, TMC5130_XACTUAL_READ_TOLERANCE_TICKS)) {
        *value = second;
        return true;
    }

    if (!tmc5130_readRegisterOnce(tmc5130, TMC5130_XACTUAL, &third)) {
        return false;
    }

    if (tmc5130_isCloseInt32(second, third, TMC5130_XACTUAL_READ_TOLERANCE_TICKS)) {
        *value = third;
        return true;
    }
    if (tmc5130_isCloseInt32(first, third, TMC5130_XACTUAL_READ_TOLERANCE_TICKS)) {
        *value = third;
        return true;
    }

    printf("TMC5130 XACTUAL连续读取不稳定 | 第一次=%ld | 第二次=%ld | 第三次=%ld\r\n",
           (long)first,
           (long)second,
           (long)third);
    snprintf(detail, sizeof(detail),
             "第一次=%ld,第二次=%ld,第三次=%ld",
             (long)first,
             (long)second,
             (long)third);
    /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：通信失败 处理：停止电机 详情：detail */
    ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                        ERROR_LOG_OP_DRIVER_CHECK,
                        ERROR_LOG_REASON_COMM_FAIL,
                        ERROR_LOG_ACTION_STOP_MOTOR,
                        detail);
    tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_XACTUAL_UNSTABLE,
                           TMC5130_DIAG_DIRECTION_READ,
                           TMC5130_XACTUAL,
                           0U,
                           (uint32_t)HAL_OK,
                           MOTOR_TMC_COMM_ERROR,
                           (uint32_t)first,
                           (uint32_t)third,
                           first,
                           second,
                           third);
    return false;
}

/**
 * @brief  从 TMC5130 指定寄存器读取 32 位整数。
 *
 * XACTUAL 直接影响电机尺带长度和运动判断，单次错位值风险最高，因此使用稳定读取；
 * 其它寄存器保持原来的两帧流水线读取流程，避免改变状态类寄存器的实时语义。
 */
bool stpr_tryReadInt(TMC5130TypeDef *tmc5130, uint8_t address, int32_t *value)
{
    if (value == NULL) {
        tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_PARAMETER,
                               TMC5130_DIAG_DIRECTION_READ,
                               address,
                               0U,
                               (uint32_t)HAL_ERROR,
                               SYSTEM_CALL_CONDITION_ERROR,
                               0U,
                               0U,
                               0,
                               0,
                               0);
        return false;
    }

    if (address == TMC5130_XACTUAL) {
        return tmc5130_readXactualStable(tmc5130, value);
    }

    return tmc5130_readRegisterOnce(tmc5130, address, value);
}


/* *********************** 内部速度模式工具 / 位置控制接口 *********************** */

/**
 * @brief  以给定速度持续旋转（速度模式）
 *
 * @param  velocity 目标速度（正：正向，负：反向）
 */
uint32_t stpr_rotate(TMC5130TypeDef *tmc5130, int32_t velocity)
{
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_VMAX, abs(velocity));
    TMC5130_REQUIRE_WRITE(tmc5130,
                          TMC5130_RAMPMODE,
                          (velocity >= 0) ? TMC5130_MODE_VELPOS : TMC5130_MODE_VELNEG);
    return NO_ERROR;
}

/**
 * @brief  停止电机并回到安全的位置模式。
 *
 * 速度模式下只把 VMAX 写成 0 虽然能停住当前动作，但 RAMPMODE 会停留在速度模式。
 * 后续如果只是恢复速度参数并写 VMAX，驱动可能在没有新位置命令的情况下重新运动。
 * 因此停止时把当前位置同步为目标位置，再切回位置模式，保证空闲态写 VMAX 不会启动电机。
 */
uint32_t stpr_stop(TMC5130TypeDef *tmc5130)
{
    uint32_t ret;
    int32_t xactual = 0;

    ret = stpr_rotate(tmc5130, 0);
    /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
    if (ret != NO_ERROR) {
        return ret;
    }
    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &xactual)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_XTARGET, xactual);
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    return NO_ERROR;
}

/**
 * @brief  以给定最大速度，运行到指定位置（位置模式）
 *
 * @param  position   目标位置（步数）
 * @param  velocityMax 最大速度（VMAX）
 */
uint32_t stpr_moveTo(TMC5130TypeDef *tmc5130, int32_t position, uint32_t velocityMax)
{
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_VMAX, velocityMax);
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_XTARGET, position);
    MotorCtrl_PersistRegistersFromDriver();
    return NO_ERROR;
}

/**
 * @brief  在当前位置基础上“相对移动”一定步数
 *         ticks 传入的是相对位移，函数内部会转换成绝对位置并回写到 *ticks
 *
 * @param  ticks       [入/出] 相对位移 / 计算后的绝对目标位置
 * @param  velocityMax 最大速度
 */
uint32_t stpr_moveBy(TMC5130TypeDef *tmc5130, int32_t *ticks, uint32_t velocityMax)
{
    int32_t xactual;
    int64_t target;

    if ((tmc5130 == NULL) || (ticks == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    /* 相对位移必须建立在可信的 XACTUAL 上；读失败时不能把当前位置当 0。 */
    if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &xactual)) {
        return MOTOR_TMC_COMM_ERROR;
    }
    target = (int64_t)xactual + (int64_t)(*ticks);
    if ((target > (int64_t)INT32_MAX) || (target < (int64_t)INT32_MIN)) {
        printf("stpr_moveBy溢出: XACTUAL=%ld, 增量：%ld, 目标：%lld\r\n",
               (long)xactual,
               (long)(*ticks),
               (long long)target);
        return SYSTEM_CALCULATION_ERROR;
    }

    *ticks = (int32_t)target;
    return stpr_moveTo(tmc5130, *ticks, velocityMax);
}

/* *********************** 使能与位置设置 *********************** */

/**
 * @brief  禁止驱动（EN 引脚拉高或拉低视硬件设计，一般为高电平关闭）
 */
void stpr_disableDriver(TMC5130TypeDef *tmc5130)
{
    HAL_GPIO_WritePin(tmc5130->en_port, tmc5130->en_pin, GPIO_PIN_SET);
}

/**
 * @brief  使能驱动（EN 引脚）
 */
void stpr_enableDriver(TMC5130TypeDef *tmc5130)
{
    /*
     * 这是最底层最终门禁：即使上层遗漏状态检查，电源监控或编码器锁存期间也只允许
     * 把ENN保持为高电平关闭状态，不能通过任何运动路径重新使能驱动。
     */
    if (PowerMonitor_IsMotorInhibited() || Encoder_HasLatchedFault()) {
        HAL_GPIO_WritePin(tmc5130->en_port, tmc5130->en_pin, GPIO_PIN_SET);
        return;
    }
    HAL_GPIO_WritePin(tmc5130->en_port, tmc5130->en_pin, GPIO_PIN_RESET);
}

/**
 * @brief  同时设置“实际位置”和“目标位置”，相当于软件重置坐标
 */
uint32_t stpr_setPos(TMC5130TypeDef *tmc5130, int32_t position)
{
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_XTARGET, position);
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_XACTUAL, position);
    MotorCtrl_PersistRegistersFromDriver();
    return NO_ERROR;
}

/* *********************** 运动等待与故障检测 *********************** */
/**
 * @brief 检查并解析 TMC5130 驱动异常状态。
 *
 * 该函数只解析 GSTAT/DRV_STATUS 故障位，不检查 CS_ACTUAL 功率级。
 * - GSTAT 非法高位只打印并丢弃，避免把 SPI 错帧误判为真实故障；
 * - drv_err 会继续读取 DRV_STATUS，细分过温、短路、开路、StallGuard；
 * - uv_cp 和 reset 会清除 GSTAT 后返回对应错误。
 * 功率级是否建立统一由 stpr_checkDriverPowerReady() 或 MotorDriver_CheckHealth() 追加检查。
 *
 * @param tmc5130 TMC5130 设备对象。
 * @return NO_ERROR 或对应电机故障错误码。
 */
uint32_t stpr_checkDriverStatus(TMC5130TypeDef *tmc5130)
{
    int32_t gstat;
    uint32_t gstat_raw;
    int32_t drvstatus = 0;
    bool drvstatus_ok;
    bool reset_flag;
    bool driver_error;
    bool charge_pump_uv;
    char detail[128];

    if (!stpr_tryReadInt(tmc5130, TMC5130_GSTAT, &gstat)) {
        MotorCtrl_InvalidateDriverInit();
        return MOTOR_TMC_COMM_ERROR;
    }
    if (gstat == 0) {
        int32_t chopconf = 0;

        /* GSTAT/RAMPSTAT 可能读成 0；再读 CHOPCONF 用于识别“读得通但配置全丢”的掉电场景。 */
        if (!stpr_tryReadInt(tmc5130, TMC5130_CHOPCONF, &chopconf)) {
            snprintf(detail, sizeof(detail), "全局状态：0x00000000,斩波配置：读取失败");
            /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：通信失败 处理：停止电机 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_DRIVER_CHECK,
                                ERROR_LOG_REASON_COMM_FAIL,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            MotorCtrl_InvalidateDriverInit();
            return MOTOR_TMC_COMM_ERROR;
        }
        if (chopconf == 0) {
            /* CHOPCONF 正常配置不应为 0；为 0 时不能再把 GSTAT=0 当作健康状态。 */
            printf("TMC5130配置丢失或驱动掉电 | GSTAT=0x00000000 | CHOPCONF=0x00000000\r\n");
            snprintf(detail, sizeof(detail), "全局状态：0x00000000,斩波配置：0x00000000");
            /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：通信失败 处理：停止电机 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_DRIVER_CHECK,
                                ERROR_LOG_REASON_COMM_FAIL,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_CONFIGURATION_LOST,
                                   TMC5130_DIAG_DIRECTION_READ,
                                   TMC5130_CHOPCONF,
                                   0U,
                                   (uint32_t)HAL_OK,
                                   MOTOR_TMC_CONFIG_LOST,
                                   1U,
                                   0U,
                                   0,
                                   0,
                                   0);
            MotorCtrl_InvalidateDriverInit();
            return MOTOR_TMC_CONFIG_LOST;
        }
        return NO_ERROR;
    }

    gstat_raw = (uint32_t)gstat;

    /* GSTAT 只有低 3 位有效。高位非 0 说明本次 SPI 读数不可信。
     * GSTAT 可能具有读后变化/读清行为，因此非法值只打印并丢弃，不复读、不清标志、不停机。 */
    if ((gstat_raw & ~TMC5130_GSTAT_VALID_MASK) != 0UL) {
        printf("TMC5130 GSTAT读数非法，已丢弃 | 全局状态：0x%08lX | 非法位：0x%08lX\r\n",
               (unsigned long)gstat_raw,
               (unsigned long)(gstat_raw & ~TMC5130_GSTAT_VALID_MASK));
        snprintf(detail, sizeof(detail),
                 "全局状态：0x%08lX,非法位：0x%08lX",
                 (unsigned long)gstat_raw,
                 (unsigned long)(gstat_raw & ~TMC5130_GSTAT_VALID_MASK));
        /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：校验失败 处理：继续尝试 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                            ERROR_LOG_OP_DRIVER_CHECK,
                            ERROR_LOG_REASON_VALIDATE_FAIL,
                            ERROR_LOG_ACTION_CONTINUE,
                            detail);
        return NO_ERROR;
    }

    /* 走到这里说明 GSTAT 编码合法，可以按低 3 位拆分真实标志。 */
    reset_flag = ((gstat_raw & (1UL << 0)) != 0UL);
    driver_error = ((gstat_raw & (1UL << 1)) != 0UL);
    charge_pump_uv = ((gstat_raw & (1UL << 2)) != 0UL);

    /* DRV_STATUS 只在 GSTAT 合法时读取。若 GSTAT 本身非法，就不继续解析详情，避免用坏帧推导假故障。 */
    drvstatus_ok = stpr_tryReadInt(tmc5130, TMC5130_DRVSTATUS, &drvstatus);

    printf("TMC5130 全局状态：0x%08lX", (unsigned long)gstat_raw);
    if (drvstatus_ok) {
        printf(" | 驱动状态：0x%08lX", (unsigned long)drvstatus);
    } else {
        printf(" | 驱动状态：读取失败");
    }
    printf("\r\n");

    /* reset 标志表示 TMC5130 发生过复位。运动过程中复位会丢失配置/状态，
     * 不能像空闲初始化那样清除后继续跑，必须交给上层重新初始化。 */
    if (reset_flag) {
        printf("TMC5130 芯片复位标志（GSTAT[0]），运动过程中复位按通信/供电异常处理\r\n");
    }

    /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
    if (driver_error) {
        uint32_t driver_ret;

        /* drv_err 只是总故障入口，真正原因要看 DRV_STATUS：过温、短路、开路、StallGuard 等。
         * 如果连 DRV_STATUS 都读不到，优先按通信异常处理，而不是猜测驱动故障类型。 */
        if (!drvstatus_ok) {
            snprintf(detail, sizeof(detail),
                     "全局状态：0x%08lX,驱动状态：读取失败",
                     (unsigned long)gstat_raw);
            /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：通信失败 处理：停止电机 详情：detail */
            ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                                ERROR_LOG_OP_DRIVER_CHECK,
                                ERROR_LOG_REASON_COMM_FAIL,
                                ERROR_LOG_ACTION_STOP_MOTOR,
                                detail);
            MotorCtrl_InvalidateDriverInit();
            return MOTOR_TMC_COMM_ERROR;
        }

        driver_ret = tmc5130_decodeDrvStatus((uint32_t)drvstatus);
        printf("检测到驱动器错误（GSTAT[1]）\r\n");
        if (!stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07)) { /* 清除所有 GSTAT 标志位 */
            MotorCtrl_InvalidateDriverInit();
            return MOTOR_TMC_COMM_ERROR;
        }
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (driver_ret == NO_ERROR) {
            driver_ret = MOTOR_UNKNOWN_FEEDBACK;
        }
        snprintf(detail, sizeof(detail),
                 "全局状态：0x%08lX,驱动状态：0x%08lX",
                 (unsigned long)gstat_raw,
                 (unsigned long)((uint32_t)drvstatus));
        /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：ErrorLog_GetReasonByCode(driver_ret) 处理：停止电机 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                            ERROR_LOG_OP_DRIVER_CHECK,
                            ErrorLog_GetReasonByCode(driver_ret),
                            ERROR_LOG_ACTION_STOP_MOTOR,
                            detail);
        MotorCtrl_InvalidateDriverInit();
        return driver_ret;
    }

    if (charge_pump_uv) {
        /* 充电泵欠压会影响高边驱动能力，继续运动风险较高，返回错误交给上层停机。 */
        printf("检测到充电泵欠压（GSTAT[2]）\r\n");
        if (!stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07)) { /* 清除所有 GSTAT 标志位 */
            MotorCtrl_InvalidateDriverInit();
            return MOTOR_TMC_COMM_ERROR;
        }
        snprintf(detail, sizeof(detail),
                 "全局状态：0x%08lX,驱动状态：%s",
                 (unsigned long)gstat_raw,
                 drvstatus_ok ? "读取成功" : "读取失败");
        /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：电荷泵欠压 处理：停止电机 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                            ERROR_LOG_OP_DRIVER_CHECK,
                            ERROR_LOG_REASON_DRIVER_UV,
                            ERROR_LOG_ACTION_STOP_MOTOR,
                            detail);
        MotorCtrl_InvalidateDriverInit();
        return MOTOR_CHARGE_PUMP_UNDER_VOLTAGE;
    }

    if (reset_flag) {
        if (!stpr_writeInt(tmc5130, TMC5130_GSTAT, 0x07)) { /* 清除复位标志，便于后续重新初始化观察 */
            MotorCtrl_InvalidateDriverInit();
            return MOTOR_TMC_COMM_ERROR;
        }
        snprintf(detail, sizeof(detail),
                 "全局状态：0x%08lX,驱动状态：%s",
                 (unsigned long)gstat_raw,
                 drvstatus_ok ? "读取成功" : "读取失败");
        /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：通信失败 处理：停止电机 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                            ERROR_LOG_OP_DRIVER_CHECK,
                            ERROR_LOG_REASON_COMM_FAIL,
                            ERROR_LOG_ACTION_STOP_MOTOR,
                            detail);
        tmc5130_saveDiagnostic(TMC5130_DIAG_STAGE_CHIP_RESET,
                               TMC5130_DIAG_DIRECTION_READ,
                               TMC5130_GSTAT,
                               0U,
                               (uint32_t)HAL_OK,
                               MOTOR_TMC_CONFIG_LOST,
                               0U,
                               gstat_raw,
                               0,
                               0,
                               0);
        MotorCtrl_InvalidateDriverInit();
        return MOTOR_TMC_CONFIG_LOST;
    }

    return NO_ERROR;
}

/**
 * @brief 检查 TMC5130 驱动功率级是否已经建立实际电流。
 *
 * SPI 和配置寄存器正常不代表电机 24V 已经上电。使能后 CS_ACTUAL 仍为 0 时，
 * 说明实际电流没有建立，按电机被禁止/功率级不可用处理。
 * 该函数会读取寄存器并打印错误报警，只能在任务上下文调用，不能放入中断链路。
 * @return NO_ERROR 表示功率级已建立，否则返回具体电机错误码。
 */
uint32_t stpr_checkDriverPowerReady(TMC5130TypeDef *tmc5130)
{
    int32_t drvstatus = 0;
    uint32_t cs_actual;
    char detail[128];

    /* 读取 DRV_STATUS 是功率级判断的依据，失败时按通信异常处理并失效初始化标记。 */
    if (!stpr_tryReadInt(tmc5130, TMC5130_DRVSTATUS, &drvstatus)) {
        snprintf(detail, sizeof(detail), "驱动状态：读取失败");
        /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：通信失败 处理：停止电机 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                            ERROR_LOG_OP_DRIVER_CHECK,
                            ERROR_LOG_REASON_COMM_FAIL,
                            ERROR_LOG_ACTION_STOP_MOTOR,
                            detail);
        MotorCtrl_InvalidateDriverInit();
        return MOTOR_TMC_COMM_ERROR;
    }

    cs_actual = tmc5130_drvStatusCurrent((uint32_t)drvstatus);
    /* SPI 能读通但 CS_ACTUAL 为 0，说明功率级没有真正建立电流，多数对应 24V 未上电。 */
    if (cs_actual == 0U) {
        printf("TMC5130驱动电流未建立 | DRV_STATUS=0x%08lX | CS_ACTUAL=0\r\n",
               (unsigned long)((uint32_t)drvstatus));
        snprintf(detail, sizeof(detail), "驱动状态：0x%08lX,实际电流档：0",
                 (unsigned long)((uint32_t)drvstatus));
        /* 错误 阶段：错误报警 模块：电机 操作：驱动状态检查 原因：电机被禁止 处理：停止电机 详情：detail */
        ErrorLog_WarnDetail(ERROR_LOG_MODULE_MOTOR,
                            ERROR_LOG_OP_DRIVER_CHECK,
                            ErrorLog_GetReasonByCode(MOTOR_DISABLED),
                            ERROR_LOG_ACTION_STOP_MOTOR,
                            detail);
        MotorCtrl_InvalidateDriverInit();
        return MOTOR_DISABLED;
    }

    return NO_ERROR;
}

/**
 * @brief 执行TMC5130 驱动中的 stpr_waitMove 逻辑。
 *
 * @param tmc5130 业务参数。
 * @return 状态码、计数值或协议数值，具体含义由调用点约定。
 */
uint32_t stpr_waitMove(TMC5130TypeDef *tmc5130)
{
    uint32_t ret;
    int32_t rampstat;

    while (1) {
        /* RAMPSTAT.vzero 只表示当前速度为 0，刚下发命令或换向减速时也可能短暂为 0。
         * 等待位置运动完成必须同时满足 POSREACHED 和 VZERO，并在退出前核对 XACTUAL/XTARGET。 */
        if (!stpr_tryReadInt(tmc5130, TMC5130_RAMPSTAT, &rampstat)) {
            MotorCtrl_SlowStop();
            return MOTOR_TMC_COMM_ERROR;
        }
        if (((rampstat & TMC5130_RS_POSREACHED) != 0) &&
            ((rampstat & TMC5130_RS_VZERO) != 0)) {
            int32_t xactual = 0;
            int32_t xtarget = 0;

            /* POSREACHED 来自驱动内部状态，XACTUAL/XTARGET 是退出前的二次确认。
             * 任何一个位置寄存器读失败，都说明当前无法可靠判断运动完成，必须按通信异常停机。 */
            if (!stpr_tryReadInt(tmc5130, TMC5130_XACTUAL, &xactual) ||
                !stpr_tryReadInt(tmc5130, TMC5130_XTARGET, &xtarget)) {
                MotorCtrl_SlowStop();
                return MOTOR_TMC_COMM_ERROR;
            }
            if (tmc5130_isCloseInt32(xactual, xtarget, TMC5130_WAIT_POS_TOLERANCE_TICKS)) {
                break;
            }

            /* 如果状态位声称已到位，但实际位置差超过容差，继续等待并打印诊断。
             * 这通常指向位置读数异常、目标刚刷新、或驱动状态与位置寄存器不同步。 */
            printf("TMC5130 waitMove未到目标但状态已到位 | RAMPSTAT=0x%08lX | XACTUAL=%ld | XTARGET=%ld\r\n",
                   (unsigned long)((uint32_t)rampstat),
                   (long)xactual,
                   (long)xtarget);
        }
        if (HasEffectiveCommandSwitchRequest()) {
            printf("检测到命令切换请求，停止当前等待运动\r\n");
            MotorCtrl_SlowStop();
            return STATE_SWITCH;
        }

        /* 在运动过程中周期性检查防撞（比如扭力超限等） */
        ret = CheckWeightCollision();
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            MotorCtrl_SlowStop();
            RETURN_ERROR(ret);
        }

        /* 检查 TMC5130 全局状态；读取失败或检测到真实故障时停止运动并把错误交给上层处理 */
        ret = stpr_checkDriverStatus(tmc5130);
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            MotorCtrl_SlowStop();
            RETURN_ERROR(ret);
        }

        /* stpr_waitMove() 是底层直接调用点，不能依赖 MotorDriver_CheckHealth() 追加功率级检查；
         * 这里单独确认 CS_ACTUAL，避免 24V 未上电时仍按普通等待继续运行。 */
        ret = stpr_checkDriverPowerReady(tmc5130);
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            MotorCtrl_SlowStop();
            RETURN_ERROR(ret);
        }

        ret = MotorCtrl_PollRuntimePosition();
        /* 先处理异常边界，避免TMC5130 驱动状态机带故障继续运行。 */
        if (ret != NO_ERROR) {
            MotorCtrl_SlowStop();
            RETURN_ERROR(ret);
        }
        /* TMC5130 驱动与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
        HAL_Delay(50);
    }
    MotorCtrl_RefreshDebugDrumState();
    return NO_ERROR; /* 正常结束运动 */
}
/* *********************** 电流 / 速度 / 初始化 *********************** */

/**
 * @brief  生成 IHOLD_IRUN 寄存器值。
 *
 * 运行时只改变 IRUN；IHOLD 和 IHOLDDELAY 保持统一口径，避免不同入口写出不同电流配置。
 */
static uint32_t tmc5130_buildCurrentSetting(uint8_t current)
{
    return TMC5130_SET_IHOLD(TMC5130_MOTOR_IHOLD_VALUE) |
           TMC5130_SET_IRUN(current) |
           TMC5130_SET_IHOLDDELAY(TMC5130_MOTOR_IHOLDDELAY_VALUE);
}

/**
 * @brief  记录本次下发的 IHOLD_IRUN 电流字段。
 */
static void tmc5130_logCurrentSetting(uint32_t setting)
{
    /* IHOLD_IRUN 是写配置寄存器，IFCNT 在 SPI 模式下禁用。
     * 初始化摘要由 MotorCtrl_Init() 统一打印，底层写寄存器保持安静。 */
    (void)setting;
}
/**
 * @brief  设置驱动电流（IHOLD_IRUN 寄存器）
 *
 * @param  current  运行电流 IRUN 的编码值（0~31）
 */
uint32_t stpr_setCurrent(TMC5130TypeDef *tmc5130, uint8_t current)
{
    const uint32_t value = tmc5130_buildCurrentSetting(current);

    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_IHOLD_IRUN, value);
    tmc5130_logCurrentSetting(value);
    return NO_ERROR;
}

/**
 * @brief  设置最大速度（VMAX）
 */
uint32_t stpr_setVelocity(TMC5130TypeDef *tmc5130, uint32_t velocity)
{
    TMC5130_REQUIRE_WRITE(tmc5130, TMC5130_VMAX, velocity);
    return NO_ERROR;
}

/**
 * @brief  初始化 TMC5130 步进电机驱动器的寄存器参数
 *         包括斩波配置、加减速、速度、stealthChop、StallGuard 等
 *
 * @param  spi      SPI 句柄
 * @param  cs_port  片选 GPIO 端口
 * @param  cs_pin   片选 GPIO 引脚
 * @param  dir      方向配置（如需从 GCONF 中配置方向，可使用）
 * @param  current  电流设置（IRUN）
 */
uint32_t stpr_initStepper(TMC5130TypeDef *tmc5130,
                          SPI_HandleTypeDef *spi,
                          GPIO_TypeDef *cs_port,
                          uint16_t cs_pin,
                          uint8_t dir,
                          uint8_t current)
{
    uint32_t value = 0;
    uint32_t init_write_failed_count = 0U;
/* TMC5130 初始化阶段写寄存器并记录失败状态的宏。 */
#define TMC5130_INIT_WRITE(handle, address, value) \
    tmc5130_initWrite((handle), (address), (value), &init_write_failed_count)

    if ((tmc5130 == NULL) || (spi == NULL) || (cs_port == NULL)) {
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    /* 更新句柄中的硬件资源 */
    tmc5130->spi     = spi;
    tmc5130->cs_pin  = cs_pin;
    tmc5130->cs_port = cs_port;
    (void)dir;

    /* 基础配置：GCONF */
    TMC5130_INIT_WRITE(tmc5130, TMC5130_GCONF, 0x0084);
    /* 若需要根据 dir 设置方向，可改为： */
    /* TMC5130_INIT_WRITE(tmc5130, TMC5130_GCONF, (dir << 4)); */

    /* 空闲功耗降低时间 */
    TMC5130_INIT_WRITE(tmc5130, TMC5130_TPOWERDOWN, 0x00000002); /* 停稳后更快进入保持电流 */

    /* 斩波器配置 CHOPCONF */
    TMC5130_INIT_WRITE(
        tmc5130,
        TMC5130_CHOPCONF,
        (2 << 15) |   /* TBL：死区时间 */
        (0 << 24) |   /* 其他参数可按需调整 */
        (3 << 0)  |   /* TOFF */
        (4 << 7)  |   /* HEND */
        (0 << 4)      /* HSTART */
    );

    /* 基本加减速、速度参数（需根据实际机械系统调优） */
    TMC5130_INIT_WRITE(tmc5130, TMC5130_AMAX,    20 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_A1,      10 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_V1,     800 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_D1,      10 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_DMAX,    20 * 32);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_VSTOP,  0x0000000A);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_VSTART, 0x00000005);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_TZEROWAIT, 10000);

    /* 位置模式 & 初始位置 */
    TMC5130_INIT_WRITE(tmc5130, TMC5130_RAMPMODE, TMC5130_MODE_POSITION);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_XACTUAL,  0x00000000);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_XTARGET,  0x00000000);

    /* 设置 IHOLD / IRUN / IHOLDDELAY */
    value = tmc5130_buildCurrentSetting(current);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_IHOLD_IRUN, value);
    tmc5130_logCurrentSetting(value);

    /* 启用 PWM 模式（stealthChop） */
    TMC5130_INIT_WRITE(tmc5130, TMC5130_GCONF, 0x00000004); /* EN_PWM_MODE=1 */

    /* 速度阈值（PWM、CoolStep 等，这里默认关闭） */
    TMC5130_INIT_WRITE(tmc5130, TMC5130_TPWMTHRS, 0);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_TCOOLTHRS, 0);
    TMC5130_INIT_WRITE(tmc5130, TMC5130_THIGH, 20);

    /* PWM 配置：AUTO=1, Fclk/1024, 振幅限制=200, Grad=1 等 */
    TMC5130_INIT_WRITE(tmc5130, TMC5130_PWMCONF, 0x000401C8);

    /* 清除 GSTAT 错误标志 */
    TMC5130_INIT_WRITE(tmc5130, TMC5130_GSTAT, 0x07);

    if (init_write_failed_count != 0U) {
        printf("TMC5130初始化失败 | 写失败次数：%lu，已停止初始化流程\r\n",
               (unsigned long)init_write_failed_count);
#undef TMC5130_INIT_WRITE
        return MOTOR_TMC_COMM_ERROR;
    }

#undef TMC5130_INIT_WRITE
    return NO_ERROR;
}

#include "ad5421.h"
#include "spi.h"
#include "stdio.h"
#include "main.h"
#include "system_parameter.h"
/* static int32_t SetCurrent(double current); */
static inline void AD5421_CS_LOW(void)
{
    HAL_GPIO_WritePin(AD5421_CS_GPIO_PORT, AD5421_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 执行AD5421 模拟输出中的 AD5421_CS_HIGH 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
static inline void AD5421_CS_HIGH(void)
{
    HAL_GPIO_WritePin(AD5421_CS_GPIO_PORT, AD5421_CS_PIN, GPIO_PIN_SET);
}

#define AD5421_SPI_TIMEOUT_MS 10U /* AD5421 SPI 传输超时时间，单位 ms。 */
#define AD5421_DAC_FULL_SCALE 65535.0f /* AD5421 16 位 DAC 满量程换算系数。 */
static volatile uint32_t ad5421_fault_flags = 0U;
static volatile uint32_t ad5421_fault_register = 0U;
static volatile uint8_t ad5421_trace_suppressed = 0U;
static volatile uint8_t ad5421_access_busy = 0U;
static volatile uint8_t ad5421_sequence_busy = 0U;
static uint8_t ad5421_last_rx_data[3] = {0x00u, 0x00u, 0x00u};
static AD5421DiagnosticSnapshot ad5421_diagnostic = {0U};

/*
 * 函数用途：保存最后一次有效 AD5421 故障现场。
 * 调用场景：SPI 访问、控制回读或故障寄存器检查失败时调用。
 * 关键约束：只保存数值，不打印日志，不改变原有返回码和控制行为。
 */
static void AD5421_SaveDiagnostic(uint8_t stage,
                                  uint8_t direction,
                                  uint8_t reg,
                                  uint32_t hal_status,
                                  uint32_t error_code,
                                  uint32_t root_error_code,
                                  uint32_t expected_value,
                                  uint32_t actual_value)
{
    uint32_t primask;
    uint32_t sequence;

    primask = __get_PRIMASK();
    __disable_irq();
    sequence = ad5421_diagnostic.sequence + 1U;
    if (sequence == 0U) {
        sequence = 1U;
    }
    ad5421_diagnostic.sequence = sequence;
    ad5421_diagnostic.error_code = error_code;
    ad5421_diagnostic.root_error_code = root_error_code;
    ad5421_diagnostic.fault_flags = ad5421_fault_flags;
    ad5421_diagnostic.fault_register = ad5421_fault_register;
    ad5421_diagnostic.hal_status = hal_status;
    ad5421_diagnostic.expected_value = expected_value;
    ad5421_diagnostic.actual_value = actual_value;
    ad5421_diagnostic.stage = stage;
    ad5421_diagnostic.direction = direction;
    ad5421_diagnostic.reg = reg;
    ad5421_diagnostic.reserved = 0U;
    __set_PRIMASK(primask);
}

/*
 * 函数用途：在上层把底层访问错误映射为现有 AD5421 故障码时更新快照。
 * 调用场景：写电流失败和故障寄存器读取失败的兼容返回路径。
 * 关键约束：保留访问阶段、寄存器和 HAL 状态，只更新对外错误码及根因码。
 */
static void AD5421_PromoteDiagnostic(uint32_t error_code, uint32_t root_error_code)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    if (ad5421_diagnostic.sequence != 0U) {
        ad5421_diagnostic.error_code = error_code;
        ad5421_diagnostic.root_error_code = root_error_code;
        ad5421_diagnostic.fault_flags = ad5421_fault_flags;
        ad5421_diagnostic.fault_register = ad5421_fault_register;
    }
    __set_PRIMASK(primask);
}

/*
 * 函数用途：尝试占用 AD5421 SPI 访问窗口。
 * 调用场景：所有 AD5421 公开访问入口和控制寄存器回读序列。
 * 关键约束：只用短临界区保护标志位，不在临界区内访问 SPI。
 */
static uint8_t AD5421_TryBeginAccess(void)
{
    uint32_t primask;
    uint8_t entered = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if (ad5421_access_busy == 0U) {
        ad5421_access_busy = 1U;
        entered = 1U;
    }
    __set_PRIMASK(primask);

    return entered;
}

/*
 * 函数用途：释放 AD5421 SPI 访问窗口。
 * 调用场景：AD5421 访问入口结束后统一调用。
 * 关键约束：只清本模块访问保护标志，不操作硬件片选。
 */
static void AD5421_EndAccess(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    ad5421_access_busy = 0U;
    __set_PRIMASK(primask);
}

/*
 * 函数用途：尝试占用 AD5421 多帧初始化序列。
 * 调用场景：Ad5421Init() 开始复位、控制回读、初始电流写入前调用。
 * 关键约束：只阻止 TIM4 中断插入 AO 刷新，单帧 SPI 访问仍由 access_busy 保护。
 */
static uint8_t AD5421_TryBeginSequence(void)
{
    uint32_t primask;
    uint8_t entered = 0U;

    primask = __get_PRIMASK();
    __disable_irq();
    if ((ad5421_sequence_busy == 0U) && (ad5421_access_busy == 0U)) {
        ad5421_sequence_busy = 1U;
        entered = 1U;
    }
    __set_PRIMASK(primask);

    return entered;
}

/*
 * 函数用途：释放 AD5421 多帧初始化序列。
 * 调用场景：Ad5421Init() 所有出口统一调用。
 * 关键约束：只清序列保护标志，不操作 SPI 和片选。
 */
static void AD5421_EndSequence(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    ad5421_sequence_busy = 0U;
    __set_PRIMASK(primask);
}

/*
 * 函数用途：切换 AD5421 调试打印抑制状态并返回旧状态。
 * 调用场景：TIM4 中断刷新 AO 前抑制 printf，退出中断前恢复。
 * 关键约束：只影响本驱动内部诊断打印，不改变错误码和故障标志。
 */
uint8_t AD5421_SetTraceSuppressed(uint8_t suppress)
{
    uint8_t previous = (uint8_t)ad5421_trace_suppressed;
    ad5421_trace_suppressed = (suppress != 0U) ? 1U : 0U;
    return previous;
}

/*
 * 函数用途：返回 AD5421 当前是否正在访问 SPI。
 * 调用场景：TIM4 AO 刷新前判断是否需要跳过本次中断刷新。
 * 关键约束：只返回软件访问保护状态，不读取 SPI 外设寄存器。
 */
uint8_t AD5421_IsAccessBusy(void)
{
    if ((ad5421_access_busy != 0U) || (ad5421_sequence_busy != 0U)) {
        return 1U;
    }
    return 0U;
}

/*
 * 函数用途：判断 AD5421 诊断打印当前是否允许输出。
 * 调用场景：本驱动内部错误诊断打印前调用。
 * 关键约束：中断刷新 AO 时应返回 0，避免 ISR 直接 printf。
 */
static uint8_t AD5421_CanPrint(void)
{
    return (ad5421_trace_suppressed == 0U) ? 1U : 0U;
}

/* 按 AD5421 READFAULT 高字节位定义返回唯一故障原因。 */
static uint32_t AD5421_MapFaultRegister(uint16_t fault_reg)
{
    /* 多位同时置位时，先返回内部通信和过温关断，再返回环路硬故障，
     * 过温预警优先级最低，避免预警覆盖已经发生的电流或电压异常。 */
    if ((fault_reg & 0xC000U) != 0U) {
        return AD5421_INTERNAL_COMM_ERROR;
    }
    if ((fault_reg & 0x0800U) != 0U) {
        return AD5421_OVERTEMP_SHUTDOWN;
    }
    if ((fault_reg & 0x2000U) != 0U) {
        return AD5421_LOOP_CURRENT_HIGH;
    }
    if ((fault_reg & 0x1000U) != 0U) {
        return AD5421_LOOP_CURRENT_LOW;
    }
    if ((fault_reg & 0x0300U) != 0U) {
        return AD5421_LOOP_VOLTAGE_LOW;
    }
    if ((fault_reg & 0x0400U) != 0U) {
        return AD5421_OVERTEMP_WARNING;
    }
    return NO_ERROR;
}

/*
 * 函数用途：向 AD5421 写入 24 位寄存器数据。
 * 调用场景：AD5421 初始化、复位和 DAC 电流更新。
 * 关键约束：通过 SPI3 阻塞发送，带有限超时，不应在中断中调用。
 */
static uint32_t AD5421_WriteRegRaw(uint8_t reg, uint16_t value)
{
    HAL_StatusTypeDef status;
    uint8_t txData[3];

    txData[0] = reg & 0x7F;
    txData[1] = (uint8_t)((value >> 8) & 0xFFu);
    txData[2] = (uint8_t)(value & 0xFFu);

    AD5421_CS_LOW();
    status = HAL_SPI_Transmit(&hspi3, txData, 3U, AD5421_SPI_TIMEOUT_MS);
    AD5421_CS_HIGH();

    if (status != HAL_OK) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_WRITE;
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_SPI_WRITE,
                              AD5421_DIAG_DIRECTION_WRITE,
                              reg,
                              (uint32_t)status,
                              AD5421_SPI_TRANSFER_ERROR,
                              AD5421_SPI_TRANSFER_ERROR,
                              (uint32_t)value,
                              0U);
        return AD5421_SPI_TRANSFER_ERROR;
    }

    return NO_ERROR;
}

/*
 * 函数用途：按 AD5421 两帧读时序读取寄存器原始值。
 * 调用场景：控制寄存器回读和故障寄存器诊断的底层实现。
 * 关键约束：调用方必须已经持有 access 窗口；本函数只负责 SPI 帧时序和错误标志。
 */
static uint32_t AD5421_ReadRegCheckedRaw(uint8_t reg, uint16_t *value)
{
    HAL_StatusTypeDef status;
    uint8_t commandData[3];
    uint8_t dummyData[3] = {NOOPAD5421, 0x00u, 0x00u};
    uint8_t rxData[3] = {0x00u, 0x00u, 0x00u};

    if (value == NULL) {
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_SPI_READ_COMMAND,
                              AD5421_DIAG_DIRECTION_READ,
                              reg,
                              (uint32_t)HAL_ERROR,
                              SYSTEM_CALL_CONDITION_ERROR,
                              SYSTEM_CALL_CONDITION_ERROR,
                              0U,
                              0U);
        return SYSTEM_CALL_CONDITION_ERROR;
    }

    commandData[0] = reg | 0x80u;
    commandData[1] = 0x00u;
    commandData[2] = 0x00u;

    AD5421_CS_LOW();
    status = HAL_SPI_Transmit(&hspi3, commandData, 3U, AD5421_SPI_TIMEOUT_MS);
    AD5421_CS_HIGH();
    if (status != HAL_OK) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_READ;
        *value = 0U;
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_SPI_READ_COMMAND,
                              AD5421_DIAG_DIRECTION_READ,
                              reg,
                              (uint32_t)status,
                              AD5421_SPI_TRANSFER_ERROR,
                              AD5421_SPI_TRANSFER_ERROR,
                              0U,
                              0U);
        return AD5421_SPI_TRANSFER_ERROR;
    }

    AD5421_CS_LOW();
    status = HAL_SPI_TransmitReceive(&hspi3, dummyData, rxData, 3U, AD5421_SPI_TIMEOUT_MS);
    AD5421_CS_HIGH();
    if (status != HAL_OK) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_READ;
        *value = 0U;
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_SPI_READ_DATA,
                              AD5421_DIAG_DIRECTION_READ,
                              reg,
                              (uint32_t)status,
                              AD5421_SPI_TRANSFER_ERROR,
                              AD5421_SPI_TRANSFER_ERROR,
                              0U,
                              0U);
        return AD5421_SPI_TRANSFER_ERROR;
    }

    ad5421_last_rx_data[0] = rxData[0];
    ad5421_last_rx_data[1] = rxData[1];
    ad5421_last_rx_data[2] = rxData[2];
    *value = ((uint16_t)rxData[1] << 8) | rxData[2];
    return NO_ERROR;
}
/*
 * 函数用途：带 access 保护地写入 AD5421 单个寄存器。
 * 调用场景：旧接口、复位接口和 DAC 原始写入路径。
 * 关键约束：会占用 SPI 访问窗口，不应在中断中调用。
 */
uint32_t AD5421_WriteReg(uint8_t reg, uint16_t value)
{
    uint32_t ret;

    if (AD5421_TryBeginAccess() == 0U) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_WRITE;
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_ACCESS_BUSY,
                              AD5421_DIAG_DIRECTION_WRITE,
                              reg,
                              (uint32_t)HAL_BUSY,
                              AD5421_ACCESS_BUSY,
                              AD5421_ACCESS_BUSY,
                              (uint32_t)value,
                              0U);
        return AD5421_ACCESS_BUSY;
    }

    ret = AD5421_WriteRegRaw(reg, value);
    AD5421_EndAccess();
    return ret;
}

/*
 * 函数用途：从 AD5421 读取寄存器并返回通信状态。
 * 调用场景：控制寄存器回读校验和故障寄存器诊断。
 * 关键约束：通过 SPI3 阻塞收发，带有限超时，不应在中断中调用。
 */
static uint32_t AD5421_ReadRegChecked(uint8_t reg, uint16_t *value)
{
    uint32_t ret;

    if (value == NULL) {
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_SPI_READ_COMMAND,
                              AD5421_DIAG_DIRECTION_READ,
                              reg,
                              (uint32_t)HAL_ERROR,
                              SYSTEM_CALL_CONDITION_ERROR,
                              SYSTEM_CALL_CONDITION_ERROR,
                              0U,
                              0U);
        return SYSTEM_CALL_CONDITION_ERROR;
    }
    if (AD5421_TryBeginAccess() == 0U) {
        *value = 0U;
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_READ;
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_ACCESS_BUSY,
                              AD5421_DIAG_DIRECTION_READ,
                              reg,
                              (uint32_t)HAL_BUSY,
                              AD5421_ACCESS_BUSY,
                              AD5421_ACCESS_BUSY,
                              0U,
                              0U);
        return AD5421_ACCESS_BUSY;
    }

    ret = AD5421_ReadRegCheckedRaw(reg, value);
    AD5421_EndAccess();
    return ret;
}

/*
 * 函数用途：兼容旧接口读取 AD5421 寄存器值。
 * 调用场景：保留给历史状态读取路径。
 * 关键约束：通信失败时返回 0，详细故障通过驱动故障标志查询。
 */
uint16_t AD5421_ReadReg(uint8_t reg)
{
    uint16_t value = 0U;
    (void)AD5421_ReadRegChecked(reg, &value);
    return value;
}

/*
 * 函数用途：直接写入 AD5421 DAC 原始值。
 * 调用场景：电流换算完成后由电流设置接口调用。
 * 关键约束：不做电流范围换算，调用方必须先完成范围约束。
 */
uint32_t AD5421_SetDacRaw(uint16_t value)
{
    return AD5421_WriteReg(WRITEDAC, value);
}

/*
 * 函数用途：按 mA 值换算并写入 AD5421 输出电流。
 * 调用场景：AO 服务需要刷新模拟电流输出时调用。
 * 关键约束：会把输入限制在 AD5421 允许的 3.2-24.0mA 范围内。
 */
uint32_t AD5421_SetCurrent(float mA)
{
    float scale;
    uint16_t dacValue;

    if (mA < (float)STARTFULLSCALE) {
        mA = (float)STARTFULLSCALE;
    }
    if (mA > (float)STOPFULLSCALE) {
        mA = (float)STOPFULLSCALE;
    }

    scale = (mA - (float)STARTFULLSCALE) / ((float)STOPFULLSCALE - (float)STARTFULLSCALE);
    dacValue = (uint16_t)((scale * AD5421_DAC_FULL_SCALE) + 0.5f);

    return AD5421_SetDacRaw(dacValue);
}

/*
 * 函数用途：按 0.01mA 单位设置 AD5421 输出电流。
 * 调用场景：AO 服务使用参数原始单位写入电流。
 * 关键约束：内部转为 mA 后复用 AD5421_SetCurrent()。
 */
uint32_t AD5421_SetCurrentX100(uint32_t mA_x100)
{
    return AD5421_SetCurrent(((float)mA_x100) / 100.0f);
}

/*
 * 函数用途：读取 AD5421 驱动累计故障标志。
 * 调用场景：AO 运行态打包和故障分析。
 * 关键约束：只读内存状态，不访问外设。
 */
uint32_t AD5421_GetFaultFlags(void)
{
    return ad5421_fault_flags;
}

/*
 * 函数用途：读取最近一次 AD5421 READFAULT 原始值。
 * 调用场景：AO 运行态打包和故障分析。
 * 关键约束：只读缓存值，不主动刷新 AD5421。
 */
uint32_t AD5421_GetFaultRegister(void)
{
    return ad5421_fault_register;
}

/*
 * 函数用途：复制最后一次有效 AD5421 故障现场。
 * 调用场景：AO 主循环延后日志和故障注入测试读取。
 * 关键约束：使用短临界区保证快照字段来自同一次故障，不访问 SPI。
 */
void AD5421_GetDiagnosticSnapshot(AD5421DiagnosticSnapshot *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = ad5421_diagnostic;
    __set_PRIMASK(primask);
}

/*
 * 函数用途：轮询 AD5421 故障寄存器。
 * 调用场景：AO 初始化和周期刷新时确认电流环/芯片状态。
 * 关键约束：会访问 SPI，不应在中断中调用；当前 PCB 未接 AD5421 FAULT 引脚。
 */
uint32_t AD5421_PollDiagnostics(void)
{
    uint16_t fault_reg = 0U;
    uint32_t ret;
    uint32_t fault_error;

    ret = AD5421_ReadRegChecked(READFAULT, &fault_reg);
    if (ret != NO_ERROR) {
        return ret;
    }

    ad5421_fault_register = fault_reg;
    ad5421_fault_flags &= ~(AD5421_FAULT_FLAG_PIN | AD5421_FAULT_FLAG_STATUS);
    fault_error = AD5421_MapFaultRegister((uint16_t)(fault_reg & 0xFF00U));
    if (fault_error == NO_ERROR) {
        return NO_ERROR;
    }

    ad5421_fault_flags |= AD5421_FAULT_FLAG_STATUS;
    AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_FAULT_STATUS,
                          AD5421_DIAG_DIRECTION_READ,
                          READFAULT,
                          (uint32_t)HAL_OK,
                          fault_error,
                          fault_error,
                          0U,
                          (uint32_t)fault_reg);
    if (AD5421_CanPrint() != 0U) {
        printf("AD5421 fault diag: fault=0x%04X code=0x%08lX flags=0x%08lX\r\n",
               (unsigned int)fault_reg,
               (unsigned long)fault_error,
               (unsigned long)ad5421_fault_flags);
    }
    return fault_error;
}

/* ============================== */
/* 初始化函数 */
/* ============================== */
void AD5421_Init(void)
{

}

/* ============================== */
/* 软件复位 */
/* ============================== */
void AD5421_Reset(void)
{
	(void)AD5421_WriteReg(RESETAD5421REG, 0x0000);
    /* AD5421 模拟输出与外设通信之间保留等待时间，避免硬件或对端协议尚未准备好。 */
    HAL_Delay(10);
}

/* ============================== */
/* 读取状态寄存器 */
/* ============================== */
uint16_t AD5421_GetStatus(void)
{
    (void)AD5421_PollDiagnostics();
    return (uint16_t)(ad5421_fault_register & 0xFFFFU);
}

/* ============================== */
/* 读取实际输出电流 (诊断值, 单位 mA) */
/* ============================== */
float AD5421_ReadCurrent(void)
{
    (void)AD5421_PollDiagnostics();
    return 0.0f;
}

/**
 * @brief 清除或复位AD5421 模拟输出中的 ResetAD5421 逻辑。
 * @note 无返回值，调用方通过全局状态、外设状态或输出参数获取结果。
 */
void ResetAD5421(void)
{
	(void)AD5421_WriteReg(RESETAD5421REG, 0x0000);
    HAL_Delay(1);
}

/*
 * 函数用途：写入 AD5421 控制寄存器并回读校验。
 * 调用场景：AD5421 初始化时配置 SPI 看门狗模式。
 * 关键约束：会访问 SPI；回读不一致时返回专用读回错误。
 */
static uint32_t WriteControlRegister(uint16_t controldata)
{
    uint32_t ret;
    uint16_t readback = 0U;

    if (AD5421_TryBeginAccess() == 0U) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_READBACK;
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_ACCESS_BUSY,
                              AD5421_DIAG_DIRECTION_WRITE,
                              WRITECONTROL,
                              (uint32_t)HAL_BUSY,
                              AD5421_INIT_ERROR,
                              AD5421_INIT_ERROR,
                              (uint32_t)controldata,
                              0U);
        return AD5421_INIT_ERROR;
    }

    ret = AD5421_WriteRegRaw(WRITECONTROL, controldata);
    if (ret != NO_ERROR) {
        AD5421_EndAccess();
        AD5421_PromoteDiagnostic(AD5421_INIT_ERROR, ret);
        if (AD5421_CanPrint() != 0U) {
            printf("AD5421 control write fail: ret=0x%08lX\r\n", (unsigned long)ret);
        }
        return AD5421_INIT_ERROR;
    }

    ret = AD5421_ReadRegCheckedRaw(READCONTROL, &readback);
    AD5421_EndAccess();
    if (ret != NO_ERROR) {
        AD5421_PromoteDiagnostic(AD5421_INIT_ERROR, ret);
        if (AD5421_CanPrint() != 0U) {
            printf("AD5421 control read fail: ret=0x%08lX\r\n", (unsigned long)ret);
        }
        return AD5421_INIT_ERROR;
    }

    if (readback != controldata) {
        if (AD5421_CanPrint() != 0U) {
            printf("AD5421 control readback mismatch: write=0x%04X read=0x%04X rx=%02X %02X %02X\r\n",
                   (unsigned int)controldata,
                   (unsigned int)readback,
                   (unsigned int)ad5421_last_rx_data[0],
                   (unsigned int)ad5421_last_rx_data[1],
                   (unsigned int)ad5421_last_rx_data[2]);
        }
        ad5421_fault_flags |= AD5421_FAULT_FLAG_READBACK;
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_CONTROL_READBACK,
                              AD5421_DIAG_DIRECTION_READ,
                              READCONTROL,
                              (uint32_t)HAL_OK,
                              AD5421_READBACK_ERROR,
                              AD5421_READBACK_ERROR,
                              (uint32_t)controldata,
                              (uint32_t)readback);
        return AD5421_READBACK_ERROR;
    }

    return NO_ERROR;
}

/*
 * 函数用途：执行 AD5421 复位、控制寄存器回读、目标电流写入和故障诊断公共序列。
 * 调用场景：Ad5421Init() 和 AD5421_RecoverCurrentX100() 共用。
 * 关键约束：会占用 sequence 并访问 SPI/GPIO，不应在中断中调用。
 */
static uint32_t AD5421_RunCurrentStartupSequence(uint32_t target_mA_x100)
{
    uint32_t ret;

    if (AD5421_TryBeginSequence() == 0U) {
        ad5421_fault_flags |= AD5421_FAULT_FLAG_SPI_WRITE;
        AD5421_SaveDiagnostic(AD5421_DIAG_STAGE_ACCESS_BUSY,
                              AD5421_DIAG_DIRECTION_NONE,
                              RESETAD5421REG,
                              (uint32_t)HAL_BUSY,
                              AD5421_INIT_ERROR,
                              AD5421_INIT_ERROR,
                              target_mA_x100,
                              0U);
        return AD5421_INIT_ERROR;
    }

    ad5421_fault_flags = 0U;
    ad5421_fault_register = 0U;

    ret = AD5421_WriteReg(RESETAD5421REG, 0x0000U);
    if (ret != NO_ERROR) {
        AD5421_PromoteDiagnostic(AD5421_INIT_ERROR, ret);
        ret = AD5421_INIT_ERROR;
    } else {
        HAL_Delay(1U);
        ret = WriteControlRegister(CUR_SPIOFF_READBACK_COMMAND);
    }
    if (ret == NO_ERROR) {
        ret = AD5421_SetCurrentX100(target_mA_x100);
    }
    if (ret == NO_ERROR) {
        HAL_Delay(10);
        ret = AD5421_PollDiagnostics();
    }

    AD5421_EndSequence();
    return ret;
}

/*
 * 函数用途：按0.01mA目标复位并初始化AD5421。
 * 调用场景：AO服务根据禁用、固定或上电电流选择初始输出时调用。
 * 关键约束：会访问SPI并等待芯片稳定，不应在中断中调用。
 */
uint32_t AD5421_InitCurrentX100(uint32_t initial_mA_x100)
{
    return AD5421_RunCurrentStartupSequence(initial_mA_x100);
}

/*
 * 函数用途：保留历史无参数初始化接口，供旧测试和调用点兼容使用。
 * 调用场景：尚未迁移到AO服务显式初始电流接口的旧调用点。
 * 关键约束：默认采用当前AO上电电流参数，不改变旧接口返回语义。
 */
uint32_t Ad5421Init(void)
{
    return AD5421_InitCurrentX100(g_deviceParams.ao_output.power_on_current_mA_x100);
}

/*
 * 函数用途：按指定目标电流恢复 AD5421 输出。
 * 调用场景：AO 运行期 READFAULT 异常后的自动恢复。
 * 关键约束：复位芯片后直接写回目标电流，保留控制回读和故障回读诊断。
 */
uint32_t AD5421_RecoverCurrentX100(uint32_t target_mA_x100)
{
    return AD5421_RunCurrentStartupSequence(target_mA_x100);
}




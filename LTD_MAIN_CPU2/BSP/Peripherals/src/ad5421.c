#include "ad5421.h"
#include "spi.h"
#include "stdio.h"
#include "main.h"
#include "system_parameter.h"
/* static int32_t SetCurrent(double current); */
/**
 * @brief 拉低 AD5421 片选信号，开始 SPI 事务。
 */
static inline void AD5421_CS_LOW(void)
{
    HAL_GPIO_WritePin(AD5421_CS_GPIO_PORT, AD5421_CS_PIN, GPIO_PIN_RESET);
}

/**
 * @brief 拉高 AD5421 片选信号，结束 SPI 事务。
 */
static inline void AD5421_CS_HIGH(void)
{
    HAL_GPIO_WritePin(AD5421_CS_GPIO_PORT, AD5421_CS_PIN, GPIO_PIN_SET);
}

#define AD5421_SPI_TIMEOUT_MS 10U /* AD5421 SPI 传输超时时间，单位 ms。 */
#define AD5421_DAC_FULL_SCALE 65535U /* AD5421 16 位 DAC 满量程换算系数。 */
static volatile uint32_t ad5421_fault_flags = 0U;
static volatile uint32_t ad5421_fault_register = 0U;
static volatile uint8_t ad5421_trace_suppressed = 0U;
static volatile uint8_t ad5421_access_busy = 0U;
static volatile uint8_t ad5421_sequence_busy = 0U;
static uint8_t ad5421_last_rx_data[3] = {0x00u, 0x00u, 0x00u};
static AD5421DiagnosticSnapshot ad5421_diagnostic = {0U};

/**
 * @brief 保存最后一次有效 AD5421 故障现场。
 *
 * @details 调用场景：SPI 访问、控制回读或故障寄存器检查失败时调用。
 * @note 关键约束：只保存数值，不打印日志，不改变原有返回码和控制行为。
 *
 * @param stage 外设故障现场的诊断阶段编号；用于区分参数检查、总线访问、寄存器读写、回读核对和设备状态检查等失败位置。
 * @param direction 外设故障现场中的访问方向枚举；写方向、读方向和未指定方向分别使用对应模块的诊断常量编码。
 * @param reg 目标寄存器地址或寄存器值。该 uint8_t 参数是 AD5421 原始寄存器地址，用于 SPI 命令和最近故障现场。
 * @param hal_status 状态。
 * @param error_code 待记录、转换或判断的错误码。该值是本次 AD5421 访问对外返回的整机错误码，并写入最近诊断快照。
 * @param root_error_code 根因故障。
 * @param expected_value 期望数值。
 * @param actual_value 实际数值。
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

/**
 * @brief 在上层把底层访问错误映射为现有 AD5421 故障码时更新快照。
 *
 * @details 调用场景：写电流失败和故障寄存器读取失败的兼容返回路径。
 * @note 关键约束：保留访问阶段、寄存器和 HAL 状态，只更新对外错误码及根因码。
 *
 * @param error_code 待记录、转换或判断的错误码。该值是本次 AD5421 访问对外返回的整机错误码，并写入最近诊断快照。
 * @param root_error_code 根因故障。
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

/**
 * @brief 尝试占用 AD5421 SPI 访问窗口。
 *
 * @details 调用场景：所有 AD5421 公开访问入口和控制寄存器回读序列。
 * @note 关键约束：只用短临界区保护标志位，不在临界区内访问 SPI。
 *
 * @return 1 表示已独占 AD5421 SPI 单帧访问窗口；已有访问或初始化序列占用时返回 0。
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

/**
 * @brief 释放 AD5421 SPI 访问窗口。
 *
 * @details 调用场景：AD5421 访问入口结束后统一调用。
 * @note 关键约束：只清本模块访问保护标志，不操作硬件片选。
 */
static void AD5421_EndAccess(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    ad5421_access_busy = 0U;
    __set_PRIMASK(primask);
}

/**
 * @brief 尝试占用 AD5421 多帧初始化序列。
 *
 * @details 调用场景：Ad5421Init() 开始复位、控制回读、初始电流写入前调用。
 * @note 关键约束：只阻止 TIM4 中断插入 AO 刷新，单帧 SPI 访问仍由 access_busy 保护。
 *
 * @return 1 表示已独占 AD5421 多帧初始化序列；SPI 正忙或已有序列执行时返回 0。
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

/**
 * @brief 释放 AD5421 多帧初始化序列。
 *
 * @details 调用场景：Ad5421Init() 所有出口统一调用。
 * @note 关键约束：只清序列保护标志，不操作 SPI 和片选。
 */
static void AD5421_EndSequence(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    ad5421_sequence_busy = 0U;
    __set_PRIMASK(primask);
}

/**
 * @brief 切换 AD5421 调试打印抑制状态并返回旧状态。
 *
 * @details 调用场景：TIM4 中断刷新 AO 前抑制 printf，退出中断前恢复。
 * @note 关键约束：只影响本驱动内部诊断打印，不改变错误码和故障标志。
 *
 * @param suppress true 表示暂时抑制 AD5421 逐次跟踪日志，false 表示恢复打印。
 * @return 返回切换前的调试打印抑制状态，供调用方退出临时静默区时恢复。
 */
uint8_t AD5421_SetTraceSuppressed(uint8_t suppress)
{
    uint8_t previous = (uint8_t)ad5421_trace_suppressed;
    ad5421_trace_suppressed = (suppress != 0U) ? 1U : 0U;
    return previous;
}

/**
 * @brief 返回 AD5421 当前是否正在访问 SPI。
 *
 * @details 调用场景：TIM4 AO 刷新前判断是否需要跳过本次中断刷新。
 * @note 关键约束：只返回软件访问保护状态，不读取 SPI 外设寄存器。
 *
 * @return 1 表示寄存器访问或分阶段输出序列正在占用 AD5421 SPI；0 表示两类访问均处于空闲状态。
 */
uint8_t AD5421_IsAccessBusy(void)
{
    if ((ad5421_access_busy != 0U) || (ad5421_sequence_busy != 0U)) {
        return 1U;
    }
    return 0U;
}

/**
 * @brief 判断 AD5421 诊断打印当前是否允许输出。
 *
 * @details 调用场景：本驱动内部错误诊断打印前调用。
 * @note 关键约束：中断刷新 AO 时应返回 0，避免 ISR 直接 printf。
 *
 * @return 1 表示 AD5421 诊断打印当前允许输出；0 表示 AD5421 诊断打印当前不允许输出。
 */
static uint8_t AD5421_CanPrint(void)
{
    return (ad5421_trace_suppressed == 0U) ? 1U : 0U;
}

/**
 * @brief 按 AD5421 READFAULT 高字节位定义返回唯一故障原因。
 *
 * @param fault_reg 故障。
 * @return 返回 READFAULT 首个命中位对应的整机故障码；寄存器无故障位时返回 NO_ERROR。
 */
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

/**
 * @brief 向 AD5421 写入 24 位寄存器数据。
 *
 * @details 调用场景：AD5421 初始化、复位和 DAC 电流更新。
 * @note 关键约束：通过 SPI3 阻塞发送，带有限超时，不应在中断中调用。
 *
 * @param reg 目标寄存器地址或寄存器值。该 uint8_t 参数是 AD5421 原始寄存器地址，用于 SPI 命令和最近故障现场。
 * @param value 写入原始使用的输入数值。
 * @return NO_ERROR 表示 24 位寄存器帧已完整发送；SPI 访问失败返回 AD5421_SPI_TRANSFER_ERROR，并在诊断快照中保留寄存器、阶段和 HAL 状态。
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

/**
 * @brief 按 AD5421 两帧读时序读取寄存器原始值。
 *
 * @details 调用场景：控制寄存器回读和故障寄存器诊断的底层实现。
 * @note 关键约束：调用方必须已经持有 access 窗口；本函数只负责 SPI 帧时序和错误标志。
 *
 * @param reg 目标寄存器地址或寄存器值。该 uint8_t 参数是 AD5421 原始寄存器地址，用于 SPI 命令和最近故障现场。
 * @param value 用于返回完成通信校验后的 AD5421 寄存器原始值。
 * @return SYSTEM_CALL_CONDITION_ERROR 表示当前系统状态不允许执行；NO_ERROR 表示操作成功。
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
/**
 * @brief 带 access 保护地写入 AD5421 单个寄存器。
 *
 * @details 调用场景：旧接口、复位接口和 DAC 原始写入路径。
 * @note 关键约束：会占用 SPI 访问窗口，不应在中断中调用。
 *
 * @param reg 目标寄存器地址或寄存器值。该 uint8_t 参数是 AD5421 原始寄存器地址，用于 SPI 命令和最近故障现场。
 * @param value 写入使用的输入数值。
 * @return 返回整机错误码；NO_ERROR 表示寄存器写入成功，AD5421_ACCESS_BUSY 表示访问窗口被占用，其他值透传 SPI 或回读校验失败。
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

/**
 * @brief 从 AD5421 读取寄存器并返回通信状态。
 *
 * @details 调用场景：控制寄存器回读校验和故障寄存器诊断。
 * @note 关键约束：通过 SPI3 阻塞收发，带有限超时，不应在中断中调用。
 *
 * @param reg 目标寄存器地址或寄存器值。该 uint8_t 参数是 AD5421 原始寄存器地址，用于 SPI 命令和最近故障现场。
 * @param value 用于返回完成通信及状态校验后的 AD5421 寄存器值。
 * @return NO_ERROR 表示寄存器读取和故障状态检查均通过；空输出指针返回 SYSTEM_CALL_CONDITION_ERROR，SPI 仲裁被占用返回 AD5421_ACCESS_BUSY，其他值为读命令、读数据或器件状态错误。
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

/**
 * @brief 兼容旧接口读取 AD5421 寄存器值。
 *
 * @details 调用场景：保留给历史状态读取路径。
 * @note 关键约束：通信失败时返回 0，详细故障通过驱动故障标志查询。
 *
 * @param reg 目标寄存器地址或寄存器值。该 uint8_t 参数是 AD5421 原始寄存器地址，用于 SPI 命令和最近故障现场。
 * @return 返回 AD5421 目标寄存器的 16 位值；兼容接口无法向调用方传播读取失败，失败详情保留在诊断状态中。
 */
uint16_t AD5421_ReadReg(uint8_t reg)
{
    uint16_t value = 0U;
    (void)AD5421_ReadRegChecked(reg, &value);
    return value;
}

/**
 * @brief 直接写入 AD5421 DAC 原始值。
 *
 * @details 调用场景：电流换算完成后由电流设置接口调用。
 * @note 关键约束：不做电流范围换算，调用方必须先完成范围约束。
 *
 * @param value 设置原始使用的输入数值。
 * @return 返回整机错误码；NO_ERROR 表示 DAC 原始值已写入，其他值透传 AD5421 访问或通信失败。
 */
uint32_t AD5421_SetDacRaw(uint16_t value)
{
    return AD5421_WriteReg(WRITEDAC, value);
}

/**
 * @brief 按 mA 值换算并写入 AD5421 输出电流。
 *
 * @details 调用场景：AO 服务需要刷新模拟电流输出时调用。
 * @note 关键约束：会把输入限制在 AD5421 允许的 3.2-24.0mA 范围内。
 *
 * @param mA 目标模拟输出电流，单位 mA。
 * @return 返回整机错误码；NO_ERROR 表示目标 mA 已换算并写入 DAC，其他值表示范围或 AD5421 通信失败。
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

/**
 * @brief 按0.001mA单位换算并写入AD5421输出电流。
 *
 * @details 调用场景：AO服务完成基础电流和修正量合成后，以整数路径写入最终目标。
 * @note 关键约束：使用64位乘法和整数四舍五入，避免浮点换算吞掉0.001mA修正步进。
 *
 * @param mA_x1000 准备写入AD5421的目标电流，单位0.001mA。
 * @return 返回整机错误码；NO_ERROR表示目标已换算并写入DAC，其他值透传驱动错误。
 */
uint32_t AD5421_SetCurrentX1000(uint32_t mA_x1000)
{
    uint32_t offset_mA_x1000;
    uint32_t span_mA_x1000 = STOPFULLSCALE_MA_X1000 - STARTFULLSCALE_MA_X1000;
    uint16_t dac_value;

    if (mA_x1000 < STARTFULLSCALE_MA_X1000) {
        mA_x1000 = STARTFULLSCALE_MA_X1000;
    }
    if (mA_x1000 > STOPFULLSCALE_MA_X1000) {
        mA_x1000 = STOPFULLSCALE_MA_X1000;
    }
    offset_mA_x1000 = mA_x1000 - STARTFULLSCALE_MA_X1000;
    dac_value = (uint16_t)((((uint64_t)offset_mA_x1000 * AD5421_DAC_FULL_SCALE) +
                            ((uint64_t)span_mA_x1000 / 2U)) /
                           (uint64_t)span_mA_x1000);
    return AD5421_SetDacRaw(dac_value);
}

/**
 * @brief 保留0.01mA旧接口并转调x1000整数输出路径。
 *
 * @param mA_x100 准备写入AD5421的目标电流，单位0.01mA。
 * @return 返回整机错误码；缩放后透传x1000接口结果。
 */
uint32_t AD5421_SetCurrentX100(uint32_t mA_x100)
{
    uint64_t scaled_mA_x1000 = (uint64_t)mA_x100 * 10U;

    if (scaled_mA_x1000 > UINT32_MAX) {
        scaled_mA_x1000 = UINT32_MAX;
    }
    return AD5421_SetCurrentX1000((uint32_t)scaled_mA_x1000);
}

/**
 * @brief 读取 AD5421 驱动累计故障标志。
 *
 * @details 调用场景：AO 运行态打包和故障分析。
 * @note 关键约束：只读内存状态，不访问外设。
 *
 * @return 返回 AD5421 驱动累计故障标志对应的位掩码；各位含义由相邻枚举或宏定义。
 */
uint32_t AD5421_GetFaultFlags(void)
{
    return ad5421_fault_flags;
}

/**
 * @brief 读取最近一次 AD5421 READFAULT 原始值。
 *
 * @details 调用场景：AO 运行态打包和故障分析。
 * @note 关键约束：只读缓存值，不主动刷新 AD5421。
 *
 * @return 返回最近一次成功读取并缓存的 AD5421 READFAULT 原始寄存器位图。
 */
uint32_t AD5421_GetFaultRegister(void)
{
    return ad5421_fault_register;
}

/**
 * @brief 复制最后一次有效 AD5421 故障现场。
 *
 * @details 调用场景：AO 主循环延后日志和故障注入测试读取。
 * @note 关键约束：使用短临界区保证快照字段来自同一次故障，不访问 SPI。
 *
 * @param snapshot AD5421 最近一次有效故障现场输出对象；写入序号、错误码、故障寄存器、HAL 状态、阶段、方向以及期望值和实测值。
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

/**
 * @brief 轮询 AD5421 故障寄存器。
 *
 * @details 调用场景：AO 初始化和周期刷新时确认电流环/芯片状态。
 * @note 关键约束：会访问 SPI，不应在中断中调用；当前 PCB 未接 AD5421 FAULT 引脚。
 *
 * @return NO_ERROR 表示故障寄存器可读且未报告有效故障；SPI 读取失败返回访问错误，寄存器置位时返回映射后的 AD5421 供电、电流环、温度或通信故障码。
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
/**
 * @brief 兼容保留的空初始化入口，当前不执行 AD5421 硬件操作。
 */
void AD5421_Init(void)
{

}

/* ============================== */
/* 软件复位 */
/* ============================== */
/**
 * @brief 写 AD5421 复位寄存器并等待 10 ms 稳定。
 */
void AD5421_Reset(void)
{
	(void)AD5421_WriteReg(RESETAD5421REG, 0x0000);
    /* 复位寄存器写入后等待 10 ms，让 AD5421 完成内部复位，再允许后续寄存器访问。 */
    HAL_Delay(10);
}

/* ============================== */
/* 读取状态寄存器 */
/* ============================== */
/**
 * @brief 轮询 AD5421 故障寄存器后返回缓存状态；当前接口不传播轮询失败码。
 *
 * @return 返回最近一次轮询后缓存的 AD5421 READFAULT 低 16 位；轮询失败不通过本返回值传播。
 */
uint16_t AD5421_GetStatus(void)
{
    (void)AD5421_PollDiagnostics();
    return (uint16_t)(ad5421_fault_register & 0xFFFFU);
}

/* ============================== */
/* 读取实际输出电流 (诊断值, 单位 mA) */
/* ============================== */
/**
 * @brief 兼容诊断接口：当前只刷新 AD5421 故障状态并固定返回 0.0 mA。
 *
 * @return 固定返回 0.0 mA；调用前仅刷新 AD5421 诊断故障状态，当前接口不会读取或估算真实环路电流。
 */
float AD5421_ReadCurrent(void)
{
    (void)AD5421_PollDiagnostics();
    return 0.0f;
}

/**
 * @brief 旧兼容复位接口：写复位寄存器后等待 1 ms，且不传播写入返回码。
 */
void ResetAD5421(void)
{
	(void)AD5421_WriteReg(RESETAD5421REG, 0x0000);
    HAL_Delay(1);
}

/**
 * @brief 写入 AD5421 控制寄存器并回读校验。
 *
 * @details 调用场景：AD5421 初始化时配置 SPI 看门狗模式。
 * @note 关键约束：会访问 SPI；回读不一致时返回专用读回错误。
 *
 * @param controldata 待写入 AD5421 控制寄存器的位字段原始值。
 * @return NO_ERROR 表示控制寄存器写入及回读完全一致；写入失败返回 AD5421_INIT_ERROR，回读值不匹配返回 AD5421_READBACK_ERROR。
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

/**
 * @brief 执行 AD5421 复位、控制寄存器回读、目标电流写入和故障诊断公共序列。
 *
 * @details 调用场景：Ad5421Init() 和 AD5421_RecoverCurrentX1000() 共用。
 * @note 关键约束：会占用 sequence 并访问 SPI/GPIO，不应在中断中调用。
 *
 * @param target_mA_x1000 目标电流定点值，单位 0.001 mA。
 * @return NO_ERROR 表示复位、控制寄存器回读、目标电流写入和最终诊断均通过；复位或参数阶段失败返回 AD5421_INIT_ERROR，其他值保留具体 SPI、回读或器件故障码。
 */
static uint32_t AD5421_RunCurrentStartupSequenceX1000(uint32_t target_mA_x1000)
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
                              target_mA_x1000,
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
        ret = AD5421_SetCurrentX1000(target_mA_x1000);
    }
    if (ret == NO_ERROR) {
        HAL_Delay(10);
        ret = AD5421_PollDiagnostics();
    }

    AD5421_EndSequence();
    return ret;
}

/**
 * @brief 按0.01mA目标复位并初始化AD5421。
 *
 * @details 调用场景：AO服务根据禁用、固定或上电电流选择初始输出时调用。
 * @note 关键约束：会访问SPI并等待芯片稳定，不应在中断中调用。
 *
 * @param initial_mA_x100 AD5421 初始化完成后准备输出的起始电流，单位 0.01 mA。
 * @return 返回整机错误码；NO_ERROR 表示 AD5421 已复位、初始化并输出目标电流，其他值标识失败阶段。
 */
uint32_t AD5421_InitCurrentX100(uint32_t initial_mA_x100)
{
    uint64_t scaled_mA_x1000 = (uint64_t)initial_mA_x100 * 10U;

    if (scaled_mA_x1000 > UINT32_MAX) {
        scaled_mA_x1000 = UINT32_MAX;
    }
    return AD5421_RunCurrentStartupSequenceX1000((uint32_t)scaled_mA_x1000);
}

/**
 * @brief 按0.001mA目标复位、初始化并写入AD5421。
 *
 * @param initial_mA_x1000 初始化完成后准备输出的起始电流，单位0.001mA。
 * @return 返回整机错误码；NO_ERROR表示完整启动序列成功。
 */
uint32_t AD5421_InitCurrentX1000(uint32_t initial_mA_x1000)
{
    return AD5421_RunCurrentStartupSequenceX1000(initial_mA_x1000);
}


/**
 * @brief 保留历史无参数初始化接口，供旧测试和调用点兼容使用。
 *
 * @details 调用场景：尚未迁移到AO服务显式初始电流接口的旧调用点。
 * @note 关键约束：默认采用当前AO上电电流参数，不改变旧接口返回语义。
 *
 * @return 返回整机错误码；NO_ERROR 表示已按上电默认电流完成 AD5421 初始化，其他值标识初始化失败阶段。
 */
uint32_t Ad5421Init(void)
{
    return AD5421_InitCurrentX100(g_deviceParams.ao_output.power_on_current_mA_x100);
}

/**
 * @brief 按指定目标电流恢复 AD5421 输出。
 *
 * @details 调用场景：AO 运行期 READFAULT 异常后的自动恢复。
 * @note 关键约束：复位芯片后直接写回目标电流，保留控制回读和故障回读诊断。
 *
 * @param target_mA_x100 目标电流定点值，单位 0.01 mA。
 * @return 返回整机错误码；NO_ERROR 表示 AD5421 已恢复目标电流，其他值标识复位、初始化或写入失败。
 */
uint32_t AD5421_RecoverCurrentX100(uint32_t target_mA_x100)
{
    uint64_t scaled_mA_x1000 = (uint64_t)target_mA_x100 * 10U;

    if (scaled_mA_x1000 > UINT32_MAX) {
        scaled_mA_x1000 = UINT32_MAX;
    }
    return AD5421_RunCurrentStartupSequenceX1000((uint32_t)scaled_mA_x1000);
}

/**
 * @brief 按0.001mA目标执行AD5421运行期恢复序列。
 *
 * @param target_mA_x1000 恢复后准备输出的目标电流，单位0.001mA。
 * @return 返回整机错误码；NO_ERROR表示复位、回读、写电流和诊断均成功。
 */
uint32_t AD5421_RecoverCurrentX1000(uint32_t target_mA_x1000)
{
    return AD5421_RunCurrentStartupSequenceX1000(target_mA_x1000);
}




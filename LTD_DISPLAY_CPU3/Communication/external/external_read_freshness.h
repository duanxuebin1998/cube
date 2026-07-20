#ifndef CPU3_EXTERNAL_READ_FRESHNESS_H_
#define CPU3_EXTERNAL_READ_FRESHNESS_H_

#include <stdint.h>

#define CPU3_EXTERNAL_WARTSILA_POINT_BASE       0x0064U
#define CPU3_EXTERNAL_WARTSILA_POINT_STRIDE     6U
#define CPU3_EXTERNAL_WARTSILA_POINT_COUNT      100U
#define CPU3_EXTERNAL_WARTSILA_POINT_WORD_COUNT \
    (CPU3_EXTERNAL_WARTSILA_POINT_STRIDE * CPU3_EXTERNAL_WARTSILA_POINT_COUNT)
#define CPU3_EXTERNAL_WARTSILA_SIGNATURE_ADDRESS  0x0672U
#define CPU3_EXTERNAL_WARTSILA_SIGNATURE_QUANTITY 20U

typedef enum
{
    CPU3_EXTERNAL_READ_LOCAL = 0U,
    CPU3_EXTERNAL_READ_RUNTIME = 1U,
    CPU3_EXTERNAL_READ_PARAMETERS = 2U
} Cpu3ExternalReadRequirement;

/*
 * 函数用途：判断 DSM 输入寄存器是否属于 CPU3 本地静态离线白名单。
 * 调用场景：DSM FC04 完成地址和数量校验后逐寄存器分类。
 * 关键约束：0x0001 依赖 CPU2 生命周期，不得作为本地静态状态返回。
 */
static inline uint8_t CPU3_ExternalDsmInputIsLocal(uint16_t address)
{
    if ((address == 0x0000U) ||
        (address == 0x0006U) ||
        (address == 0x000FU) ||
        (address == 0x0018U) ||
        ((address >= 0x0028U) && (address <= 0x002BU)) ||
        ((address >= 0x0100U) && (address <= 0x0103U)) ||
        ((address >= 0x010EU) && (address <= 0x0111U)))
    {
        return 1U;
    }

    return 0U;
}

/*
 * 函数用途：判断 DSM FC04 整帧是否包含 CPU2 派生运行态字段。
 * 调用场景：读取输入寄存器影子和刷新 Input_Write 之前调用。
 * 关键约束：混合本地与 CPU2 字段的请求按整帧运行态门禁处理。
 */
static inline uint8_t CPU3_ExternalDsmInputRangeNeedsRuntime(uint16_t start,
                                                             uint16_t quantity)
{
    uint32_t index;

    for (index = 0U; index < (uint32_t)quantity; ++index)
    {
        uint16_t address = (uint16_t)((uint32_t)start + index);

        if (CPU3_ExternalDsmInputIsLocal(address) == 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

/*
 * 函数用途：判断 SI FC01/FC02 的目标位是否依赖 CPU2 运行态快照。
 * 调用场景：SI 位读取完成功能码、地址和数量校验后逐位分类。
 * 关键约束：未列入本地静态白名单的位保守地按 CPU2 派生位处理。
 */
static inline uint8_t CPU3_ExternalSiBitNeedsRuntime(uint8_t function,
                                                     uint16_t offset)
{
    if (function == 0x01U)
    {
        return (((offset <= 3U) ||
                 ((offset >= 8U) && (offset <= 14U))) ? 1U : 0U);
    }

    if (function == 0x02U)
    {
        return (((offset == 0U) ||
                 ((offset >= 2U) && (offset <= 4U)) ||
                 (offset == 8U) ||
                 (offset == 12U) ||
                 ((offset >= 16U) && (offset <= 25U)) ||
                 ((offset >= 28U) && (offset <= 31U))) ? 1U : 0U);
    }

    return 1U;
}

/*
 * 函数用途：判断 SI FC01/FC02 整帧是否包含 CPU2 派生位。
 * 调用场景：位池打包前执行一次整帧新鲜度门禁。
 * 关键约束：请求跨越本地与 CPU2 位时不得部分返回旧影子。
 */
static inline uint8_t CPU3_ExternalSiBitRangeNeedsRuntime(uint8_t function,
                                                          uint16_t start,
                                                          uint16_t quantity)
{
    uint32_t index;

    for (index = 0U; index < (uint32_t)quantity; ++index)
    {
        uint16_t offset = (uint16_t)((uint32_t)start + index);

        if (CPU3_ExternalSiBitNeedsRuntime(function, offset) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

/*
 * 函数用途：判断 SI FC04 输入寄存器是否依赖 CPU2 运行态快照。
 * 调用场景：SI 输入寄存器响应复制前逐 offset 分类。
 * 关键约束：Profile 时间、镜像和点阵均随 CPU2 生命周期失效。
 */
static inline uint8_t CPU3_ExternalSiInputNeedsRuntime(uint16_t offset)
{
    if ((offset <= 3U) ||
        ((offset >= 5U) && (offset <= 9U)) ||
        ((offset >= 13U) && (offset <= 15U)) ||
        ((offset >= 20U) && (offset <= 619U)))
    {
        return 1U;
    }

    return 0U;
}

/*
 * 函数用途：判断 SI FC04 整帧是否包含 CPU2 派生寄存器。
 * 调用场景：输入寄存器池序列化前执行一次整帧新鲜度门禁。
 * 关键约束：混合范围必须整体返回 Busy，不得拼接本地与旧 CPU2 数据。
 */
static inline uint8_t CPU3_ExternalSiInputRangeNeedsRuntime(uint16_t start,
                                                            uint16_t quantity)
{
    uint32_t index;

    for (index = 0U; index < (uint32_t)quantity; ++index)
    {
        uint16_t offset = (uint16_t)((uint32_t)start + index);

        if (CPU3_ExternalSiInputNeedsRuntime(offset) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

/*
 * 函数用途：分类一个 Wärtsilä FC03 保持寄存器的新鲜度要求。
 * 调用场景：合法读取范围在复制寄存器影子前逐项分类。
 * 关键约束：未列入静态白名单的兼容空洞保守地要求运行态快照。
 */
static inline Cpu3ExternalReadRequirement
CPU3_ExternalWartsilaRegisterRequirement(uint16_t address)
{
    uint32_t point_end = (uint32_t)CPU3_EXTERNAL_WARTSILA_POINT_BASE +
                         (uint32_t)CPU3_EXTERNAL_WARTSILA_POINT_WORD_COUNT;

    if ((address >= 0x005AU) && (address <= 0x005DU))
    {
        return CPU3_EXTERNAL_READ_PARAMETERS;
    }

    if ((address == 0x0000U) ||
        (address == 0x0002U) ||
        (address == 0x0004U) ||
        (address == 0x0007U) ||
        (address == 0x0009U) ||
        (address == 0x000BU) ||
        (address == 0x000DU) ||
        (address == 0x0050U) ||
        (address == 0x0051U))
    {
        return CPU3_EXTERNAL_READ_RUNTIME;
    }

    if ((address == 0x0001U) ||
        (address == 0x0003U) ||
        (address == 0x0005U) ||
        (address == 0x0006U) ||
        (address == 0x0008U) ||
        (address == 0x000AU) ||
        (address == 0x000CU) ||
        (address == 0x000EU) ||
        (address == 0x000FU) ||
        (address == 0x0052U) ||
        (address == 0x0053U))
    {
        return CPU3_EXTERNAL_READ_LOCAL;
    }

    if (((uint32_t)address >= (uint32_t)CPU3_EXTERNAL_WARTSILA_POINT_BASE) &&
        ((uint32_t)address < point_end))
    {
        uint16_t lane = (uint16_t)(((uint32_t)address -
                                    (uint32_t)CPU3_EXTERNAL_WARTSILA_POINT_BASE) %
                                   (uint32_t)CPU3_EXTERNAL_WARTSILA_POINT_STRIDE);

        return (lane <= 2U) ? CPU3_EXTERNAL_READ_RUNTIME : CPU3_EXTERNAL_READ_LOCAL;
    }

    return CPU3_EXTERNAL_READ_RUNTIME;
}

/*
 * 函数用途：汇总 Wärtsilä FC03 整帧所需的运行态和参数门禁。
 * 调用场景：合法范围读取寄存器影子前调用。
 * 关键约束：返回值为位组合，混合范围必须同时满足全部要求。
 */
static inline uint8_t CPU3_ExternalWartsilaReadRequirements(uint16_t start,
                                                            uint16_t quantity)
{
    uint8_t requirements = (uint8_t)CPU3_EXTERNAL_READ_LOCAL;
    uint32_t index;

    for (index = 0U; index < (uint32_t)quantity; ++index)
    {
        uint16_t address = (uint16_t)((uint32_t)start + index);

        requirements |= (uint8_t)CPU3_ExternalWartsilaRegisterRequirement(address);
    }

    return requirements;
}

/*
 * 函数用途：把 CPU2 发布点数限制到 Wärtsilä 第一版可靠容量。
 * 调用场景：投影点数和复制点表前调用。
 * 关键约束：101 至 200 点只受控截断到前 100 点，不得越界访问结构尾部。
 */
static inline uint16_t CPU3_ExternalWartsilaClampPointCount(uint32_t point_count)
{
    return (point_count > CPU3_EXTERNAL_WARTSILA_POINT_COUNT) ?
           (uint16_t)CPU3_EXTERNAL_WARTSILA_POINT_COUNT :
           (uint16_t)point_count;
}

/*
 * 函数用途：判断 Wärtsilä 版本签名读取是否命中特殊地址。
 * 调用场景：FC03 在普通保持寄存器边界检查前识别签名请求。
 * 关键约束：只有 0x0672 使用签名响应，其余地址必须走普通寄存器规则。
 */
static inline uint8_t CPU3_ExternalWartsilaIsSignatureAddress(uint16_t start)
{
    return (start == CPU3_EXTERNAL_WARTSILA_SIGNATURE_ADDRESS) ? 1U : 0U;
}

/*
 * 函数用途：校验 Wärtsilä 版本签名请求的固定寄存器数量。
 * 调用场景：FC03 命中特殊签名地址后、复制 40 字节签名前调用。
 * 关键约束：响应固定为 20 个寄存器，任何其它数量均不得成功回包。
 */
static inline uint8_t CPU3_ExternalWartsilaSignatureQuantityIsValid(uint16_t quantity)
{
    return (quantity == CPU3_EXTERNAL_WARTSILA_SIGNATURE_QUANTITY) ? 1U : 0U;
}

/*
 * 函数用途：判断 Wärtsilä 下发命令是否具有已实现且可确认的 CPU2 语义。
 * 调用场景：FC10 覆盖命令寄存器时，在写入影子和下发 CPU2 前调用。
 * 关键约束：0 仅用于清空命令；未知命令必须返回非法数据值。
 */
static inline uint8_t CPU3_ExternalWartsilaCommandIsSupported(uint16_t command)
{
    return ((command == 0U) ||
            (command == 3U) ||
            (command == 4U) ||
            (command == 5U)) ? 1U : 0U;
}

/*
 * 【Wärtsilä 莆田现场专用格式，禁止回退为标准 Modbus FC10】
 * 函数用途：校验 Wärtsilä FC10 的数量、字节数和 PDU 长度关系。
 * 调用场景：FC10 解析头部后、访问数据区或寄存器池之前调用。
 * 关键约束：所有 Wärtsilä FC10 指令仅接受 byteCount=quantity，数据区仍为
 *           2*quantity 字节；标准 Modbus 的 byteCount=2*quantity 必须拒绝。
 *           PDU 长度必须严格等于 6+2*quantity，不能按现场 byteCount 字段
 *           直接推导数据区长度。FC03 响应不使用本特例，
 *           byteCount 仍为 2*quantity。
 */
static inline uint8_t CPU3_ExternalWartsilaWriteShapeIsValid(uint16_t quantity,
                                                             uint8_t byte_count,
                                                             uint16_t pdu_length)
{
    uint16_t expected_data_bytes;

    if ((quantity < 1U) || (quantity > 0x007BU))
    {
        return 0U;
    }

    expected_data_bytes = (uint16_t)(quantity * 2U);
    if ((uint16_t)byte_count != quantity)
    {
        return 0U;
    }

    return (pdu_length == (uint16_t)(6U + expected_data_bytes)) ? 1U : 0U;
}

/*
 * 函数用途：清空 Wärtsilä 第一版全部点表寄存器。
 * 调用场景：每次从本轮 CPU2 快照投影点表之前调用。
 * 关键约束：清空后只填充本轮有效点，长轮次尾部不得泄漏到短轮次。
 */
static inline void CPU3_ExternalWartsilaClearPointRegisters(uint16_t *point_registers)
{
    uint32_t index;

    if (point_registers == (uint16_t *)0)
    {
        return;
    }

    for (index = 0U; index < (uint32_t)CPU3_EXTERNAL_WARTSILA_POINT_WORD_COUNT; ++index)
    {
        point_registers[index] = 0U;
    }
}

#endif /* CPU3_EXTERNAL_READ_FRESHNESS_H_ */

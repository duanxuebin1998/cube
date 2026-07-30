#ifndef CPU3_EXTERNAL_READ_FRESHNESS_H_
/* CPU3_EXTERNAL_READ_FRESHNESS_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define CPU3_EXTERNAL_READ_FRESHNESS_H_

#include <stdint.h>

/*
 * CPU3 读取 Wartsila 密度点阵时使用的统一布局：起始地址 0x0064，每点固定占 6 个 16 位寄存器，当前可靠支持 100 点，合计 600 个字。
 * 这四个宏共同描述同构点阵的基址、步长、点数和总长度；任何一项变化都必须与 Wartsila 寄存器表及分块读取逻辑同步。
 */
/* CPU3 读取 Wartsila 密度点阵的起始寄存器地址 0x0064；第 0 点从此地址开始。 */
#define CPU3_EXTERNAL_WARTSILA_POINT_BASE       0x0064U
/* 单个 Wartsila 密度点固定占用 6 个 16 位寄存器，用于计算相邻点的地址步进。 */
#define CPU3_EXTERNAL_WARTSILA_POINT_STRIDE     6U
/* 当前 CPU3 分块读取和缓存可靠支持 100 个 Wartsila 密度点，合法零基点号为 0～99。 */
#define CPU3_EXTERNAL_WARTSILA_POINT_COUNT      100U
/* Wartsila 密度点阵的总寄存器字数，由每点步长乘点数得到，当前为 600 个 16 位字。 */
#define CPU3_EXTERNAL_WARTSILA_POINT_WORD_COUNT \
    (CPU3_EXTERNAL_WARTSILA_POINT_STRIDE * CPU3_EXTERNAL_WARTSILA_POINT_COUNT)
/* Wartsila 数据新鲜度签名读取的起始地址 0x0672；签名区位于点阵之外，用于判断外部快照是否更新。 */
#define CPU3_EXTERNAL_WARTSILA_SIGNATURE_ADDRESS  0x0672U
/* Wartsila 新鲜度签名连续读取的寄存器数量 20；比较时必须完整读取该区间，不能用部分字段代表整份快照。 */
#define CPU3_EXTERNAL_WARTSILA_SIGNATURE_QUANTITY 20U

/* CPU3 外部读请求的数据新鲜度位掩码；调用方按业务需要组合运行态、参数和固定结果快照要求。 */
typedef enum
{
    /* 外部协议读取前要求刷新的 CPU2 数据类别位；多个要求可以按位组合。 */
    CPU3_EXTERNAL_READ_LOCAL = 0U, /* 只读取 CPU3 本机静态镜像，不要求刷新 CPU2 数据。 */
    CPU3_EXTERNAL_READ_RUNTIME = 1U, /* 应答前要求取得新鲜的 CPU2 运行态快照。 */
    CPU3_EXTERNAL_READ_PARAMETERS = 2U, /* 应答前要求取得新鲜的 CPU2 参数快照。 */
    CPU3_EXTERNAL_READ_FIXED_POINT = 4U /* 应答前要求取得前后计数一致的固定结果快照。 */
} Cpu3ExternalReadRequirement;

/**
 * @brief 判断 DSM 输入寄存器是否属于 CPU3 本地静态离线白名单。
 *
 * @details 调用场景：DSM FC04 完成地址和数量校验后逐寄存器分类。
 * @note 关键约束：0x0001 依赖 CPU2 生命周期，不得作为本地静态状态返回。
 *
 * @param address 待分类的外部协议寄存器地址；函数据此确定字段属于 CPU3 本地、CPU2 运行态、固定点结果或参数快照。
 * @return 1 表示 DSM 输入寄存器属于 CPU3 本地静态离线白名单；0 表示 DSM 输入寄存器不属于 CPU3 本地静态离线白名单。
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

/**
 * @brief 判断 DSM FC04 整帧是否包含 CPU2 派生运行态字段。
 *
 * @details 调用场景：读取输入寄存器影子和刷新 Input_Write 之前调用。
 * @note 关键约束：混合本地与 CPU2 字段的请求按整帧运行态门禁处理。
 *
 * @param start 本次连续访问区间的起始寄存器或位地址。
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @return 1 表示 [start, start + quantity) 中至少一个 DSM 输入寄存器不在本地静态白名单，应先刷新 CPU2 运行态；0 表示 quantity 为 0，或整段地址均可由 CPU3 本地静态数据回答。
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

/**
 * @brief 判断 DSM 输入寄存器是否来自 CPU2 固定点测量或监测结果。
 *
 * @details 调用场景：DSM FC04 在基础运行态门禁后逐寄存器分类。
 * @note 关键约束：0x0006 和 0x000F 是 CPU3 本地状态常量，不得扩大固定点门禁范围。
 *
 * @param address 待分类的外部协议寄存器地址；函数据此确定字段属于 CPU3 本地、CPU2 运行态、固定点结果或参数快照。
 * @return 1 表示地址位于 0x0004～0x0005、0x0007～0x000E 或 0x0010～0x0015，其值来自 CPU2 固定点测量或监测结果；0 表示地址不在这些固定点派生区间。
 */
static inline uint8_t CPU3_ExternalDsmInputNeedsFixedPoint(uint16_t address)
{
    if (((address >= 0x0004U) && (address <= 0x0005U)) ||
        ((address >= 0x0007U) && (address <= 0x000EU)) ||
        ((address >= 0x0010U) && (address <= 0x0015U)))
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 判断 DSM FC04 整帧是否包含固定点派生字段。
 *
 * @details 调用场景：读取输入寄存器影子前执行独立固定点门禁。
 * @note 关键约束：混合普通运行态和固定点字段时整帧返回 Busy。
 *
 * @param start 本次连续访问区间的起始寄存器或位地址。
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @return 1 表示 [start, start + quantity) 中至少一个 DSM 输入寄存器属于固定点派生区间；0 表示 quantity 为 0，或整段地址均不依赖固定点快照。
 */
static inline uint8_t CPU3_ExternalDsmInputRangeNeedsFixedPoint(uint16_t start,
                                                                uint16_t quantity)
{
    uint32_t index;

    for (index = 0U; index < (uint32_t)quantity; ++index)
    {
        uint16_t address = (uint16_t)((uint32_t)start + index);

        if (CPU3_ExternalDsmInputNeedsFixedPoint(address) != 0U)
        {
            return 1U;
        }
    }

    return 0U;
}

/**
 * @brief 判断 SI FC01/FC02 的目标位是否依赖 CPU2 运行态快照。
 *
 * @details 调用场景：SI 位读取完成功能码、地址和数量校验后逐位分类。
 * @note 关键约束：未列入本地静态白名单的位保守地按 CPU2 派生位处理。
 *
 * @param function 满足当前流程条件时调用的处理函数。
 * @param offset 相对起始位置的偏移量。
 * @return 1 表示功能码不是 0x01 或 0x02，需按保守策略刷新，或目标位落入对应功能码的 CPU2 运行态派生范围；0 仅表示已知 0x01 或 0x02 功能码的目标位属于本地静态范围。
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

/**
 * @brief 判断 SI FC01/FC02 整帧是否包含 CPU2 派生位。
 *
 * @details 调用场景：位池打包前执行一次整帧新鲜度门禁。
 * @note 关键约束：请求跨越本地与 CPU2 位时不得部分返回旧影子。
 *
 * @param function 满足当前流程条件时调用的处理函数。
 * @param start 本次连续访问区间的起始寄存器或位地址。
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @return 1 表示 [start, start + quantity) 中至少一个 SI 线圈或离散输入位依赖 CPU2 运行态；0 表示 quantity 为 0，或整段目标位均属于本地静态范围。
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

/**
 * @brief 判断 SI FC02 整帧是否读取固定点密度派生报警。
 *
 * @details 调用场景：离散输入位池打包前执行独立固定点门禁。
 * @note 关键约束：只有 16、17 位来自固定点密度；FC01 线圈不受该门禁影响。
 *
 * @param function 满足当前流程条件时调用的处理函数。
 * @param start 本次连续访问区间的起始寄存器或位地址。
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @return 1 表示功能码为 0x02，且 [start, start + quantity) 与离散输入偏移 16～17 的固定点密度派生报警相交；0 表示功能码不是 0x02，或读取范围没有触及这两个位。
 */
static inline uint8_t CPU3_ExternalSiBitRangeNeedsFixedPoint(uint8_t function,
                                                             uint16_t start,
                                                             uint16_t quantity)
{
    uint32_t range_end = (uint32_t)start + (uint32_t)quantity;

    if (function != 0x02U)
    {
        return 0U;
    }

    return (((uint32_t)start < 18U) && (range_end > 16U)) ? 1U : 0U;
}

/**
 * @brief 判断 SI FC04 输入寄存器是否依赖 CPU2 运行态快照。
 *
 * @details 调用场景：SI 输入寄存器响应复制前逐 offset 分类。
 * @note 关键约束：Profile 时间、镜像和点阵均随 CPU2 生命周期失效。
 *
 * @param offset 相对起始位置的偏移量。
 * @return 1 表示输入寄存器偏移位于 0～3、5～9、13～15 或 20～619，应使用 CPU2 运行态快照；0 表示偏移位于本地静态间隙 4、10～12、16～19，或超出当前 CPU2 派生范围。
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

/**
 * @brief 判断 SI FC04 整帧是否包含 CPU2 派生寄存器。
 *
 * @details 调用场景：输入寄存器池序列化前执行一次整帧新鲜度门禁。
 * @note 关键约束：混合范围必须整体返回 Busy，不得拼接本地与旧 CPU2 数据。
 *
 * @param start 本次连续访问区间的起始寄存器或位地址。
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @return 1 表示 [start, start + quantity) 中至少一个 SI 输入寄存器依赖 CPU2 运行态；0 表示 quantity 为 0，或整段寄存器均属于本地静态范围。
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

/**
 * @brief 判断 SI FC04 整帧是否读取固定点密度或其离散输入镜像。
 *
 * @details 调用场景：输入寄存器池序列化前执行独立固定点门禁。
 * @note 关键约束：offset 2 是当前密度，offset 15 含低高密度报警镜像。
 *
 * @param start 本次连续访问区间的起始寄存器或位地址。
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @return 1 表示 [start, start + quantity) 触及偏移 2 的当前密度，或偏移 15 的固定点密度报警镜像；0 表示两处固定点派生寄存器均未被读取。
 */
static inline uint8_t CPU3_ExternalSiInputRangeNeedsFixedPoint(uint16_t start,
                                                               uint16_t quantity)
{
    uint32_t range_end = (uint32_t)start + (uint32_t)quantity;
    uint8_t contains_density = ((uint32_t)start < 3U) && (range_end > 2U);
    uint8_t contains_alarm_mirror = ((uint32_t)start < 16U) && (range_end > 15U);

    return ((contains_density != 0U) || (contains_alarm_mirror != 0U)) ? 1U : 0U;
}

/**
 * @brief 分类一个 Wärtsilä FC03 保持寄存器的新鲜度要求。
 *
 * @details 调用场景：合法读取范围在复制寄存器影子前逐项分类。
 * @note 关键约束：未列入静态白名单的兼容空洞保守地要求运行态快照。
 *
 * @param address 待分类的外部协议寄存器地址；函数据此确定字段属于 CPU3 本地、CPU2 运行态、固定点结果或参数快照。
 * @return 返回目标瓦锡兰保持寄存器对应的新鲜度要求枚举：本机静态、参数、固定点或运行态。
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

    if ((address == 0x0002U) ||
        (address == 0x0004U) ||
        (address == 0x000BU))
    {
        return CPU3_EXTERNAL_READ_FIXED_POINT;
    }

    if ((address == 0x0000U) ||
        (address == 0x0007U) ||
        (address == 0x0009U) ||
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

/**
 * @brief 汇总 Wärtsilä FC03 整帧所需的运行态和参数门禁。
 *
 * @details 调用场景：合法范围读取寄存器影子前调用。
 * @note 关键约束：返回值为位组合，混合范围必须同时满足全部要求。
 *
 * @param start 本次连续访问区间的起始寄存器或位地址。
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @return 返回整段瓦锡兰 FC03 请求需要满足的新鲜度门禁位掩码。
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

/**
 * @brief 把 CPU2 发布点数限制到 Wärtsilä 第一版可靠容量。
 *
 * @details 调用场景：投影点数和复制点表前调用。
 * @note 关键约束：101 至 200 点只受控截断到前 100 点，不得越界访问结构尾部。
 *
 * @param point_count CPU2 快照声明的瓦锡兰测点数量，钳位前可能超过对外容量。
 * @return 返回瓦锡兰第一版外部快照允许的点数；超过可靠容量时钳位到 CPU3_EXTERNAL_WARTSILA_POINT_COUNT。
 */
static inline uint16_t CPU3_ExternalWartsilaClampPointCount(uint32_t point_count)
{
    return (point_count > CPU3_EXTERNAL_WARTSILA_POINT_COUNT) ?
           (uint16_t)CPU3_EXTERNAL_WARTSILA_POINT_COUNT :
           (uint16_t)point_count;
}

/**
 * @brief 判断 Wärtsilä 版本签名读取是否命中特殊地址。
 *
 * @details 调用场景：FC03 在普通保持寄存器边界检查前识别签名请求。
 * @note 关键约束：只有 0x0672 使用签名响应，其余地址必须走普通寄存器规则。
 *
 * @param start 本次连续访问区间的起始寄存器或位地址。
 * @return 1 表示起始地址等于 CPU3_EXTERNAL_WARTSILA_SIGNATURE_ADDRESS（0x0672），应走版本签名读取路径；0 表示其它起始地址。
 */
static inline uint8_t CPU3_ExternalWartsilaIsSignatureAddress(uint16_t start)
{
    return (start == CPU3_EXTERNAL_WARTSILA_SIGNATURE_ADDRESS) ? 1U : 0U;
}

/**
 * @brief 校验 Wärtsilä 版本签名请求的固定寄存器数量。
 *
 * @details 调用场景：FC03 命中特殊签名地址后、复制 40 字节签名前调用。
 * @note 关键约束：响应固定为 20 个寄存器，任何其它数量均不得成功回包。
 *
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @return 1 表示请求数量等于瓦锡兰版本签名固定寄存器数；否则返回 0。
 */
static inline uint8_t CPU3_ExternalWartsilaSignatureQuantityIsValid(uint16_t quantity)
{
    return (quantity == CPU3_EXTERNAL_WARTSILA_SIGNATURE_QUANTITY) ? 1U : 0U;
}

/**
 * @brief 判断 Wärtsilä 下发命令是否具有已实现且可确认的 CPU2 语义。
 *
 * @details 调用场景：FC10 覆盖命令寄存器时，在写入影子和下发 CPU2 前调用。
 * @note 关键约束：0 仅用于清空命令；未知命令必须返回非法数据值。
 *
 * @param command 瓦锡兰保持寄存器写入的 16 位外部命令码；函数只接受可映射并由 CPU2 确认的命令。
 * @return 1 表示命令值为 0、3、4 或 5，CPU3 已实现相应 CPU2 语义并可确认结果；0 表示其它命令值，当前桥接层不支持下发。
 */
static inline uint8_t CPU3_ExternalWartsilaCommandIsSupported(uint16_t command)
{
    return ((command == 0U) ||
            (command == 3U) ||
            (command == 4U) ||
            (command == 5U)) ? 1U : 0U;
}

/**
 * 【Wärtsilä 莆田现场专用格式，禁止回退为标准 Modbus FC10】
 * @brief 校验 Wärtsilä FC10 的数量、字节数和 PDU 长度关系。
 *
 * @details 调用场景：FC10 解析头部后、访问数据区或寄存器池之前调用。
 * @note 关键约束：所有 Wärtsilä FC10 指令仅接受 byteCount=quantity，数据区仍为
 *           2*quantity 字节；标准 Modbus 的 byteCount=2*quantity 必须拒绝。
 *           PDU 长度必须严格等于 6+2*quantity，不能按现场 byteCount 字段
 *           直接推导数据区长度。FC03 响应不使用本特例，
 *           byteCount 仍为 2*quantity。
 *
 * @param quantity 本次连续处理的数据项或寄存器数量。
 * @param byte_count FC10 PDU 字节数字段声明的数据区长度，单位字节。
 * @param pdu_length 从功能码开始计算的完整 FC10 PDU 长度，单位字节。
 * @return 1 表示寄存器数量、字节数和 PDU 长度相互一致且在上限内；否则返回 0。
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

/**
 * @brief 清空 Wärtsilä 第一版全部点表寄存器。
 *
 * @details 调用场景：每次从本轮 CPU2 快照投影点表之前调用。
 * @note 关键约束：清空后只填充本轮有效点，长轮次尾部不得泄漏到短轮次。
 *
 * @param point_registers 待清零的瓦锡兰外部测点寄存器数组。
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

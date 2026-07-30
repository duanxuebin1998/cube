#ifndef SI_MODBUS_SLAVE_H_
/* SI_MODBUS_SLAVE_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SI_MODBUS_SLAVE_H_

#include <stdbool.h>
#include <stdint.h>

/* 当前兼容的 SI 地址窗口大小，必须与 .c 中枚举和 golden frame 脚本保持一致。 */
#define SI_COIL_COUNT            16U
/* SI 离散输入表容量 32 位；FC02 读取区间不得超过该上界。 */
#define SI_DISCRETE_INPUT_COUNT  32U
/* SI 保持寄存器表容量 23；FC03/FC06/FC10 必须按完整字段校验。 */
#define SI_HOLDING_REG_COUNT     23U
/* SI 输入寄存器表容量 620；包含固定状态与剖面点数据，FC04 连续读取不得越界。 */
#define SI_INPUT_REG_COUNT       620U

/* SI 帧处理结果；tx_len>0 时即使返回非 OK，也表示已经生成可发送的异常响应。 */
typedef enum {
    /* SI Modbus 帧处理结果。 */
    SI_MODBUS_OK = 0, /* 处理成功。 */
    SI_MODBUS_ERR_BADLEN, /* 帧长度与协议要求不符。 */
    SI_MODBUS_ERR_CRC, /* 帧 CRC 校验失败。 */
    SI_MODBUS_ERR_ADDR_MISMATCH, /* 帧地址不是本机或目标设备地址。 */
    SI_MODBUS_ERR_FUNC_UNSUPPORT /* 请求的 Modbus 功能码不受支持。 */
} SiModbusResult;

/**
 * @brief 校验并处理一帧完整的 SI Modbus RTU 请求。
 *
 * 地址和 CRC 通过后才同步系统状态，避免无关帧扰动 CPU3 快照；tx_len 非零时表示已经生成可发送的正常或异常响应。
 *
 * @param rx_buf 接收到的原始请求帧首地址；函数只读取 rx_buf[0..rx_len-1]，不会修改请求内容。
 * @param rx_len 请求帧有效长度，单位字节；访问固定字段前会校验协议要求的最小长度。
 * @param tx_buf 正常或异常响应帧的输出缓冲区；容量必须覆盖 SI 协议允许的最大响应长度。
 * @param tx_len 用于返回响应帧有效长度的输出指针，单位字节；长度包含地址、数据区和 CRC。
 * @return SI_MODBUS_OK 表示请求已完成处理；其他值区分帧长、CRC、从机地址和功能码错误。tx_len 非零时仍应发送已生成的异常响应。
 */
SiModbusResult si_modbus_process(const uint8_t *rx_buf,
                                         uint16_t rx_len,
                                         uint8_t *tx_buf,
                                         uint16_t *tx_len);
/**
 * @brief 把 SI Modbus 处理结果适配为 CPU3 外部协议分发表使用的 uint32_t 口径。
 *
 * 只要核心处理器已经生成正常或异常响应帧，分发层就按本帧已处理并发送响应；没有响应帧时才向上返回解析错误。
 *
 * @param rx_buf 接收到的原始 SI Modbus RTU 请求帧首地址。
 * @param rx_len 请求帧有效长度，单位字节。
 * @param tx_buf 转交 SI 核心处理器的响应输出缓冲区。
 * @param tx_len 用于返回响应帧有效长度的输出指针，单位字节。
 * @return 已生成响应帧时返回 0；没有响应帧时返回对应的 SI Modbus 解析错误码。
 */
uint32_t si_modbus_process_for_dispatch(const uint8_t *rx_buf,
                                            uint16_t rx_len,
                                            uint8_t *tx_buf,
                                            uint16_t *tx_len);


/**
 * @brief 手动同步 SI 四类寄存器影子区。
 *
 * 正常处理请求前会自动调用，外部入口主要用于测试或联调前刷新快照。
 *
 * 手动刷新四类 SI 影子区，主要供测试或后续联调入口调用。
 */
void si_modbus_sync_from_system(void);

/**
 * @brief 推进 SI 自动剖面调度、候选点同步和报警状态。
 *
 * 每次调用先分批同步 SI Profile 候选点阵，并检查自动 Profile 配置是否变化；关闭自动调度或周期为 0 时清除现有计划锚点后返回。
 * CPU3 启动后必须先取得 CPU2 运行态快照，才能判断计划分钟是否已有正在准备、测量或回液位的 SI 周期，避免重启后重复覆盖活动流程。
 * 首次建立锚点时使用当天配置时刻；配置在当天时刻之后生效时可顺延到次日。锚点建立后用累计分钟差对 interval 取模，因此 61 或 1000 分钟等非整日周期能够连续跨越午夜。
 * 同一计划分钟最多触发一次；启动请求未被接受时按 SI_AUTO_PROFILE_RETRY_DELAY_MS 限流重试，只有请求成功才记录该分钟已经触发。
 *
 * 自动 profile 调度入口；由主循环周期调用，内部按 RTC 分钟去重。
 *
 * @note 该函数由 CPU3 主循环周期调用；候选点阵同步可能跨多轮完成，不应挪入外部 Modbus 请求响应路径。
 */
void si_modbus_periodic_task(void);

/**
 * @brief 请求启动 SI Profile 测量。
 *
 * 调用场景：外部 00004 Profile 线圈、自动 profile 调度和 CPU3 屏幕
 * SI Profile 菜单入口共用。
 * 关键约束：这里只下发命令；Profile Timestamp必须等CPU2发布Point0 cycle事件后锁存。
 *
 * 请求启动 SI Profile；只向CPU2下发命令，时间由后续Point0周期事件锁存。
 *
 * @return true 表示 CMD_SI_PROFILE 已通过当前 CPU2 命令通道接受；false 表示 CPU2 通信不可用、命令门禁拒绝、Modbus 异常或命令 ACK 无效。
 */
bool si_profile_request_start(void);

#endif /* SI_MODBUS_SLAVE_H_ */

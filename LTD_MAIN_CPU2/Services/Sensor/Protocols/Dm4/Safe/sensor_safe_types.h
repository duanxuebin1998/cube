/*
 * 模块职责：集中定义Safe V1线协议常量、结果码、枚举和编解码数据结构。
 * 兼容边界：本文件中的数值、字段宽度和缩放属于外部协议契约，不能按编译器布局推导。
 * 分层约束：只声明协议领域类型，不包含UART6、FRAM或业务流程实现。
 */
#ifndef SENSOR_SAFE_TYPES_H_
/* SENSOR_SAFE_TYPES_H_ 是本头文件的包含保护标记；首次展开后置位，防止重复包含造成类型或接口重复定义。 */
#define SENSOR_SAFE_TYPES_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制帧与固定长度快报的线格式常量；修改任一值都属于外部安全协议卷变化。 */
/* 所有安全协议帧的第一个同步字节 0xA5；接收状态机必须与 SOF1 连续匹配后才开始解析帧头。 */
#define SENSOR_SAFE_SOF0                         0xA5U
/* 所有安全协议帧的第二个同步字节 0x5A；与 SOF0 共同降低普通串口噪声被误判为帧起始的概率。 */
#define SENSOR_SAFE_SOF1                         0x5AU
/* 当前安全协议线格式版本 1；握手与控制帧必须校验该值，不兼容版本不得继续按当前字段布局解析。 */
#define SENSOR_SAFE_PROTOCOL_VERSION             0x01U
/* 控制帧固定头长度 28 字节，从 SOF0 起计并包含 payload 之前的全部字段。 */
#define SENSOR_SAFE_CTRL_HEADER_LEN              28U
/* 控制帧尾 CRC32C 字段长度 4 字节。 */
#define SENSOR_SAFE_CTRL_CRC_LEN                 4U
/* 无业务负载控制帧的最小总长度 32 字节，等于固定头长度加 CRC32C 长度。 */
#define SENSOR_SAFE_CTRL_MIN_FRAME_LEN           32U
/* 控制帧允许的最大总长度 128 字节；接收缓冲与长度校验均以此值为硬上限。 */
#define SENSOR_SAFE_CTRL_MAX_FRAME_LEN           128U
/* 控制帧最大业务负载长度 96 字节；由最大帧长扣除固定头和 CRC 字段得到。 */
#define SENSOR_SAFE_CTRL_MAX_PAYLOAD_LEN         96U
/* 固定长度周期快报的帧类型标识 0xA1；用于在同步字节之后区分紧凑快报与普通控制帧。 */
#define SENSOR_SAFE_FAST_FRAME_TYPE              0xA1U
/* 周期快报固定总长度 44 字节；解析器必须精确匹配，禁止接受截短或附加字节。 */
#define SENSOR_SAFE_FAST_FRAME_LEN               44U
/* 安全协议通用响应负载长度 20 字节；请求与响应处理必须按固定字段表校验。 */
#define SENSOR_SAFE_COMMON_RESPONSE_LEN          20U
/* 参数分页中单条参数记录的线格式长度 22 字节；包含参数标识、类型、属性、缩放和值等固定字段。 */
#define SENSOR_SAFE_PARAM_RECORD_LEN             22U
/* 参数分页负载头长度 6 字节；记录区从该偏移之后开始。 */
#define SENSOR_SAFE_PARAM_PAGE_HEADER_LEN        6U
/* 单个参数分页帧最多携带 3 条记录；该限制保证分页负载不超过控制帧最大负载长度。 */
#define SENSOR_SAFE_PARAM_PAGE_MAX_ITEMS         3U
/* CPU2 允许缓存和索引的安全参数表最大条目数 256；总数超过该值时必须拒绝，防止索引和位图越界。 */
#define SENSOR_SAFE_PARAM_TABLE_MAX_ITEMS        256U
/* 参数分页标志位 bit0：当前页是本次参数表传输的最后一页。 */
#define SENSOR_SAFE_PARAM_PAGE_FLAG_LAST         0x01U
/* 参数分页当前允许的标志位掩码；接收端必须拒绝该掩码之外的保留位。 */
#define SENSOR_SAFE_PARAM_PAGE_ALLOWED_FLAGS     SENSOR_SAFE_PARAM_PAGE_FLAG_LAST
/* 参数总数未知时在线格式中使用的哨兵值 0xFFFF；不能把它当作真实条目数量参与分配或循环。 */
#define SENSOR_SAFE_PARAM_TOTAL_UNKNOWN          0xFFFFU
/* LTD 安全参数表的首个参数标识 0x1000；用于限定本机参数命名空间。 */
#define SENSOR_SAFE_LTD_PARAM_BASE               0x1000U
/* LTD 安全参数表当前条目数量 115；必须与参数定义表和末项标识保持一致。 */
#define SENSOR_SAFE_LTD_PARAM_COUNT              115U
/* LTD 安全参数表当前最后一个合法参数标识 0x1072；地址范围检查按 BASE（含）到 LAST（含）执行。 */
#define SENSOR_SAFE_LTD_PARAM_LAST               0x1072U
/* LTD 安全参数表结构版本 2；字段增删、类型或缩放变化时必须递增。 */
#define SENSOR_SAFE_LTD_PARAM_TABLE_VERSION      2U
/* LTD 浮点参数在线协议中的十进制缩放指数 -6；表示整数线值按 10^-6 还原，避免直接传输平台浮点布局。 */
#define SENSOR_SAFE_LTD_FLOAT_SCALE               (-6)
/* 历史参数未迁移到标准单位枚举时使用的单位标识 0；接收端必须按对应参数定义解释，不能假设统一物理单位。 */
#define SENSOR_SAFE_UNIT_LEGACY_NATIVE            0U

#if ((SENSOR_SAFE_COMMON_RESPONSE_LEN + SENSOR_SAFE_PARAM_PAGE_HEADER_LEN + \
      (SENSOR_SAFE_PARAM_PAGE_MAX_ITEMS * SENSOR_SAFE_PARAM_RECORD_LEN)) > \
     SENSOR_SAFE_CTRL_MAX_PAYLOAD_LEN)
#error "sensor safe parameter page exceeds control payload"
#endif

#if ((SENSOR_SAFE_LTD_PARAM_BASE + SENSOR_SAFE_LTD_PARAM_COUNT - 1U) != \
     SENSOR_SAFE_LTD_PARAM_LAST)
#error "sensor safe LTD parameter range is inconsistent"
#endif

/* 控制帧字段偏移均从 SOF 首字节起算，禁止以 C 结构体布局替代。 */
/* 控制帧“协议版本”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_PROTOCOL_VERSION 2U
/* 控制帧“控制头长度”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_HEADER_LEN       3U
/* 控制帧“整帧长度”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_FRAME_LEN        4U
/* 控制帧“源节点 ID”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_SRC_ID           6U
/* 控制帧“目标节点 ID”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_DST_ID           7U
/* 控制帧“消息类型”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_MSG_TYPE         8U
/* 控制帧“命令码”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_CMD              9U
/* 控制帧“控制标志位”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_FLAGS            10U
/* 控制帧“帧序列号”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_SEQ              12U
/* 控制帧“会话 ID”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_SESSION_ID       16U
/* 控制帧“传感器 ID”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_SENSOR_ID        20U
/* 控制帧“参数表 CRC”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_PARAM_CRC        24U
/* 控制帧“业务负载首字节”字段相对 SOF0 首字节的字节偏移；序列化与解析必须按线格式逐字节访问，不能依赖 C 结构体对齐。 */
#define SENSOR_SAFE_CTRL_OFFSET_PAYLOAD          28U

/* 周期快报字段偏移；快报采用紧凑固定布局并允许非自然对齐字段。 */
/* 固定长度周期快报“固定快报帧类型”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_FRAME_TYPE       2U
/* 固定长度周期快报“固定快报整帧长度”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_FRAME_LEN        3U
/* 固定长度周期快报“源节点 ID”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_SRC_ID           4U
/* 固定长度周期快报“目标节点 ID”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_DST_ID           5U
/* 固定长度周期快报“周期流状态”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_STREAM_STATE     6U
/* 固定长度周期快报“会话 ID”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_SESSION_ID       7U
/* 固定长度周期快报“数据流 ID”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_STREAM_ID        11U
/* 固定长度周期快报“数据流序列号”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_STREAM_SEQ       13U
/* 固定长度周期快报“传感器采样计数”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_SAMPLE_COUNTER   17U
/* 固定长度周期快报“数据年龄，单位 ms”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_DATA_AGE_MS      21U
/* 固定长度周期快报“数据状态位”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_STATUS_FLAGS     23U
/* 固定长度周期快报“诊断状态位”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_DIAG_FLAGS       25U
/* 固定长度周期快报“温度原始字段”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_TEMPERATURE      27U
/* 固定长度周期快报“密度原始字段”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_DENSITY          29U
/* 固定长度周期快报“频率原始字段”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_FREQUENCY        33U
/* 固定长度周期快报“信号质量”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_QUALITY          37U
/* 固定长度周期快报“配置代际”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_CONFIG_EPOCH     38U
/* 固定长度周期快报“帧尾 CRC32C”字段相对 SOF0 首字节的字节偏移；字段可能非自然对齐，必须使用显式字节编解码函数。 */
#define SENSOR_SAFE_FAST_OFFSET_CRC              40U

/* 控制帧标志位及允许掩码；接收端必须拒绝掩码之外的保留位。 */
/* 控制帧标志位：发送方要求对端返回确认帧；该位只描述当前帧语义，不能脱离命令、序列号和会话状态单独处理。 */
#define SENSOR_SAFE_FLAG_ACK_REQUIRED            (1UL << 0U)
/* 控制帧标志位：本帧包含安全相关业务数据；该位只描述当前帧语义，不能脱离命令、序列号和会话状态单独处理。 */
#define SENSOR_SAFE_FLAG_SAFETY_RELEVANT         (1UL << 1U)
/* 控制帧标志位：本帧是同一事务的重试发送；该位只描述当前帧语义，不能脱离命令、序列号和会话状态单独处理。 */
#define SENSOR_SAFE_FLAG_RETRY_FRAME             (1UL << 2U)
/* 控制帧标志位：发送方配置代际已经变化；该位只描述当前帧语义，不能脱离命令、序列号和会话状态单独处理。 */
#define SENSOR_SAFE_FLAG_CONFIG_CHANGED          (1UL << 3U)
/* 控制帧标志位：发送方刚完成冷启动；该位只描述当前帧语义，不能脱离命令、序列号和会话状态单独处理。 */
#define SENSOR_SAFE_FLAG_COLD_START              (1UL << 4U)
/* 控制帧标志位：发送方当前存在错误状态；该位只描述当前帧语义，不能脱离命令、序列号和会话状态单独处理。 */
#define SENSOR_SAFE_FLAG_ERROR_PRESENT           (1UL << 5U)
/* 控制帧标志位：请求对端停止周期快报；该位只描述当前帧语义，不能脱离命令、序列号和会话状态单独处理。 */
#define SENSOR_SAFE_FLAG_STOP_STREAM_REQUEST     (1UL << 7U)
/* 控制帧标志位：本次参数事务包含写操作；该位只描述当前帧语义，不能脱离命令、序列号和会话状态单独处理。 */
#define SENSOR_SAFE_FLAG_PARAM_WRITE             (1UL << 8U)
/* 控制帧当前允许置位的标志位合集 0x01BF；任何保留位非零都视为协议格式错误。 */
#define SENSOR_SAFE_ALLOWED_FLAGS                0x01BFU

/* 数据状态位；VALID 只是必要条件，仍需结合 STALE、MODE 和诊断位判定。 */
/* 周期数据状态位：本帧业务数据通过发送端基础有效性检查；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_DATA_VALID            (1UL << 0U)
/* 周期数据状态位：数据年龄已超过发送端允许窗口；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_DATA_STALE            (1UL << 1U)
/* 周期数据状态位：当前测量模式与请求模式一致；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_MODE_MATCH            (1UL << 2U)
/* 周期数据状态位：测量模式切换后仍处于稳定等待期；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_MODE_SETTLING         (1UL << 3U)
/* 周期数据状态位：至少一个过程量被限制到允许范围；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_VALUE_CLAMPED         (1UL << 4U)
/* 周期数据状态位：传感信号质量低于可信阈值；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_LOW_SIGNAL_QUALITY    (1UL << 5U)
/* 周期数据状态位：双方使用的配置代际或参数表不一致；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_CONFIG_MISMATCH       (1UL << 6U)
/* 周期数据状态位：传感器要求执行自检；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_SELF_TEST_REQUIRED    (1UL << 7U)
/* 周期数据状态位：传感器要求维护处理；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_MAINTENANCE_REQUIRED  (1UL << 8U)
/* 周期数据状态位：周期数据流当前处于活动状态；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_STREAM_ACTIVE         (1UL << 9U)
/* 周期数据状态位：周期帧序列检测到丢帧或中断；CPU2 必须与其它状态位、诊断位及数据年龄联合判定有效性。 */
#define SENSOR_SAFE_STATUS_STREAM_LOSS_DETECTED  (1UL << 10U)
/* 周期快报允许的 16 位数据状态掩码 0x07FF；接收端必须拒绝超出已定义 bit0～bit10 的状态位。 */
#define SENSOR_SAFE_ALLOWED_FAST_STATUS16        0x07FFU
/* 直接使周期快报判无效的数据状态掩码；当前包含数据流丢失，仍需叠加 DATA_VALID、STALE 和模式状态判断。 */
#define SENSOR_SAFE_STATUS_FAST_INVALID_MASK     SENSOR_SAFE_STATUS_STREAM_LOSS_DETECTED

/* 诊断位及数据失效掩码；通道诊断还需按当前测量模式单独选择。 */
/* 传感器诊断位：传感器自检失败；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_SELF_TEST_FAILED        (1UL << 0U)
/* 传感器诊断位：传感器供电电压低于允许范围；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_POWER_LOW               (1UL << 1U)
/* 传感器诊断位：传感器供电电压高于允许范围；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_POWER_HIGH              (1UL << 2U)
/* 传感器诊断位：温度通道超出有效量程；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_TEMP_RANGE_ERROR        (1UL << 3U)
/* 传感器诊断位：密度测量通道故障；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_DENSITY_CHANNEL_ERROR   (1UL << 4U)
/* 传感器诊断位：液位测量通道故障；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_LEVEL_CHANNEL_ERROR     (1UL << 5U)
/* 传感器诊断位：水相电容测量通道故障；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_WATER_CAP_ERROR         (1UL << 6U)
/* 传感器诊断位：姿态/陀螺通道故障；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_GYRO_CHANNEL_ERROR      (1UL << 7U)
/* 传感器诊断位：ADC 或测量定时资源故障；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_ADC_OR_TIMER_ERROR      (1UL << 8U)
/* 传感器诊断位：参数表 CRC 校验失败；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_PARAM_CRC_ERROR         (1UL << 9U)
/* 传感器诊断位：标定数据无效；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_CALIBRATION_INVALID     (1UL << 10U)
/* 传感器诊断位：传感器内部通信故障；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_INTERNAL_COMM_ERROR     (1UL << 11U)
/* 传感器诊断位：周期快报缓冲区发生溢出；是否使当前数据失效还要结合全局掩码和当前测量模式的通道需求。 */
#define SENSOR_SAFE_DIAG_STREAM_BUFFER_OVERFLOW  (1UL << 12U)
/* 周期快报允许的 16 位诊断掩码 0x1FFF；超出 bit0～bit12 的保留诊断位必须判为协议错误。 */
#define SENSOR_SAFE_ALLOWED_FAST_DIAG16          0x1FFFU
/* 无论当前测量模式为何都必须使数据判无效的全局诊断位合集；包含自检、采样资源、参数 CRC、标定和内部通信故障。 */
#define SENSOR_SAFE_DIAG_GLOBAL_INVALID_MASK     0x0F01U
/* 周期快报全局失效诊断位合集；在通用全局故障基础上额外包含快报缓冲区溢出。 */
#define SENSOR_SAFE_DIAG_FAST_INVALID_MASK       \
    (SENSOR_SAFE_DIAG_GLOBAL_INVALID_MASK | SENSOR_SAFE_DIAG_STREAM_BUFFER_OVERFLOW)

/* HELLO 能力位；CPU2 只能调用传感器明确声明支持的可选命令。 */
/* HELLO 能力位：传感器支持“密度测量”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_DENSITY                  (1UL << 0U)
/* HELLO 能力位：传感器支持“液位测量”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_LEVEL                    (1UL << 1U)
/* HELLO 能力位：传感器支持“组合测量命令”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_MEAS_COMBO               (1UL << 2U)
/* HELLO 能力位：传感器支持“水相电容测量”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_WATER_CAP                (1UL << 3U)
/* HELLO 能力位：传感器支持“姿态/陀螺数据”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_GYRO                     (1UL << 4U)
/* HELLO 能力位：传感器支持“探底检测”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_BOTTOM                   (1UL << 5U)
/* HELLO 能力位：传感器支持“传感器电源控制”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_POWER                    (1UL << 6U)
/* HELLO 能力位：传感器支持“周期快报上行”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_PERIODIC_UPLINK          (1UL << 7U)
/* HELLO 能力位：传感器支持“安全参数分页读写”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_PARAM_RW                 (1UL << 8U)
/* HELLO 能力位：传感器支持“传感器自检命令”；CPU2 只有在该位明确置位后才可发送对应可选命令。 */
#define SENSOR_SAFE_CAP_SELF_TEST                (1UL << 9U)
/* HELLO 响应当前允许声明的能力位掩码 0x000003FF；未知保留能力位必须为 0，CPU2 也不得据此调用未定义功能。 */
#define SENSOR_SAFE_ALLOWED_CAPABILITIES         0x000003FFUL

/* 控制消息方向和错误类别，编码值与协议卷冻结值一致。 */
typedef enum {
    /* 安全协议控制帧的消息方向和响应类别。 */
    SENSOR_SAFE_MSG_REQ = 0x01, /* CPU2 发往传感器的控制请求帧。 */
    SENSOR_SAFE_MSG_RSP = 0x81, /* 传感器对控制请求的成功响应帧。 */
    SENSOR_SAFE_MSG_ERR = 0xC1 /* 传感器对控制请求的错误响应帧。 */
} SensorSafeMessageType;

/* 安全协议命令空间；请求与响应使用同一 cmd 并通过 msg_type 区分方向。 */
typedef enum {
    /* 安全协议控制命令码。 */
    SENSOR_SAFE_CMD_HELLO = 0x01, /* 建立新会话并交换节点身份、启动计数、nonce、能力和配置摘要。 */
    SENSOR_SAFE_CMD_GET_STATUS = 0x02, /* 读取远端当前模式、状态、诊断和数据质量。 */
    SENSOR_SAFE_CMD_GET_CONFIG_DIGEST = 0x03, /* 读取远端安全参数 CRC、能力和配置版本摘要。 */
    SENSOR_SAFE_CMD_RESET_SESSION = 0x04, /* 请求远端终止当前会话并返回离线状态。 */
    SENSOR_SAFE_CMD_SET_COMM_MODE = 0x05, /* 切换请求—响应或周期主动上报通信模式。 */
    SENSOR_SAFE_CMD_PING = 0x06, /* 验证当前会话和控制通道仍然可用。 */
    SENSOR_SAFE_CMD_SET_MEASURE_MODE = 0x10, /* 切换传感器测量模式。 */
    SENSOR_SAFE_CMD_READ_MEAS_COMBO = 0x11, /* 读取温度、密度和多频通道的同一采样组合结果。 */
    SENSOR_SAFE_CMD_READ_LEVEL_FREQ = 0x12, /* 读取液位频率及有效范围。 */
    SENSOR_SAFE_CMD_READ_WATER_CAP = 0x13, /* 读取水位电容测量结果。 */
    SENSOR_SAFE_CMD_READ_GYRO = 0x14, /* 读取陀螺仪双轴角度和状态。 */
    SENSOR_SAFE_CMD_READ_POWER = 0x15, /* 读取传感器供电电压和状态。 */
    SENSOR_SAFE_CMD_REPORT_MEAS_FAST = 0x40, /* 周期主动上报固定长度快速测量帧。 */
    SENSOR_SAFE_CMD_READ_PARAM = 0x60, /* 按参数编号读取单个安全参数。 */
    SENSOR_SAFE_CMD_WRITE_PARAM = 0x61, /* 写入单个安全参数的暂存值。 */
    SENSOR_SAFE_CMD_COMMIT_PARAM = 0x62, /* 提交已暂存参数并推进配置代际。 */
    SENSOR_SAFE_CMD_READ_PARAM_PAGE = 0x63, /* 按稳定索引分页读取安全参数表。 */
    SENSOR_SAFE_CMD_RUN_SELF_TEST = 0x70, /* 启动远端自检任务。 */
    SENSOR_SAFE_CMD_READ_SELF_TEST_RESULT = 0x71 /* 读取最近一次远端自检结果。 */
} SensorSafeCommand;

/* 参数类型统一使用 i64 载荷承载；类型字段决定符号、范围和上层解释方式。 */
typedef enum {
    /* 安全参数值的线格式数据类型。 */
    SENSOR_SAFE_PARAM_TYPE_BOOLEAN = 0x01, /* 参数值是布尔量，只允许协议规定的真假编码。 */
    SENSOR_SAFE_PARAM_TYPE_UNSIGNED = 0x02, /* 参数值是无符号整数。 */
    SENSOR_SAFE_PARAM_TYPE_SIGNED = 0x03, /* 参数值是有符号整数。 */
    SENSOR_SAFE_PARAM_TYPE_ENUM = 0x04, /* 参数值是枚举编号，取值范围由参数元数据限定。 */
    SENSOR_SAFE_PARAM_TYPE_BITMASK = 0x05 /* 参数值是可组合的位掩码。 */
} SensorSafeParamType;

/* 参数访问属性采用位掩码；所有分页返回项必须至少声明可读。 */
/* 参数访问属性 bit0：该参数允许通过安全协议读取；所有分页返回记录都必须具备此属性。 */
#define SENSOR_SAFE_PARAM_ACCESS_READ            (1U << 0U)
/* 参数访问属性 bit1：该参数允许通过安全协议写入；实际写入仍需通过类型、范围、会话和 CRC 校验。 */
#define SENSOR_SAFE_PARAM_ACCESS_WRITE           (1U << 1U)
/* 参数记录允许出现的访问属性掩码；当前仅接受读和写两位，保留位非零必须拒绝。 */
#define SENSOR_SAFE_PARAM_ACCESS_ALLOWED         0x03U

/* 传感器测量状态；SELF_TEST 和 FAULT 为诊断状态，不是常规测量通道。 */
typedef enum {
    /* 传感器当前测量模式。 */
    SENSOR_SAFE_MEASURE_IDLE = 0x00, /* 传感器空闲，不执行测量。 */
    SENSOR_SAFE_MEASURE_DENSITY = 0x01, /* 传感器执行密度测量。 */
    SENSOR_SAFE_MEASURE_LEVEL = 0x02, /* 传感器执行液位频率测量。 */
    SENSOR_SAFE_MEASURE_WATER_CAP = 0x03, /* 传感器执行水位电容测量。 */
    SENSOR_SAFE_MEASURE_GYRO = 0x04, /* 传感器执行陀螺仪角度测量。 */
    SENSOR_SAFE_MEASURE_BOTTOM = 0x05, /* 传感器执行罐底检测相关测量。 */
    SENSOR_SAFE_MEASURE_SELF_TEST = 0x7E, /* 传感器正在执行自检。 */
    SENSOR_SAFE_MEASURE_FAULT = 0x7F /* 传感器因故障进入不可测量状态。 */
} SensorSafeMeasureMode;

/* 通信模式；周期主动上报必须通过 SET_COMM_MODE 显式进入和退出。 */
typedef enum {
    /* 安全协议通信模式。 */
    SENSOR_SAFE_COMM_REQUEST_RESPONSE = 0x00, /* 使用请求—响应通信，不接收周期主动上报。 */
    SENSOR_SAFE_COMM_PERIODIC_UPLINK = 0x01 /* 启用传感器周期主动上报快报。 */
} SensorSafeCommMode;

/* 远端在线结果码；只有完整验收的 ERR 或非零 result_code 才可信。 */
typedef enum {
    /* 传感器在响应载荷中返回的线协议结果码。 */
    SENSOR_SAFE_WIRE_OK = 0x0000, /* 远端成功完成请求。 */
    SENSOR_SAFE_WIRE_UNSUPPORTED_VERSION = 0x0001, /* 远端不支持请求中的协议版本。 */
    SENSOR_SAFE_WIRE_UNSUPPORTED_COMMAND = 0x0002, /* 远端不支持该命令码。 */
    SENSOR_SAFE_WIRE_BAD_LENGTH = 0x0003, /* 远端判定请求帧或载荷长度错误。 */
    SENSOR_SAFE_WIRE_BAD_CRC = 0x0004, /* 远端判定请求 CRC 错误。 */
    SENSOR_SAFE_WIRE_BAD_ARGUMENT = 0x0005, /* 远端判定命令参数非法。 */
    SENSOR_SAFE_WIRE_BAD_SEQUENCE = 0x0006, /* 远端判定事务序号不符合当前会话。 */
    SENSOR_SAFE_WIRE_BAD_SESSION = 0x0007, /* 远端判定会话标识无效。 */
    SENSOR_SAFE_WIRE_BAD_ADDRESS = 0x0008, /* 远端判定源或目标节点地址无效。 */
    SENSOR_SAFE_WIRE_MODE_NOT_READY = 0x0009, /* 远端当前模式尚未准备好执行请求。 */
    SENSOR_SAFE_WIRE_MODE_NOT_ALLOWED = 0x000A, /* 远端当前模式不允许执行请求。 */
    SENSOR_SAFE_WIRE_DEVICE_BUSY = 0x000B, /* 远端正在执行其它互斥操作。 */
    SENSOR_SAFE_WIRE_DATA_INVALID = 0x000C, /* 远端当前数据未通过有效性判定。 */
    SENSOR_SAFE_WIRE_DATA_STALE = 0x000D, /* 远端当前数据超过允许年龄。 */
    SENSOR_SAFE_WIRE_CONFIG_MISMATCH = 0x000E, /* 请求携带的安全参数摘要与远端不一致。 */
    SENSOR_SAFE_WIRE_PARAM_CRC_ERROR = 0x000F, /* 远端判定参数级 CRC 错误。 */
    SENSOR_SAFE_WIRE_SELF_TEST_FAILED = 0x0010, /* 远端自检执行失败。 */
    SENSOR_SAFE_WIRE_POWER_ERROR = 0x0011, /* 远端检测到供电异常。 */
    SENSOR_SAFE_WIRE_TEMPERATURE_ERROR = 0x0012, /* 远端检测到温度异常。 */
    SENSOR_SAFE_WIRE_STREAM_NOT_ACTIVE = 0x0013, /* 请求停止或控制的周期流尚未启动。 */
    SENSOR_SAFE_WIRE_STREAM_ALREADY_ACTIVE = 0x0014, /* 请求启动的周期流已经处于活动状态。 */
    SENSOR_SAFE_WIRE_STREAM_EXIT_FAILED = 0x0015 /* 远端无法安全退出周期流模式。 */
} SensorSafeWireResult;

/* 本地协议校验结果，保留具体失效原因供诊断和恢复策略使用。 */
typedef enum {
    /* CPU2 本地帧校验、会话校验和数据质量校验结果。 */
    SENSOR_SAFE_OK = 0, /* 本地安全协议处理成功。 */
    SENSOR_SAFE_INVALID_ARGUMENT, /* 调用参数、缓冲区或长度非法。 */
    SENSOR_SAFE_BUFFER_TOO_SMALL, /* 目标缓冲区不足以容纳编码或解码结果。 */
    SENSOR_SAFE_BAD_SOF, /* 帧起始标志不正确。 */
    SENSOR_SAFE_BAD_LENGTH, /* 帧长字段与实际数据或协议边界不一致。 */
    SENSOR_SAFE_BAD_VERSION, /* 协议版本不受当前实现支持。 */
    SENSOR_SAFE_BAD_HEADER, /* 固定帧头字段组合非法。 */
    SENSOR_SAFE_BAD_CRC, /* CRC32C 校验失败。 */
    SENSOR_SAFE_BAD_RESERVED_FLAGS, /* 控制帧设置了未定义的保留标志位。 */
    SENSOR_SAFE_BAD_MESSAGE_TYPE, /* 消息类型与当前命令方向不一致。 */
    SENSOR_SAFE_BAD_STREAM_STATE, /* 周期流字段与当前流状态不一致。 */
    SENSOR_SAFE_BAD_RESERVED_STATUS, /* 状态或诊断字段设置了未定义保留位。 */
    SENSOR_SAFE_BAD_ADDRESS, /* 源或目标节点地址与当前会话不匹配。 */
    SENSOR_SAFE_BAD_SESSION, /* 会话标识与当前已接受会话不匹配。 */
    SENSOR_SAFE_BAD_SENSOR, /* 传感器编号或类型与 HELLO 身份不一致。 */
    SENSOR_SAFE_BAD_PARAM_CRC, /* 安全参数 CRC 与会话基线不一致。 */
    SENSOR_SAFE_BAD_SEQUENCE, /* 控制事务序号不匹配或发生回放。 */
    SENSOR_SAFE_BAD_SAMPLE_COUNTER, /* 采样计数重复、倒退或不符合当前基线。 */
    SENSOR_SAFE_BAD_CONFIG_EPOCH, /* 配置代际与当前已确认摘要不一致。 */
    SENSOR_SAFE_BAD_CAPABILITY, /* 帧使用了远端未声明的可选能力。 */
    SENSOR_SAFE_DATA_INVALID, /* 状态位表明本次测量数据无效。 */
    SENSOR_SAFE_DATA_STALE, /* 数据年龄超过允许的新鲜度窗口。 */
    SENSOR_SAFE_MODE_MISMATCH, /* 响应测量模式与请求或会话期望不一致。 */
    SENSOR_SAFE_TRANSACTION_PENDING, /* 已有控制事务等待响应，不能并发发起新事务。 */
    SENSOR_SAFE_NEEDS_HELLO, /* 当前会话不可用，需要重新执行 HELLO。 */
    SENSOR_SAFE_REPLAY_DETECTED, /* 检测到旧会话、旧序号或旧采样的回放。 */
    SENSOR_SAFE_STREAM_STOPPED /* 周期流已经停止，当前快报不能再接受。 */
} SensorSafeResult;

/* 控制帧编码字段；payload 仅在编码调用期间有效，不拥有所指内存。 */
typedef struct {
    /* 待编码或已解码的控制帧头字段及载荷视图。 */
    uint8_t src_id; /* 帧源节点地址。 */
    uint8_t dst_id; /* 帧目标节点地址。 */
    uint8_t msg_type; /* 控制帧消息类型，区分请求、响应和错误响应。 */
    uint8_t cmd; /* 向 CPU2 下发的设备命令码。 */
    uint16_t flags; /* 控制帧标志位；接收端必须拒绝允许掩码之外的保留位。 */
    uint32_t seq; /* 控制事务序号，用于请求与响应配对并检测回放。 */
    uint32_t session_id; /* HELLO 协商得到的会话标识；控制帧和快报必须一致。 */
    uint32_t sensor_id; /* HELLO 返回的传感器唯一标识；会话内必须保持不变。 */
    uint32_t safety_param_crc; /* 传感器安全参数集合的 CRC 摘要；会话及测量响应必须与接受基线一致。 */
    const uint8_t *payload; /* 指向调用方载荷缓冲区的只读视图，不拥有该内存。 */
    uint16_t payload_len; /* 控制帧载荷字节数；必须与命令的固定或可变长度规则一致。 */
} SensorSafeControlFields;

/* 已解码控制帧视图；fields.payload 指向调用方原始接收缓冲区。 */
typedef struct {
    /* 控制帧字段视图、实际帧长和接收到的 CRC。 */
    SensorSafeControlFields fields; /* 控制帧已经解码的头字段及载荷视图。 */
    uint16_t frame_len; /* 接收到或编码出的完整控制帧字节数。 */
    uint32_t received_crc; /* 报文尾部携带的 CRC32C 原始值。 */
} SensorSafeControlFrame;

/* 已通过线格式校验的周期快报，定点单位由字段名后缀明确标识。 */
typedef struct {
    /* 固定长度周期快报的全部线字段和接收 CRC。 */
    uint8_t src_id; /* 帧源节点地址。 */
    uint8_t dst_id; /* 帧目标节点地址。 */
    uint8_t measure_mode; /* 响应或快报声明的传感器测量模式。 */
    uint8_t comm_mode; /* 响应或快报声明的通信模式。 */
    uint32_t session_id; /* HELLO 协商得到的会话标识；控制帧和快报必须一致。 */
    uint16_t stream_id; /* 当前周期快报流标识。 */
    uint32_t stream_seq; /* 周期快报流内序号，接收端必须按会话基线校验连续性。 */
    uint32_t sample_counter; /* 传感器单调采样计数；接收端用它检测重复、倒退或过旧数据。 */
    uint16_t data_age_ms; /* 远端报告的数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
    uint16_t status_flags16; /* 16 位状态标志位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint16_t diag_flags16; /* 16 位诊断标志位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    int16_t temperature_c_x100; /* 温度百倍定点值，单位为 0.01 ℃；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    int32_t density_kg_m3_x100; /* 密度百倍定点值，单位为 0.01 kg/m3；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint32_t frequency_hz_x1000; /* 频率千倍定点值，单位为 0.001 Hz；该字段保存已经缩放的整数定点值，换算物理量时只能应用一次缩放。 */
    uint8_t signal_quality; /* 传感器报告的信号质量指标；仍需结合状态位和数据年龄判定可用性。 */
    uint16_t config_epoch; /* 传感器配置提交代际；周期流中变化时需要重新确认配置摘要。 */
    uint32_t received_crc; /* 报文尾部携带的 CRC32C 原始值。 */
} SensorSafeFastReport;

/* 所有控制响应共有的 20 字节载荷前缀。 */
typedef struct {
    /* 所有控制响应共享的结果、模式、状态、样本和数据质量字段。 */
    uint16_t result_code; /* 远端控制响应结果码，取值遵循 SensorSafeWireResult。 */
    uint8_t measure_mode; /* 响应或快报声明的传感器测量模式。 */
    uint8_t comm_mode; /* 响应或快报声明的通信模式。 */
    uint32_t status_flags; /* 状态标志位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint32_t diag_flags; /* 诊断标志位集合；每一位按协议定义独立解释，未知保留位不得当作有效状态。 */
    uint32_t sample_counter; /* 传感器单调采样计数；接收端用它检测重复、倒退或过旧数据。 */
    uint16_t data_age_ms; /* 远端报告的数据年龄，单位为 ms；该字段保存时间间隔或数据年龄，不得与绝对节拍混用。 */
    uint8_t payload_version; /* 命令响应载荷版本；未知版本不得按当前布局解析。 */
    uint8_t data_quality; /* 远端给出的数据质量等级；本地仍需执行状态、年龄和序号校验。 */
} SensorSafeCommonResponse;

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_TYPES_H_ */

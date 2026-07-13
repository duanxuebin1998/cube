#ifndef SENSOR_SAFE_TYPES_H_
#define SENSOR_SAFE_TYPES_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 控制帧与固定长度快报的线格式常量；修改任一值都属于外部安全协议卷变化。 */
#define SENSOR_SAFE_SOF0                         0xA5U
#define SENSOR_SAFE_SOF1                         0x5AU
#define SENSOR_SAFE_PROTOCOL_VERSION             0x01U
#define SENSOR_SAFE_CTRL_HEADER_LEN              28U
#define SENSOR_SAFE_CTRL_CRC_LEN                 4U
#define SENSOR_SAFE_CTRL_MIN_FRAME_LEN           32U
#define SENSOR_SAFE_CTRL_MAX_FRAME_LEN           128U
#define SENSOR_SAFE_CTRL_MAX_PAYLOAD_LEN         96U
#define SENSOR_SAFE_FAST_FRAME_TYPE              0xA1U
#define SENSOR_SAFE_FAST_FRAME_LEN               44U
#define SENSOR_SAFE_COMMON_RESPONSE_LEN          20U
#define SENSOR_SAFE_PARAM_RECORD_LEN             22U
#define SENSOR_SAFE_PARAM_PAGE_HEADER_LEN        6U
#define SENSOR_SAFE_PARAM_PAGE_MAX_ITEMS         3U
#define SENSOR_SAFE_PARAM_TABLE_MAX_ITEMS        256U
#define SENSOR_SAFE_PARAM_PAGE_FLAG_LAST         0x01U
#define SENSOR_SAFE_PARAM_PAGE_ALLOWED_FLAGS     SENSOR_SAFE_PARAM_PAGE_FLAG_LAST
#define SENSOR_SAFE_PARAM_TOTAL_UNKNOWN          0xFFFFU
#define SENSOR_SAFE_LTD_PARAM_BASE               0x1000U
#define SENSOR_SAFE_LTD_PARAM_COUNT              115U
#define SENSOR_SAFE_LTD_PARAM_LAST               0x1072U
#define SENSOR_SAFE_LTD_PARAM_TABLE_VERSION      2U
#define SENSOR_SAFE_LTD_FLOAT_SCALE               (-6)
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
#define SENSOR_SAFE_CTRL_OFFSET_PROTOCOL_VERSION 2U
#define SENSOR_SAFE_CTRL_OFFSET_HEADER_LEN       3U
#define SENSOR_SAFE_CTRL_OFFSET_FRAME_LEN        4U
#define SENSOR_SAFE_CTRL_OFFSET_SRC_ID           6U
#define SENSOR_SAFE_CTRL_OFFSET_DST_ID           7U
#define SENSOR_SAFE_CTRL_OFFSET_MSG_TYPE         8U
#define SENSOR_SAFE_CTRL_OFFSET_CMD              9U
#define SENSOR_SAFE_CTRL_OFFSET_FLAGS            10U
#define SENSOR_SAFE_CTRL_OFFSET_SEQ              12U
#define SENSOR_SAFE_CTRL_OFFSET_SESSION_ID       16U
#define SENSOR_SAFE_CTRL_OFFSET_SENSOR_ID        20U
#define SENSOR_SAFE_CTRL_OFFSET_PARAM_CRC        24U
#define SENSOR_SAFE_CTRL_OFFSET_PAYLOAD          28U

/* 周期快报字段偏移；快报采用紧凑固定布局并允许非自然对齐字段。 */
#define SENSOR_SAFE_FAST_OFFSET_FRAME_TYPE       2U
#define SENSOR_SAFE_FAST_OFFSET_FRAME_LEN        3U
#define SENSOR_SAFE_FAST_OFFSET_SRC_ID           4U
#define SENSOR_SAFE_FAST_OFFSET_DST_ID           5U
#define SENSOR_SAFE_FAST_OFFSET_STREAM_STATE     6U
#define SENSOR_SAFE_FAST_OFFSET_SESSION_ID       7U
#define SENSOR_SAFE_FAST_OFFSET_STREAM_ID        11U
#define SENSOR_SAFE_FAST_OFFSET_STREAM_SEQ       13U
#define SENSOR_SAFE_FAST_OFFSET_SAMPLE_COUNTER   17U
#define SENSOR_SAFE_FAST_OFFSET_DATA_AGE_MS      21U
#define SENSOR_SAFE_FAST_OFFSET_STATUS_FLAGS     23U
#define SENSOR_SAFE_FAST_OFFSET_DIAG_FLAGS       25U
#define SENSOR_SAFE_FAST_OFFSET_TEMPERATURE      27U
#define SENSOR_SAFE_FAST_OFFSET_DENSITY          29U
#define SENSOR_SAFE_FAST_OFFSET_FREQUENCY        33U
#define SENSOR_SAFE_FAST_OFFSET_QUALITY          37U
#define SENSOR_SAFE_FAST_OFFSET_CONFIG_EPOCH     38U
#define SENSOR_SAFE_FAST_OFFSET_CRC              40U

/* 控制帧标志位及允许掩码；接收端必须拒绝掩码之外的保留位。 */
#define SENSOR_SAFE_FLAG_ACK_REQUIRED            (1UL << 0U)
#define SENSOR_SAFE_FLAG_SAFETY_RELEVANT         (1UL << 1U)
#define SENSOR_SAFE_FLAG_RETRY_FRAME             (1UL << 2U)
#define SENSOR_SAFE_FLAG_CONFIG_CHANGED          (1UL << 3U)
#define SENSOR_SAFE_FLAG_COLD_START              (1UL << 4U)
#define SENSOR_SAFE_FLAG_ERROR_PRESENT           (1UL << 5U)
#define SENSOR_SAFE_FLAG_STOP_STREAM_REQUEST     (1UL << 7U)
#define SENSOR_SAFE_FLAG_PARAM_WRITE             (1UL << 8U)
#define SENSOR_SAFE_ALLOWED_FLAGS                0x01BFU

/* 数据状态位；VALID 只是必要条件，仍需结合 STALE、MODE 和诊断位判定。 */
#define SENSOR_SAFE_STATUS_DATA_VALID            (1UL << 0U)
#define SENSOR_SAFE_STATUS_DATA_STALE            (1UL << 1U)
#define SENSOR_SAFE_STATUS_MODE_MATCH            (1UL << 2U)
#define SENSOR_SAFE_STATUS_MODE_SETTLING         (1UL << 3U)
#define SENSOR_SAFE_STATUS_VALUE_CLAMPED         (1UL << 4U)
#define SENSOR_SAFE_STATUS_LOW_SIGNAL_QUALITY    (1UL << 5U)
#define SENSOR_SAFE_STATUS_CONFIG_MISMATCH       (1UL << 6U)
#define SENSOR_SAFE_STATUS_SELF_TEST_REQUIRED    (1UL << 7U)
#define SENSOR_SAFE_STATUS_MAINTENANCE_REQUIRED  (1UL << 8U)
#define SENSOR_SAFE_STATUS_STREAM_ACTIVE         (1UL << 9U)
#define SENSOR_SAFE_STATUS_STREAM_LOSS_DETECTED  (1UL << 10U)
#define SENSOR_SAFE_ALLOWED_FAST_STATUS16        0x07FFU
#define SENSOR_SAFE_STATUS_FAST_INVALID_MASK     SENSOR_SAFE_STATUS_STREAM_LOSS_DETECTED

/* 诊断位及数据失效掩码；通道诊断还需按当前测量模式单独选择。 */
#define SENSOR_SAFE_DIAG_SELF_TEST_FAILED        (1UL << 0U)
#define SENSOR_SAFE_DIAG_POWER_LOW               (1UL << 1U)
#define SENSOR_SAFE_DIAG_POWER_HIGH              (1UL << 2U)
#define SENSOR_SAFE_DIAG_TEMP_RANGE_ERROR        (1UL << 3U)
#define SENSOR_SAFE_DIAG_DENSITY_CHANNEL_ERROR   (1UL << 4U)
#define SENSOR_SAFE_DIAG_LEVEL_CHANNEL_ERROR     (1UL << 5U)
#define SENSOR_SAFE_DIAG_WATER_CAP_ERROR         (1UL << 6U)
#define SENSOR_SAFE_DIAG_GYRO_CHANNEL_ERROR      (1UL << 7U)
#define SENSOR_SAFE_DIAG_ADC_OR_TIMER_ERROR      (1UL << 8U)
#define SENSOR_SAFE_DIAG_PARAM_CRC_ERROR         (1UL << 9U)
#define SENSOR_SAFE_DIAG_CALIBRATION_INVALID     (1UL << 10U)
#define SENSOR_SAFE_DIAG_INTERNAL_COMM_ERROR     (1UL << 11U)
#define SENSOR_SAFE_DIAG_STREAM_BUFFER_OVERFLOW  (1UL << 12U)
#define SENSOR_SAFE_ALLOWED_FAST_DIAG16          0x1FFFU
#define SENSOR_SAFE_DIAG_GLOBAL_INVALID_MASK     0x0F01U
#define SENSOR_SAFE_DIAG_FAST_INVALID_MASK       \
    (SENSOR_SAFE_DIAG_GLOBAL_INVALID_MASK | SENSOR_SAFE_DIAG_STREAM_BUFFER_OVERFLOW)

/* HELLO 能力位；CPU2 只能调用传感器明确声明支持的可选命令。 */
#define SENSOR_SAFE_CAP_DENSITY                  (1UL << 0U)
#define SENSOR_SAFE_CAP_LEVEL                    (1UL << 1U)
#define SENSOR_SAFE_CAP_MEAS_COMBO               (1UL << 2U)
#define SENSOR_SAFE_CAP_WATER_CAP                (1UL << 3U)
#define SENSOR_SAFE_CAP_GYRO                     (1UL << 4U)
#define SENSOR_SAFE_CAP_BOTTOM                   (1UL << 5U)
#define SENSOR_SAFE_CAP_POWER                    (1UL << 6U)
#define SENSOR_SAFE_CAP_PERIODIC_UPLINK          (1UL << 7U)
#define SENSOR_SAFE_CAP_PARAM_RW                 (1UL << 8U)
#define SENSOR_SAFE_CAP_SELF_TEST                (1UL << 9U)
#define SENSOR_SAFE_ALLOWED_CAPABILITIES         0x000003FFUL

/* 控制消息方向和错误类别，编码值与协议卷冻结值一致。 */
typedef enum {
    SENSOR_SAFE_MSG_REQ = 0x01,
    SENSOR_SAFE_MSG_RSP = 0x81,
    SENSOR_SAFE_MSG_ERR = 0xC1
} SensorSafeMessageType;

/* 安全协议命令空间；请求与响应使用同一 cmd 并通过 msg_type 区分方向。 */
typedef enum {
    SENSOR_SAFE_CMD_HELLO = 0x01,
    SENSOR_SAFE_CMD_GET_STATUS = 0x02,
    SENSOR_SAFE_CMD_GET_CONFIG_DIGEST = 0x03,
    SENSOR_SAFE_CMD_RESET_SESSION = 0x04,
    SENSOR_SAFE_CMD_SET_COMM_MODE = 0x05,
    SENSOR_SAFE_CMD_PING = 0x06,
    SENSOR_SAFE_CMD_SET_MEASURE_MODE = 0x10,
    SENSOR_SAFE_CMD_READ_MEAS_COMBO = 0x11,
    SENSOR_SAFE_CMD_READ_LEVEL_FREQ = 0x12,
    SENSOR_SAFE_CMD_READ_WATER_CAP = 0x13,
    SENSOR_SAFE_CMD_READ_GYRO = 0x14,
    SENSOR_SAFE_CMD_READ_POWER = 0x15,
    SENSOR_SAFE_CMD_REPORT_MEAS_FAST = 0x40,
    SENSOR_SAFE_CMD_READ_PARAM = 0x60,
    SENSOR_SAFE_CMD_WRITE_PARAM = 0x61,
    SENSOR_SAFE_CMD_COMMIT_PARAM = 0x62,
    SENSOR_SAFE_CMD_READ_PARAM_PAGE = 0x63,
    SENSOR_SAFE_CMD_RUN_SELF_TEST = 0x70,
    SENSOR_SAFE_CMD_READ_SELF_TEST_RESULT = 0x71
} SensorSafeCommand;

/* 参数类型统一使用 i64 载荷承载；类型字段决定符号、范围和上层解释方式。 */
typedef enum {
    SENSOR_SAFE_PARAM_TYPE_BOOLEAN = 0x01,
    SENSOR_SAFE_PARAM_TYPE_UNSIGNED = 0x02,
    SENSOR_SAFE_PARAM_TYPE_SIGNED = 0x03,
    SENSOR_SAFE_PARAM_TYPE_ENUM = 0x04,
    SENSOR_SAFE_PARAM_TYPE_BITMASK = 0x05
} SensorSafeParamType;

/* 参数访问属性采用位掩码；所有分页返回项必须至少声明可读。 */
#define SENSOR_SAFE_PARAM_ACCESS_READ            (1U << 0U)
#define SENSOR_SAFE_PARAM_ACCESS_WRITE           (1U << 1U)
#define SENSOR_SAFE_PARAM_ACCESS_ALLOWED         0x03U

/* 传感器测量状态；SELF_TEST 和 FAULT 为诊断状态，不是常规测量通道。 */
typedef enum {
    SENSOR_SAFE_MEASURE_IDLE = 0x00,
    SENSOR_SAFE_MEASURE_DENSITY = 0x01,
    SENSOR_SAFE_MEASURE_LEVEL = 0x02,
    SENSOR_SAFE_MEASURE_WATER_CAP = 0x03,
    SENSOR_SAFE_MEASURE_GYRO = 0x04,
    SENSOR_SAFE_MEASURE_BOTTOM = 0x05,
    SENSOR_SAFE_MEASURE_SELF_TEST = 0x7E,
    SENSOR_SAFE_MEASURE_FAULT = 0x7F
} SensorSafeMeasureMode;

/* 通信模式；周期主动上报必须通过 SET_COMM_MODE 显式进入和退出。 */
typedef enum {
    SENSOR_SAFE_COMM_REQUEST_RESPONSE = 0x00,
    SENSOR_SAFE_COMM_PERIODIC_UPLINK = 0x01
} SensorSafeCommMode;

/* 远端在线结果码；只有完整验收的 ERR 或非零 result_code 才可信。 */
typedef enum {
    SENSOR_SAFE_WIRE_OK = 0x0000,
    SENSOR_SAFE_WIRE_UNSUPPORTED_VERSION = 0x0001,
    SENSOR_SAFE_WIRE_UNSUPPORTED_COMMAND = 0x0002,
    SENSOR_SAFE_WIRE_BAD_LENGTH = 0x0003,
    SENSOR_SAFE_WIRE_BAD_CRC = 0x0004,
    SENSOR_SAFE_WIRE_BAD_ARGUMENT = 0x0005,
    SENSOR_SAFE_WIRE_BAD_SEQUENCE = 0x0006,
    SENSOR_SAFE_WIRE_BAD_SESSION = 0x0007,
    SENSOR_SAFE_WIRE_BAD_ADDRESS = 0x0008,
    SENSOR_SAFE_WIRE_MODE_NOT_READY = 0x0009,
    SENSOR_SAFE_WIRE_MODE_NOT_ALLOWED = 0x000A,
    SENSOR_SAFE_WIRE_DEVICE_BUSY = 0x000B,
    SENSOR_SAFE_WIRE_DATA_INVALID = 0x000C,
    SENSOR_SAFE_WIRE_DATA_STALE = 0x000D,
    SENSOR_SAFE_WIRE_CONFIG_MISMATCH = 0x000E,
    SENSOR_SAFE_WIRE_PARAM_CRC_ERROR = 0x000F,
    SENSOR_SAFE_WIRE_SELF_TEST_FAILED = 0x0010,
    SENSOR_SAFE_WIRE_POWER_ERROR = 0x0011,
    SENSOR_SAFE_WIRE_TEMPERATURE_ERROR = 0x0012,
    SENSOR_SAFE_WIRE_STREAM_NOT_ACTIVE = 0x0013,
    SENSOR_SAFE_WIRE_STREAM_ALREADY_ACTIVE = 0x0014,
    SENSOR_SAFE_WIRE_STREAM_EXIT_FAILED = 0x0015
} SensorSafeWireResult;

/* 本地协议校验结果，保留具体失效原因供诊断和恢复策略使用。 */
typedef enum {
    SENSOR_SAFE_OK = 0,
    SENSOR_SAFE_INVALID_ARGUMENT,
    SENSOR_SAFE_BUFFER_TOO_SMALL,
    SENSOR_SAFE_BAD_SOF,
    SENSOR_SAFE_BAD_LENGTH,
    SENSOR_SAFE_BAD_VERSION,
    SENSOR_SAFE_BAD_HEADER,
    SENSOR_SAFE_BAD_CRC,
    SENSOR_SAFE_BAD_RESERVED_FLAGS,
    SENSOR_SAFE_BAD_MESSAGE_TYPE,
    SENSOR_SAFE_BAD_STREAM_STATE,
    SENSOR_SAFE_BAD_RESERVED_STATUS,
    SENSOR_SAFE_BAD_ADDRESS,
    SENSOR_SAFE_BAD_SESSION,
    SENSOR_SAFE_BAD_SENSOR,
    SENSOR_SAFE_BAD_PARAM_CRC,
    SENSOR_SAFE_BAD_SEQUENCE,
    SENSOR_SAFE_BAD_SAMPLE_COUNTER,
    SENSOR_SAFE_BAD_CONFIG_EPOCH,
    SENSOR_SAFE_BAD_CAPABILITY,
    SENSOR_SAFE_DATA_INVALID,
    SENSOR_SAFE_DATA_STALE,
    SENSOR_SAFE_MODE_MISMATCH,
    SENSOR_SAFE_TRANSACTION_PENDING,
    SENSOR_SAFE_NEEDS_HELLO,
    SENSOR_SAFE_REPLAY_DETECTED,
    SENSOR_SAFE_STREAM_STOPPED
} SensorSafeResult;

/* 控制帧编码字段；payload 仅在编码调用期间有效，不拥有所指内存。 */
typedef struct {
    uint8_t src_id;
    uint8_t dst_id;
    uint8_t msg_type;
    uint8_t cmd;
    uint16_t flags;
    uint32_t seq;
    uint32_t session_id;
    uint32_t sensor_id;
    uint32_t safety_param_crc;
    const uint8_t *payload;
    uint16_t payload_len;
} SensorSafeControlFields;

/* 已解码控制帧视图；fields.payload 指向调用方原始接收缓冲区。 */
typedef struct {
    SensorSafeControlFields fields;
    uint16_t frame_len;
    uint32_t received_crc;
} SensorSafeControlFrame;

/* 已通过线格式校验的周期快报，定点单位由字段名后缀明确标识。 */
typedef struct {
    uint8_t src_id;
    uint8_t dst_id;
    uint8_t measure_mode;
    uint8_t comm_mode;
    uint32_t session_id;
    uint16_t stream_id;
    uint32_t stream_seq;
    uint32_t sample_counter;
    uint16_t data_age_ms;
    uint16_t status_flags16;
    uint16_t diag_flags16;
    int16_t temperature_c_x100;
    int32_t density_kg_m3_x100;
    uint32_t frequency_hz_x1000;
    uint8_t signal_quality;
    uint16_t config_epoch;
    uint32_t received_crc;
} SensorSafeFastReport;

/* 所有控制响应共有的 20 字节载荷前缀。 */
typedef struct {
    uint16_t result_code;
    uint8_t measure_mode;
    uint8_t comm_mode;
    uint32_t status_flags;
    uint32_t diag_flags;
    uint32_t sample_counter;
    uint16_t data_age_ms;
    uint8_t payload_version;
    uint8_t data_quality;
} SensorSafeCommonResponse;

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_SAFE_TYPES_H_ */
